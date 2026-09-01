use crate::layout::*;
use crate::shm::{ShmError, ShmRegion};
use crate::triple_buffer::TripleBufferProducer;
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

pub struct ShmPublisher {
    meta_region: ShmRegion,
    image_pool: ShmRegion,
    current_buffer_id: u8,
    last_synchronized_frame_seq: Option<u64>,
}

impl ShmPublisher {
    pub fn create() -> Result<Self, ShmError> {
        Self::create_named(SHM_NAME_META, SHM_NAME_IMAGE_POOL)
    }

    /// 用显式的区域名创建。仿真器本体永远走 [`Self::create`]（固定名字才能被
    /// 消费端找到）；测试必须用这个：固定名字会与本机正在跑的仿真器互相覆盖，
    /// 并行跑的两个测试之间也会互相踩。
    pub fn create_named(meta_name: &str, image_pool_name: &str) -> Result<Self, ShmError> {
        let mut meta_region = ShmRegion::create(meta_name, size_of::<ShmMetaRegion>())?;
        let image_pool = ShmRegion::create(image_pool_name, IMAGE_POOL_SIZE)?;

        unsafe {
            let meta = meta_region.as_mut::<ShmMetaRegion>();

            // 初始化 header
            meta.header = ShmHeader {
                magic: SHM_MAGIC,
                version: SHM_VERSION,
                created_ns: Self::now_ns(),
                heartbeat_ns: Self::now_ns(),
                image_width: IMAGE_WIDTH,
                image_height: IMAGE_HEIGHT,
                // 仿真器本体一定会发布真值/世界枪口位姿/底盘观测/运行状态：
                // `TalosPlugin::build` 无条件注册了这几个 system。
                capabilities: SIMULATOR_CAPABILITIES,
                _pad: [0; 28],
            };

            // 初始化所有 TripleBuffer (CRITICAL: 零填充破坏了正确的初始状态)
            // 正确初始状态: state=1 (ready slot), write_idx=0, read_idx=2
            Self::init_triple_buffer(&mut meta.image);
            for pose in &mut meta.poses {
                Self::init_triple_buffer(pose);
            }
            Self::init_triple_buffer(&mut meta.gimbal_cmd);
        }

        Ok(Self {
            meta_region,
            image_pool,
            current_buffer_id: 0,
            last_synchronized_frame_seq: None,
        })
    }

    pub fn publish_image(&mut self, data: &[u8], seq: u64, timestamp_ns: u64) {
        self.publish_image_with(data, seq, timestamp_ns, |_| {});
    }

    pub fn publish_image_with<F>(
        &mut self,
        data: &[u8],
        seq: u64,
        timestamp_ns: u64,
        before_commit: F,
    ) where
        F: FnOnce(&mut Self),
    {
        assert_eq!(data.len(), IMAGE_SIZE, "Image size mismatch");

        let buffer_id = self.current_buffer_id;
        self.current_buffer_id = (self.current_buffer_id + 1) % 3;

        unsafe {
            let pool_ptr = self.image_pool.as_ptr();
            let dst = pool_ptr.add(buffer_id as usize * IMAGE_SIZE);
            std::ptr::copy_nonoverlapping(data.as_ptr(), dst, IMAGE_SIZE);
        }

        // Publish data associated with this image only after the expensive pixel copy. The image
        // metadata below is the commit marker observed by consumers.
        before_commit(self);

        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let mut producer = TripleBufferProducer::new(
                &meta.image.state,
                &mut meta.image.write_idx,
                &mut meta.image.slots,
            );

            let slot = producer.borrow_mut();
            slot.seq = seq;
            slot.timestamp_ns = timestamp_ns;
            slot.width = IMAGE_WIDTH;
            slot.height = IMAGE_HEIGHT;
            slot.buffer_id = buffer_id;
            slot.format = 0;
            producer.publish();
        }
    }

    #[must_use]
    pub fn try_publish_synchronized_image<F>(
        &mut self,
        data: &[u8],
        seq: u64,
        timestamp_ns: u64,
        before_commit: F,
    ) -> bool
    where
        F: FnOnce(&mut Self),
    {
        // Never publish an older async readback, and never overwrite one half of an image/pose
        // bundle while the consumer is between the two triple buffers.
        if self
            .last_synchronized_frame_seq
            .is_some_and(|last_seq| seq <= last_seq)
            || !self.synchronized_frame_consumed()
        {
            return false;
        }

        self.publish_image_with(data, seq, timestamp_ns, before_commit);
        self.last_synchronized_frame_seq = Some(seq);
        true
    }

    fn synchronized_frame_consumed(&self) -> bool {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            let image_consumed = meta.image.state.load(Ordering::Acquire) & FLAG_NEW == 0;
            // Gimbal, odom, muzzle and camera are consumed with each image. Slot 4 is the legacy
            // chassis-observation channel and is intentionally not part of this handshake.
            let poses_consumed = meta.poses[..=PoseIndex::Camera as usize]
                .iter()
                .all(|pose| pose.state.load(Ordering::Acquire) & FLAG_NEW == 0);
            image_consumed && poses_consumed
        }
    }

    pub fn publish_pose(
        &mut self,
        index: PoseIndex,
        position: [f32; 3],
        quaternion: [f32; 4],
        frame_seq: u64,
        timestamp_ns: u64,
    ) {
        self.publish_pose_with_aux(
            index,
            position,
            quaternion,
            [0.0; 4],
            frame_seq,
            timestamp_ns,
        );
    }

    pub fn publish_pose_with_aux(
        &mut self,
        index: PoseIndex,
        position: [f32; 3],
        quaternion: [f32; 4],
        aux_f32: [f32; 4],
        frame_seq: u64,
        timestamp_ns: u64,
    ) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let pose_buf = &mut meta.poses[index as usize];
            let mut producer = TripleBufferProducer::new(
                &pose_buf.state,
                &mut pose_buf.write_idx,
                &mut pose_buf.slots,
            );

            let slot = producer.borrow_mut();
            slot.frame_seq = frame_seq;
            slot.position = position;
            slot.quaternion = quaternion;
            slot.timestamp_ns = timestamp_ns;
            slot._pad = aux_f32_to_bytes(aux_f32);

            producer.publish();
        }
    }

    pub fn set_camera_info(&mut self, info: CameraInfo) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.camera_info = info;
        }
    }

    pub fn publish_chassis_observation(&mut self, observation: ChassisObservation) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.chassis_observation = observation;
        }
    }

    /// 发布真值（seqlock 写端）：奇数 = 正在写，偶数 = 稳定。
    ///
    /// 消费端原来只能靠"memcpy 前后读到同一个 frame_seq"来猜这份拷贝是否完整，
    /// 那不是同步保证：同一帧号内重发时 frame_seq 根本不变，targets[] 却在被改写，
    /// 消费端会拿到半新半旧的一批目标；反过来编译器/CPU 也可以先写 frame_seq
    /// 再写 body。
    ///
    /// 标记只用原子读写访问，payload 只拷 [`GROUND_TRUTH_PAYLOAD_BYTES`] 字节的
    /// 前缀，两者互不重叠。原来是 `meta.ground_truth = staged;` 整块赋值，那一次
    /// 非原子写会跨过标记：既与读端对同一个原子变量的并发访问构成数据竞争，又把
    /// 刚置好的奇数标记覆盖成 `staged.seqlock`。后者在这里恰好等于 `begin`，所以
    /// 看起来"能用"——但它靠的是写端手工维持两个副本一致，而不是 seqlock 协议本身，
    /// 编译器也完全可以把这次结构体赋值拆分或重排。
    pub fn publish_ground_truth(&mut self, batch: &GroundTruthBatch) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let slot = core::ptr::addr_of_mut!(meta.ground_truth);
            let seq = &*(core::ptr::addr_of!(meta.ground_truth.seqlock) as *const AtomicU32);

            // 奇数 = 正在写。上一次提交后一定是偶数（初始 0 也是偶数）。
            let begin = seq.load(Ordering::Relaxed).wrapping_add(1) | 1;
            seq.store(begin, Ordering::Release);

            // 这道 fence 不能省：Release **store** 只阻止之前的写往后跑，不阻止
            // 之后的写往前跑。少了它，payload 的写可以先于奇数标记可见，读端就会
            // 看到偶数标记 + 撕裂 payload + 同一个偶数标记，前后比较照样通过。
            // 等价于内核 seqlock 写端 `sequence++; smp_wmb();` 里的那个屏障。
            core::sync::atomic::fence(Ordering::Release);

            core::ptr::copy_nonoverlapping(
                (batch as *const GroundTruthBatch).cast::<u8>(),
                slot.cast::<u8>(),
                GROUND_TRUTH_PAYLOAD_BYTES,
            );

            // 偶数 = 写完。payload 必须先于标记递增对读端可见。
            core::sync::atomic::fence(Ordering::Release);
            seq.store(begin.wrapping_add(1), Ordering::Release);
        }
    }

    pub fn publish_runtime_state(&mut self, state: RuntimeState) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.runtime_state = state;
        }
    }

    pub fn update_heartbeat(&mut self) {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            meta.header.heartbeat_ns = Self::now_ns();
        }
    }

    fn now_ns() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0)
    }

    /// 初始化 TripleBuffer 到正确的初始状态
    ///
    /// ShmRegion::create() 使用零填充，会破坏 TripleBuffer 的正确初始状态。
    /// 必须手动重新初始化。
    ///
    /// 正确初始状态:
    /// - state = 1 (ready slot 是 1, 无 FLAG_NEW)
    /// - write_idx = 0 (生产者写入 slot 0)
    /// - read_idx = 2 (消费者上次读取 slot 2)
    fn init_triple_buffer(buf: &mut impl TripleBufferInit) {
        buf.init_state();
    }
}

fn aux_f32_to_bytes(aux_f32: [f32; 4]) -> [u8; 16] {
    let mut bytes = [0u8; 16];
    for (i, value) in aux_f32.iter().enumerate() {
        bytes[i * 4..(i + 1) * 4].copy_from_slice(&value.to_le_bytes());
    }
    bytes
}

/// Trait for initializing triple buffer state
trait TripleBufferInit {
    fn init_state(&mut self);
}

impl TripleBufferInit for ImageTripleBuffer {
    fn init_state(&mut self) {
        self.state.store(1, Ordering::Relaxed);
        self.write_idx = 0;
        self.read_idx = 2;
    }
}

impl TripleBufferInit for PoseTripleBuffer {
    fn init_state(&mut self) {
        self.state.store(1, Ordering::Relaxed);
        self.write_idx = 0;
        self.read_idx = 2;
    }
}

impl TripleBufferInit for GimbalTripleBuffer {
    fn init_state(&mut self) {
        self.state.store(1, Ordering::Relaxed);
        self.write_idx = 0;
        self.read_idx = 2;
    }
}
