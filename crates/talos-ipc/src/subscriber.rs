use crate::layout::*;
use crate::shm::{ShmError, ShmRegion};
use crate::triple_buffer::TripleBufferConsumer;

pub struct ShmSubscriber {
    meta_region: ShmRegion,
}

impl ShmSubscriber {
    pub fn connect() -> Result<Self, ShmError> {
        Self::connect_named(SHM_NAME_META)
    }

    /// Connect to an explicit metadata region.
    ///
    /// Production code uses [`Self::connect`] and the fixed protocol name.  Isolated tests use
    /// this constructor so they never replace a running simulator's shared-memory region.
    pub fn connect_named(meta_name: &str) -> Result<Self, ShmError> {
        let meta_region = ShmRegion::open(meta_name, size_of::<ShmMetaRegion>())?;

        unsafe {
            let meta = meta_region.as_ref::<ShmMetaRegion>();
            if meta.header.magic != SHM_MAGIC {
                return Err(ShmError::InvalidSize);
            }
            if meta.header.version != SHM_VERSION {
                return Err(ShmError::InvalidSize);
            }
        }

        Ok(Self { meta_region })
    }

    pub fn recv_gimbal_cmd(&mut self) -> Option<GimbalCmd> {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let mut consumer = TripleBufferConsumer::new(
                &meta.gimbal_cmd.state,
                &mut meta.gimbal_cmd.read_idx,
                &meta.gimbal_cmd.slots,
            );

            consumer.borrow().copied()
        }
    }

    /// Consume the latest pose sample from a protocol pose channel.
    pub fn recv_pose(&mut self, index: PoseIndex) -> Option<PoseMeta> {
        unsafe {
            let meta = self.meta_region.as_mut::<ShmMetaRegion>();
            let pose = &mut meta.poses[index as usize];
            let mut consumer =
                TripleBufferConsumer::new(&pose.state, &mut pose.read_idx, &pose.slots);

            consumer.borrow().copied()
        }
    }

    pub fn has_gimbal_cmd(&self) -> bool {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            (meta
                .gimbal_cmd
                .state
                .load(std::sync::atomic::Ordering::Acquire)
                & FLAG_NEW)
                != 0
        }
    }

    pub fn chassis_observation(&self) -> Option<ChassisObservation> {
        unsafe {
            let meta = self.meta_region.as_ref::<ShmMetaRegion>();
            let observation = meta.chassis_observation;
            if observation.timestamp_ns == 0 {
                None
            } else {
                Some(observation)
            }
        }
    }
}
