use std::sync::atomic::AtomicU8;

pub const IMAGE_WIDTH: u32 = 1440;
pub const IMAGE_HEIGHT: u32 = 1080;

pub const CACHE_LINE_SIZE: usize = 64;
pub const SHM_MAGIC: u32 = 0x54414C05;
/// 协议版本。
///
/// v2 -> v3：`poses[Muzzle]` 由"相对云台的局部平移 + 单位四元数"改为**完整世界
/// 位姿**（见 [`PoseIndex::Muzzle`]），并在 [`ShmHeader::capabilities`] 里显式声明
/// 各可选区域是否有效。
///
/// 语义变更必须升版本号：字节布局没变，旧消费端照旧能 mmap、能读出数字，只是把
/// 世界坐标当成局部偏移继续算——那是静默的错误答案，比连不上更难发现。
pub const SHM_VERSION: u32 = 3;

/// `ShmHeader::capabilities` 的位定义。
///
/// 版本号只能表达"协议第几代"，表达不了"这一代的发布端实际填了哪些可选区域"。
/// 真值区就是典型：布局里一直有 `ground_truth`，但只有仿真器本体会填，测试用的
/// 精简发布端不会。消费端不看能力位就只能靠"读出来是不是全零"猜，而全零同样是
/// 合法数据，于是**新消费端对着不发真值的发布端会静默失去真值**。
pub const CAP_GROUND_TRUTH: u32 = 1 << 0;
/// `poses[Muzzle]` 是世界位姿（v3 语义）。未置位表示 v2 的局部平移语义。
pub const CAP_MUZZLE_WORLD_POSE: u32 = 1 << 1;
/// `chassis_observation` 区有效。
pub const CAP_CHASSIS_OBSERVATION: u32 = 1 << 2;
/// `runtime_state` 区有效。
pub const CAP_RUNTIME_STATE: u32 = 1 << 3;

/// 仿真器本体发布的能力集合。
pub const SIMULATOR_CAPABILITIES: u32 =
    CAP_GROUND_TRUTH | CAP_MUZZLE_WORLD_POSE | CAP_CHASSIS_OBSERVATION | CAP_RUNTIME_STATE;

pub const IMAGE_CHANNELS: u32 = 3;
pub const IMAGE_SIZE: usize = (IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS) as usize;
pub const IMAGE_POOL_SIZE: usize = IMAGE_SIZE * 3;
pub const SHM_NAME_META: &str = "talos_ipc_meta";
pub const SHM_NAME_IMAGE_POOL: &str = "talos_ipc_image_pool";

pub const FLAG_NEW: u8 = 0x80;
pub const INDEX_MASK: u8 = 0x03;

#[repr(C, align(32))]
#[derive(Debug, Clone, Copy, Default)]
pub struct ImageMeta {
    pub seq: u64,
    pub timestamp_ns: u64,
    pub width: u32,
    pub height: u32,
    pub buffer_id: u8,
    pub format: u8,
    pub _pad: [u8; 6],
}
const _: () = assert!(size_of::<ImageMeta>() == 32);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct PoseMeta {
    pub frame_seq: u64,
    pub position: [f32; 3],
    pub quaternion: [f32; 4],
    pub timestamp_ns: u64,
    pub _pad: [u8; 16],
}
const _: () = assert!(size_of::<PoseMeta>() == 64);

impl Default for PoseMeta {
    fn default() -> Self {
        Self {
            frame_seq: 0,
            position: [0.0; 3],
            quaternion: [0.0; 4],
            timestamp_ns: 0,
            _pad: [0; 16],
        }
    }
}

#[repr(C, align(32))]
#[derive(Debug, Clone, Copy, Default)]
pub struct GimbalCmd {
    pub timestamp_ns: u64,
    pub yaw_deg: f32,
    pub pitch_deg: f32,
    pub distance_m: f32,
    pub fire_advice: u8,
    pub _pad: [u8; 11],
}
const _: () = assert!(size_of::<GimbalCmd>() == 32);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy, Default)]
pub struct CameraInfo {
    pub timestamp_ns: u64,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub distortion: [f64; 5],
    pub width: u32,
    pub height: u32,
    pub _pad: [u8; 24],
}
const _: () = assert!(size_of::<CameraInfo>() == 128);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct ChassisObservation {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub dt_s: f32,
    pub v_body: [f32; 2],
    pub wz_radps: f32,
    pub wheel_linear_mps: [f32; 4],
    pub wheel_angular_radps: [f32; 4],
    pub a_body: [f32; 2],
    pub alpha_z_radps2: f32,
    pub rpy_rad: [f32; 3],
    pub gyro_xyz_radps: [f32; 3],
    pub accel_xyz_mps2: [f32; 3],
    pub _pad: [u8; 16],
}
const _: () = assert!(size_of::<ChassisObservation>() == 128);

impl Default for ChassisObservation {
    fn default() -> Self {
        Self {
            frame_seq: 0,
            timestamp_ns: 0,
            dt_s: 0.0,
            v_body: [0.0; 2],
            wz_radps: 0.0,
            wheel_linear_mps: [0.0; 4],
            wheel_angular_radps: [0.0; 4],
            a_body: [0.0; 2],
            alpha_z_radps2: 0.0,
            rpy_rad: [0.0; 3],
            gyro_xyz_radps: [0.0; 3],
            accel_xyz_mps2: [0.0; 3],
            _pad: [0; 16],
        }
    }
}

#[repr(C, align(64))]
pub struct ImageTripleBuffer {
    pub state: AtomicU8,
    pub write_idx: u8,
    pub read_idx: u8,
    pub _pad1: [u8; 61],
    pub slots: [ImageMeta; 3],
}
const _: () = assert!(size_of::<ImageTripleBuffer>() == 192);

#[repr(C, align(64))]
pub struct PoseTripleBuffer {
    pub state: AtomicU8,
    pub write_idx: u8,
    pub read_idx: u8,
    pub _pad1: [u8; 61],
    pub slots: [PoseMeta; 3],
}
const _: () = assert!(size_of::<PoseTripleBuffer>() == 256);

#[repr(C, align(64))]
pub struct GimbalTripleBuffer {
    pub state: AtomicU8,
    pub write_idx: u8,
    pub read_idx: u8,
    pub _pad1: [u8; 61],
    pub slots: [GimbalCmd; 3],
}
const _: () = assert!(size_of::<GimbalTripleBuffer>() == 192);

#[repr(C, align(64))]
pub struct ShmHeader {
    pub magic: u32,
    pub version: u32,
    pub created_ns: u64,
    pub heartbeat_ns: u64,
    pub image_width: u32,
    pub image_height: u32,
    /// 可选区域能力位，见 [`CAP_GROUND_TRUTH`] 等。占用原 `_pad` 的前 4 字节，
    /// 结构体大小与其余偏移不变。
    pub capabilities: u32,
    pub _pad: [u8; 28],
}
const _: () = assert!(size_of::<ShmHeader>() == 64);
const _: () = assert!(core::mem::offset_of!(ShmHeader, capabilities) == 32);

pub const GROUND_TRUTH_MAX_TARGETS: usize = 16;
pub const GROUND_TRUTH_MAX_RUNES: usize = 4;

#[repr(C, align(32))]
#[derive(Debug, Clone, Copy, Default)]
pub struct GroundTruthTarget {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub team: u8,
    pub armor_label: u8,
    pub is_outpost: u8,
    pub _pad1: u8,
    pub position: [f32; 3],
    pub vyaw: f32,
    pub yaw: f32,
    /// 被选中装甲板板心在 odom 系下的位置。整车中心（`position`）不是自瞄真正
    /// 瞄的点：板心偏心半径约 0.2m、比车心高约 0.06m，1.5m 距离上折算 2 度量级。
    /// 消费端拿整车中心算瞄准误差会把这段固定几何差当成闭环残差。
    /// `armor_position_valid == 0` 时该字段无意义。占用原 `_pad` 的前 16 字节。
    pub armor_position: [f32; 3],
    pub armor_position_valid: u8,
    pub _pad: [u8; 11],
}
const _: () = assert!(size_of::<GroundTruthTarget>() == 64);
const _: () = assert!(core::mem::offset_of!(GroundTruthTarget, armor_position) == 40);
const _: () = assert!(core::mem::offset_of!(GroundTruthTarget, armor_position_valid) == 52);

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct GroundTruthRune {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub team: u8,
    pub rune_mode: u8,
    pub mechanism_state: u8,
    pub _pad1: u8,
    pub r_center_odom: [f32; 3],
    pub radius: f32,
    pub current_angle: f32,
    pub v_roll: f32,
    pub direction: i32,
    pub sin_amplitude: f32,
    pub sin_omega: f32,
    pub sin_phase: f32,
    pub sin_offset: f32,
    pub relative_time: f32,
    pub blade_id: i32,
    pub target_activations: [u8; 5],
    pub _pad: [u8; 20],
}
const _: () = assert!(size_of::<GroundTruthRune>() == 128);

impl Default for GroundTruthRune {
    fn default() -> Self {
        Self {
            frame_seq: 0,
            timestamp_ns: 0,
            team: 0,
            rune_mode: 0,
            mechanism_state: 0,
            _pad1: 0,
            r_center_odom: [0.0; 3],
            radius: 0.0,
            current_angle: 0.0,
            v_roll: 0.0,
            direction: 0,
            sin_amplitude: 0.0,
            sin_omega: 0.0,
            sin_phase: 0.0,
            sin_offset: 0.0,
            relative_time: 0.0,
            blade_id: -1,
            target_activations: [0; 5],
            _pad: [0; 20],
        }
    }
}

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct GroundTruthBatch {
    pub frame_seq: u64,
    pub timestamp_ns: u64,
    pub target_count: u32,
    pub rune_count: u32,
    pub targets: [GroundTruthTarget; GROUND_TRUTH_MAX_TARGETS],
    pub runes: [GroundTruthRune; GROUND_TRUTH_MAX_RUNES],
    /// seqlock 序号。发布端写 body 之前置奇、写完置偶；消费端读到奇数或前后不等
    /// 就重试。原来消费端靠"memcpy 前后 frame_seq 相等"近似判断整块稳定，那不是
    /// 同步保证：同一帧号内重发时 frame_seq 不变，body 却在被改写。
    /// 占用原 `_pad` 的前 4 字节。
    pub seqlock: u32,
    pub _pad: [u8; 60],
}
const _: () = assert!(size_of::<GroundTruthBatch>() == 1664);
const _: () = assert!(core::mem::offset_of!(GroundTruthBatch, seqlock) == 1600);

/// seqlock 标记之前的 payload 字节数，即整块里"真正的数据"。
///
/// 两端拷贝 payload 时都只拷这段前缀，标记本身只用原子读写访问。整块 memcpy
/// 会顺带覆盖/读取 4 字节的标记：写端等于用非原子写踩自己的同步变量，读端等于
/// 用非原子读取一个正在被并发修改的原子变量——两者都是数据竞争（UB），而且写端
/// 那一路还会让"标记先置奇再被 body 覆盖成别的值"，读端的前后比较可能意外通过。
///
/// 标记之后只有 `_pad`，所以这段前缀覆盖了全部有效字段。
pub const GROUND_TRUTH_PAYLOAD_BYTES: usize = 1600;
const _: () =
    assert!(GROUND_TRUTH_PAYLOAD_BYTES == core::mem::offset_of!(GroundTruthBatch, seqlock));

impl Default for GroundTruthBatch {
    fn default() -> Self {
        Self {
            frame_seq: 0,
            timestamp_ns: 0,
            target_count: 0,
            rune_count: 0,
            targets: [GroundTruthTarget::default(); GROUND_TRUTH_MAX_TARGETS],
            runes: [GroundTruthRune::default(); GROUND_TRUTH_MAX_RUNES],
            seqlock: 0,
            _pad: [0; 60],
        }
    }
}

#[repr(C, align(64))]
#[derive(Debug, Clone, Copy)]
pub struct RuntimeState {
    pub timestamp_ns: u64,
    pub following: u8,
    pub _pad: [u8; 55],
}
const _: () = assert!(size_of::<RuntimeState>() == 64);

impl Default for RuntimeState {
    fn default() -> Self {
        Self {
            timestamp_ns: 0,
            following: 0,
            _pad: [0; 55],
        }
    }
}

#[repr(C)]
pub struct ShmMetaRegion {
    pub header: ShmHeader,
    pub image: ImageTripleBuffer,
    pub poses: [PoseTripleBuffer; 5],
    pub gimbal_cmd: GimbalTripleBuffer,
    pub camera_info: CameraInfo,
    pub chassis_observation: ChassisObservation,
    pub ground_truth: GroundTruthBatch,
    pub runtime_state: RuntimeState,
}
const _: () = assert!(size_of::<ShmMetaRegion>() == 3712);
const _: () = assert!(std::mem::offset_of!(ShmMetaRegion, camera_info) == 1728);
const _: () = assert!(std::mem::offset_of!(ShmMetaRegion, chassis_observation) == 1856);
const _: () = assert!(std::mem::offset_of!(ShmMetaRegion, ground_truth) == 1984);
const _: () = assert!(std::mem::offset_of!(ShmMetaRegion, runtime_state) == 3648);

/// 位姿通道。所有平移与旋转都已经转到 ROS 约定（x 前、y 左、z 上），
/// 四元数按 `[w, x, y, z]` 发布。
///
/// **各通道的参考系不同，混用会静默出错**，逐条写明：
///
/// | 通道 | position | quaternion |
/// |------|----------|------------|
/// | `Gimbal` | 恒为 `[0,0,0]`（占位，不是坐标） | 云台/枪管的**世界**姿态 `world <- gimbal` |
/// | `Odom` | 云台回转中心的**世界**位置 | 单位四元数（占位） |
/// | `Muzzle` | 枪口的**世界**位置 | 枪口的**世界**姿态（同 `Gimbal`） |
/// | `Camera` | 相机相对云台的**局部**平移 | 单位四元数（占位） |
///
/// `Camera` 刻意保持局部：消费端拿它与自己配置里的 `t_camera2gimbal` 外参做
/// 自检，那是一个局部量。
///
/// `Muzzle` 在 v2 里也是局部平移，v3 改为世界位姿（能力位
/// [`CAP_MUZZLE_WORLD_POSE`]）。原因是局部量太容易被误用：消费端见到 `Odom` 是
/// 世界位置、`Muzzle` 是"偏移"，最自然的写法就是 `odom + muzzle`，而那是把一个
/// **未经云台旋转**的局部平移直接加到世界坐标上。yaw=90° 时 0.11 m 的局部 +X
/// 实际指向世界 +Y，误差达到偏移量的全长，且随姿态变化，看起来像闭环残差。
/// 直接发布世界量让消费端无从误用。
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseIndex {
    Gimbal = 0,
    Odom = 1,
    Muzzle = 2,
    Camera = 3,
    // Legacy compatibility channel.
    // New integrations should consume `ShmMetaRegion::chassis_observation` instead.
    ChassisObservation = 4,
}

impl Default for ImageTripleBuffer {
    fn default() -> Self {
        Self {
            state: AtomicU8::new(1),
            write_idx: 0,
            read_idx: 2,
            _pad1: [0; 61],
            slots: [ImageMeta::default(); 3],
        }
    }
}

impl Default for PoseTripleBuffer {
    fn default() -> Self {
        Self {
            state: AtomicU8::new(1),
            write_idx: 0,
            read_idx: 2,
            _pad1: [0; 61],
            slots: [PoseMeta::default(); 3],
        }
    }
}

impl Default for GimbalTripleBuffer {
    fn default() -> Self {
        Self {
            state: AtomicU8::new(1),
            write_idx: 0,
            read_idx: 2,
            _pad1: [0; 61],
            slots: [GimbalCmd::default(); 3],
        }
    }
}

impl Default for ShmHeader {
    fn default() -> Self {
        Self {
            magic: SHM_MAGIC,
            version: SHM_VERSION,
            created_ns: 0,
            heartbeat_ns: 0,
            image_width: IMAGE_WIDTH,
            image_height: IMAGE_HEIGHT,
            // Default 不声明任何能力：谁真的填了对应区域，谁就自己置位。
            // 让 Default 直接宣称 SIMULATOR_CAPABILITIES 会把"布局里有这个字段"
            // 冒充成"这一路发布端真的在写这个字段"，正是能力位要防的那件事。
            capabilities: 0,
            _pad: [0; 28],
        }
    }
}

impl Default for ShmMetaRegion {
    fn default() -> Self {
        Self {
            header: ShmHeader::default(),
            image: ImageTripleBuffer::default(),
            poses: [
                PoseTripleBuffer::default(),
                PoseTripleBuffer::default(),
                PoseTripleBuffer::default(),
                PoseTripleBuffer::default(),
                PoseTripleBuffer::default(),
            ],
            gimbal_cmd: GimbalTripleBuffer::default(),
            camera_info: CameraInfo::default(),
            chassis_observation: ChassisObservation::default(),
            ground_truth: GroundTruthBatch::default(),
            runtime_state: RuntimeState::default(),
        }
    }
}
