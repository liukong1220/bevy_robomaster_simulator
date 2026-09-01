//! 发布端 seqlock 的真并发测试。
//!
//! C++ 侧 `tests/sim_shm_stress_test` 验证的是读端（SharedMemoryClient）；这里验证的
//! 是**写端**——`ShmPublisher::publish_ground_truth` 的两道 release 栅栏与"标记只做
//! 原子访问"。两边都要有，因为它们能挂的方式不同：读端漏掉重试会读到奇数标记，写端
//! 漏掉栅栏则会让 payload 的写先于奇数标记可见，此时读端**完全正确**也会看到
//! "偶数标记 + 撕裂 payload + 同一个偶数标记"，前后比较照样通过。
//!
//! 读端在这里是手写的（subscriber.rs 不读真值区，真值只给 C++ 评估器用），与
//! shared_memory_client.cpp::read_ground_truth 的算法逐句对应。
//!
//! 撕裂判据用"代数戳"：同一个 generation 写进 batch 每一个数值字段，任何字段与
//! frame_seq 不一致就说明这一批混了两次写入。只比较前后 frame_seq 是不够的——
//! 同一帧号内重发时它根本不变。
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::time::{Duration, Instant};

use talos_ipc::*;

/// float 能精确表示的整数上限是 2^24，超过之后相邻代数会舍入到同一个 f32，
/// 撕裂就检测不出来了。
const GEN_LIMIT: u32 = 1 << 24;

fn stamp(generation: u32) -> GroundTruthBatch {
    let f = generation as f32;
    let u8v = (generation & 0xff) as u8;
    let mut b = GroundTruthBatch::default();
    b.frame_seq = generation as u64;
    b.timestamp_ns = generation as u64;
    // 计数字段固定打满，读端才会扫描全部槽位；跟着代数变的话后半段撕裂会被漏掉。
    b.target_count = GROUND_TRUTH_MAX_TARGETS as u32;
    b.rune_count = GROUND_TRUTH_MAX_RUNES as u32;
    for t in b.targets.iter_mut() {
        t.frame_seq = generation as u64;
        t.timestamp_ns = generation as u64;
        t.team = u8v;
        t.armor_label = u8v;
        t.is_outpost = u8v;
        t.position = [f; 3];
        t.vyaw = f;
        t.yaw = f;
        t.armor_position = [f; 3];
        t.armor_position_valid = u8v;
    }
    for r in b.runes.iter_mut() {
        r.frame_seq = generation as u64;
        r.timestamp_ns = generation as u64;
        r.team = u8v;
        r.rune_mode = u8v;
        r.mechanism_state = u8v;
        r.r_center_odom = [f; 3];
        r.radius = f;
        r.current_angle = f;
        r.v_roll = f;
        r.direction = generation as i32;
        r.sin_amplitude = f;
        r.sin_omega = f;
        r.sin_phase = f;
        r.sin_offset = f;
        r.relative_time = f;
        r.blade_id = generation as i32;
        r.target_activations = [u8v; 5];
    }
    b
}

/// 以 `frame_seq` 为基准代数，返回第一个代数不符的字段名。整批自洽时返回 None。
fn find_mixed_field(b: &GroundTruthBatch) -> Option<String> {
    let generation = b.frame_seq as u32;
    let f = generation as f32;
    let u8v = (generation & 0xff) as u8;

    if b.timestamp_ns != generation as u64 {
        return Some("batch.timestamp_ns".into());
    }
    if b.target_count != GROUND_TRUTH_MAX_TARGETS as u32 {
        return Some("batch.target_count".into());
    }
    if b.rune_count != GROUND_TRUTH_MAX_RUNES as u32 {
        return Some("batch.rune_count".into());
    }
    for (i, t) in b.targets.iter().enumerate() {
        let bad = if t.frame_seq != generation as u64 {
            Some("frame_seq")
        } else if t.timestamp_ns != generation as u64 {
            Some("timestamp_ns")
        } else if t.team != u8v {
            Some("team")
        } else if t.armor_label != u8v {
            Some("armor_label")
        } else if t.is_outpost != u8v {
            Some("is_outpost")
        } else if t.position != [f; 3] {
            Some("position")
        } else if t.vyaw != f {
            Some("vyaw")
        } else if t.yaw != f {
            Some("yaw")
        } else if t.armor_position != [f; 3] {
            Some("armor_position")
        } else if t.armor_position_valid != u8v {
            Some("armor_position_valid")
        } else {
            None
        };
        if let Some(name) = bad {
            return Some(format!("targets[{i}].{name}"));
        }
    }
    for (i, r) in b.runes.iter().enumerate() {
        let bad = if r.frame_seq != generation as u64 {
            Some("frame_seq")
        } else if r.timestamp_ns != generation as u64 {
            Some("timestamp_ns")
        } else if r.team != u8v {
            Some("team")
        } else if r.rune_mode != u8v {
            Some("rune_mode")
        } else if r.mechanism_state != u8v {
            Some("mechanism_state")
        } else if r.r_center_odom != [f; 3] {
            Some("r_center_odom")
        } else if r.radius != f {
            Some("radius")
        } else if r.current_angle != f {
            Some("current_angle")
        } else if r.v_roll != f {
            Some("v_roll")
        } else if r.direction != generation as i32 {
            Some("direction")
        } else if r.sin_amplitude != f {
            Some("sin_amplitude")
        } else if r.sin_omega != f {
            Some("sin_omega")
        } else if r.sin_phase != f {
            Some("sin_phase")
        } else if r.sin_offset != f {
            Some("sin_offset")
        } else if r.relative_time != f {
            Some("relative_time")
        } else if r.blade_id != generation as i32 {
            Some("blade_id")
        } else if r.target_activations != [u8v; 5] {
            Some("target_activations")
        } else {
            None
        };
        if let Some(name) = bad {
            return Some(format!("runes[{i}].{name}"));
        }
    }
    None
}

/// 与 shared_memory_client.cpp::read_ground_truth 同构的读端。
///
/// 关键的三点在这里必须一模一样，否则测的就不是同一个协议：
///   1. 标记只用 `AtomicU32` 读，不随 payload 一起整块拷；
///   2. 只拷 `GROUND_TRUTH_PAYLOAD_BYTES` 字节的前缀（标记之后只有 pad_）；
///   3. 拷完补一道 acquire 栅栏，再读第二次标记。
///
/// # Safety
/// `meta` 必须指向一个活着的、已按协议初始化的 `ShmMetaRegion` 映射。
unsafe fn read_ground_truth(meta: *const ShmMetaRegion) -> Option<(GroundTruthBatch, u32)> {
    let slot = unsafe { core::ptr::addr_of!((*meta).ground_truth) };
    let seq = unsafe { &*(core::ptr::addr_of!((*meta).ground_truth.seqlock) as *const AtomicU32) };

    for _ in 0..8 {
        let before = seq.load(Ordering::Acquire);
        if before & 1 != 0 {
            continue; // 写入进行中
        }
        let mut out = GroundTruthBatch::default();
        unsafe {
            core::ptr::copy_nonoverlapping(
                slot.cast::<u8>(),
                (&mut out as *mut GroundTruthBatch).cast::<u8>(),
                GROUND_TRUTH_PAYLOAD_BYTES,
            );
        }
        core::sync::atomic::fence(Ordering::Acquire);
        let after = seq.load(Ordering::Acquire);
        if before != after {
            continue;
        }
        if before == 0 {
            return None; // 发布端一次都没提交过
        }
        return Some((out, before));
    }
    None
}

/// 只是为了能把裸映射指针搬进读线程。安全性由 `main` 侧的生命周期保证：
/// 映射在 join 之后才被释放。
struct MetaPtr(*const ShmMetaRegion);
unsafe impl Send for MetaPtr {}

#[test]
fn detector_flags_mixed_generations() {
    // 先自检探测器。若 find_mixed_field 恒为 None，下面的并发测试会 100% 通过而
    // 什么都没验证——一个永真的判据比没有判据更糟。
    let a = stamp(1000);
    assert_eq!(find_mixed_field(&a), None, "单代数据应判为自洽");

    // 构造"前半段是新数据、后半段还是旧数据"的撕裂：只覆盖头部与前 3 个目标。
    let mut mixed = stamp(1000);
    let newer = stamp(2000);
    let head = core::mem::offset_of!(GroundTruthBatch, targets)
        + 3 * core::mem::size_of::<GroundTruthTarget>();
    unsafe {
        core::ptr::copy_nonoverlapping(
            (&newer as *const GroundTruthBatch).cast::<u8>(),
            (&mut mixed as *mut GroundTruthBatch).cast::<u8>(),
            head,
        );
    }
    assert!(find_mixed_field(&mixed).is_some(), "半新半旧数据应判为撕裂");
}

#[test]
fn concurrent_writer_never_exposes_torn_or_odd_batch() {
    // 固定区域名会与本机正在跑的仿真器互相覆盖，所以带上 pid。
    let pid = std::process::id();
    let meta_name = format!("talos_gt_seqlock_meta_{pid}");
    let pool_name = format!("talos_gt_seqlock_pool_{pid}");

    let mut publisher = ShmPublisher::create_named(&meta_name, &pool_name).expect("创建发布端失败");
    // 先提交一代，读端一开始就有可读数据（seqlock != 0）。
    publisher.publish_ground_truth(&stamp(1));

    let reader_region =
        ShmRegion::open(&meta_name, core::mem::size_of::<ShmMetaRegion>()).expect("打开映射失败");
    let meta = MetaPtr(reader_region.as_ptr().cast::<ShmMetaRegion>());

    let stop = Arc::new(AtomicBool::new(false));
    let writes = Arc::new(AtomicU64::new(0));

    let writer = {
        let stop = Arc::clone(&stop);
        let writes = Arc::clone(&writes);
        std::thread::spawn(move || {
            let mut generation: u32 = 2;
            while !stop.load(Ordering::Relaxed) && generation < GEN_LIMIT {
                publisher.publish_ground_truth(&stamp(generation));
                writes.store(generation as u64, Ordering::Relaxed);
                generation += 1;
            }
            // publisher 在这里被 drop：区域随之 unlink。必须在读线程之后结束，
            // 所以下面先 join 读线程再 join 写线程。
        })
    };

    let reader = {
        let stop = Arc::clone(&stop);
        std::thread::spawn(move || {
            let meta = meta;
            let mut reads = 0u64;
            let mut read_failures = 0u64;
            let mut odd_markers = 0u64;
            let mut first_mixed: Option<String> = None;
            let mut mixed_count = 0u64;
            let mut distinct = 0u64;
            let mut last_gen = u64::MAX;
            let mut min_gen = u32::MAX;
            let mut max_gen = 0u32;

            let deadline = Instant::now() + Duration::from_millis(1500);
            while Instant::now() < deadline {
                match unsafe { read_ground_truth(meta.0) } {
                    None => read_failures += 1,
                    Some((batch, marker)) => {
                        reads += 1;
                        if marker & 1 != 0 {
                            odd_markers += 1;
                        }
                        if let Some(field) = find_mixed_field(&batch) {
                            mixed_count += 1;
                            if first_mixed.is_none() {
                                first_mixed = Some(format!(
                                    "generation={} marker={marker} 字段={field}",
                                    batch.frame_seq
                                ));
                            }
                        }
                        let generation = batch.frame_seq as u32;
                        min_gen = min_gen.min(generation);
                        max_gen = max_gen.max(generation);
                        if batch.frame_seq != last_gen {
                            distinct += 1;
                            last_gen = batch.frame_seq;
                        }
                    }
                }
            }
            stop.store(true, Ordering::Relaxed);
            (
                reads,
                read_failures,
                odd_markers,
                mixed_count,
                first_mixed,
                distinct,
                min_gen,
                max_gen,
            )
        })
    };

    let (reads, read_failures, odd_markers, mixed_count, first_mixed, distinct, min_gen, max_gen) =
        reader.join().expect("读线程 panic");
    stop.store(true, Ordering::Relaxed);
    writer.join().expect("写线程 panic");

    println!(
        "并发统计: 成功读 {reads} 次，读重试失败 {read_failures} 次，写 {} 代，\
         代数区间 [{min_gen}, {max_gen}]，不同代数 {distinct} 个",
        writes.load(Ordering::Relaxed)
    );

    // 这三条是"测试确实跑起来了"的前提。少了它们，一个读不到数据的空转循环也会报绿。
    assert!(reads > 0, "读端一次都没读成功");
    assert!(
        writes.load(Ordering::Relaxed) >= 100,
        "写端只提交了 {} 代，太少，说明没真正并发跑起来",
        writes.load(Ordering::Relaxed)
    );
    assert!(
        distinct >= 10,
        "读端只看到 {distinct} 个不同代数，读写没有交叠"
    );
    assert!(max_gen > min_gen, "读端看到的代数没有推进");

    assert_eq!(odd_markers, 0, "有成功读取带奇数 marker");
    assert_eq!(mixed_count, 0, "有半新半旧的批次: {first_mixed:?}");
}
