//! 协议 ABI 的权威表。
//!
//! layout.rs 里已经有若干 `const _: () = assert!(...)`，但它们分散且不完整。
//! 这个测试把整张表集中打印出来，用来和 sp_vision25 的
//! `tests/sim_ipc_contract_test` 输出逐行对照：C++ 侧的镜像是手写的，
//! 只有两边都以同一张表为准，才能排除“各自自洽但互不一致”的情况。
//!
//! 跑 `cargo test -p talos-ipc --test layout_abi -- --nocapture` 可以看到表格。

use std::mem::{align_of, offset_of, size_of};
use talos_ipc::*;

fn row(name: &str, actual: usize, expected: usize) -> bool {
    let ok = actual == expected;
    println!(
        "{:<52} {:>10} {:>10}  {}",
        name,
        actual,
        expected,
        if ok { "ok" } else { "MISMATCH" }
    );
    ok
}

#[test]
fn layout_abi_table_is_canonical() {
    let mut failures = 0usize;
    let mut check = |name: &str, actual: usize, expected: usize| {
        if !row(name, actual, expected) {
            failures += 1;
        }
    };

    println!("{:<52} {:>10} {:>10}", "item", "actual", "expected");
    println!("--- 常量 ---------------------------------------------------------------");
    check("SHM_MAGIC", SHM_MAGIC as usize, 0x54414C05);
    check("SHM_VERSION", SHM_VERSION as usize, 2);
    check("IMAGE_WIDTH", IMAGE_WIDTH as usize, 1440);
    check("IMAGE_HEIGHT", IMAGE_HEIGHT as usize, 1080);
    check("IMAGE_CHANNELS", IMAGE_CHANNELS as usize, 3);
    check("IMAGE_SIZE", IMAGE_SIZE, 4_665_600);
    check("IMAGE_POOL_SIZE", IMAGE_POOL_SIZE, 13_996_800);
    check("FLAG_NEW", FLAG_NEW as usize, 0x80);
    check("INDEX_MASK", INDEX_MASK as usize, 0x03);
    check("GROUND_TRUTH_MAX_TARGETS", GROUND_TRUTH_MAX_TARGETS, 16);
    check("GROUND_TRUTH_MAX_RUNES", GROUND_TRUTH_MAX_RUNES, 4);
    check("PoseIndex::Gimbal", PoseIndex::Gimbal as usize, 0);
    check("PoseIndex::Odom", PoseIndex::Odom as usize, 1);
    check("PoseIndex::Muzzle", PoseIndex::Muzzle as usize, 2);
    check("PoseIndex::Camera", PoseIndex::Camera as usize, 3);
    check(
        "PoseIndex::ChassisObservation",
        PoseIndex::ChassisObservation as usize,
        4,
    );
    assert_eq!(SHM_NAME_META, "talos_ipc_meta");
    assert_eq!(SHM_NAME_IMAGE_POOL, "talos_ipc_image_pool");

    println!("--- 结构体大小与对齐 ---------------------------------------------------");
    check("ImageMeta sizeof", size_of::<ImageMeta>(), 32);
    check("ImageMeta alignof", align_of::<ImageMeta>(), 32);
    check("PoseMeta sizeof", size_of::<PoseMeta>(), 64);
    check("PoseMeta alignof", align_of::<PoseMeta>(), 64);
    check("GimbalCmd sizeof", size_of::<GimbalCmd>(), 32);
    check("GimbalCmd alignof", align_of::<GimbalCmd>(), 32);
    check("CameraInfo sizeof", size_of::<CameraInfo>(), 128);
    check("CameraInfo alignof", align_of::<CameraInfo>(), 64);
    check(
        "ChassisObservation sizeof",
        size_of::<ChassisObservation>(),
        128,
    );
    check(
        "ChassisObservation alignof",
        align_of::<ChassisObservation>(),
        64,
    );
    check("ShmHeader sizeof", size_of::<ShmHeader>(), 64);
    check("ShmHeader alignof", align_of::<ShmHeader>(), 64);
    check(
        "GroundTruthTarget sizeof",
        size_of::<GroundTruthTarget>(),
        64,
    );
    check("GroundTruthRune sizeof", size_of::<GroundTruthRune>(), 128);
    check(
        "GroundTruthBatch sizeof",
        size_of::<GroundTruthBatch>(),
        1664,
    );
    check("RuntimeState sizeof", size_of::<RuntimeState>(), 64);
    check(
        "ImageTripleBuffer sizeof",
        size_of::<ImageTripleBuffer>(),
        192,
    );
    check(
        "ImageTripleBuffer alignof",
        align_of::<ImageTripleBuffer>(),
        64,
    );
    check(
        "PoseTripleBuffer sizeof",
        size_of::<PoseTripleBuffer>(),
        256,
    );
    check(
        "PoseTripleBuffer alignof",
        align_of::<PoseTripleBuffer>(),
        64,
    );
    check(
        "GimbalTripleBuffer sizeof",
        size_of::<GimbalTripleBuffer>(),
        192,
    );
    check(
        "GimbalTripleBuffer alignof",
        align_of::<GimbalTripleBuffer>(),
        64,
    );
    check("ShmMetaRegion sizeof", size_of::<ShmMetaRegion>(), 3712);

    println!("--- 字段偏移 -----------------------------------------------------------");
    check("ImageMeta::seq offset", offset_of!(ImageMeta, seq), 0);
    check(
        "ImageMeta::timestamp_ns offset",
        offset_of!(ImageMeta, timestamp_ns),
        8,
    );
    check("ImageMeta::width offset", offset_of!(ImageMeta, width), 16);
    check(
        "ImageMeta::height offset",
        offset_of!(ImageMeta, height),
        20,
    );
    check(
        "ImageMeta::buffer_id offset",
        offset_of!(ImageMeta, buffer_id),
        24,
    );
    check(
        "ImageMeta::format offset",
        offset_of!(ImageMeta, format),
        25,
    );

    check(
        "PoseMeta::frame_seq offset",
        offset_of!(PoseMeta, frame_seq),
        0,
    );
    check(
        "PoseMeta::position offset",
        offset_of!(PoseMeta, position),
        8,
    );
    check(
        "PoseMeta::quaternion offset",
        offset_of!(PoseMeta, quaternion),
        20,
    );
    check(
        "PoseMeta::timestamp_ns offset",
        offset_of!(PoseMeta, timestamp_ns),
        40,
    );

    check(
        "GimbalCmd::timestamp_ns offset",
        offset_of!(GimbalCmd, timestamp_ns),
        0,
    );
    check(
        "GimbalCmd::yaw_deg offset",
        offset_of!(GimbalCmd, yaw_deg),
        8,
    );
    check(
        "GimbalCmd::pitch_deg offset",
        offset_of!(GimbalCmd, pitch_deg),
        12,
    );
    check(
        "GimbalCmd::distance_m offset",
        offset_of!(GimbalCmd, distance_m),
        16,
    );
    check(
        "GimbalCmd::fire_advice offset",
        offset_of!(GimbalCmd, fire_advice),
        20,
    );

    check("ShmHeader::magic offset", offset_of!(ShmHeader, magic), 0);
    check(
        "ShmHeader::version offset",
        offset_of!(ShmHeader, version),
        4,
    );
    check(
        "ShmHeader::created_ns offset",
        offset_of!(ShmHeader, created_ns),
        8,
    );
    check(
        "ShmHeader::heartbeat_ns offset",
        offset_of!(ShmHeader, heartbeat_ns),
        16,
    );
    check(
        "ShmHeader::image_width offset",
        offset_of!(ShmHeader, image_width),
        24,
    );
    check(
        "ShmHeader::image_height offset",
        offset_of!(ShmHeader, image_height),
        28,
    );

    check(
        "CameraInfo::timestamp_ns offset",
        offset_of!(CameraInfo, timestamp_ns),
        0,
    );
    check("CameraInfo::fx offset", offset_of!(CameraInfo, fx), 8);
    check("CameraInfo::fy offset", offset_of!(CameraInfo, fy), 16);
    check("CameraInfo::cx offset", offset_of!(CameraInfo, cx), 24);
    check("CameraInfo::cy offset", offset_of!(CameraInfo, cy), 32);
    check(
        "CameraInfo::distortion offset",
        offset_of!(CameraInfo, distortion),
        40,
    );
    check(
        "CameraInfo::width offset",
        offset_of!(CameraInfo, width),
        80,
    );
    check(
        "CameraInfo::height offset",
        offset_of!(CameraInfo, height),
        84,
    );

    check(
        "GroundTruthTarget::frame_seq offset",
        offset_of!(GroundTruthTarget, frame_seq),
        0,
    );
    check(
        "GroundTruthTarget::timestamp_ns offset",
        offset_of!(GroundTruthTarget, timestamp_ns),
        8,
    );
    check(
        "GroundTruthTarget::team offset",
        offset_of!(GroundTruthTarget, team),
        16,
    );
    check(
        "GroundTruthTarget::armor_label offset",
        offset_of!(GroundTruthTarget, armor_label),
        17,
    );
    check(
        "GroundTruthTarget::is_outpost offset",
        offset_of!(GroundTruthTarget, is_outpost),
        18,
    );
    check(
        "GroundTruthTarget::position offset",
        offset_of!(GroundTruthTarget, position),
        20,
    );
    check(
        "GroundTruthTarget::vyaw offset",
        offset_of!(GroundTruthTarget, vyaw),
        32,
    );
    check(
        "GroundTruthTarget::yaw offset",
        offset_of!(GroundTruthTarget, yaw),
        36,
    );

    check(
        "GroundTruthBatch::frame_seq offset",
        offset_of!(GroundTruthBatch, frame_seq),
        0,
    );
    check(
        "GroundTruthBatch::timestamp_ns offset",
        offset_of!(GroundTruthBatch, timestamp_ns),
        8,
    );
    check(
        "GroundTruthBatch::target_count offset",
        offset_of!(GroundTruthBatch, target_count),
        16,
    );
    check(
        "GroundTruthBatch::rune_count offset",
        offset_of!(GroundTruthBatch, rune_count),
        20,
    );
    check(
        "GroundTruthBatch::targets offset",
        offset_of!(GroundTruthBatch, targets),
        32,
    );
    check(
        "GroundTruthBatch::runes offset",
        offset_of!(GroundTruthBatch, runes),
        1088,
    );

    check(
        "RuntimeState::timestamp_ns offset",
        offset_of!(RuntimeState, timestamp_ns),
        0,
    );
    check(
        "RuntimeState::following offset",
        offset_of!(RuntimeState, following),
        8,
    );

    println!("--- 三缓冲字段偏移 -----------------------------------------------------");
    check(
        "ImageTripleBuffer::state offset",
        offset_of!(ImageTripleBuffer, state),
        0,
    );
    check(
        "ImageTripleBuffer::write_idx offset",
        offset_of!(ImageTripleBuffer, write_idx),
        1,
    );
    check(
        "ImageTripleBuffer::read_idx offset",
        offset_of!(ImageTripleBuffer, read_idx),
        2,
    );
    check(
        "ImageTripleBuffer::slots offset",
        offset_of!(ImageTripleBuffer, slots),
        64,
    );
    check(
        "PoseTripleBuffer::slots offset",
        offset_of!(PoseTripleBuffer, slots),
        64,
    );
    check(
        "GimbalTripleBuffer::slots offset",
        offset_of!(GimbalTripleBuffer, slots),
        64,
    );

    println!("--- ShmMetaRegion 布局 -------------------------------------------------");
    check(
        "ShmMetaRegion::header offset",
        offset_of!(ShmMetaRegion, header),
        0,
    );
    check(
        "ShmMetaRegion::image offset",
        offset_of!(ShmMetaRegion, image),
        64,
    );
    check(
        "ShmMetaRegion::poses offset",
        offset_of!(ShmMetaRegion, poses),
        256,
    );
    check(
        "ShmMetaRegion::gimbal_cmd offset",
        offset_of!(ShmMetaRegion, gimbal_cmd),
        1536,
    );
    check(
        "ShmMetaRegion::camera_info offset",
        offset_of!(ShmMetaRegion, camera_info),
        1728,
    );
    check(
        "ShmMetaRegion::chassis_observation offset",
        offset_of!(ShmMetaRegion, chassis_observation),
        1856,
    );
    check(
        "ShmMetaRegion::ground_truth offset",
        offset_of!(ShmMetaRegion, ground_truth),
        1984,
    );
    check(
        "ShmMetaRegion::runtime_state offset",
        offset_of!(ShmMetaRegion, runtime_state),
        3648,
    );

    assert_eq!(failures, 0, "{failures} 项 ABI 与权威表不一致");
}
