#[path = "counters_v1.rs"]
mod counters;

use std::hint::black_box;

fn burn() -> u64 {
    (0..100_000_u64).fold(0, |sum, value| black_box(sum) + black_box(value))
}

fn fd_count() -> usize {
    std::fs::read_dir("/proc/self/fd")
        .expect("own descriptor inventory")
        .count()
}

fn main() {
    let before_fds = fd_count();
    let group = counters::Group::open().expect("open cycle/instruction group");
    assert_eq!(fd_count(), before_fds + 2);
    let (_, empty) = group.observe(|| black_box(()));
    let (sum, busy) = group.observe(burn);
    assert_eq!(sum, 100_000_u64 * 99_999 / 2);
    assert!(empty.valid(), "empty: {empty:?}");
    assert!(busy.valid(), "busy: {busy:?}");
    assert!(busy.after.unwrap().instructions > empty.after.unwrap().instructions + 100_000);
    assert!(busy.after.unwrap().cycles > empty.after.unwrap().cycles);
    let before = group.snapshot_for_control().unwrap();
    black_box(burn());
    let after = group.snapshot_for_control().unwrap();
    assert_eq!(before.cycles, after.cycles);
    assert_eq!(before.instructions, after.instructions);
    assert_eq!(before.enabled_ns, after.enabled_ns);
    assert_eq!(before.running_ns, after.running_ns);
    let unwind = std::panic::catch_unwind(|| {
        group.observe(|| panic!("intentional counter-scope unwind control"));
    });
    assert!(unwind.is_err());
    let before = group.snapshot_for_control().unwrap();
    black_box(burn());
    let after = group.snapshot_for_control().unwrap();
    assert_eq!(before.cycles, after.cycles);
    assert_eq!(before.instructions, after.instructions);
    assert_eq!(before.enabled_ns, after.enabled_ns);
    assert_eq!(before.running_ns, after.running_ns);
    let (_, replay) = group.observe(|| black_box(()));
    assert!(replay.valid(), "replay: {replay:?}");
    let (value, missing) =
        counters::unavailable(|| black_box(17), "intentional unavailable control");
    assert_eq!(value, 17);
    assert!(!missing.valid() && missing.before.is_none() && missing.after.is_none());
    drop(group);
    assert_eq!(fd_count(), before_fds);
    println!(
        "{}",
        serde_json::json!({
            "control": "counter-api", "passed": true, "empty": empty.json(),
            "busy": busy.json(), "replay": replay.json(), "unavailable": missing.json(),
            "normal_and_unwind_disable_verified": true,
            "both_descriptors_closed": true,
        })
    );
}
