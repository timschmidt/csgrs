//! Fresh-process replay of a fixed matrix followed by norm and margin arithmetic.
//!
//! Only source geometry crosses the parent/child boundary. Derived dominance
//! margins also round-trip within the child before a further vector norm query.

#![cfg(all(feature = "fixed", feature = "serde"))]
use hyperlattice_interval::hyperinterval::FixedInterval;
use hyperlattice_interval::{IntervalMatrix, IntervalVector};
/// i32 fixed intervals at scale four, allowing exact small integer fixtures.
type I = FixedInterval<i32, 4>;
/// Build ordered integer-valued fixture bounds by multiplying raw endpoints by 16.
/// Callers supply small valid bounds. Multiplication is unchecked by this helper;
/// constructor failure panics.
fn scalar(a: i32, b: i32) -> I {
    I::new(a * 16, b * 16).unwrap()
}
#[test]
/// Replay source bounds, verify encoding identity and recompute exact expectations.
///
/// The parent invokes this exact test in the same executable and checks captured
/// exit status. A readable marker selects the child branch, so external runs must
/// clear it to exercise process creation. There is no internal timeout. Derived
/// norm ranges are later treated as independent vector lanes; their original
/// joint correlations are not asserted to survive that construction.
fn fresh_process_replay_recomputes_and_consumes_ranges() {
    /// Source-matrix JSON payload and child-branch marker; unset for a parent run.
    const KEY: &str = "HYPERLATTICE_NORM_REPLAY";
    if let Ok(text) = std::env::var(KEY) {
        let matrix: IntervalMatrix<I, 2, 2> = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&matrix).unwrap(), text);
        assert_eq!(matrix.norm_one(&mut ()).unwrap(), scalar(4, 5));
        assert_eq!(matrix.norm_infinity(&mut ()).unwrap(), scalar(5, 6));
        assert_eq!(matrix.measure_one(&mut ()).unwrap(), scalar(-2, -1));
        assert_eq!(matrix.measure_infinity(&mut ()).unwrap(), scalar(-3, -2));
        let margins = matrix.row_dominance_margins(&mut ()).unwrap();
        assert_eq!(margins.lanes(), Some(&[scalar(2, 3); 2]));
        let text = serde_json::to_string(&margins).unwrap();
        let replay: IntervalVector<I, 2> = serde_json::from_str(&text).unwrap();
        assert_eq!(replay.norm_one(&mut ()).unwrap(), scalar(4, 6));
        let norms = IntervalVector::new([
            matrix.norm_one(&mut ()).unwrap(),
            matrix.norm_infinity(&mut ()).unwrap(),
        ])
        .unwrap();
        assert_eq!(norms.norm_one(&mut ()).unwrap(), scalar(9, 11));
        return;
    }
    let matrix = IntervalMatrix::new([
        [scalar(-4, -4), scalar(1, 2)],
        [scalar(-1, 1), scalar(-3, -3)],
    ])
    .unwrap();
    let text = serde_json::to_string(&matrix).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "fresh_process_replay_recomputes_and_consumes_ranges",
            "--nocapture",
        ])
        .env(KEY, text)
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}
