//! Process-boundary source replay followed by exact width reconstruction and selection.
//!
//! The parent sends JSON source vectors to a child running this same test. Width arrays
//! are then serialized/reloaded within the child; they are not the process payload.
//! A readable preexisting marker selects child assertions directly. The harness captures
//! child output and checks exit success, with no internal timeout or sandbox.

#![cfg(all(feature = "fixed", feature = "serde"))]
use hyperlattice_interval::{
    IntervalVector,
    hyperinterval::{FixedInterval, FixedWidth},
};
/// Signed 64-bit endpoints at 63 fractional bits for the process payload.
type I = FixedInterval<i64, 63>;
#[test]
/// Launch a child when the marker is absent, otherwise validate the supplied source JSON.
/// The fixed source must re-encode identically and select its full-range second lane;
/// reloaded unsigned widths preserve order and fail signed projection when too large.
/// With float-enclosure, a second environment payload supplies f64 endpoint definitions:
/// exact widths remain ordered even though their rounded enclosures are equal. Float
/// encoding byte identity is not asserted. The parent reports captured output on failure;
/// external environment setup determines whether a new process is actually launched.
fn fresh_process_replay_preserves_exact_widths_and_selection() {
    /// Environment marker and exact fixed-vector payload selecting the child branch.
    const KEY: &str = "HYPERLATTICE_WIDTH_REPLAY";
    if let Ok(text) = std::env::var(KEY) {
        let source: IntervalVector<I, 2> = serde_json::from_str(&text).unwrap();
        assert_eq!(serde_json::to_string(&source).unwrap(), text);
        assert_eq!(source.widest_axis(&mut ()).unwrap(), Some(1));
        let widths = source.widths(&mut ()).unwrap().unwrap();
        let text = serde_json::to_string(&widths).unwrap();
        let replay: [FixedWidth<63>; 2] = serde_json::from_str(&text).unwrap();
        assert_eq!(replay, widths);
        assert_eq!(replay[1].numerator(), u64::MAX);
        assert!(replay[1] > replay[0]);
        assert!(replay[1].to_interval().is_err());
        #[cfg(feature = "float-enclosure")]
        {
            use hyperlattice_interval::hyperinterval::{F64Enclosure as F, F64Width};
            let text = std::env::var("HYPERLATTICE_FLOAT_WIDTH_REPLAY").unwrap();
            let replay: IntervalVector<F, 2> = serde_json::from_str(&text).unwrap();
            assert_eq!(replay.widest_axis(&mut ()).unwrap(), Some(1));
            let text = serde_json::to_string(&replay.widths(&mut ()).unwrap().unwrap()).unwrap();
            let widths: [F64Width; 2] = serde_json::from_str(&text).unwrap();
            assert!(widths[1] > widths[0]);
            assert_eq!(
                widths[0].enclosure().unwrap(),
                widths[1].enclosure().unwrap()
            );
        }
        return;
    }
    let source =
        IntervalVector::new([I::new(-1, 1).unwrap(), I::new(i64::MIN, i64::MAX).unwrap()]).unwrap();
    let mut command = std::process::Command::new(std::env::current_exe().unwrap());
    command
        .args([
            "--exact",
            "fresh_process_replay_preserves_exact_widths_and_selection",
            "--nocapture",
        ])
        .env(KEY, serde_json::to_string(&source).unwrap());
    #[cfg(feature = "float-enclosure")]
    {
        use hyperlattice_interval::hyperinterval::F64Enclosure as F;
        let source = IntervalVector::new([
            F::new(f64::from_bits(2), 1.).unwrap(),
            F::new(f64::from_bits(1), 1.).unwrap(),
        ])
        .unwrap();
        command.env(
            "HYPERLATTICE_FLOAT_WIDTH_REPLAY",
            serde_json::to_string(&source).unwrap(),
        );
    }
    let output = command.output().unwrap();
    assert!(
        output.status.success(),
        "{} {}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}
