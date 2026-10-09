// Untimed replay of the five changed workloads, linked to both retained libraries.
// Reuse the measured board, feature and query helpers without changing their inputs.
#![allow(dead_code)]
use hyperdrc::{PcbRegion, Scalar};
use std::{hint::black_box, path::Path, time::Instant};

// These two functions are copied byte-for-byte from operation_probe.rs.
fn measure_value<T>(
    label: &str,
    iterations: usize,
    mut query: impl FnMut() -> T,
    check: impl Fn(&T) -> String,
) {
    let expected = check(&query());
    let started = Instant::now();
    for _ in 0..iterations {
        black_box(query());
    }
    let elapsed = started.elapsed().as_nanos();
    assert_eq!(check(&query()), expected);
    println!("operation_{label}\t{iterations}\t{elapsed}\t{expected}");
}

fn findings_witness(findings: &[hyperdrc::Violation], count: usize, check: &str) -> String {
    assert_eq!(findings.len(), count, "{findings:?}");
    assert!(findings.iter().all(|finding| finding.check == check));
    let bytes = serde_json::to_vec(findings).unwrap();
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in &bytes {
        hash ^= u64::from(*byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    format!("{count}:{}:{hash}", bytes.len())
}

fn save(out: &Path, label: &str, findings: Vec<hyperdrc::Violation>) {
    std::fs::write(out.join(format!("{label}.json")), serde_json::to_vec(&findings).unwrap()).unwrap();
}

fn original_rectangle(region: &PcbRegion) {
    match hyperdrc::readiness::CHECK_IMPLEMENTATION_VERSION {
        16 => {
            assert!(region.try_bounding_box().is_err());
            assert!(region.try_area().is_err());
        }
        17 => {
            assert!(region.try_bounding_box().unwrap().is_some());
            assert_eq!(region.try_area().unwrap(), Scalar::from(268_435_456));
        }
        version => panic!("unexpected implementation version: {version}"),
    }
}

mod dense_cluster_probe {
    include!("dense_cluster_probe.rs");

    pub(super) fn audit(out: &Path) {
        let one = Scalar::one();
        for rule in [Rule::ViaSpacing, Rule::MaskBridge] {
            let mut board = rule.bad_board(Scalar::zero());
            board.copper[2].region = board.copper[1].region.clone();
            if matches!(rule, Rule::ViaSpacing) {
                board.copper[1].region = board.copper[16].region.clone();
            }
            board.copper[0].region = rectangle(1e20, 16_384.0);
            original_rectangle(&board.copper[0].region);
            let label = format!("operation_dense_{rule:?}_optional_minimum").to_lowercase();
            save(out, &label, rule.run(&board, &one, &one));

            let mut board = rule.board(Scalar::zero());
            board.copper[0].region = rectangle(1e20, 16_384.0);
            original_rectangle(&board.copper[0].region);
            let label = format!("operation_dense_{rule:?}_required_large_shape").to_lowercase();
            save(out, &label, rule.run(&board, &one, &one));
        }
    }
}

mod fixture_access_probe {
    include!("fixture_access_probe.rs");

    pub(super) fn audit(out: &Path) {
        let one = Scalar::one();
        let zero = Scalar::zero();
        let mut f = feature(CopperKind::Pad, "OTHER", "F.Cu", zero.clone());
        f.region = rectangle(1e20, 16_384.0);
        original_rectangle(&f.region);
        let b = board(vec![f]);
        let points = [point(0.into())];
        save(out, "operation_fixture_copper_large_shape", testpoint_copper_clearance_readiness(
            &b, &points, &[], &one, &one, &zero,
        ));
    }
}

fn main() {
    let out = std::env::args_os().nth(1).unwrap();
    let out = Path::new(&out);
    std::fs::create_dir_all(out).unwrap();
    dense_cluster_probe::audit(out);
    fixture_access_probe::audit(out);
}
