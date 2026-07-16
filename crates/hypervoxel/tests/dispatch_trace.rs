#![cfg(feature = "dispatch-trace")]

use hyperreal::{Rational, Real};
use hypervoxel::{ExactBox, GridFrame, MaterialRegionId, VoxelizationPolicy, voxelize_exact_box};

fn q(numerator: i64, denominator: u64) -> Real {
    Rational::fraction(numerator, denominator)
        .expect("nonzero denominator")
        .into()
}

#[test]
fn exact_rational_voxelization_does_not_request_approximation() {
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();

    let frame = GridFrame::builder()
        .origin([q(0, 1), q(0, 1), q(0, 1)])
        .pitch([q(1, 3), q(1, 3), q(1, 3)])
        .depth(2)
        .build()
        .unwrap();
    let solid = ExactBox::new(
        [q(1, 3), q(1, 3), q(1, 3)],
        [q(2, 3), q(2, 3), q(2, 3)],
        None,
    );
    let (_, report) = voxelize_exact_box(
        frame,
        &solid,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    )
    .unwrap();
    assert!(report.exact_topology_ready());

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.rational_reductions > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
