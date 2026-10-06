#![cfg(feature = "dispatch-trace")]

use hyperpack::{
    AxisBox3, Bin3, FeasibilityStatus, Item3, ItemId, Placement3, Real, pair_incompatibilities_3d,
    verify_packing_3d,
};
use hyperreal::Rational;

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
}

#[test]
fn exact_replay_and_bounds_do_not_request_approximation() {
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();

    let bin = Bin3 {
        size: AxisBox3::new(q(8, 3), q(4, 3), q(2, 3)).unwrap(),
    };
    let items = vec![
        Item3 {
            id: ItemId::new("left").unwrap(),
            size: AxisBox3::new(q(2, 3), q(4, 3), q(2, 3)).unwrap(),
        },
        Item3 {
            id: ItemId::new("right").unwrap(),
            size: AxisBox3::new(q(2, 3), q(4, 3), q(2, 3)).unwrap(),
        },
    ];
    let placements = vec![
        Placement3 {
            item: items[0].id.clone(),
            x: q(0, 1),
            y: q(0, 1),
            z: q(0, 1),
        },
        Placement3 {
            item: items[1].id.clone(),
            x: q(2, 3),
            y: q(0, 1),
            z: q(0, 1),
        },
    ];

    assert_eq!(
        verify_packing_3d(&bin, &items, &placements)
            .unwrap()
            .feasibility
            .status,
        FeasibilityStatus::Feasible
    );
    let bounds = pair_incompatibilities_3d(&bin, &items);
    assert!(bounds.incompatible_pairs.is_empty());

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.rational_reductions > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
