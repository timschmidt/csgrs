#![cfg(feature = "dispatch-trace")]

use hyperlattice::{Matrix4, Vector3};
use hyperlimit::Point3;
use hyperreal::Real;
use hypersdf::{SdfExpr, prepare};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn exact_point_and_affine_interval_replay_do_not_request_approximation() {
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();

    let csg =
        prepare(SdfExpr::sphere(p(-4, 0, 0), r(25)).union(SdfExpr::sphere(p(4, 0, 0), r(25))));
    let point = csg.classify_point(&p(0, 0, 0));
    assert!(point.is_self_consistent());

    let shear_xy = Matrix4([
        [r(1), r(1), r(0), r(0)],
        [r(0), r(1), r(0), r(0)],
        [r(0), r(0), r(1), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let affine = prepare(
        SdfExpr::linear(Vector3([r(2), r(-3), r(5)]), r(-7))
            .affine_transform(shear_xy)
            .expect("invertible shear"),
    );
    assert!(
        affine
            .interval_cell(&p(-10, -10, -10), &p(10, 10, 10))
            .is_certified()
    );

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.dispatch_events > 0);
    assert!(correlation.sign_or_zero_query_events > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
