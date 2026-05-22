#![no_main]

use hyperreal::Real;
use hypersolve::{
    CertifiedCandidateStatus, PreparedProblem, SketchResidualStrategy, SketchSolveProblem,
    certify_candidate, context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let ax = value(0);
    let ay = value(2);
    let az = value(4);
    let u = value(6);
    let n = value(8);

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a = sketch.add_point3d("a", Real::from(ax), Real::from(ay), Real::from(az));
    let b = sketch.add_point3d("b", Real::from(ax + u), Real::from(ay), Real::from(az + n));
    let distance = sketch.add_distance("distance", Real::from(u.abs()));
    sketch.add_projected_point_point_distance("projected", workplane, a, b, distance);

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedDistance)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(matches!(
        certification.rows[1].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
});
