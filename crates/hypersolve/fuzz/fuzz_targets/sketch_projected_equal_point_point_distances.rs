#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, CertifiedCandidateStatus, PreparedProblem,
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let ax = value(0);
    let ay = value(2);
    let bx = value(4);
    let by = value(6);
    let dx = value(8);
    let dy = value(10);
    let az = value(12);
    let bz = value(14);

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
    let b = sketch.add_point3d(
        "b",
        Real::from(ax + dx),
        Real::from(ay + dy),
        Real::from(-az),
    );
    let c = sketch.add_point3d("c", Real::from(bx), Real::from(by), Real::from(bz));
    let d = sketch.add_point3d(
        "d",
        Real::from(bx + dx),
        Real::from(by + dy),
        Real::from(-bz),
    );
    let handle = sketch.add_projected_equal_point_point_distances3(
        "projected equal point distances",
        workplane,
        a,
        b,
        c,
        d,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedEqualPointPointDistances)
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

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedEqualPointPointDistancesPolynomial
    );
});
