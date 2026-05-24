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
    let x0 = value(0);
    let y0 = value(2);
    let distance = value(4).abs();
    let z0 = value(6);
    let z1 = value(8);
    let z2 = value(10);
    let z3 = value(12);
    let z4 = value(14);

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
    let a_point = sketch.add_point3d(
        "a_point",
        Real::from(x0),
        Real::from(y0 + distance),
        Real::from(z0),
    );
    let a0 = sketch.add_point3d("a0", Real::from(x0 - 3), Real::from(y0), Real::from(z1));
    let a1 = sketch.add_point3d("a1", Real::from(x0 + 3), Real::from(y0), Real::from(z2));
    let b_point = sketch.add_point3d(
        "b_point",
        Real::from(x0 + 1),
        Real::from(y0 + 17 + distance),
        Real::from(z3),
    );
    let b0 = sketch.add_point3d(
        "b0",
        Real::from(x0 - 5),
        Real::from(y0 + 17),
        Real::from(z4),
    );
    let b1 = sketch.add_point3d(
        "b1",
        Real::from(x0 + 5),
        Real::from(y0 + 17),
        Real::from(-z4),
    );
    let a_line = sketch.add_line_segment3("a_line", a0, a1);
    let b_line = sketch.add_line_segment3("b_line", b0, b1);
    let handle = sketch.add_projected_equal_point_line_distances3(
        "projected equal point-line distances",
        workplane,
        a_point,
        a_line,
        b_point,
        b_line,
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
        Some(SketchResidualStrategy::SquaredProjectedEqualPointLineDistances)
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
        SketchResidualFormKind::SquaredProjectedEqualPointLineDistancesPolynomial
    );
});
