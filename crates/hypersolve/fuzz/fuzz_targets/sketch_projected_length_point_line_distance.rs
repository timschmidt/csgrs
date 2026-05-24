#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, CertifiedCandidateStatus, PreparedProblem,
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 14 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let x0 = value(0);
    let y0 = value(2);
    let length = value(4).abs();
    let z0 = value(6);
    let z1 = value(8);
    let z2 = value(10);
    let z3 = value(12);

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
    let len0 = sketch.add_point3d("len0", Real::from(x0), Real::from(y0 + 7), Real::from(z0));
    let len1 = sketch.add_point3d(
        "len1",
        Real::from(x0 + length),
        Real::from(y0 + 7),
        Real::from(z1),
    );
    let point = sketch.add_point3d(
        "point",
        Real::from(x0),
        Real::from(y0 + length),
        Real::from(z2),
    );
    let dist0 = sketch.add_point3d("dist0", Real::from(x0 - 1), Real::from(y0), Real::from(z3));
    let dist1 = sketch.add_point3d("dist1", Real::from(x0 + 1), Real::from(y0), Real::from(-z3));
    let length_line = sketch.add_line_segment3("length_line", len0, len1);
    let distance_line = sketch.add_line_segment3("distance_line", dist0, dist1);
    let handle = sketch.add_projected_equal_length_point_line_distance3(
        "projected length equals point-line distance",
        workplane,
        length_line,
        point,
        distance_line,
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
        Some(SketchResidualStrategy::SquaredProjectedLineLengthPointLineDistance)
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
        SketchResidualFormKind::SquaredProjectedLineLengthPointLineDistancePolynomial
    );
});
