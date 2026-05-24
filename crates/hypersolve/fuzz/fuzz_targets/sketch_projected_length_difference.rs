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
    let shorter = value(4).abs();
    let difference = value(6).abs();
    let z0 = value(8);
    let z1 = value(10);
    let z2 = value(12);
    let longer = shorter + difference;

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
    let long0 = sketch.add_point3d("long0", Real::from(x0), Real::from(y0), Real::from(z0));
    let long1 = sketch.add_point3d(
        "long1",
        Real::from(x0 + longer),
        Real::from(y0),
        Real::from(z1),
    );
    let short0 = sketch.add_point3d("short0", Real::from(x0), Real::from(y0 + 3), Real::from(z2));
    let short1 = sketch.add_point3d(
        "short1",
        Real::from(x0 + shorter),
        Real::from(y0 + 3),
        Real::from(-z2),
    );
    let longer = sketch.add_line_segment3("longer", long0, long1);
    let shorter = sketch.add_line_segment3("shorter", short0, short1);
    let difference = sketch.add_distance("difference", Real::from(difference));
    let handle = sketch.add_projected_length_difference_lines3(
        "projected length difference",
        workplane,
        longer,
        shorter,
        difference,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedLineLengthDifference)
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
    assert!(matches!(
        certification.rows[2].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
            | CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. }
    ));

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedLineLengthDifferencePolynomial
    );
});
