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
    let mut dx = value(8);
    let dy = value(10);
    if dx == 0 && dy == 0 {
        dx = 1;
    }
    let scale = i64::from(data[12] % 4) + 1;
    let az = value(13);

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
    let a0 = sketch.add_point3d("a0", Real::from(ax), Real::from(ay), Real::from(az));
    let a1 = sketch.add_point3d(
        "a1",
        Real::from(ax + scale * dx),
        Real::from(ay + scale * dy),
        Real::from(az + 7),
    );
    let b0 = sketch.add_point3d("b0", Real::from(bx), Real::from(by), Real::from(-az));
    let b1 = sketch.add_point3d(
        "b1",
        Real::from(bx + dx),
        Real::from(by + dy),
        Real::from(-az - 11),
    );
    let a = sketch.add_line_segment3("a", a0, a1);
    let b = sketch.add_line_segment3("b", b0, b1);
    let handle = sketch.add_projected_length_ratio_lines3(
        "projected length ratio",
        workplane,
        a,
        b,
        Real::from(scale),
        Real::from(1),
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
        Some(SketchResidualStrategy::SquaredProjectedLineLengthRatio)
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
        SketchResidualFormKind::SquaredProjectedLineLengthRatioPolynomial
    );
});
