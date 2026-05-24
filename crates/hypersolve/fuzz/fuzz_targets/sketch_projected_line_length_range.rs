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
    let lx = value(0);
    let ly = value(2);
    let lz = value(4);
    let length = value(6).abs() + 1;
    let slack = value(8).abs();
    let lower = (length - slack).max(0);
    let upper = length + slack;
    let line_z_offset = value(10);
    let origin_z = value(12);

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(origin_z));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let line_start =
        sketch.add_point3d("line start", Real::from(lx), Real::from(ly), Real::from(lz));
    let line_end = sketch.add_point3d(
        "line end",
        Real::from(lx + length),
        Real::from(ly),
        Real::from(lz + line_z_offset),
    );
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let handle = sketch.add_projected_line_length_range3(
        "projected line length window",
        workplane,
        line,
        Some(Real::from(lower)),
        Some(Real::from(upper)),
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
        Some(SketchResidualStrategy::BoundedSquaredProjectedLineLength)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    assert!(certification.all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::BoundedSquaredProjectedLineLengthPolynomial
    );
});
