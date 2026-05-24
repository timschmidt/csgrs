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
    let cx = value(0);
    let cy = value(2);
    let radius = value(4).abs() + 1;
    let half_length = value(6).abs() + 1;
    let ox = value(8);
    let oy = value(10);
    let oz = value(12);
    let z_delta = value(14);

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
    let radius_entity = sketch.add_distance("radius", Real::from(radius));
    let circle = sketch.add_circle2("circle", center, radius_entity);
    let start = sketch.add_point3d(
        "start",
        Real::from(ox + cx - half_length),
        Real::from(oy + cy + radius),
        Real::from(oz),
    );
    let end = sketch.add_point3d(
        "end",
        Real::from(ox + cx + half_length),
        Real::from(oy + cy + radius),
        Real::from(oz + z_delta),
    );
    let line = sketch.add_line_segment3("line", start, end);
    let handle = sketch.add_projected_line_circle_tangent3(
        "projected line circle tangent",
        workplane,
        line,
        circle,
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
        Some(SketchResidualStrategy::ProjectedLineCircleTangency)
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
        SketchResidualFormKind::ProjectedLineCircleTangencyPolynomial
    );
});
