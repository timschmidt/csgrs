#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchResidualFormKind,
    SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 14 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let ox = value(0);
    let oy = value(2);
    let oz = value(4);
    let cx = value(6);
    let cy = value(8);
    let normal_offset = value(10);
    let radius = value(12).unsigned_abs().clamp(1, 32) as i64;

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
    let point = sketch.add_point3d(
        "point",
        Real::from(ox + cx + radius),
        Real::from(oy + cy),
        Real::from(oz + normal_offset),
    );
    let handle = sketch.add_projected_point_on_circle3(
        "projected point on circle",
        workplane,
        point,
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
        Some(SketchResidualStrategy::ProjectedSquaredIncidence)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(certification.all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedCircleIncidencePolynomial
    );
});
