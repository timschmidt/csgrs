#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchResidualFormKind,
    SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 18 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let ox = value(0);
    let oy = value(2);
    let oz = value(4);
    let dx = value(6);
    let dy = value(8);
    if dx == 0 && dy == 0 {
        return;
    }
    let parallel_scale = value(10).abs() + 1;
    let perpendicular_scale = value(12).abs() + 1;
    let a_z = value(14);
    let b_z = value(16);

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
    let base = sketch.add_point3d("base", Real::from(ox), Real::from(oy), Real::from(oz));
    let a_end = sketch.add_point3d(
        "a end",
        Real::from(ox + dx),
        Real::from(oy + dy),
        Real::from(oz + a_z),
    );
    let b_end = sketch.add_point3d(
        "b end",
        Real::from(ox + parallel_scale * dx),
        Real::from(oy + parallel_scale * dy),
        Real::from(oz + b_z),
    );
    let c_end = sketch.add_point3d(
        "c end",
        Real::from(ox - perpendicular_scale * dy),
        Real::from(oy + perpendicular_scale * dx),
        Real::from(oz - b_z),
    );
    let a = sketch.add_line_segment3("a", base, a_end);
    let b = sketch.add_line_segment3("b", base, b_end);
    let c = sketch.add_line_segment3("c", base, c_end);
    let parallel = sketch.add_projected_parallel_lines3("projected parallel", workplane, a, b);
    let perpendicular =
        sketch.add_projected_perpendicular_lines3("projected perpendicular", workplane, a, c);

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 4);
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::ProjectedDirectionCrossProduct)
    );
    assert_eq!(
        lowered.rows[3].strategy,
        Some(SketchResidualStrategy::ProjectedDirectionDotProduct)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(certification.all_satisfied());

    let parallel_forms = sketch.residual_forms_for_constraint(parallel);
    assert_eq!(parallel_forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(
        parallel_forms.forms[1].kind,
        SketchResidualFormKind::ProjectedDirectionCrossProductPolynomial
    );
    let perpendicular_forms = sketch.residual_forms_for_constraint(perpendicular);
    assert_eq!(
        perpendicular_forms.status,
        SketchResidualFormsStatus::Generated
    );
    assert_eq!(
        perpendicular_forms.forms[1].kind,
        SketchResidualFormKind::ProjectedDirectionDotProductPolynomial
    );
});
