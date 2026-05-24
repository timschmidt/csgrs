#![no_main]

use hyperreal::Real;
use hypersolve::{
    PreparedProblem, SketchLineEndpoint, SketchResidualFormKind, SketchResidualFormsStatus,
    SketchResidualStrategy, SketchSolveProblem, certify_candidate, context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 6]| {
    let ax = i64::from(data[0] % 16);
    let ay = i64::from(data[1] % 16);
    let mut dx = i64::from(data[2] % 8);
    let dy = i64::from(data[3] % 8);
    if dx == 0 && dy == 0 {
        dx = 1;
    }
    let t = i64::from(data[4] % 4);
    let tangent_scale = i64::from((data[5] % 4).abs()) + 1;

    let mut sketch = SketchSolveProblem::new();
    let p0 = sketch.add_point2d("p0", Real::from(ax), Real::from(ay));
    let p1 = sketch.add_point2d("p1", Real::from(ax + dx), Real::from(ay + dy));
    let p2 = sketch.add_point2d("p2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
    let p3 = sketch.add_point2d("p3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let point = sketch.add_point2d(
        "point",
        Real::from(ax + 3 * dx * t),
        Real::from(ay + 3 * dy * t),
    );
    let tangent_end = sketch.add_point2d(
        "tangent end",
        Real::from(ax + 3 * dx * t + dx * tangent_scale),
        Real::from(ay + 3 * dy * t + dy * tangent_scale),
    );
    let line = sketch.add_line_segment2("line", point, tangent_end);
    let parameter = sketch.add_parameter("t", Real::from(t));
    let handle =
        sketch.add_cubic_line_tangent2("cubic tangent", cubic, parameter, line, SketchLineEndpoint::Start);

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 4);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicLineTangent)
    }));
    let context = context_from_problem(&lowered.problem);
    assert!(certify_candidate(&PreparedProblem::new(&lowered.problem), &context).all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 4);
    for form in &forms.forms[..3] {
        assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
    }
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::CubicLineTangentDotProductPredicate
    );
});
