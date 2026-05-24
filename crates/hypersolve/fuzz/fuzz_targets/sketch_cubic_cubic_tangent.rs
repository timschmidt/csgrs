#![no_main]

use hyperreal::Real;
use hypersolve::{
    PreparedProblem, SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy,
    SketchSolveProblem, certify_candidate, context_from_problem,
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
    let second_scale = i64::from((data[5] % 4).abs()) + 1;
    let join_x = ax + 3 * dx * t;
    let join_y = ay + 3 * dy * t;

    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
    let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
    let a2 = sketch.add_point2d("a2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
    let a3 = sketch.add_point2d("a3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
    let b0 = sketch.add_point2d("b0", Real::from(join_x), Real::from(join_y));
    let b1 = sketch.add_point2d(
        "b1",
        Real::from(join_x + dx * second_scale),
        Real::from(join_y + dy * second_scale),
    );
    let b2 = sketch.add_point2d(
        "b2",
        Real::from(join_x + 2 * dx * second_scale),
        Real::from(join_y + 2 * dy * second_scale),
    );
    let b3 = sketch.add_point2d(
        "b3",
        Real::from(join_x + 3 * dx * second_scale),
        Real::from(join_y + 3 * dy * second_scale),
    );
    let first = sketch.add_cubic2("first", a0, a1, a2, a3);
    let second = sketch.add_cubic2("second", b0, b1, b2, b3);
    let first_parameter = sketch.add_parameter("first t", Real::from(t));
    let second_parameter = sketch.add_parameter("second t", Real::from(0));
    let handle = sketch.add_cubic_cubic_tangent2(
        "cubic cubic tangent",
        first,
        first_parameter,
        second,
        second_parameter,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 4);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicCubicTangent)
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
        SketchResidualFormKind::CubicCubicTangentDotProductPredicate
    );
});
