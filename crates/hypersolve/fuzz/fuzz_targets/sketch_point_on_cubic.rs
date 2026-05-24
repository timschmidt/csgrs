#![no_main]

use hyperreal::Real;
use hypersolve::{
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
    context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 5]| {
    let ax = i64::from(data[0] % 16);
    let ay = i64::from(data[1] % 16);
    let dx = i64::from(data[2] % 8);
    let dy = i64::from(data[3] % 8);
    let t = i64::from(data[4] % 4);

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
    let parameter = sketch.add_parameter("t", Real::from(t));
    let handle = sketch.add_point_on_cubic2("point on cubic", point, cubic, parameter);

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 2);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicBezierIncidence)
    }));

    let forms = sketch.residual_forms_for_constraint(handle);
    let context = context_from_problem(&lowered.problem);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    for form in forms.forms {
        assert_eq!(form.kind, SketchResidualFormKind::CubicBezierIncidencePolynomial);
        assert_eq!(form.residual.eval_real(context.bindings()).unwrap(), Real::zero());
    }
});
