#![no_main]

use hyperreal::{Real, RealSign};
use hypersolve::{
    PreparedProblem, SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy,
    SketchSolveProblem, certify_candidate, context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 4]| {
    let ax = i64::from(data[0] % 16);
    let ay = i64::from(data[1] % 16);
    let mut dx = i64::from(data[2] % 8);
    let dy = i64::from(data[3] % 8);
    if dx == 0 && dy == 0 {
        dx = 1;
    }

    let mut sketch = SketchSolveProblem::new();
    let a0 = sketch.add_point2d("a0", Real::from(ax), Real::from(ay));
    let a1 = sketch.add_point2d("a1", Real::from(ax + dx), Real::from(ay + dy));
    let a2 = sketch.add_point2d("a2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
    let join = sketch.add_point2d("join", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
    let b1 = sketch.add_point2d("b1", Real::from(ax + 4 * dx), Real::from(ay + 4 * dy));
    let b2 = sketch.add_point2d("b2", Real::from(ax + 5 * dx), Real::from(ay + 5 * dy));
    let b3 = sketch.add_point2d("b3", Real::from(ax + 6 * dx), Real::from(ay + 6 * dy));
    let first = sketch.add_cubic2("first", a0, a1, a2, join);
    let second = sketch.add_cubic2("second", join, b1, b2, b3);
    let first_parameter = sketch.add_parameter("first t", Real::from(1));
    let second_parameter = sketch.add_parameter("second t", Real::from(0));
    let handle = sketch.add_cubic_cubic_g2_continuity2(
        "cubic g2",
        first,
        first_parameter,
        second,
        second_parameter,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 6);
    assert!(lowered.rows.iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::CubicCubicG2Continuity)
    }));
    let context = context_from_problem(&lowered.problem);
    assert!(certify_candidate(&PreparedProblem::new(&lowered.problem), &context).all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 6);
    for form in [
        &forms.forms[0],
        &forms.forms[1],
        &forms.forms[2],
        &forms.forms[4],
        &forms.forms[5],
    ] {
        assert_eq!(
            form.residual.eval_real(context.bindings()).unwrap(),
            Real::zero()
        );
    }
    assert_eq!(
        forms.forms[3]
            .residual
            .eval_real(context.bindings())
            .unwrap()
            .structural_facts()
            .sign,
        Some(RealSign::Positive)
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::CubicCubicG2CurvatureMagnitudePolynomial
    );
});
