#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchGeneratedRowStatus,
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let mode = data[0] % 5;
    let cx = value(data, 1);
    let cy = value(data, 3);
    let radius = i64::from(data[5] % 16) + 1;
    let dx = if mode == 2 {
        i64::from(data[6] % 7) + 1
    } else {
        0
    };
    let dy = if mode == 3 {
        i64::from(data[7] % 7) + 1
    } else {
        0
    };

    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
    let other = sketch.add_point2d("other", Real::from(cx + dx), Real::from(cy + dy));
    let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
    let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
    let radius_entity = sketch.add_distance("radius", Real::from(radius));
    let circle = sketch.add_circle2("circle", center, radius_entity);
    let arc = sketch.add_arc_of_circle2("arc", other, start, end, radius_entity);
    let handle = if mode == 4 {
        sketch.add_concentric2("wrong concentric", circle, radius_entity)
    } else {
        sketch.add_concentric2("concentric", circle, arc)
    };

    let lowered = sketch.lower_to_problem();
    if mode == 4 {
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        let forms = sketch.residual_forms_for_constraint(handle);
        assert_eq!(forms.status, SketchResidualFormsStatus::InvalidInputs);
        return;
    }

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 2);
    assert!(lowered.rows.iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::Concentricity)
    }));
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    if dx == 0 && dy == 0 {
        assert!(certification.all_satisfied());
    } else {
        assert!(!certification.all_satisfied());
    }

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert!(forms
        .forms
        .iter()
        .all(|form| { form.kind == SketchResidualFormKind::ConcentricCenterCoordinatePolynomial }));
});
