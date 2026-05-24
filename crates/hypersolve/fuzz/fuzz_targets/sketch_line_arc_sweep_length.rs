#![no_main]

use hyperreal::{Rational, Real};
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchArcLengthSweep,
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let x = value(0);
    let y = value(2);
    let radius = value(4).unsigned_abs().clamp(1, 32) as i64;
    let three_halves = Real::new(Rational::fraction(3, 2).unwrap());

    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", Real::from(x), Real::from(y));
    let start = sketch.add_point2d("start", Real::from(x + radius), Real::from(y));
    let end = sketch.add_point2d("end", Real::from(x), Real::from(y + radius));
    let radius_entity = sketch.add_distance("radius", Real::from(radius));
    let arc = sketch.add_arc_of_circle2("quarter arc", center, start, end, radius_entity);
    let line_start = sketch.add_point2d("line start", Real::from(0), Real::from(0));
    let line_end = sketch.add_point2d(
        "line end",
        Real::from(radius) * Real::pi() * three_halves,
        Real::from(0),
    );
    let line = sketch.add_line_segment2("major line", line_start, line_end);
    let handle = sketch.add_equal_line_arc_sweep_length2(
        "clockwise major sweep",
        line,
        arc,
        SketchArcLengthSweep::ClockwiseMajor,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 4);
    assert!(lowered
        .rows
        .iter()
        .all(|row| { row.strategy == Some(SketchResidualStrategy::LineArcSweepLength) }));

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(certification.all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 4);
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::LineArcSweepLengthBranchPredicate
    );
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::LineArcSweepLengthTranscendentalEquality
    );
});
