#![no_main]

use hyperreal::{Rational, Real};
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchArcLengthSweep,
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let x = value(0);
    let y = value(2);
    let z = value(4);
    let radius = value(6).unsigned_abs().clamp(1, 32) as i64;
    let three_halves = Real::new(Rational::fraction(3, 2).unwrap());

    let mut sketch = SketchSolveProblem::new();
    let origin3 = sketch.add_point3d("origin3", Real::from(0), Real::from(0), Real::from(0));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin3, normal);
    let center = sketch.add_point2d("center", Real::from(x), Real::from(y));
    let start = sketch.add_point2d("start", Real::from(x + radius), Real::from(y));
    let end = sketch.add_point2d("end", Real::from(x), Real::from(y + radius));
    let radius_entity = sketch.add_distance("radius", Real::from(radius));
    let arc = sketch.add_arc_of_circle2("quarter arc", center, start, end, radius_entity);
    let line_start = sketch.add_point3d("line start", Real::from(0), Real::from(0), Real::from(z));
    let line_end = sketch.add_point3d(
        "line end",
        Real::from(radius) * Real::pi() * three_halves,
        Real::from(0),
        Real::from(z + 3),
    );
    let line = sketch.add_line_segment3("projected major line", line_start, line_end);
    let handle = sketch.add_projected_equal_line_arc_sweep_length3(
        "projected sweep length",
        workplane,
        line,
        arc,
        SketchArcLengthSweep::ClockwiseMajor,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 5);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..]
        .iter()
        .all(|row| row.strategy == Some(SketchResidualStrategy::ProjectedLineArcSweepLength)));

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(certification.all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ProjectedLineArcSweepLengthTranscendentalEquality
    );
});
