#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchArcPointSweep,
    SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let cx = value(0);
    let cy = value(2);
    let radius = value(4).unsigned_abs().clamp(1, 32) as i64;

    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
    let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
    let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
    let radius_entity = sketch.add_distance("radius", Real::from(radius));
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
    let point = sketch.add_point2d("point", Real::from(cx + radius), Real::from(cy));
    let handle = sketch.add_point_on_arc2(
        "point on arc",
        point,
        arc,
        SketchArcPointSweep::CounterClockwiseMinor,
    );

    let lowered = sketch.lower_to_problem();
    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 6);
    assert!(lowered
        .rows
        .iter()
        .all(|row| { row.strategy == Some(SketchResidualStrategy::PointArcIncidence) }));

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(certification.all_satisfied());

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(
        forms.forms[3].kind,
        SketchResidualFormKind::ArcIncidenceSweepBranchPredicate
    );
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::ArcIncidencePointBranchPredicate
    );
});
