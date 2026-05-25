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
    if data.len() < 14 {
        return;
    }
    let mode = data[0] % 4;
    let x0 = value(data, 1);
    let y0 = value(data, 3);
    let z0 = value(data, 5);
    let z1 = value(data, 7);
    let z2 = value(data, 9);
    let z3 = value(data, 11);
    let shorter = i64::from(data[13] % 16);
    let difference = i64::from(data[12] % 16);
    let bad_delta = if mode == 2 {
        i64::from(data[13] % 7) + 1
    } else {
        0
    };
    let longer = shorter + difference + bad_delta;

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
    let long0 = sketch.add_point3d("long0", Real::from(x0), Real::from(y0), Real::from(z0));
    let long1 = sketch.add_point3d(
        "long1",
        Real::from(x0 + longer),
        Real::from(y0),
        Real::from(z1),
    );
    let short0 = sketch.add_point3d("short0", Real::from(x0), Real::from(y0 + 4), Real::from(z2));
    let short1 = sketch.add_point3d(
        "short1",
        Real::from(x0 + shorter),
        Real::from(y0 + 4),
        Real::from(z3),
    );
    let difference = sketch.add_distance("difference", Real::from(difference));
    let handle = if mode == 3 {
        let wrong = sketch.add_line_segment3("wrong line", long0, long1);
        sketch.add_projected_point_distance_difference3(
            "wrong projected point distance difference",
            workplane,
            (long0, wrong),
            (short0, short1),
            difference,
        )
    } else {
        sketch.add_projected_point_distance_difference3(
            "projected point distance difference",
            workplane,
            (long0, long1),
            (short0, short1),
            difference,
        )
    };

    let lowered = sketch.lower_to_problem();
    if mode == 3 {
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        let forms = sketch.residual_forms_for_constraint(handle);
        assert_eq!(forms.status, SketchResidualFormsStatus::InvalidInputs);
        return;
    }

    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedPointDistanceDifference)
    );
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    if bad_delta == 0 {
        assert!(certification.all_satisfied());
    } else {
        assert!(!certification.all_satisfied());
    }

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedPointDistanceDifferencePolynomial
    );
    assert_eq!(
        forms.forms[2].kind,
        SketchResidualFormKind::ProjectedPointDistanceDifferenceBranchPredicate
    );
});
