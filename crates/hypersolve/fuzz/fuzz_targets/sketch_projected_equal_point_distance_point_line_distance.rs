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
    let distance = i64::from(data[11] % 16);
    let signed_distance = if data[12] & 1 == 0 {
        distance
    } else {
        -distance
    };
    let bad_delta = if mode == 2 {
        i64::from(data[13] % 7) + 1
    } else {
        0
    };

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
    let a = sketch.add_point3d("a", Real::from(x0), Real::from(y0 + 9), Real::from(z0));
    let b = sketch.add_point3d(
        "b",
        Real::from(x0 + distance + bad_delta),
        Real::from(y0 + 9),
        Real::from(z1),
    );
    let point = sketch.add_point3d(
        "point",
        Real::from(x0),
        Real::from(y0 + signed_distance),
        Real::from(z2),
    );
    let line_start = sketch.add_point3d(
        "line start",
        Real::from(x0 - 5),
        Real::from(y0),
        Real::from(0),
    );
    let line_end = sketch.add_point3d(
        "line end",
        Real::from(x0 + 5),
        Real::from(y0),
        Real::from(i64::from(data[13])),
    );
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let handle = if mode == 3 {
        sketch.add_projected_equal_point_distance_point_line_distance3(
            "wrong projected point distance point-line distance",
            workplane,
            a,
            line,
            point,
            line,
        )
    } else {
        sketch.add_projected_equal_point_distance_point_line_distance3(
            "projected point distance point-line distance",
            workplane,
            a,
            b,
            point,
            line,
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

    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(lowered.rows.len(), 2);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedPointDistancePointLineDistance)
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
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedPointDistancePointLineDistancePolynomial
    );
});
