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
    if data.len() < 30 {
        return;
    }
    let mode = data[0] % 4;
    let ox = value(data, 1);
    let oy = value(data, 3);
    let oz = value(data, 5);
    let ax = value(data, 7);
    let ay = value(data, 9);
    let mut dx = i64::from(data[11] % 9) - 4;
    let dy = i64::from(data[12] % 9) - 4;
    if dx == 0 && dy == 0 {
        dx = 1;
    }
    let t = i64::from(data[13] % 7) - 3;
    let z0 = value(data, 14);
    let z1 = value(data, 16);
    let z2 = value(data, 18);
    let z3 = value(data, 20);
    let qz0 = value(data, 22);
    let qz1 = value(data, 24);
    let qz2 = value(data, 26);
    let tangent_scale = i64::from(data[28] % 5) + 1;
    let point_offset = i64::from(data[29] % 7) + 1;

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let p0 = sketch.add_point3d(
        "p0",
        Real::from(ox + ax),
        Real::from(oy + ay),
        Real::from(oz + z0),
    );
    let p1 = sketch.add_point3d(
        "p1",
        Real::from(ox + ax + dx),
        Real::from(oy + ay + dy),
        Real::from(oz + z1),
    );
    let p2 = sketch.add_point3d(
        "p2",
        Real::from(ox + ax + 2 * dx),
        Real::from(oy + ay + 2 * dy),
        Real::from(oz + z2),
    );
    let p3 = sketch.add_point3d(
        "p3",
        Real::from(ox + ax + 3 * dx),
        Real::from(oy + ay + 3 * dy),
        Real::from(oz + z3),
    );
    let first = sketch.add_cubic3("first", p0, p1, p2, p3);
    let junction_x = ox + ax + 3 * dx * t;
    let junction_y = oy + ay + 3 * dy * t;
    let second_x = if mode == 1 {
        junction_x + point_offset
    } else {
        junction_x
    };
    let tangent_sign = if mode == 2 { -1 } else { 1 };
    let q0 = sketch.add_point3d(
        "q0",
        Real::from(second_x),
        Real::from(junction_y),
        Real::from(oz + qz0),
    );
    let q1 = sketch.add_point3d(
        "q1",
        Real::from(second_x + tangent_sign * dx * tangent_scale),
        Real::from(junction_y + tangent_sign * dy * tangent_scale),
        Real::from(oz + qz1),
    );
    let q2 = sketch.add_point3d(
        "q2",
        Real::from(second_x + 2 * tangent_sign * dx * tangent_scale),
        Real::from(junction_y + 2 * tangent_sign * dy * tangent_scale),
        Real::from(oz + qz2),
    );
    let q3 = sketch.add_point3d(
        "q3",
        Real::from(second_x + 3 * tangent_sign * dx * tangent_scale),
        Real::from(junction_y + 3 * tangent_sign * dy * tangent_scale),
        Real::from(oz + qz0 - qz1 + qz2),
    );
    let second = sketch.add_cubic3("second", q0, q1, q2, q3);
    let first_parameter = sketch.add_parameter("first t", Real::from(t));
    let second_parameter = sketch.add_parameter("second t", Real::zero());

    if mode == 3 {
        sketch.add_projected_cubic_curve_cubic_curve_tangent3(
            "wrong second cubic kind",
            workplane,
            first,
            first_parameter,
            q0,
            second_parameter,
        );
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        return;
    }

    let handle = sketch.add_projected_cubic_curve_cubic_curve_tangent3(
        "projected cubic curve/cubic curve tangent",
        workplane,
        first,
        first_parameter,
        second,
        second_parameter,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 5);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ProjectedCubicCurveCubicCurveTangent)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    let context = context_from_problem(&lowered.problem);
    let certification = certify_candidate(&PreparedProblem::new(&lowered.problem), &context);
    if mode == 0 {
        assert!(certification.all_satisfied());
    } else {
        assert!(!certification.all_satisfied());
    }

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 5);
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ProjectedCubicCurveCubicCurveTangentDotProductPredicate
    );
});
