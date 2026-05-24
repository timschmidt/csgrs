#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, PreparedProblem, SketchGeneratedRowStatus,
    SketchLineEndpoint, SketchResidualFormKind, SketchResidualFormsStatus, SketchResidualStrategy,
    SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 22 {
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
    let tangent_scale = i64::from(data[18] % 5) + 1;
    let endpoint_offset = i64::from(data[19] % 7) + 1;

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
    let p0 = sketch.add_point2d("p0", Real::from(ax), Real::from(ay));
    let p1 = sketch.add_point2d("p1", Real::from(ax + dx), Real::from(ay + dy));
    let p2 = sketch.add_point2d("p2", Real::from(ax + 2 * dx), Real::from(ay + 2 * dy));
    let p3 = sketch.add_point2d("p3", Real::from(ax + 3 * dx), Real::from(ay + 3 * dy));
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let projected_x = ax + 3 * dx * t;
    let projected_y = ay + 3 * dy * t;
    let endpoint_x = if mode == 1 {
        projected_x + endpoint_offset
    } else {
        projected_x
    };
    let point = sketch.add_point3d(
        "point",
        Real::from(ox + endpoint_x),
        Real::from(oy + projected_y),
        Real::from(oz + z0),
    );
    let tangent_sign = if mode == 2 { -1 } else { 1 };
    let tangent_end = sketch.add_point3d(
        "tangent end",
        Real::from(ox + endpoint_x + tangent_sign * dx * tangent_scale),
        Real::from(oy + projected_y + tangent_sign * dy * tangent_scale),
        Real::from(oz + z0 + z1),
    );
    let line = sketch.add_line_segment3("line", point, tangent_end);
    let parameter = sketch.add_parameter("t", Real::from(t));

    if mode == 3 {
        sketch.add_projected_cubic_line_tangent3(
            "wrong line kind",
            workplane,
            cubic,
            parameter,
            p3,
            SketchLineEndpoint::Start,
        );
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        return;
    }

    let handle = sketch.add_projected_cubic_line_tangent3(
        "projected cubic tangent",
        workplane,
        cubic,
        parameter,
        line,
        SketchLineEndpoint::Start,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 5);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ProjectedCubicLineTangent)
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
        forms.forms[0].kind,
        SketchResidualFormKind::WorkplaneUnitQuaternionPolynomial
    );
    assert_eq!(
        forms.forms[4].kind,
        SketchResidualFormKind::ProjectedCubicLineTangentDotProductPredicate
    );
});
