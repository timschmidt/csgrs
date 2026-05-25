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
    let mode = data[0] % 5;
    let lx = value(data, 1);
    let ly = value(data, 3);
    let px_offset = value(data, 5);
    let z0 = value(data, 7);
    let z1 = value(data, 9);
    let radius = i64::from(data[11] % 16) + 1;
    let line_dx = i64::from(data[12] % 16) + 1;
    let signed_radius = if data[13] & 1 == 0 { radius } else { -radius };
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
    let point = sketch.add_point3d(
        "point",
        Real::from(lx + px_offset),
        Real::from(ly + signed_radius),
        Real::from(z0),
    );
    let line_start =
        sketch.add_point3d("line start", Real::from(lx), Real::from(ly), Real::from(0));
    let line_end = sketch.add_point3d(
        "line end",
        Real::from(lx + line_dx),
        Real::from(ly),
        Real::from(z1),
    );
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let center = sketch.add_point2d("center", Real::from(0), Real::from(0));
    let start = sketch.add_point2d("start", Real::from(radius), Real::from(0));
    let end = sketch.add_point2d("end", Real::from(0), Real::from(radius));
    let radius_entity = sketch.add_distance("radius", Real::from(radius + bad_delta));
    let circle = sketch.add_circle2("circle", center, radius_entity);
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
    let curve = if mode == 1 { arc } else { circle };
    let handle = if mode == 4 {
        sketch.add_projected_point_line_radius_equality3(
            "wrong projected point-line radius",
            workplane,
            point,
            line,
            radius_entity,
        )
    } else {
        sketch.add_projected_point_line_radius_equality3(
            "projected point-line radius",
            workplane,
            point,
            line,
            curve,
        )
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
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedPointLineRadiusEquality)
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
        SketchResidualFormKind::SquaredProjectedPointLineRadiusEqualityPolynomial
    );
});
