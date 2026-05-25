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
    if data.len() < 12 {
        return;
    }
    let mode = data[0] % 5;
    let ox = value(data, 1);
    let oy = value(data, 3);
    let oz = value(data, 5);
    let cx = i64::from(i8::from_le_bytes([data[7]]));
    let cy = i64::from(i8::from_le_bytes([data[8]]));
    let radius = i64::from(data[9] % 16) + 1;
    let dx = if mode == 2 {
        i64::from(data[10] % 7) + 1
    } else {
        0
    };
    let dy = if mode == 3 {
        i64::from(data[11] % 7) + 1
    } else {
        0
    };

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
    let center = sketch.add_point2d("center", Real::from(cx + dx), Real::from(cy + dy));
    let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
    let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
    let radius_entity = sketch.add_distance("radius", Real::from(radius));
    let circle = sketch.add_circle2("circle", center, radius_entity);
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
    let point = sketch.add_point3d(
        "point",
        Real::from(ox + cx),
        Real::from(oy + cy),
        Real::from(oz + i64::from(data[6])),
    );
    let curve = if mode == 1 { arc } else { circle };
    let handle = if mode == 4 {
        sketch.add_projected_point_concentric3(
            "wrong projected point concentric",
            workplane,
            point,
            radius_entity,
        )
    } else {
        sketch.add_projected_point_concentric3(
            "projected point concentric",
            workplane,
            point,
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

    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(lowered.rows.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::ProjectedConcentricity)
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
    assert_eq!(forms.forms.len(), 3);
    assert!(forms.forms[1..].iter().all(|form| {
        form.kind == SketchResidualFormKind::ProjectedConcentricCenterCoordinatePolynomial
    }));
});
