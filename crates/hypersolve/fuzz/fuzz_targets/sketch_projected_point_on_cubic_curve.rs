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
    if data.len() < 24 {
        return;
    }
    let mode = data[0] % 4;
    let ox = value(data, 1);
    let oy = value(data, 3);
    let oz = value(data, 5);
    let ax = value(data, 7);
    let ay = value(data, 9);
    let dx = i64::from(data[11] % 9) - 4;
    let dy = i64::from(data[12] % 9) - 4;
    let t = i64::from(data[13] % 7) - 3;
    let z0 = value(data, 14);
    let z1 = value(data, 16);
    let z2 = value(data, 18);
    let z3 = value(data, 20);
    let point_z = value(data, 22);
    let offset = i64::from(data[23] % 7) + 1;

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
    let cubic = sketch.add_cubic3("cubic", p0, p1, p2, p3);
    let point_x = if mode == 1 {
        ax + 3 * dx * t + offset
    } else {
        ax + 3 * dx * t
    };
    let point_y = if mode == 2 {
        ay + 3 * dy * t + offset
    } else {
        ay + 3 * dy * t
    };
    let point = sketch.add_point3d(
        "point",
        Real::from(ox + point_x),
        Real::from(oy + point_y),
        Real::from(oz + point_z),
    );
    let parameter = sketch.add_parameter("t", Real::from(t));

    if mode == 3 {
        sketch.add_projected_point_on_cubic_curve3(
            "wrong cubic kind",
            workplane,
            point,
            point,
            parameter,
        );
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        return;
    }

    let handle = sketch.add_projected_point_on_cubic_curve3(
        "projected point on cubic curve",
        workplane,
        point,
        cubic,
        parameter,
    );

    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 3);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.strategy == Some(SketchResidualStrategy::ProjectedCubicCurveIncidence)
            && row.status == SketchGeneratedRowStatus::Generated
    }));
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    if mode == 0 {
        assert!(certification.all_satisfied());
    } else {
        assert!(!certification.all_satisfied());
    }

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 3);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::ProjectedCubicCurveIncidencePolynomial
    );
});
