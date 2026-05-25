#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, CertifiedCandidateStatus, PreparedProblem,
    SketchArcEndpoint, SketchGeneratedRowStatus, SketchResidualFormKind, SketchResidualFormsStatus,
    SketchResidualStrategy, SketchSolveProblem, SketchTangentOrientation,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 24 {
        return;
    }
    let mode = data[0] % 8;
    let ox = value(data, 1);
    let oy = value(data, 3);
    let oz = value(data, 5);
    let cx = value(data, 7);
    let cy = value(data, 9);
    let radius = i64::from(data[11] % 8) + 1;
    let wrong_shift = i64::from(data[12] % 7) + 1;
    let z0 = value(data, 13);
    let z1 = value(data, 15);
    let z2 = value(data, 17);
    let z3 = value(data, 19);
    let tangent_step = 10 * radius;
    let tangent_sign = if mode == 2 { -1 } else { 1 };

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(ox), Real::from(oy), Real::from(oz));
    let normal = sketch.add_normal3d(
        "normal",
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
    let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
    let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
    let radius_entity = sketch.add_distance(
        "radius",
        Real::from(if mode == 5 {
            radius + wrong_shift
        } else {
            radius
        }),
    );
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
    let p0_x = if mode == 4 {
        ox + cx + radius + wrong_shift
    } else {
        ox + cx + radius
    };
    let p2_x = if mode == 6 {
        ox + cx - 149 * radius + wrong_shift
    } else {
        ox + cx - 149 * radius
    };
    let p0 = sketch.add_point3d(
        "p0",
        Real::from(p0_x),
        Real::from(oy + cy),
        Real::from(oz + z0),
    );
    let p1 = sketch.add_point3d(
        "p1",
        Real::from(p0_x),
        Real::from(oy + cy + tangent_sign * tangent_step),
        Real::from(oz + z1),
    );
    let p2 = sketch.add_point3d(
        "p2",
        Real::from(p2_x),
        Real::from(oy + cy + 2 * tangent_sign * tangent_step),
        Real::from(oz + z2),
    );
    let p3 = sketch.add_point3d(
        "p3",
        Real::from(ox + cx - 299 * radius),
        Real::from(oy + cy + 3 * tangent_sign * tangent_step),
        Real::from(oz + z3),
    );
    let cubic = sketch.add_cubic3("cubic", p0, p1, p2, p3);
    let point2 = sketch.add_point2d("point2", Real::from(cx), Real::from(cy));
    let parameter = sketch.add_parameter("t", Real::zero());

    if mode == 3 {
        sketch.add_projected_arc_cubic_curve_second_order_contact3(
            "wrong cubic",
            workplane,
            arc,
            SketchArcEndpoint::Start,
            point2,
            parameter,
            SketchTangentOrientation::CounterClockwise,
        );
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        return;
    }

    let handle = sketch.add_projected_arc_cubic_curve_second_order_contact3(
        "projected arc cubic second order contact",
        workplane,
        arc,
        SketchArcEndpoint::Start,
        cubic,
        parameter,
        SketchTangentOrientation::CounterClockwise,
    );
    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 7);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy
                == Some(SketchResidualStrategy::ProjectedArcCubicCurveSecondOrderContact)
    }));
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    match mode {
        0 | 1 | 7 => assert!(certification.all_satisfied()),
        2 | 4 | 5 | 6 => assert!(!certification.all_satisfied()),
        _ => unreachable!(),
    }
    if mode == 2 {
        assert!(matches!(
            certification.rows[5].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    }
    if mode == 6 {
        assert!(matches!(
            certification.rows[6].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    }

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 7);
    assert_eq!(
        forms.forms[5].kind,
        SketchResidualFormKind::ProjectedArcCubicCurveSecondOrderOrientationPredicate
    );
    assert_eq!(
        forms.forms[6].kind,
        SketchResidualFormKind::ProjectedArcCubicCurveSecondOrderContactPolynomial
    );
});
