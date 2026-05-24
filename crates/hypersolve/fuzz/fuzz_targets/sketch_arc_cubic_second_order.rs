#![no_main]

use hyperreal::Real;
use hypersolve::{
    CertifiedCandidateStatus, PreparedProblem, SketchArcEndpoint, SketchGeneratedRowStatus,
    SketchResidualStrategy, SketchSolveProblem, SketchTangentOrientation, certify_candidate,
    context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let mode = data[0] % 7;
    let cx = value(data, 1);
    let cy = value(data, 3);
    let radius = i64::from(data[5] % 16) + 1;
    let wrong_shift = i64::from(data[6] % 7) + 1;

    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
    let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
    let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
    let radius_entity = sketch.add_distance(
        "radius",
        Real::from(if mode == 4 {
            radius + wrong_shift
        } else {
            radius
        }),
    );
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
    let p0 = if mode == 3 {
        sketch.add_point2d(
            "wrong p0",
            Real::from(cx + radius + wrong_shift),
            Real::from(cy),
        )
    } else {
        start
    };
    let p1 = if mode == 1 {
        sketch.add_point2d("radial p1", Real::from(cx + 3 * radius), Real::from(cy))
    } else if mode == 2 {
        sketch.add_point2d("clockwise p1", Real::from(cx + radius), Real::from(cy - 2 * radius))
    } else {
        sketch.add_point2d("p1", Real::from(cx + radius), Real::from(cy + 2 * radius))
    };
    let p2 = if mode == 5 {
        sketch.add_point2d("tangent only p2", Real::from(cx + radius), Real::from(cy + 4 * radius))
    } else {
        sketch.add_point2d("p2", Real::from(cx - 5 * radius), Real::from(cy + 4 * radius))
    };
    let p3 = if mode == 5 {
        sketch.add_point2d("tangent only p3", Real::from(cx + radius), Real::from(cy + 6 * radius))
    } else {
        sketch.add_point2d("p3", Real::from(cx - 11 * radius), Real::from(cy + 6 * radius))
    };
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let parameter = sketch.add_parameter("t", Real::from(0));
    let point2 = sketch.add_point2d("point2", Real::from(cx), Real::from(cy));

    if mode == 6 {
        sketch.add_arc_cubic_second_order_contact2(
            "wrong arc",
            point2,
            SketchArcEndpoint::Start,
            cubic,
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

    sketch.add_arc_cubic_second_order_contact2(
        "arc cubic second order",
        arc,
        SketchArcEndpoint::Start,
        cubic,
        parameter,
        SketchTangentOrientation::CounterClockwise,
    );
    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 6);
    assert!(lowered.rows.iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::ArcCubicSecondOrderContact)
    }));
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    match mode {
        0 => assert!(certification.all_satisfied()),
        1 | 2 | 3 | 4 | 5 => assert!(!certification.all_satisfied()),
        _ => unreachable!(),
    }
    if mode == 5 {
        assert!(matches!(
            certification.rows[5].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    }
});
