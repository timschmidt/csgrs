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
    let tangent_step = i64::from(data[6] % 8) + 1;
    let wrong_shift = i64::from(data[7] % 7) + 1;

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
    let direction = if mode == 2 { -tangent_step } else { tangent_step };
    let p1 = if mode == 1 {
        sketch.add_point2d(
            "radial p1",
            Real::from(cx + radius + tangent_step),
            Real::from(cy),
        )
    } else {
        sketch.add_point2d("p1", Real::from(cx + radius), Real::from(cy + direction))
    };
    let p2 = if mode == 1 {
        sketch.add_point2d(
            "radial p2",
            Real::from(cx + radius + 2 * tangent_step),
            Real::from(cy),
        )
    } else {
        sketch.add_point2d(
            "p2",
            Real::from(cx + radius),
            Real::from(cy + 2 * direction),
        )
    };
    let p3 = if mode == 1 {
        sketch.add_point2d(
            "radial p3",
            Real::from(cx + radius + 3 * tangent_step),
            Real::from(cy),
        )
    } else {
        sketch.add_point2d(
            "p3",
            Real::from(cx + radius),
            Real::from(cy + 3 * direction),
        )
    };
    let cubic = sketch.add_cubic2("cubic", p0, p1, p2, p3);
    let parameter = sketch.add_parameter("t", Real::from(0));
    let point2 = sketch.add_point2d("point2", Real::from(cx), Real::from(cy));

    if mode == 5 {
        sketch.add_arc_cubic_tangent2(
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

    sketch.add_arc_cubic_tangent2(
        "arc cubic tangent",
        arc,
        SketchArcEndpoint::Start,
        cubic,
        parameter,
        SketchTangentOrientation::CounterClockwise,
    );
    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 5);
    assert!(lowered.rows.iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::ArcCubicTangent)
    }));
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    match mode {
        0 | 6 => assert!(certification.all_satisfied()),
        1 | 2 | 3 | 4 => assert!(!certification.all_satisfied()),
        _ => unreachable!(),
    }
    if mode == 2 {
        assert!(matches!(
            certification.rows[4].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    }
});
