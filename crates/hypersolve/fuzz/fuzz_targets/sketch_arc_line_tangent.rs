#![no_main]

use hyperreal::Real;
use hypersolve::{
    CertifiedCandidateStatus, PreparedProblem, SketchArcEndpoint, SketchGeneratedRowStatus,
    SketchLineEndpoint, SketchResidualStrategy, SketchSolveProblem, SketchTangentOrientation,
    certify_candidate, context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 10 {
        return;
    }
    let mode = data[0] % 8;
    let cx = value(data, 1);
    let cy = value(data, 3);
    let radius = i64::from(data[5] % 16) + 1;
    let tangent = i64::from(data[6] % 16) + 1;
    let wrong_shift = i64::from(data[7] % 7) + 1;

    let mut sketch = SketchSolveProblem::new();
    let center = sketch.add_point2d("center", Real::from(cx), Real::from(cy));
    let start = sketch.add_point2d("start", Real::from(cx + radius), Real::from(cy));
    let end = sketch.add_point2d("end", Real::from(cx), Real::from(cy + radius));
    let radius_entity = sketch.add_distance(
        "radius",
        Real::from(if mode == 5 { radius + wrong_shift } else { radius }),
    );
    let arc = sketch.add_arc_of_circle2("arc", center, start, end, radius_entity);
    let line_start = match mode {
        4 => sketch.add_point2d(
            "line start wrong",
            Real::from(cx + radius + wrong_shift),
            Real::from(cy),
        ),
        6 => sketch.add_point2d(
            "arc end tangent start",
            Real::from(cx - tangent),
            Real::from(cy + radius),
        ),
        7 => sketch.add_point2d(
            "arc end wrong tangent start",
            Real::from(cx + tangent),
            Real::from(cy + radius),
        ),
        _ => start,
    };
    let line_end = match mode {
        1 => sketch.add_point2d("radial end", Real::from(cx + radius + tangent), Real::from(cy)),
        2 => sketch.add_point2d("clockwise end", Real::from(cx + radius), Real::from(cy - tangent)),
        6 | 7 => end,
        _ => sketch.add_point2d(
            "ccw end",
            Real::from(cx + radius),
            Real::from(cy + tangent),
        ),
    };
    let line = sketch.add_line_segment2("line", line_start, line_end);
    let point2 = sketch.add_point2d("point2", Real::from(cx), Real::from(cy));
    let arc_endpoint = if matches!(mode, 6 | 7) {
        SketchArcEndpoint::End
    } else {
        SketchArcEndpoint::Start
    };
    let line_endpoint = if matches!(mode, 6 | 7) {
        SketchLineEndpoint::End
    } else {
        SketchLineEndpoint::Start
    };

    match mode {
        3 => {
            sketch.add_arc_line_tangent2(
                "wrong arc",
                point2,
                SketchArcEndpoint::Start,
                line,
                SketchLineEndpoint::Start,
                SketchTangentOrientation::CounterClockwise,
            );
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 0);
            assert!(matches!(
                lowered.rows[0].status,
                SketchGeneratedRowStatus::WrongEntityKind { .. }
            ));
        }
        _ => {
            sketch.add_arc_line_tangent2(
                "arc line tangent",
                arc,
                arc_endpoint,
                line,
                line_endpoint,
                SketchTangentOrientation::CounterClockwise,
            );
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 5);
            assert!(lowered.rows.iter().all(|row| {
                row.status == SketchGeneratedRowStatus::Generated
                    && row.strategy == Some(SketchResidualStrategy::ArcLineTangent)
            }));
            let certification = certify_candidate(
                &PreparedProblem::new(&lowered.problem),
                &context_from_problem(&lowered.problem),
            );
            match mode {
                0 | 6 => assert!(certification.all_satisfied()),
                1 | 2 | 4 | 5 | 7 => assert!(!certification.all_satisfied()),
                _ => unreachable!(),
            }
            if matches!(mode, 2 | 7) {
                assert!(matches!(
                    certification.rows[4].status,
                    CertifiedCandidateStatus::CertifiedViolation { .. }
                ));
            }
        }
    }
});
