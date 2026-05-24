#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, CertifiedCandidateStatus, PreparedProblem,
    SketchArcEndpoint, SketchGeneratedRowStatus, SketchLineEndpoint, SketchResidualStrategy,
    SketchSolveProblem, SketchTangentOrientation,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }
    let mode = data[0] % 7;
    let ox = value(data, 1);
    let oy = value(data, 3);
    let oz = value(data, 5);
    let cx = value(data, 7);
    let cy = value(data, 9);
    let radius = i64::from(data[11] % 16) + 1;
    let tangent = i64::from(data[12] % 16) + 1;
    let wrong_shift = i64::from(data[13] % 7) + 1;
    let z0 = value(data, 14);

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
    let line_start = match mode {
        4 => sketch.add_point3d(
            "wrong projected start",
            Real::from(ox + cx + radius + wrong_shift),
            Real::from(oy + cy),
            Real::from(oz + z0),
        ),
        _ => sketch.add_point3d(
            "line start",
            Real::from(ox + cx + radius),
            Real::from(oy + cy),
            Real::from(oz + z0),
        ),
    };
    let line_end = match mode {
        1 => sketch.add_point3d(
            "radial end",
            Real::from(ox + cx + radius + tangent),
            Real::from(oy + cy),
            Real::from(oz - z0),
        ),
        2 => sketch.add_point3d(
            "clockwise end",
            Real::from(ox + cx + radius),
            Real::from(oy + cy - tangent),
            Real::from(oz - z0),
        ),
        _ => sketch.add_point3d(
            "ccw end",
            Real::from(ox + cx + radius),
            Real::from(oy + cy + tangent),
            Real::from(oz - z0),
        ),
    };
    let line = sketch.add_line_segment3("line", line_start, line_end);
    let point2 = sketch.add_point2d("point2", Real::from(cx), Real::from(cy));

    if mode == 3 {
        sketch.add_projected_arc_line_tangent3(
            "wrong line",
            workplane,
            arc,
            SketchArcEndpoint::Start,
            point2,
            SketchLineEndpoint::Start,
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

    sketch.add_projected_arc_line_tangent3(
        "projected arc line tangent",
        workplane,
        arc,
        SketchArcEndpoint::Start,
        line,
        SketchLineEndpoint::Start,
        SketchTangentOrientation::CounterClockwise,
    );
    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 6);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert!(lowered.rows[1..].iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::ProjectedArcLineTangent)
    }));
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    match mode {
        0 | 6 => assert!(certification.all_satisfied()),
        1 | 2 | 4 | 5 => assert!(!certification.all_satisfied()),
        _ => unreachable!(),
    }
    if mode == 2 {
        assert!(matches!(
            certification.rows[5].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    }
});
