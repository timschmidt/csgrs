#![no_main]

use hyperreal::Real;
use hypersolve::{
    CertifiedCandidateStatus, PreparedProblem, SketchArcEndpoint, SketchArcTangencyBranch,
    SketchGeneratedRowStatus, SketchResidualStrategy, SketchSolveProblem, certify_candidate,
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
    let first_center = sketch.add_point2d("first center", Real::from(cx), Real::from(cy));
    let shared = sketch.add_point2d("shared", Real::from(cx + radius), Real::from(cy));
    let first_end = sketch.add_point2d("first end", Real::from(cx), Real::from(cy + radius));
    let first_radius = sketch.add_distance("first radius", Real::from(radius));
    let first = sketch.add_arc_of_circle2(
        "first arc",
        first_center,
        shared,
        first_end,
        first_radius,
    );
    let second_center = match mode {
        2 => sketch.add_point2d(
            "skew center",
            Real::from(cx + radius),
            Real::from(cy + radius),
        ),
        _ => sketch.add_point2d("second center", Real::from(cx + 2 * radius), Real::from(cy)),
    };
    let second_start = if mode == 3 {
        sketch.add_point2d(
            "wrong shared",
            Real::from(cx + radius + wrong_shift),
            Real::from(cy),
        )
    } else {
        shared
    };
    let second_end = sketch.add_point2d(
        "second end",
        Real::from(cx + 2 * radius),
        Real::from(cy + radius),
    );
    let second_radius = sketch.add_distance(
        "second radius",
        Real::from(if mode == 4 {
            radius + wrong_shift
        } else {
            radius
        }),
    );
    let second = sketch.add_arc_of_circle2(
        "second arc",
        second_center,
        second_start,
        second_end,
        second_radius,
    );
    let point2 = sketch.add_point2d("point2", Real::from(cx), Real::from(cy));

    if mode == 5 {
        sketch.add_arc_arc_tangent2(
            "wrong first arc",
            point2,
            SketchArcEndpoint::Start,
            second,
            SketchArcEndpoint::Start,
            SketchArcTangencyBranch::OppositeRadiusDirection,
        );
        let lowered = sketch.lower_to_problem();
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        return;
    }

    let branch = if mode == 1 {
        SketchArcTangencyBranch::SameRadiusDirection
    } else {
        SketchArcTangencyBranch::OppositeRadiusDirection
    };
    sketch.add_arc_arc_tangent2(
        "arc arc tangent",
        first,
        SketchArcEndpoint::Start,
        second,
        SketchArcEndpoint::Start,
        branch,
    );
    let lowered = sketch.lower_to_problem();
    assert_eq!(lowered.problem.constraints.len(), 6);
    assert!(lowered.rows.iter().all(|row| {
        row.status == SketchGeneratedRowStatus::Generated
            && row.strategy == Some(SketchResidualStrategy::ArcArcTangent)
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
    if mode == 1 {
        assert!(matches!(
            certification.rows[5].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    }
});
