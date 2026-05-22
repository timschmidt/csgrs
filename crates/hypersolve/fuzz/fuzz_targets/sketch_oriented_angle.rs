#![no_main]

use hyperreal::Real;
use hypersolve::{
    CertifiedCandidateStatus, PreparedProblem, SketchEntityHandle, SketchGeneratedRowStatus,
    SketchResidualStrategy, SketchSolveProblem, certify_candidate, context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let mode = data[0] % 6;
    let cx = value(data, 1);
    let cy = value(data, 3);
    let x_len = i64::from(data[5] % 16) + 1;
    let y_len = i64::from(data[6] % 16) + 1;
    let scale = i64::from(data[7] % 8) + 1;

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point2d("origin", Real::from(cx), Real::from(cy));
    let x = sketch.add_point2d("x", Real::from(cx + x_len), Real::from(cy));
    let y = sketch.add_point2d("y", Real::from(cx), Real::from(cy + y_len));
    let shifted = sketch.add_point2d("shifted", Real::from(cx + 13), Real::from(cy - 11));
    let shifted_x =
        sketch.add_point2d("shifted x", Real::from(cx + 13 + scale * x_len), Real::from(cy - 11));
    let shifted_y = match mode {
        1 => sketch.add_point2d(
            "shifted y reversed",
            Real::from(cx + 13),
            Real::from(cy - 11 - scale * y_len),
        ),
        2 => sketch.add_point2d(
            "shifted y wrong angle",
            Real::from(cx + 13 + scale * x_len),
            Real::from(cy - 11 + scale * y_len),
        ),
        _ => sketch.add_point2d(
            "shifted y",
            Real::from(cx + 13),
            Real::from(cy - 11 + scale * y_len),
        ),
    };
    let point2 = sketch.add_point2d("point2", Real::from(cx), Real::from(cy));
    let point3 = sketch.add_point3d("point3", Real::from(cx), Real::from(cy), Real::from(1));
    let a = sketch.add_line_segment2("a", origin, x);
    let b = sketch.add_line_segment2("b", origin, y);
    let c = sketch.add_line_segment2("c", shifted, shifted_x);
    let d = sketch.add_line_segment2("d", shifted, shifted_y);
    let non_2d = sketch.add_line_segment2("non 2d", shifted, point3);

    match mode {
        3 => {
            sketch.add_equal_oriented_angle_lines2("wrong family", a, b, c, point2);
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 0);
            assert!(matches!(
                lowered.rows[0].status,
                SketchGeneratedRowStatus::WrongEntityKind { .. }
            ));
        }
        4 => {
            sketch.add_equal_oriented_angle_lines2("non 2d", a, b, c, non_2d);
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 0);
            assert!(matches!(
                lowered.rows[0].status,
                SketchGeneratedRowStatus::WrongEntityKind { .. }
            ));
        }
        5 => {
            sketch.add_equal_oriented_angle_lines2(
                "missing",
                a,
                b,
                c,
                SketchEntityHandle(999_999),
            );
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 0);
            assert_eq!(
                lowered.rows[0].status,
                SketchGeneratedRowStatus::MissingEntity(SketchEntityHandle(999_999))
            );
        }
        _ => {
            sketch.add_equal_oriented_angle_lines2("oriented angle", a, b, c, d);
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 2);
            assert!(lowered.rows.iter().all(|row| {
                row.status == SketchGeneratedRowStatus::Generated
                    && row.strategy == Some(SketchResidualStrategy::OrientedAngleEquality)
            }));
            let certification = certify_candidate(
                &PreparedProblem::new(&lowered.problem),
                &context_from_problem(&lowered.problem),
            );
            match mode {
                0 => assert!(certification.all_satisfied()),
                1 | 2 => assert!(!certification.all_satisfied()),
                _ => unreachable!(),
            }
            if mode == 1 {
                assert!(matches!(
                    certification.rows[1].status,
                    CertifiedCandidateStatus::CertifiedViolation { .. }
                ));
            }
            if mode == 2 {
                assert!(matches!(
                    certification.rows[0].status,
                    CertifiedCandidateStatus::CertifiedViolation { .. }
                ));
            }
        }
    }
});
