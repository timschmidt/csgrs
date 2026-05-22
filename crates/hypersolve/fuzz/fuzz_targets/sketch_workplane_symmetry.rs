#![no_main]

use hyperreal::Real;
use hypersolve::{
    CertifiedCandidateStatus, PreparedProblem, SketchGeneratedRowStatus, SketchResidualStrategy,
    SketchSolveProblem, certify_candidate, context_from_problem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 11 {
        return;
    }
    let mode = data[0] % 5;
    let x = value(data, 1);
    let y = value(data, 3);
    let z0 = value(data, 5);
    let offset = value(data, 7);
    let wrong_shift = i64::from(data[9] % 7) + 1;

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(z0));
    let normal = if mode == 1 {
        sketch.add_normal3d(
            "nonunit",
            Real::from(2),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        )
    } else {
        sketch.add_normal3d(
            "normal",
            Real::from(1),
            Real::from(0),
            Real::from(0),
            Real::from(0),
        )
    };
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a = sketch.add_point3d(
        "a",
        Real::from(x),
        Real::from(y),
        Real::from(z0 + offset),
    );
    let b_z = if mode == 4 {
        z0 - offset + wrong_shift
    } else {
        z0 - offset
    };
    let b_x = if mode == 4 { x + wrong_shift } else { x };
    let b = sketch.add_point3d("b", Real::from(b_x), Real::from(y), Real::from(b_z));
    let point2 = sketch.add_point2d("point2", Real::from(x), Real::from(y));

    match mode {
        2 => {
            sketch.add_symmetric_workplane3("wrong workplane", a, b, point2);
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 0);
            assert!(matches!(
                lowered.rows[0].status,
                SketchGeneratedRowStatus::WrongEntityKind { .. }
            ));
        }
        3 => {
            sketch.add_symmetric_workplane3("wrong point", point2, b, workplane);
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 0);
            assert!(matches!(
                lowered.rows[0].status,
                SketchGeneratedRowStatus::WrongEntityKind { .. }
            ));
        }
        _ => {
            sketch.add_symmetric_workplane3("workplane symmetry", a, b, workplane);
            let lowered = sketch.lower_to_problem();
            assert_eq!(lowered.problem.constraints.len(), 5);
            assert_eq!(
                lowered.rows[0].strategy,
                Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
            );
            assert!(lowered.rows[1..].iter().all(|row| {
                row.status == SketchGeneratedRowStatus::Generated
                    && row.strategy == Some(SketchResidualStrategy::WorkplaneSymmetryPolynomial)
            }));
            let certification = certify_candidate(
                &PreparedProblem::new(&lowered.problem),
                &context_from_problem(&lowered.problem),
            );
            match mode {
                0 => assert!(certification.all_satisfied()),
                1 => assert!(matches!(
                    certification.rows[0].status,
                    CertifiedCandidateStatus::CertifiedViolation { .. }
                )),
                4 => assert!(!certification.all_satisfied()),
                _ => unreachable!(),
            }
        }
    }
});
