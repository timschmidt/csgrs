#![no_main]

use hyperreal::Real;
use hypersolve::{
    diagnose_sketch_failed_constraints, FailedConstraintStatus, SketchFailedConstraintStatus,
    SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: [i16; 5]| {
    let ax = i64::from(data[0]);
    let ay = i64::from(data[1]);
    let dx = i64::from(data[2]);
    let dy = i64::from(data[3]);
    let distance = i64::from(data[4]).abs();
    let squared = dx * dx + dy * dy;

    let mut sketch = SketchSolveProblem::new();
    let a = sketch.add_point2d("a", Real::from(ax), Real::from(ay));
    let b = sketch.add_point2d("b", Real::from(ax + dx), Real::from(ay + dy));
    let distance_entity = sketch.add_distance("distance", Real::from(distance));
    let handle = sketch.add_point_point_distance("distance row", a, b, distance_entity);

    let report = diagnose_sketch_failed_constraints(&sketch);
    if squared == distance * distance {
        assert_eq!(report.blocking_rows, 0);
        assert_eq!(report.exact_failure_rows, 0);
    } else {
        assert_eq!(report.blocking_rows, 1);
        assert_eq!(report.exact_failure_rows, 1);
        assert_eq!(report.rows.len(), 1);
        assert_eq!(report.rows[0].constraint, handle);
        assert!(matches!(
            report.rows[0].status,
            SketchFailedConstraintStatus::ExactFailure(
                FailedConstraintStatus::CertifiedCandidateViolation
            )
        ));
    }
});
