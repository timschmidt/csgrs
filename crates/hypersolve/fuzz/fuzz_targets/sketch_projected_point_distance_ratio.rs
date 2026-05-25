#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, CertifiedCandidateStatus, PreparedProblem,
    SketchGeneratedRowStatus, SketchResidualFormKind, SketchResidualFormsStatus,
    SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 19 {
        return;
    }
    let value = |index: usize| i64::from(i16::from_le_bytes([data[index], data[index + 1]]));
    let x0 = value(0);
    let y0 = value(2);
    let z0 = value(4);
    let z1 = value(6);
    let z2 = value(8);
    let z3 = value(10);
    let base = i64::from(data[12] % 8) + 1;
    let numerator = i64::from(data[13] % 8);
    let denominator = i64::from(data[14] % 8) + 1;
    let bad_delta = i64::from(data[15] % 3) + 1;
    let mode = data[16] % 4;
    let y_gap = i64::from(data[17] % 7) + 1;
    let wrong_offset = i64::from(data[18] % 5) + 1;

    let first_len = base * numerator + if mode == 2 { bad_delta } else { 0 };
    let second_len = base * denominator;
    let ratio_numerator = if mode == 1 { -1 } else { numerator };
    let ratio_denominator = if mode == 1 { 0 } else { denominator };

    let mut sketch = SketchSolveProblem::new();
    let origin = sketch.add_point3d("origin", Real::from(0), Real::from(0), Real::from(0));
    let normal = sketch.add_normal3d(
        "normal",
        Real::from(1),
        Real::from(0),
        Real::from(0),
        Real::from(0),
    );
    let workplane = sketch.add_workplane("workplane", origin, normal);
    let a = sketch.add_point3d("a", Real::from(x0), Real::from(y0), Real::from(z0));
    let b = sketch.add_point3d(
        "b",
        Real::from(x0 + first_len),
        Real::from(y0),
        Real::from(z1),
    );
    let c = sketch.add_point3d("c", Real::from(x0), Real::from(y0 + y_gap), Real::from(z2));
    let d = sketch.add_point3d(
        "d",
        Real::from(x0 + second_len),
        Real::from(y0 + y_gap),
        Real::from(z3),
    );
    let wrong_endpoint = sketch.add_point3d(
        "wrong-endpoint",
        Real::from(x0 + wrong_offset),
        Real::from(y0),
        Real::from(z0),
    );
    let wrong_line = sketch.add_line_segment3("wrong-line", a, wrong_endpoint);
    let first = if mode == 3 { (a, wrong_line) } else { (a, b) };
    let handle = sketch.add_projected_point_distance_ratio3(
        "projected point distance ratio",
        workplane,
        first,
        (c, d),
        Real::from(ratio_numerator),
        Real::from(ratio_denominator),
    );

    let lowered = sketch.lower_to_problem();
    if mode == 1 {
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert_eq!(lowered.rows.len(), 1);
        assert_eq!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::InvalidExactBound
        );
        assert_eq!(
            lowered.rows[0].strategy,
            Some(SketchResidualStrategy::SquaredProjectedPointDistanceRatio)
        );
        let forms = sketch.residual_forms_for_constraint(handle);
        assert_eq!(forms.status, SketchResidualFormsStatus::InvalidInputs);
        return;
    }
    if mode == 3 {
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert_eq!(lowered.rows.len(), 1);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        let forms = sketch.residual_forms_for_constraint(handle);
        assert_eq!(forms.status, SketchResidualFormsStatus::InvalidInputs);
        return;
    }

    assert!(lowered.all_generated());
    assert_eq!(lowered.problem.constraints.len(), 2);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::WorkplaneUnitQuaternion)
    );
    assert_eq!(
        lowered.rows[1].strategy,
        Some(SketchResidualStrategy::SquaredProjectedPointDistanceRatio)
    );

    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    assert!(matches!(
        certification.rows[0].status,
        CertifiedCandidateStatus::CertifiedZero { .. }
    ));
    if mode == 2 && numerator != 0 {
        assert!(matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    } else {
        assert!(matches!(
            certification.rows[1].status,
            CertifiedCandidateStatus::CertifiedZero { .. }
        ));
    }

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 2);
    assert_eq!(
        forms.forms[1].kind,
        SketchResidualFormKind::SquaredProjectedPointDistanceRatioPolynomial
    );
});
