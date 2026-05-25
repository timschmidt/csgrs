#![no_main]

use hyperreal::Real;
use hypersolve::{
    certify_candidate, context_from_problem, CertifiedCandidateStatus, PreparedProblem,
    SketchCircleTangencyBranch, SketchGeneratedRowStatus, SketchResidualFormKind,
    SketchResidualFormsStatus, SketchResidualStrategy, SketchSolveProblem,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }
    let mode = data[0] % 6;
    let cx = value(data, 1);
    let cy = value(data, 3);
    let first_radius = i64::from(data[5] % 16) + 1;
    let second_radius = i64::from(data[6] % 16) + 1;
    let perturb = i64::from(data[7] % 7) + 1;
    let wrong_kind = mode == 4;
    let external = matches!(mode, 0 | 2 | 4 | 5);
    let bad = matches!(mode, 2 | 3 | 5);

    let mut sketch = SketchSolveProblem::new();
    let first_center = sketch.add_point2d("first center", Real::from(cx), Real::from(cy));
    let signed_offset = if external {
        first_radius + second_radius
    } else {
        first_radius - second_radius
    };
    let second_center = sketch.add_point2d(
        "second center",
        Real::from(cx + signed_offset),
        Real::from(cy + if bad { perturb } else { 0 }),
    );
    let first_radius_entity = sketch.add_distance("first radius", Real::from(first_radius));
    let second_radius_entity = sketch.add_distance("second radius", Real::from(second_radius));
    let first = sketch.add_circle2("first", first_center, first_radius_entity);
    let second = sketch.add_circle2("second", second_center, second_radius_entity);
    let branch = if external {
        SketchCircleTangencyBranch::External
    } else {
        SketchCircleTangencyBranch::Internal
    };

    let handle = if wrong_kind {
        sketch.add_circle_circle_tangent2("wrong circle", first, second_radius_entity, branch)
    } else {
        sketch.add_circle_circle_tangent2("circle tangent", first, second, branch)
    };
    let lowered = sketch.lower_to_problem();
    if wrong_kind {
        assert_eq!(lowered.problem.constraints.len(), 0);
        assert!(matches!(
            lowered.rows[0].status,
            SketchGeneratedRowStatus::WrongEntityKind { .. }
        ));
        let forms = sketch.residual_forms_for_constraint(handle);
        assert_eq!(forms.status, SketchResidualFormsStatus::InvalidInputs);
        return;
    }

    assert_eq!(lowered.problem.constraints.len(), 1);
    assert_eq!(lowered.rows.len(), 1);
    assert_eq!(
        lowered.rows[0].strategy,
        Some(SketchResidualStrategy::CircleCircleTangency)
    );
    assert_eq!(lowered.rows[0].status, SketchGeneratedRowStatus::Generated);
    let certification = certify_candidate(
        &PreparedProblem::new(&lowered.problem),
        &context_from_problem(&lowered.problem),
    );
    if bad {
        assert!(matches!(
            certification.rows[0].status,
            CertifiedCandidateStatus::CertifiedViolation { .. }
        ));
    } else {
        assert!(certification.all_satisfied());
    }

    let forms = sketch.residual_forms_for_constraint(handle);
    assert_eq!(forms.status, SketchResidualFormsStatus::Generated);
    assert_eq!(forms.forms.len(), 1);
    assert_eq!(
        forms.forms[0].kind,
        SketchResidualFormKind::CircleCircleTangencyPolynomial
    );
});
