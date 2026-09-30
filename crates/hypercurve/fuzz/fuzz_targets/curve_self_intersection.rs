#![no_main]

use hypercurve::{
    Classification, CubicBezier2, Curve2, CurveCertainty, CurveContext, CurveIntersectionResult2,
    CurveLocation2, NurbsCurve2, Point2, Real,
};
use libfuzzer_sys::fuzz_target;

fn coordinate(byte: u8) -> Real {
    Real::from(i32::from(byte % 16) - 8)
}

/// Isolated contact sets only; retracing and constant-image components are
/// not additive under splitting.
fn complete(
    outcome: hypercurve::CurveOutcome<CurveIntersectionResult2>,
) -> Option<CurveIntersectionResult2> {
    (outcome.certainty == CurveCertainty::Certified
        && outcome.value.is_complete()
        && outcome.value.overlaps().is_empty()
        && outcome.value.parameter_components().is_empty())
    .then_some(outcome.value)
}

fn parameter(location: &CurveLocation2, policy: &CurveContext) -> hypercurve::CurveParameter2 {
    match location
        .parameter(policy)
        .expect("contact parameters are exact")
    {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(reason) => {
            panic!("contact parameter became uncertain: {reason:?}")
        }
    }
}

/// Both sides of every published contact must evaluate to one exact point.
fn assert_contacts_coincide(
    curve: &Curve2,
    result: &CurveIntersectionResult2,
    policy: &CurveContext,
) {
    for contact in result.contacts() {
        let first = curve
            .point_at(&parameter(contact.first(), policy), policy)
            .unwrap()
            .value;
        let second = curve
            .point_at(&parameter(contact.second(), policy), policy)
            .unwrap()
            .value;
        assert_eq!(
            first.coincides_with(&second, policy).value,
            Classification::Decided(true)
        );
        assert_eq!(
            first.coincides_with(contact.point(), policy).value,
            Classification::Decided(true)
        );
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 9 {
        return;
    }
    let policy = CurveContext::STRICT;
    let controls: Vec<Point2> = data[..8]
        .chunks_exact(2)
        .map(|pair| Point2::new(coordinate(pair[0]), coordinate(pair[1])))
        .collect();
    let split = (Real::from(i32::from(data[8] % 15) + 1) / Real::from(16)).unwrap();
    let curve = Curve2::from(CubicBezier2::new(
        controls[0].clone(),
        controls[1].clone(),
        controls[2].clone(),
        controls[3].clone(),
    ));
    let Ok(outcome) = curve.self_intersections(&policy) else {
        return;
    };
    let Some(whole) = complete(outcome) else {
        return;
    };
    assert_contacts_coincide(&curve, &whole, &policy);

    // Refining at an interior knot changes only the span structure: joints
    // are the identity and every off-diagonal contact survives.
    let zero = Real::zero;
    let one = Real::one;
    let refined = NurbsCurve2::try_new(
        3,
        controls,
        vec![one(); 4],
        vec![zero(), zero(), zero(), zero(), one(), one(), one(), one()],
        &policy,
    )
    .and_then(|nurbs| nurbs.into_value().insert_knot(split.clone(), &policy))
    .map(|nurbs| Curve2::from(nurbs.into_value()));
    if let Ok(refined) = refined
        && let Ok(outcome) = refined.self_intersections(&policy)
        && let Some(result) = complete(outcome)
    {
        assert_eq!(result.contacts().len(), whole.contacts().len());
        assert_contacts_coincide(&refined, &result, &policy);
    }

    // The same off-diagonal contacts split between both pieces' own
    // incidence and their pair, whose shared joint is not a contact.
    let Ok(pieces) = curve.split_at(split.into(), &policy) else {
        return;
    };
    let (left, right) = pieces.into_value();
    let (Ok(left_self), Ok(right_self), Ok(pair)) = (
        left.self_intersections(&policy),
        right.self_intersections(&policy),
        left.intersect_curve(&right, &policy),
    ) else {
        return;
    };
    let (Some(left_self), Some(right_self), Some(pair)) =
        (complete(left_self), complete(right_self), complete(pair))
    else {
        return;
    };
    let joint = pair
        .contacts()
        .iter()
        .filter(|contact| {
            let first = parameter(contact.first(), &policy);
            let second = parameter(contact.second(), &policy);
            first.scalar() == Some(&Real::one()) && second.scalar() == Some(&Real::zero())
        })
        .count();
    assert_eq!(joint, 1, "the pieces share exactly one joint");
    assert_eq!(
        left_self.contacts().len() + right_self.contacts().len() + pair.contacts().len() - joint,
        whole.contacts().len()
    );
});
