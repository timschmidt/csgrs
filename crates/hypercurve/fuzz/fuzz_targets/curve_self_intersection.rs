#![no_main]

mod support;

use hypercurve::{
    Classification, CubicBezier2, Curve2, CurveContext, CurveIntersectionResult2, CurveLocation2,
    NurbsCurve2, Point2, Real,
};
use libfuzzer_sys::fuzz_target;

fn coordinate(byte: u8) -> Real {
    Real::from(i32::from(byte % 16) - 8)
}

/// Isolated contact sets only; retracing and constant-image components are
/// not additive under splitting.
fn complete(result: CurveIntersectionResult2) -> Option<CurveIntersectionResult2> {
    (result.is_complete()
        && result.overlaps().is_empty()
        && result.parameter_components().is_empty())
    .then_some(result)
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
        let first = support::under(policy, || {
            curve.point_at(&parameter(contact.first(), policy))
        })
        .unwrap();
        let second = support::under(policy, || {
            curve.point_at(&parameter(contact.second(), policy))
        })
        .unwrap();
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
    let Some(whole) =
        support::certified_under(&policy, || curve.self_intersections()).and_then(complete)
    else {
        return;
    };
    assert_contacts_coincide(&curve, &whole, &policy);

    // Refining at an interior knot changes only the span structure: joints
    // are the identity and every off-diagonal contact survives.
    let zero = Real::zero;
    let one = Real::one;
    let refined = support::under(&policy, || {
        NurbsCurve2::try_new(
            3,
            controls,
            vec![one(); 4],
            vec![zero(), zero(), zero(), zero(), one(), one(), one(), one()],
        )
    })
    .and_then(|nurbs| support::under(&policy, || nurbs.insert_knot(split.clone())))
    .map(Curve2::from);
    if let Ok(refined) = refined
        && let Some(result) =
            support::certified_under(&policy, || refined.self_intersections()).and_then(complete)
    {
        assert_eq!(result.contacts().len(), whole.contacts().len());
        assert_contacts_coincide(&refined, &result, &policy);
    }

    // The same off-diagonal contacts split between both pieces' own
    // incidence and their pair, whose shared joint is not a contact.
    let Ok(pieces) = support::under(&policy, || curve.split_at(split.into())) else {
        return;
    };
    let (left, right) = pieces;
    let (Some(left_self), Some(right_self), Some(pair)) = (
        support::certified_under(&policy, || left.self_intersections()).and_then(complete),
        support::certified_under(&policy, || right.self_intersections()).and_then(complete),
        support::certified_under(&policy, || left.intersect_curve(&right)).and_then(complete),
    ) else {
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
