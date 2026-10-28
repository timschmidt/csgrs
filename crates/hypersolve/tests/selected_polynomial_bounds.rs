//! Bounds certify the retained value without creating an independent scalar root.
#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;
use hyperreal::{Real, RealSign};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::{
    SelectedPolynomialEnclosure, SelectedPolynomialError, SelectedPolynomialField,
    SelectedPolynomialValue,
};
use hypersolve::work_budget::{WorkBudget, WorkKind, with_budget};
use hypersolve::{RealInterval, SelectedAlgebraContext};
use support::*;

fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("expected certified completion: {other:?}"),
    }
}
fn root(
    parent: &SelectedPolynomialField,
    value: &SelectedPolynomialValue,
    degree: usize,
) -> SelectedPolynomialValue {
    let mut p = vec![parent.constant(real(0)).unwrap(); degree + 1];
    p[0] = value.scale(&real(-1)).unwrap();
    p[degree] = parent.constant(real(1)).unwrap();
    decided(
        parent
            .select(p, real(1), real(2), &TestContext::strict())
            .unwrap(),
    )
    .generator()
    .unwrap()
}
fn cube_root() -> SelectedPolynomialValue {
    let q = SelectedPolynomialField::from_recursive(
        RecursiveQuadraticField::base(
            Vec::new(),
            constant_tensor(0, real(1)),
            constant_tensor(0, real(1)),
        )
        .unwrap(),
    );
    root(&q, &q.constant(real(2)).unwrap(), 3)
}
fn power(value: &SelectedPolynomialValue, n: usize) -> SelectedPolynomialValue {
    (0..n).fold(value.field().constant(real(1)).unwrap(), |p, _| {
        p.multiply(value).unwrap()
    })
}
fn rational_power(value: &Real, n: usize) -> Real {
    (0..n).fold(real(1), |p, _| p * value)
}
fn sign(value: Real) -> RealSign {
    TestContext::strict().real_sign(&value).unwrap()
}

#[test]
fn dependent_root_bounds_have_independent_rational_certificates() {
    let a = cube_root();
    let b = root(a.field(), &a, 3);
    let value = root(b.field(), &b, 5);
    let initial = decided(value.enclosed(&TestContext::strict()).unwrap());
    let narrowed = decided(initial.refined(9, &TestContext::strict()).unwrap());
    let i = narrowed.interval();
    assert_eq!(
        sign(rational_power(&i.lower, 45) - real(2)),
        RealSign::Negative
    );
    assert_eq!(
        sign(rational_power(&i.upper, 45) - real(2)),
        RealSign::Positive
    );
    assert_ne!(
        sign(&i.upper - &i.lower - fraction(1, 512)),
        RealSign::Positive
    );
    assert!(narrowed.value().field().shares_authority(value.field()));
    assert_eq!(
        power(narrowed.value(), 45)
            .subtract(&value.field().constant(real(2)).unwrap())
            .unwrap()
            .sign(&TestContext::strict())
            .unwrap(),
        Classification::Decided(RealSign::Zero)
    );
    // The input certificate remains reusable after narrowing its clone.
    assert_eq!(
        initial.interval(),
        &RealInterval {
            lower: real(1),
            upper: real(2)
        }
    );
}
#[test]
fn rational_images_negative_values_and_exact_zero_keep_their_meaning() {
    let a = cube_root();
    let one = a.field().constant(real(1)).unwrap();
    let quotient = decided(
        a.divide(&a.subtract(&one).unwrap(), &TestContext::strict())
            .unwrap(),
    );
    let bound = decided(quotient.enclosed(&TestContext::strict()).unwrap());
    let bound = decided(bound.refined(10, &TestContext::strict()).unwrap());
    // y=a/(a-1) satisfies -y^3+6y^2-6y+2=0, decreasing throughout [4,8].
    let p =
        |y: &Real| -rational_power(y, 3) + real(6) * rational_power(y, 2) - real(6) * y + real(2);
    assert_ne!(sign(&bound.interval().lower - real(4)), RealSign::Negative);
    assert_eq!(sign(p(&bound.interval().lower)), RealSign::Positive);
    assert_eq!(sign(p(&bound.interval().upper)), RealSign::Negative);
    let negative = quotient.scale(&real(-1)).unwrap();
    let n = decided(negative.enclosed(&TestContext::strict()).unwrap());
    let n = decided(n.refined(10, &TestContext::strict()).unwrap());
    assert_eq!(n.interval().lower, -bound.interval().upper.clone());
    assert_eq!(n.interval().upper, -bound.interval().lower.clone());
    let zero = power(&a, 3)
        .subtract(&a.field().constant(real(2)).unwrap())
        .unwrap();
    let z = decided(zero.enclosed(&TestContext::strict()).unwrap());
    assert_eq!(
        z.interval(),
        &RealInterval {
            lower: real(0),
            upper: real(0)
        }
    );
}
#[test]
fn authored_bounds_are_checked_including_closed_endpoints() {
    let a = cube_root();
    for (lo, hi) in [(2, 1), (0, 1), (2, 3), (1, 1)] {
        assert!(matches!(
            SelectedPolynomialEnclosure::try_new(
                a.clone(),
                RealInterval {
                    lower: real(lo),
                    upper: real(hi)
                },
                &TestContext::strict()
            ),
            Err(SelectedPolynomialError::InvalidEnclosure)
        ));
    }
    let accepted = decided(
        SelectedPolynomialEnclosure::try_new(
            a.clone(),
            RealInterval {
                lower: real(1),
                upper: real(2),
            },
            &TestContext::strict(),
        )
        .unwrap(),
    );
    assert_eq!(
        accepted.interval(),
        &RealInterval {
            lower: real(1),
            upper: real(2)
        }
    );
    let one = a.field().constant(real(1)).unwrap();
    let endpoint = decided(
        SelectedPolynomialEnclosure::try_new(
            one,
            RealInterval {
                lower: real(1),
                upper: real(1),
            },
            &TestContext::strict(),
        )
        .unwrap(),
    );
    assert_eq!(
        endpoint.interval(),
        &RealInterval {
            lower: real(1),
            upper: real(1)
        }
    );
}
#[test]
fn near_values_are_separated_without_merging_their_fields() {
    let a = cube_root();
    let epsilon = rational_power(&fraction(1, 1_u64 << 40), 4);
    let b = a.add(&a.field().constant(epsilon).unwrap()).unwrap();
    let first = decided(a.enclosed(&TestContext::strict()).unwrap());
    let second = decided(b.enclosed(&TestContext::strict()).unwrap());
    let first = decided(first.refined(164, &TestContext::strict()).unwrap());
    let second = decided(second.refined(164, &TestContext::strict()).unwrap());
    assert_eq!(
        sign(&second.interval().lower - &first.interval().upper),
        RealSign::Positive
    );
}
#[test]
fn bounds_refusal_retries_and_never_consumes_approximation() {
    let value = cube_root();
    let policy = TestContext::approximate();
    let refused = with_budget(WorkBudget::ZERO, || value.enclosed(&policy).unwrap());
    assert_eq!(refused.result.unwrap_err().kind, WorkKind::RootRefinement);
    assert_eq!(refused.usage.refinements, 0);
    let retried = with_budget(
        WorkBudget {
            refinements: 100,
            promotions: 0,
        },
        || value.enclosed(&policy).unwrap(),
    );
    let certificate = decided(retried.result.unwrap());
    assert_eq!(retried.usage.promotions, 0);
    let refused = with_budget(WorkBudget::ZERO, || {
        let result = certificate.refined(1, &policy).unwrap();
        assert!(matches!(
            result,
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        ));
        result
    });
    assert!(refused.result.is_err());
    let refined = with_budget(
        WorkBudget {
            refinements: 6,
            promotions: 0,
        },
        || certificate.refined(6, &policy).unwrap(),
    );
    let narrowed = decided(refined.result.unwrap());
    assert_eq!(refined.usage.refinements, 6);
    assert_ne!(
        sign(&narrowed.interval().upper - &narrowed.interval().lower - fraction(1, 64)),
        RealSign::Positive
    );
    let reused = with_budget(WorkBudget::ZERO, || narrowed.refined(0, &policy).unwrap());
    assert_eq!(
        decided(reused.result.unwrap()).interval(),
        narrowed.interval()
    );
    assert_eq!(policy.approximations(), 0);
}
