//! A common field preserves each source selection and its dependent values.
#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use hyperreal::{Real, RealSign};
use hypersolve::classification::{Classification, UncertaintyReason};
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::{SelectedPolynomialField, SelectedPolynomialValue};
use hypersolve::work_budget::{WorkBudget, WorkKind, with_budget};
use support::*;

fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("expected exact completion, got {other:?}"),
    }
}
fn base() -> SelectedPolynomialField {
    SelectedPolynomialField::from_recursive(
        RecursiveQuadraticField::base(
            Vec::new(),
            constant_tensor(0, real(1)),
            constant_tensor(0, real(1)),
        )
        .unwrap(),
    )
}
fn coefficients(field: &SelectedPolynomialField, values: &[i32]) -> Vec<SelectedPolynomialValue> {
    values
        .iter()
        .map(|v| field.constant(Real::from(*v)).unwrap())
        .collect()
}
fn select(
    field: &SelectedPolynomialField,
    values: Vec<SelectedPolynomialValue>,
    lo: i32,
    hi: i32,
) -> SelectedPolynomialField {
    decided(
        field
            .select(
                values,
                Real::from(lo),
                Real::from(hi),
                &TestContext::strict(),
            )
            .unwrap(),
    )
}
fn root(
    parent: &SelectedPolynomialField,
    radicand: &SelectedPolynomialValue,
    degree: usize,
) -> SelectedPolynomialField {
    let mut p = vec![parent.constant(real(0)).unwrap(); degree + 1];
    p[0] = radicand.scale(&real(-1)).unwrap();
    p[degree] = parent.constant(real(1)).unwrap();
    select(parent, p, 1, 2)
}
fn power(value: &SelectedPolynomialValue, exponent: usize) -> SelectedPolynomialValue {
    (0..exponent).fold(value.field().constant(real(1)).unwrap(), |result, _| {
        result.multiply(value).unwrap()
    })
}
fn is_zero(value: &SelectedPolynomialValue) {
    assert_eq!(
        value.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Zero)
    );
}

#[test]
fn divergent_generators_embed_quotients_and_feed_a_further_selection() {
    let q = base();
    let a_field = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = a_field.generator().unwrap();
    let b_field = root(&a_field, &a, 3);
    let c_field = root(&a_field, &a, 5);
    let b = b_field.generator().unwrap();
    let c = c_field.generator().unwrap();
    let c_quotient = decided(
        c.divide(&c.add(&a).unwrap(), &TestContext::strict())
            .unwrap(),
    );
    let join = decided(
        b_field
            .joined_with(&c_field, &TestContext::strict())
            .unwrap(),
    );
    let x = join.embed(&b).unwrap();
    let y = join.embed(&c).unwrap();
    let a = join.embed(&a).unwrap();
    is_zero(&power(&x, 3).subtract(&a).unwrap());
    is_zero(&power(&y, 5).subtract(&a).unwrap());
    is_zero(
        &power(&x, 9)
            .subtract(&join.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    is_zero(
        &join
            .embed(&c_quotient)
            .unwrap()
            .multiply(&y.add(&a).unwrap())
            .unwrap()
            .subtract(&y)
            .unwrap(),
    );
    assert_eq!(
        x.subtract(&y)
            .unwrap()
            .sign(&TestContext::strict())
            .unwrap(),
        Classification::Decided(RealSign::Positive)
    );
    // The joined values remain ordinary coefficients of another exact construction.
    let sum = x.add(&y).unwrap();
    let next = root(join.field(), &sum, 2);
    is_zero(
        &power(&next.generator().unwrap(), 2)
            .subtract(&next.lift(&sum).unwrap())
            .unwrap(),
    );
    assert!(join.field().parent().unwrap().shares_authority(&b_field));
}

#[test]
fn independently_selected_equal_values_and_opposite_roots_are_distinguished() {
    let q = base();
    let first = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let second = select(&q, coefficients(&q, &[-12, 0, 0, 12, 0, 0, -3]), 1, 2);
    assert!(!first.shares_authority(&second));
    let join = decided(first.joined_with(&second, &TestContext::strict()).unwrap());
    is_zero(
        &join
            .embed(&first.generator().unwrap())
            .unwrap()
            .subtract(&join.embed(&second.generator().unwrap()).unwrap())
            .unwrap(),
    );
    let positive = select(&q, coefficients(&q, &[-2, 0, 1]), 1, 2);
    let negative = select(&q, coefficients(&q, &[-2, 0, 1]), -2, -1);
    let join = decided(
        positive
            .joined_with(&negative, &TestContext::strict())
            .unwrap(),
    );
    let p = join.embed(&positive.generator().unwrap()).unwrap();
    let n = join.embed(&negative.generator().unwrap()).unwrap();
    is_zero(&p.add(&n).unwrap());
    assert_eq!(
        p.subtract(&n)
            .unwrap()
            .sign(&TestContext::strict())
            .unwrap(),
        Classification::Decided(RealSign::Positive)
    );
}

#[test]
fn foreign_native_roots_and_radicals_keep_their_relations() {
    let first_native = RecursiveQuadraticField::base(
        vec![cube_root_two()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let a_native = first_native.retained_root_value(&cube_root_two()).unwrap();
    let first = SelectedPolynomialField::from_recursive(first_native);
    let a = first.import_recursive(&a_native).unwrap();
    let b = root(&first, &a, 3);

    let second_base = RecursiveQuadraticField::base(
        vec![sqrt_two_root()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(3)),
    )
    .unwrap();
    let s_native = second_base.retained_root_value(&sqrt_two_root()).unwrap();
    let radicand = s_native
        .add(&second_base.constant(real(1)).unwrap())
        .unwrap();
    let second_native = second_base.extension(radicand.clone()).unwrap();
    let r_native = second_native
        .retained_positive_square_root(&radicand)
        .unwrap();
    let second = SelectedPolynomialField::from_recursive(second_native);
    let r = second.import_recursive(&r_native).unwrap();
    let s = second.import_recursive(&s_native).unwrap();
    let c = root(&second, &r, 3);
    let join = decided(b.joined_with(&c, &TestContext::strict()).unwrap());
    let x = join.embed(&b.generator().unwrap()).unwrap();
    let y = join.embed(&c.generator().unwrap()).unwrap();
    let mapped_s = join.embed(&s).unwrap();
    let mapped_r = join.embed(&r).unwrap();
    is_zero(
        &power(&x, 9)
            .subtract(&join.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    is_zero(&power(&y, 3).subtract(&mapped_r).unwrap());
    is_zero(
        &power(&mapped_r, 2)
            .subtract(&mapped_s)
            .unwrap()
            .subtract(&join.field().constant(real(1)).unwrap())
            .unwrap(),
    );
    is_zero(
        &power(&mapped_s, 2)
            .subtract(&join.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    let quotient = decided(x.divide(&y, &TestContext::strict()).unwrap());
    is_zero(&quotient.multiply(&y).unwrap().subtract(&x).unwrap());
}

#[test]
fn joining_preserves_nearby_distinct_selected_roots() {
    let q = base();
    let epsilon = power(&q.constant(fraction(1, 1_u64 << 40)).unwrap(), 4);
    let one = q.constant(real(1)).unwrap();
    let next = one.add(&epsilon).unwrap();
    let equation = vec![
        next.clone(),
        one.add(&next).unwrap().scale(&real(-1)).unwrap(),
        one,
    ];
    let delta = (fraction(1, 1_u64 << 40) * fraction(1, 1_u64 << 40))
        * (fraction(1, 1_u64 << 40) * fraction(1, 1_u64 << 40));
    let half = &delta * fraction(1, 2);
    let first = decided(
        q.select(
            equation.clone(),
            real(1) - &half,
            real(1) + &half,
            &TestContext::strict(),
        )
        .unwrap(),
    );
    let second = decided(
        q.select(
            equation,
            real(1) + &half,
            real(1) + &delta + &half,
            &TestContext::strict(),
        )
        .unwrap(),
    );
    let join = decided(first.joined_with(&second, &TestContext::strict()).unwrap());
    let distance = join
        .embed(&second.generator().unwrap())
        .unwrap()
        .subtract(&join.embed(&first.generator().unwrap()).unwrap())
        .unwrap();
    is_zero(&distance.subtract(&join.embed(&epsilon).unwrap()).unwrap());
    assert_eq!(
        distance.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Positive)
    );
}

#[test]
fn join_budget_refusal_retries_and_ancestor_join_reuses_authority() {
    let q = base();
    let first = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let second = select(&q, coefficients(&q, &[-3, 0, 0, 1]), 1, 2);
    let policy = TestContext::approximate();
    let refused = with_budget(WorkBudget::ZERO, || {
        let outcome = first.joined_with(&second, &policy).unwrap();
        assert!(matches!(
            outcome,
            Classification::Uncertain(UncertaintyReason::BudgetExhausted)
        ));
        outcome
    });
    assert_eq!(refused.usage.promotions, 0);
    assert_eq!(
        refused.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    let retried = with_budget(
        WorkBudget {
            refinements: 1000,
            promotions: 10,
        },
        || first.joined_with(&second, &policy).unwrap(),
    );
    let join = decided(retried.result.unwrap());
    assert_eq!(retried.usage.promotions, 1);
    is_zero(
        &power(&join.embed(&second.generator().unwrap()).unwrap(), 3)
            .subtract(&join.field().constant(real(3)).unwrap())
            .unwrap(),
    );
    let reused = with_budget(WorkBudget::ZERO, || first.joined_with(&q, &policy).unwrap());
    assert!(
        decided(reused.result.unwrap())
            .field()
            .shares_authority(&first)
    );
    assert_eq!(reused.usage.promotions, 0);
    assert_eq!(policy.approximations(), 0);
}
