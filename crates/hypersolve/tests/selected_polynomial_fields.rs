//! Dependent algebraic roots must keep their defining field relations.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use hyperreal::{Real, RealSign};
use hypersolve::classification::Classification;
use hypersolve::recursive_quadratic_field::{
    RecursiveQuadraticField, RecursiveQuadraticOrderedFieldContext,
};
use hypersolve::selected_polynomial_field::{
    SelectedPolynomialError, SelectedPolynomialField, SelectedPolynomialValue,
};
use hypersolve::work_budget::{WorkBudget, WorkKind, with_budget};
use support::*;

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

fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("expected exact completion, got {other:?}"),
    }
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

fn power(value: &SelectedPolynomialValue, mut exponent: usize) -> SelectedPolynomialValue {
    let mut result = value.field().constant(real(1)).unwrap();
    let mut factor = value.clone();
    while exponent != 0 {
        if exponent % 2 == 1 {
            result = result.multiply(&factor).unwrap();
        }
        exponent /= 2;
        if exponent != 0 {
            factor = factor.multiply(&factor).unwrap();
        }
    }
    result
}

fn is_zero(value: &SelectedPolynomialValue) {
    assert_eq!(
        value.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Zero)
    );
}

#[test]
fn dependent_nonradical_generators_preserve_joint_relations() {
    let q = base();
    let a_field = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = a_field.generator().unwrap();
    let b_field = select(
        &a_field,
        vec![
            a.scale(&real(-1)).unwrap(),
            a_field.constant(real(0)).unwrap(),
            a_field.constant(real(0)).unwrap(),
            a_field.constant(real(1)).unwrap(),
        ],
        1,
        2,
    );
    let b = b_field.generator().unwrap();
    let mut c_equation = coefficients(&b_field, &[0, 0, 0, 0, 0, 1]);
    c_equation[0] = b.scale(&real(-1)).unwrap();
    let c_field = select(&b_field, c_equation, 1, 2);
    let c = c_field.generator().unwrap();
    assert!(c_field.parent().unwrap().shares_authority(&b_field));
    assert!(b_field.parent().unwrap().shares_authority(&a_field));
    is_zero(
        &power(&a, 3)
            .subtract(&a_field.constant(real(2)).unwrap())
            .unwrap(),
    );
    is_zero(&power(&b, 3).subtract(&b_field.lift(&a).unwrap()).unwrap());
    is_zero(&power(&c, 5).subtract(&c_field.lift(&b).unwrap()).unwrap());
    // Independent elimination of the three original defining relations.
    is_zero(
        &power(&c, 45)
            .subtract(&c_field.constant(real(2)).unwrap())
            .unwrap(),
    );
    assert_eq!(
        c.subtract(&c_field.constant(real(1)).unwrap())
            .unwrap()
            .sign(&TestContext::strict())
            .unwrap(),
        Classification::Decided(RealSign::Positive)
    );
    // A quotient produced in the deepest field remains an ordinary operand.
    let denominator = c.add(&c_field.lift(&a).unwrap()).unwrap();
    let image = decided(c.divide(&denominator, &TestContext::strict()).unwrap());
    is_zero(&image.multiply(&denominator).unwrap().subtract(&c).unwrap());
}

#[test]
fn repeated_nonmonic_relation_selects_one_distinct_root() {
    let q = base();
    // -3(t^3-2)^2 has one distinct root in (1,2), of multiplicity two.
    let selected = select(&q, coefficients(&q, &[-12, 0, 0, 12, 0, 0, -3]), 1, 2);
    let a = selected.generator().unwrap();
    is_zero(
        &power(&a, 3)
            .subtract(&selected.constant(real(2)).unwrap())
            .unwrap(),
    );
    assert_eq!(selected.defining_polynomial().unwrap().len(), 7);
    let [lower, upper] = selected.isolating_bounds().unwrap();
    assert_eq!(lower, &real(1));
    assert_eq!(upper, &real(2));
    // Re-selection replays the original relation and interval, including its
    // multiplicity; no serialized singleton flag is an input to this API.
    let replay = select(&q, selected.defining_polynomial().unwrap().to_vec(), 1, 2);
    let r = replay.generator().unwrap();
    is_zero(
        &power(&r, 3)
            .subtract(&replay.constant(real(2)).unwrap())
            .unwrap(),
    );
}

#[test]
fn local_denominator_guards_do_not_require_global_invertibility() {
    let q = base();
    // (t^2-2)(t-3), selecting sqrt(2). t-3 vanishes on another component.
    let selected = select(&q, coefficients(&q, &[6, -2, -3, 1]), 1, 2);
    let a = selected.generator().unwrap();
    let denominator = a.subtract(&selected.constant(real(3)).unwrap()).unwrap();
    let numerator = a.add(&selected.constant(real(1)).unwrap()).unwrap();
    let quotient = decided(
        numerator
            .divide(&denominator, &TestContext::strict())
            .unwrap(),
    );
    is_zero(
        &quotient
            .multiply(&denominator)
            .unwrap()
            .subtract(&numerator)
            .unwrap(),
    );
    assert_eq!(
        quotient.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Negative)
    );
    let zero = power(&a, 2)
        .subtract(&selected.constant(real(2)).unwrap())
        .unwrap();
    assert!(matches!(
        numerator.divide(&zero, &TestContext::strict()),
        Err(SelectedPolynomialError::ZeroDenominator)
    ));
}

#[test]
fn selectors_distinguish_no_root_ambiguity_and_invalid_evidence() {
    let q = base();
    let policy = TestContext::strict();
    let run = |p: &[i32], lo: i32, hi: i32| {
        q.select(coefficients(&q, p), Real::from(lo), Real::from(hi), &policy)
    };
    assert!(matches!(
        run(&[1, 0, 1], -1, 1),
        Err(SelectedPolynomialError::NoRoot)
    ));
    assert!(matches!(
        run(&[-2, 0, 1], -2, 2),
        Err(SelectedPolynomialError::MultipleRoots(2))
    ));
    assert!(matches!(
        run(&[0], -1, 1),
        Err(SelectedPolynomialError::ZeroPolynomial)
    ));
    assert!(matches!(
        run(&[-2, 0, 1], 2, 1),
        Err(SelectedPolynomialError::InvalidSelector)
    ));
    assert!(matches!(
        run(&[0, 1], 0, 1),
        Err(SelectedPolynomialError::InvalidSelector)
    ));
}

#[test]
fn selection_budget_refusal_does_not_poison_retry_or_permit_approximation() {
    let q = base();
    let policy = TestContext::approximate();
    let run = || q.select(coefficients(&q, &[-2, 0, 0, 1]), real(1), real(2), &policy);
    let refused = with_budget(WorkBudget::ZERO, run);
    assert_eq!(
        refused.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    assert_eq!(refused.usage.promotions, 0);
    let retried = with_budget(
        WorkBudget {
            refinements: 1000,
            promotions: 10,
        },
        run,
    );
    let field = decided(retried.result.unwrap().unwrap());
    assert_eq!(retried.usage.promotions, 1);
    is_zero(
        &power(&field.generator().unwrap(), 3)
            .subtract(&field.constant(real(2)).unwrap())
            .unwrap(),
    );
    assert_eq!(policy.approximations(), 0);
}

#[test]
fn exact_root_count_preserves_repeated_and_negative_factors() {
    let field = RecursiveQuadraticField::base(
        Vec::new(),
        constant_tensor(0, real(1)),
        constant_tensor(0, real(1)),
    )
    .unwrap();
    let mut context = RecursiveQuadraticOrderedFieldContext {
        field: field.clone(),
        policy: TestContext::strict(),
    };
    let mut count = |p: &[i32], lo: i32, hi: i32| {
        let p = p
            .iter()
            .map(|v| field.constant(Real::from(*v)).unwrap())
            .collect::<Vec<_>>();
        hypersolve::ordered_field_count_roots(&p, &Real::from(lo), &Real::from(hi), &mut context)
            .unwrap()
    };
    // (t^2-1)^2(t-2) = t^5-2t^4-2t^3+4t^2+t-2.
    assert_eq!(count(&[-2, 1, 4, -2, -2, 1], -3, 3), Some(3));
    assert_eq!(count(&[6, -3, -12, 6, 6, -3], -3, 3), Some(3));
    assert_eq!(count(&[-2, 1, 4, -2, -2, 1], 0, 3), Some(2));
    assert_eq!(count(&[-2, 1, 4, -2, -2, 1], 3, 4), Some(0));
    assert_eq!(count(&[-2, 1, 4, -2, -2, 1], 1, 3), None);
    assert_eq!(count(&[0], -1, 1), None);
}

#[test]
fn elliptic_corner_contact_keeps_original_curve_distance_and_selected_side() {
    use hypersolve::BivariatePolynomial;
    use hypersolve::recursive_quadratic_field::compose_bivariate_homogeneous_polynomial;

    // The Boolean-generated corner of y^2=1+x^3 clipped at y=1/2 is
    // q=(a,1/2), where 4a^3+3=0, -1<a<0. Its coordinates are not
    // independently materialized. This is the original radius 1/64 witness.
    let root = selected_root(
        vec![real(3), real(0), real(0), real(4)],
        real(-1),
        real(0),
        13,
    );
    let native = RecursiveQuadraticField::base(
        vec![root.clone()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let a = native.retained_root_value(&root).unwrap();
    let r = fraction(1, 64);
    let zero = native.constant(real(0)).unwrap();
    let one = native.constant(real(1)).unwrap();
    let radius = native.constant(r.clone()).unwrap();
    let x = [
        a.add(&radius).unwrap(),
        zero.clone(),
        a.subtract(&radius).unwrap(),
    ];
    let y = [
        native.constant(fraction(1, 2)).unwrap(),
        native.constant(real(2) * &r).unwrap(),
        native.constant(fraction(1, 2)).unwrap(),
    ];
    let w = [one.clone(), zero, one];
    let source = BivariatePolynomial::new(vec![
        vec![real(-1), real(0), real(1)],
        vec![],
        vec![],
        vec![real(-1)],
    ]);
    let contact = compose_bivariate_homogeneous_polynomial(&source, &x, &y, &w, 3).unwrap();
    let parent = SelectedPolynomialField::from_recursive(native);
    let equation = contact
        .iter()
        .map(|v| parent.import_recursive(v).unwrap())
        .collect();
    eprintln!("elliptic contact: select retained circle root");
    let field = select(&parent, equation, -2, -1);
    let t = field.generator().unwrap();
    let a = field.import_recursive(&a).unwrap();
    let one = field.constant(real(1)).unwrap();
    let t2 = power(&t, 2);
    let denominator = one.add(&t2).unwrap();
    eprintln!("elliptic contact: construct correlated coordinates");
    let dx = decided(
        one.subtract(&t2)
            .unwrap()
            .scale(&r)
            .unwrap()
            .divide(&denominator, &TestContext::strict())
            .unwrap(),
    );
    let dy = decided(
        t.scale(&(real(2) * &r))
            .unwrap()
            .divide(&denominator, &TestContext::strict())
            .unwrap(),
    );
    let x = a.add(&dx).unwrap();
    let y = field.constant(fraction(1, 2)).unwrap().add(&dy).unwrap();
    assert!(x.field().shares_authority(y.field()));
    eprintln!("elliptic contact: replay original source and distance");
    is_zero(
        &power(&y, 2)
            .subtract(&power(&x, 3))
            .unwrap()
            .subtract(&one)
            .unwrap(),
    );
    is_zero(
        &power(&dx, 2)
            .add(&power(&dy, 2))
            .unwrap()
            .subtract(&field.constant(&r * &r).unwrap())
            .unwrap(),
    );
    // The cut remains on the original upper branch between x=-1 and x=a;
    // the right/up circle contact would violate the authored trim.
    for value in [dx, dy] {
        assert_sign(value.sign(&TestContext::strict()), RealSign::Negative);
    }
    assert_sign(
        x.add(&one).unwrap().sign(&TestContext::strict()),
        RealSign::Positive,
    );
    assert_sign(y.sign(&TestContext::strict()), RealSign::Positive);
}

#[test]
fn nearby_exact_selectors_remain_distinct() {
    let q = base();
    let small = fraction(1, 1_u64 << 40);
    let small = &small * &small;
    let small = &small * &small;
    let quarter = (&small / real(4)).unwrap();
    let first = real(1);
    let second = &first + &small;
    // (t-1)(t-1-e), e=2^-160. Both selectors are exact and disjoint.
    let polynomial = vec![
        q.constant(second.clone()).unwrap(),
        q.constant(-(&first + &second)).unwrap(),
        q.constant(real(1)).unwrap(),
    ];
    for center in [first, second] {
        let field = decided(
            q.select(
                polynomial.clone(),
                &center - &quarter,
                &center + &quarter,
                &TestContext::strict(),
            )
            .unwrap(),
        );
        is_zero(
            &field
                .generator()
                .unwrap()
                .subtract(&field.constant(center).unwrap())
                .unwrap(),
        );
    }
}
