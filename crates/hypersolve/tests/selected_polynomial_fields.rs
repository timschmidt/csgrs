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

#[test]
fn selected_field_unit_arithmetic_retains_ancestor_authority_and_guards() {
    let base = base();
    let first = select(&base, coefficients(&base, &[-2, 0, 0, 1]), 1, 2);
    let a = first.generator().unwrap();
    let second = select(
        &first,
        vec![
            a.scale(&real(-1)).unwrap(),
            first.constant(real(0)).unwrap(),
            first.constant(real(1)).unwrap(),
        ],
        1,
        2,
    );
    let b = second.generator().unwrap();
    let zero = first.constant(real(0)).unwrap();
    let one = first.constant(real(1)).unwrap();
    for value in [
        b.add(&zero).unwrap(),
        zero.add(&b).unwrap(),
        b.multiply(&one).unwrap(),
        one.multiply(&b).unwrap(),
    ] {
        assert!(value.field().shares_authority(&second));
        is_zero(&value.subtract(&b).unwrap());
        is_zero(
            &power(&value, 6)
                .subtract(&second.constant(real(2)).unwrap())
                .unwrap(),
        );
    }
    let product = b.multiply(&zero).unwrap();
    assert!(product.field().shares_authority(&second));
    is_zero(&product);
    assert!(matches!(
        product.divide(&product, &TestContext::strict()),
        Err(SelectedPolynomialError::ZeroDenominator)
    ));
    assert!(matches!(
        b.divide(&product, &TestContext::strict()),
        Err(SelectedPolynomialError::ZeroDenominator)
    ));
    is_zero(
        &second
            .evaluate_polynomial(&[a.clone(), a.scale(&real(-1)).unwrap()], &real(1))
            .unwrap(),
    );
    assert!(base.evaluate_polynomial(&[a], &real(1)).is_err());
}

#[test]
fn selector_enclosures_preserve_exact_zero_nearby_signs_and_budget_retry() {
    let q = base();
    let first = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = first.generator().unwrap();
    // A repeated defining factor keeps y^2-a nonzero as a stored polynomial,
    // while its value at the selected root is exactly zero.
    let second = select(
        &first,
        vec![
            a.multiply(&a).unwrap(),
            first.constant(real(0)).unwrap(),
            a.scale(&real(-2)).unwrap(),
            first.constant(real(0)).unwrap(),
            first.constant(real(1)).unwrap(),
        ],
        1,
        2,
    );
    let y = second.generator().unwrap();
    let square = y.multiply(&y).unwrap();
    let positive = square.add(&second.constant(real(1)).unwrap()).unwrap();
    let residual = square.subtract(&second.lift(&a).unwrap()).unwrap();
    let epsilon = (Real::one() / Real::from(1_u128 << 100)).unwrap();
    let near = residual.add(&second.constant(epsilon).unwrap()).unwrap();
    let policy = TestContext::approximate();
    for (value, expected) in [
        (&positive, RealSign::Positive),
        (&positive.scale(&real(-1)).unwrap(), RealSign::Negative),
        (&residual, RealSign::Zero),
        (&near, RealSign::Positive),
        (&near.scale(&real(-1)).unwrap(), RealSign::Negative),
    ] {
        assert_eq!(
            value.sign(&policy).unwrap(),
            Classification::Decided(expected)
        );
    }
    assert_eq!(policy.approximations(), 0);
    let refused = with_budget(WorkBudget::ZERO, || {
        assert!(hypersolve::work_budget::consume(WorkKind::RootRefinement, 1).is_err());
        assert_eq!(
            positive.sign(&policy).unwrap(),
            Classification::Uncertain(
                hypersolve::classification::UncertaintyReason::BudgetExhausted
            )
        );
    });
    assert!(refused.result.is_err());
    assert_eq!(
        positive.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Positive)
    );
}

#[path = "support/selected_polynomial_replay.rs"]
mod inverse_codec;

#[test]
fn polynomial_inverse_over_selected_coefficients_keeps_replay() {
    use hypersolve::selected_polynomial_field::SelectedPolynomialDefinition;
    let q = base();
    let parent = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = parent.generator().unwrap();
    for repeated in [false, true] {
        let mut equation = coefficients(&parent, &[0, 0, 0, 1]);
        equation[0] = a.scale(&real(-1)).unwrap();
        if repeated {
            equation = coefficients(&parent, &[0, 0, 0, 0, 0, 0, 1]);
            equation[0] = power(&a, 2);
            equation[3] = a.scale(&real(-2)).unwrap();
        }
        let degree = equation.len() - 1;
        let selected = select(&parent, equation, 1, 2);
        let t = selected.generator().unwrap();
        let one = selected.constant(real(1)).unwrap();
        for sign in [-1, 1] {
            let denominator = power(&t, 2).add(&one).unwrap().scale(&real(sign)).unwrap();
            let inverse = decided(one.divide(&denominator, &TestContext::strict()).unwrap());
            assert!(inverse.field().shares_authority(&selected));
            assert_eq!(selected.defining_polynomial().unwrap().len(), degree + 1);
            // (t^2+1)(t^4-t^2+1)=t^6+1=a^2+1, independently of
            // the inverse's polynomial basis or the original multiplicity.
            let scale = selected.lift(&power(&a, 2)).unwrap().add(&one).unwrap();
            let expected = power(&t, 4)
                .subtract(&power(&t, 2))
                .unwrap()
                .add(&one)
                .unwrap()
                .scale(&real(sign))
                .unwrap();
            is_zero(
                &inverse
                    .multiply(&scale)
                    .unwrap()
                    .subtract(&expected)
                    .unwrap(),
            );
            is_zero(
                &inverse
                    .multiply(&denominator)
                    .unwrap()
                    .subtract(&one)
                    .unwrap(),
            );
            let graph = SelectedPolynomialDefinition::from_values(
                &[t.clone(), denominator, inverse, selected.lift(&a).unwrap()],
                10000,
            )
            .unwrap();
            eprintln!(
                "inverse repeated={repeated} sign={sign} graph_nodes={}",
                graph.nodes.len()
            );
            let path = std::env::temp_dir().join(format!(
                "hypersolve-field-inverse-{}-{}-{sign}-{repeated}.json",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos(),
            ));
            std::fs::write(&path, inverse_codec::encode(&graph)).unwrap();
            let child = std::process::Command::new(std::env::current_exe().unwrap())
                .args([
                    "--exact",
                    "selected_polynomial_inverse_replay_child",
                    "--nocapture",
                ])
                .env("HYPERSOLVE_FIELD_INVERSE_REPLAY", &path)
                .output();
            std::fs::remove_file(path).unwrap();
            let child = child.unwrap();
            assert!(
                child.status.success(),
                "{}{}",
                String::from_utf8_lossy(&child.stdout),
                String::from_utf8_lossy(&child.stderr)
            );
        }
    }
}

#[test]
fn selected_polynomial_inverse_replay_child() {
    let Some(path) = std::env::var_os("HYPERSOLVE_FIELD_INVERSE_REPLAY") else {
        return;
    };
    let graph = inverse_codec::decode(&std::fs::read_to_string(path).unwrap());
    let values = decided(graph.replay(10000, &TestContext::strict()).unwrap());
    let [t, denominator, inverse, a] = values.as_slice() else {
        panic!("four retained values");
    };
    let one = t.field().constant(real(1)).unwrap();
    is_zero(&power(t, 3).subtract(a).unwrap());
    is_zero(
        &power(a, 3)
            .subtract(&t.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    is_zero(
        &inverse
            .multiply(denominator)
            .unwrap()
            .subtract(&one)
            .unwrap(),
    );
    let divisor = inverse.add(&one).unwrap();
    let next = decided(one.divide(&divisor, &TestContext::strict()).unwrap());
    is_zero(&next.multiply(&divisor).unwrap().subtract(&one).unwrap());
}

#[test]
fn polynomial_reciprocal_composition_retains_its_original_field() {
    let q = base();
    let parent = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = parent.generator().unwrap();
    let mut equation = coefficients(&parent, &[0, 0, 0, 1]);
    equation[0] = a.scale(&real(-1)).unwrap();
    let field = select(&parent, equation, 1, 2);
    let t = field.generator().unwrap();
    let one = field.constant(real(1)).unwrap();
    let fibonacci = [0, 1, 1, 2, 3, 5, 8, 13, 21, 34];
    let mut value = t.clone();
    for step in 1..=8 {
        value = decided(
            one.divide(&value.add(&one).unwrap(), &TestContext::strict())
                .unwrap(),
        );
        let numerator = t
            .scale(&real(fibonacci[step - 1]))
            .unwrap()
            .add(&field.constant(real(fibonacci[step])).unwrap())
            .unwrap();
        let denominator = t
            .scale(&real(fibonacci[step]))
            .unwrap()
            .add(&field.constant(real(fibonacci[step + 1])).unwrap())
            .unwrap();
        is_zero(
            &value
                .multiply(&denominator)
                .unwrap()
                .subtract(&numerator)
                .unwrap(),
        );
        assert!(value.field().shares_authority(&field));
    }
    assert!(field.parent().unwrap().shares_authority(&parent));
    assert_eq!(field.defining_polynomial().unwrap().len(), 4);
}
