use hyperreal::{Rational, Real, ZeroKnowledge};

fn root(value: i64) -> Real {
    Real::from(value).sqrt().unwrap()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn assert_rational(value: Real, expected: i64) {
    assert_eq!(
        value.exact_rational_normal_form(),
        Some(Rational::new(expected)),
    );
    if expected == 0 {
        assert_eq!(value.zero_status(), ZeroKnowledge::Zero);
    }
}

#[test]
fn independent_radical_linear_forms_cancel_across_grouping_and_scaling() {
    let a = root(2);
    let b = root(3);
    let c = root(5);
    for (x, y, z) in [(1, 2, 3), (-2, 3, 1), (3, -1, -2)] {
        let left = (&a * Real::from(x) + &b * Real::from(y)) + &c * Real::from(z);
        let right = &a * Real::from(x) + (&b * Real::from(y) + &c * Real::from(z));
        assert_rational(left - right + Real::from(7), 7);
    }
}

#[test]
fn independent_radical_products_replay_exact_square_relations() {
    let a = root(2);
    let b = root(3);
    let c = root(5);
    assert_rational((&a + &b) * (&a - &b), -1);
    let sum = (&a + &b) + &c;
    let cross_terms = Real::from(2) * ((root(6) + root(10)) + root(15));
    assert_rational(&sum * &sum - cross_terms, 10);
    assert_rational((root(6) * root(15)) - Real::from(3) * root(10), 0);
}

#[test]
fn exact_circle_tangency_survives_multiradical_translation() {
    let translation = root(2) + q(2, 3) * root(3);
    let radius = q(1, 1000);
    let center = &translation + &radius;
    let denominator = q(4, 3);
    let direction = Real::from(10) * &denominator;
    let separation = (&translation - center) * &denominator;
    let cross = separation * &direction;
    let radial = &direction * &direction * &denominator * &denominator * &radius * &radius;
    assert_rational(Real::from(4) * (radial - &cross * &cross), 0);
}

#[test]
fn distinct_radicals_remain_nonrational() {
    for value in [root(2) - root(3), root(2) + root(3), root(6) - root(10)] {
        assert!(value.exact_rational_normal_form().is_none());
        assert_ne!(value.zero_status(), ZeroKnowledge::Zero);
    }
}

#[test]
fn radical_cancellation_preserves_laurent_and_opaque_linear_terms() {
    let a = root(2);
    let b = root(3);
    let angle = (root(5) + Real::one()).atan().unwrap();
    let left = (&a + &angle) + &b;
    let right = &a + (&b + &angle);
    assert_rational(left - right, 0);

    let pi = Real::pi();
    let expanded = (&a * &pi + &b * &pi) - (&a + &b) * &pi;
    assert_rational(expanded, 0);
    let product = ((&a + &b) * &pi) * ((&a - &b) / pi).unwrap();
    assert_rational(product, -1);
}

#[test]
fn cancelled_independent_radicals_do_not_hide_a_quadratic_square() {
    let a = root(2);
    let b = root(3);
    let value = ((&a + &b) + Real::from(3)) * ((&a - &b) + Real::from(3)) + Real::from(3);
    let principal = value.sqrt().unwrap();
    let expected = Real::from(3) + a;
    assert_rational(&principal - &expected, 0);
    assert_rational((expected / principal).unwrap(), 1);
}
