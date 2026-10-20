//! Independent exact oracles for conservative real-interval enclosures.
//!
//! Containment is checked against closed forms with exact rational bounds,
//! tightness against corner extrema computed with rational comparisons, and
//! irrational bounds against a certified STRICT comparison. Undecidable
//! endpoint signs must produce no enclosure rather than a guessed one.

use hyperreal::{Rational, Real, RealSign};
use hypersolve::real_interval::{
    rational_interval_bernstein_strict_sign, strict_signs_are_opposite,
};
use hypersolve::{BivariatePolynomial, RealInterval};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn h(numerator: i64, denominator: u64) -> Rational {
    Rational::fraction(numerator, denominator).unwrap()
}

fn rational(value: &Real) -> Rational {
    value
        .exact_rational()
        .expect("oracle values stay exact rationals")
}

fn interval(lower: Real, upper: Real) -> RealInterval {
    RealInterval { lower, upper }
}

fn strict_sign(value: &Real) -> Option<hypersolve::PredicateSign> {
    hypersolve::classify_real_sign_predicate(value, hypersolve::PredicatePolicy::STRICT).value()
}

/// `left <= right`, certified exactly for rationals or by STRICT otherwise.
fn at_most(left: &Real, right: &Real) -> bool {
    if let (Some(left), Some(right)) = (left.exact_rational(), right.exact_rational()) {
        return left <= right;
    }
    matches!(
        strict_sign(&(right - left)),
        Some(hypersolve::PredicateSign::Positive | hypersolve::PredicateSign::Zero)
    )
}

fn assert_contains(enclosure: &RealInterval, value: &Real) {
    assert!(
        at_most(&enclosure.lower, value),
        "lower bound exceeds {value:?}"
    );
    assert!(
        at_most(value, &enclosure.upper),
        "upper bound below {value:?}"
    );
}

fn assert_exact(enclosure: &RealInterval, lower: &Rational, upper: &Rational) {
    assert_eq!(&rational(&enclosure.lower), lower);
    assert_eq!(&rational(&enclosure.upper), upper);
}

/// Exact sample points inside a rational interval: endpoints and interior.
fn samples(interval: &RealInterval) -> Vec<Real> {
    let (lower, upper) = (&interval.lower, &interval.upper);
    [h(0, 1), h(1, 3), h(1, 2), h(3, 4), h(1, 1)]
        .into_iter()
        .map(|weight| lower + (upper - lower) * Real::new(weight))
        .collect()
}

/// sin(e + 1) - sin(e)cos(1) - cos(e)sin(1): exactly zero, but outside the
/// admitted normal forms, so STRICT cannot certify its sign.
fn undecided_zero() -> Real {
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(strict_sign(&zero).is_none());
    zero
}

fn intervals() -> Vec<RealInterval> {
    vec![
        interval(r(1), r(2)),
        interval(r(0), q(3, 2)),
        interval(r(-3), r(-1)),
        interval(r(-2), r(0)),
        interval(r(-1), r(3)),
        interval(r(0), r(0)),
        interval(q(-1, 2), q(5, 2)),
        interval(q(1, 4), q(1, 4)),
    ]
}

fn spans_zero(interval: &RealInterval) -> bool {
    rational(&interval.lower) < Rational::zero() && Rational::zero() < rational(&interval.upper)
}

fn hull(values: &[Rational]) -> (Rational, Rational) {
    let mut lower = values[0].clone();
    let mut upper = values[0].clone();
    for value in values {
        if value < &lower {
            lower = value.clone();
        }
        if value > &upper {
            upper = value.clone();
        }
    }
    (lower, upper)
}

#[test]
fn certified_signs_require_the_whole_interval_to_exclude_zero() {
    assert_eq!(
        interval(r(1), r(2)).strict_nonzero_sign(),
        Some(RealSign::Positive)
    );
    assert_eq!(
        interval(r(-2), r(-1)).strict_nonzero_sign(),
        Some(RealSign::Negative)
    );
    assert_eq!(interval(r(-1), r(1)).strict_nonzero_sign(), None);
    assert_eq!(interval(r(0), r(1)).strict_nonzero_sign(), None);
    assert_eq!(interval(r(-1), r(0)).strict_nonzero_sign(), None);
    let root_two = Real::from(2).sqrt().unwrap();
    assert_eq!(
        interval(&root_two - r(1), r(2)).strict_nonzero_sign(),
        Some(RealSign::Positive)
    );
    assert_eq!(
        interval(r(-2), r(1) - &root_two).strict_nonzero_sign(),
        Some(RealSign::Negative)
    );
    assert_eq!(interval(undecided_zero(), r(1)).strict_nonzero_sign(), None);
    assert_eq!(
        interval(r(-1), undecided_zero()).strict_nonzero_sign(),
        None
    );
}

#[test]
fn sums_differences_and_products_enclose_every_pointwise_result() {
    let intervals = intervals();
    for first in &intervals {
        for second in &intervals {
            let sum = first.add(second);
            let difference = first.subtract(second);
            let product = first.multiply(second).unwrap();
            for a in samples(first) {
                for b in samples(second) {
                    assert_contains(&sum, &(&a + &b));
                    assert_contains(&difference, &(&a - &b));
                    assert_contains(&product, &(&a * &b));
                }
            }
            // Sums and differences are exact; products are the corner hull
            // except when both factors cross zero, where they stay outward.
            assert_exact(
                &sum,
                &(rational(&first.lower) + rational(&second.lower)),
                &(rational(&first.upper) + rational(&second.upper)),
            );
            assert_exact(
                &difference,
                &(rational(&first.lower) - rational(&second.upper)),
                &(rational(&first.upper) - rational(&second.lower)),
            );
            let corners = [
                rational(&first.lower) * rational(&second.lower),
                rational(&first.lower) * rational(&second.upper),
                rational(&first.upper) * rational(&second.lower),
                rational(&first.upper) * rational(&second.upper),
            ];
            let (lower, upper) = hull(&corners);
            if spans_zero(first) && spans_zero(second) {
                assert!(rational(&product.lower) <= lower && upper <= rational(&product.upper));
            } else {
                assert_exact(&product, &lower, &upper);
            }
        }
    }

    // Irrational endpoints with certified signs.
    let root_two = Real::from(2).sqrt().unwrap();
    let root_three = Real::from(3).sqrt().unwrap();
    let positive = interval(&root_two - r(1), root_two.clone());
    let negative = interval(-root_three.clone(), r(-1));
    let product = positive.multiply(&negative).unwrap();
    for a in [&root_two - r(1), root_two.clone(), q(1, 2)] {
        for b in [-root_three.clone(), r(-1), q(-3, 2)] {
            assert_contains(&product, &(&a * &b));
        }
    }

    // An undecided endpoint sign that leaves the corner order undecided
    // yields no enclosure.
    let hidden = undecided_zero();
    assert!(
        interval(hidden.clone(), r(1))
            .multiply(&interval(r(1), r(2)))
            .is_none()
    );
    assert!(
        interval(r(-1), r(1))
            .multiply(&interval(hidden.clone(), r(1)))
            .is_none()
    );
}

#[test]
fn squares_avoid_dependent_cross_products() {
    for (lower, upper, square_lower, square_upper) in [
        (h(1, 1), h(2, 1), h(1, 1), h(4, 1)),
        (h(-3, 1), h(-1, 1), h(1, 1), h(9, 1)),
        (h(-3, 1), h(2, 1), h(0, 1), h(9, 1)),
        (h(-1, 1), h(2, 1), h(0, 1), h(4, 1)),
        (h(-2, 1), h(2, 1), h(0, 1), h(4, 1)),
        (h(0, 1), h(3, 1), h(0, 1), h(9, 1)),
        (h(-2, 1), h(0, 1), h(0, 1), h(4, 1)),
        (h(-1, 2), h(1, 3), h(0, 1), h(1, 4)),
    ] {
        let square = interval(Real::new(lower), Real::new(upper))
            .square()
            .unwrap();
        assert_exact(&square, &square_lower, &square_upper);
    }
    // |lower| versus upper is undecided for [-1 + z, 1] with z an unproved
    // zero; the enclosure stays outward with lower^2 + upper^2.
    let hidden = undecided_zero();
    let square = interval(r(-1) + &hidden, r(1)).square().unwrap();
    assert_eq!(rational(&square.lower), Rational::zero());
    assert_contains(&square, &r(1));
    assert!(interval(hidden.clone(), r(1)).square().is_none());
    assert!(interval(r(-1), hidden).square().is_none());
}

#[test]
fn quotients_require_a_denominator_that_excludes_zero() {
    let numerators = [
        interval(r(1), r(2)),
        interval(r(-1), r(3)),
        interval(r(-4), r(-2)),
    ];
    let denominators = [
        interval(r(2), r(4)),
        interval(r(-4), r(-2)),
        interval(q(1, 3), q(1, 2)),
    ];
    for numerator in &numerators {
        for denominator in &denominators {
            let quotient = numerator.divide(denominator).unwrap();
            let mut corners = Vec::new();
            for a in [&numerator.lower, &numerator.upper] {
                for b in [&denominator.lower, &denominator.upper] {
                    corners.push(rational(a) / rational(b));
                }
            }
            let (lower, upper) = hull(&corners);
            assert_exact(&quotient, &lower, &upper);
            for a in samples(numerator) {
                for b in samples(denominator) {
                    assert_contains(&quotient, &(&a / &b).unwrap());
                }
            }
        }
    }
    let numerator = interval(r(1), r(2));
    assert!(numerator.divide(&interval(r(-1), r(1))).is_none());
    assert!(numerator.divide(&interval(r(0), r(1))).is_none());
    assert!(numerator.divide(&interval(r(-1), r(0))).is_none());
    assert!(
        numerator
            .divide(&interval(undecided_zero(), r(1)))
            .is_none()
    );
}

#[test]
fn polynomial_enclosures_contain_every_sampled_value() {
    // p(x) = 1 - 3x + x^2.
    let coefficients = [r(1), r(-3), r(1)];
    let evaluate = |x: &Real| r(1) - r(3) * x + x * x;
    for parameter in [
        interval(r(0), r(1)),
        interval(r(-1), r(2)),
        interval(q(1, 2), q(1, 2)),
    ] {
        let enclosure = RealInterval::evaluate_power_basis(&coefficients, &parameter).unwrap();
        for x in samples(&parameter) {
            assert_contains(&enclosure, &evaluate(&x));
        }
    }
    let empty = RealInterval::evaluate_power_basis(&[], &interval(r(0), r(1))).unwrap();
    assert_exact(&empty, &Rational::zero(), &Rational::zero());

    // P(s, t) = 1 + st - 2s^2 over [-1, 1] x [0, 2].
    let polynomial = BivariatePolynomial::new(vec![vec![r(1)], vec![r(0), r(1)], vec![r(-2)]]);
    let (first, second) = (interval(r(-1), r(1)), interval(r(0), r(2)));
    let enclosure =
        RealInterval::evaluate_bivariate_power_basis(&polynomial, &first, &second).unwrap();
    for s in samples(&first) {
        for t in samples(&second) {
            assert_contains(&enclosure, &(r(1) + &s * &t - r(2) * &s * &s));
        }
    }

    // A parameter interval [z, 2z] with an unproved zero z leaves the order
    // of z and 2z undecided.
    let hidden = undecided_zero();
    let undecided = interval(hidden.clone(), r(2) * &hidden);
    assert!(RealInterval::evaluate_power_basis(&[r(1), r(1)], &undecided).is_none());
    assert!(
        RealInterval::evaluate_bivariate_power_basis(
            &BivariatePolynomial::new(vec![vec![r(1), r(1)]]),
            &first,
            &undecided
        )
        .is_none()
    );
}

/// Whether a rational lies on the dyadic grid with spacing `2^precision`.
fn on_grid(value: &Rational, precision: i32) -> bool {
    let scale = Real::from(2).powi_i64(-i64::from(precision)).unwrap();
    rational(&(Real::new(value.clone()) * scale)).denominator() == &num::BigUint::from(1_u8)
}

#[test]
fn square_root_enclosures_round_outward() {
    let exact = interval(r(4), r(9)).nonnegative_square_root(None).unwrap();
    assert_exact(&exact, &h(2, 1), &h(3, 1));
    let symbolic = interval(r(2), r(3)).nonnegative_square_root(None).unwrap();
    for (bound, radicand) in [(&symbolic.lower, r(2)), (&symbolic.upper, r(3))] {
        let difference = bound * bound - radicand;
        assert_eq!(
            strict_sign(&difference),
            Some(hypersolve::PredicateSign::Zero)
        );
    }

    // On the grid 2^-20, sqrt(2) rounds down to the largest grid point whose
    // square is at most 2, and sqrt(9/4) = 3/2 is exact.
    let precision = -20;
    let step = h(1, 1 << 20);
    let dyadic = interval(r(2), q(9, 4))
        .nonnegative_square_root(Some(precision))
        .unwrap();
    let lower = rational(&dyadic.lower);
    assert!(on_grid(&lower, precision));
    assert!(&lower * &lower <= h(2, 1));
    let next = &lower + &step;
    assert!(&next * &next > h(2, 1));
    assert_exact(&dyadic, &lower, &h(3, 2));
    // sqrt(3) rounds up on the same grid.
    let upper = rational(
        &interval(r(1), r(3))
            .nonnegative_square_root(Some(precision))
            .unwrap()
            .upper,
    );
    assert!(on_grid(&upper, precision) && &upper * &upper >= h(3, 1));
    let previous = &upper - &step;
    assert!(&previous * &previous < h(3, 1));

    // Irrational radicands are first enclosed outward on the grid.
    let root_two = Real::from(2).sqrt().unwrap();
    let bounds = interval(root_two.clone(), Real::pi())
        .nonnegative_square_root(Some(-10))
        .unwrap();
    let (lower, upper) = (rational(&bounds.lower), rational(&bounds.upper));
    assert!(on_grid(&lower, -10) && on_grid(&upper, -10));
    assert!(at_most(&Real::new(&lower * &lower), &root_two));
    assert!(at_most(&Real::pi(), &Real::new(&upper * &upper)));
    // A coarse grid may enclose a small positive radicand below zero; the
    // root's lower bound is then clamped to zero.
    let small = interval(&root_two - r(1), r(4))
        .nonnegative_square_root(Some(0))
        .unwrap();
    assert!(rational(&small.lower) >= Rational::zero());
    assert!(at_most(
        &Real::new(rational(&small.lower) * rational(&small.lower)),
        &(&root_two - r(1))
    ));
    assert_exact(&small, &rational(&small.lower), &h(2, 1));
    // sqrt(2) - 7/5 is about 0.014; on the grid 1/4 its radicand enclosure
    // dips below zero, and the root's lower bound is clamped to zero.
    let tiny = &root_two - q(7, 5);
    let clamped = interval(tiny.clone(), r(1))
        .nonnegative_square_root(Some(-2))
        .unwrap();
    assert_exact(&clamped, &h(0, 1), &h(1, 1));
    assert!(at_most(&r(0), &tiny));
    // A positive precision is outside the integer-root fast path; the bounds
    // still enclose the true roots.
    let coarse = interval(r(2), r(3))
        .nonnegative_square_root(Some(3))
        .unwrap();
    let (lower, upper) = (rational(&coarse.lower), rational(&coarse.upper));
    assert!(lower <= Rational::zero() || &lower * &lower <= h(2, 1));
    assert!(upper >= Rational::zero() && &upper * &upper >= h(3, 1));

    // Negative parts clamp to zero; a wholly negative interval has no root.
    assert_exact(
        &interval(r(-1), r(4)).nonnegative_square_root(None).unwrap(),
        &h(0, 1),
        &h(2, 1),
    );
    assert_exact(
        &interval(r(-1), r(0))
            .nonnegative_square_root(Some(-8))
            .unwrap(),
        &h(0, 1),
        &h(0, 1),
    );
    assert!(
        interval(r(-4), r(-1))
            .nonnegative_square_root(None)
            .is_none()
    );
    assert!(
        interval(r(0), undecided_zero())
            .nonnegative_square_root(None)
            .is_none()
    );
    assert!(
        interval(undecided_zero(), r(1))
            .nonnegative_square_root(None)
            .is_none()
    );
}

#[test]
fn value_hulls_need_certified_orders() {
    let hull = RealInterval::from_values([r(3), r(-1), q(5, 2), q(-1, 2)]).unwrap();
    assert_exact(&hull, &h(-1, 1), &h(3, 1));
    let rising = RealInterval::from_values([q(-1, 2), r(4), r(-2), q(9, 2)]).unwrap();
    assert_exact(&rising, &h(-2, 1), &h(9, 2));
    assert!(RealInterval::from_values::<0>([]).is_none());
    let single = RealInterval::from_values([q(7, 3)]).unwrap();
    assert_exact(&single, &h(7, 3), &h(7, 3));
    let hidden = undecided_zero();
    assert!(RealInterval::from_values([hidden.clone(), r(2) * &hidden]).is_none());
}

fn controls(values: &[(Rational, Rational)]) -> Vec<[Rational; 2]> {
    values
        .iter()
        .map(|(lower, upper)| [lower.clone(), upper.clone()])
        .collect()
}

#[test]
fn bernstein_interval_signs_are_certified_or_withheld() {
    let point = |value: Rational| (value.clone(), value);
    // Positive and negative controls certify immediately.
    assert_eq!(
        rational_interval_bernstein_strict_sign(controls(&[
            point(h(1, 1)),
            point(h(2, 1)),
            point(h(1, 2))
        ])),
        Some(RealSign::Positive)
    );
    assert_eq!(
        rational_interval_bernstein_strict_sign(controls(&[(h(-2, 1), h(-1, 1)), point(h(-3, 1))])),
        Some(RealSign::Negative)
    );
    // (t - 1/2)^2 + 1/16 has a negative middle control but is positive on
    // [0, 1]; subdivision certifies it.
    assert_eq!(
        rational_interval_bernstein_strict_sign(controls(&[
            point(h(5, 16)),
            point(h(-3, 16)),
            point(h(5, 16)),
        ])),
        Some(RealSign::Positive)
    );
    // -(t - 1/2)^2 - 1/16 is negative throughout.
    assert_eq!(
        rational_interval_bernstein_strict_sign(controls(&[
            point(h(-5, 16)),
            point(h(3, 16)),
            point(h(-5, 16)),
        ])),
        Some(RealSign::Negative)
    );
    // t - 1/2 changes sign, (t - 1/2)^2 touches zero, and an interval of
    // controls containing (t - 1/2)^2 cannot be certified either.
    assert_eq!(
        rational_interval_bernstein_strict_sign(controls(&[point(h(-1, 2)), point(h(1, 2))])),
        None
    );
    assert_eq!(
        rational_interval_bernstein_strict_sign(controls(&[
            point(h(1, 4)),
            point(h(-1, 4)),
            point(h(1, 4)),
        ])),
        None
    );
    assert_eq!(
        rational_interval_bernstein_strict_sign(controls(&[
            (h(1, 4), h(1, 2)),
            (h(-1, 2), h(1, 1)),
            (h(1, 4), h(1, 2)),
        ])),
        None
    );
    assert_eq!(rational_interval_bernstein_strict_sign(Vec::new()), None);
}

#[test]
fn opposite_signs_are_only_certified_strict_pairs() {
    let signs = [
        None,
        Some(RealSign::Negative),
        Some(RealSign::Zero),
        Some(RealSign::Positive),
    ];
    for first in signs {
        for second in signs {
            let expected = matches!(
                (first, second),
                (Some(RealSign::Negative), Some(RealSign::Positive))
                    | (Some(RealSign::Positive), Some(RealSign::Negative))
            );
            assert_eq!(strict_signs_are_opposite(first, second), expected);
        }
    }
}
