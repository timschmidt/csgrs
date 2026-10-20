//! Independent exact oracles for rational roots, bivariate square roots,
//! bilinear factors, constraint resultants and multi-affine lift helpers.
//!
//! Factors are multiplied back with local convolution helpers, roots are
//! substituted by local Horner-free evaluation, and polynomial identities
//! are checked on grids with one more node per axis than the degree bound.

use hyperreal::{Rational, Real};
use hypersolve::BivariatePolynomial;
use hypersolve::exact_factor::*;

fn r(value: i64) -> Real {
    Real::from(value)
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn rational(value: &Real) -> Rational {
    value
        .exact_rational()
        .expect("oracle values stay exact rationals")
}

fn assert_equal(left: &Real, right: &Real) {
    assert_eq!(rational(left), rational(right));
}

/// Exact zero, by structure or by a certified STRICT sign.
fn assert_zero(value: &Real) {
    if let Some(value) = value.exact_rational() {
        assert_eq!(value, Rational::zero());
        return;
    }
    assert_eq!(
        hypersolve::classify_real_sign_predicate(value, hypersolve::PredicatePolicy::STRICT)
            .value(),
        Some(hypersolve::PredicateSign::Zero)
    );
}

const NODES: [(i64, u64); 8] = [
    (0, 1),
    (1, 1),
    (-1, 1),
    (2, 1),
    (1, 2),
    (-3, 1),
    (5, 3),
    (-7, 4),
];

fn nodes(count: usize) -> Vec<Real> {
    NODES[..count]
        .iter()
        .map(|(numerator, denominator)| q(*numerator, *denominator))
        .collect()
}

fn power(base: &Real, exponent: usize) -> Real {
    (0..exponent).fold(Real::one(), |product, _| product * base)
}

fn eval1(coefficients: &[Real], x: &Real) -> Real {
    coefficients
        .iter()
        .enumerate()
        .fold(Real::zero(), |sum, (k, coefficient)| {
            sum + coefficient * power(x, k)
        })
}

fn eval2(polynomial: &BivariatePolynomial, s: &Real, t: &Real) -> Real {
    let mut sum = Real::zero();
    for (i, row) in polynomial.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            sum += coefficient * power(s, i) * power(t, j);
        }
    }
    sum
}

fn assert_identity1(bound: usize, left: impl Fn(&Real) -> Real, right: impl Fn(&Real) -> Real) {
    for x in nodes(bound + 1) {
        assert_equal(&left(&x), &right(&x));
    }
}

fn assert_identity2(
    bounds: [usize; 2],
    left: impl Fn(&Real, &Real) -> Real,
    right: impl Fn(&Real, &Real) -> Real,
) {
    let axes = bounds.map(|bound| nodes(bound + 1));
    for s in &axes[0] {
        for t in &axes[1] {
            assert_equal(&left(s, t), &right(s, t));
        }
    }
}

fn degree1(coefficients: &[Real]) -> usize {
    coefficients
        .iter()
        .rposition(|coefficient| rational(coefficient) != Rational::zero())
        .unwrap_or(0)
}

/// Per-axis degrees of the nonzero coefficients (zero for the zero polynomial).
fn degrees2(polynomial: &BivariatePolynomial) -> [usize; 2] {
    let mut degrees = [0; 2];
    for (i, row) in polynomial.coefficients.iter().enumerate() {
        for (j, coefficient) in row.iter().enumerate() {
            if rational(coefficient) != Rational::zero() {
                degrees = [degrees[0].max(i), degrees[1].max(j)];
            }
        }
    }
    degrees
}

/// Proves `coefficients == expected` for an `expected` of degree at most
/// `bound`: the result's own degree is bounded too.
fn assert_polynomial1(coefficients: &[Real], bound: usize, expected: impl Fn(&Real) -> Real) {
    assert!(degree1(coefficients) <= bound);
    assert_identity1(bound, |x| eval1(coefficients, x), expected);
}

fn assert_polynomial2(
    polynomial: &BivariatePolynomial,
    bounds: [usize; 2],
    expected: impl Fn(&Real, &Real) -> Real,
) {
    let found = degrees2(polynomial);
    assert!(
        found
            .iter()
            .zip(&bounds)
            .all(|(degree, bound)| degree <= bound),
        "degrees {found:?} exceed {bounds:?}"
    );
    assert_identity2(bounds, |s, t| eval2(polynomial, s, t), expected);
}

/// Proves `left * right == target`: over a field per-axis degrees add, and
/// the product then agrees with the target on a grid of the target degree.
fn assert_product2(
    left: &BivariatePolynomial,
    right: &BivariatePolynomial,
    target: &BivariatePolynomial,
) {
    let (first, second, expected) = (degrees2(left), degrees2(right), degrees2(target));
    for axis in 0..2 {
        assert_eq!(first[axis] + second[axis], expected[axis]);
    }
    assert_identity2(
        expected,
        |s, t| eval2(left, s, t) * eval2(right, s, t),
        |s, t| eval2(target, s, t),
    );
}

fn bi(terms: &[(Real, [usize; 2])]) -> BivariatePolynomial {
    let mut dimensions = [1_usize; 2];
    for (_, exponents) in terms {
        for axis in 0..2 {
            dimensions[axis] = dimensions[axis].max(exponents[axis] + 1);
        }
    }
    let mut coefficients = vec![vec![Real::zero(); dimensions[1]]; dimensions[0]];
    for (value, exponents) in terms {
        coefficients[exponents[0]][exponents[1]] += value;
    }
    BivariatePolynomial::new(coefficients)
}

fn mul2(left: &BivariatePolynomial, right: &BivariatePolynomial) -> BivariatePolynomial {
    let mut terms = Vec::new();
    for (i, row) in left.coefficients.iter().enumerate() {
        for (j, a) in row.iter().enumerate() {
            for (k, other_row) in right.coefficients.iter().enumerate() {
                for (l, b) in other_row.iter().enumerate() {
                    terms.push((a * b, [i + k, j + l]));
                }
            }
        }
    }
    bi(&terms)
}

fn add2(left: &BivariatePolynomial, right: &BivariatePolynomial) -> BivariatePolynomial {
    let mut terms = Vec::new();
    for polynomial in [left, right] {
        for (i, row) in polynomial.coefficients.iter().enumerate() {
            for (j, value) in row.iter().enumerate() {
                terms.push((value.clone(), [i, j]));
            }
        }
    }
    bi(&terms)
}

fn mul1(left: &[Real], right: &[Real]) -> Vec<Real> {
    let mut product = vec![Real::zero(); left.len() + right.len() - 1];
    for (i, a) in left.iter().enumerate() {
        for (j, b) in right.iter().enumerate() {
            product[i + j] += a * b;
        }
    }
    product
}

fn coefficient(polynomial: &BivariatePolynomial, first: usize, second: usize) -> Real {
    polynomial
        .coefficients
        .get(first)
        .and_then(|row| row.get(second))
        .cloned()
        .unwrap_or_else(Real::zero)
}

/// sin(e + 1) - sin(e)cos(1) - cos(e)sin(1): exactly zero, but outside the
/// admitted normal forms, so STRICT cannot certify its sign.
fn undecided_zero() -> Real {
    let zero = (Real::e() + Real::one()).sin()
        - Real::e().sin() * Real::one().cos()
        - Real::e().cos() * Real::one().sin();
    assert!(
        hypersolve::classify_real_sign_predicate(&zero, hypersolve::PredicatePolicy::STRICT)
            .value()
            .is_none()
    );
    zero
}

#[test]
fn rational_root_search_returns_only_true_rational_roots() {
    // (3x - 2)(x^2 + 1): 2/3 is the only real root.
    let cubic = [r(-2), r(3), r(-2), r(3)];
    let root = exact_rational_polynomial_root(&cubic).unwrap();
    assert_equal(&root, &q(2, 3));
    assert_equal(&eval1(&cubic, &root), &Real::zero());
    // Integer content and rational coefficients reach the same root.
    assert_equal(
        &exact_rational_polynomial_root(&[r(-4), r(6)]).unwrap(),
        &q(2, 3),
    );
    assert_equal(
        &exact_rational_polynomial_root(&[q(-1, 3), q(1, 2)]).unwrap(),
        &q(2, 3),
    );
    assert_equal(
        &exact_rational_polynomial_root(&[r(5), r(1)]).unwrap(),
        &r(-5),
    );
    // (4x + 3)(2x - 5) has two rational roots; either is a valid answer.
    let quadratic = [r(-15), r(-14), r(8)];
    let root = exact_rational_polynomial_root(&quadratic).unwrap();
    assert_equal(&eval1(&quadratic, &root), &Real::zero());

    // No rational roots exist, so no answer may be returned.
    assert!(exact_rational_polynomial_root(&[r(-2), r(0), r(1)]).is_none());
    assert!(exact_rational_polynomial_root(&[r(1), r(0), r(1)]).is_none());
    // 2x^2 + x + 4 has discriminant -31; its candidate list includes the
    // non-reduced pair 2/2.
    assert!(exact_rational_polynomial_root(&[r(4), r(1), r(2)]).is_none());
    assert!(exact_rational_polynomial_root(&[r(5)]).is_none());
    assert!(exact_rational_polynomial_root(&[]).is_none());
    let root_two = Real::from(2).sqrt().unwrap();
    assert!(exact_rational_polynomial_root(&[root_two, r(1)]).is_none());

    // Outside the bounded search, any answer must still be a root.
    let huge = Real::from(2).powi_i64(70).unwrap();
    for polynomial in [
        vec![r(0), r(1), r(1)],
        vec![r(3_000_000_000), r(1)],
        vec![r(1), r(3_000_000_000)],
        vec![r(1), huge],
    ] {
        if let Some(root) = exact_rational_polynomial_root(&polynomial) {
            assert_equal(&eval1(&polynomial, &root), &Real::zero());
        }
    }
}

#[test]
fn bivariate_support_metadata_reports_degrees_and_undecided_signs() {
    // 3 + 4st + 5s^2, with a trailing zero row and column.
    let polynomial = BivariatePolynomial::new(vec![
        vec![r(3), r(0)],
        vec![r(0), r(4), r(0)],
        vec![r(5)],
        vec![r(0), r(0)],
    ]);
    let (first_degree, second_degree, first, second, leading) =
        bivariate_exact_nonzero_metadata(&polynomial)
            .unwrap()
            .unwrap();
    assert_eq!((first_degree, second_degree, first, second), (2, 1, 2, 0));
    assert_equal(&leading, &r(5));
    assert_eq!(
        bivariate_exact_nonzero_metadata(&BivariatePolynomial::new(vec![vec![r(0)], vec![r(0)]])),
        Some(None)
    );
    assert_eq!(
        bivariate_exact_nonzero_metadata(&BivariatePolynomial::new(Vec::new())),
        Some(None)
    );
    let undecided = BivariatePolynomial::new(vec![vec![r(1), undecided_zero()]]);
    assert!(bivariate_exact_nonzero_metadata(&undecided).is_none());
    // sin(e)^2 + cos(e)^2 - 1 is exactly zero by a certified identity, so a
    // polynomial with only that coefficient is the zero polynomial.
    let identity = Real::e().sin() * Real::e().sin() + Real::e().cos() * Real::e().cos() - r(1);
    assert_eq!(
        bivariate_exact_nonzero_metadata(&BivariatePolynomial::new(vec![vec![identity]])),
        Some(None)
    );
}

#[test]
fn bivariate_square_roots_square_back_and_non_squares_are_rejected() {
    // base = 1 + 2t + 3s - s^2 t.
    let base = bi(&[
        (r(1), [0, 0]),
        (r(2), [0, 1]),
        (r(3), [1, 0]),
        (r(-1), [2, 1]),
    ]);
    let square = mul2(&base, &base);
    let root = bivariate_exact_square_root(&square).unwrap();
    // The root is +base or -base: fix the sign at (0, 0), where base = 1.
    let sign = eval2(&root, &r(0), &r(0));
    assert!(rational(&sign) == Rational::one() || rational(&sign) == -Rational::one());
    assert_polynomial2(&root, [2, 1], |s, t| &sign * eval2(&base, s, t));

    let zero = bivariate_exact_square_root(&BivariatePolynomial::new(vec![vec![r(0)]])).unwrap();
    assert_polynomial2(&zero, [1, 1], |_, _| Real::zero());

    // 2s^2 = (sqrt(2) s)^2 has an irrational square root.
    let doubled = bi(&[(r(2), [2, 0])]);
    let root = bivariate_exact_square_root(&doubled).unwrap();
    for (s, t) in [(r(1), r(0)), (r(3), r(-2)), (q(1, 2), r(5))] {
        let value = eval2(&root, &s, &t);
        assert_zero(&(&value * &value - eval2(&doubled, &s, &t)));
    }

    // None of these is the square of a real polynomial.
    let unit = BivariatePolynomial::new(vec![vec![r(1)]]);
    for non_square in [
        add2(&square, &unit),
        bi(&[(r(1), [1, 0])]),
        bi(&[(r(1), [2, 1]), (r(1), [0, 2])]),
        bi(&[(r(-1), [2, 0])]),
        bi(&[(r(1), [2, 0]), (r(1), [0, 0])]),
        bi(&[(r(1), [2, 0]), (r(1), [1, 2])]),
        bi(&[(r(1), [2, 0]), (r(2), [1, 0]), (r(2), [0, 0])]),
    ] {
        assert!(bivariate_exact_square_root(&non_square).is_none());
    }
    let undecided = BivariatePolynomial::new(vec![vec![r(1), undecided_zero()]]);
    assert!(bivariate_exact_square_root(&undecided).is_none());
}

#[test]
fn bivariate_exact_evaluation_matches_monomial_sums() {
    let polynomial = BivariatePolynomial::new(vec![
        vec![r(1), q(-1, 2)],
        vec![r(0), r(3), r(2)],
        vec![r(-4)],
    ]);
    assert_identity2(
        [2, 2],
        |s, t| bivariate_evaluate_exact(&polynomial, s, t),
        |s, t| eval2(&polynomial, s, t),
    );
}

#[test]
fn cubic_repeated_factor_probes_never_reject_a_genuine_square() {
    // (w - s)^2 (w + t) = w^3 + (t - 2s) w^2 + (s^2 - 2st) w + s^2 t.
    let constant = bi(&[(r(1), [2, 1])]);
    let linear = bi(&[(r(1), [2, 0]), (r(-2), [1, 1])]);
    let quadratic = bi(&[(r(1), [0, 1]), (r(-2), [1, 0])]);
    let cubic = bi(&[(r(1), [0, 0])]);
    assert!(!cubic_specialization_rejects_repeated_factor([
        &constant, &linear, &quadratic, &cubic
    ]));
    // w^3 - (s - 1) is square-free; its discriminant -27(s - 1)^2 vanishes
    // at the first probe s = 1 but not at the next.
    let zero = bi(&[(r(0), [0, 0])]);
    let shifted = bi(&[(r(1), [0, 0]), (r(-1), [1, 0])]);
    assert!(cubic_specialization_rejects_repeated_factor([
        &shifted, &zero, &zero, &cubic
    ]));
}

#[test]
fn trimming_removes_only_trailing_zeros() {
    let polynomial = BivariatePolynomial::new(vec![
        vec![r(0), r(2), r(0)],
        vec![r(3), r(0)],
        vec![r(0), r(0)],
    ]);
    let trimmed = bivariate_trim_exact(polynomial.clone()).unwrap();
    assert_eq!(
        trimmed,
        BivariatePolynomial::new(vec![vec![r(0), r(2)], vec![r(3)]])
    );
    assert_polynomial2(&trimmed, [2, 2], |s, t| eval2(&polynomial, s, t));
    let zero =
        bivariate_trim_exact(BivariatePolynomial::new(vec![vec![r(0), r(0)], vec![r(0)]])).unwrap();
    assert_eq!(zero, BivariatePolynomial::new(vec![vec![r(0)]]));
    let undecided = BivariatePolynomial::new(vec![vec![r(1), undecided_zero()]]);
    assert!(bivariate_trim_exact(undecided).is_none());
}

#[test]
fn bilinear_factor_from_roots_vanishes_on_the_sampled_curve() {
    // F = (2 - y) + x (1 + 3y); its root in x at sample y is -(2 - y)/(1 + 3y).
    let factor = bi(&[
        (r(2), [0, 0]),
        (r(-1), [0, 1]),
        (r(1), [1, 0]),
        (r(3), [1, 1]),
    ]);
    let root_at = |y: &Real| (-(r(2) - y) / (r(1) + r(3) * y)).unwrap();
    let samples = [r(0), r(1), r(-1)];
    let roots = samples.each_ref().map(root_at);
    let proposed = bivariate_bilinear_factor_from_roots(
        [&samples[0], &samples[1], &samples[2]],
        [&roots[0], &roots[1], &roots[2]],
    )
    .unwrap();
    assert!(
        proposed.coefficients.len() <= 2 && proposed.coefficients.iter().all(|row| row.len() <= 2)
    );
    for y in [r(2), r(5), q(1, 2), q(-7, 4)] {
        assert_equal(&eval2(&proposed, &root_at(&y), &y), &Real::zero());
    }
    // Proportional to F: G * F(1, 1) = F * G(1, 1) everywhere.
    let (one, factor_at_one) = (r(1), eval2(&factor, &r(1), &r(1)));
    let proposed_at_one = eval2(&proposed, &one, &one);
    assert_identity2(
        [1, 1],
        |s, t| eval2(&proposed, s, t) * &factor_at_one,
        |s, t| eval2(&factor, s, t) * &proposed_at_one,
    );

    // Three equal roots give the y-independent factor x - root.
    let repeated = q(2, 5);
    let constant = bivariate_bilinear_factor_from_roots(
        [&samples[0], &samples[1], &samples[2]],
        [&repeated, &repeated, &repeated],
    )
    .unwrap();
    for y in nodes(4) {
        assert_equal(&eval2(&constant, &repeated, &y), &Real::zero());
    }
    assert!(rational(&eval2(&constant, &r(0), &r(0))) != Rational::zero());

    // A repeated sample with distinct roots does not fix one factor; any
    // proposal must still vanish at every supplied point.
    let (first_root, second_root, third_root) = (r(1), r(2), r(3));
    if let Some(proposal) = bivariate_bilinear_factor_from_roots(
        [&samples[0], &samples[0], &samples[0]],
        [&first_root, &second_root, &third_root],
    ) {
        for root in [&first_root, &second_root, &third_root] {
            assert_equal(&eval2(&proposal, root, &samples[0]), &Real::zero());
        }
    }
}

#[test]
fn bounded_bilinear_factorizations_multiply_back() {
    let first = bi(&[
        (r(1), [0, 0]),
        (r(1), [1, 0]),
        (r(2), [0, 1]),
        (r(-1), [1, 1]),
    ]);
    let second = bi(&[
        (r(3), [0, 0]),
        (r(-1), [1, 0]),
        (r(1), [0, 1]),
        (r(2), [1, 1]),
    ]);
    let product = mul2(&first, &second);
    let check = |factorizations: &[(BivariatePolynomial, BivariatePolynomial)],
                 polynomial: &BivariatePolynomial| {
        for (factor, quotient) in factorizations {
            assert!(factor.coefficients.len() <= 2);
            assert_product2(factor, quotient, polynomial);
        }
    };
    let factorizations = bivariate_bilinear_factorizations_bounded(&product, 4, 64);
    assert!(!factorizations.is_empty());
    check(&factorizations, &product);
    let single = bivariate_bilinear_factorizations_bounded(&product, 1, 64);
    assert_eq!(single.len(), 1);
    check(&single, &product);
    check(
        &bivariate_bilinear_factorizations_bounded(&product, 4, 1),
        &product,
    );
    assert!(bivariate_bilinear_factorizations_bounded(&product, 0, 64).is_empty());
    assert!(bivariate_bilinear_factorizations_bounded(&product, 4, 0).is_empty());

    // (x - 1)(y - 2)(x + 3): the y-content of a cofactor is attached to the
    // y-independent factor x - 1 as a further bilinear divisor.
    let content = mul2(
        &mul2(
            &bi(&[(r(-1), [0, 0]), (r(1), [1, 0])]),
            &bi(&[(r(-2), [0, 0]), (r(1), [0, 1])]),
        ),
        &bi(&[(r(3), [0, 0]), (r(1), [1, 0])]),
    );
    let factorizations = bivariate_bilinear_factorizations_bounded(&content, 8, 256);
    assert!(!factorizations.is_empty());
    check(&factorizations, &content);
    check(
        &bivariate_bilinear_factorizations_bounded(&content, 1, 256),
        &content,
    );

    // (x - 1)(x + y): the cofactor x + y has no content in y to attach.
    let contentless = mul2(
        &bi(&[(r(-1), [0, 0]), (r(1), [1, 0])]),
        &bi(&[(r(1), [1, 0]), (r(1), [0, 1])]),
    );
    let factorizations = bivariate_bilinear_factorizations_bounded(&contentless, 4, 64);
    assert!(!factorizations.is_empty());
    check(&factorizations, &contentless);

    // x^2 + y^2 + 1 is irreducible, so it has no bilinear divisor.
    let irreducible = bi(&[(r(1), [2, 0]), (r(1), [0, 2]), (r(1), [0, 0])]);
    assert!(bivariate_bilinear_factorizations_bounded(&irreducible, 4, 64).is_empty());
}

#[test]
fn scaled_accumulation_grows_the_target_grid() {
    let mut target = bi(&[(r(1), [0, 0]), (r(1), [1, 0])]);
    let original = target.clone();
    let source = bi(&[(r(1), [0, 2]), (r(-3), [2, 1])]);
    let scale = q(-2, 3);
    bivariate_add_scaled_assign(&mut target, &source, &scale).unwrap();
    assert_eq!(target.coefficients.len(), 3);
    assert!(target.coefficients.iter().all(|row| row.len() == 3));
    assert_polynomial2(&target, [2, 2], |s, t| {
        eval2(&original, s, t) + &scale * eval2(&source, s, t)
    });
}

#[test]
fn linear_root_resultant_matches_the_homogenized_constraint() {
    let constant = bi(&[(r(1), [0, 0]), (r(1), [1, 1])]);
    let linear = bi(&[(r(2), [0, 0]), (r(-1), [1, 0]), (r(1), [0, 1])]);
    // defining = 2x^2 + 5x - 3: l^2 * defining(-c/l) = 2c^2 - 5cl - 3l^2.
    let defining = [r(-3), r(5), r(2)];
    let resultant = bivariate_linear_root_resultant(&constant, &linear, &defining).unwrap();
    assert_polynomial2(&resultant, [2, 2], |s, t| {
        let (c, l) = (eval2(&constant, s, t), eval2(&linear, s, t));
        r(2) * &c * &c - r(5) * &c * &l - r(3) * &l * &l
    });
    // x^3 - 2: l^3 * ((-c/l)^3 - 2) = -c^3 - 2l^3.
    let cubic =
        bivariate_linear_root_resultant(&constant, &linear, &[r(-2), r(0), r(0), r(1)]).unwrap();
    assert_polynomial2(&cubic, [3, 3], |s, t| {
        let (c, l) = (eval2(&constant, s, t), eval2(&linear, s, t));
        -(&c * &c * &c) - r(2) * &l * &l * &l
    });
    assert!(bivariate_linear_root_resultant(&constant, &linear, &[r(3)]).is_none());
    assert!(bivariate_linear_root_resultant(&constant, &linear, &[]).is_none());
}

#[test]
fn quadratic_constraint_resultant_matches_the_root_product() {
    // G = 3(x - 2)(x + 1/3) = 3x^2 - 5x - 2; Res(F, G) = 9 F(2) F(-1/3).
    let defining = [r(-2), r(-5), r(3)];
    let roots = [r(2), q(-1, 3)];
    let check = |polynomial: &BivariatePolynomial, bound: usize| {
        let resultant = bivariate_quadratic_constraint_resultant(polynomial, &defining).unwrap();
        assert_polynomial1(&resultant, bound, |t| {
            r(9) * eval2(polynomial, &roots[0], t) * eval2(polynomial, &roots[1], t)
        });
    };
    // F = 3x^2 + (t^2 - t) x + (1 + 2t).
    check(
        &BivariatePolynomial::new(vec![vec![r(1), r(2)], vec![r(0), r(-1), r(1)], vec![r(3)]]),
        4,
    );
    // A missing quadratic row is zero: F = (1 - t) x + t.
    check(
        &BivariatePolynomial::new(vec![vec![r(0), r(1)], vec![r(1), r(-1)]]),
        2,
    );
    let cubic = BivariatePolynomial::new(vec![vec![r(1)], vec![r(0)], vec![r(0)], vec![r(1)]]);
    assert!(bivariate_quadratic_constraint_resultant(&cubic, &defining).is_none());
    let unit = BivariatePolynomial::new(vec![vec![r(1)]]);
    assert!(bivariate_quadratic_constraint_resultant(&unit, &[r(1), r(1)]).is_none());
}

#[test]
fn univariate_reparameterization_division_and_bernstein_conversion() {
    let polynomial = [r(2), r(-1), r(0), r(3)];
    let (start, end) = (q(-1, 2), r(3));
    let restricted = polynomial_restrict_to_interval(&polynomial, &start, &end);
    assert_polynomial1(&restricted, 3, |u| {
        eval1(&polynomial, &(&start + (&end - &start) * u))
    });
    assert!(polynomial_restrict_to_interval(&[], &start, &end).is_empty());

    let cofactor = [r(1), r(-2), r(5)];
    let root = q(3, 2);
    let dividend = mul1(&[-root.clone(), r(1)], &cofactor);
    let quotient = divide_by_linear_root(&dividend, &root);
    assert_eq!(quotient.len(), cofactor.len());
    for (found, expected) in quotient.iter().zip(&cofactor) {
        assert_equal(found, expected);
    }

    let controls = [r(1), r(-2), q(5, 2)];
    let power_basis = quadratic_bernstein_to_power([&controls[0], &controls[1], &controls[2]]);
    assert_polynomial1(&power_basis, 2, |t| {
        let complement = r(1) - t;
        &controls[0] * &complement * &complement
            + r(2) * &controls[1] * t * &complement
            + &controls[2] * t * t
    });
}

/// C_0 = 1 + 2a - a^2 b, C_1 = b + 3ab, C_2 = 4a^2, in coordinates (a, b).
fn lift_fixture() -> Vec<BivariatePolynomial> {
    vec![
        bi(&[(r(1), [0, 0]), (r(2), [1, 0]), (r(-1), [2, 1])]),
        bi(&[(r(1), [0, 1]), (r(3), [1, 1])]),
        bi(&[(r(4), [2, 0])]),
    ]
}

#[test]
fn axis_lift_slices_read_powers_values_and_derivatives() {
    let coefficients = lift_fixture();
    for power_index in 0..3 {
        let slice = trivariate_axis_lift_power_slice(&coefficients, 0, power_index).unwrap();
        assert_eq!(slice.coefficients.len(), 3);
        for (axis_power, row) in slice.coefficients.iter().enumerate() {
            assert_eq!(row.len(), 2);
            for (retained, value) in row.iter().enumerate() {
                assert_equal(
                    value,
                    &coefficient(&coefficients[axis_power], power_index, retained),
                );
            }
        }
    }
    for power_index in 0..2 {
        let slice = trivariate_axis_lift_power_slice(&coefficients, 1, power_index).unwrap();
        for (axis_power, row) in slice.coefficients.iter().enumerate() {
            assert_eq!(row.len(), 3);
            for (retained, value) in row.iter().enumerate() {
                assert_equal(
                    value,
                    &coefficient(&coefficients[axis_power], retained, power_index),
                );
            }
        }
    }
    assert!(trivariate_axis_lift_power_slice(&coefficients, 2, 0).is_none());
    assert!(trivariate_axis_lift_power_slice(&[], 0, 0).is_none());
    assert_eq!(trivariate_axis_lift_degree(&coefficients, 0), Some(2));
    assert_eq!(trivariate_axis_lift_degree(&coefficients, 1), Some(1));
    assert_eq!(
        trivariate_axis_lift_degree(&[bi(&[(r(0), [1, 1])])], 0),
        None
    );
    assert_eq!(trivariate_axis_lift_degree(&[], 1), None);

    // Values and first derivatives of each C_k along the lift coordinate.
    let value = |k: usize, a: &Real, b: &Real| eval2(&coefficients[k], a, b);
    let d_first = |k: usize, a: &Real, b: &Real| match k {
        0 => r(2) - r(2) * a * b,
        1 => r(3) * b,
        _ => r(8) * a,
    };
    let d_second = |k: usize, a: &Real, _: &Real| match k {
        0 => -(a * a),
        1 => r(1) + r(3) * a,
        _ => r(0),
    };
    for anchor in [r(1), r(0), r(-1), q(5, 3)] {
        for (lift, derivative) in [(0, false), (0, true), (1, false), (1, true)] {
            let slice = trivariate_axis_lift_taylor_slice(&coefficients, lift, &anchor, derivative)
                .unwrap();
            assert_eq!(slice.coefficients.len(), coefficients.len());
            for (k, row) in slice.coefficients.iter().enumerate() {
                for other in nodes(4) {
                    let found = eval1(row, &other);
                    let expected = match (lift, derivative) {
                        (0, false) => value(k, &anchor, &other),
                        (0, true) => d_first(k, &anchor, &other),
                        (_, false) => value(k, &other, &anchor),
                        (_, true) => d_second(k, &other, &anchor),
                    };
                    assert_equal(&found, &expected);
                }
            }
        }
    }
    assert!(trivariate_axis_lift_taylor_slice(&[], 0, &r(1), false).is_none());
}

#[test]
fn multi_affine_lift_scales_and_coefficients_recover_the_factor() {
    // F(t, w, l) = A0 + l T0 with bilinear A0 = 1 + 2w + t(3 - w), T0 = w + 2t.
    let a0 = bi(&[
        (r(1), [0, 0]),
        (r(2), [0, 1]),
        (r(3), [1, 0]),
        (r(-1), [1, 1]),
    ]);
    let t0 = bi(&[(r(1), [0, 1]), (r(2), [1, 0])]);
    let at = |anchor: &Real| add2(&a0, &mul2(&bi(&[(anchor.clone(), [0, 0])]), &t0));
    let (anchor, other_anchor) = (r(1), r(-1));
    let (anchor_factor, other_factor) = (at(&anchor), at(&other_anchor));

    // Scaled slices 2A, 3B and 5T0 satisfy (1/3)3B - (1/2)2A - (b - a)(1/5)5T0 = 0,
    // so the lift scale s_t / s_a is (1/5) / (1/2).
    let scaled =
        |factor: &BivariatePolynomial, scale: i64| mul2(&bi(&[(r(scale), [0, 0])]), factor);
    let scale = rational_multi_affine_lift_scale_from_anchor_pair(
        &scaled(&anchor_factor, 2),
        &scaled(&other_factor, 3),
        &scaled(&t0, 5),
        &(&other_anchor - &anchor),
    )
    .unwrap();
    assert_equal(&scale, &q(2, 5));

    // With that scale the lifted coefficients rebuild 2F along either lift
    // coordinate: F(l = a) = 2A and dF/dl = 2T0.
    for lift_coordinate in 0..2 {
        let lifted = rational_multi_affine_lift_factor_coefficients(
            &scaled(&anchor_factor, 2),
            &scaled(&t0, 5),
            &scale,
            &anchor,
            lift_coordinate,
        )
        .unwrap();
        // Affine in the lift and retained coordinates, so the rebuilt factor
        // has degree at most one in each variable.
        assert!(
            lifted
                .iter()
                .all(|part| degrees2(part).iter().all(|degree| *degree <= 1))
        );
        let rebuilt = |t: &Real, w: &Real, l: &Real| {
            lifted
                .iter()
                .enumerate()
                .fold(Real::zero(), |sum, (axis_power, part)| {
                    let value = if lift_coordinate == 0 {
                        eval2(part, l, w)
                    } else {
                        eval2(part, w, l)
                    };
                    sum + value * power(t, axis_power)
                })
        };
        assert_identity2(
            [1, 1],
            |t, w| rebuilt(t, w, &anchor),
            |t, w| r(2) * eval2(&anchor_factor, t, w),
        );
        assert_identity2(
            [1, 1],
            |t, w| rebuilt(t, w, &(&anchor + r(1))) - rebuilt(t, w, &anchor),
            |t, w| r(2) * eval2(&t0, t, w),
        );
    }
    assert!(
        rational_multi_affine_lift_factor_coefficients(&anchor_factor, &t0, &scale, &anchor, 2)
            .is_none()
    );

    // Proportional slices leave the scale undetermined; non-bilinear slices
    // are outside the alignment.
    let proportional = rational_multi_affine_lift_scale_from_anchor_pair(
        &scaled(&t0, 2),
        &scaled(&t0, 3),
        &t0,
        &r(-2),
    );
    assert!(proportional.is_none());
    let quadratic = bi(&[(r(1), [2, 0])]);
    assert!(
        rational_multi_affine_lift_scale_from_anchor_pair(&quadratic, &other_factor, &t0, &r(-2))
            .is_none()
    );

    // P = F Q with Q = Q0 + l Q1. At l = a, P = A Qa and dP/dl = T0 Qa + A Q1;
    // on the zero set of A the ratio dP/dl / (2T0 Qa) is 1/2.
    let q0 = bi(&[(r(2), [0, 0]), (r(1), [1, 1])]);
    let q1 = bi(&[(r(1), [0, 0]), (r(-1), [0, 1])]);
    let anchor_quotient = add2(&q0, &mul2(&bi(&[(anchor.clone(), [0, 0])]), &q1));
    let first_taylor = add2(&mul2(&t0, &anchor_quotient), &mul2(&anchor_factor, &q1));
    let lift_scale = rational_multi_affine_lift_scale(
        &first_taylor,
        &anchor_factor,
        &scaled(&t0, 2),
        &anchor_quotient,
    )
    .unwrap();
    assert_equal(&lift_scale, &q(1, 2));
    // A slice with no axis-linear coefficient has no root to probe.
    let flat = BivariatePolynomial::new(vec![vec![r(1), r(1)], vec![r(0), r(0)]]);
    assert!(
        rational_multi_affine_lift_scale(&first_taylor, &flat, &t0, &anchor_quotient).is_none()
    );
}

#[test]
fn checked_binomials_match_pascals_triangle() {
    let mut row = vec![1_u64];
    for n in 0..=40_usize {
        for (k, expected) in row.iter().enumerate() {
            assert_eq!(checked_binomial(n, k), Some(*expected));
        }
        assert_eq!(checked_binomial(n, n + 1), None);
        let mut next = vec![1_u64; n + 2];
        for k in 1..=n {
            next[k] = row[k - 1] + row[k];
        }
        row = next;
    }
    // C(100, 50) exceeds u64.
    assert_eq!(checked_binomial(100, 50), None);
}
