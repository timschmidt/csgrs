//! Independent exact oracles for dense trivariate parameter arithmetic.
//!
//! Polynomial identities are checked by exact rational evaluation on a grid
//! with one more node per axis than the asserted per-axis degree bound. Two
//! polynomials of that degree that agree on the grid are equal, so every
//! identity below is a proof rather than a sample. Inputs are built and
//! multiplied with local helpers, never with the functions under test.

use hyperreal::{Rational, Real};
use hypersolve::trivariate_arithmetic::*;
use hypersolve::work_budget::{WorkBudget, WorkKind, consume, with_budget};
use hypersolve::{BivariatePolynomial, CurveIntersectionResultantConfig, TrivariatePolynomial};

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

/// Distinct rational nodes; the first `degree + 1` prove a degree bound.
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

fn eval3(polynomial: &TrivariatePolynomial, point: [&Real; 3]) -> Real {
    let mut sum = Real::zero();
    for (i, rows) in polynomial.coefficients.iter().enumerate() {
        for (j, row) in rows.iter().enumerate() {
            for (k, coefficient) in row.iter().enumerate() {
                sum += coefficient * power(point[0], i) * power(point[1], j) * power(point[2], k);
            }
        }
    }
    sum
}

fn assert_identity3(
    bounds: [usize; 3],
    left: impl Fn([&Real; 3]) -> Real,
    right: impl Fn([&Real; 3]) -> Real,
) {
    let axes = bounds.map(|bound| nodes(bound + 1));
    for x in &axes[0] {
        for y in &axes[1] {
            for z in &axes[2] {
                assert_equal(&left([x, y, z]), &right([x, y, z]));
            }
        }
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

/// Per-axis degrees of the nonzero coefficients (zero for the zero polynomial).
fn degrees3(polynomial: &TrivariatePolynomial) -> [usize; 3] {
    let mut degrees = [0; 3];
    for (i, rows) in polynomial.coefficients.iter().enumerate() {
        for (j, row) in rows.iter().enumerate() {
            for (k, coefficient) in row.iter().enumerate() {
                if rational(coefficient) != Rational::zero() {
                    degrees = [degrees[0].max(i), degrees[1].max(j), degrees[2].max(k)];
                }
            }
        }
    }
    degrees
}

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

/// Proves `polynomial == expected` when `expected` has per-axis degree at
/// most `bounds`: the result's own degrees are bounded too, so agreement on
/// the grid is equality.
fn assert_polynomial3(
    polynomial: &TrivariatePolynomial,
    bounds: [usize; 3],
    expected: impl Fn([&Real; 3]) -> Real,
) {
    let found = degrees3(polynomial);
    assert!(
        found
            .iter()
            .zip(&bounds)
            .all(|(degree, bound)| degree <= bound),
        "degrees {found:?} exceed {bounds:?}"
    );
    assert_identity3(bounds, |point| eval3(polynomial, point), expected);
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
fn assert_product3(
    left: &TrivariatePolynomial,
    right: &TrivariatePolynomial,
    target: &TrivariatePolynomial,
) {
    let (first, second, expected) = (degrees3(left), degrees3(right), degrees3(target));
    for axis in 0..3 {
        assert_eq!(first[axis] + second[axis], expected[axis]);
    }
    assert_identity3(
        expected,
        |point| eval3(left, point) * eval3(right, point),
        |point| eval3(target, point),
    );
}

/// Builds a rectangular tensor from `(coefficient, [x, y, z] exponents)`.
fn tri(terms: &[(Real, [usize; 3])]) -> TrivariatePolynomial {
    let mut dimensions = [1_usize; 3];
    for (_, exponents) in terms {
        for axis in 0..3 {
            dimensions[axis] = dimensions[axis].max(exponents[axis] + 1);
        }
    }
    let mut coefficients =
        vec![vec![vec![Real::zero(); dimensions[2]]; dimensions[1]]; dimensions[0]];
    for (value, exponents) in terms {
        coefficients[exponents[0]][exponents[1]][exponents[2]] += value;
    }
    TrivariatePolynomial { coefficients }
}

/// Builds a rectangular grid from `(coefficient, [s, t] exponents)`.
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

/// Dense convolution, independent of the tensor product under test.
fn mul3(left: &TrivariatePolynomial, right: &TrivariatePolynomial) -> TrivariatePolynomial {
    let mut terms = Vec::new();
    for (i, rows) in left.coefficients.iter().enumerate() {
        for (j, row) in rows.iter().enumerate() {
            for (k, a) in row.iter().enumerate() {
                for (l, other_rows) in right.coefficients.iter().enumerate() {
                    for (m, other_row) in other_rows.iter().enumerate() {
                        for (n, b) in other_row.iter().enumerate() {
                            terms.push((a * b, [i + l, j + m, k + n]));
                        }
                    }
                }
            }
        }
    }
    tri(&terms)
}

/// Places bivariate coefficients of ascending `axis` powers on the tensor,
/// with the two other axes in ascending order.
fn embed_axis(coefficients: &[BivariatePolynomial], axis: usize) -> TrivariatePolynomial {
    let remaining = other_axes(axis);
    let mut terms = Vec::new();
    for (power, coefficient) in coefficients.iter().enumerate() {
        for (first, row) in coefficient.coefficients.iter().enumerate() {
            for (second, value) in row.iter().enumerate() {
                let mut exponents = [0; 3];
                exponents[axis] = power;
                exponents[remaining[0]] = first;
                exponents[remaining[1]] = second;
                terms.push((value.clone(), exponents));
            }
        }
    }
    tri(&terms)
}

fn other_axes(axis: usize) -> [usize; 2] {
    match axis {
        0 => [1, 2],
        1 => [0, 2],
        _ => [0, 1],
    }
}

fn extents(polynomial: &TrivariatePolynomial) -> [usize; 3] {
    let (first, second, third) = polynomial.dimensions();
    [first, second, third]
}

fn point_with(axis_values: [(usize, &Real); 3]) -> [Real; 3] {
    let mut point = [Real::zero(), Real::zero(), Real::zero()];
    for (axis, value) in axis_values {
        point[axis] = value.clone();
    }
    point
}

fn refs(point: &[Real; 3]) -> [&Real; 3] {
    [&point[0], &point[1], &point[2]]
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

/// 1 + 2x - yz + 3x^2z + y^2/2.
fn sample_polynomial() -> TrivariatePolynomial {
    tri(&[
        (r(1), [0, 0, 0]),
        (r(2), [1, 0, 0]),
        (r(-1), [0, 1, 1]),
        (r(3), [2, 0, 1]),
        (q(1, 2), [0, 2, 0]),
    ])
}

#[test]
fn zero_tensors_report_shape_overflow_and_capacity_failure() {
    let zero = try_zero_trivariate_coefficients([2, 3, 4]).unwrap();
    assert_eq!(zero.len(), 2);
    for rows in &zero {
        assert_eq!(rows.len(), 3);
        for row in rows {
            assert_eq!(row.len(), 4);
            assert!(row.iter().all(|value| rational(value) == Rational::zero()));
        }
    }
    assert!(try_zero_trivariate_coefficients([usize::MAX, 2, 1]).is_none());
    assert!(try_zero_trivariate_coefficients([1 << 40, 1 << 40, 1]).is_none());
    // Each extent product fits in `usize`, but no reservation of that many
    // planes, rows or coefficients can be satisfied.
    assert!(try_zero_trivariate_coefficients([1 << 62, 1, 1]).is_none());
    assert!(try_zero_trivariate_coefficients([1, 1 << 62, 1]).is_none());
    assert!(try_zero_trivariate_coefficients([1, 1, 1 << 62]).is_none());
}

#[test]
fn construction_pads_ragged_tensors_and_trims_only_trailing_zeros() {
    assert!(TrivariatePolynomial::from_coefficients(Vec::new()).is_none());
    assert!(TrivariatePolynomial::from_coefficients(vec![Vec::new()]).is_none());
    assert!(TrivariatePolynomial::from_coefficients(vec![vec![Vec::new()]]).is_none());

    // 1 + 2z, stored raggedly behind zero planes, rows and columns.
    let ragged = vec![
        vec![vec![r(1), r(2), r(0)], vec![r(0)]],
        vec![vec![r(0), r(0)], vec![r(0), r(0), r(0)]],
        vec![vec![r(0)]],
    ];
    let source = TrivariatePolynomial {
        coefficients: ragged.clone(),
    };
    assert_eq!(source.dimensions(), (3, 2, 3));
    let built = TrivariatePolynomial::from_coefficients(ragged).unwrap();
    assert_eq!(built.dimensions(), (1, 1, 2));
    assert_polynomial3(&built, [2, 1, 2], |point| eval1(&[r(1), r(2)], point[2]));
    assert_polynomial3(&source, [2, 1, 2], |point| eval3(&built, point));

    // 4x^2y keeps its interior zero coefficients: extents (3, 2, 1).
    let sparse = TrivariatePolynomial::from_coefficients(vec![
        vec![vec![r(0)]],
        vec![vec![r(0)]],
        vec![vec![r(0)], vec![r(4)]],
    ])
    .unwrap();
    assert_eq!(sparse.dimensions(), (3, 2, 1));
    assert_eq!(
        sparse.coefficient(2, 1, 0).map(rational),
        Some(Rational::new(4))
    );
    assert_eq!(
        sparse.coefficient(1, 1, 0).map(rational),
        Some(Rational::zero())
    );
    assert!(sparse.coefficient(3, 0, 0).is_none());
    assert!(sparse.coefficient(0, 2, 0).is_none());
    assert!(sparse.coefficient(0, 0, 1).is_none());

    let zero = TrivariatePolynomial::from_coefficients(vec![
        vec![vec![r(0), r(0)]],
        vec![vec![r(0)], vec![r(0)]],
    ])
    .unwrap();
    assert_eq!(zero.dimensions(), (1, 1, 1));
    assert_eq!(
        zero.coefficient(0, 0, 0).map(rational),
        Some(Rational::zero())
    );
}

#[test]
fn axis_lifts_place_univariate_coefficients_on_the_requested_axis() {
    let coefficients = [r(3), r(-1), q(1, 2)];
    for axis in 0..3 {
        let lifted = TrivariatePolynomial::from_axis_polynomial(&coefficients, axis).unwrap();
        let mut expected = [1, 1, 1];
        expected[axis] = 3;
        assert_eq!(extents(&lifted), expected);
        assert_polynomial3(&lifted, [2, 2, 2], |point| {
            eval1(&coefficients, point[axis])
        });
        let same = TrivariatePolynomial::from_axis_polynomial_or_zero(&coefficients, axis).unwrap();
        assert_polynomial3(&same, [2, 2, 2], |point| eval1(&coefficients, point[axis]));
        let zero = TrivariatePolynomial::from_axis_polynomial_or_zero(&[], axis).unwrap();
        assert_eq!(extents(&zero), [1, 1, 1]);
        assert_polynomial3(&zero, [1, 1, 1], |_| Real::zero());
    }
    assert!(TrivariatePolynomial::from_axis_polynomial(&coefficients, 3).is_none());
    assert!(TrivariatePolynomial::from_axis_polynomial(&[], 0).is_none());
    assert!(TrivariatePolynomial::from_axis_polynomial_or_zero(&[], 3).is_none());
}

#[test]
fn ring_operations_match_pointwise_arithmetic() {
    let p = sample_polynomial();
    // -2 + y^2 + xz/2 - 5xyz.
    let other = tri(&[
        (r(-2), [0, 0, 0]),
        (r(1), [0, 2, 0]),
        (q(1, 2), [1, 0, 1]),
        (r(-5), [1, 1, 1]),
    ]);
    let third = tri(&[(r(7), [0, 0, 2]), (q(-1, 3), [1, 0, 0])]);
    let sum = p.add(&other).unwrap();
    assert_polynomial3(&sum, [2, 2, 1], |point| {
        eval3(&p, point) + eval3(&other, point)
    });
    let difference = p.subtract(&other).unwrap();
    assert_polynomial3(&difference, [2, 2, 1], |point| {
        eval3(&p, point) - eval3(&other, point)
    });
    let combined = other.combine(&p, true).unwrap();
    assert_polynomial3(&combined, [2, 2, 1], |point| {
        eval3(&other, point) - eval3(&p, point)
    });
    let combined = other.combine(&third, false).unwrap();
    assert_polynomial3(&combined, [1, 2, 2], |point| {
        eval3(&other, point) + eval3(&third, point)
    });
    let scaled = p.scale(&q(-3, 4)).unwrap();
    assert_polynomial3(&scaled, [2, 2, 1], |point| q(-3, 4) * eval3(&p, point));
    let cancelled = p.subtract(&p).unwrap();
    assert_eq!(extents(&cancelled), [1, 1, 1]);
    assert_polynomial3(&cancelled, [2, 2, 1], |_| Real::zero());

    let weights = [r(2), r(0), q(-1, 2)];
    let combination = TrivariatePolynomial::linear_combination(&[
        (&p, &weights[0]),
        (&other, &weights[1]),
        (&third, &weights[2]),
    ])
    .unwrap();
    assert_polynomial3(&combination, [2, 2, 2], |point| {
        r(2) * eval3(&p, point) - q(1, 2) * eval3(&third, point)
    });

    let product = p.multiply(&other).unwrap();
    assert_polynomial3(&product, [3, 4, 2], |point| {
        eval3(&p, point) * eval3(&other, point)
    });
    let products =
        TrivariatePolynomial::sum_products(&[(&p, &other, false), (&third, &other, true)]).unwrap();
    assert_polynomial3(&products, [3, 4, 3], |point| {
        eval3(&p, point) * eval3(&other, point) - eval3(&third, point) * eval3(&other, point)
    });

    // Shapes with no coefficient extent have no exact result to publish.
    let empty = TrivariatePolynomial {
        coefficients: Vec::new(),
    };
    assert_eq!(empty.dimensions(), (0, 0, 0));
    assert!(TrivariatePolynomial::linear_combination(&[]).is_none());
    assert!(TrivariatePolynomial::linear_combination(&[(&empty, &weights[0])]).is_none());
    assert!(TrivariatePolynomial::sum_products(&[]).is_none());
    assert!(empty.multiply(&empty).is_none());
    let unit = tri(&[(r(1), [0, 0, 0])]);
    assert!(TrivariatePolynomial::sum_products(&[(&empty, &unit, false)]).is_none());
}

#[test]
fn ab_ac_determinant_matches_its_two_products() {
    // Factors over (a, b) and (a, c).
    let positive_ab = bi(&[(r(1), [0, 0]), (r(2), [1, 0]), (r(1), [1, 1])]);
    let positive_ac = bi(&[(r(3), [0, 0]), (r(-1), [1, 0]), (r(1), [0, 1])]);
    let negative_ab = bi(&[(r(-1), [0, 0]), (r(1), [0, 2])]);
    let negative_ac = bi(&[(r(2), [0, 0]), (q(1, 2), [1, 1])]);
    let determinant = TrivariatePolynomial::ab_ac_determinant(
        &positive_ab,
        &positive_ac,
        &negative_ab,
        &negative_ac,
    )
    .unwrap();
    assert_polynomial3(&determinant, [2, 2, 1], |[a, b, c]| {
        eval2(&positive_ab, a, b) * eval2(&positive_ac, a, c)
            - eval2(&negative_ab, a, b) * eval2(&negative_ac, a, c)
    });
}

#[test]
fn dense_conversion_preserves_every_coefficient_value() {
    let ragged = TrivariatePolynomial {
        coefficients: vec![
            vec![vec![r(1), r(2)], vec![r(3)]],
            vec![vec![q(-1, 2)]],
            vec![vec![r(0)], vec![r(0), r(0), r(9)]],
        ],
    };
    let dense = ragged.to_dense_polynomial().unwrap();
    assert_eq!(dense.dimensions(), &[3, 2, 3]);
    assert_identity3(
        [2, 1, 2],
        |point| {
            let mut sum = Real::zero();
            for (index, coefficient) in dense.coefficients().iter().enumerate() {
                let (i, j, k) = (index / 6, (index / 3) % 2, index % 3);
                sum += coefficient * power(point[0], i) * power(point[1], j) * power(point[2], k);
            }
            sum
        },
        |point| eval3(&ragged, point),
    );
}

#[test]
fn axis_reduction_preserves_values_at_both_defining_roots() {
    // (2t - 1)(t + 3) = 2t^2 + 5t - 3 has the exact roots 1/2 and -3.
    let defining = [r(-3), r(5), r(2)];
    let roots = [q(1, 2), r(-3)];
    // Degree three on every axis, stored raggedly.
    let polynomial = TrivariatePolynomial {
        coefficients: vec![
            vec![vec![r(1), r(0), r(2), r(-1)], vec![r(3)]],
            vec![vec![r(0), r(1)]],
            vec![vec![r(2)], vec![r(0), r(0), r(1)], vec![r(1)]],
            vec![vec![r(-1), r(1)], vec![r(0)], vec![r(0), r(0), r(0), r(5)]],
        ],
    };
    for axis in 0..3 {
        let reduced =
            trivariate_reduce_axis_mod_defining(polynomial.clone(), axis, &defining).unwrap();
        // A remainder of degree one on `axis` is determined by its values at
        // the two roots, and degree three on the others by four nodes each.
        let [first, second] = other_axes(axis);
        let found = degrees3(&reduced);
        assert!(found[axis] <= 1 && found[first] <= 3 && found[second] <= 3);
        let other_nodes = nodes(4);
        for root in &roots {
            for u in &other_nodes {
                for w in &other_nodes {
                    let point = point_with([(axis, root), (first, u), (second, w)]);
                    assert_equal(
                        &eval3(&reduced, refs(&point)),
                        &eval3(&polynomial, refs(&point)),
                    );
                }
            }
        }
    }
    assert!(trivariate_reduce_axis_mod_defining(polynomial.clone(), 3, &defining).is_none());
    assert!(trivariate_reduce_axis_mod_defining(polynomial.clone(), 0, &[]).is_none());
    assert!(trivariate_reduce_axis_mod_defining(polynomial.clone(), 0, &[r(4)]).is_none());
    let linear = tri(&[(r(1), [1, 0, 0]), (r(1), [0, 2, 2])]);
    assert!(trivariate_reduce_axis_mod_defining(linear, 0, &defining).is_none());
    // A vanishing leading coefficient defines no reduction relation.
    assert!(trivariate_reduce_axis_mod_defining(polynomial, 0, &[r(1), r(1), r(0)]).is_none());
}

#[test]
fn axis_content_divides_every_fiber_exactly() {
    for axis in 0..3 {
        let [first, second] = other_axes(axis);
        let exponents = |a: usize, b: usize, c: usize| {
            let mut exponent = [0; 3];
            exponent[axis] = a;
            exponent[first] = b;
            exponent[second] = c;
            exponent
        };
        // (t - 2)(2t + 1) along `axis` times 1 + u + w^2 t; several fibers vanish.
        let content = tri(&[
            (r(-2), exponents(0, 0, 0)),
            (r(-3), exponents(1, 0, 0)),
            (r(2), exponents(2, 0, 0)),
        ]);
        let cofactor = tri(&[
            (r(1), exponents(0, 0, 0)),
            (r(1), exponents(0, 1, 0)),
            (r(1), exponents(1, 0, 2)),
        ]);
        let polynomial = mul3(&content, &cofactor);
        let found = trivariate_axis_content(&polynomial, axis).unwrap();
        // A quadratic vanishing at both content roots is a nonzero multiple
        // of the content.
        assert_eq!(found.len(), 3);
        assert!(rational(&found[2]) != Rational::zero());
        assert_equal(&eval1(&found, &r(2)), &Real::zero());
        assert_equal(&eval1(&found, &q(-1, 2)), &Real::zero());
        let quotient = trivariate_divide_axis_content(&polynomial, axis, &found).unwrap();
        let content_tensor = tri(&found
            .iter()
            .enumerate()
            .map(|(power, value)| {
                let mut exponent = [0; 3];
                exponent[axis] = power;
                (value.clone(), exponent)
            })
            .collect::<Vec<_>>());
        assert_product3(&quotient, &content_tensor, &polynomial);
        assert!(trivariate_axis_content(&cofactor, axis).is_none());
        assert!(trivariate_divide_axis_content(&polynomial, axis, &[r(2)]).is_none());
        let too_long = vec![r(1); extents(&polynomial)[axis] + 1];
        assert!(trivariate_divide_axis_content(&polynomial, axis, &too_long).is_none());
        // 1 + u is not divisible by t - 2.
        assert!(trivariate_divide_axis_content(&cofactor, axis, &[r(-2), r(1)]).is_none());
    }
    let polynomial = sample_polynomial();
    assert!(trivariate_axis_content(&polynomial, 3).is_none());
    assert!(trivariate_divide_axis_content(&polynomial, 3, &[r(1), r(1)]).is_none());
}

#[test]
fn sum_product_and_affine_substitutions_match_composition() {
    // Degree two on every axis.
    let polynomial = tri(&[
        (r(1), [0, 0, 0]),
        (r(-2), [1, 0, 1]),
        (r(3), [0, 2, 0]),
        (q(1, 2), [2, 1, 2]),
        (r(1), [0, 1, 2]),
    ]);
    let permutations = [
        [0, 1, 2],
        [0, 2, 1],
        [1, 0, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
    ];
    for [left, right, combined] in permutations {
        let sum = trivariate_substitute_sum_axis(&polynomial, left, right, combined).unwrap();
        assert_polynomial2(&sum, [4, 4], |u, w| {
            let total = u + w;
            let point = point_with([(left, u), (right, w), (combined, &total)]);
            eval3(&polynomial, refs(&point))
        });
        let product =
            trivariate_substitute_product_axis(&polynomial, left, right, combined).unwrap();
        assert_polynomial2(&product, [4, 4], |u, w| {
            let total = u * w;
            let point = point_with([(left, u), (right, w), (combined, &total)]);
            eval3(&polynomial, refs(&point))
        });
        let (scale, offset) = (q(-2, 3), r(5));
        let affine =
            trivariate_substitute_affine_axis(&polynomial, left, right, &scale, &offset).unwrap();
        assert_polynomial2(&affine, [4, 2], |u, w| {
            let substituted = &scale * u + &offset;
            let point = point_with([(left, u), (right, &substituted), (combined, w)]);
            eval3(&polynomial, refs(&point))
        });
    }
    let empty = TrivariatePolynomial {
        coefficients: Vec::new(),
    };
    let (scale, offset) = (r(1), r(0));
    for repeated in [[0, 0, 1], [0, 1, 1], [2, 1, 2], [0, 1, 3]] {
        let [left, right, combined] = repeated;
        assert!(trivariate_substitute_sum_axis(&polynomial, left, right, combined).is_none());
        assert!(trivariate_substitute_product_axis(&polynomial, left, right, combined).is_none());
    }
    assert!(trivariate_substitute_sum_axis(&empty, 0, 1, 2).is_none());
    assert!(trivariate_substitute_product_axis(&empty, 0, 1, 2).is_none());
    for (retained, substituted) in [(0, 0), (3, 0), (0, 3), (2, 2)] {
        assert!(
            trivariate_substitute_affine_axis(&polynomial, retained, substituted, &scale, &offset)
                .is_none()
        );
    }
    assert!(trivariate_substitute_affine_axis(&empty, 0, 1, &scale, &offset).is_none());
}

#[test]
fn axis_bivariate_slices_rebuild_and_specialize_the_tensor() {
    let polynomial = sample_polynomial();
    for axis in 0..3 {
        let (coefficients, remaining) =
            trivariate_axis_bivariate_coefficients(&polynomial, axis).unwrap();
        assert_eq!(remaining, other_axes(axis));
        assert_eq!(coefficients.len(), extents(&polynomial)[axis]);
        // Each slice keeps the source degree on its own axes, which bounds
        // the reassembled sum by the source degrees (2, 2, 1).
        let source_degrees = degrees3(&polynomial);
        for coefficient in &coefficients {
            let [first, second] = degrees2(coefficient);
            assert!(first <= source_degrees[remaining[0]]);
            assert!(second <= source_degrees[remaining[1]]);
        }
        assert_identity3(
            [2, 2, 1],
            |point| {
                coefficients.iter().enumerate().fold(
                    Real::zero(),
                    |sum, (power_index, coefficient)| {
                        sum + eval2(coefficient, point[remaining[0]], point[remaining[1]])
                            * power(point[axis], power_index)
                    },
                )
            },
            |point| eval3(&polynomial, point),
        );
        let rebuilt =
            trivariate_from_axis_bivariate_coefficients(&coefficients, axis, remaining).unwrap();
        assert_polynomial3(&rebuilt, [2, 2, 1], |point| eval3(&polynomial, point));
        for parameter in nodes(4) {
            let (specialized, axes) =
                trivariate_specialize_axis_bivariate(&polynomial, axis, &parameter).unwrap();
            assert_eq!(axes, remaining);
            assert_polynomial2(&specialized, [2, 2], |u, w| {
                let point = point_with([(axis, &parameter), (remaining[0], u), (remaining[1], w)]);
                eval3(&polynomial, refs(&point))
            });
        }
        assert!(trivariate_from_axis_bivariate_coefficients(&[], axis, remaining).is_none());
        assert!(
            trivariate_from_axis_bivariate_coefficients(&coefficients, axis, [axis, remaining[0]])
                .is_none()
        );
        assert!(
            trivariate_from_axis_bivariate_coefficients(
                &[BivariatePolynomial::new(Vec::new())],
                axis,
                remaining
            )
            .is_none()
        );
    }
    assert!(trivariate_axis_bivariate_coefficients(&polynomial, 3).is_none());
    assert!(trivariate_specialize_axis_bivariate(&polynomial, 3, &r(1)).is_none());
    let empty = TrivariatePolynomial {
        coefficients: Vec::new(),
    };
    assert!(trivariate_axis_bivariate_coefficients(&empty, 0).is_none());
    let hollow = TrivariatePolynomial {
        coefficients: vec![Vec::new()],
    };
    assert!(trivariate_axis_bivariate_coefficients(&hollow, 0).is_none());
}

#[test]
fn linear_axis_coefficients_and_bivariate_embeddings_are_exact() {
    // (1 + x - z) + y (2 - xz).
    let linear_in_y = tri(&[
        (r(1), [0, 0, 0]),
        (r(1), [1, 0, 0]),
        (r(-1), [0, 0, 1]),
        (r(2), [0, 1, 0]),
        (r(-1), [1, 1, 1]),
    ]);
    let (constant, linear, remaining) =
        trivariate_linear_axis_coefficients(&linear_in_y, 1).unwrap();
    assert_eq!(remaining, [0, 2]);
    assert!(
        degrees2(&constant)
            .iter()
            .chain(&degrees2(&linear))
            .all(|degree| *degree <= 1)
    );
    assert_identity3(
        [1, 1, 1],
        |[x, y, z]| eval2(&constant, x, z) + y * eval2(&linear, x, z),
        |point| eval3(&linear_in_y, point),
    );
    assert!(trivariate_linear_axis_coefficients(&sample_polynomial(), 0).is_none());
    assert!(trivariate_linear_axis_coefficients(&linear_in_y, 3).is_none());

    let surface = bi(&[
        (r(2), [0, 0]),
        (r(-1), [1, 0]),
        (q(3, 2), [1, 2]),
        (r(4), [2, 1]),
    ]);
    for axes in [[0, 1], [1, 0], [0, 2], [2, 1]] {
        let embedded = trivariate_from_bivariate_axes(&surface, axes).unwrap();
        assert_polynomial3(&embedded, [2, 2, 2], |point| {
            eval2(&surface, point[axes[0]], point[axes[1]])
        });
    }
    let empty =
        trivariate_from_bivariate_axes(&BivariatePolynomial::new(Vec::new()), [0, 2]).unwrap();
    assert_polynomial3(&empty, [1, 1, 1], |_| Real::zero());
    assert!(trivariate_from_bivariate_axes(&surface, [1, 1]).is_none());
    assert!(trivariate_from_bivariate_axes(&surface, [3, 1]).is_none());
    assert!(trivariate_from_bivariate_axes(&surface, [0, 3]).is_none());
}

#[test]
fn linear_axis_division_recovers_the_exact_cofactor() {
    for axis in 0..3 {
        let (factor, polynomial) = factored_fixture(axis);
        let factor_tensor = embed_axis(&factor, axis);
        let quotient = trivariate_divide_linear_axis_factor(&polynomial, axis, &factor).unwrap();
        assert_product3(&quotient, &factor_tensor, &polynomial);

        // A nonzero remainder, a vanishing axis coefficient, a constant axis
        // and an undecided axis coefficient each make no division claim.
        let mut shifted = polynomial.clone();
        shifted.coefficients[0][0][0] += r(1);
        assert!(trivariate_divide_linear_axis_factor(&shifted, axis, &factor).is_none());
        let zero_axis = [factor[0].clone(), bi(&[(r(0), [0, 0])])];
        assert!(trivariate_divide_linear_axis_factor(&polynomial, axis, &zero_axis).is_none());
        let constant_axis = embed_axis(&[bi(&[(r(1), [1, 1])])], axis);
        assert!(trivariate_divide_linear_axis_factor(&constant_axis, axis, &factor).is_none());
        let undecided = [factor[0].clone(), bi(&[(undecided_zero(), [0, 0])])];
        assert!(trivariate_divide_linear_axis_factor(&polynomial, axis, &undecided).is_none());
        // t is not divisible by u * t: its leading coefficient 1 is not a multiple of u.
        let bare_axis = embed_axis(&[bi(&[(r(0), [0, 0])]), bi(&[(r(1), [0, 0])])], axis);
        let by_u = [bi(&[(r(0), [0, 0])]), bi(&[(r(1), [1, 0])])];
        assert!(trivariate_divide_linear_axis_factor(&bare_axis, axis, &by_u).is_none());
    }
}

#[test]
fn box_restriction_reparameterizes_every_axis() {
    let polynomial = sample_polynomial();
    let bounds = [(q(-1, 2), r(3)), (r(2), r(5)), (q(1, 3), q(-4, 3))];
    let restricted = trivariate_restrict_to_box_bounds(
        &polynomial,
        [
            (&bounds[0].0, &bounds[0].1),
            (&bounds[1].0, &bounds[1].1),
            (&bounds[2].0, &bounds[2].1),
        ],
    );
    assert_polynomial3(&restricted, [2, 2, 1], |point| {
        let mapped: [Real; 3] = std::array::from_fn(|axis| {
            &bounds[axis].0 + (&bounds[axis].1 - &bounds[axis].0) * point[axis]
        });
        eval3(&polynomial, refs(&mapped))
    });
}

#[test]
fn box_restriction_reparameterizes_ragged_tensors() {
    // Omitted coefficients are zero. A short first row, a short later row and
    // planes with different row counts must all restrict as the polynomial
    // they denote; previously the first case lost a column and the second
    // indexed past the end of a short row.
    let cases = [
        // 1 + 5yz with a short first row.
        (vec![vec![vec![r(1)], vec![r(0), r(5)]]], [0, 1, 1]),
        // 1 + 2z + 3y with a short later row.
        (vec![vec![vec![r(1), r(2)], vec![r(3)]]], [0, 1, 1]),
        // 2 - y + x(4 + 3yz), the second plane longer than the first.
        (
            vec![
                vec![vec![r(2)], vec![r(-1)]],
                vec![vec![r(4)], vec![r(0), r(3)]],
            ],
            [1, 1, 1],
        ),
    ];
    let bounds = [(r(0), r(2)), (q(-1, 2), r(1)), (r(3), q(7, 2))];
    for (coefficients, degrees) in cases {
        let polynomial = TrivariatePolynomial { coefficients };
        let restricted = trivariate_restrict_to_box_bounds(
            &polynomial,
            [
                (&bounds[0].0, &bounds[0].1),
                (&bounds[1].0, &bounds[1].1),
                (&bounds[2].0, &bounds[2].1),
            ],
        );
        assert_polynomial3(&restricted, degrees, |point| {
            let mapped: [Real; 3] = std::array::from_fn(|axis| {
                &bounds[axis].0 + (&bounds[axis].1 - &bounds[axis].0) * point[axis]
            });
            eval3(&polynomial, refs(&mapped))
        });
    }
}

/// Shared fixture: P = F * Q along `axis`, with F = (1 + u) + (2 - uw) t and
/// Q = t^2 - u + 3w, together with F's bivariate coefficients.
fn factored_fixture(axis: usize) -> ([BivariatePolynomial; 2], TrivariatePolynomial) {
    let factor = [
        bi(&[(r(1), [0, 0]), (r(1), [1, 0])]),
        bi(&[(r(2), [0, 0]), (r(-1), [1, 1])]),
    ];
    let cofactor = embed_axis(
        &[
            bi(&[(r(-1), [1, 0]), (r(3), [0, 1])]),
            bi(&[(r(0), [0, 0])]),
            bi(&[(r(1), [0, 0])]),
        ],
        axis,
    );
    let polynomial = mul3(&embed_axis(&factor, axis), &cofactor);
    (factor, polynomial)
}

fn multiply_bivariate(
    left: &BivariatePolynomial,
    right: &BivariatePolynomial,
) -> BivariatePolynomial {
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

#[test]
fn normalized_linear_factor_division_strips_common_content() {
    let config = CurveIntersectionResultantConfig::default();
    for axis in 0..3 {
        let (factor, polynomial) = factored_fixture(axis);
        let check = |result: ([BivariatePolynomial; 2], TrivariatePolynomial)| {
            let (coefficients, quotient) = result;
            let recovered = embed_axis(&coefficients, axis);
            assert_product3(&recovered, &quotient, &polynomial);
            coefficients
        };
        // An exact divisor is returned unchanged.
        let direct = trivariate_normalize_and_divide_linear_axis_factor(
            &polynomial,
            axis,
            factor.clone(),
            config,
        )
        .unwrap()
        .unwrap();
        assert_eq!(check(direct), factor);

        // Common content in the first remaining axis, (2 + 3u), is removed.
        let content = bi(&[(r(2), [0, 0]), (r(3), [1, 0])]);
        let raw = factor
            .each_ref()
            .map(|part| multiply_bivariate(&content, part));
        let normalized = trivariate_normalize_and_divide_linear_axis_factor(
            &polynomial,
            axis,
            raw.clone(),
            config,
        )
        .unwrap()
        .unwrap();
        check(normalized);

        // A common component 1 + u + w depends on both remaining axes; it is
        // removed as a shared curve component rather than an axis factor.
        let component = bi(&[(r(1), [0, 0]), (r(1), [1, 0]), (r(1), [0, 1])]);
        let raw_component = factor
            .each_ref()
            .map(|part| multiply_bivariate(&component, part));
        let normalized = trivariate_normalize_and_divide_linear_axis_factor(
            &polynomial,
            axis,
            raw_component.clone(),
            config,
        )
        .unwrap()
        .unwrap();
        check(normalized);

        // Removing that component is admitted work. Each allowance either
        // reports exhaustion or publishes a divisor that multiplies back, and
        // once one allowance suffices every larger one does too.
        let mut recovered = false;
        for promotions in 0..12 {
            let budget = WorkBudget {
                refinements: usize::MAX,
                promotions,
            };
            let outcome = with_budget(budget, || {
                trivariate_normalize_and_divide_linear_axis_factor(
                    &polynomial,
                    axis,
                    raw_component.clone(),
                    config,
                )
            });
            match outcome.result {
                Ok(result) => {
                    check(result.unwrap().unwrap());
                    recovered = true;
                }
                Err(exceeded) => {
                    assert!(!recovered, "a larger allowance exhausted after recovery");
                    assert_eq!(exceeded.kind, WorkKind::AlgebraicPromotion);
                    assert_eq!(exceeded.limit, promotions);
                }
            }
        }
        assert!(recovered);

        // Without a budget unit, removing the (2 + 3u) content is a resource
        // failure, not a claim that no divisor exists.
        let denied = with_budget(WorkBudget::ZERO, || {
            trivariate_normalize_and_divide_linear_axis_factor(
                &polynomial,
                axis,
                raw.clone(),
                config,
            )
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        // An exact divisor needs no work units at all.
        let ready = with_budget(WorkBudget::ZERO, || {
            trivariate_normalize_and_divide_linear_axis_factor(
                &polynomial,
                axis,
                factor.clone(),
                config,
            )
        });
        assert_eq!(ready.usage, Default::default());
        check(ready.result.unwrap().unwrap().unwrap());
        // An already exhausted scope publishes nothing, even for that divisor.
        let exhausted = with_budget(WorkBudget::ZERO, || {
            assert!(consume(WorkKind::RootRefinement, 1).is_err());
            trivariate_normalize_and_divide_linear_axis_factor(
                &polynomial,
                axis,
                factor.clone(),
                config,
            )
        });
        assert!(exhausted.result.is_err());

        // (1 + u) + (3 - uw) t shares no factor with F, so nothing divides.
        let unrelated = [factor[0].clone(), bi(&[(r(3), [0, 0]), (r(-1), [1, 1])])];
        assert!(
            trivariate_normalize_and_divide_linear_axis_factor(
                &polynomial,
                axis,
                unrelated,
                config
            )
            .unwrap()
            .is_none()
        );
    }
}

#[test]
fn quadratic_axis_factorizations_multiply_back_to_the_tensor() {
    let config = CurveIntersectionResultantConfig::default();
    for axis in 0..3 {
        // (1 + u + w t)(u - 2w + (u + 2) t): distinct linear factors.
        let first = [bi(&[(r(1), [0, 0]), (r(1), [1, 0])]), bi(&[(r(1), [0, 1])])];
        let second = [
            bi(&[(r(1), [1, 0]), (r(-2), [0, 1])]),
            bi(&[(r(2), [0, 0]), (r(1), [1, 0])]),
        ];
        let polynomial = mul3(&embed_axis(&first, axis), &embed_axis(&second, axis));
        let factorizations = trivariate_quadratic_axis_factorizations(&polynomial, axis, config)
            .unwrap()
            .unwrap();
        assert!(!factorizations.is_empty() && factorizations.len() <= 2);
        for (factor, quotient) in &factorizations {
            assert_eq!(extents(factor)[axis], 2);
            assert_eq!(extents(quotient)[axis], 2);
            assert_product3(factor, quotient, &polynomial);
        }

        // A repeated factor (u + t)^2 has a zero discriminant.
        let repeated_factor = [bi(&[(r(1), [1, 0])]), bi(&[(r(1), [0, 0])])];
        let square = mul3(
            &embed_axis(&repeated_factor, axis),
            &embed_axis(&repeated_factor, axis),
        );
        let factorizations = trivariate_quadratic_axis_factorizations(&square, axis, config)
            .unwrap()
            .unwrap();
        for (factor, quotient) in &factorizations {
            assert_product3(factor, quotient, &square);
        }

        // t^2 - u is irreducible: its discriminant 4u is not a square.
        let irreducible = embed_axis(
            &[
                bi(&[(r(-1), [1, 0])]),
                bi(&[(r(0), [0, 0])]),
                bi(&[(r(1), [0, 0])]),
            ],
            axis,
        );
        assert!(
            trivariate_quadratic_axis_factorizations(&irreducible, axis, config)
                .unwrap()
                .is_none()
        );
        // Only quadratic axes are split.
        let cubic = mul3(&polynomial, &embed_axis(&first, axis));
        assert!(
            trivariate_quadratic_axis_factorizations(&cubic, axis, config)
                .unwrap()
                .is_none()
        );
        assert!(
            trivariate_quadratic_axis_factorizations(&embed_axis(&first, axis), axis, config)
                .unwrap()
                .is_none()
        );

        // Each square-root branch arrives with bivariate content: 2(u + 2)
        // times 1 + u + wt, and 2w times u - 2w + (u + 2)t. Removing that
        // content is admitted work.
        let denied = with_budget(WorkBudget::ZERO, || {
            trivariate_quadratic_axis_factorizations(&polynomial, axis, config)
        });
        assert_eq!(
            denied.result.unwrap_err().kind,
            WorkKind::AlgebraicPromotion
        );
        let exhausted = with_budget(WorkBudget::ZERO, || {
            assert!(consume(WorkKind::AlgebraicPromotion, 1).is_err());
            trivariate_quadratic_axis_factorizations(&polynomial, axis, config)
        });
        assert!(exhausted.result.is_err());
    }
    let polynomial = sample_polynomial();
    assert!(
        trivariate_quadratic_axis_factorizations(&polynomial, 3, config)
            .unwrap()
            .is_none()
    );
}

/// One multi-affine divisor of degree one on `axis` whose product with its
/// quotient is the input.
fn assert_multi_affine_factorization(
    factorizations: &[(TrivariatePolynomial, TrivariatePolynomial)],
    polynomial: &TrivariatePolynomial,
    axis: usize,
) {
    assert_eq!(factorizations.len(), 1);
    for (found, quotient) in factorizations {
        assert!(degrees3(found).iter().all(|degree| *degree <= 1));
        assert_eq!(degrees3(found)[axis], 1);
        assert_product3(found, quotient, polynomial);
    }
}

#[test]
fn multi_affine_axis_factorization_recovers_a_trilinear_divisor() {
    let config = CurveIntersectionResultantConfig::default();
    for axis in 0..3 {
        // F = 1 + u + (2w + uw) t is affine in each variable; G = t^2 + u - w + 1.
        let factor = [
            bi(&[(r(1), [0, 0]), (r(1), [1, 0])]),
            bi(&[(r(2), [0, 1]), (r(1), [1, 1])]),
        ];
        let cofactor = embed_axis(
            &[
                bi(&[(r(1), [0, 0]), (r(1), [1, 0]), (r(-1), [0, 1])]),
                bi(&[(r(0), [0, 0])]),
                bi(&[(r(1), [0, 0])]),
            ],
            axis,
        );
        let polynomial = mul3(&embed_axis(&factor, axis), &cofactor);
        let factorizations =
            trivariate_rational_multi_affine_axis_factorizations(&polynomial, axis, config)
                .unwrap()
                .unwrap();
        assert_multi_affine_factorization(&factorizations, &polynomial, axis);

        // F^2 (t + 2): at a root of F the first Taylor slice vanishes too, so
        // the scale must come from aligning two anchor slices.
        let shift = embed_axis(&[bi(&[(r(2), [0, 0])]), bi(&[(r(1), [0, 0])])], axis);
        let factor_tensor = embed_axis(&factor, axis);
        let repeated = mul3(&mul3(&factor_tensor, &factor_tensor), &shift);
        let factorizations =
            trivariate_rational_multi_affine_axis_factorizations(&repeated, axis, config)
                .unwrap()
                .unwrap();
        assert_multi_affine_factorization(&factorizations, &repeated, axis);

        // (1 + w + (2 + w) t)(t^2 + w + 1) never involves u, so the first
        // lift coordinate has degree zero and only w can carry the lift.
        let planar_factor = [
            bi(&[(r(1), [0, 0]), (r(1), [0, 1])]),
            bi(&[(r(2), [0, 0]), (r(1), [0, 1])]),
        ];
        let planar_cofactor = embed_axis(
            &[
                bi(&[(r(1), [0, 0]), (r(1), [0, 1])]),
                bi(&[(r(0), [0, 0])]),
                bi(&[(r(1), [0, 0])]),
            ],
            axis,
        );
        let planar = mul3(&embed_axis(&planar_factor, axis), &planar_cofactor);
        let factorizations =
            trivariate_rational_multi_affine_axis_factorizations(&planar, axis, config)
                .unwrap()
                .unwrap();
        assert_multi_affine_factorization(&factorizations, &planar, axis);

        // t^3 + u + w + 1 is irreducible over Q(u, w): u + w + 1 is no cube.
        let irreducible = embed_axis(
            &[
                bi(&[(r(1), [0, 0]), (r(1), [1, 0]), (r(1), [0, 1])]),
                bi(&[(r(0), [0, 0])]),
                bi(&[(r(0), [0, 0])]),
                bi(&[(r(1), [0, 0])]),
            ],
            axis,
        );
        assert!(
            trivariate_rational_multi_affine_axis_factorizations(&irreducible, axis, config)
                .unwrap()
                .is_none()
        );
        // Axes below degree three are outside this search.
        assert!(
            trivariate_rational_multi_affine_axis_factorizations(&cofactor, axis, config)
                .unwrap()
                .is_none()
        );
        let exhausted = with_budget(WorkBudget::ZERO, || {
            assert!(consume(WorkKind::RootRefinement, 1).is_err());
            trivariate_rational_multi_affine_axis_factorizations(&polynomial, axis, config)
        });
        assert!(exhausted.result.is_err());
    }
    assert!(
        trivariate_rational_multi_affine_axis_factorizations(&sample_polynomial(), 3, config)
            .unwrap()
            .is_none()
    );
}
