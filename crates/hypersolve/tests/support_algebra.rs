//! Independent exact oracles for radical expressions, dense tensor support
//! and quotient-ring rational-map matrices.
//!
//! Radical arithmetic is checked by evaluating every conjugate branch at
//! points where each radicand is an exact rational square. Tensor helpers are
//! checked by exact evaluation, and selected-root reductions by the value they
//! must preserve at the root. Quotient-ring matrices are checked through
//! their eigenvector relation at the roots of the source polynomial.

use hyperreal::{Rational, Real, RealSign};
use hypersolve::quotient_ring::*;
use hypersolve::radical_expression::{SquareRootExpression, TwoSquareRootExpression};
use hypersolve::tensor_support::*;
use hypersolve::{
    AlgebraicRootRepresentation, BivariatePolynomial, DenseTensorPolynomial, TrivariatePolynomial,
};

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

fn strict_sign(value: &Real) -> Option<hypersolve::PredicateSign> {
    hypersolve::classify_real_sign_predicate(value, hypersolve::PredicatePolicy::STRICT).value()
}

/// Exact zero, by structure or by a certified STRICT sign.
fn assert_zero(value: &Real) {
    if let Some(value) = value.exact_rational() {
        assert_eq!(value, Rational::zero());
        return;
    }
    assert_eq!(strict_sign(value), Some(hypersolve::PredicateSign::Zero));
}

const NODES: [(i64, u64); 6] = [(0, 1), (1, 1), (-1, 1), (2, 1), (1, 2), (-3, 1)];

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

/// Row-major exponents of one flat dense-tensor index.
fn exponents(dimensions: &[usize], mut index: usize) -> Vec<usize> {
    let mut exponents = vec![0; dimensions.len()];
    for axis in (0..dimensions.len()).rev() {
        exponents[axis] = index % dimensions[axis];
        index /= dimensions[axis];
    }
    exponents
}

fn eval_dense(polynomial: &DenseTensorPolynomial, point: &[Real]) -> Real {
    assert_eq!(point.len(), polynomial.dimensions().len());
    polynomial
        .coefficients()
        .iter()
        .enumerate()
        .fold(Real::zero(), |sum, (index, coefficient)| {
            let monomial = exponents(polynomial.dimensions(), index)
                .iter()
                .zip(point)
                .fold(Real::one(), |product, (exponent, value)| {
                    product * power(value, *exponent)
                });
            sum + coefficient * monomial
        })
}

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

fn dense(dimensions: &[usize], terms: &[(Real, &[usize])]) -> DenseTensorPolynomial {
    let count = dimensions.iter().product();
    let mut coefficients = vec![Real::zero(); count];
    for (value, exponents) in terms {
        let index = exponents
            .iter()
            .zip(dimensions)
            .fold(0, |index, (exponent, dimension)| {
                index * dimension + exponent
            });
        coefficients[index] += value;
    }
    DenseTensorPolynomial::try_new(dimensions.to_vec(), coefficients).unwrap()
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

/// `x` values where A = x + 1 is a rational square, with its root.
fn a_squares() -> Vec<(Real, Real)> {
    [
        (0, 1, 1, 1),
        (3, 1, 2, 1),
        (8, 1, 3, 1),
        (-3, 4, 1, 2),
        (15, 1, 4, 1),
        (24, 1, 5, 1),
    ]
    .into_iter()
    .map(|(n, d, root_n, root_d)| (q(n, d), q(root_n, root_d)))
    .collect()
}

/// `y` values where B = y + 4 is a rational square, with its root.
fn b_squares() -> Vec<(Real, Real)> {
    [
        (-3, 1, 1, 1),
        (0, 1, 2, 1),
        (5, 1, 3, 1),
        (-15, 4, 1, 2),
        (12, 1, 4, 1),
    ]
    .into_iter()
    .map(|(n, d, root_n, root_d)| (q(n, d), q(root_n, root_d)))
    .collect()
}

fn radicand_a() -> TrivariatePolynomial {
    tri(&[(r(1), [0, 0, 0]), (r(1), [1, 0, 0])])
}

fn radicand_b() -> TrivariatePolynomial {
    tri(&[(r(4), [0, 0, 0]), (r(1), [0, 1, 0])])
}

fn value1(
    expression: &SquareRootExpression<TrivariatePolynomial>,
    point: [&Real; 3],
    root: &Real,
) -> Real {
    eval3(&expression.rational, point) + root * eval3(&expression.radical, point)
}

fn value2(
    expression: &TwoSquareRootExpression<TrivariatePolynomial>,
    point: [&Real; 3],
    first_root: &Real,
    second_root: &Real,
) -> Real {
    eval3(&expression.rational, point)
        + first_root * eval3(&expression.first, point)
        + second_root * eval3(&expression.second, point)
        + first_root * second_root * eval3(&expression.product, point)
}

#[test]
fn square_root_expressions_respect_both_conjugate_branches() {
    // e1 = (1 + x - yz) + (2 - y) sqrt(A), e2 = (z + xy) + (1/2 + xz) sqrt(A).
    let e1 = SquareRootExpression {
        rational: tri(&[(r(1), [0, 0, 0]), (r(1), [1, 0, 0]), (r(-1), [0, 1, 1])]),
        radical: tri(&[(r(2), [0, 0, 0]), (r(-1), [0, 1, 0])]),
    };
    let e2 = SquareRootExpression {
        rational: tri(&[(r(1), [0, 0, 1]), (r(1), [1, 1, 0])]),
        radical: tri(&[(q(1, 2), [0, 0, 0]), (r(1), [1, 0, 1])]),
    };
    let weight = tri(&[(r(3), [0, 0, 0]), (r(-1), [0, 0, 1])]);
    let a = radicand_a();
    let (two, minus_one, scale) = (r(2), r(-1), q(-2, 3));
    let sum = e1.add(&e2).unwrap();
    let difference = e1.subtract(&e2).unwrap();
    let scaled = e1.scale(&scale).unwrap();
    let combination =
        SquareRootExpression::linear_combination(&[(&e1, &two), (&e2, &minus_one)]).unwrap();
    let weighted = e1.multiply_rational(&weight).unwrap();
    let product = e1.multiply(&e2, &a).unwrap();
    let square = e1.square(&a).unwrap();
    let projection = e1.projection(&a).unwrap();
    let embedded = SquareRootExpression::from_rational(weight.clone()).unwrap();
    // Comparing both conjugate branches at every node checks the rational
    // and radical parts of each result separately.
    for (x, root) in a_squares() {
        for y in nodes(3) {
            for z in nodes(3) {
                let point = [&x, &y, &z];
                let w = eval3(&weight, point);
                for sign in [r(1), r(-1)] {
                    let root = &sign * &root;
                    let (v1, v2) = (value1(&e1, point, &root), value1(&e2, point, &root));
                    assert_equal(&value1(&sum, point, &root), &(&v1 + &v2));
                    assert_equal(&value1(&difference, point, &root), &(&v1 - &v2));
                    assert_equal(&value1(&scaled, point, &root), &(&scale * &v1));
                    assert_equal(&value1(&combination, point, &root), &(r(2) * &v1 - &v2));
                    assert_equal(&value1(&weighted, point, &root), &(&w * &v1));
                    assert_equal(&value1(&product, point, &root), &(&v1 * &v2));
                    assert_equal(&value1(&square, point, &root), &(&v1 * &v1));
                    assert_equal(&value1(&embedded, point, &root), &w);
                }
                let conjugates = value1(&e1, point, &root) * value1(&e1, point, &(-root.clone()));
                assert_equal(&eval3(&projection, point), &conjugates);
            }
        }
    }

    // Mismatched dense ranks have no exact sum, product or norm.
    let rank_two = DenseTensorPolynomial::from_axis_polynomial(2, 0, &[r(1), r(1)]).unwrap();
    let rank_three = DenseTensorPolynomial::from_axis_polynomial(3, 0, &[r(1), r(1)]).unwrap();
    let left = SquareRootExpression::from_rational(rank_two.clone()).unwrap();
    let right = SquareRootExpression::from_rational(rank_three.clone()).unwrap();
    assert!(left.add(&right).is_none());
    assert!(left.subtract(&right).is_none());
    assert!(left.multiply(&right, &rank_two).is_none());
    assert!(left.multiply_rational(&rank_three).is_none());
    assert!(left.projection(&rank_three).is_none());
    assert!(SquareRootExpression::linear_combination(&[(&left, &two), (&right, &two)]).is_none());
}

#[test]
fn two_square_root_expressions_respect_all_four_conjugate_branches() {
    // t1 = (1 + xz) + (2 - y) sqrt(A) + (x + z) sqrt(B) + (1/3) sqrt(A) sqrt(B).
    let t1 = TwoSquareRootExpression {
        rational: tri(&[(r(1), [0, 0, 0]), (r(1), [1, 0, 1])]),
        first: tri(&[(r(2), [0, 0, 0]), (r(-1), [0, 1, 0])]),
        second: tri(&[(r(1), [1, 0, 0]), (r(1), [0, 0, 1])]),
        product: tri(&[(q(1, 3), [0, 0, 0])]),
    };
    // t2 = (y - 1) + (3 - z) sqrt(A) + xy sqrt(B), assembled from its parts.
    let t2 = TwoSquareRootExpression::from_rational(tri(&[(r(-1), [0, 0, 0]), (r(1), [0, 1, 0])]))
        .unwrap()
        .add(
            &TwoSquareRootExpression::from_first_radical(tri(&[
                (r(3), [0, 0, 0]),
                (r(-1), [0, 0, 1]),
            ]))
            .unwrap(),
        )
        .unwrap()
        .add(&TwoSquareRootExpression::from_second_radical(tri(&[(r(1), [1, 1, 0])])).unwrap())
        .unwrap();
    let weight = tri(&[(r(1), [0, 0, 0]), (r(2), [0, 0, 1])]);
    let (a, b) = (radicand_a(), radicand_b());
    let scale = q(5, 4);
    let sum = t1.combine(&t2, false).unwrap();
    let difference = t1.subtract(&t2).unwrap();
    let scaled = t1.scale(&scale).unwrap();
    let weighted = t1.multiply_rational(&weight).unwrap();
    let product = t1.multiply(&t2, &a, &b).unwrap();
    let square = t1.square(&a, &b).unwrap();
    let norm = t1.second_norm(&a, &b).unwrap();
    for (x, first_root) in a_squares() {
        for (y, second_root) in b_squares() {
            for z in nodes(3) {
                let point = [&x, &y, &z];
                let w = eval3(&weight, point);
                for first_sign in [r(1), r(-1)] {
                    let sa = &first_sign * &first_root;
                    for second_sign in [r(1), r(-1)] {
                        let sb = &second_sign * &second_root;
                        let v1 = value2(&t1, point, &sa, &sb);
                        let v2 = value2(&t2, point, &sa, &sb);
                        assert_equal(&value2(&sum, point, &sa, &sb), &(&v1 + &v2));
                        assert_equal(&value2(&difference, point, &sa, &sb), &(&v1 - &v2));
                        assert_equal(&value2(&scaled, point, &sa, &sb), &(&scale * &v1));
                        assert_equal(&value2(&weighted, point, &sa, &sb), &(&w * &v1));
                        assert_equal(&value2(&product, point, &sa, &sb), &(&v1 * &v2));
                        assert_equal(&value2(&square, point, &sa, &sb), &(&v1 * &v1));
                    }
                    let conjugates = value2(&t1, point, &sa, &second_root)
                        * value2(&t1, point, &sa, &(-second_root.clone()));
                    assert_equal(&value1(&norm, point, &sa), &conjugates);
                }
            }
        }
    }

    let rank_two = DenseTensorPolynomial::from_axis_polynomial(2, 0, &[r(1), r(1)]).unwrap();
    let rank_three = DenseTensorPolynomial::from_axis_polynomial(3, 0, &[r(1), r(1)]).unwrap();
    let left = TwoSquareRootExpression::from_first_radical(rank_two.clone()).unwrap();
    let right = TwoSquareRootExpression::from_second_radical(rank_three.clone()).unwrap();
    assert!(left.add(&right).is_none());
    assert!(left.multiply(&right, &rank_two, &rank_two).is_none());
    assert!(left.square(&rank_two, &rank_three).is_none());
    assert!(left.second_norm(&rank_two, &rank_three).is_none());
    assert!(left.multiply_rational(&rank_three).is_none());
}

fn dense_expression(
    parts: [DenseTensorPolynomial; 4],
) -> TwoSquareRootExpression<DenseTensorPolynomial> {
    let [rational, first, second, product] = parts;
    TwoSquareRootExpression {
        rational,
        first,
        second,
        product,
    }
}

fn dense_value(
    expression: &TwoSquareRootExpression<DenseTensorPolynomial>,
    point: &[Real],
    first_root: &Real,
    second_root: &Real,
) -> Real {
    eval_dense(&expression.rational, point)
        + first_root * eval_dense(&expression.first, point)
        + second_root * eval_dense(&expression.second, point)
        + first_root * second_root * eval_dense(&expression.product, point)
}

/// `t` values where t + 1 and t + 4 are both rational squares, with both
/// roots: (b - a)(b + a) = 3 for each rational choice of b - a.
fn double_squares() -> Vec<(Real, Real, Real)> {
    vec![
        (r(0), r(1), r(2)),
        (q(105, 16), q(11, 4), q(13, 4)),
        (q(160, 9), q(13, 3), q(14, 3)),
        (q(-15, 16), q(1, 4), q(7, 4)),
        (q(385, 144), q(23, 12), q(31, 12)),
        (q(2145, 64), q(47, 8), q(49, 8)),
    ]
}

#[test]
fn dense_radical_arithmetic_matches_conjugate_values() {
    // Univariate parts in t with radicands A = t + 1 and B = t + 4.
    let a = dense(&[2], &[(r(1), &[0]), (r(1), &[1])]);
    let b = dense(&[2], &[(r(4), &[0]), (r(1), &[1])]);
    let e1 = SquareRootExpression {
        rational: dense(&[2], &[(r(1), &[0]), (r(-2), &[1])]),
        radical: dense(&[3], &[(r(1), &[0]), (q(1, 2), &[2])]),
    };
    let e2 = SquareRootExpression {
        rational: dense(&[2], &[(r(3), &[0]), (r(-1), &[1])]),
        radical: dense(&[2], &[(r(1), &[1])]),
    };
    let weight = dense(&[2], &[(r(2), &[0]), (r(1), &[1])]);
    let (scale, minus_one) = (q(-3, 2), r(-1));
    let scaled = e1.scale(&scale).unwrap();
    let combination =
        SquareRootExpression::linear_combination(&[(&e1, &scale), (&e2, &minus_one)]).unwrap();
    let product = e1.multiply(&e2, &a).unwrap();
    let square = e1.square(&a).unwrap();
    let weighted = e1.multiply_rational(&weight).unwrap();
    let norm = e1.projection(&a).unwrap();

    let t1 = dense_expression([
        dense(&[2], &[(r(1), &[0]), (r(1), &[1])]),
        dense(&[1], &[(r(2), &[0])]),
        dense(&[2], &[(r(1), &[1])]),
        dense(&[1], &[(r(-1), &[0])]),
    ]);
    let t2 =
        TwoSquareRootExpression::from_second_radical(dense(&[2], &[(r(1), &[0]), (r(3), &[1])]))
            .unwrap()
            .combine(
                &TwoSquareRootExpression::from_rational(dense(&[1], &[(q(1, 3), &[0])])).unwrap(),
                true,
            )
            .unwrap();
    let two_scaled = t1.scale(&scale).unwrap();
    let two_product = t1.multiply(&t2, &a, &b).unwrap();
    let two_square = t1.square(&a, &b).unwrap();
    let second_norm = t1.second_norm(&a, &b).unwrap();
    let value =
        |expression: &SquareRootExpression<DenseTensorPolynomial>, t: &Real, root: &Real| {
            eval_dense(&expression.rational, std::slice::from_ref(t))
                + root * eval_dense(&expression.radical, std::slice::from_ref(t))
        };
    for (t, first_root, second_root) in double_squares() {
        let point = std::slice::from_ref(&t);
        let w = eval_dense(&weight, point);
        for first_sign in [r(1), r(-1)] {
            let sa = &first_sign * &first_root;
            let (v1, v2) = (value(&e1, &t, &sa), value(&e2, &t, &sa));
            assert_equal(&value(&scaled, &t, &sa), &(&scale * &v1));
            assert_equal(&value(&combination, &t, &sa), &(&scale * &v1 - &v2));
            assert_equal(&value(&product, &t, &sa), &(&v1 * &v2));
            assert_equal(&value(&square, &t, &sa), &(&v1 * &v1));
            assert_equal(&value(&weighted, &t, &sa), &(&w * &v1));
            for second_sign in [r(1), r(-1)] {
                let sb = &second_sign * &second_root;
                let (u1, u2) = (
                    dense_value(&t1, point, &sa, &sb),
                    dense_value(&t2, point, &sa, &sb),
                );
                assert_equal(&dense_value(&two_scaled, point, &sa, &sb), &(&scale * &u1));
                assert_equal(&dense_value(&two_product, point, &sa, &sb), &(&u1 * &u2));
                assert_equal(&dense_value(&two_square, point, &sa, &sb), &(&u1 * &u1));
            }
            let conjugates = dense_value(&t1, point, &sa, &second_root)
                * dense_value(&t1, point, &sa, &(-second_root.clone()));
            assert_equal(&value(&second_norm, &t, &sa), &conjugates);
        }
        let conjugates = value(&e1, &t, &first_root) * value(&e1, &t, &(-first_root.clone()));
        assert_equal(&eval_dense(&norm, point), &conjugates);
    }
}

#[test]
fn dense_two_root_expressions_detect_stored_constants() {
    let zero = DenseTensorPolynomial::zero(vec![2, 2]).unwrap();
    let one = dense(&[2, 2], &[(r(1), &[0, 0])]);
    let constant = dense_expression([one.clone(), zero.clone(), zero.clone(), zero.clone()]);
    assert!(constant.is_stored_one());
    assert!(!constant.is_stored_zero());
    let nothing = dense_expression([zero.clone(), zero.clone(), zero.clone(), zero.clone()]);
    assert!(nothing.is_stored_zero());
    assert!(!nothing.is_stored_one());
    let two = dense_expression([
        dense(&[2, 2], &[(r(2), &[0, 0])]),
        zero.clone(),
        zero.clone(),
        zero.clone(),
    ]);
    assert!(!two.is_stored_one());
    let linear = dense_expression([
        dense(&[2, 2], &[(r(1), &[0, 0]), (q(1, 2), &[1, 0])]),
        zero.clone(),
        zero.clone(),
        zero.clone(),
    ]);
    assert!(!linear.is_stored_one());
    let radical = dense_expression([one.clone(), zero.clone(), one.clone(), zero.clone()]);
    assert!(!radical.is_stored_one());
    assert!(!radical.is_stored_zero());
    // An unproved zero is not a stored exact zero.
    let hidden = dense(&[2, 2], &[(undecided_zero(), &[1, 1])]);
    let hidden_zero = dense_expression([hidden.clone(), zero.clone(), zero.clone(), zero.clone()]);
    assert!(!hidden_zero.is_stored_zero());
    let hidden_one =
        dense_expression([one.add(&hidden).unwrap(), zero.clone(), zero.clone(), zero]);
    assert!(!hidden_one.is_stored_one());
}

#[test]
fn dense_two_root_reductions_preserve_values_at_selected_roots() {
    // Parts over (x, t): source axis x, output axis t.
    let parts = [
        dense(
            &[3, 2],
            &[(r(1), &[0, 0]), (r(2), &[2, 1]), (r(-1), &[1, 0])],
        ),
        dense(&[3, 2], &[(r(3), &[2, 0]), (r(1), &[0, 1])]),
        dense(&[2, 2], &[(q(1, 2), &[1, 1]), (r(-4), &[0, 0])]),
        dense(&[3, 1], &[(r(1), &[2, 0])]),
    ];
    let expression = dense_expression(parts.clone());
    let half = AlgebraicRootRepresentation::from_exact_value(&q(1, 2));
    let reduced = expression.reduced(std::slice::from_ref(&half)).unwrap();
    for (part, original) in [
        &reduced.rational,
        &reduced.first,
        &reduced.second,
        &reduced.product,
    ]
    .into_iter()
    .zip(&parts)
    {
        assert_eq!(part.dimensions()[0], 1);
        for t in nodes(3) {
            assert_equal(
                &eval_dense(part, &[q(1, 2), t.clone()]),
                &eval_dense(original, &[q(1, 2), t]),
            );
        }
    }
    // Over x^2 - 2, values at both conjugates sqrt(2) and -sqrt(2) survive.
    let root_two = Real::from(2).sqrt().unwrap();
    let quadratic = AlgebraicRootRepresentation::from_exact_value(&root_two);
    let reduced = expression
        .reduced(std::slice::from_ref(&quadratic))
        .unwrap();
    for (part, original) in [
        &reduced.rational,
        &reduced.first,
        &reduced.second,
        &reduced.product,
    ]
    .into_iter()
    .zip(&parts)
    {
        assert!(part.dimensions()[0] <= 2);
        for root in [root_two.clone(), -root_two.clone()] {
            for t in nodes(3) {
                assert_zero(
                    &(eval_dense(part, &[root.clone(), t.clone()])
                        - eval_dense(original, &[root.clone(), t])),
                );
            }
        }
    }
    assert!(expression.reduced(&[half.clone(), half.clone()]).is_none());
    assert!(
        expression
            .reduced_at_source_tuple(std::slice::from_ref(&half))
            .is_none()
    );

    // With no output axis, both source axes reduce to their selected values.
    let minus_three = AlgebraicRootRepresentation::from_exact_value(&r(-3));
    let sources = [half.clone(), minus_three];
    let tuple = expression.reduced_at_source_tuple(&sources).unwrap();
    for (part, original) in [&tuple.rational, &tuple.first, &tuple.second, &tuple.product]
        .into_iter()
        .zip(&parts)
    {
        assert_eq!(part.dimensions(), &[1, 1]);
        assert_equal(
            &part.coefficients()[0],
            &eval_dense(original, &[q(1, 2), r(-3)]),
        );
    }
}

#[test]
fn dense_projection_is_the_product_of_every_conjugate_sheet() {
    let half = AlgebraicRootRepresentation::from_exact_value(&q(1, 2));
    let sources = std::slice::from_ref(&half);
    // Over (x, t) with the source root x = 1/2: A = t + 1 + (2x - 1)t^2 and
    // B = t + 4 + 3(x - 1/2) reduce to t + 1 and t + 4.
    let a = dense(
        &[2, 3],
        &[
            (r(1), &[0, 1]),
            (r(1), &[0, 0]),
            (r(2), &[1, 2]),
            (r(-1), &[0, 2]),
        ],
    );
    let b = dense(
        &[2, 2],
        &[(r(1), &[0, 1]), (q(5, 2), &[0, 0]), (r(3), &[1, 0])],
    );
    let samples = double_squares();
    let zero = DenseTensorPolynomial::zero(vec![1, 1]).unwrap();
    let full = dense_expression([
        dense(&[2, 2], &[(r(1), &[0, 0]), (r(1), &[1, 1])]),
        dense(&[1, 2], &[(r(2), &[0, 0]), (r(-1), &[0, 1])]),
        dense(&[3, 1], &[(r(1), &[2, 0]), (r(1), &[0, 0])]),
        dense(&[1, 1], &[(q(1, 3), &[0, 0])]),
    ]);
    let first_only = dense_expression([
        dense(&[2, 2], &[(r(1), &[0, 1]), (r(-1), &[1, 0])]),
        dense(&[1, 2], &[(r(3), &[0, 0]), (r(1), &[0, 1])]),
        zero.clone(),
        zero.clone(),
    ]);
    let rational_only = dense_expression([
        dense(&[2, 2], &[(r(2), &[0, 0]), (r(1), &[1, 1])]),
        zero.clone(),
        zero.clone(),
        zero.clone(),
    ]);
    let second_only = dense_expression([
        zero.clone(),
        zero.clone(),
        dense(&[1, 1], &[(r(1), &[0, 0])]),
        zero,
    ]);
    for (t, first_root, second_root) in samples {
        let point = [q(1, 2), t];
        let conjugates = |expression: &TwoSquareRootExpression<DenseTensorPolynomial>,
                          first_signs: &[i64],
                          second_signs: &[i64]| {
            let mut product = Real::one();
            for first_sign in first_signs {
                for second_sign in second_signs {
                    product *= dense_value(
                        expression,
                        &point,
                        &(r(*first_sign) * &first_root),
                        &(r(*second_sign) * &second_root),
                    );
                }
            }
            product
        };
        let projected = full.projection(&a, &b, sources).unwrap();
        assert_equal(
            &eval_dense(&projected, &point),
            &conjugates(&full, &[1, -1], &[1, -1]),
        );
        // An absent second generator keeps only the two first-generator sheets.
        let projected = first_only.projection(&a, &b, sources).unwrap();
        assert_equal(
            &eval_dense(&projected, &point),
            &conjugates(&first_only, &[1, -1], &[1]),
        );
        // No radical remains: the rational base itself carries the zeros.
        let projected = rational_only.projection(&a, &b, sources).unwrap();
        assert_equal(
            &eval_dense(&projected, &point),
            &conjugates(&rational_only, &[1], &[1]),
        );
        // sqrt(B) alone projects to -B, the product of its two sheets.
        let projected = second_only.projection(&a, &b, sources).unwrap();
        assert_equal(
            &eval_dense(&projected, &point),
            &conjugates(&second_only, &[1], &[1, -1]),
        );
    }
    assert!(full.projection(&a, &b, &[]).is_none());
}

#[test]
fn dense_tensor_axis_helpers_preserve_polynomial_values() {
    // P(x, y) = 1 - 2y + x y^2 + 3x.
    let polynomial = dense(
        &[2, 3],
        &[
            (r(1), &[0, 0]),
            (r(-2), &[0, 1]),
            (r(1), &[1, 2]),
            (r(3), &[1, 0]),
        ],
    );
    let lifted = dense_tensor_with_output_axis(&polynomial).unwrap();
    assert_eq!(lifted.dimensions(), &[2, 3, 1]);
    let cloned = try_clone_dense_tensor(&polynomial).unwrap();
    assert_eq!(cloned.dimensions(), polynomial.dimensions());
    for x in nodes(2) {
        for y in nodes(3) {
            let expected = eval_dense(&polynomial, &[x.clone(), y.clone()]);
            assert_equal(
                &eval_dense(&lifted, &[x.clone(), y.clone(), r(7)]),
                &expected,
            );
            assert_equal(&eval_dense(&cloned, &[x.clone(), y.clone()]), &expected);
        }
    }

    // A ragged grid 1 + 2t + 3s - s^2 t^3 and the empty grid.
    let grid = BivariatePolynomial::new(vec![
        vec![r(1), r(2)],
        vec![r(3)],
        vec![r(0), r(0), r(0), r(-1)],
    ]);
    let flat = bivariate_dense_tensor(&grid).unwrap();
    assert_eq!(flat.dimensions(), &[3, 4]);
    let with_output = bivariate_tensor_with_output_axis(&grid).unwrap();
    assert_eq!(with_output.dimensions(), &[3, 4, 1]);
    for s in nodes(3) {
        for t in nodes(4) {
            let expected = eval2(&grid, &s, &t);
            assert_equal(&eval_dense(&flat, &[s.clone(), t.clone()]), &expected);
            assert_equal(
                &eval_dense(&with_output, &[s.clone(), t.clone(), r(-5)]),
                &expected,
            );
        }
    }
    let empty = bivariate_dense_tensor(&BivariatePolynomial::new(Vec::new())).unwrap();
    assert_eq!(empty.dimensions(), &[1, 1]);
    assert!(dense_tensor_is_stored_zero(&empty));

    // Coefficients and derivative along the last axis of
    // Q(x, t) = (1 + x) + (2 - x) t + 0 t^2 + 5x t^3.
    let fibered = dense(
        &[2, 4],
        &[
            (r(1), &[0, 0]),
            (r(1), &[1, 0]),
            (r(2), &[0, 1]),
            (r(-1), &[1, 1]),
            (r(5), &[1, 3]),
        ],
    );
    let expected_coefficients = |x: &Real, power_index: usize| match power_index {
        0 => r(1) + x,
        1 => r(2) - x,
        3 => r(5) * x,
        _ => r(0),
    };
    for power_index in 0..6 {
        let coefficient = dense_last_axis_coefficient(&fibered, power_index).unwrap();
        assert_eq!(coefficient.dimensions(), &[2, 1]);
        for x in nodes(2) {
            assert_equal(
                &eval_dense(&coefficient, &[x.clone(), r(9)]),
                &expected_coefficients(&x, power_index),
            );
        }
    }
    let derivative = dense_last_axis_derivative(&fibered).unwrap();
    assert_eq!(derivative.dimensions(), &[2, 3]);
    for x in nodes(2) {
        for t in nodes(3) {
            // d/dt Q = (2 - x) + 15x t^2.
            assert_equal(
                &eval_dense(&derivative, &[x.clone(), t.clone()]),
                &(r(2) - &x + r(15) * &x * &t * &t),
            );
        }
        for t in [q(-1, 2), r(3)] {
            let specialized = dense_specialize_last_axis(&fibered, &t).unwrap();
            assert_eq!(specialized.dimensions(), &[2, 1]);
            assert_equal(
                &eval_dense(&specialized, &[x.clone(), r(4)]),
                &eval_dense(&fibered, &[x.clone(), t]),
            );
        }
    }
    let constant_in_t = dense(&[2, 1], &[(r(4), &[1, 0])]);
    let flat_derivative = dense_last_axis_derivative(&constant_in_t).unwrap();
    assert_eq!(flat_derivative.dimensions(), &[2, 1]);
    assert!(dense_tensor_is_stored_zero(&flat_derivative));

    let scalar = DenseTensorPolynomial::try_new(Vec::new(), vec![r(3)]).unwrap();
    assert!(dense_last_axis_coefficient(&scalar, 0).is_none());
    assert!(dense_last_axis_derivative(&scalar).is_none());
    assert!(dense_specialize_last_axis(&scalar, &r(1)).is_none());
    assert!(!dense_tensor_is_stored_zero(&scalar));
    let hidden = dense(&[1, 2], &[(undecided_zero(), &[0, 1])]);
    assert!(!dense_tensor_is_stored_zero(&hidden));
}

#[test]
fn dense_tensor_embedding_and_packing_compose_axes() {
    // P(u, v) = 2 + u v - 3v^2 placed on target axes (2, 0) of rank three.
    let polynomial = dense(
        &[2, 3],
        &[(r(2), &[0, 0]), (r(1), &[1, 1]), (r(-3), &[0, 2])],
    );
    let embedded = dense_tensor_embed_axes(&polynomial, 3, &[2, 0]).unwrap();
    let diagonal = dense_tensor_embed_axes(&polynomial, 2, &[1, 1]).unwrap();
    assert_eq!(diagonal.dimensions(), &[1, 4]);
    for a in nodes(3) {
        for b in nodes(2) {
            for c in nodes(2) {
                assert_equal(
                    &eval_dense(&embedded, &[a.clone(), b.clone(), c.clone()]),
                    &eval_dense(&polynomial, &[c, a.clone()]),
                );
            }
        }
    }
    for v in nodes(4) {
        assert_equal(
            &eval_dense(&diagonal, &[r(6), v.clone()]),
            &eval_dense(&polynomial, &[v.clone(), v]),
        );
    }
    assert!(dense_tensor_embed_axes(&polynomial, 3, &[0]).is_none());
    assert!(dense_tensor_embed_axes(&polynomial, 2, &[0, 2]).is_none());

    // Pack C0 = 1 + x, C1 = x^2 - 2, C2 = 3 as coefficients of t.
    let c0 = dense(&[2], &[(r(1), &[0]), (r(1), &[1])]);
    let c1 = dense(&[3], &[(r(-2), &[0]), (r(1), &[2])]);
    let c2 = dense(&[1], &[(r(3), &[0])]);
    let packed = dense_tensor_from_polynomial_coefficients(&[&c0, &c1, &c2]).unwrap();
    assert_eq!(packed.dimensions(), &[3, 3]);
    for x in nodes(3) {
        for t in nodes(3) {
            let expected = eval_dense(&c0, std::slice::from_ref(&x))
                + &t * eval_dense(&c1, std::slice::from_ref(&x))
                + &t * &t * eval_dense(&c2, std::slice::from_ref(&x));
            assert_equal(&eval_dense(&packed, &[x.clone(), t]), &expected);
        }
    }
    assert!(dense_tensor_from_polynomial_coefficients(&[]).is_none());
    assert!(dense_tensor_from_polynomial_coefficients(&[&c0, &polynomial]).is_none());
}

#[test]
fn selected_root_tensor_reductions_preserve_values_at_the_root() {
    // P(x, y, t) of degree two in each source axis.
    let polynomial = dense(
        &[3, 3, 2],
        &[
            (r(1), &[2, 0, 0]),
            (r(-2), &[1, 2, 1]),
            (q(1, 2), &[0, 1, 0]),
            (r(3), &[2, 2, 1]),
        ],
    );
    let half = AlgebraicRootRepresentation::from_exact_value(&q(1, 2));
    let minus_three = AlgebraicRootRepresentation::from_exact_value(&r(-3));
    let reduced = dense_reduce_selected_root_relations(
        polynomial.clone(),
        &[half.clone(), minus_three.clone()],
    )
    .unwrap();
    assert_eq!(reduced.dimensions(), &[1, 1, 2]);
    for t in nodes(2) {
        assert_equal(
            &eval_dense(&reduced, &[r(0), r(0), t.clone()]),
            &eval_dense(&polynomial, &[q(1, 2), r(-3), t]),
        );
    }
    assert!(
        dense_reduce_selected_root_relations(polynomial.clone(), std::slice::from_ref(&half))
            .is_none()
    );

    // A quadratic source leaves an axis of degree below two unchanged.
    let root_two = AlgebraicRootRepresentation::from_exact_value(&Real::from(2).sqrt().unwrap());
    let small = dense(&[2, 2], &[(r(1), &[1, 0]), (r(4), &[0, 1])]);
    let unchanged =
        dense_reduce_selected_root_relations(small.clone(), std::slice::from_ref(&root_two))
            .unwrap();
    assert_eq!(unchanged, small);

    let pair = dense(
        &[3, 3],
        &[(r(1), &[2, 1]), (r(-1), &[0, 2]), (r(5), &[1, 0])],
    );
    let tuple =
        dense_reduce_selected_tuple_relations(pair.clone(), &[half.clone(), minus_three]).unwrap();
    assert_eq!(tuple.dimensions(), &[1, 1]);
    assert_equal(
        &tuple.coefficients()[0],
        &eval_dense(&pair, &[q(1, 2), r(-3)]),
    );
    assert!(
        dense_reduce_selected_tuple_relations(pair.clone(), std::slice::from_ref(&half)).is_none()
    );

    // A defining polynomial whose leading coefficient has no certified sign
    // admits no division, so neither reduction may claim a remainder.
    let mut undecided = half.clone();
    undecided.polynomial_coefficients = vec![r(-1), r(1), undecided_zero()];
    assert!(
        dense_reduce_selected_tuple_relations(pair.clone(), &[undecided.clone(), half.clone()])
            .is_none()
    );
    assert!(dense_reduce_selected_root_relations(pair, std::slice::from_ref(&undecided)).is_none());
}

#[test]
fn rational_tensor_constants_require_an_exact_rational_value() {
    assert_equal(&rational_tensor_constant(&q(3, 4)).unwrap(), &q(3, 4));
    // sin(e)^2 + cos(e)^2 is not stored as a rational, but its normal form is 1.
    let pythagorean = Real::e().sin() * Real::e().sin() + Real::e().cos() * Real::e().cos();
    assert!(pythagorean.exact_rational().is_none());
    assert_equal(&rational_tensor_constant(&pythagorean).unwrap(), &r(1));
    assert!(rational_tensor_constant(&Real::from(2).sqrt().unwrap()).is_none());
    assert!(rational_tensor_constant(&Real::pi()).is_none());
}

/// Checks `v_a^T M = f(a) v_a^T` for the power vector of a root `a`.
fn assert_eigen_relation(matrix: &[Real], degree: usize, root: &Real, value: &Real) {
    for column in 0..degree {
        let left = (0..degree).fold(Real::zero(), |sum, row| {
            sum + power(root, row) * &matrix[row * degree + column]
        });
        assert_zero(&(left - value * power(root, column)));
    }
}

#[test]
fn quotient_ring_matrices_multiply_by_the_map_at_every_root() {
    // (x - 1)(x + 2)(2x - 3) = 2x^3 - x^2 - 7x + 6 with three rational roots.
    let source = [r(6), r(-7), r(-1), r(2)];
    let roots = [r(1), r(-2), q(3, 2)];
    for (numerator, denominator) in [
        (vec![r(1), r(0), r(1)], vec![r(3), r(1)]),
        (vec![r(0), r(0), r(0), r(0), r(1)], vec![q(1, 2)]),
    ] {
        let matrices =
            quotient_ring_rational_map_matrices(&source, &numerator, &denominator).unwrap();
        assert_eq!(matrices.degree, 3);
        assert_eq!(matrices.numerator.len(), 9);
        for root in &roots {
            assert_eigen_relation(&matrices.numerator, 3, root, &eval1(&numerator, root));
            assert_eigen_relation(&matrices.denominator, 3, root, &eval1(&denominator, root));
        }
    }
    // An irrational leading coefficient is inverted once: sqrt(2)(x - 1)(x - 2).
    let root_two = Real::from(2).sqrt().unwrap();
    let scaled = [r(2) * &root_two, r(-3) * &root_two, root_two.clone()];
    let numerator = [r(-1), r(0), r(0), r(1)];
    let matrices = quotient_ring_rational_map_matrices(&scaled, &numerator, &[r(1)]).unwrap();
    for root in [r(1), r(2)] {
        assert_eigen_relation(&matrices.numerator, 2, &root, &eval1(&numerator, &root));
        assert_eigen_relation(&matrices.denominator, 2, &root, &r(1));
    }

    let unit = [r(1)];
    assert!(quotient_ring_rational_map_matrices(&[r(1)], &unit, &unit).is_none());
    assert!(quotient_ring_rational_map_matrices(&[], &unit, &unit).is_none());
    assert!(quotient_ring_rational_map_matrices(&source, &[], &unit).is_none());
    assert!(quotient_ring_rational_map_matrices(&source, &unit, &[]).is_none());
    let mut thirteenth = vec![r(0); 14];
    thirteenth[0] = r(-1);
    thirteenth[13] = r(1);
    assert!(quotient_ring_rational_map_matrices(&thirteenth, &unit, &unit).is_none());
    let twelfth = &thirteenth[1..];
    assert_eq!(
        quotient_ring_rational_map_matrices(twelfth, &unit, &unit)
            .unwrap()
            .degree,
        12
    );
    // A vanishing leading coefficient defines no quotient ring.
    assert!(
        quotient_ring_rational_map_matrices(&[r(1), r(2), r(0)], &[r(0), r(0), r(1)], &unit)
            .is_none()
    );
}

fn binomial(n: usize, k: usize) -> Rational {
    (0..k).fold(Rational::one(), |value, index| {
        value * Rational::new((n - index) as i64) / Rational::new((index + 1) as i64)
    })
}

fn real_sign(value: &Real) -> RealSign {
    match strict_sign(value).expect("oracle signs are certified") {
        hypersolve::PredicateSign::Negative => RealSign::Negative,
        hypersolve::PredicateSign::Zero => RealSign::Zero,
        hypersolve::PredicateSign::Positive => RealSign::Positive,
    }
}

/// Signs of the Bernstein coefficients on [0, 1], and of the leading power,
/// of prod_i (f_i - (lower + span t) g_i) for eigenvalue pairs (f_i, g_i).
fn expected_bernstein_signs(
    eigenvalues: &[(Real, Real)],
    lower: &Real,
    upper: &Real,
) -> (Vec<RealSign>, RealSign) {
    let span = upper - lower;
    let mut power_basis = vec![Real::one()];
    for (numerator, denominator) in eigenvalues {
        let factor = [numerator - lower * denominator, -(&span * denominator)];
        let mut next = vec![Real::zero(); power_basis.len() + 1];
        for (k, coefficient) in power_basis.iter().enumerate() {
            next[k] += coefficient * &factor[0];
            next[k + 1] += coefficient * &factor[1];
        }
        power_basis = next;
    }
    let degree = eigenvalues.len();
    let signs = (0..=degree)
        .map(|index| {
            let coefficient = (0..=index).fold(Real::zero(), |sum, k| {
                sum + &power_basis[k] * Real::new(binomial(index, k) / binomial(degree, k))
            });
            real_sign(&coefficient)
        })
        .collect();
    (signs, real_sign(power_basis.last().unwrap()))
}

#[test]
fn determinant_bernstein_signs_match_the_eigenvalue_product() {
    // Over x^2 - 2 with y = x: det(N - yD) = (sqrt 2 - y)(-sqrt 2 - y) = y^2 - 2.
    let matrices =
        quotient_ring_rational_map_matrices(&[r(-2), r(0), r(1)], &[r(0), r(1)], &[r(1)]).unwrap();
    let root_two = Real::from(2).sqrt().unwrap();
    let eigenvalues = [(root_two.clone(), r(1)), (-root_two.clone(), r(1))];
    for (lower, upper) in [(1, 2), (-2, 2), (0, 1), (2, 3)] {
        let (bounds_lower, bounds_upper) = (Rational::new(lower), Rational::new(upper));
        let found = determinant_local_bernstein_signs_from_enclosures(
            &matrices,
            &bounds_lower,
            &bounds_upper,
            -16,
        )
        .unwrap();
        assert_eq!(
            found,
            expected_bernstein_signs(&eigenvalues, &r(lower), &r(upper))
        );
    }
    // On [1, 2], t^2 + 2t - 1 has the exact middle control -1 + 2/2 = 0.
    let (signs, _) = determinant_local_bernstein_signs_from_enclosures(
        &matrices,
        &Rational::new(1),
        &Rational::new(2),
        -16,
    )
    .unwrap();
    assert_eq!(signs[1], RealSign::Zero);

    // y = (x^2 + 1) / (x + 3) over the roots 1, -2 and 3/2 of the source.
    let source = [r(6), r(-7), r(-1), r(2)];
    let (numerator, denominator) = ([r(1), r(0), r(1)], [r(3), r(1)]);
    let matrices = quotient_ring_rational_map_matrices(&source, &numerator, &denominator).unwrap();
    let eigenvalues =
        [r(1), r(-2), q(3, 2)].map(|root| (eval1(&numerator, &root), eval1(&denominator, &root)));
    for (lower, upper) in [((0, 1), (1, 1)), ((1, 3), (3, 4)), ((-5, 1), (6, 1))] {
        let bounds_lower = Rational::fraction(lower.0, lower.1).unwrap();
        let bounds_upper = Rational::fraction(upper.0, upper.1).unwrap();
        let found = determinant_local_bernstein_signs_from_enclosures(
            &matrices,
            &bounds_lower,
            &bounds_upper,
            -16,
        )
        .unwrap();
        assert_eq!(
            found,
            expected_bernstein_signs(
                &eigenvalues,
                &Real::new(bounds_lower),
                &Real::new(bounds_upper)
            )
        );
    }

    // Irrational entries: y = (sqrt5 - 2) + sqrt3 x over x^2 - 2, divided by
    // 1 - (sqrt2/10) x. Fine enclosures certify the true signs; coarse ones
    // cannot exclude zero and must make no claim.
    let root_three = Real::from(3).sqrt().unwrap();
    let root_five = Real::from(5).sqrt().unwrap();
    let numerator = [&root_five - r(2), root_three.clone()];
    let denominator = [r(1), -(&root_two / r(10)).unwrap()];
    let matrices =
        quotient_ring_rational_map_matrices(&[r(-2), r(0), r(1)], &numerator, &denominator)
            .unwrap();
    let eigenvalues = [root_two.clone(), -root_two.clone()].map(|root| {
        (
            &numerator[0] + &numerator[1] * &root,
            &denominator[0] + &denominator[1] * &root,
        )
    });
    for (lower, upper) in [((-3, 1), (3, 1)), ((0, 1), (1, 1)), ((-1, 2), (5, 2))] {
        let bounds_lower = Rational::fraction(lower.0, lower.1).unwrap();
        let bounds_upper = Rational::fraction(upper.0, upper.1).unwrap();
        let expected = expected_bernstein_signs(
            &eigenvalues,
            &Real::new(bounds_lower.clone()),
            &Real::new(bounds_upper.clone()),
        );
        for precision in [-60, -12, -4, 0] {
            if let Some(found) = determinant_local_bernstein_signs_from_enclosures(
                &matrices,
                &bounds_lower,
                &bounds_upper,
                precision,
            ) {
                assert_eq!(found, expected);
            }
        }
        assert_eq!(
            determinant_local_bernstein_signs_from_enclosures(
                &matrices,
                &bounds_lower,
                &bounds_upper,
                -60
            ),
            Some(expected)
        );
        assert!(
            determinant_local_bernstein_signs_from_enclosures(
                &matrices,
                &bounds_lower,
                &bounds_upper,
                8
            )
            .is_none()
        );
    }

    // A denominator divisible by the source vanishes at every root: the
    // determinant does not depend on y and no leading sign exists.
    let matrices = quotient_ring_rational_map_matrices(
        &[r(-2), r(0), r(1)],
        &[r(0), r(1)],
        &[r(-2), r(0), r(1)],
    )
    .unwrap();
    assert!(
        determinant_local_bernstein_signs_from_enclosures(
            &matrices,
            &Rational::new(0),
            &Rational::new(1),
            -16
        )
        .is_none()
    );
}
