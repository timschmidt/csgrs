//! Exact arithmetic on bivariate parameter polynomials.
//!
//! Coefficient grids follow [`BivariatePolynomial`]: `coefficients[i][j]`
//! multiplies `s^i * t^j`, and rows may be ragged. Every operation is exact
//! and performs no sign decisions.

use hyperreal::Real;

use crate::curve_resultant::BivariatePolynomial;

/// Swaps the two parameters of a bivariate polynomial.
pub fn bivariate_swap_parameters(polynomial: &BivariatePolynomial) -> BivariatePolynomial {
    let first_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let second_count = polynomial.coefficients.len();
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            coefficients[second][first] = coefficient.clone();
        }
    }
    BivariatePolynomial::new(coefficients)
}

/// Returns `polynomial^0, ..., polynomial^maximum` in ascending power coefficients.
pub fn polynomial_powers(polynomial: &[Real], maximum: usize) -> Vec<Vec<Real>> {
    let mut powers = Vec::with_capacity(maximum + 1);
    powers.push(vec![Real::one()]);
    for power in 1..=maximum {
        powers.push(polynomial_multiply(&powers[power - 1], polynomial));
    }
    powers
}

/// Substitutes an exact value for the first parameter, leaving a polynomial in the second.
pub fn bivariate_specialize_first(polynomial: &BivariatePolynomial, value: &Real) -> Vec<Real> {
    let second_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    (0..second_count)
        .map(|second_power| {
            if polynomial.coefficients.len() == 3 && value.exact_rational_ref().is_none() {
                let coefficients: [Real; 3] = std::array::from_fn(|first_power| {
                    polynomial.coefficients[first_power]
                        .get(second_power)
                        .cloned()
                        .unwrap_or_else(Real::zero)
                });
                if coefficients
                    .iter()
                    .any(|coefficient| coefficient.exact_rational_ref().is_none())
                {
                    return Real::eval_poly(&coefficients, value);
                }
            }
            polynomial
                .coefficients
                .iter()
                .rev()
                .fold(Real::zero(), |accumulator, row| {
                    accumulator * value + row.get(second_power).cloned().unwrap_or_else(Real::zero)
                })
        })
        .collect()
}

/// Substitutes an exact value for the second parameter, leaving a polynomial in the first.
pub fn bivariate_specialize_second(polynomial: &BivariatePolynomial, value: &Real) -> Vec<Real> {
    polynomial
        .coefficients
        .iter()
        .map(|row| Real::eval_poly(row, value))
        .collect()
}

/// Restricts a bivariate polynomial to the diagonal `second = first`.
pub fn bivariate_substitute_second_equal_first(polynomial: &BivariatePolynomial) -> Vec<Real> {
    let degree = polynomial
        .coefficients
        .iter()
        .enumerate()
        .flat_map(|(first, row)| {
            row.iter()
                .enumerate()
                .map(move |(second, _)| first + second)
        })
        .max()
        .unwrap_or(0);
    let mut coefficients = vec![Real::zero(); degree + 1];
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            coefficients[first + second] += coefficient;
        }
    }
    coefficients
}

/// Restricts a bivariate polynomial to `second = 1 - first`.
pub fn bivariate_substitute_second_equal_one_minus_first(
    polynomial: &BivariatePolynomial,
) -> Vec<Real> {
    let second_degree = polynomial
        .coefficients
        .iter()
        .map(|row| row.len().saturating_sub(1))
        .max()
        .unwrap_or(0);
    let mut complement_powers = Vec::with_capacity(second_degree + 1);
    complement_powers.push(vec![Real::one()]);
    for degree in 1..=second_degree {
        complement_powers.push(polynomial_multiply(
            &complement_powers[degree - 1],
            &[Real::one(), Real::from(-1_i8)],
        ));
    }
    let degree = polynomial.coefficients.len().saturating_sub(1) + second_degree;
    let mut coefficients = vec![Real::zero(); degree + 1];
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            for (power, factor) in complement_powers[second].iter().enumerate() {
                coefficients[first + power] += coefficient * factor;
            }
        }
    }
    coefficients
}

/// Restricts a bivariate polynomial to `second = offset + scale * first`.
pub fn bivariate_substitute_second_equal_affine_first(
    polynomial: &BivariatePolynomial,
    scale: &Real,
    offset: &Real,
) -> Vec<Real> {
    let second_degree = polynomial
        .coefficients
        .iter()
        .map(|row| row.len().saturating_sub(1))
        .max()
        .unwrap_or(0);
    let affine_powers = polynomial_powers(&[offset.clone(), scale.clone()], second_degree);
    let degree = polynomial.coefficients.len().saturating_sub(1) + second_degree;
    let mut coefficients = vec![Real::zero(); degree + 1];
    for (first, row) in polynomial.coefficients.iter().enumerate() {
        for (second, coefficient) in row.iter().enumerate() {
            for (power, factor) in affine_powers[second].iter().enumerate() {
                coefficients[first + power] += coefficient * factor;
            }
        }
    }
    coefficients
}

/// Returns `first_left(s) * second_left(t) - first_right(s) * second_right(t)`.
pub fn bivariate_parameter_difference(
    first_left: &[Real],
    second_left: &[Real],
    first_right: &[Real],
    second_right: &[Real],
) -> BivariatePolynomial {
    bivariate_subtract(
        &bivariate_outer_product(first_left, second_left),
        &bivariate_outer_product(first_right, second_right),
    )
}

/// Returns the separable product `first(s) * second(t)`.
pub fn bivariate_outer_product(first: &[Real], second: &[Real]) -> BivariatePolynomial {
    let mut coefficients = vec![vec![Real::zero(); second.len()]; first.len()];
    for (first_power, first_coefficient) in first.iter().enumerate() {
        for (second_power, second_coefficient) in second.iter().enumerate() {
            coefficients[first_power][second_power] = first_coefficient * second_coefficient;
        }
    }
    BivariatePolynomial::new(coefficients)
}

/// Exact bivariate sum.
pub fn bivariate_add(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> BivariatePolynomial {
    bivariate_combine(first, second, false)
}

/// Exact bivariate difference.
pub fn bivariate_subtract(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> BivariatePolynomial {
    bivariate_combine(first, second, true)
}

/// Exact bivariate sum or difference of two coefficient grids.
fn bivariate_combine(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
    subtract_second: bool,
) -> BivariatePolynomial {
    let first_count = first.coefficients.len().max(second.coefficients.len());
    let second_count = first
        .coefficients
        .iter()
        .chain(&second.coefficients)
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (target, source) in coefficients.iter_mut().zip(&first.coefficients) {
        for (target, source) in target.iter_mut().zip(source) {
            *target += source;
        }
    }
    for (target, source) in coefficients.iter_mut().zip(&second.coefficients) {
        for (target, source) in target.iter_mut().zip(source) {
            if subtract_second {
                *target -= source;
            } else {
                *target += source;
            }
        }
    }
    BivariatePolynomial::new(coefficients)
}

/// Returns `first * first_scale - second * second_scale`.
pub fn bivariate_scaled_difference(
    first: &BivariatePolynomial,
    first_scale: &Real,
    second: &BivariatePolynomial,
    second_scale: &Real,
) -> BivariatePolynomial {
    let first_count = first.coefficients.len().max(second.coefficients.len());
    let second_count = first
        .coefficients
        .iter()
        .chain(&second.coefficients)
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (target, source) in coefficients.iter_mut().zip(&first.coefficients) {
        for (target, source) in target.iter_mut().zip(source) {
            *target += source * first_scale;
        }
    }
    for (target, source) in coefficients.iter_mut().zip(&second.coefficients) {
        for (target, source) in target.iter_mut().zip(source) {
            *target -= source * second_scale;
        }
    }
    BivariatePolynomial::new(coefficients)
}

/// Multiplies by a univariate polynomial in the first parameter.
pub fn bivariate_multiply_first_parameter(
    polynomial: &BivariatePolynomial,
    factor: &[Real],
) -> BivariatePolynomial {
    if factor.is_empty() || polynomial.coefficients.iter().all(Vec::is_empty) {
        return BivariatePolynomial::new(Vec::new());
    }
    let first_count = polynomial.coefficients.len() + factor.len() - 1;
    let second_count = polynomial
        .coefficients
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0);
    let mut coefficients = vec![vec![Real::zero(); second_count]; first_count];
    for (first_power, row) in polynomial.coefficients.iter().enumerate() {
        for (factor_power, factor) in factor.iter().enumerate() {
            for (second_power, coefficient) in row.iter().enumerate() {
                coefficients[first_power + factor_power][second_power] += coefficient * factor;
            }
        }
    }
    BivariatePolynomial::new(coefficients)
}

/// Exact bivariate product.
pub fn bivariate_multiply(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> BivariatePolynomial {
    let first_second_count = first.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    let second_second_count = second.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    if first_second_count == 0 || second_second_count == 0 {
        return BivariatePolynomial::new(Vec::new());
    }
    let mut coefficients = vec![
        vec![Real::zero(); first_second_count + second_second_count - 1];
        first.coefficients.len() + second.coefficients.len() - 1
    ];
    for (first_power, first_row) in first.coefficients.iter().enumerate() {
        for (second_power, second_row) in second.coefficients.iter().enumerate() {
            for (first_column, first_coefficient) in first_row.iter().enumerate() {
                for (second_column, second_coefficient) in second_row.iter().enumerate() {
                    coefficients[first_power + second_power][first_column + second_column] +=
                        first_coefficient * second_coefficient;
                }
            }
        }
    }
    BivariatePolynomial::new(coefficients)
}

/// Exact bivariate product, or `None` if the result size overflows or cannot be allocated.
pub fn try_bivariate_multiply(
    first: &BivariatePolynomial,
    second: &BivariatePolynomial,
) -> Option<BivariatePolynomial> {
    let first_second_count = first.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    let second_second_count = second.coefficients.iter().map(Vec::len).max().unwrap_or(0);
    if first_second_count == 0 || second_second_count == 0 {
        return Some(BivariatePolynomial::new(Vec::new()));
    }
    let first_count = first
        .coefficients
        .len()
        .checked_add(second.coefficients.len())?
        .checked_sub(1)?;
    let second_count = first_second_count
        .checked_add(second_second_count)?
        .checked_sub(1)?;
    let mut coefficients = try_zero_bivariate_coefficients(first_count, second_count)?;
    for (first_power, first_row) in first.coefficients.iter().enumerate() {
        for (second_power, second_row) in second.coefficients.iter().enumerate() {
            for (first_column, first_coefficient) in first_row.iter().enumerate() {
                for (second_column, second_coefficient) in second_row.iter().enumerate() {
                    coefficients[first_power + second_power][first_column + second_column] +=
                        first_coefficient * second_coefficient;
                }
            }
        }
    }
    Some(BivariatePolynomial::new(coefficients))
}

/// Multiplies every coefficient by one exact scalar.
pub fn bivariate_scale(mut polynomial: BivariatePolynomial, scale: &Real) -> BivariatePolynomial {
    for coefficient in polynomial.coefficients.iter_mut().flatten() {
        *coefficient *= scale;
    }
    polynomial
}

/// Exact univariate product in ascending power coefficients.
pub fn polynomial_multiply(first: &[Real], second: &[Real]) -> Vec<Real> {
    if first.is_empty() || second.is_empty() {
        return Vec::new();
    }
    let mut product = vec![Real::zero(); first.len() + second.len() - 1];
    for (first_degree, first_coefficient) in first.iter().enumerate() {
        for (second_degree, second_coefficient) in second.iter().enumerate() {
            let term = first_coefficient * second_coefficient;
            product[first_degree + second_degree] = &product[first_degree + second_degree] + term;
        }
    }
    product
}

/// Allocates a zero coefficient grid, or `None` if its size overflows or the
/// allocation fails.
pub fn try_zero_bivariate_coefficients(
    first_count: usize,
    second_count: usize,
) -> Option<Vec<Vec<Real>>> {
    first_count.checked_mul(second_count)?;
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(first_count).ok()?;
    for _ in 0..first_count {
        let mut row = Vec::new();
        row.try_reserve_exact(second_count).ok()?;
        row.resize_with(second_count, Real::zero);
        coefficients.push(row);
    }
    Some(coefficients)
}

/// Exact univariate sum in ascending power coefficients.
pub fn polynomial_add(first: &[Real], second: &[Real]) -> Vec<Real> {
    let length = first.len().max(second.len());
    (0..length)
        .map(|index| {
            first.get(index).cloned().unwrap_or_else(Real::zero)
                + second.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect()
}

/// Exact univariate difference in ascending power coefficients.
pub fn polynomial_subtract(first: &[Real], second: &[Real]) -> Vec<Real> {
    let length = first.len().max(second.len());
    (0..length)
        .map(|index| {
            first.get(index).cloned().unwrap_or_else(Real::zero)
                - second.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect()
}

/// Multiplies every univariate coefficient by one exact scalar.
pub fn polynomial_scale(coefficients: &[Real], scale: &Real) -> Vec<Real> {
    coefficients
        .iter()
        .map(|coefficient| coefficient * scale)
        .collect()
}

/// Exact univariate power by repeated multiplication.
pub fn polynomial_power(coefficients: &[Real], exponent: usize) -> Vec<Real> {
    let mut result = vec![Real::one()];
    for _ in 0..exponent {
        result = polynomial_multiply(&result, coefficients);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    fn r(value: i64) -> Real {
        Real::from(value)
    }

    fn evaluate(polynomial: &BivariatePolynomial, s: &Real, t: &Real) -> Real {
        Real::eval_poly(&bivariate_specialize_second(polynomial, t), s)
    }

    #[test]
    fn arithmetic_matches_pointwise_evaluation() {
        // p = 1 + 2s + 3t + 4st, q = s - t^2.
        let p = BivariatePolynomial::new(vec![vec![r(1), r(3)], vec![r(2), r(4)]]);
        let q = BivariatePolynomial::new(vec![vec![r(0), r(0), r(-1)], vec![r(1)]]);
        for (s, t) in [(r(0), r(0)), (r(2), r(-3)), (r(-1), r(5))] {
            let (ps, qs) = (evaluate(&p, &s, &t), evaluate(&q, &s, &t));
            assert_eq!(evaluate(&bivariate_add(&p, &q), &s, &t), &ps + &qs);
            assert_eq!(evaluate(&bivariate_subtract(&p, &q), &s, &t), &ps - &qs);
            assert_eq!(evaluate(&bivariate_multiply(&p, &q), &s, &t), &ps * &qs);
            assert_eq!(
                evaluate(&try_bivariate_multiply(&p, &q).unwrap(), &s, &t),
                &ps * &qs
            );
            assert_eq!(
                evaluate(&bivariate_scale(p.clone(), &r(7)), &s, &t),
                &ps * r(7)
            );
            assert_eq!(
                evaluate(&bivariate_scaled_difference(&p, &r(2), &q, &r(3)), &s, &t),
                &ps * r(2) - &qs * r(3)
            );
            assert_eq!(evaluate(&bivariate_swap_parameters(&p), &t, &s), ps.clone());
            assert_eq!(
                Real::eval_poly(&bivariate_specialize_first(&p, &s), &t),
                ps.clone()
            );
            let factor = [r(1), r(-2)];
            assert_eq!(
                evaluate(&bivariate_multiply_first_parameter(&p, &factor), &s, &t),
                &ps * Real::eval_poly(&factor, &s)
            );
        }
    }

    #[test]
    fn substitutions_and_products_are_exact() {
        let p = BivariatePolynomial::new(vec![vec![r(1), r(3)], vec![r(2), r(4)]]);
        let s = r(3);
        let diagonal = bivariate_substitute_second_equal_first(&p);
        assert_eq!(Real::eval_poly(&diagonal, &s), evaluate(&p, &s, &s));
        let complement = bivariate_substitute_second_equal_one_minus_first(&p);
        assert_eq!(
            Real::eval_poly(&complement, &s),
            evaluate(&p, &s, &(Real::one() - &s))
        );
        let affine = bivariate_substitute_second_equal_affine_first(&p, &r(2), &r(-1));
        assert_eq!(
            Real::eval_poly(&affine, &s),
            evaluate(&p, &s, &(r(2) * &s - r(1)))
        );
        let outer = bivariate_outer_product(&[r(1), r(2)], &[r(3), r(-1)]);
        assert_eq!(evaluate(&outer, &r(2), &r(5)), r(5) * r(-2));
        let difference =
            bivariate_parameter_difference(&[r(1), r(1)], &[r(2)], &[r(0), r(2)], &[r(1)]);
        assert_eq!(evaluate(&difference, &r(4), &r(9)), r(10) - r(8));
        assert_eq!(
            polynomial_powers(&[r(1), r(1)], 2)[2],
            vec![r(1), r(2), r(1)]
        );
        assert_eq!(
            polynomial_multiply(&[r(1), r(1)], &[r(-1), r(1)]),
            vec![r(-1), r(0), r(1)]
        );
    }
}
