//! Exact univariate resultant reports.
//!
//! Resultants are the elimination step that lets a curve or solver pipeline
//! remove one variable before root isolation. This module starts with the
//! classical Sylvester-matrix determinant: the determinant is computed by
//! [`crate::determinant_bareiss`], so coefficient arithmetic remains exact and
//! pivot decisions remain certified. See Sylvester, "On a Theory of the
//! Syzygetic Relations of Two Rational Integral Functions" (1853), Bareiss,
//! "Sylvester's Identity and Multistep Integer-Preserving Gaussian
//! Elimination," *Mathematics of Computation* 22.103 (1968), and Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997).

use hyperreal::{CertifiedRealSign, Real, RealSign};

use crate::bareiss::{BareissDeterminantReport, BareissError, determinant_bareiss};

/// Failure mode for exact univariate resultant construction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum UnivariateResultantError {
    /// One of the input coefficient slices was empty.
    EmptyPolynomial,
    /// A coefficient needed for degree trimming could not be certified zero or
    /// nonzero within the requested bound.
    UndecidedCoefficient {
        /// Input side whose coefficient was undecided.
        side: ResultantInputSide,
        /// Coefficient index in ascending power order.
        index: usize,
    },
    /// The Sylvester determinant could not be computed exactly.
    Determinant(BareissError),
}

/// Identifies the input polynomial that produced a resultant error.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResultantInputSide {
    /// Left-hand polynomial.
    Left,
    /// Right-hand polynomial.
    Right,
}

/// Exact Sylvester-resultant report for two univariate polynomials.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateResultantReport {
    /// Trimmed left polynomial coefficients in ascending power order.
    pub left_coefficients: Vec<Real>,
    /// Trimmed right polynomial coefficients in ascending power order.
    pub right_coefficients: Vec<Real>,
    /// Degree of the left polynomial after certified trimming.
    pub left_degree: usize,
    /// Degree of the right polynomial after certified trimming.
    pub right_degree: usize,
    /// Size of the square Sylvester matrix.
    pub sylvester_dimension: usize,
    /// Exact resultant value.
    pub resultant: Real,
    /// Determinant report for nonconstant/nonconstant inputs.
    pub determinant: Option<BareissDeterminantReport>,
}

/// Computes the exact Sylvester resultant of two univariate polynomials.
///
/// Coefficients are supplied in ascending power order. Constant inputs use the
/// standard resultant convention `Res(f, c) = c^deg(f)` and
/// `Res(c, g) = c^deg(g)`, avoiding an artificial zero-size determinant. For
/// two nonconstant inputs, the Sylvester matrix is built exactly and evaluated
/// through Bareiss fraction-free elimination.
pub fn resultant_univariate_polynomials(
    left: &[Real],
    right: &[Real],
    min_precision: i32,
) -> Result<UnivariateResultantReport, UnivariateResultantError> {
    if left.is_empty() || right.is_empty() {
        return Err(UnivariateResultantError::EmptyPolynomial);
    }

    let left = trim_coefficients(left, ResultantInputSide::Left, min_precision)?;
    let right = trim_coefficients(right, ResultantInputSide::Right, min_precision)?;
    let left_degree = left.len() - 1;
    let right_degree = right.len() - 1;

    if left_degree == 0 && right_degree == 0 {
        return Ok(resultant_report(left, right, Real::one(), None));
    }
    if right_degree == 0 {
        let resultant = real_pow(&right[0], left_degree);
        return Ok(resultant_report(left, right, resultant, None));
    }
    if left_degree == 0 {
        let resultant = real_pow(&left[0], right_degree);
        return Ok(resultant_report(left, right, resultant, None));
    }

    let sylvester = sylvester_matrix(&left, &right);
    let determinant = determinant_bareiss(&sylvester, min_precision)
        .map_err(UnivariateResultantError::Determinant)?;
    Ok(resultant_report(
        left,
        right,
        determinant.determinant.clone(),
        Some(determinant),
    ))
}

fn resultant_report(
    left_coefficients: Vec<Real>,
    right_coefficients: Vec<Real>,
    resultant: Real,
    determinant: Option<BareissDeterminantReport>,
) -> UnivariateResultantReport {
    let left_degree = left_coefficients.len() - 1;
    let right_degree = right_coefficients.len() - 1;
    UnivariateResultantReport {
        left_coefficients,
        right_coefficients,
        left_degree,
        right_degree,
        sylvester_dimension: left_degree + right_degree,
        resultant,
        determinant,
    }
}

fn trim_coefficients(
    coefficients: &[Real],
    side: ResultantInputSide,
    min_precision: i32,
) -> Result<Vec<Real>, UnivariateResultantError> {
    let mut last_nonzero = None;
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => {
                last_nonzero = Some(index);
                break;
            }
            CertifiedRealSign::Unknown { .. } => {
                return Err(UnivariateResultantError::UndecidedCoefficient { side, index });
            }
        }
    }

    let Some(last_nonzero) = last_nonzero else {
        return Ok(vec![Real::zero()]);
    };
    Ok(coefficients[..=last_nonzero].to_vec())
}

fn sylvester_matrix(left: &[Real], right: &[Real]) -> Vec<Vec<Real>> {
    let left_degree = left.len() - 1;
    let right_degree = right.len() - 1;
    let dimension = left_degree + right_degree;
    let mut matrix = vec![vec![Real::zero(); dimension]; dimension];

    for row in 0..right_degree {
        for (degree, coefficient) in left.iter().enumerate() {
            matrix[row][row + left_degree - degree] = coefficient.clone();
        }
    }
    for row in 0..left_degree {
        for (degree, coefficient) in right.iter().enumerate() {
            matrix[right_degree + row][row + right_degree - degree] = coefficient.clone();
        }
    }

    matrix
}

fn real_pow(value: &Real, exponent: usize) -> Real {
    let mut result = Real::one();
    for _ in 0..exponent {
        result = result * value.clone();
    }
    result
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::*;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    #[test]
    fn resultant_reports_common_linear_root_as_zero() {
        let report = resultant_univariate_polynomials(
            &[real(-1), real(1)],
            &[real(-1), real(0), real(1)],
            -64,
        )
        .unwrap();

        assert_eq!(report.left_degree, 1);
        assert_eq!(report.right_degree, 2);
        assert_eq!(report.sylvester_dimension, 3);
        assert_eq!(report.resultant, Real::zero());
        assert!(report.determinant.is_some());
    }

    #[test]
    fn resultant_of_linear_polynomials_is_exact_difference() {
        let report =
            resultant_univariate_polynomials(&[real(-2), real(1)], &[real(-5), real(1)], -64)
                .unwrap();

        assert_eq!(report.resultant, real(-3));
    }

    #[test]
    fn resultant_validates_empty_and_zero_polynomials() {
        assert_eq!(
            resultant_univariate_polynomials(&[], &[real(1)], -64).unwrap_err(),
            UnivariateResultantError::EmptyPolynomial
        );

        let report =
            resultant_univariate_polynomials(&[Real::zero()], &[real(-3), real(1)], -64).unwrap();
        assert_eq!(report.left_degree, 0);
        assert_eq!(report.resultant, Real::zero());
        assert!(report.determinant.is_none());
    }

    proptest! {
        #[test]
        fn generated_linear_resultants_match_root_difference(
            left_root in -64_i16..=64,
            right_root in -64_i16..=64,
        ) {
            let left_root = i64::from(left_root);
            let right_root = i64::from(right_root);
            let report = resultant_univariate_polynomials(
                &[real(-left_root), real(1)],
                &[real(-right_root), real(1)],
                -64,
            ).unwrap();

            prop_assert_eq!(report.resultant, real(left_root - right_root));
        }

        #[test]
        fn generated_common_linear_factor_resultant_is_zero(
            shared_root in -32_i16..=32,
            other_root in -32_i16..=32,
        ) {
            let shared_root = i64::from(shared_root);
            let other_root = i64::from(other_root);
            let quadratic = vec![
                real(shared_root * other_root),
                real(-(shared_root + other_root)),
                real(1),
            ];
            let report = resultant_univariate_polynomials(
                &[real(-shared_root), real(1)],
                &quadratic,
                -64,
            ).unwrap();

            prop_assert_eq!(report.resultant, Real::zero());
        }
    }
}
