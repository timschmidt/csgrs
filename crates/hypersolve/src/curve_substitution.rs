//! Exact curve-substitution carriers for resultant-based handoffs.
//!
//! This module keeps curve-specific algebraic rewrites explicit before a
//! resultant is built. Bernstein control nets are converted to power basis by
//! the exact Bernstein expansion used in CAGD texts such as Farin, *Curves and
//! Surfaces for CAGD* (2002). Rational parametric curves retain homogeneous
//! numerator/weight polynomials so intersection code can cross-multiply
//! equations instead of dividing by a possibly vanishing weight. This is the
//! Yap EGC boundary: preserve structure and report the algebraic transform,
//! then let downstream geometry replay decide topology.

use hyperreal::Real;

use crate::curve_resultant::PolynomialParametricCurve2;

const MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE: usize = 32;

/// Exact 2D control point used by curve substitution helpers.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialCurvePoint2 {
    /// x-coordinate.
    pub x: Real,
    /// y-coordinate.
    pub y: Real,
}

impl PolynomialCurvePoint2 {
    /// Construct an exact 2D curve point.
    pub const fn new(x: Real, y: Real) -> Self {
        Self { x, y }
    }
}

/// Exact weighted 2D control point for homogeneous rational Bezier curves.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalCurveControlPoint2 {
    /// Affine x-coordinate before homogeneous weighting.
    pub x: Real,
    /// Affine y-coordinate before homogeneous weighting.
    pub y: Real,
    /// Homogeneous Bernstein weight.
    pub weight: Real,
}

impl RationalCurveControlPoint2 {
    /// Construct an exact weighted control point.
    pub const fn new(x: Real, y: Real, weight: Real) -> Self {
        Self { x, y, weight }
    }
}

/// Homogeneous rational 2D parametric curve in power basis.
///
/// The represented affine curve is
/// `(x_numerator(t) / weight(t), y_numerator(t) / weight(t))` when the weight
/// is nonzero. Resultant construction consumes the homogeneous equations
/// directly, so this carrier deliberately does not divide by `weight`.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalParametricCurve2 {
    /// Power-basis numerator for the x-coordinate.
    pub x_numerator: Vec<Real>,
    /// Power-basis numerator for the y-coordinate.
    pub y_numerator: Vec<Real>,
    /// Power-basis homogeneous weight.
    pub weight: Vec<Real>,
}

impl RationalParametricCurve2 {
    /// Construct a homogeneous rational parametric curve in power basis.
    pub const fn new(x_numerator: Vec<Real>, y_numerator: Vec<Real>, weight: Vec<Real>) -> Self {
        Self {
            x_numerator,
            y_numerator,
            weight,
        }
    }

    /// Lift a polynomial parametric curve into homogeneous form with unit
    /// weight.
    pub fn from_polynomial(curve: &PolynomialParametricCurve2) -> Self {
        Self {
            x_numerator: curve.x_coefficients.clone(),
            y_numerator: curve.y_coefficients.clone(),
            weight: vec![Real::one()],
        }
    }
}

/// Configuration for Bernstein-to-power-basis substitution.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BezierPowerBasisSubstitutionConfig {
    /// Maximum Bernstein degree this bounded exact package may expand.
    pub max_degree: usize,
}

impl Default for BezierPowerBasisSubstitutionConfig {
    fn default() -> Self {
        Self { max_degree: 8 }
    }
}

/// Final status for Bernstein-to-power-basis substitution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BezierPowerBasisSubstitutionStatus {
    /// The control net was expanded exactly.
    Constructed,
    /// The control net was empty.
    EmptyControlNet,
    /// The requested degree exceeds the configured bounded package.
    DegreeExceeded,
}

/// Report for exact Bernstein-to-power-basis substitution.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierPowerBasisSubstitutionReport {
    /// Final substitution status.
    pub status: BezierPowerBasisSubstitutionStatus,
    /// Bernstein degree when known.
    pub degree: Option<usize>,
    /// Source control points retained for replay.
    pub control_points: Vec<PolynomialCurvePoint2>,
    /// Exact power-basis curve when construction succeeds.
    pub power_basis: Option<PolynomialParametricCurve2>,
}

/// Final status for rational Bezier homogeneous substitution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RationalBezierPowerBasisSubstitutionStatus {
    /// The weighted control net was expanded exactly.
    Constructed,
    /// The weighted control net was empty.
    EmptyControlNet,
    /// The requested degree exceeds the configured bounded package.
    DegreeExceeded,
}

/// Report for exact rational Bezier homogeneous power-basis substitution.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierPowerBasisSubstitutionReport {
    /// Final substitution status.
    pub status: RationalBezierPowerBasisSubstitutionStatus,
    /// Bernstein degree when known.
    pub degree: Option<usize>,
    /// Source weighted control points retained for replay.
    pub control_points: Vec<RationalCurveControlPoint2>,
    /// Exact homogeneous power-basis curve when construction succeeds.
    pub homogeneous_power_basis: Option<RationalParametricCurve2>,
}

/// Convert a polynomial Bezier control net to exact power basis.
///
/// For degree `n`, the coefficient of `t^k` is
/// `sum_{i=0..k} P_i * binom(n, i) * binom(n-i, k-i) * (-1)^(k-i)`.
/// This is just the exact expansion of `P_i * B_i^n(t)`, not a sampled fit.
pub fn substitute_bezier_power_basis(
    control_points: &[PolynomialCurvePoint2],
    config: BezierPowerBasisSubstitutionConfig,
) -> BezierPowerBasisSubstitutionReport {
    if control_points.is_empty() {
        return BezierPowerBasisSubstitutionReport {
            status: BezierPowerBasisSubstitutionStatus::EmptyControlNet,
            degree: None,
            control_points: Vec::new(),
            power_basis: None,
        };
    }

    let degree = control_points.len() - 1;
    if degree > config.max_degree || degree > MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE {
        return BezierPowerBasisSubstitutionReport {
            status: BezierPowerBasisSubstitutionStatus::DegreeExceeded,
            degree: Some(degree),
            control_points: control_points.to_vec(),
            power_basis: None,
        };
    }

    let mut x_coefficients = vec![Real::zero(); degree + 1];
    let mut y_coefficients = vec![Real::zero(); degree + 1];
    for power in 0..=degree {
        for (basis_index, point) in control_points.iter().enumerate().take(power + 1) {
            let mut scale = Real::from(
                (binomial(degree, basis_index)
                    * binomial(degree - basis_index, power - basis_index)) as i64,
            );
            if (power - basis_index) % 2 == 1 {
                scale = -scale;
            }
            x_coefficients[power] += point.x.clone() * scale.clone();
            y_coefficients[power] += point.y.clone() * scale;
        }
    }

    BezierPowerBasisSubstitutionReport {
        status: BezierPowerBasisSubstitutionStatus::Constructed,
        degree: Some(degree),
        control_points: control_points.to_vec(),
        power_basis: Some(PolynomialParametricCurve2::new(
            x_coefficients,
            y_coefficients,
        )),
    }
}

/// Convert a rational Bezier control net to homogeneous exact power basis.
///
/// A rational Bezier curve is represented as
/// `sum_i w_i P_i B_i^n(t) / sum_i w_i B_i^n(t)`. This function expands the
/// three Bernstein polynomials `w_i*x_i`, `w_i*y_i`, and `w_i` into power
/// basis and returns a [`RationalParametricCurve2`] without ever dividing by
/// the weight. This is the homogeneous rational Bezier construction described
/// by Farin, *Curves and Surfaces for CAGD* (2002), used here as a Yap-style
/// exact proposal package: denominator/domain validity remains a replay
/// obligation for downstream curve code.
pub fn substitute_rational_bezier_power_basis(
    control_points: &[RationalCurveControlPoint2],
    config: BezierPowerBasisSubstitutionConfig,
) -> RationalBezierPowerBasisSubstitutionReport {
    if control_points.is_empty() {
        return RationalBezierPowerBasisSubstitutionReport {
            status: RationalBezierPowerBasisSubstitutionStatus::EmptyControlNet,
            degree: None,
            control_points: Vec::new(),
            homogeneous_power_basis: None,
        };
    }

    let degree = control_points.len() - 1;
    if degree > config.max_degree || degree > MAX_INTERNAL_BEZIER_POWER_BASIS_DEGREE {
        return RationalBezierPowerBasisSubstitutionReport {
            status: RationalBezierPowerBasisSubstitutionStatus::DegreeExceeded,
            degree: Some(degree),
            control_points: control_points.to_vec(),
            homogeneous_power_basis: None,
        };
    }

    let weighted_x = control_points
        .iter()
        .map(|point| point.x.clone() * point.weight.clone())
        .collect::<Vec<_>>();
    let weighted_y = control_points
        .iter()
        .map(|point| point.y.clone() * point.weight.clone())
        .collect::<Vec<_>>();
    let weights = control_points
        .iter()
        .map(|point| point.weight.clone())
        .collect::<Vec<_>>();
    let x_numerator = bernstein_scalars_to_power_basis(&weighted_x);
    let y_numerator = bernstein_scalars_to_power_basis(&weighted_y);
    let weight = bernstein_scalars_to_power_basis(&weights);

    RationalBezierPowerBasisSubstitutionReport {
        status: RationalBezierPowerBasisSubstitutionStatus::Constructed,
        degree: Some(degree),
        control_points: control_points.to_vec(),
        homogeneous_power_basis: Some(RationalParametricCurve2::new(
            x_numerator,
            y_numerator,
            weight,
        )),
    }
}

fn bernstein_scalars_to_power_basis(values: &[Real]) -> Vec<Real> {
    let degree = values.len() - 1;
    let mut coefficients = vec![Real::zero(); degree + 1];
    for (power, coefficient) in coefficients.iter_mut().enumerate().take(degree + 1) {
        for (basis_index, value) in values.iter().enumerate().take(power + 1) {
            let mut scale = Real::from(
                (binomial(degree, basis_index)
                    * binomial(degree - basis_index, power - basis_index)) as i64,
            );
            if (power - basis_index) % 2 == 1 {
                scale = -scale;
            }
            *coefficient += value.clone() * scale;
        }
    }
    coefficients
}

fn binomial(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    let k = k.min(n - k);
    let mut result = 1u64;
    for index in 1..=k {
        result = result * (n - k + index) as u64 / index as u64;
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

    fn point(x: i64, y: i64) -> PolynomialCurvePoint2 {
        PolynomialCurvePoint2::new(real(x), real(y))
    }

    fn weighted(x: i64, y: i64, weight: i64) -> RationalCurveControlPoint2 {
        RationalCurveControlPoint2::new(real(x), real(y), real(weight))
    }

    #[test]
    fn quadratic_bezier_substitution_expands_exact_power_basis() {
        let report = substitute_bezier_power_basis(
            &[point(0, 0), point(1, 2), point(2, 0)],
            BezierPowerBasisSubstitutionConfig::default(),
        );

        assert_eq!(
            report.status,
            BezierPowerBasisSubstitutionStatus::Constructed
        );
        let curve = report.power_basis.unwrap();
        assert_eq!(curve.x_coefficients, vec![real(0), real(2), real(0)]);
        assert_eq!(curve.y_coefficients, vec![real(0), real(4), real(-4)]);
    }

    #[test]
    fn bezier_substitution_reports_empty_and_degree_limit() {
        let empty =
            substitute_bezier_power_basis(&[], BezierPowerBasisSubstitutionConfig::default());
        assert_eq!(
            empty.status,
            BezierPowerBasisSubstitutionStatus::EmptyControlNet
        );

        let too_large = substitute_bezier_power_basis(
            &[point(0, 0), point(1, 1), point(2, 2)],
            BezierPowerBasisSubstitutionConfig { max_degree: 1 },
        );
        assert_eq!(
            too_large.status,
            BezierPowerBasisSubstitutionStatus::DegreeExceeded
        );
        assert!(too_large.power_basis.is_none());
    }

    #[test]
    fn rational_quadratic_bezier_substitution_expands_homogeneous_power_basis() {
        let report = substitute_rational_bezier_power_basis(
            &[weighted(0, 0, 1), weighted(1, 2, 2), weighted(2, 0, 1)],
            BezierPowerBasisSubstitutionConfig::default(),
        );

        assert_eq!(
            report.status,
            RationalBezierPowerBasisSubstitutionStatus::Constructed
        );
        let curve = report.homogeneous_power_basis.unwrap();
        assert_eq!(curve.x_numerator, vec![real(0), real(4), real(-2)]);
        assert_eq!(curve.y_numerator, vec![real(0), real(8), real(-8)]);
        assert_eq!(curve.weight, vec![real(1), real(2), real(-2)]);
    }

    #[test]
    fn rational_bezier_substitution_reports_empty_and_degree_limit() {
        let empty = substitute_rational_bezier_power_basis(
            &[],
            BezierPowerBasisSubstitutionConfig::default(),
        );
        assert_eq!(
            empty.status,
            RationalBezierPowerBasisSubstitutionStatus::EmptyControlNet
        );

        let too_large = substitute_rational_bezier_power_basis(
            &[weighted(0, 0, 1), weighted(1, 1, 1), weighted(2, 2, 1)],
            BezierPowerBasisSubstitutionConfig { max_degree: 1 },
        );
        assert_eq!(
            too_large.status,
            RationalBezierPowerBasisSubstitutionStatus::DegreeExceeded
        );
        assert!(too_large.homogeneous_power_basis.is_none());
    }

    proptest! {
        #[test]
        fn generated_linear_bezier_substitution_matches_endpoint_delta(
            x0 in -32_i16..=32,
            y0 in -32_i16..=32,
            x1 in -32_i16..=32,
            y1 in -32_i16..=32,
        ) {
            let (x0, y0, x1, y1) = (
                i64::from(x0),
                i64::from(y0),
                i64::from(x1),
                i64::from(y1),
            );
            let report = substitute_bezier_power_basis(
                &[point(x0, y0), point(x1, y1)],
                BezierPowerBasisSubstitutionConfig::default(),
            );

            prop_assert_eq!(report.status, BezierPowerBasisSubstitutionStatus::Constructed);
            let curve = report.power_basis.unwrap();
            prop_assert_eq!(curve.x_coefficients, vec![real(x0), real(x1 - x0)]);
            prop_assert_eq!(curve.y_coefficients, vec![real(y0), real(y1 - y0)]);
        }

        #[test]
        fn generated_linear_rational_bezier_substitution_keeps_homogeneous_weights(
            x0 in -16_i16..=16,
            y0 in -16_i16..=16,
            x1 in -16_i16..=16,
            y1 in -16_i16..=16,
            w0 in 1_i16..=8,
            w1 in 1_i16..=8,
        ) {
            let (x0, y0, x1, y1, w0, w1) = (
                i64::from(x0),
                i64::from(y0),
                i64::from(x1),
                i64::from(y1),
                i64::from(w0),
                i64::from(w1),
            );
            let report = substitute_rational_bezier_power_basis(
                &[weighted(x0, y0, w0), weighted(x1, y1, w1)],
                BezierPowerBasisSubstitutionConfig::default(),
            );

            prop_assert_eq!(
                report.status,
                RationalBezierPowerBasisSubstitutionStatus::Constructed
            );
            let curve = report.homogeneous_power_basis.unwrap();
            prop_assert_eq!(curve.x_numerator, vec![real(x0 * w0), real(x1 * w1 - x0 * w0)]);
            prop_assert_eq!(curve.y_numerator, vec![real(y0 * w0), real(y1 * w1 - y0 * w0)]);
            prop_assert_eq!(curve.weight, vec![real(w0), real(w1 - w0)]);
        }
    }
}
