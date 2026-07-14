//! Exact area and moment-style adapters for Bezier and conic segments.
//!
//! A Bezier segment's signed area contribution is the Green's-theorem boundary
//! integral `1/2 * integral(x dy - y dx)`. We evaluate that integral exactly by
//! converting the Bezier coordinates from Bernstein to power form and
//! integrating the resulting polynomial. Rational quadratic conics use the same
//! Green integral in homogeneous coordinates: `x = Nx/W`, `y = Ny/W`, so
//! `x dy - y dx = (Nx dNy - Ny dNx) / W^2`. The resulting rational integral is
//! evaluated symbolically with exact `atan`/`ln`/`sqrt` branches after the
//! Bernstein weights certify that `W` has no projective zero on `[0, 1]`.
//! This preserves the exact object structure required by Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997), and supplies
//! the area facts needed by fitting/simplification pipelines discussed by Raph
//! Levien, "Simplifying Bezier paths" (2021). The polynomial and rational
//! Bezier identities follow Farin, *Curves and Surfaces for Computer-Aided
//! Geometric Design* (5th ed., 2002).

use std::ops::Range;

use hyperreal::Real;

use crate::classify::{compare_reals, in_closed_unit_interval};
use crate::{
    Classification, CubicBezier2, CurveError, CurvePolicy, CurveResult, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, UncertaintyReason,
};

/// Exact Green's-theorem area and first-moment boundary contributions.
///
/// The `signed_area` component is `1/2 * integral(x dy - y dx)`. The
/// `x_moment` component is `integral integral x dA = 1/2 * integral(x^2 dy)`,
/// and the `y_moment` component is
/// `integral integral y dA = -1/2 * integral(y^2 dx)`. These are boundary
/// contributions for an oriented path segment; closed-region semantics come
/// from summing all boundary segments with the chosen winding convention.
///
/// The formulas are the standard Green's-theorem moment identities used by
/// exact geometric-computation pipelines; retaining them as symbolic
/// polynomial integrals follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997). The Bezier polynomial conversion
/// follows Farin, *Curves and Surfaces for Computer-Aided Geometric Design*
/// (5th ed., 2002), and the path simplification motivation follows Raph
/// Levien, "Simplifying Bezier paths" (2021).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAreaMoments2 {
    signed_area: Real,
    x_moment: Real,
    y_moment: Real,
}

impl BezierAreaMoments2 {
    /// Returns a zero contribution.
    pub fn zero() -> Self {
        Self {
            signed_area: Real::zero(),
            x_moment: Real::zero(),
            y_moment: Real::zero(),
        }
    }

    /// Returns the exact signed-area boundary contribution.
    pub fn signed_area(&self) -> &Real {
        &self.signed_area
    }

    /// Returns the exact `integral integral x dA` boundary contribution.
    pub fn x_moment(&self) -> &Real {
        &self.x_moment
    }

    /// Returns the exact `integral integral y dA` boundary contribution.
    pub fn y_moment(&self) -> &Real {
        &self.y_moment
    }

    fn plus(&self, other: &Self) -> Self {
        Self {
            signed_area: &self.signed_area + &other.signed_area,
            x_moment: &self.x_moment + &other.x_moment,
            y_moment: &self.y_moment + &other.y_moment,
        }
    }

    fn minus(&self, other: &Self) -> Self {
        Self {
            signed_area: &self.signed_area - &other.signed_area,
            x_moment: &self.x_moment - &other.x_moment,
            y_moment: &self.y_moment - &other.y_moment,
        }
    }
}

/// Exact prefix sums of Bezier signed-area boundary contributions.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAreaPrefixSums2 {
    prefixes: Vec<Real>,
}

impl BezierAreaPrefixSums2 {
    /// Builds prefix sums from exact per-segment signed-area contributions.
    pub fn from_contributions(contributions: impl IntoIterator<Item = Real>) -> Self {
        let mut prefixes = vec![Real::zero()];
        for contribution in contributions {
            let next = prefixes.last().expect("prefix list always contains zero") + &contribution;
            prefixes.push(next);
        }
        Self { prefixes }
    }

    /// Builds prefix sums from polynomial quadratic Bezier segments.
    pub fn from_quadratics<'a>(
        curves: impl IntoIterator<Item = &'a QuadraticBezier2>,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(QuadraticBezier2::signed_area_contribution)
            .collect::<CurveResult<Vec<_>>>()
            .map(Self::from_contributions)
    }

    /// Builds prefix sums from polynomial cubic Bezier segments.
    pub fn from_cubics<'a>(
        curves: impl IntoIterator<Item = &'a CubicBezier2>,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(CubicBezier2::signed_area_contribution)
            .collect::<CurveResult<Vec<_>>>()
            .map(Self::from_contributions)
    }

    /// Returns the number of segment contributions represented by this table.
    pub fn segment_count(&self) -> usize {
        self.prefixes.len().saturating_sub(1)
    }

    /// Returns the total signed-area contribution of all stored segments.
    pub fn total(&self) -> &Real {
        self.prefixes
            .last()
            .expect("prefix list always contains zero")
    }

    /// Returns all exact prefix sums, including the initial zero.
    pub fn prefixes(&self) -> &[Real] {
        &self.prefixes
    }

    /// Returns the exact signed-area contribution over a segment range.
    pub fn range_contribution(&self, range: Range<usize>) -> CurveResult<Real> {
        if range.start > range.end || range.end > self.segment_count() {
            return Err(CurveError::InvalidBezierRange);
        }
        Ok(&self.prefixes[range.end] - &self.prefixes[range.start])
    }
}

/// Exact prefix sums of Bezier area and first-moment boundary contributions.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierAreaMomentPrefixSums2 {
    prefixes: Vec<BezierAreaMoments2>,
}

impl BezierAreaMomentPrefixSums2 {
    /// Builds prefix sums from exact per-segment area/moment contributions.
    pub fn from_contributions(contributions: impl IntoIterator<Item = BezierAreaMoments2>) -> Self {
        let mut prefixes = vec![BezierAreaMoments2::zero()];
        for contribution in contributions {
            let next = prefixes
                .last()
                .expect("prefix list always contains zero")
                .plus(&contribution);
            prefixes.push(next);
        }
        Self { prefixes }
    }

    /// Builds area/moment prefix sums from polynomial quadratic Bezier segments.
    pub fn from_quadratics<'a>(
        curves: impl IntoIterator<Item = &'a QuadraticBezier2>,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(QuadraticBezier2::area_moments_contribution)
            .collect::<CurveResult<Vec<_>>>()
            .map(Self::from_contributions)
    }

    /// Builds area/moment prefix sums from polynomial cubic Bezier segments.
    pub fn from_cubics<'a>(
        curves: impl IntoIterator<Item = &'a CubicBezier2>,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(CubicBezier2::area_moments_contribution)
            .collect::<CurveResult<Vec<_>>>()
            .map(Self::from_contributions)
    }

    /// Returns the number of segment contributions represented by this table.
    pub fn segment_count(&self) -> usize {
        self.prefixes.len().saturating_sub(1)
    }

    /// Returns the total area/moment contribution of all stored segments.
    pub fn total(&self) -> &BezierAreaMoments2 {
        self.prefixes
            .last()
            .expect("prefix list always contains zero")
    }

    /// Returns all exact prefix sums, including the initial zero.
    pub fn prefixes(&self) -> &[BezierAreaMoments2] {
        &self.prefixes
    }

    /// Returns the exact area/moment contribution over a segment range.
    pub fn range_contribution(&self, range: Range<usize>) -> CurveResult<BezierAreaMoments2> {
        if range.start > range.end || range.end > self.segment_count() {
            return Err(CurveError::InvalidBezierRange);
        }
        Ok(self.prefixes[range.end].minus(&self.prefixes[range.start]))
    }
}

impl QuadraticBezier2 {
    /// Returns this quadratic's exact signed area boundary contribution.
    ///
    /// This is the Green's-theorem integral over the oriented curve segment,
    /// not an area of the control polygon and not a sampled approximation.
    pub fn signed_area_contribution(&self) -> CurveResult<Real> {
        Ok(area_moments_for_controls(&self.control_points())?.signed_area)
    }

    /// Returns this quadratic's exact signed area and first moment contributions.
    ///
    /// The moment formulas are evaluated as exact polynomial integrals after
    /// Bernstein-to-power conversion, preserving the Yap-style object fact
    /// rather than sampling or flattening the curve.
    pub fn area_moments_contribution(&self) -> CurveResult<BezierAreaMoments2> {
        area_moments_for_controls(&self.control_points())
    }

    /// Returns the exact signed area contribution over the prefix interval `[0, t]`.
    ///
    /// The parameter is certified against `[0, 1]` through `policy` before the
    /// prefix curve is produced by exact de Casteljau subdivision. Ambiguous
    /// parameter ordering remains explicit uncertainty, following Yap's EGC
    /// predicate boundary.
    pub fn prefix_signed_area_contribution(
        &self,
        t: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Real>> {
        prefix_signed_area_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            policy,
        )
    }

    /// Returns exact area and first moments over the prefix interval `[0, t]`.
    pub fn prefix_area_moments_contribution(
        &self,
        t: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierAreaMoments2>> {
        prefix_area_moments_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            policy,
        )
    }
}

impl CubicBezier2 {
    /// Returns this cubic's exact signed area boundary contribution.
    pub fn signed_area_contribution(&self) -> CurveResult<Real> {
        Ok(area_moments_for_controls(&self.control_points())?.signed_area)
    }

    /// Returns this cubic's exact signed area and first moment contributions.
    pub fn area_moments_contribution(&self) -> CurveResult<BezierAreaMoments2> {
        area_moments_for_controls(&self.control_points())
    }

    /// Returns the exact signed area contribution over the prefix interval `[0, t]`.
    pub fn prefix_signed_area_contribution(
        &self,
        t: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Real>> {
        prefix_signed_area_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            policy,
        )
    }

    /// Returns exact area and first moments over the prefix interval `[0, t]`.
    pub fn prefix_area_moments_contribution(
        &self,
        t: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierAreaMoments2>> {
        prefix_area_moments_for_controls(
            self.control_points().into_iter().cloned().collect(),
            t,
            policy,
        )
    }
}

impl RationalQuadraticBezier2 {
    /// Returns this rational quadratic's exact signed-area boundary contribution.
    ///
    /// The contribution is the Green integral
    /// `1/2 * integral((Nx dNy - Ny dNx) / W^2)`, where `Nx`, `Ny`, and `W`
    /// are the weighted Bernstein numerator and denominator polynomials.  The
    /// implementation keeps the conic in homogeneous form until the final exact
    /// rational integral, following Yap's exact-geometric-computation boundary
    /// from "Towards Exact Geometric Computation" (1997).  The homogeneous
    /// rational Bezier identities follow Farin, *Curves and Surfaces for
    /// Computer-Aided Geometric Design* (5th ed., 2002).
    ///
    /// `None` means the current exact object model cannot certify a finite
    /// affine integral: this happens when the weights do not have one proven
    /// nonzero sign, or when a symbolic transcendental branch reports a domain
    /// boundary.  It is deliberately not a sampled fallback.
    pub fn signed_area_contribution(&self) -> CurveResult<Option<Real>> {
        rational_quadratic_signed_area_contribution(self)
    }
}

fn prefix_signed_area_for_controls(
    controls: Vec<Point2>,
    t: Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Real>> {
    match in_closed_unit_interval(&t, policy) {
        Some(true) => {}
        Some(false) => return Err(CurveError::InvalidBezierParameter),
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
    let (prefix, _) = subdivide_controls_at(&controls, t)?;
    let refs = prefix.iter().collect::<Vec<_>>();
    area_moments_for_controls(&refs).map(|moments| Classification::Decided(moments.signed_area))
}

fn prefix_area_moments_for_controls(
    controls: Vec<Point2>,
    t: Real,
    policy: &CurvePolicy,
) -> CurveResult<Classification<BezierAreaMoments2>> {
    match in_closed_unit_interval(&t, policy) {
        Some(true) => {}
        Some(false) => return Err(CurveError::InvalidBezierParameter),
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
    let (prefix, _) = subdivide_controls_at(&controls, t)?;
    let refs = prefix.iter().collect::<Vec<_>>();
    area_moments_for_controls(&refs).map(Classification::Decided)
}

fn area_moments_for_controls(controls: &[&Point2]) -> CurveResult<BezierAreaMoments2> {
    let x = bernstein_to_power(
        controls
            .iter()
            .map(|point| point.x().clone())
            .collect::<Vec<_>>(),
    )?;
    let y = bernstein_to_power(
        controls
            .iter()
            .map(|point| point.y().clone())
            .collect::<Vec<_>>(),
    )?;
    let dx = derivative_coefficients(&x)?;
    let dy = derivative_coefficients(&y)?;
    let first = polynomial_product(&x, &dy);
    let second = polynomial_product(&y, &dx);
    let signed_area_integral = integrate_polynomial_difference(&first, &second)?;
    let x_squared = polynomial_product(&x, &x);
    let y_squared = polynomial_product(&y, &y);
    let x_moment_integral = integrate_polynomial(&polynomial_product(&x_squared, &dy))?;
    let y_moment_integral = integrate_polynomial(&polynomial_product(&y_squared, &dx))?;

    Ok(BezierAreaMoments2 {
        signed_area: (signed_area_integral / Real::from(2_i8))?,
        x_moment: (x_moment_integral / Real::from(2_i8))?,
        y_moment: (Real::zero() - (y_moment_integral / Real::from(2_i8))?),
    })
}

fn rational_quadratic_signed_area_contribution(
    curve: &RationalQuadraticBezier2,
) -> CurveResult<Option<Real>> {
    let policy = CurvePolicy::certified();
    if curve.common_nonzero_weight_sign(&policy).is_none() {
        return Ok(None);
    }

    let weights = curve.weights();
    let controls = curve.control_points();
    let x_weighted = [
        controls[0].x() * weights[0],
        controls[1].x() * weights[1],
        controls[2].x() * weights[2],
    ];
    let y_weighted = [
        controls[0].y() * weights[0],
        controls[1].y() * weights[1],
        controls[2].y() * weights[2],
    ];
    let w = quadratic_bernstein_power_coefficients([
        weights[0].clone(),
        weights[1].clone(),
        weights[2].clone(),
    ]);
    let nx = quadratic_bernstein_power_coefficients(x_weighted);
    let ny = quadratic_bernstein_power_coefficients(y_weighted);
    let dnx = derivative_coefficients(&nx)?;
    let dny = derivative_coefficients(&ny)?;
    let numerator = polynomial_difference(
        &polynomial_product(&nx, &dny),
        &polynomial_product(&ny, &dnx),
    );

    let Some(integral) = integrate_quadratic_over_quadratic_square(&numerator, &w, &policy)? else {
        return Ok(None);
    };
    Ok(Some((integral / Real::from(2_i8))?))
}

fn quadratic_bernstein_power_coefficients(values: [Real; 3]) -> [Real; 3] {
    let two = Real::from(2_i8);
    let c = values[0].clone();
    let b = &two * &(&values[1] - &values[0]);
    let a = &values[0] - &(&two * &values[1]) + &values[2];
    [c, b, a]
}

fn polynomial_difference(first: &[Real], second: &[Real]) -> Vec<Real> {
    (0..first.len().max(second.len()))
        .map(|degree| {
            first.get(degree).cloned().unwrap_or_else(Real::zero)
                - second.get(degree).cloned().unwrap_or_else(Real::zero)
        })
        .collect()
}

fn integrate_quadratic_over_quadratic_square(
    numerator: &[Real],
    denominator: &[Real; 3],
    policy: &CurvePolicy,
) -> CurveResult<Option<Real>> {
    let m0 = coefficient(numerator, 0);
    let m1 = coefficient(numerator, 1);
    let m2 = coefficient(numerator, 2);
    let c = &denominator[0];
    let b = &denominator[1];
    let a = &denominator[2];

    if compare_reals(a, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal) {
        return integrate_quadratic_over_linear_square(&m0, &m1, &m2, b, c, policy);
    }

    let four = Real::from(4_i8);
    let two = Real::from(2_i8);
    let delta = &(&four * a * c) - &(b * b);
    if compare_reals(&delta, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal) {
        return integrate_quadratic_over_repeated_quadratic_square(&m0, &m1, &m2, a, b);
    }

    let m2_over_a = match m2.clone() / a {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let two_a = &two * a;
    let b_m1_over_two_a = match (b * &m1) / &two_a {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let c_m2_over_a = c * &m2_over_a;
    let k_numerator = &m0 + &c_m2_over_a - &b_m1_over_two_a;
    let k_denominator = &(&two * c) - &((b * b) / &two_a)?;
    let k = match k_numerator / k_denominator {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let u = &k - &m2_over_a;
    let v = match (&(&k * b) - &m1) / &two_a {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let derivative_part = rational_linear_over_quadratic_at(&u, &v, a, b, c, &Real::one())?
        - rational_linear_over_quadratic_at(&u, &v, a, b, c, &Real::zero())?;
    let Some(inverse_integral) = integrate_inverse_quadratic(a, b, &delta, policy)? else {
        return Ok(None);
    };
    Ok(Some(derivative_part + k * inverse_integral))
}

fn integrate_quadratic_over_linear_square(
    m0: &Real,
    m1: &Real,
    m2: &Real,
    b: &Real,
    c: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Option<Real>> {
    if compare_reals(b, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal) {
        let denominator = c * c;
        let polynomial_integral = integrate_polynomial(&[m0.clone(), m1.clone(), m2.clone()])?;
        return match polynomial_integral / denominator {
            Ok(value) => Ok(Some(value)),
            Err(_) => Ok(None),
        };
    }

    let b2 = b * b;
    let b3 = &b2 * b;
    let a_term = match m2.clone() / &b3 {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let m1_over_b2 = match m1.clone() / &b2 {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let two_c_m2_over_b3 = match (Real::from(2_i8) * c * m2) / &b3 {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let b_term = m1_over_b2 - two_c_m2_over_b3;
    let c2_m2_over_b3 = match (c * c * m2) / &b3 {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let c_m1_over_b2 = match (c * m1) / &b2 {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let m0_over_b = match m0.clone() / b {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let c_term = c2_m2_over_b3 - c_m1_over_b2 + m0_over_b;
    let u0 = c.clone();
    let u1 = b + c;
    let log_ratio = match (u1.clone() / &u0).and_then(Real::ln) {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let reciprocal_delta = match (Real::one() / &u1, Real::one() / &u0) {
        (Ok(upper), Ok(lower)) => upper - lower,
        _ => return Ok(None),
    };
    Ok(Some(
        a_term * (&u1 - &u0) + b_term * log_ratio - c_term * reciprocal_delta,
    ))
}

fn integrate_quadratic_over_repeated_quadratic_square(
    m0: &Real,
    m1: &Real,
    m2: &Real,
    a: &Real,
    b: &Real,
) -> CurveResult<Option<Real>> {
    let two = Real::from(2_i8);
    let three = Real::from(3_i8);
    let r = match (Real::zero() - b) / &(two.clone() * a) {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let a2 = a * a;
    let shifted_b = &(two * &r * m2) + m1;
    let shifted_c = &(m2 * &r * &r) + &(m1 * &r) + m0;
    let primitive = |t: Real| -> CurveResult<Option<Real>> {
        let u = t - &r;
        let u2 = &u * &u;
        let u3 = &u2 * &u;
        let first = match (Real::zero() - m2) / &u {
            Ok(value) => value,
            Err(_) => return Ok(None),
        };
        let second = match (Real::zero() - &shifted_b) / &(Real::from(2_i8) * &u2) {
            Ok(value) => value,
            Err(_) => return Ok(None),
        };
        let third = match (Real::zero() - &shifted_c) / &(three.clone() * &u3) {
            Ok(value) => value,
            Err(_) => return Ok(None),
        };
        match (first + second + third) / &a2 {
            Ok(value) => Ok(Some(value)),
            Err(_) => Ok(None),
        }
    };
    let Some(upper) = primitive(Real::one())? else {
        return Ok(None);
    };
    let Some(lower) = primitive(Real::zero())? else {
        return Ok(None);
    };
    Ok(Some(upper - lower))
}

fn integrate_inverse_quadratic(
    a: &Real,
    b: &Real,
    delta: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Option<Real>> {
    match compare_reals(delta, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Greater) => {
            let sqrt_delta = match delta.clone().sqrt() {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
            let upper = match ((Real::from(2_i8) * a + b) / &sqrt_delta).and_then(Real::atan) {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
            let lower = match (b.clone() / &sqrt_delta).and_then(Real::atan) {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
            Ok(Some((Real::from(2_i8) * (upper - lower) / sqrt_delta)?))
        }
        Some(std::cmp::Ordering::Less) => {
            let discriminant = Real::zero() - delta;
            let sqrt_discriminant = match discriminant.sqrt() {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
            let ratio_at = |t: Real| -> CurveResult<Option<Real>> {
                let u = Real::from(2_i8) * a * &t + b;
                let numerator = &u - &sqrt_discriminant;
                let denominator = &u + &sqrt_discriminant;
                match numerator / denominator {
                    Ok(value) => Ok(Some(value)),
                    Err(_) => Ok(None),
                }
            };
            let Some(upper_ratio) = ratio_at(Real::one())? else {
                return Ok(None);
            };
            let Some(lower_ratio) = ratio_at(Real::zero())? else {
                return Ok(None);
            };
            let log_ratio = match (upper_ratio / lower_ratio).and_then(Real::ln) {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
            Ok(Some((log_ratio / sqrt_discriminant)?))
        }
        Some(std::cmp::Ordering::Equal) => {
            let upper = match Real::from(-2_i8) / &(Real::from(2_i8) * a + b) {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
            let lower = match Real::from(-2_i8) / b {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
            Ok(Some(upper - lower))
        }
        None => Ok(None),
    }
}

fn rational_linear_over_quadratic_at(
    u: &Real,
    v: &Real,
    a: &Real,
    b: &Real,
    c: &Real,
    t: &Real,
) -> CurveResult<Real> {
    let numerator = u * t + v;
    let denominator = a * t * t + b * t + c;
    (numerator / denominator).map_err(CurveError::from)
}

fn coefficient(coefficients: &[Real], degree: usize) -> Real {
    coefficients.get(degree).cloned().unwrap_or_else(Real::zero)
}

fn integrate_polynomial_difference(first: &[Real], second: &[Real]) -> CurveResult<Real> {
    let mut integral = Real::zero();
    for degree in 0..first.len().max(second.len()) {
        let value = first.get(degree).cloned().unwrap_or_else(Real::zero)
            - second.get(degree).cloned().unwrap_or_else(Real::zero);
        integral = &integral + (value / positive_degree_denominator(degree)?)?;
    }
    Ok(integral)
}

fn integrate_polynomial(coefficients: &[Real]) -> CurveResult<Real> {
    let mut integral = Real::zero();
    for (degree, coefficient) in coefficients.iter().enumerate() {
        integral = &integral + (coefficient.clone() / positive_degree_denominator(degree)?)?;
    }
    Ok(integral)
}

fn positive_degree_denominator(zero_based_degree: usize) -> CurveResult<Real> {
    let denominator = zero_based_degree
        .checked_add(1)
        .ok_or(CurveError::InvalidBezierPolynomial)?;
    let denominator =
        i32::try_from(denominator).map_err(|_| CurveError::InvalidBezierPolynomial)?;
    Ok(Real::from(denominator))
}

fn bernstein_to_power(values: Vec<Real>) -> CurveResult<Vec<Real>> {
    let Some(degree) = values.len().checked_sub(1) else {
        return Err(CurveError::InvalidBezierRange);
    };
    let mut coeffs = vec![Real::zero(); values.len()];
    for (i, value) in values.into_iter().enumerate() {
        for (k, coefficient) in coeffs.iter_mut().enumerate().take(degree + 1).skip(i) {
            let magnitude = binomial(degree, i)?
                .checked_mul(binomial(degree - i, k - i)?)
                .ok_or(CurveError::InvalidBezierPolynomial)?;
            let magnitude =
                i32::try_from(magnitude).map_err(|_| CurveError::InvalidBezierPolynomial)?;
            let signed = if (k - i) % 2 == 0 {
                magnitude
            } else {
                magnitude
                    .checked_neg()
                    .ok_or(CurveError::InvalidBezierPolynomial)?
            };
            *coefficient = &*coefficient + (&value * &Real::from(signed));
        }
    }
    Ok(coeffs)
}

fn derivative_coefficients(coefficients: &[Real]) -> CurveResult<Vec<Real>> {
    coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| {
            let degree = i32::try_from(degree).map_err(|_| CurveError::InvalidBezierPolynomial)?;
            Ok(coefficient * &Real::from(degree))
        })
        .collect()
}

fn polynomial_product(first: &[Real], second: &[Real]) -> Vec<Real> {
    if first.is_empty() || second.is_empty() {
        return Vec::new();
    }
    let mut product = vec![Real::zero(); first.len() + second.len() - 1];
    for (i, a) in first.iter().enumerate() {
        for (j, b) in second.iter().enumerate() {
            product[i + j] = &product[i + j] + &(a * b);
        }
    }
    product
}

fn subdivide_controls_at(controls: &[Point2], t: Real) -> CurveResult<(Vec<Point2>, Vec<Point2>)> {
    if controls.is_empty() {
        return Err(CurveError::InvalidBezierRange);
    }

    let mut levels = vec![controls.to_vec()];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let Some(previous) = levels.last() else {
            return Err(CurveError::InvalidBezierRange);
        };
        let next = previous
            .windows(2)
            .map(|pair| pair[0].lerp(&pair[1], t.clone()))
            .collect::<Vec<_>>();
        levels.push(next);
    }

    let left = levels
        .iter()
        .map(|level| level[0].clone())
        .collect::<Vec<_>>();
    let right = levels
        .iter()
        .rev()
        .map(|level| level[level.len() - 1].clone())
        .collect::<Vec<_>>();
    Ok((left, right))
}

fn binomial(n: usize, k: usize) -> CurveResult<usize> {
    if k > n {
        return Ok(0);
    }

    let k = k.min(n - k);
    let mut value = 1usize;
    for step in 1..=k {
        let numerator = n
            .checked_add(1)
            .and_then(|value| value.checked_sub(step))
            .ok_or(CurveError::InvalidBezierPolynomial)?;
        value = value
            .checked_mul(numerator)
            .ok_or(CurveError::InvalidBezierPolynomial)?
            / step;
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn moment_subdivision_rejects_empty_controls() {
        assert_eq!(
            subdivide_controls_at(&[], Real::zero()),
            Err(CurveError::InvalidBezierRange)
        );
    }

    #[test]
    fn area_moments_reject_empty_controls() {
        let controls = Vec::<&Point2>::new();
        assert_eq!(
            area_moments_for_controls(&controls),
            Err(CurveError::InvalidBezierRange)
        );
    }

    #[test]
    fn bernstein_to_power_handles_supported_higher_degree() {
        let coefficients = bernstein_to_power(vec![
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ])
        .unwrap();

        assert_eq!(
            coefficients,
            vec![
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::zero(),
                Real::one()
            ]
        );
        assert_eq!(binomial(4, 2), Ok(6));
    }

    #[test]
    fn area_moments_accept_constant_control() {
        let control = point(3, 5);
        let controls = vec![&control];
        assert_eq!(
            area_moments_for_controls(&controls).unwrap(),
            BezierAreaMoments2::zero()
        );
    }
}
