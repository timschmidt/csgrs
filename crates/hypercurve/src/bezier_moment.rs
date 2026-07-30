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
//! Rational-quadratic first moments similarly retain their homogeneous `W^4`
//! integrands and use exact Hermite reduction to the same inverse-quadratic
//! branches.
//! This preserves the exact object structure required by exact-computation discipline, and supplies
//! the area facts needed by fitting/simplification pipelines discussed by Raph
//! Bezier approximation analysis. The polynomial and rational
//! Bezier identities follow the Bernstein and de Casteljau curve model.

use std::ops::Range;

use hyperreal::Real;

use crate::classify::{compare_reals, in_closed_unit_interval};
use crate::{
    Classification, CubicBezier2, CurveError, CurvePolicy, CurveResult, Point2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, UncertaintyReason,
};

#[derive(Default)]
pub(crate) struct RationalQuadraticAreaIntegralCache {
    inverse_quadratic_integrals: Vec<([Real; 3], Real)>,
}

impl RationalQuadraticAreaIntegralCache {
    fn inverse_quadratic_integral(
        &mut self,
        denominator: &[Real; 3],
        delta: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Option<Real>> {
        if let Some((_, integral)) = self
            .inverse_quadratic_integrals
            .iter()
            .find(|(cached_denominator, _)| cached_denominator == denominator)
        {
            return Ok(Some(integral.clone()));
        }
        let Some(integral) =
            integrate_inverse_quadratic(&denominator[2], &denominator[1], delta, policy)?
        else {
            return Ok(None);
        };
        self.inverse_quadratic_integrals
            .push((denominator.clone(), integral.clone()));
        Ok(Some(integral))
    }

    #[cfg(test)]
    pub(crate) fn retained_integral_count(&self) -> usize {
        self.inverse_quadratic_integrals.len()
    }
}

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
/// polynomial integrals follows exact-computation discipline. The Bezier polynomial conversion
/// follows the Bernstein and de Casteljau curve model, and the path simplification motivation follows Raph
/// Bezier approximation analysis.
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

    /// Returns the exact Green-theorem contribution of one oriented line
    /// segment.
    ///
    /// This is image geometry, independent of any nonuniform rational
    /// parameterization that may retain the same finite line.
    pub fn line_contribution(start: &Point2, end: &Point2) -> CurveResult<Self> {
        area_moments_for_controls(&[start, end])
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

    pub(crate) fn plus(&self, other: &Self) -> Self {
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
        signed_area_for_controls(&self.control_points())
    }

    /// Returns this quadratic's exact signed area and first moment contributions.
    ///
    /// The moment formulas are evaluated as exact polynomial integrals after
    /// Bernstein-to-power conversion, preserving exactness- object fact
    /// rather than sampling or flattening the curve.
    pub fn area_moments_contribution(&self) -> CurveResult<BezierAreaMoments2> {
        area_moments_for_controls(&self.control_points())
    }

    /// Returns the exact signed area contribution over the prefix interval `[0, t]`.
    ///
    /// The parameter is certified against `[0, 1]` through `policy` before the
    /// prefix curve is produced by exact de Casteljau subdivision. Ambiguous
    /// parameter ordering remains explicit uncertainty, following the exactness model's EGC
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
        signed_area_for_controls(&self.control_points())
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
    /// rational integral, following the exact-geometric-computation boundary
    /// from exact-computation discipline.  The homogeneous
    /// rational Bezier identities follow the Bernstein and de Casteljau curve model.
    ///
    /// `None` means the current exact object model cannot certify a finite
    /// affine integral: this happens when the weights do not have one proven
    /// nonzero sign, or when a symbolic transcendental branch evidence a domain
    /// boundary.  It is deliberately not a sampled fallback.
    pub fn signed_area_contribution(&self) -> CurveResult<Option<Real>> {
        rational_quadratic_signed_area_contribution(self, None)
    }

    /// Returns exact area and first moments for a finite rational quadratic.
    ///
    /// The first moments are homogeneous Green integrals with denominator
    /// `W^4`. Hypercurve reduces those rational functions symbolically to
    /// endpoint rational terms plus the same exact inverse-quadratic
    /// `atan`/`ln` branch used by signed area. `None` means weight-domain or
    /// branch evidence was insufficient; no sampled or flattened fallback is
    /// used.
    pub fn area_moments_contribution(&self) -> CurveResult<Option<BezierAreaMoments2>> {
        let weights = self.weights();
        if weights.iter().all(|weight| *weight == weights[0]) {
            return area_moments_for_controls(&self.control_points()).map(Some);
        }
        let policy = CurvePolicy::certified();
        if self.common_nonzero_weight_sign(&policy).is_none() {
            return Ok(None);
        }
        let controls = self.control_points();
        let weighted_coordinate = |coordinate: fn(&Point2) -> &Real| {
            quadratic_bernstein_power_coefficients([
                weights[0] * coordinate(controls[0]),
                weights[1] * coordinate(controls[1]),
                weights[2] * coordinate(controls[2]),
            ])
        };
        let nx = weighted_coordinate(Point2::x);
        let ny = weighted_coordinate(Point2::y);
        let w = quadratic_bernstein_power_coefficients([
            weights[0].clone(),
            weights[1].clone(),
            weights[2].clone(),
        ]);
        let dnx = derivative_coefficients(&nx)?;
        let dny = derivative_coefficients(&ny)?;
        let dw = derivative_coefficients(&w)?;
        let dx_numerator =
            polynomial_difference(&polynomial_product(&dnx, &w), &polynomial_product(&nx, &dw));
        let dy_numerator =
            polynomial_difference(&polynomial_product(&dny, &w), &polynomial_product(&ny, &dw));
        let x_numerator = polynomial_product(&polynomial_product(&nx, &nx), &dy_numerator);
        let y_numerator = polynomial_product(&polynomial_product(&ny, &ny), &dx_numerator);
        let mut cache = RationalQuadraticAreaIntegralCache::default();
        let Some(signed_area) =
            rational_quadratic_signed_area_contribution(self, Some(&mut cache))?
        else {
            return Ok(None);
        };
        let Some(x_integral) =
            integrate_polynomial_over_quadratic_fourth(&x_numerator, &w, &policy, &mut cache)?
        else {
            return Ok(None);
        };
        let Some(y_integral) =
            integrate_polynomial_over_quadratic_fourth(&y_numerator, &w, &policy, &mut cache)?
        else {
            return Ok(None);
        };
        Ok(Some(BezierAreaMoments2 {
            signed_area,
            x_moment: (x_integral / Real::from(2_i8))?,
            y_moment: (Real::zero() - (y_integral / Real::from(2_i8))?),
        }))
    }

    pub(crate) fn signed_area_contribution_with_cache(
        &self,
        cache: &mut RationalQuadraticAreaIntegralCache,
    ) -> CurveResult<Option<Real>> {
        rational_quadratic_signed_area_contribution(self, Some(cache))
    }
}

impl RationalBezier2 {
    /// Returns the exact signed-area contribution for polynomial-equivalent
    /// or degree-two rational Béziers.
    ///
    /// Equal nonzero weights cancel from every homogeneous coordinate. The
    /// affine controls can therefore use the arbitrary-degree polynomial
    /// Green integral directly without changing the curve family or sampling.
    /// Nonuniform degree-two carriers specialize exactly to the conic kernel.
    /// `None` means a higher-degree genuinely rational integral is not
    /// implemented; it does not approximate one.
    pub fn signed_area_contribution(&self) -> CurveResult<Option<Real>> {
        let Some(first_weight) = self.weights().first() else {
            return Err(CurveError::InvalidRationalBezier);
        };
        if !self.weights().iter().all(|weight| weight == first_weight) {
            return match rational_quadratic_specialization(self)? {
                Some(curve) => curve.signed_area_contribution(),
                None => Ok(None),
            };
        }
        let controls = self.control_points().iter().collect::<Vec<_>>();
        signed_area_for_controls(&controls).map(Some)
    }

    /// Returns exact area and first moments for polynomial-equivalent or
    /// degree-two rational Béziers.
    ///
    /// `None` is an explicit unsupported symbolic integral for a genuinely
    /// rational degree-three-or-higher image, never a finite approximation.
    pub fn area_moments_contribution(&self) -> CurveResult<Option<BezierAreaMoments2>> {
        let Some(first_weight) = self.weights().first() else {
            return Err(CurveError::InvalidRationalBezier);
        };
        if !self.weights().iter().all(|weight| weight == first_weight) {
            return match rational_quadratic_specialization(self)? {
                Some(curve) => curve.area_moments_contribution(),
                None => Ok(None),
            };
        }
        let controls = self.control_points().iter().collect::<Vec<_>>();
        area_moments_for_controls(&controls).map(Some)
    }
}

fn rational_quadratic_specialization(
    curve: &RationalBezier2,
) -> CurveResult<Option<RationalQuadraticBezier2>> {
    let [start, control, end] = curve.control_points() else {
        return Ok(None);
    };
    let [start_weight, control_weight, end_weight] = curve.weights() else {
        return Ok(None);
    };
    RationalQuadraticBezier2::try_new(
        start.clone(),
        control.clone(),
        end.clone(),
        start_weight.clone(),
        control_weight.clone(),
        end_weight.clone(),
    )
    .map(Some)
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
    signed_area_for_controls(&refs).map(Classification::Decided)
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
    let (x, y, dx, dy) = coordinate_power_derivatives(controls)?;
    let signed_area = signed_area_for_power_coordinates(&x, &y, &dx, &dy)?;
    let x_squared = polynomial_product(&x, &x);
    let y_squared = polynomial_product(&y, &y);
    let x_moment_integral = integrate_polynomial(&polynomial_product(&x_squared, &dy))?;
    let y_moment_integral = integrate_polynomial(&polynomial_product(&y_squared, &dx))?;

    Ok(BezierAreaMoments2 {
        signed_area,
        x_moment: (x_moment_integral / Real::from(2_i8))?,
        y_moment: (Real::zero() - (y_moment_integral / Real::from(2_i8))?),
    })
}

fn signed_area_for_controls(controls: &[&Point2]) -> CurveResult<Real> {
    match controls {
        [first, middle, last] => {
            let adjacent = point_cross_product(first, middle) + point_cross_product(middle, last);
            let numerator = Real::from(2_i8) * adjacent + point_cross_product(first, last);
            return Ok((numerator / Real::from(6_i8))?);
        }
        [first, first_middle, second_middle, last] => {
            let outer =
                point_cross_product(first, first_middle) + point_cross_product(second_middle, last);
            let inner = point_cross_product(first, second_middle)
                + point_cross_product(first_middle, second_middle)
                + point_cross_product(first_middle, last);
            let numerator = Real::from(6_i8) * outer
                + Real::from(3_i8) * inner
                + point_cross_product(first, last);
            return Ok((numerator / Real::from(20_i8))?);
        }
        _ => {}
    }
    let (x, y, dx, dy) = coordinate_power_derivatives(controls)?;
    signed_area_for_power_coordinates(&x, &y, &dx, &dy)
}

fn point_cross_product(first: &Point2, second: &Point2) -> Real {
    first.cross_product(second)
}

fn coordinate_power_derivatives(
    controls: &[&Point2],
) -> CurveResult<(Vec<Real>, Vec<Real>, Vec<Real>, Vec<Real>)> {
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
    Ok((x, y, dx, dy))
}

fn signed_area_for_power_coordinates(
    x: &[Real],
    y: &[Real],
    dx: &[Real],
    dy: &[Real],
) -> CurveResult<Real> {
    let first = polynomial_product(x, dy);
    let second = polynomial_product(y, dx);
    let signed_area_integral = integrate_polynomial_difference(&first, &second)?;
    Ok((signed_area_integral / Real::from(2_i8))?)
}

fn rational_quadratic_signed_area_contribution(
    curve: &RationalQuadraticBezier2,
    cache: Option<&mut RationalQuadraticAreaIntegralCache>,
) -> CurveResult<Option<Real>> {
    let policy = CurvePolicy::certified();
    if curve.common_nonzero_weight_sign(&policy).is_none() {
        return Ok(None);
    }

    let weights = curve.weights();
    let controls = curve.control_points();
    // After cancelling the Green integral's factor one half against the
    // homogeneous cross product's factor two, the numerator has these
    // quadratic Bernstein controls. Forming them directly avoids expanding
    // two weighted coordinates, differentiating both, and cancelling the
    // resulting cubic terms.
    let a = weights[0] * weights[1] * point_cross_product(controls[0], controls[1]);
    let b = weights[0] * weights[2] * point_cross_product(controls[0], controls[2]);
    let c = weights[1] * weights[2] * point_cross_product(controls[1], controls[2]);
    let numerator = [a.clone(), &b - &(Real::from(2_i8) * &a), &a - &b + c];
    let w = quadratic_bernstein_power_coefficients([
        weights[0].clone(),
        weights[1].clone(),
        weights[2].clone(),
    ]);

    let Some(integral) = integrate_quadratic_over_quadratic_square(&numerator, &w, &policy, cache)?
    else {
        return Ok(None);
    };
    Ok(Some(integral))
}

fn quadratic_bernstein_power_coefficients(values: [Real; 3]) -> [Real; 3] {
    let two = Real::from(2_i8);
    let c = values[0].clone();
    let b = &two * &(&values[1] - &values[0]);
    let a = &values[0] - &(&two * &values[1]) + &values[2];
    [c, b, a]
}

fn integrate_quadratic_over_quadratic_square(
    numerator: &[Real],
    denominator: &[Real; 3],
    policy: &CurvePolicy,
    cache: Option<&mut RationalQuadraticAreaIntegralCache>,
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
    let inverse_integral = if let Some(cache) = cache {
        cache.inverse_quadratic_integral(denominator, &delta, policy)?
    } else {
        integrate_inverse_quadratic(a, b, &delta, policy)?
    };
    let Some(inverse_integral) = inverse_integral else {
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

fn integrate_polynomial_over_quadratic_fourth(
    numerator: &[Real],
    denominator: &[Real; 3],
    policy: &CurvePolicy,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let c = &denominator[0];
    let b = &denominator[1];
    let a = &denominator[2];
    match compare_reals(a, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            return integrate_polynomial_over_linear_power(numerator, b, c, 4, policy);
        }
        Some(_) => {}
        None => return Ok(None),
    }
    let delta = Real::from(4_i8) * a * c - b * b;
    match compare_reals(&delta, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            integrate_polynomial_over_repeated_quadratic_fourth(numerator, a, b, policy)
        }
        Some(_) => integrate_polynomial_over_square_free_quadratic_fourth(
            numerator,
            denominator,
            &delta,
            policy,
            cache,
        ),
        None => Ok(None),
    }
}

fn integrate_polynomial_over_square_free_quadratic_fourth(
    numerator: &[Real],
    denominator: &[Real; 3],
    delta: &Real,
    policy: &CurvePolicy,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let q = denominator.to_vec();
    let dq = vec![q[1].clone(), Real::from(2_i8) * &q[2]];
    let q2 = polynomial_product(&q, &q);
    let q3 = polynomial_product(&q2, &q);
    let linear_basis = [vec![Real::one()], vec![Real::zero(), Real::one()]];
    let mut basis = Vec::with_capacity(8);
    for linear in &linear_basis {
        basis.push(polynomial_difference(
            &polynomial_product(&derivative_coefficients(linear)?, &q),
            &polynomial_scaled(&polynomial_product(linear, &dq), &Real::from(3_i8)),
        ));
    }
    for linear in &linear_basis {
        basis.push(polynomial_product(
            &polynomial_difference(
                &polynomial_product(&derivative_coefficients(linear)?, &q),
                &polynomial_scaled(&polynomial_product(linear, &dq), &Real::from(2_i8)),
            ),
            &q,
        ));
    }
    for linear in &linear_basis {
        basis.push(polynomial_product(
            &polynomial_difference(
                &polynomial_product(&derivative_coefficients(linear)?, &q),
                &polynomial_product(linear, &dq),
            ),
            &q2,
        ));
    }
    for linear in &linear_basis {
        basis.push(polynomial_product(linear, &q3));
    }

    let mut augmented = vec![vec![Real::zero(); 9]; 8];
    for (row, values) in augmented.iter_mut().enumerate() {
        for (column, polynomial) in basis.iter().enumerate() {
            values[column] = coefficient(polynomial, row);
        }
        values[8] = coefficient(numerator, row);
    }
    let Some(solution) = solve_exact_linear_system(augmented, policy)? else {
        return Ok(None);
    };
    let rational_at = |t: &Real| -> CurveResult<Option<Real>> {
        let q_at = evaluate_polynomial(&q, t);
        let a_term = evaluate_linear(&solution[0], &solution[1], t);
        let b_term = evaluate_linear(&solution[2], &solution[3], t);
        let c_term = evaluate_linear(&solution[4], &solution[5], t);
        let q2_at = &q_at * &q_at;
        let q3_at = &q2_at * &q_at;
        match (a_term / q3_at, b_term / q2_at, c_term / q_at) {
            (Ok(first), Ok(second), Ok(third)) => Ok(Some(first + second + third)),
            _ => Ok(None),
        }
    };
    let Some(upper_rational) = rational_at(&Real::one())? else {
        return Ok(None);
    };
    let Some(lower_rational) = rational_at(&Real::zero())? else {
        return Ok(None);
    };

    let two_a = Real::from(2_i8) * &q[2];
    let alpha = match solution[7].clone() / &two_a {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let beta = &solution[6] - &(&alpha * &q[1]);
    let q0 = q[0].clone();
    let q1 = &q[0] + &q[1] + &q[2];
    let log_ratio = match (q1 / q0).and_then(Real::ln) {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let Some(inverse_integral) = cache.inverse_quadratic_integral(denominator, delta, policy)?
    else {
        return Ok(None);
    };
    Ok(Some(
        upper_rational - lower_rational + alpha * log_ratio + beta * inverse_integral,
    ))
}

fn integrate_polynomial_over_linear_power(
    numerator: &[Real],
    b: &Real,
    c: &Real,
    power: usize,
    policy: &CurvePolicy,
) -> CurveResult<Option<Real>> {
    match compare_reals(b, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            let denominator = integer_power(c, i32::try_from(power).unwrap())?;
            return match integrate_polynomial(numerator)? / denominator {
                Ok(value) => Ok(Some(value)),
                Err(_) => Ok(None),
            };
        }
        Some(_) => {}
        None => return Ok(None),
    }
    let mut transformed = vec![Real::zero(); numerator.len()];
    for (degree, source) in numerator.iter().enumerate() {
        for (target_degree, target) in transformed.iter_mut().enumerate().take(degree + 1) {
            let binomial = Real::from(
                i32::try_from(binomial(degree, target_degree)?)
                    .map_err(|_| CurveError::InvalidBezierPolynomial)?,
            );
            let c_power = integer_power(
                &(Real::zero() - c),
                i32::try_from(degree - target_degree)
                    .map_err(|_| CurveError::InvalidBezierPolynomial)?,
            )?;
            let b_power = integer_power(
                b,
                i32::try_from(degree + 1).map_err(|_| CurveError::InvalidBezierPolynomial)?,
            )?;
            *target = &*target + &((source * binomial * c_power) / b_power)?;
        }
    }
    integrate_laurent_polynomial(
        &transformed,
        c,
        &(b + c),
        -i32::try_from(power).map_err(|_| CurveError::InvalidBezierPolynomial)?,
    )
}

fn integrate_polynomial_over_repeated_quadratic_fourth(
    numerator: &[Real],
    a: &Real,
    b: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Option<Real>> {
    let r = match (Real::zero() - b) / &(Real::from(2_i8) * a) {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let lower = Real::zero() - &r;
    let upper = Real::one() - &r;
    if compare_reals(&lower, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal)
        || compare_reals(&upper, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal)
    {
        return Ok(None);
    }
    let mut shifted = vec![Real::zero(); numerator.len()];
    for (degree, source) in numerator.iter().enumerate() {
        for (target_degree, target) in shifted.iter_mut().enumerate().take(degree + 1) {
            let binomial = Real::from(
                i32::try_from(binomial(degree, target_degree)?)
                    .map_err(|_| CurveError::InvalidBezierPolynomial)?,
            );
            let shift_power = integer_power(
                &r,
                i32::try_from(degree - target_degree)
                    .map_err(|_| CurveError::InvalidBezierPolynomial)?,
            )?;
            *target = &*target + &(source * binomial * shift_power);
        }
    }
    let Some(integral) = integrate_laurent_polynomial(&shifted, &lower, &upper, -8)? else {
        return Ok(None);
    };
    match integral / integer_power(a, 4)? {
        Ok(value) => Ok(Some(value)),
        Err(_) => Ok(None),
    }
}

fn integrate_laurent_polynomial(
    coefficients: &[Real],
    lower: &Real,
    upper: &Real,
    exponent_offset: i32,
) -> CurveResult<Option<Real>> {
    let mut total = Real::zero();
    for (degree, coefficient) in coefficients.iter().enumerate() {
        let exponent = i32::try_from(degree).map_err(|_| CurveError::InvalidBezierPolynomial)?
            + exponent_offset;
        let contribution = if exponent == -1 {
            match (upper.clone() / lower).and_then(Real::ln) {
                Ok(log_ratio) => coefficient * log_ratio,
                Err(_) => return Ok(None),
            }
        } else {
            let primitive_exponent = exponent + 1;
            let upper_power = integer_power(upper, primitive_exponent)?;
            let lower_power = integer_power(lower, primitive_exponent)?;
            match coefficient * (upper_power - lower_power) / Real::from(primitive_exponent) {
                Ok(value) => value,
                Err(_) => return Ok(None),
            }
        };
        total += contribution;
    }
    Ok(Some(total))
}

fn solve_exact_linear_system(
    mut augmented: Vec<Vec<Real>>,
    policy: &CurvePolicy,
) -> CurveResult<Option<Vec<Real>>> {
    let dimension = augmented.len();
    for column in 0..dimension {
        let mut pivot = None;
        for (row, values) in augmented.iter().enumerate().skip(column) {
            match compare_reals(&values[column], &Real::zero(), policy) {
                Some(std::cmp::Ordering::Equal) => {}
                Some(_) => {
                    pivot = Some(row);
                    break;
                }
                None => return Ok(None),
            }
        }
        let Some(pivot) = pivot else {
            return Ok(None);
        };
        augmented.swap(column, pivot);
        let pivot_value = augmented[column][column].clone();
        for entry in &mut augmented[column][column..=dimension] {
            *entry = match entry.clone() / &pivot_value {
                Ok(value) => value,
                Err(_) => return Ok(None),
            };
        }
        let pivot_row = augmented[column].clone();
        for (row, values) in augmented.iter_mut().enumerate() {
            if row == column {
                continue;
            }
            let factor = values[column].clone();
            for entry_column in column..=dimension {
                values[entry_column] =
                    &values[entry_column] - &(&factor * &pivot_row[entry_column]);
            }
        }
    }
    Ok(Some(
        augmented
            .into_iter()
            .map(|values| values[dimension].clone())
            .collect(),
    ))
}

fn polynomial_scaled(coefficients: &[Real], scale: &Real) -> Vec<Real> {
    coefficients
        .iter()
        .map(|coefficient| coefficient * scale)
        .collect()
}

fn polynomial_difference(first: &[Real], second: &[Real]) -> Vec<Real> {
    (0..first.len().max(second.len()))
        .map(|degree| coefficient(first, degree) - coefficient(second, degree))
        .collect()
}

fn evaluate_polynomial(coefficients: &[Real], parameter: &Real) -> Real {
    coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |value, coefficient| {
            value * parameter + coefficient
        })
}

fn evaluate_linear(constant: &Real, linear: &Real, parameter: &Real) -> Real {
    constant + linear * parameter
}

fn integer_power(base: &Real, exponent: i32) -> CurveResult<Real> {
    let magnitude = exponent.unsigned_abs();
    let mut value = Real::one();
    for _ in 0..magnitude {
        value *= base;
    }
    if exponent < 0 {
        (Real::one() / value).map_err(CurveError::from)
    } else {
        Ok(value)
    }
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

    let one_minus_t = Real::one() - &t;
    let mut levels = vec![controls.to_vec()];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let Some(previous) = levels.last() else {
            return Err(CurveError::InvalidBezierRange);
        };
        let next = previous
            .windows(2)
            .map(|pair| pair[0].lerp_with_weights(&pair[1], &one_minus_t, &t))
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

    #[test]
    fn specialized_signed_area_matches_full_moment_evaluation() {
        let quadratic = [point(-2, 3), point(5, 11), point(13, -7)];
        let cubic = [point(-2, 3), point(5, 11), point(13, -7), point(17, 2)];

        for controls in [&quadratic[..], &cubic[..]] {
            let controls = controls.iter().collect::<Vec<_>>();
            assert_eq!(
                signed_area_for_controls(&controls).unwrap(),
                area_moments_for_controls(&controls).unwrap().signed_area
            );
        }
    }

    #[test]
    fn uniform_weight_general_rational_bezier_uses_exact_polynomial_area() {
        let controls = vec![point(-2, 3), point(5, 11), point(13, -7), point(17, 2)];
        let polynomial = CubicBezier2::new(
            controls[0].clone(),
            controls[1].clone(),
            controls[2].clone(),
            controls[3].clone(),
        );
        let rational = RationalBezier2::try_new(controls.clone(), vec![Real::from(3_i8); 4])
            .expect("uniform nonzero weights are valid");
        assert_eq!(
            rational.signed_area_contribution().unwrap(),
            Some(polynomial.signed_area_contribution().unwrap())
        );

        let nonuniform =
            RationalBezier2::try_new(controls, vec![Real::one(), 2.into(), 3.into(), 4.into()])
                .expect("positive nonuniform weights are valid");
        assert_eq!(nonuniform.signed_area_contribution().unwrap(), None);
        assert_eq!(nonuniform.area_moments_contribution().unwrap(), None);
    }

    #[test]
    fn uniform_weight_rational_beziers_use_exact_polynomial_moments() {
        let quadratic_controls = [point(1, 0), point(3, 4), point(5, 0)];
        let quadratic = QuadraticBezier2::new(
            quadratic_controls[0].clone(),
            quadratic_controls[1].clone(),
            quadratic_controls[2].clone(),
        );
        let rational_quadratic = RationalQuadraticBezier2::try_new(
            quadratic_controls[0].clone(),
            quadratic_controls[1].clone(),
            quadratic_controls[2].clone(),
            Real::from(7_i8),
            Real::from(7_i8),
            Real::from(7_i8),
        )
        .unwrap();
        assert_eq!(
            rational_quadratic.area_moments_contribution().unwrap(),
            Some(quadratic.area_moments_contribution().unwrap())
        );

        let cubic_controls = vec![point(1, 0), point(2, 4), point(4, 4), point(5, 0)];
        let cubic = CubicBezier2::new(
            cubic_controls[0].clone(),
            cubic_controls[1].clone(),
            cubic_controls[2].clone(),
            cubic_controls[3].clone(),
        );
        let rational =
            RationalBezier2::try_new(cubic_controls, vec![Real::from(-3_i8); 4]).unwrap();
        assert_eq!(
            rational.area_moments_contribution().unwrap(),
            Some(cubic.area_moments_contribution().unwrap())
        );

        let nonuniform = RationalQuadraticBezier2::try_new(
            point(1, 0),
            point(3, 4),
            point(5, 0),
            Real::one(),
            Real::from(2_i8),
            Real::one(),
        )
        .unwrap();
        assert!(nonuniform.area_moments_contribution().unwrap().is_some());
    }

    #[test]
    fn rational_quadratic_quarter_circle_has_exact_green_first_moments() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let diagonal_weight = half.sqrt().unwrap();
        let quarter_circle = RationalQuadraticBezier2::try_new(
            point(1, 0),
            point(1, 1),
            point(0, 1),
            Real::one(),
            diagonal_weight,
            Real::one(),
        )
        .unwrap();
        let moments = quarter_circle
            .area_moments_contribution()
            .unwrap()
            .expect("finite positive-weight conic moments are exact");

        assert_eq!(
            compare_reals(
                moments.signed_area(),
                &((Real::pi() / Real::from(4_i8)).unwrap()),
                &CurvePolicy::certified(),
            ),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                moments.x_moment(),
                &((Real::one() / Real::from(3_i8)).unwrap()),
                &CurvePolicy::certified(),
            ),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                moments.y_moment(),
                &((Real::one() / Real::from(3_i8)).unwrap()),
                &CurvePolicy::certified(),
            ),
            Some(std::cmp::Ordering::Equal)
        );

        let general = RationalBezier2::try_new(
            quarter_circle
                .control_points()
                .into_iter()
                .cloned()
                .collect(),
            quarter_circle.weights().into_iter().cloned().collect(),
        )
        .unwrap();
        assert_eq!(general.area_moments_contribution().unwrap(), Some(moments));
    }

    #[test]
    fn rational_quadratic_linear_weight_denominator_keeps_geometric_line_moments() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let curve = RationalQuadraticBezier2::try_new(
            point(1, 2),
            point(3, 4),
            point(5, 6),
            Real::one(),
            Real::one() + half,
            Real::from(2_i8),
        )
        .unwrap();
        assert_eq!(
            curve.area_moments_contribution().unwrap(),
            Some(BezierAreaMoments2::line_contribution(curve.start(), curve.end()).unwrap())
        );
    }

    #[test]
    fn repeated_quadratic_weight_denominator_moments_reverse_exactly() {
        let curve = RationalQuadraticBezier2::try_new(
            point(1, 0),
            point(4, 5),
            point(6, 1),
            Real::one(),
            Real::from(2_i8),
            Real::from(4_i8),
        )
        .unwrap();
        let reversed = RationalQuadraticBezier2::try_new(
            curve.end().clone(),
            curve.control().clone(),
            curve.start().clone(),
            curve.end_weight().clone(),
            curve.control_weight().clone(),
            curve.start_weight().clone(),
        )
        .unwrap();
        let forward = curve
            .area_moments_contribution()
            .unwrap()
            .expect("positive repeated-quadratic denominator is finite");
        let backward = reversed
            .area_moments_contribution()
            .unwrap()
            .expect("reversed positive denominator is finite");
        for sum in [
            forward.signed_area() + backward.signed_area(),
            forward.x_moment() + backward.x_moment(),
            forward.y_moment() + backward.y_moment(),
        ] {
            assert_eq!(
                compare_reals(&sum, &Real::zero(), &CurvePolicy::certified()),
                Some(std::cmp::Ordering::Equal)
            );
        }
    }

    #[test]
    fn rational_quadratic_first_moments_are_exactly_additive_under_subdivision() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        for weights in [
            [Real::one(), Real::from(2_i8), Real::one()],
            [Real::one(), half.clone(), Real::one()],
            [Real::one(), Real::from(2_i8), Real::from(4_i8)],
        ] {
            let curve = RationalQuadraticBezier2::try_new(
                point(1, 0),
                point(4, 5),
                point(6, 1),
                weights[0].clone(),
                weights[1].clone(),
                weights[2].clone(),
            )
            .unwrap();
            let (left, right) = curve
                .split_at_exact(half.clone(), &CurvePolicy::certified())
                .unwrap();
            let whole = curve
                .area_moments_contribution()
                .unwrap()
                .expect("positive rational quadratic is finite");
            let parts = left
                .area_moments_contribution()
                .unwrap()
                .expect("left subcurve is finite")
                .plus(
                    &right
                        .area_moments_contribution()
                        .unwrap()
                        .expect("right subcurve is finite"),
                );
            for (actual, expected) in [
                (parts.signed_area(), whole.signed_area()),
                (parts.x_moment(), whole.x_moment()),
                (parts.y_moment(), whole.y_moment()),
            ] {
                assert_eq!(
                    compare_reals(actual, expected, &CurvePolicy::certified()),
                    Some(std::cmp::Ordering::Equal)
                );
            }
        }
    }

    #[test]
    fn rational_quadratic_area_cache_reuses_equal_weight_integrals() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let first = RationalQuadraticBezier2::try_new(
            point(0, 0),
            point(1, 2),
            point(3, 0),
            Real::one(),
            half.clone(),
            Real::one(),
        )
        .unwrap();
        let second = RationalQuadraticBezier2::try_new(
            point(3, 0),
            point(5, -4),
            point(8, 1),
            Real::one(),
            half,
            Real::one(),
        )
        .unwrap();
        let expected = [
            first.signed_area_contribution().unwrap(),
            second.signed_area_contribution().unwrap(),
        ];
        let mut cache = RationalQuadraticAreaIntegralCache::default();

        assert_eq!(
            first
                .signed_area_contribution_with_cache(&mut cache)
                .unwrap(),
            expected[0]
        );
        assert_eq!(cache.inverse_quadratic_integrals.len(), 1);
        assert_eq!(
            second
                .signed_area_contribution_with_cache(&mut cache)
                .unwrap(),
            expected[1]
        );
        assert_eq!(cache.inverse_quadratic_integrals.len(), 1);
    }
}
