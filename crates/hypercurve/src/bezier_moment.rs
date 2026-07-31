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
//! branches. Higher-degree rational carriers first attempt exact inverse
//! Bernstein degree elevation in homogeneous space, so serialized or otherwise
//! provenance-free degree-elevated conics reuse the same integral. Genuinely
//! higher-degree images whose Bernstein weight polynomial certifiably reduces
//! to degree two or less use polynomial division followed by the same Hermite
//! kernel. Cubic weights extend that reduction with exact rational-root,
//! Cardano, trigonometric, and repeated-factor partial-fraction branches.
//! Arbitrary-degree rational weight polynomials use the same partial-fraction
//! kernel when exact rational-root deflation leaves an exact power of one
//! certified irreducible quadratic or a quartic product of two certified
//! irreducible quadratics. Work remains bounded by the authored polynomial
//! degree; unsupported residual factors stay explicit rather than sampled.
//! This preserves the exact object structure required by exact-computation discipline, and supplies
//! the area facts needed by fitting/simplification pipelines discussed by Raph
//! Bezier approximation analysis. The polynomial and rational
//! Bezier identities follow the Bernstein and de Casteljau curve model.

use std::ops::Range;

use hyperreal::Real;
use num::{BigInt, BigUint, Integer, One, Signed, ToPrimitive};

use crate::classify::{compare_reals, in_closed_unit_interval};
use crate::{
    Classification, CubicBezier2, CurveContext, CurveError, CurveResult, Point2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, UncertaintyReason,
};

#[derive(Default)]
pub(crate) struct RationalQuadraticAreaIntegralCache {
    inverse_quadratic_integrals: Vec<([Real; 3], Real)>,
    inverse_quadratic_power_integrals: Vec<([Real; 3], Vec<Real>)>,
}

impl RationalQuadraticAreaIntegralCache {
    fn inverse_quadratic_integral(
        &mut self,
        denominator: &[Real; 3],
        delta: &Real,
        policy: &CurveContext,
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

    fn inverse_quadratic_power_integral(
        &mut self,
        denominator: &[Real; 3],
        delta: &Real,
        power: usize,
        policy: &CurveContext,
    ) -> CurveResult<Option<Real>> {
        if power == 0 {
            return Err(CurveError::InvalidBezierPolynomial);
        }
        let index = if let Some(index) = self
            .inverse_quadratic_power_integrals
            .iter()
            .position(|(cached_denominator, _)| cached_denominator == denominator)
        {
            index
        } else {
            let Some(first) = self.inverse_quadratic_integral(denominator, delta, policy)? else {
                return Ok(None);
            };
            self.inverse_quadratic_power_integrals
                .push((denominator.clone(), vec![first]));
            self.inverse_quadratic_power_integrals.len() - 1
        };
        let (_, integrals) = &mut self.inverse_quadratic_power_integrals[index];
        while integrals.len() < power {
            let current_power = integrals.len() + 1;
            let previous = integrals[current_power - 2].clone();
            let exponent = 1_i32
                .checked_sub(
                    i32::try_from(current_power)
                        .map_err(|_| CurveError::InvalidBezierPolynomial)?,
                )
                .ok_or(CurveError::InvalidBezierPolynomial)?;
            let q0 = denominator[0].clone();
            let q1 = &denominator[0] + &denominator[1] + &denominator[2];
            let endpoint = ((Real::from(2_i8) * &denominator[2] + &denominator[1])
                * integer_power(&q1, exponent)?
                - &denominator[1] * integer_power(&q0, exponent)?)
                / &(Real::from(
                    i32::try_from(current_power - 1)
                        .map_err(|_| CurveError::InvalidBezierPolynomial)?,
                ) * delta);
            let recurrence = (Real::from(2_i8)
                * &denominator[2]
                * Real::from(
                    i32::try_from(2 * current_power - 3)
                        .map_err(|_| CurveError::InvalidBezierPolynomial)?,
                )
                * previous)
                / &(Real::from(
                    i32::try_from(current_power - 1)
                        .map_err(|_| CurveError::InvalidBezierPolynomial)?,
                ) * delta);
            integrals.push(endpoint? + recurrence?);
        }
        Ok(Some(integrals[power - 1].clone()))
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
        policy: &CurveContext,
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
        policy: &CurveContext,
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
        policy: &CurveContext,
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
        policy: &CurveContext,
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
        let policy = CurveContext::STRICT;
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
    /// Returns the exact signed-area contribution for polynomial-equivalent,
    /// conic, or supported low-degree-weight rational Béziers.
    ///
    /// Equal nonzero weights cancel from every homogeneous coordinate. The
    /// affine controls can therefore use the arbitrary-degree polynomial
    /// Green integral directly without changing the curve family or sampling.
    /// Nonuniform degree-two carriers and exact degree elevations of them
    /// specialize to the retained conic kernel. Other arbitrary-degree
    /// carriers are integrated when exact Bernstein conversion certifies that
    /// their weight polynomial has degree at most two, is a cubic with an
    /// exactly classified discriminant, or exact rational-root deflation
    /// leaves an exact power of one irreducible quadratic or a quartic product
    /// of two irreducible quadratics. `None` means another genuinely rational
    /// integral is not implemented; it does not approximate one.
    pub fn signed_area_contribution(&self) -> CurveResult<Option<Real>> {
        let Some(first_weight) = self.weights().first() else {
            return Err(CurveError::InvalidRationalBezier);
        };
        if !self.weights().iter().all(|weight| weight == first_weight) {
            return match rational_quadratic_specialization(self)? {
                Some(curve) => curve.signed_area_contribution(),
                None => rational_bezier_quadratic_weight_signed_area(self),
            };
        }
        let controls = self.control_points().iter().collect::<Vec<_>>();
        signed_area_for_controls(&controls).map(Some)
    }

    /// Returns exact area and first moments for polynomial-equivalent, conic,
    /// or supported low-degree-weight rational Béziers.
    ///
    /// Cubic weight polynomials use exact Cardano or repeated-factor reduction.
    /// Higher weight polynomials use exact multiplicity-aware partial
    /// fractions when rational-root deflation leaves an exact power of one
    /// irreducible quadratic or a quartic product of two. `None` is an explicit
    /// unsupported symbolic integral for any remaining weight polynomial,
    /// never a finite approximation.
    pub fn area_moments_contribution(&self) -> CurveResult<Option<BezierAreaMoments2>> {
        let Some(first_weight) = self.weights().first() else {
            return Err(CurveError::InvalidRationalBezier);
        };
        if !self.weights().iter().all(|weight| weight == first_weight) {
            return match rational_quadratic_specialization(self)? {
                Some(curve) => curve.area_moments_contribution(),
                None => rational_bezier_quadratic_weight_area_moments(self),
            };
        }
        let controls = self.control_points().iter().collect::<Vec<_>>();
        area_moments_for_controls(&controls).map(Some)
    }
}

fn rational_quadratic_specialization(
    curve: &RationalBezier2,
) -> CurveResult<Option<RationalQuadraticBezier2>> {
    match curve.retained_quadratic_representative(&CurveContext::STRICT)? {
        Classification::Decided(representative) => Ok(representative),
        Classification::Uncertain(_) => Ok(None),
    }
}

fn rational_bezier_quadratic_weight_signed_area(
    curve: &RationalBezier2,
) -> CurveResult<Option<Real>> {
    let policy = CurveContext::STRICT;
    let Some((nx, ny, w)) = rational_bezier_supported_weight_power_coordinates(curve, &policy)?
    else {
        return Ok(None);
    };
    let dnx = derivative_coefficients(&nx)?;
    let dny = derivative_coefficients(&ny)?;
    let numerator = polynomial_difference(
        &polynomial_product(&nx, &dny),
        &polynomial_product(&ny, &dnx),
    );
    let mut cache = RationalQuadraticAreaIntegralCache::default();
    let Some(integral) =
        integrate_polynomial_over_low_degree_weight_power(&numerator, &w, 2, &policy, &mut cache)?
    else {
        return Ok(None);
    };
    Ok(Some((integral / Real::from(2_i8))?))
}

fn rational_bezier_quadratic_weight_area_moments(
    curve: &RationalBezier2,
) -> CurveResult<Option<BezierAreaMoments2>> {
    let policy = CurveContext::STRICT;
    let Some((nx, ny, w)) = rational_bezier_supported_weight_power_coordinates(curve, &policy)?
    else {
        return Ok(None);
    };
    let dnx = derivative_coefficients(&nx)?;
    let dny = derivative_coefficients(&ny)?;
    let dw = derivative_coefficients(&w)?;
    let dx_numerator =
        polynomial_difference(&polynomial_product(&dnx, &w), &polynomial_product(&nx, &dw));
    let dy_numerator =
        polynomial_difference(&polynomial_product(&dny, &w), &polynomial_product(&ny, &dw));
    let area_numerator = polynomial_difference(
        &polynomial_product(&nx, &dny),
        &polynomial_product(&ny, &dnx),
    );
    let x_numerator = polynomial_product(&polynomial_product(&nx, &nx), &dy_numerator);
    let y_numerator = polynomial_product(&polynomial_product(&ny, &ny), &dx_numerator);
    let mut cache = RationalQuadraticAreaIntegralCache::default();
    let Some(area_integral) = integrate_polynomial_over_low_degree_weight_power(
        &area_numerator,
        &w,
        2,
        &policy,
        &mut cache,
    )?
    else {
        return Ok(None);
    };
    let Some(x_integral) = integrate_polynomial_over_low_degree_weight_power(
        &x_numerator,
        &w,
        4,
        &policy,
        &mut cache,
    )?
    else {
        return Ok(None);
    };
    let Some(y_integral) = integrate_polynomial_over_low_degree_weight_power(
        &y_numerator,
        &w,
        4,
        &policy,
        &mut cache,
    )?
    else {
        return Ok(None);
    };
    Ok(Some(BezierAreaMoments2 {
        signed_area: (area_integral / Real::from(2_i8))?,
        x_moment: (x_integral / Real::from(2_i8))?,
        y_moment: (Real::zero() - (y_integral / Real::from(2_i8))?),
    }))
}

fn rational_bezier_supported_weight_power_coordinates(
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> CurveResult<Option<(Vec<Real>, Vec<Real>, Vec<Real>)>> {
    let Some(first_sign) = compare_reals(&curve.weights()[0], &Real::zero(), policy) else {
        return Ok(None);
    };
    if first_sign == std::cmp::Ordering::Equal {
        return Ok(None);
    }
    for weight in &curve.weights()[1..] {
        let Some(sign) = compare_reals(weight, &Real::zero(), policy) else {
            return Ok(None);
        };
        if sign != first_sign {
            return Ok(None);
        }
    }
    let nx = bernstein_to_power(
        curve
            .control_points()
            .iter()
            .zip(curve.weights())
            .map(|(point, weight)| point.x() * weight)
            .collect(),
    )?;
    let ny = bernstein_to_power(
        curve
            .control_points()
            .iter()
            .zip(curve.weights())
            .map(|(point, weight)| point.y() * weight)
            .collect(),
    )?;
    let mut weight_power = bernstein_to_power(curve.weights().to_vec())?;
    while weight_power.len() > 1 {
        let leading = weight_power
            .last()
            .expect("nonempty Bernstein conversion has a leading coefficient");
        match compare_reals(leading, &Real::zero(), policy) {
            Some(std::cmp::Ordering::Equal) => {
                weight_power.pop();
            }
            Some(_) => break,
            None => return Ok(None),
        }
    }
    Ok(Some((nx, ny, weight_power)))
}

fn prefix_signed_area_for_controls(
    controls: Vec<Point2>,
    t: Real,
    policy: &CurveContext,
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
    policy: &CurveContext,
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
    let policy = CurveContext::STRICT;
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
    policy: &CurveContext,
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
    policy: &CurveContext,
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

fn integrate_polynomial_over_low_degree_weight_power(
    numerator: &[Real],
    denominator: &[Real],
    power: usize,
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    match denominator.len() {
        0 => Err(CurveError::InvalidBezierPolynomial),
        1..=3 => {
            let quadratic = [
                coefficient(denominator, 0),
                coefficient(denominator, 1),
                coefficient(denominator, 2),
            ];
            match power {
                2 => {
                    integrate_polynomial_over_quadratic_square(numerator, &quadratic, policy, cache)
                }
                4 => {
                    integrate_polynomial_over_quadratic_fourth(numerator, &quadratic, policy, cache)
                }
                _ => Err(CurveError::InvalidBezierPolynomial),
            }
        }
        4 => integrate_polynomial_over_cubic_power(numerator, denominator, power, policy, cache),
        5.. => {
            let Some(factors) = exact_rational_polynomial_factors(denominator, policy)? else {
                return Ok(None);
            };
            integrate_polynomial_over_factored_power(
                numerator,
                denominator,
                power,
                &factors,
                policy,
                cache,
            )
        }
    }
}

fn integrate_polynomial_over_quadratic_square(
    numerator: &[Real],
    denominator: &[Real; 3],
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let c = &denominator[0];
    let b = &denominator[1];
    let a = &denominator[2];
    match compare_reals(a, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            return integrate_polynomial_over_linear_power(numerator, b, c, 2, policy);
        }
        Some(_) => {}
        None => return Ok(None),
    }
    let delta = Real::from(4_i8) * a * c - b * b;
    match compare_reals(&delta, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            integrate_polynomial_over_repeated_quadratic_power(numerator, a, b, 2, policy)
        }
        Some(_) => integrate_polynomial_over_square_free_quadratic_square(
            numerator,
            denominator,
            &delta,
            policy,
            cache,
        ),
        None => Ok(None),
    }
}

fn integrate_polynomial_over_square_free_quadratic_square(
    numerator: &[Real],
    denominator: &[Real; 3],
    delta: &Real,
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let q = denominator.to_vec();
    let dq = vec![q[1].clone(), Real::from(2_i8) * &q[2]];
    let q2 = polynomial_product(&q, &q);
    let Some((quotient, remainder)) = polynomial_division(numerator, &q2, policy)? else {
        return Ok(None);
    };
    let polynomial_integral = integrate_polynomial(&quotient)?;
    let linear_basis = [vec![Real::one()], vec![Real::zero(), Real::one()]];
    let mut basis = Vec::with_capacity(4);
    for linear in &linear_basis {
        basis.push(polynomial_difference(
            &polynomial_product(&derivative_coefficients(linear)?, &q),
            &polynomial_product(linear, &dq),
        ));
    }
    for linear in &linear_basis {
        basis.push(polynomial_product(linear, &q));
    }
    let mut augmented = vec![vec![Real::zero(); 5]; 4];
    for (row, values) in augmented.iter_mut().enumerate() {
        for (column, polynomial) in basis.iter().enumerate() {
            values[column] = coefficient(polynomial, row);
        }
        values[4] = coefficient(&remainder, row);
    }
    let Some(solution) = solve_exact_linear_system(augmented, policy)? else {
        return Ok(None);
    };
    let rational_at = |t: &Real| -> CurveResult<Option<Real>> {
        let q_at = evaluate_polynomial(&q, t);
        match evaluate_linear(&solution[0], &solution[1], t) / q_at {
            Ok(value) => Ok(Some(value)),
            Err(_) => Ok(None),
        }
    };
    let Some(upper_rational) = rational_at(&Real::one())? else {
        return Ok(None);
    };
    let Some(lower_rational) = rational_at(&Real::zero())? else {
        return Ok(None);
    };
    let two_a = Real::from(2_i8) * &q[2];
    let alpha = match solution[3].clone() / &two_a {
        Ok(value) => value,
        Err(_) => return Ok(None),
    };
    let beta = &solution[2] - &(&alpha * &q[1]);
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
        polynomial_integral + upper_rational - lower_rational
            + alpha * log_ratio
            + beta * inverse_integral,
    ))
}

fn integrate_polynomial_over_quadratic_fourth(
    numerator: &[Real],
    denominator: &[Real; 3],
    policy: &CurveContext,
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
            integrate_polynomial_over_repeated_quadratic_power(numerator, a, b, 4, policy)
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
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let q = denominator.to_vec();
    let dq = vec![q[1].clone(), Real::from(2_i8) * &q[2]];
    let q2 = polynomial_product(&q, &q);
    let q3 = polynomial_product(&q2, &q);
    let q4 = polynomial_product(&q3, &q);
    let Some((quotient, remainder)) = polynomial_division(numerator, &q4, policy)? else {
        return Ok(None);
    };
    let polynomial_integral = integrate_polynomial(&quotient)?;
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
        values[8] = coefficient(&remainder, row);
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
        polynomial_integral + upper_rational - lower_rational
            + alpha * log_ratio
            + beta * inverse_integral,
    ))
}

fn integrate_polynomial_over_cubic_power(
    numerator: &[Real],
    denominator: &[Real],
    power: usize,
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    if denominator.len() != 4 || !matches!(power, 2 | 4) {
        return Err(CurveError::InvalidBezierPolynomial);
    }
    match compare_reals(&denominator[3], &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            return Err(CurveError::InvalidBezierPolynomial);
        }
        Some(_) => {}
        None => return Ok(None),
    }
    if let Some(factors) = exact_rational_polynomial_factors(denominator, policy)? {
        return integrate_polynomial_over_factored_power(
            numerator,
            denominator,
            power,
            &factors,
            policy,
            cache,
        );
    }
    let (_, _, cardano_discriminant) = cubic_cardano_data(denominator)?;
    match compare_reals(&cardano_discriminant, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            return integrate_polynomial_over_repeated_cubic_power(
                numerator,
                denominator,
                power,
                policy,
                cache,
            );
        }
        Some(_) => {}
        None => return Ok(None),
    }
    let denominator_power = polynomial_integer_power(denominator, power);
    let Some((quotient, remainder)) = polynomial_division(numerator, &denominator_power, policy)?
    else {
        return Ok(None);
    };
    let polynomial_integral = integrate_polynomial(&quotient)?;
    let derivative = derivative_coefficients(denominator)?;
    let monomials = [
        vec![Real::one()],
        vec![Real::zero(), Real::one()],
        vec![Real::zero(), Real::zero(), Real::one()],
    ];
    let dimension = 3 * power;
    let mut basis = Vec::with_capacity(dimension);
    for denominator_exponent in (1..power).rev() {
        let multiplier = polynomial_integer_power(denominator, power - denominator_exponent - 1);
        for polynomial in &monomials {
            let derivative_numerator = polynomial_difference(
                &polynomial_product(&derivative_coefficients(polynomial)?, denominator),
                &polynomial_scaled(
                    &polynomial_product(polynomial, &derivative),
                    &Real::from(
                        i32::try_from(denominator_exponent)
                            .map_err(|_| CurveError::InvalidBezierPolynomial)?,
                    ),
                ),
            );
            basis.push(polynomial_product(&derivative_numerator, &multiplier));
        }
    }
    let final_multiplier = polynomial_integer_power(denominator, power - 1);
    for polynomial in &monomials {
        basis.push(polynomial_product(polynomial, &final_multiplier));
    }
    let mut augmented = vec![vec![Real::zero(); dimension + 1]; dimension];
    for (row, values) in augmented.iter_mut().enumerate() {
        for (column, polynomial) in basis.iter().enumerate() {
            values[column] = coefficient(polynomial, row);
        }
        values[dimension] = coefficient(&remainder, row);
    }
    let Some(solution) = solve_exact_linear_system(augmented, policy)? else {
        return Ok(None);
    };
    let rational_at = |t: &Real| -> CurveResult<Option<Real>> {
        let denominator_at = evaluate_polynomial(denominator, t);
        let mut total = Real::zero();
        let mut offset = 0;
        for denominator_exponent in (1..power).rev() {
            let polynomial_at = evaluate_polynomial(&solution[offset..offset + 3], t);
            let denominator_power = integer_power(
                &denominator_at,
                i32::try_from(denominator_exponent)
                    .map_err(|_| CurveError::InvalidBezierPolynomial)?,
            )?;
            total = match polynomial_at / denominator_power {
                Ok(value) => total + value,
                Err(_) => return Ok(None),
            };
            offset += 3;
        }
        Ok(Some(total))
    };
    let Some(upper_rational) = rational_at(&Real::one())? else {
        return Ok(None);
    };
    let Some(lower_rational) = rational_at(&Real::zero())? else {
        return Ok(None);
    };
    let residual_offset = 3 * (power - 1);
    let Some(residual_integral) = integrate_quadratic_over_cubic(
        &solution[residual_offset..residual_offset + 3],
        denominator,
        policy,
        cache,
    )?
    else {
        return Ok(None);
    };
    Ok(Some(
        polynomial_integral + upper_rational - lower_rational + residual_integral,
    ))
}

fn integrate_quadratic_over_cubic(
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let c = &denominator[1];
    let b = &denominator[2];
    let a = &denominator[3];
    let (p, depressed_q, cardano_discriminant) = cubic_cardano_data(denominator)?;
    let half_q = (depressed_q.clone() / Real::from(2_i8))?;
    match compare_reals(&cardano_discriminant, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Greater) => {
            let root = if let Some(root) = exact_rational_polynomial_root(denominator) {
                root
            } else {
                let square_root = cardano_discriminant.sqrt()?;
                (Real::zero() - &half_q + &square_root).cbrt()?
                    + (Real::zero() - &half_q - &square_root).cbrt()?
                    - (b.clone() / &(Real::from(3_i8) * a))?
            };
            integrate_quadratic_over_linear_quadratic_factor(
                numerator,
                denominator,
                &root,
                policy,
                cache,
            )
        }
        Some(std::cmp::Ordering::Less) => {
            let argument_scale = (Real::from(3_i8) * &depressed_q / &(Real::from(2_i8) * &p))?;
            let argument_radical = (Real::from(-3_i8) / &p)?.sqrt()?;
            let argument = argument_scale * argument_radical;
            let theta = argument.acos()?;
            let radius =
                Real::from(2_i8) * ((Real::from(-1_i8) * &p / Real::from(3_i8))?).sqrt()?;
            let shift = (b.clone() / &(Real::from(3_i8) * a))?;
            let mut total = Real::zero();
            for index in 0..3_i8 {
                let angle = ((&theta + Real::from(2_i8 * index) * Real::pi()) / Real::from(3_i8))?;
                let root = &radius * angle.cos() - &shift;
                let derivative_at =
                    Real::from(3_i8) * a * &root * &root + Real::from(2_i8) * b * &root + c;
                let coefficient = (evaluate_polynomial(numerator, &root) / derivative_at)?;
                let log_ratio = ((Real::one() - &root) / (Real::zero() - &root))?.ln()?;
                total += coefficient * log_ratio;
            }
            Ok(Some(total))
        }
        Some(std::cmp::Ordering::Equal) | None => Ok(None),
    }
}

fn cubic_cardano_data(denominator: &[Real]) -> CurveResult<(Real, Real, Real)> {
    if denominator.len() != 4 {
        return Err(CurveError::InvalidBezierPolynomial);
    }
    let d = &denominator[0];
    let c = &denominator[1];
    let b = &denominator[2];
    let a = &denominator[3];
    let a2 = a * a;
    let a3 = &a2 * a;
    let p = ((Real::from(3_i8) * a * c - b * b) / &(Real::from(3_i8) * &a2))?;
    let depressed_q = ((Real::from(2_i8) * b * b * b - Real::from(9_i8) * a * b * c
        + Real::from(27_i8) * &a2 * d)
        / &(Real::from(27_i8) * &a3))?;
    let half_q = (depressed_q.clone() / Real::from(2_i8))?;
    let third_p = (p.clone() / Real::from(3_i8))?;
    let cardano_discriminant = &half_q * &half_q + &third_p * &third_p * &third_p;
    Ok((p, depressed_q, cardano_discriminant))
}

fn integrate_polynomial_over_repeated_cubic_power(
    numerator: &[Real],
    denominator: &[Real],
    power: usize,
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let Some(factors) = repeated_cubic_linear_factors(denominator, policy)? else {
        return Ok(None);
    };
    let factors = factors
        .into_iter()
        .map(|(root, multiplicity)| ExactPolynomialFactor::Linear { root, multiplicity })
        .collect::<Vec<_>>();
    integrate_polynomial_over_factored_power(numerator, denominator, power, &factors, policy, cache)
}

#[derive(Clone)]
enum ExactPolynomialFactor {
    Linear {
        root: Real,
        multiplicity: usize,
    },
    IrreducibleQuadratic {
        denominator: [Real; 3],
        multiplicity: usize,
    },
}

impl ExactPolynomialFactor {
    fn degree_with_multiplicity(&self) -> usize {
        match self {
            Self::Linear { multiplicity, .. } => *multiplicity,
            Self::IrreducibleQuadratic { multiplicity, .. } => 2 * multiplicity,
        }
    }
}

enum PartialFractionIntegral {
    Linear {
        root: Real,
        power: usize,
    },
    QuadraticConstant {
        denominator: [Real; 3],
        power: usize,
    },
    QuadraticLinear {
        denominator: [Real; 3],
        power: usize,
    },
}

fn integrate_polynomial_over_factored_power(
    numerator: &[Real],
    denominator: &[Real],
    power: usize,
    factors: &[ExactPolynomialFactor],
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let denominator_degree = denominator
        .len()
        .checked_sub(1)
        .ok_or(CurveError::InvalidBezierPolynomial)?;
    if factors.iter().try_fold(0_usize, |total, factor| {
        total.checked_add(factor.degree_with_multiplicity())
    }) != Some(denominator_degree)
    {
        return Err(CurveError::InvalidBezierPolynomial);
    }
    let denominator_power = polynomial_integer_power(denominator, power);
    let Some((quotient, remainder)) = polynomial_division(numerator, &denominator_power, policy)?
    else {
        return Ok(None);
    };
    let dimension = denominator_degree
        .checked_mul(power)
        .ok_or(CurveError::InvalidBezierPolynomial)?;
    let mut entries = Vec::with_capacity(dimension);
    for factor in factors {
        match factor {
            ExactPolynomialFactor::Linear { root, multiplicity } => {
                let maximum_exponent = multiplicity
                    .checked_mul(power)
                    .ok_or(CurveError::InvalidBezierPolynomial)?;
                for exponent in 1..=maximum_exponent {
                    let factor = [Real::zero() - root, Real::one()];
                    let factor_power = polynomial_integer_power(&factor, exponent);
                    let Some((basis, factor_remainder)) =
                        polynomial_division(&denominator_power, &factor_power, policy)?
                    else {
                        return Ok(None);
                    };
                    if !polynomial_is_certified_zero(&factor_remainder, policy) {
                        return Ok(None);
                    }
                    entries.push((
                        PartialFractionIntegral::Linear {
                            root: root.clone(),
                            power: exponent,
                        },
                        basis,
                    ));
                }
            }
            ExactPolynomialFactor::IrreducibleQuadratic {
                denominator,
                multiplicity,
                ..
            } => {
                let maximum_exponent = multiplicity
                    .checked_mul(power)
                    .ok_or(CurveError::InvalidBezierPolynomial)?;
                for exponent in 1..=maximum_exponent {
                    let factor_power = polynomial_integer_power(denominator, exponent);
                    let Some((basis, factor_remainder)) =
                        polynomial_division(&denominator_power, &factor_power, policy)?
                    else {
                        return Ok(None);
                    };
                    if !polynomial_is_certified_zero(&factor_remainder, policy) {
                        return Ok(None);
                    }
                    entries.push((
                        PartialFractionIntegral::QuadraticConstant {
                            denominator: denominator.clone(),
                            power: exponent,
                        },
                        basis.clone(),
                    ));
                    entries.push((
                        PartialFractionIntegral::QuadraticLinear {
                            denominator: denominator.clone(),
                            power: exponent,
                        },
                        polynomial_product(&basis, &[Real::zero(), Real::one()]),
                    ));
                }
            }
        }
    }
    if entries.len() != dimension {
        return Err(CurveError::InvalidBezierPolynomial);
    }
    let mut augmented = vec![vec![Real::zero(); dimension + 1]; dimension];
    for (row, values) in augmented.iter_mut().enumerate() {
        for (column, (_, basis)) in entries.iter().enumerate() {
            values[column] = coefficient(basis, row);
        }
        values[dimension] = coefficient(&remainder, row);
    }
    let Some(solution) = solve_exact_linear_system(augmented, policy)? else {
        return Ok(None);
    };
    let mut integral = integrate_polynomial(&quotient)?;
    for (coefficient, (term, _)) in solution.into_iter().zip(entries) {
        let factor_integral = match term {
            PartialFractionIntegral::Linear { root, power } => {
                integrate_inverse_linear_factor(&root, power, policy)?
            }
            PartialFractionIntegral::QuadraticConstant { denominator, power } => {
                integrate_linear_over_irreducible_quadratic_power(
                    &Real::one(),
                    &Real::zero(),
                    &denominator,
                    power,
                    policy,
                    cache,
                )?
            }
            PartialFractionIntegral::QuadraticLinear { denominator, power } => {
                integrate_linear_over_irreducible_quadratic_power(
                    &Real::zero(),
                    &Real::one(),
                    &denominator,
                    power,
                    policy,
                    cache,
                )?
            }
        };
        let Some(factor_integral) = factor_integral else {
            return Ok(None);
        };
        integral += coefficient * factor_integral;
    }
    Ok(Some(integral))
}

fn polynomial_is_certified_zero(polynomial: &[Real], policy: &CurveContext) -> bool {
    polynomial.iter().all(|coefficient| {
        compare_reals(coefficient, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal)
    })
}

fn repeated_cubic_linear_factors(
    denominator: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<Vec<(Real, usize)>>> {
    let (p, depressed_q, discriminant) = cubic_cardano_data(denominator)?;
    if compare_reals(&discriminant, &Real::zero(), policy) != Some(std::cmp::Ordering::Equal) {
        return Ok(None);
    }
    let a = &denominator[3];
    let b = &denominator[2];
    let shift = (b.clone() / &(Real::from(3_i8) * a))?;
    match compare_reals(&p, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            if compare_reals(&depressed_q, &Real::zero(), policy) != Some(std::cmp::Ordering::Equal)
            {
                return Ok(None);
            }
            Ok(Some(vec![(Real::zero() - shift, 3)]))
        }
        Some(_) => {
            let double_root =
                (Real::from(-3_i8) * &depressed_q / &(Real::from(2_i8) * &p))? - &shift;
            let simple_root = (Real::from(3_i8) * &depressed_q / &p)? - &shift;
            Ok(Some(vec![(double_root, 2), (simple_root, 1)]))
        }
        None => Ok(None),
    }
}

fn integrate_inverse_linear_factor(
    root: &Real,
    power: usize,
    policy: &CurveContext,
) -> CurveResult<Option<Real>> {
    let lower = Real::zero() - root;
    let upper = Real::one() - root;
    if compare_reals(&lower, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal)
        || compare_reals(&upper, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal)
    {
        return Ok(None);
    }
    if power == 1 {
        let ratio = (upper / lower)?;
        if compare_reals(&ratio, &Real::zero(), policy) != Some(std::cmp::Ordering::Greater) {
            return Ok(None);
        }
        return Ok(Some(ratio.ln()?));
    }
    let exponent = 1_i32
        .checked_sub(i32::try_from(power).map_err(|_| CurveError::InvalidBezierPolynomial)?)
        .ok_or(CurveError::InvalidBezierPolynomial)?;
    let integral = ((integer_power(&upper, exponent)? - integer_power(&lower, exponent)?)
        / Real::from(exponent))?;
    Ok(Some(integral))
}

fn integrate_linear_over_irreducible_quadratic_power(
    constant: &Real,
    linear: &Real,
    denominator: &[Real; 3],
    power: usize,
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let delta =
        Real::from(4_i8) * &denominator[2] * &denominator[0] - &denominator[1] * &denominator[1];
    if compare_reals(&delta, &Real::zero(), policy) != Some(std::cmp::Ordering::Greater) {
        return Ok(None);
    }
    let alpha = (linear.clone() / &(Real::from(2_i8) * &denominator[2]))?;
    let beta = constant - &(&alpha * &denominator[1]);
    let q0 = denominator[0].clone();
    let q1 = &denominator[0] + &denominator[1] + &denominator[2];
    let derivative_integral = if power == 1 {
        let ratio = (q1 / q0)?;
        if compare_reals(&ratio, &Real::zero(), policy) != Some(std::cmp::Ordering::Greater) {
            return Ok(None);
        }
        ratio.ln()?
    } else {
        let exponent = 1_i32
            .checked_sub(i32::try_from(power).map_err(|_| CurveError::InvalidBezierPolynomial)?)
            .ok_or(CurveError::InvalidBezierPolynomial)?;
        ((integer_power(&q1, exponent)? - integer_power(&q0, exponent)?) / Real::from(exponent))?
    };
    let Some(inverse_integral) =
        cache.inverse_quadratic_power_integral(denominator, &delta, power, policy)?
    else {
        return Ok(None);
    };
    Ok(Some(alpha * derivative_integral + beta * inverse_integral))
}

fn exact_rational_polynomial_factors(
    polynomial: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<Vec<ExactPolynomialFactor>>> {
    if polynomial.len() < 2 {
        return Err(CurveError::InvalidBezierPolynomial);
    }
    let mut remaining = polynomial.to_vec();
    let mut factors = Vec::with_capacity(polynomial.len() - 1);
    while remaining.len() > 1 {
        let Some(root) = exact_rational_polynomial_root(&remaining) else {
            if let Some(factor) = exact_repeated_irreducible_quadratic_factor(&remaining, policy)? {
                factors.push(factor);
                return Ok(Some(factors));
            }
            if let Some(quadratics) = exact_irreducible_quadratic_pair_factors(&remaining, policy)?
            {
                factors.extend(quadratics);
                return Ok(Some(factors));
            }
            return Ok(None);
        };
        let factor = [Real::zero() - &root, Real::one()];
        let mut multiplicity = 0_usize;
        loop {
            let Some((quotient, factor_remainder)) =
                polynomial_division(&remaining, &factor, policy)?
            else {
                return Ok(None);
            };
            if factor_remainder.iter().any(|coefficient| {
                compare_reals(coefficient, &Real::zero(), policy) != Some(std::cmp::Ordering::Equal)
            }) {
                break;
            }
            remaining = quotient;
            multiplicity = multiplicity
                .checked_add(1)
                .ok_or(CurveError::InvalidBezierPolynomial)?;
            if remaining.len() <= 1
                || compare_reals(
                    &evaluate_polynomial(&remaining, &root),
                    &Real::zero(),
                    policy,
                ) != Some(std::cmp::Ordering::Equal)
            {
                break;
            }
        }
        if multiplicity == 0 {
            return Ok(None);
        }
        factors.push(ExactPolynomialFactor::Linear { root, multiplicity });
    }
    Ok(Some(factors))
}

fn exact_repeated_irreducible_quadratic_factor(
    polynomial: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<ExactPolynomialFactor>> {
    let degree = polynomial
        .len()
        .checked_sub(1)
        .ok_or(CurveError::InvalidBezierPolynomial)?;
    if degree < 2 || !degree.is_multiple_of(2) {
        return Ok(None);
    }
    let multiplicity = degree / 2;
    let leading = &polynomial[degree];
    match compare_reals(leading, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            return Err(CurveError::InvalidBezierPolynomial);
        }
        Some(_) => {}
        None => return Ok(None),
    }
    let multiplicity_real =
        Real::from(i32::try_from(multiplicity).map_err(|_| CurveError::InvalidBezierPolynomial)?);
    let linear = (polynomial[degree - 1].clone() / &(&multiplicity_real * leading))?;
    let normalized_next = (polynomial[degree - 2].clone() / leading)?;
    let pair_count = Real::from(
        i32::try_from(binomial(multiplicity, 2)?)
            .map_err(|_| CurveError::InvalidBezierPolynomial)?,
    );
    let constant = ((normalized_next - pair_count * &linear * &linear) / &multiplicity_real)?;
    let denominator = [constant, linear, Real::one()];
    let reconstructed = polynomial_scaled(
        &polynomial_integer_power(&denominator, multiplicity),
        leading,
    );
    if !polynomial_is_certified_zero(&polynomial_difference(polynomial, &reconstructed), policy) {
        return Ok(None);
    }
    let delta = Real::from(4_i8) * &denominator[0] - &denominator[1] * &denominator[1];
    if compare_reals(&delta, &Real::zero(), policy) != Some(std::cmp::Ordering::Greater) {
        return Ok(None);
    }
    Ok(Some(ExactPolynomialFactor::IrreducibleQuadratic {
        denominator,
        multiplicity,
    }))
}

fn exact_irreducible_quadratic_pair_factors(
    polynomial: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<Vec<ExactPolynomialFactor>>> {
    if polynomial.len() != 5 {
        return Ok(None);
    }
    let leading = &polynomial[4];
    match compare_reals(leading, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            return Err(CurveError::InvalidBezierPolynomial);
        }
        Some(_) => {}
        None => return Ok(None),
    }
    let a = (polynomial[3].clone() / leading)?;
    let b = (polynomial[2].clone() / leading)?;
    let c = (polynomial[1].clone() / leading)?;
    let d = (polynomial[0].clone() / leading)?;
    let normalized = [a, b, c, d];
    let mut resolvent = vec![
        &normalized[3] * &(Real::from(4_i8) * &normalized[1] - &normalized[0] * &normalized[0])
            - &normalized[2] * &normalized[2],
        &normalized[0] * &normalized[2] - Real::from(4_i8) * &normalized[3],
        Real::zero() - &normalized[1],
        Real::one(),
    ];
    while resolvent.len() > 1 {
        let Some(sum) = exact_rational_polynomial_root(&resolvent) else {
            return Ok(None);
        };
        if let Some(factors) = irreducible_quadratic_pair_from_resolvent_root(
            polynomial,
            leading,
            &normalized,
            &sum,
            policy,
        )? {
            return Ok(Some(factors));
        }
        let factor = [Real::zero() - &sum, Real::one()];
        let Some((quotient, remainder)) = polynomial_division(&resolvent, &factor, policy)? else {
            return Ok(None);
        };
        if !polynomial_is_certified_zero(&remainder, policy) {
            return Ok(None);
        }
        resolvent = quotient;
    }
    Ok(None)
}

fn irreducible_quadratic_pair_from_resolvent_root(
    polynomial: &[Real],
    leading: &Real,
    normalized: &[Real; 4],
    sum: &Real,
    policy: &CurveContext,
) -> CurveResult<Option<Vec<ExactPolynomialFactor>>> {
    let [a, b, c, d] = normalized;
    let difference_squared = a * a - Real::from(4_i8) * b + Real::from(4_i8) * sum;
    let product = a * sum - Real::from(2_i8) * c;
    let difference = match compare_reals(&difference_squared, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Greater) => difference_squared.sqrt()?,
        Some(std::cmp::Ordering::Equal) => Real::zero(),
        Some(std::cmp::Ordering::Less) | None => return Ok(None),
    };
    let constant_difference = match compare_reals(&difference, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            if compare_reals(&product, &Real::zero(), policy) != Some(std::cmp::Ordering::Equal) {
                return Ok(None);
            }
            let squared = sum * sum - Real::from(4_i8) * d;
            match compare_reals(&squared, &Real::zero(), policy) {
                Some(std::cmp::Ordering::Greater) => squared.sqrt()?,
                Some(std::cmp::Ordering::Equal) => Real::zero(),
                Some(std::cmp::Ordering::Less) | None => return Ok(None),
            }
        }
        Some(_) => (product / &difference)?,
        None => return Ok(None),
    };
    let two = Real::from(2_i8);
    let first = [
        ((sum + &constant_difference) / &two)?,
        ((a + &difference) / &two)?,
        Real::one(),
    ];
    let second = [
        ((sum - &constant_difference) / &two)?,
        ((a - &difference) / &two)?,
        Real::one(),
    ];
    for denominator in [&first, &second] {
        let delta = Real::from(4_i8) * &denominator[0] - &denominator[1] * &denominator[1];
        if compare_reals(&delta, &Real::zero(), policy) != Some(std::cmp::Ordering::Greater) {
            return Ok(None);
        }
    }
    let reconstructed = polynomial_scaled(&polynomial_product(&first, &second), leading);
    if !polynomial_is_certified_zero(&polynomial_difference(polynomial, &reconstructed), policy) {
        return Ok(None);
    }
    Ok(Some(vec![
        ExactPolynomialFactor::IrreducibleQuadratic {
            denominator: first,
            multiplicity: 1,
        },
        ExactPolynomialFactor::IrreducibleQuadratic {
            denominator: second,
            multiplicity: 1,
        },
    ]))
}

fn exact_rational_polynomial_root(polynomial: &[Real]) -> Option<Real> {
    const MAX_RATIONAL_ROOT_FACTOR: u64 = 1_000_000_000;

    if polynomial.len() < 2 {
        return None;
    }
    let coefficients = polynomial
        .iter()
        .map(Real::exact_rational)
        .collect::<Option<Vec<_>>>()?;
    let common_denominator = coefficients
        .iter()
        .fold(BigUint::one(), |common, coefficient| {
            common.lcm(coefficient.denominator())
        });
    let mut integer_coefficients = coefficients
        .iter()
        .map(|coefficient| {
            let scale = &common_denominator / coefficient.denominator();
            let magnitude = BigInt::from(coefficient.numerator().clone()) * BigInt::from(scale);
            if coefficient.is_negative() {
                -magnitude
            } else {
                magnitude
            }
        })
        .collect::<Vec<_>>();
    let content = integer_coefficients
        .iter()
        .fold(BigInt::from(0_i8), |content, coefficient| {
            content.gcd(coefficient)
        })
        .abs();
    if content != BigInt::from(0_i8) && content != BigInt::from(1_i8) {
        for coefficient in &mut integer_coefficients {
            *coefficient /= &content;
        }
    }
    let constant = integer_coefficients[0].abs().to_u64()?;
    let leading = integer_coefficients.last()?.abs().to_u64()?;
    if constant == 0
        || leading == 0
        || constant > MAX_RATIONAL_ROOT_FACTOR
        || leading > MAX_RATIONAL_ROOT_FACTOR
    {
        return None;
    }
    let numerators = positive_divisors(constant);
    let denominators = positive_divisors(leading);
    for numerator in numerators {
        for factor_denominator in &denominators {
            if numerator.gcd(factor_denominator) != 1 {
                continue;
            }
            let numerator = i64::try_from(numerator).ok()?;
            for signed_numerator in [numerator, -numerator] {
                let candidate = Real::new(
                    hyperreal::Rational::fraction(signed_numerator, *factor_denominator).ok()?,
                );
                if evaluate_polynomial(polynomial, &candidate).definitely_zero() {
                    return Some(candidate);
                }
            }
        }
    }
    None
}

fn positive_divisors(value: u64) -> Vec<u64> {
    let mut low = Vec::new();
    let mut high = Vec::new();
    let mut divisor = 1_u64;
    while divisor <= value / divisor {
        if value.is_multiple_of(divisor) {
            low.push(divisor);
            let paired = value / divisor;
            if paired != divisor {
                high.push(paired);
            }
        }
        divisor += 1;
    }
    high.reverse();
    low.extend(high);
    low
}

fn integrate_quadratic_over_linear_quadratic_factor(
    numerator: &[Real],
    denominator: &[Real],
    root: &Real,
    policy: &CurveContext,
    cache: &mut RationalQuadraticAreaIntegralCache,
) -> CurveResult<Option<Real>> {
    let quadratic = [
        &denominator[1] + &(&denominator[2] * root) + &(&denominator[3] * root * root),
        &denominator[2] + &(&denominator[3] * root),
        denominator[3].clone(),
    ];
    let quadratic_at_root = evaluate_polynomial(&quadratic, root);
    let linear_coefficient = (evaluate_polynomial(numerator, root) / quadratic_at_root)?;
    let remainder = polynomial_difference(
        numerator,
        &polynomial_scaled(&quadratic, &linear_coefficient),
    );
    let b = coefficient(&remainder, 2);
    let c = coefficient(&remainder, 1) + &b * root;
    let linear_log = ((Real::one() - root) / (Real::zero() - root))?.ln()?;
    let two_a = Real::from(2_i8) * &quadratic[2];
    let alpha = (b / &two_a)?;
    let beta = &c - &(&alpha * &quadratic[1]);
    let q0 = quadratic[0].clone();
    let q1 = &quadratic[0] + &quadratic[1] + &quadratic[2];
    let quadratic_log = (q1 / q0)?.ln()?;
    let delta = Real::from(4_i8) * &quadratic[2] * &quadratic[0] - &quadratic[1] * &quadratic[1];
    if compare_reals(&delta, &Real::zero(), policy) != Some(std::cmp::Ordering::Greater) {
        return Ok(None);
    }
    let Some(inverse_integral) = cache.inverse_quadratic_integral(&quadratic, &delta, policy)?
    else {
        return Ok(None);
    };
    Ok(Some(
        linear_coefficient * linear_log + alpha * quadratic_log + beta * inverse_integral,
    ))
}

fn integrate_polynomial_over_linear_power(
    numerator: &[Real],
    b: &Real,
    c: &Real,
    power: usize,
    policy: &CurveContext,
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

fn integrate_polynomial_over_repeated_quadratic_power(
    numerator: &[Real],
    a: &Real,
    b: &Real,
    power: usize,
    policy: &CurveContext,
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
    let exponent_offset = i32::try_from(power)
        .map_err(|_| CurveError::InvalidBezierPolynomial)?
        .checked_mul(-2)
        .ok_or(CurveError::InvalidBezierPolynomial)?;
    let Some(integral) = integrate_laurent_polynomial(&shifted, &lower, &upper, exponent_offset)?
    else {
        return Ok(None);
    };
    match integral
        / integer_power(
            a,
            i32::try_from(power).map_err(|_| CurveError::InvalidBezierPolynomial)?,
        )? {
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
    policy: &CurveContext,
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

fn polynomial_division(
    numerator: &[Real],
    divisor: &[Real],
    policy: &CurveContext,
) -> CurveResult<Option<(Vec<Real>, Vec<Real>)>> {
    let Some(leading) = divisor.last() else {
        return Err(CurveError::InvalidBezierPolynomial);
    };
    match compare_reals(leading, &Real::zero(), policy) {
        Some(std::cmp::Ordering::Equal) => {
            return Err(CurveError::InvalidBezierPolynomial);
        }
        Some(_) => {}
        None => return Ok(None),
    }
    if numerator.len() < divisor.len() {
        return Ok(Some((Vec::new(), numerator.to_vec())));
    }
    let divisor_degree = divisor.len() - 1;
    let mut remainder = numerator.to_vec();
    let mut quotient = vec![Real::zero(); numerator.len() - divisor_degree];
    for degree in (divisor_degree..remainder.len()).rev() {
        let factor = match remainder[degree].clone() / leading {
            Ok(value) => value,
            Err(_) => return Ok(None),
        };
        let quotient_degree = degree - divisor_degree;
        quotient[quotient_degree] = factor.clone();
        for (divisor_degree, divisor_coefficient) in divisor.iter().enumerate() {
            let target_degree = quotient_degree + divisor_degree;
            remainder[target_degree] = &remainder[target_degree] - &(&factor * divisor_coefficient);
        }
    }
    remainder.truncate(divisor_degree);
    Ok(Some((quotient, remainder)))
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
    policy: &CurveContext,
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

fn polynomial_integer_power(polynomial: &[Real], exponent: usize) -> Vec<Real> {
    let mut result = vec![Real::one()];
    for _ in 0..exponent {
        result = polynomial_product(&result, polynomial);
    }
    result
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

    fn assert_rational_moments_are_exactly_additive(curve: &RationalBezier2) {
        let whole = curve
            .area_moments_contribution()
            .unwrap()
            .expect("the rational moments have an exact symbolic reduction");
        let signed_area = curve
            .signed_area_contribution()
            .unwrap()
            .expect("the rational signed area has an exact symbolic reduction");
        assert_eq!(
            compare_reals(&signed_area, whole.signed_area(), &CurveContext::STRICT),
            Some(std::cmp::Ordering::Equal)
        );
        let Classification::Decided((left, right)) = curve
            .split_at_exact(
                &((Real::one() / Real::from(2_i8)).unwrap()),
                &CurveContext::STRICT,
            )
            .unwrap()
        else {
            panic!("represented midpoint split is exact");
        };
        let parts = left
            .area_moments_contribution()
            .unwrap()
            .expect("left rational moment is exact")
            .plus(
                &right
                    .area_moments_contribution()
                    .unwrap()
                    .expect("right rational moment is exact"),
            );
        for (actual, expected) in [
            (parts.signed_area(), whole.signed_area()),
            (parts.x_moment(), whole.x_moment()),
            (parts.y_moment(), whole.y_moment()),
        ] {
            // Integration and subdivision produce distinct symbolic forms.
            // This assertion explicitly permits Hyperlimit's terminal
            // approximate-512 equality; topology above remains strict.
            assert_eq!(
                compare_reals(actual, expected, &CurveContext::APPROXIMATE_512),
                Some(std::cmp::Ordering::Equal)
            );
        }
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
    fn general_rational_bezier_uses_exact_polynomial_and_low_degree_weight_moments() {
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

        let nonuniform = RationalBezier2::try_new(
            controls.clone(),
            vec![Real::one(), 2.into(), 3.into(), 4.into()],
        )
        .expect("positive nonuniform weights are valid");
        let signed_area = nonuniform
            .signed_area_contribution()
            .unwrap()
            .expect("linear weight denominator is symbolically integrable");
        let moments = nonuniform
            .area_moments_contribution()
            .unwrap()
            .expect("linear weight denominator first moments are symbolically integrable");
        assert_eq!(
            compare_reals(&signed_area, moments.signed_area(), &CurveContext::STRICT),
            Some(std::cmp::Ordering::Equal)
        );

        let quartic_weight = RationalBezier2::try_new(
            vec![
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
                controls[3].clone(),
                point(19, -3),
            ],
            vec![Real::one(), 2.into(), 3.into(), 5.into(), 10.into()],
        )
        .expect("positive quartic weight function is valid");
        assert_eq!(quartic_weight.signed_area_contribution().unwrap(), None);
        assert_eq!(quartic_weight.area_moments_contribution().unwrap(), None);
    }

    #[test]
    fn cubic_weight_moments_with_one_real_root_are_exactly_additive() {
        assert_eq!(
            exact_rational_polynomial_root(&[Real::one(), Real::one(), Real::one(), Real::one(),]),
            Some(Real::from(-1_i8))
        );
        let one_third = (Real::one() / Real::from(3_i8)).unwrap();
        let curve = RationalBezier2::try_new(
            vec![point(4, 0), point(6, 0), point(6, 2), point(4, 2)],
            vec![
                Real::one(),
                Real::one() + &one_third,
                Real::from(2_i8),
                Real::from(4_i8),
            ],
        )
        .unwrap();
        let whole = curve
            .area_moments_contribution()
            .unwrap()
            .expect("square-free cubic weight moments have an exact Cardano reduction");
        let signed_area = curve
            .signed_area_contribution()
            .unwrap()
            .expect("square-free cubic weight signed area has an exact Cardano reduction");
        assert_eq!(
            compare_reals(&signed_area, whole.signed_area(), &CurveContext::STRICT),
            Some(std::cmp::Ordering::Equal)
        );
        let Classification::Decided((left, right)) = curve
            .split_at_exact(
                &((Real::one() / Real::from(2_i8)).unwrap()),
                &CurveContext::STRICT,
            )
            .unwrap()
        else {
            panic!("represented midpoint split is exact");
        };
        let parts = left
            .area_moments_contribution()
            .unwrap()
            .expect("left cubic-denominator moment is exact")
            .plus(
                &right
                    .area_moments_contribution()
                    .unwrap()
                    .expect("right cubic-denominator moment is exact"),
            );
        for (actual, expected) in [
            (parts.signed_area(), whole.signed_area()),
            (parts.x_moment(), whole.x_moment()),
            (parts.y_moment(), whole.y_moment()),
        ] {
            assert_eq!(
                compare_reals(actual, expected, &CurveContext::APPROXIMATE_512),
                Some(std::cmp::Ordering::Equal)
            );
        }
    }

    #[test]
    fn cubic_weight_moments_with_three_real_roots_are_exactly_additive() {
        let one_third = (Real::one() / Real::from(3_i8)).unwrap();
        let curve = RationalBezier2::try_new(
            vec![point(4, 0), point(6, 0), point(6, 2), point(4, 2)],
            vec![
                Real::from(6_i8),
                Real::from(9_i8) + &(Real::from(2_i8) * &one_third),
                Real::from(15_i8) + &one_third,
                Real::from(24_i8),
            ],
        )
        .unwrap();
        let whole = curve
            .area_moments_contribution()
            .unwrap()
            .expect("three-root cubic weight moments have an exact trigonometric reduction");
        let signed_area = curve
            .signed_area_contribution()
            .unwrap()
            .expect("three-root cubic weight signed area has an exact reduction");
        assert_eq!(
            compare_reals(&signed_area, whole.signed_area(), &CurveContext::STRICT),
            Some(std::cmp::Ordering::Equal)
        );
        let Classification::Decided((left, right)) = curve
            .split_at_exact(
                &((Real::one() / Real::from(2_i8)).unwrap()),
                &CurveContext::STRICT,
            )
            .unwrap()
        else {
            panic!("represented midpoint split is exact");
        };
        let parts = left
            .area_moments_contribution()
            .unwrap()
            .expect("left three-root cubic-denominator moment is exact")
            .plus(
                &right
                    .area_moments_contribution()
                    .unwrap()
                    .expect("right three-root cubic-denominator moment is exact"),
            );
        for (actual, expected) in [
            (parts.signed_area(), whole.signed_area()),
            (parts.x_moment(), whole.x_moment()),
            (parts.y_moment(), whole.y_moment()),
        ] {
            assert_eq!(
                compare_reals(actual, expected, &CurveContext::APPROXIMATE_512),
                Some(std::cmp::Ordering::Equal)
            );
        }
    }

    #[test]
    fn repeated_root_cubic_weight_moments_are_exactly_additive() {
        let one_third = (Real::one() / Real::from(3_i8)).unwrap();
        let curve = RationalBezier2::try_new(
            vec![point(4, 0), point(6, 0), point(6, 2), point(4, 2)],
            vec![
                Real::from(2_i8),
                Real::from(3_i8) + &(Real::from(2_i8) * &one_third),
                Real::from(6_i8) + &(Real::from(2_i8) * &one_third),
                Real::from(12_i8),
            ],
        )
        .unwrap();

        assert_rational_moments_are_exactly_additive(&curve);
    }

    #[test]
    fn triple_root_cubic_weight_moments_are_exactly_additive() {
        let curve = RationalBezier2::try_new(
            vec![point(4, 0), point(6, 0), point(6, 2), point(4, 2)],
            vec![
                Real::one(),
                Real::from(2_i8),
                Real::from(4_i8),
                Real::from(8_i8),
            ],
        )
        .unwrap();

        assert_rational_moments_are_exactly_additive(&curve);
    }

    #[test]
    fn fully_split_quartic_weight_moments_are_exactly_additive() {
        let curve = RationalBezier2::try_new(
            vec![
                point(4, 0),
                point(6, 0),
                point(6, 1),
                point(6, 2),
                point(4, 2),
            ],
            vec![
                Real::one(),
                Real::from(2_i8),
                Real::from(4_i8),
                Real::from(8_i8),
                Real::from(16_i8),
            ],
        )
        .unwrap();

        assert_rational_moments_are_exactly_additive(&curve);
    }

    #[test]
    fn arbitrary_degree_split_weight_moments_are_exactly_additive() {
        let curve = RationalBezier2::try_new(
            vec![
                point(4, 0),
                point(5, 0),
                point(6, 0),
                point(6, 1),
                point(6, 2),
                point(5, 3),
                point(4, 3),
                point(3, 2),
                point(3, 1),
                point(4, 0),
            ],
            vec![
                Real::one(),
                Real::from(2_i16),
                Real::from(4_i16),
                Real::from(8_i16),
                Real::from(16_i16),
                Real::from(32_i16),
                Real::from(64_i16),
                Real::from(128_i16),
                Real::from(256_i16),
                Real::from(512_i16),
            ],
        )
        .unwrap();

        assert_rational_moments_are_exactly_additive(&curve);
    }

    #[test]
    fn mixed_linear_quadratic_quartic_weight_moments_are_exactly_additive() {
        let curve = RationalBezier2::try_new(
            vec![
                point(4, 0),
                point(6, 0),
                point(6, 1),
                point(6, 2),
                point(4, 2),
            ],
            vec![
                Real::one(),
                (Real::from(3_i8) / Real::from(2_i8)).unwrap(),
                (Real::from(7_i8) / Real::from(3_i8)).unwrap(),
                Real::from(4_i8),
                Real::from(8_i8),
            ],
        )
        .unwrap();

        assert_rational_moments_are_exactly_additive(&curve);
    }

    #[test]
    fn repeated_irreducible_quadratic_weight_moments_are_exactly_additive() {
        let curve = RationalBezier2::try_new(
            vec![
                point(4, 0),
                point(6, 0),
                point(6, 1),
                point(6, 2),
                point(4, 2),
            ],
            vec![
                Real::one(),
                Real::one(),
                (Real::from(4_i8) / Real::from(3_i8)).unwrap(),
                Real::from(2_i8),
                Real::from(4_i8),
            ],
        )
        .unwrap();

        assert_rational_moments_are_exactly_additive(&curve);
    }

    #[test]
    fn distinct_irreducible_quadratic_weight_moments_are_exactly_additive() {
        let curve = RationalBezier2::try_new(
            vec![
                point(4, 0),
                point(6, 0),
                point(6, 1),
                point(6, 2),
                point(4, 2),
            ],
            vec![
                Real::from(2_i8),
                (Real::from(3_i8) / Real::from(2_i8)).unwrap(),
                (Real::from(3_i8) / Real::from(2_i8)).unwrap(),
                (Real::from(3_i8) / Real::from(2_i8)).unwrap(),
                Real::from(2_i8),
            ],
        )
        .unwrap();

        assert_rational_moments_are_exactly_additive(&curve);
    }

    #[test]
    fn genuinely_cubic_rational_quadratic_weight_moments_are_exactly_additive() {
        let two_thirds = (Real::from(2_i8) / Real::from(3_i8)).unwrap();
        let curve = RationalBezier2::try_new(
            vec![point(4, 0), point(6, 0), point(6, 2), point(4, 2)],
            vec![Real::one(), two_thirds.clone(), two_thirds, Real::one()],
        )
        .unwrap();
        let whole = curve
            .area_moments_contribution()
            .unwrap()
            .expect("quadratic weight polynomial has an exact symbolic integral");
        let signed_area = curve
            .signed_area_contribution()
            .unwrap()
            .expect("quadratic weight polynomial signed area is exact");
        assert_eq!(
            compare_reals(&signed_area, whole.signed_area(), &CurveContext::STRICT),
            Some(std::cmp::Ordering::Equal)
        );
        let Classification::Decided((left, right)) = curve
            .split_at_exact(
                &((Real::one() / Real::from(2_i8)).unwrap()),
                &CurveContext::STRICT,
            )
            .unwrap()
        else {
            panic!("represented midpoint split is exact");
        };
        let parts = left
            .area_moments_contribution()
            .unwrap()
            .expect("left quadratic-denominator moment is exact")
            .plus(
                &right
                    .area_moments_contribution()
                    .unwrap()
                    .expect("right quadratic-denominator moment is exact"),
            );
        for (actual, expected) in [
            (parts.signed_area(), whole.signed_area()),
            (parts.x_moment(), whole.x_moment()),
            (parts.y_moment(), whole.y_moment()),
        ] {
            assert_eq!(
                compare_reals(actual, expected, &CurveContext::APPROXIMATE_512),
                Some(std::cmp::Ordering::Equal)
            );
        }
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
                &CurveContext::APPROXIMATE_512,
            ),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                moments.x_moment(),
                &((Real::one() / Real::from(3_i8)).unwrap()),
                &CurveContext::APPROXIMATE_512,
            ),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(
                moments.y_moment(),
                &((Real::one() / Real::from(3_i8)).unwrap()),
                &CurveContext::APPROXIMATE_512,
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
        assert_eq!(
            general.area_moments_contribution().unwrap(),
            Some(moments.clone())
        );

        let elevated = general.elevated_to_degree(7).unwrap();
        assert_eq!(elevated.degree(), 7);
        assert_eq!(
            elevated.area_moments_contribution().unwrap(),
            Some(moments.clone())
        );
        assert_eq!(
            elevated.signed_area_contribution().unwrap(),
            Some(moments.signed_area().clone())
        );

        let reconstructed = RationalBezier2::try_new(
            elevated.control_points().to_vec(),
            elevated.weights().to_vec(),
        )
        .unwrap();
        let reconstructed_moments = reconstructed
            .area_moments_contribution()
            .unwrap()
            .expect("exact inverse degree elevation recovers the conic kernel");
        for (actual, expected) in [
            (reconstructed_moments.signed_area(), moments.signed_area()),
            (reconstructed_moments.x_moment(), moments.x_moment()),
            (reconstructed_moments.y_moment(), moments.y_moment()),
        ] {
            assert_eq!(
                compare_reals(actual, expected, &CurveContext::APPROXIMATE_512),
                Some(std::cmp::Ordering::Equal)
            );
        }
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
                compare_reals(&sum, &Real::zero(), &CurveContext::STRICT),
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
                .split_at_exact(half.clone(), &CurveContext::STRICT)
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
                    compare_reals(actual, expected, &CurveContext::APPROXIMATE_512),
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
