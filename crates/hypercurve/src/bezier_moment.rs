//! Exact area and moment-style adapters for polynomial Bezier segments.
//!
//! A Bezier segment's signed area contribution is the Green's-theorem boundary
//! integral `1/2 * integral(x dy - y dx)`. We evaluate that integral exactly by
//! converting the Bezier coordinates from Bernstein to power form and
//! integrating the resulting polynomial. This preserves the exact object
//! structure required by Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997), and supplies the area facts needed by
//! fitting/simplification pipelines discussed by Raph Levien, "Simplifying
//! Bezier paths" (2021). The Bezier polynomial identities follow Farin,
//! *Curves and Surfaces for Computer-Aided Geometric Design* (5th ed., 2002).

use std::ops::Range;

use hyperreal::Real;

use crate::classify::in_closed_unit_interval;
use crate::{
    Classification, CubicBezier2, CurveError, CurvePolicy, CurveResult, Point2, QuadraticBezier2,
    UncertaintyReason,
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
    let (prefix, _) = subdivide_controls_at(&controls, t);
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
    let (prefix, _) = subdivide_controls_at(&controls, t);
    let refs = prefix.iter().collect::<Vec<_>>();
    area_moments_for_controls(&refs).map(Classification::Decided)
}

fn area_moments_for_controls(controls: &[&Point2]) -> CurveResult<BezierAreaMoments2> {
    let x = bernstein_to_power(
        controls
            .iter()
            .map(|point| point.x().clone())
            .collect::<Vec<_>>(),
    );
    let y = bernstein_to_power(
        controls
            .iter()
            .map(|point| point.y().clone())
            .collect::<Vec<_>>(),
    );
    let dx = derivative_coefficients(&x);
    let dy = derivative_coefficients(&y);
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

fn integrate_polynomial_difference(first: &[Real], second: &[Real]) -> CurveResult<Real> {
    let mut integral = Real::zero();
    for degree in 0..first.len().max(second.len()) {
        let value = first.get(degree).cloned().unwrap_or_else(Real::zero)
            - second.get(degree).cloned().unwrap_or_else(Real::zero);
        integral = &integral + (value / Real::from((degree + 1) as i32))?;
    }
    Ok(integral)
}

fn integrate_polynomial(coefficients: &[Real]) -> CurveResult<Real> {
    let mut integral = Real::zero();
    for (degree, coefficient) in coefficients.iter().enumerate() {
        integral = &integral + (coefficient.clone() / Real::from((degree + 1) as i32))?;
    }
    Ok(integral)
}

fn bernstein_to_power(values: Vec<Real>) -> Vec<Real> {
    let degree = values.len() - 1;
    let mut coeffs = vec![Real::zero(); values.len()];
    for (i, value) in values.into_iter().enumerate() {
        for (k, coefficient) in coeffs.iter_mut().enumerate().take(degree + 1).skip(i) {
            let magnitude = binomial(degree, i) * binomial(degree - i, k - i);
            let signed = if (k - i) % 2 == 0 {
                magnitude as i32
            } else {
                -(magnitude as i32)
            };
            *coefficient = &*coefficient + (&value * &Real::from(signed));
        }
    }
    coeffs
}

fn derivative_coefficients(coefficients: &[Real]) -> Vec<Real> {
    coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(degree, coefficient)| coefficient * &Real::from(degree as i32))
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

fn subdivide_controls_at(controls: &[Point2], t: Real) -> (Vec<Point2>, Vec<Point2>) {
    let mut levels = vec![controls.to_vec()];
    while levels.last().map(|level| level.len()).unwrap_or(0) > 1 {
        let previous = levels.last().expect("level exists");
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
    (left, right)
}

fn binomial(n: usize, k: usize) -> usize {
    match (n, k) {
        (_, 0) => 1,
        (n, k) if n == k => 1,
        (2, 1) => 2,
        (3, 1 | 2) => 3,
        _ => unreachable!("Bezier moment support is currently quadratic/cubic"),
    }
}
