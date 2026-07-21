//! Certified flattening adapters for polynomial Bezier segments.
//!
//! Flattening is an output adapter, not a topology kernel. The code below only
//! emits a polyline after exact predicates certify that each Bezier sub-curve's
//! control hull is within the requested distance of its chord. This keeps the
//! branch boundary aligned with exact-computation discipline. The recursive hull-to-chord test is
//! the standard Bezier flatness criterion discussed by Raph Bezier approximation analysis, with exact signs replacing floating
//! tolerances.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::classify::{compare_reals, is_zero, orient2d_real_expr, real_sign};
use crate::{
    BezierSubcurve2, Classification, CubicBezier2, Curve2, CurveError, CurvePath2, CurvePolicy,
    CurveResult, ExactCurveResult, Point2, QuadraticBezier2, UncertaintyReason,
};

/// Options for certified Bezier-to-polyline flattening.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierFlatteningOptions {
    max_error: Real,
    max_depth: usize,
}

impl BezierFlatteningOptions {
    /// Constructs flattening options after certifying a positive error budget.
    pub fn try_new(max_error: Real, max_depth: usize, policy: &CurvePolicy) -> CurveResult<Self> {
        if max_depth == 0 {
            return Err(CurveError::InvalidFlatteningOptions);
        }
        match real_sign(&max_error, policy) {
            Some(RealSign::Positive) => Ok(Self {
                max_error,
                max_depth,
            }),
            Some(RealSign::Zero | RealSign::Negative) | None => {
                Err(CurveError::InvalidFlatteningOptions)
            }
        }
    }

    /// Returns the certified maximum distance from curve to emitted chord.
    pub const fn max_error(&self) -> &Real {
        &self.max_error
    }

    /// Returns the maximum recursive subdivision depth.
    pub const fn max_depth(&self) -> usize {
        self.max_depth
    }
}

/// Certificate attached to a flattened Bezier polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierFlatteningCertificate {
    max_error: Real,
    segment_count: usize,
    max_depth: usize,
}

impl BezierFlatteningCertificate {
    /// Returns the requested maximum curve-to-chord distance.
    pub const fn max_error(&self) -> &Real {
        &self.max_error
    }

    /// Returns the number of certified chord segments.
    pub const fn segment_count(&self) -> usize {
        self.segment_count
    }

    /// Returns the maximum recursive subdivision depth used by flattening.
    pub const fn max_depth(&self) -> usize {
        self.max_depth
    }
}

/// A polyline produced by certified Bezier flattening.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierPolyline2 {
    points: Vec<Point2>,
    certificate: BezierFlatteningCertificate,
}

/// An exact-scalar polyline produced by certified subdivision of a top-level curve or path.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedCurvePolyline2 {
    points: Vec<Point2>,
    certificate: BezierFlatteningCertificate,
    source_fragment_count: usize,
}

impl CertifiedBezierPolyline2 {
    /// Returns the emitted polyline vertices.
    pub fn points(&self) -> &[Point2] {
        &self.points
    }

    /// Returns the flattening certificate.
    pub const fn certificate(&self) -> &BezierFlatteningCertificate {
        &self.certificate
    }
}

impl CertifiedCurvePolyline2 {
    /// Returns exact `Real` vertices; no finite scalar conversion occurs.
    pub fn points(&self) -> &[Point2] {
        &self.points
    }

    /// Returns the aggregate certified chord-error bound and subdivision depth.
    pub const fn certificate(&self) -> &BezierFlatteningCertificate {
        &self.certificate
    }

    /// Returns the number of native Bezier/conic spans covered by the certificate.
    pub const fn source_fragment_count(&self) -> usize {
        self.source_fragment_count
    }
}

impl QuadraticBezier2 {
    /// Flattens this quadratic Bezier only after exact flatness certification.
    pub fn flatten_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> Classification<CertifiedBezierPolyline2> {
        flatten_curve(self.clone(), options, policy)
    }
}

impl CubicBezier2 {
    /// Flattens this cubic Bezier only after exact flatness certification.
    pub fn flatten_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> Classification<CertifiedBezierPolyline2> {
        flatten_curve(self.clone(), options, policy)
    }
}

impl BezierSubcurve2 {
    /// Flattens any materialized polynomial or rational Bezier span to exact-scalar chords.
    ///
    /// Rational spans are accepted only when every weight has one certified
    /// nonzero sign, which proves the affine image remains in its control hull.
    pub fn flatten_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> Classification<CertifiedBezierPolyline2> {
        flatten_curve(self.clone(), options, policy)
    }
}

impl Curve2 {
    /// Segments this top-level curve into exact-scalar chords with a certified error bound.
    pub fn segment_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<CertifiedCurvePolyline2>> {
        segment_native_fragments(self.native_bezier_fragments()?, options, policy)
    }
}

impl CurvePath2 {
    /// Segments every retained span in this path without converting coordinates to `f64`.
    pub fn segment_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Classification<CertifiedCurvePolyline2>> {
        segment_native_fragments(self.native_bezier_fragments()?, options, policy)
    }
}

trait FlattenableBezier: Clone {
    fn start(&self) -> &Point2;
    fn end(&self) -> &Point2;
    fn controls(&self) -> Vec<&Point2>;
    fn split_half(&self, policy: &CurvePolicy) -> Result<(Self, Self), UncertaintyReason>;
    fn certify_convex_hull(&self, _policy: &CurvePolicy) -> Result<(), UncertaintyReason> {
        Ok(())
    }
}

impl FlattenableBezier for QuadraticBezier2 {
    fn start(&self) -> &Point2 {
        self.start()
    }

    fn end(&self) -> &Point2 {
        self.end()
    }

    fn controls(&self) -> Vec<&Point2> {
        self.control_points().into_iter().collect()
    }

    fn split_half(&self, _policy: &CurvePolicy) -> Result<(Self, Self), UncertaintyReason> {
        Ok(self.split_at_exact(half()?))
    }
}

impl FlattenableBezier for CubicBezier2 {
    fn start(&self) -> &Point2 {
        self.start()
    }

    fn end(&self) -> &Point2 {
        self.end()
    }

    fn controls(&self) -> Vec<&Point2> {
        self.control_points().into_iter().collect()
    }

    fn split_half(&self, _policy: &CurvePolicy) -> Result<(Self, Self), UncertaintyReason> {
        Ok(self.split_at_exact(half()?))
    }
}

impl FlattenableBezier for BezierSubcurve2 {
    fn start(&self) -> &Point2 {
        self.start()
    }

    fn end(&self) -> &Point2 {
        self.end()
    }

    fn controls(&self) -> Vec<&Point2> {
        match self {
            Self::Quadratic(curve) => curve.control_points().into_iter().collect(),
            Self::Cubic(curve) => curve.control_points().into_iter().collect(),
            Self::RationalQuadratic(curve) => curve.control_points().into_iter().collect(),
            Self::Rational(curve) => curve.control_points().iter().collect(),
        }
    }

    fn split_half(&self, policy: &CurvePolicy) -> Result<(Self, Self), UncertaintyReason> {
        let half = half()?;
        let left = match self.subcurve_between_exact(&Real::zero(), &half, policy) {
            Ok(Classification::Decided(curve)) => curve,
            Ok(Classification::Uncertain(reason)) => return Err(reason),
            Err(_) => return Err(UncertaintyReason::Unsupported),
        };
        let right = match self.subcurve_between_exact(&half, &Real::one(), policy) {
            Ok(Classification::Decided(curve)) => curve,
            Ok(Classification::Uncertain(reason)) => return Err(reason),
            Err(_) => return Err(UncertaintyReason::Unsupported),
        };
        Ok((left, right))
    }

    fn certify_convex_hull(&self, policy: &CurvePolicy) -> Result<(), UncertaintyReason> {
        let weights = match self {
            Self::Quadratic(_) | Self::Cubic(_) => return Ok(()),
            Self::RationalQuadratic(curve) => curve.weights().into_iter().collect::<Vec<_>>(),
            Self::Rational(curve) => curve.weights().iter().collect::<Vec<_>>(),
        };
        let mut expected = None;
        for weight in weights {
            let sign = real_sign(weight, policy).ok_or(UncertaintyReason::RealSign)?;
            match sign {
                RealSign::Zero => return Err(UncertaintyReason::Unsupported),
                RealSign::Positive | RealSign::Negative => {
                    if expected.is_some_and(|expected| expected != sign) {
                        return Err(UncertaintyReason::Unsupported);
                    }
                    expected = Some(sign);
                }
            }
        }
        Ok(())
    }
}

fn flatten_curve<C>(
    curve: C,
    options: &BezierFlatteningOptions,
    policy: &CurvePolicy,
) -> Classification<CertifiedBezierPolyline2>
where
    C: FlattenableBezier,
{
    if let Err(reason) = curve.certify_convex_hull(policy) {
        return Classification::Uncertain(reason);
    }
    let mut points = vec![curve.start().clone()];
    let max_error_squared = options.max_error() * options.max_error();
    let mut max_depth_used = 0_usize;
    if let Err(reason) = flatten_recursive(
        curve,
        &max_error_squared,
        options.max_depth(),
        0,
        policy,
        &mut points,
        &mut max_depth_used,
    ) {
        return Classification::Uncertain(reason);
    }
    let segment_count = points.len().saturating_sub(1);
    Classification::Decided(CertifiedBezierPolyline2 {
        points,
        certificate: BezierFlatteningCertificate {
            max_error: options.max_error().clone(),
            segment_count,
            max_depth: max_depth_used,
        },
    })
}

fn segment_native_fragments(
    fragments: &[crate::NativeBezierFragment2],
    options: &BezierFlatteningOptions,
    policy: &CurvePolicy,
) -> ExactCurveResult<Classification<CertifiedCurvePolyline2>> {
    let mut points = Vec::new();
    let mut max_depth = 0_usize;
    for fragment in fragments {
        let polyline = match fragment.curve().flatten_certified(options, policy) {
            Classification::Decided(polyline) => polyline,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        max_depth = max_depth.max(polyline.certificate().max_depth());
        for point in polyline.points() {
            if points.last() != Some(point) {
                points.push(point.clone());
            }
        }
    }
    let segment_count = points.len().saturating_sub(1);
    Ok(Classification::Decided(CertifiedCurvePolyline2 {
        points,
        certificate: BezierFlatteningCertificate {
            max_error: options.max_error().clone(),
            segment_count,
            max_depth,
        },
        source_fragment_count: fragments.len(),
    }))
}

fn flatten_recursive<C>(
    curve: C,
    max_error_squared: &Real,
    max_depth: usize,
    depth: usize,
    policy: &CurvePolicy,
    points: &mut Vec<Point2>,
    max_depth_used: &mut usize,
) -> Result<(), UncertaintyReason>
where
    C: FlattenableBezier,
{
    *max_depth_used = (*max_depth_used).max(depth);
    if curve_is_flat(&curve, max_error_squared, policy)? {
        points.push(curve.end().clone());
        return Ok(());
    }
    if depth >= max_depth {
        return Err(UncertaintyReason::Unsupported);
    }
    let (left, right) = curve.split_half(policy)?;
    flatten_recursive(
        left,
        max_error_squared,
        max_depth,
        depth + 1,
        policy,
        points,
        max_depth_used,
    )?;
    flatten_recursive(
        right,
        max_error_squared,
        max_depth,
        depth + 1,
        policy,
        points,
        max_depth_used,
    )
}

fn curve_is_flat<C>(
    curve: &C,
    max_error_squared: &Real,
    policy: &CurvePolicy,
) -> Result<bool, UncertaintyReason>
where
    C: FlattenableBezier,
{
    if is_zero(&curve.start().distance_squared(curve.end()), policy) == Some(true) {
        for point in curve.controls() {
            if !squared_distance_within(point, curve.start(), max_error_squared, policy)? {
                return Ok(false);
            }
        }
        return Ok(true);
    }

    let chord_length_squared = curve.start().distance_squared(curve.end());
    let threshold = max_error_squared * &chord_length_squared;
    for point in curve.controls().into_iter().skip(1).rev().skip(1) {
        let signed_area = orient2d_real_expr(curve.start(), curve.end(), point);
        let area_squared = &signed_area * &signed_area;
        match compare_reals(&area_squared, &threshold, policy) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => return Ok(false),
            None => return Err(UncertaintyReason::Ordering),
        }
    }
    Ok(true)
}

fn squared_distance_within(
    point: &Point2,
    center: &Point2,
    max_error_squared: &Real,
    policy: &CurvePolicy,
) -> Result<bool, UncertaintyReason> {
    match compare_reals(&point.distance_squared(center), max_error_squared, policy) {
        Some(Ordering::Less | Ordering::Equal) => Ok(true),
        Some(Ordering::Greater) => Ok(false),
        None => Err(UncertaintyReason::Ordering),
    }
}

fn half() -> Result<Real, UncertaintyReason> {
    (Real::one() / Real::from(2_i8)).map_err(|_| UncertaintyReason::Unsupported)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(Real::from(x), Real::from(y))
    }

    #[test]
    fn cubic_half_split_keeps_exact_de_casteljau_values() {
        let curve = CubicBezier2::new(point(0, 0), point(2, 0), point(4, 0), point(6, 0));
        let (left, right) = curve.split_half(&CurvePolicy::certified()).unwrap();

        let left_controls = left
            .control_points()
            .into_iter()
            .cloned()
            .collect::<Vec<_>>();
        let right_controls = right
            .control_points()
            .into_iter()
            .cloned()
            .collect::<Vec<_>>();
        assert_eq!(
            left_controls,
            vec![point(0, 0), point(1, 0), point(2, 0), point(3, 0)]
        );
        assert_eq!(
            right_controls,
            vec![point(3, 0), point(4, 0), point(5, 0), point(6, 0)]
        );
    }

    #[test]
    fn flatten_certificate_reports_actual_depth_used() {
        let policy = CurvePolicy::certified();
        let options = BezierFlatteningOptions::try_new(Real::one(), 8, &policy).unwrap();
        let curve = QuadraticBezier2::new(point(0, 0), point(1, 0), point(2, 0));

        let Classification::Decided(polyline) = curve.flatten_certified(&options, &policy) else {
            panic!("flat line-image quadratic should certify without subdivision");
        };

        assert_eq!(polyline.points(), &[point(0, 0), point(2, 0)]);
        assert_eq!(polyline.certificate().segment_count(), 1);
        assert_eq!(polyline.certificate().max_depth(), 0);
    }
}
