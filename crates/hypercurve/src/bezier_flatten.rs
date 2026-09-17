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

use crate::classify::{compare_reals, is_zero, orient2_real_expr, real_sign};
use crate::{
    BezierSubcurve2, Classification, CubicBezier2, Curve2, CurveContext, CurveError, CurvePath2,
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
    pub fn try_new(max_error: Real, max_depth: usize, policy: &CurveContext) -> CurveResult<Self> {
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
        policy: &CurveContext,
    ) -> Classification<CertifiedBezierPolyline2> {
        flatten_curve(self.clone(), options, policy)
    }
}

impl CubicBezier2 {
    /// Flattens this cubic Bezier only after exact flatness certification.
    pub fn flatten_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> Classification<CertifiedBezierPolyline2> {
        flatten_curve(self.clone(), options, policy)
    }
}

impl BezierSubcurve2 {
    /// Flattens any materialized polynomial or rational Bezier span to exact-scalar chords.
    ///
    /// Rational spans must have a certified nonzero denominator. Subdivision
    /// establishes a finite control hull separately for each emitted chord.
    pub fn flatten_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> Classification<CertifiedBezierPolyline2> {
        flatten_curve(self.clone(), options, policy)
    }
}

impl Curve2 {
    /// Segments this top-level curve into exact-scalar chords with a certified error bound.
    pub fn segment_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CertifiedCurvePolyline2>> {
        segment_curves(std::slice::from_ref(self), options, policy)
    }
}

impl CurvePath2 {
    /// Segments every retained span in this path without converting coordinates to `f64`.
    pub fn segment_certified(
        &self,
        options: &BezierFlatteningOptions,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<CertifiedCurvePolyline2>> {
        segment_curves(self.curves(), options, policy)
    }
}

trait FlattenableBezier: Clone {
    fn start(&self) -> &Point2;
    fn end(&self) -> &Point2;
    fn controls(&self, policy: &CurveContext) -> Option<Vec<&Point2>>;
    fn split_half(&self, policy: &CurveContext) -> Result<(Self, Self), UncertaintyReason>;
    fn certify_finite_domain(&self) -> Result<(), UncertaintyReason> {
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

    fn controls(&self, _policy: &CurveContext) -> Option<Vec<&Point2>> {
        Some(self.control_points().into_iter().collect())
    }

    fn split_half(&self, _policy: &CurveContext) -> Result<(Self, Self), UncertaintyReason> {
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

    fn controls(&self, _policy: &CurveContext) -> Option<Vec<&Point2>> {
        Some(self.control_points().into_iter().collect())
    }

    fn split_half(&self, _policy: &CurveContext) -> Result<(Self, Self), UncertaintyReason> {
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

    fn controls(&self, policy: &CurveContext) -> Option<Vec<&Point2>> {
        Some(match self {
            Self::Quadratic(curve) => curve.control_points().into_iter().collect(),
            Self::Cubic(curve) => curve.control_points().into_iter().collect(),
            Self::RationalQuadratic(curve) => {
                let weights = curve.weights();
                let sign = real_sign(weights[0], policy)?;
                if sign == RealSign::Zero
                    || weights[1..]
                        .iter()
                        .any(|weight| real_sign(weight, policy) != Some(sign))
                {
                    return None;
                }
                curve.control_points().into_iter().collect()
            }
            Self::Rational(curve) => {
                if !matches!(curve.control_weight_sign(), Classification::Decided(_)) {
                    return None;
                }
                curve.affine_control_points()?.iter().collect()
            }
        })
    }

    fn split_half(&self, policy: &CurveContext) -> Result<(Self, Self), UncertaintyReason> {
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

    fn certify_finite_domain(&self) -> Result<(), UncertaintyReason> {
        let sign = match self {
            Self::Quadratic(_) | Self::Cubic(_) => return Ok(()),
            Self::RationalQuadratic(curve) => crate::RationalBezier2::from(curve.clone())
                .denominator_sign(&crate::CurveParameterRange2::unit()),
            Self::Rational(curve) => curve.denominator_sign(&crate::CurveParameterRange2::unit()),
        };
        match sign {
            Classification::Decided(RealSign::Positive | RealSign::Negative) => Ok(()),
            Classification::Decided(RealSign::Zero) => Err(UncertaintyReason::Boundary),
            Classification::Uncertain(reason) => Err(reason),
        }
    }
}

fn flatten_curve<C>(
    curve: C,
    options: &BezierFlatteningOptions,
    policy: &CurveContext,
) -> Classification<CertifiedBezierPolyline2>
where
    C: FlattenableBezier,
{
    if let Err(reason) = curve.certify_finite_domain() {
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

fn segment_curves(
    curves: &[Curve2],
    options: &BezierFlatteningOptions,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<CertifiedCurvePolyline2>> {
    let mut points = Vec::new();
    let mut max_depth = 0_usize;
    let mut source_fragment_count = 0;
    let mut append = |point: &Point2| {
        if points.last() != Some(point) {
            points.push(point.clone());
        }
    };
    for curve in curves {
        // The line image is sufficient for segmentation. Keep the retained
        // chord's parameter chart on the curve; no native parameter map is
        // invented for this output adapter.
        if let Some(crate::BezierSplitFragment2::AlgebraicChord(chord)) = curve.retained_fragment()
        {
            chord.validate_policy(policy).map_err(|cause| {
                crate::ExactCurveError::invalid(
                    crate::CurveOperation2::Subdivision,
                    curve.family(),
                    cause,
                )
            })?;
            if let Some(line) = chord.exact_line() {
                append(line.start());
                append(line.end());
                source_fragment_count += 1;
                continue;
            }
        }
        let fragments = match curve.native_bezier_fragments_with_policy(policy)? {
            Classification::Decided(fragments) => fragments,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        source_fragment_count += fragments.len();
        for fragment in fragments {
            let polyline = match fragment.curve().flatten_certified(options, policy) {
                Classification::Decided(polyline) => polyline,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            max_depth = max_depth.max(polyline.certificate().max_depth());
            for point in polyline.points() {
                append(point);
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
        source_fragment_count,
    }))
}

fn flatten_recursive<C>(
    curve: C,
    max_error_squared: &Real,
    max_depth: usize,
    depth: usize,
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> Result<bool, UncertaintyReason>
where
    C: FlattenableBezier,
{
    let Some(controls) = curve.controls(policy) else {
        return Ok(false);
    };
    if is_zero(&curve.start().distance_squared(curve.end()), policy) == Some(true) {
        for point in &controls {
            if !squared_distance_within(point, curve.start(), max_error_squared, policy)? {
                return Ok(false);
            }
        }
        return Ok(true);
    }

    let chord_length_squared = curve.start().distance_squared(curve.end());
    let threshold = max_error_squared * &chord_length_squared;
    let dx = curve.end().x() - curve.start().x();
    let dy = curve.end().y() - curve.start().y();
    for point in controls.into_iter().skip(1).rev().skip(1) {
        // Distance to the supporting line alone misses collinear overshoot.
        // The convex capsule around the finite chord must contain each control.
        let along = (point.x() - curve.start().x()) * &dx + (point.y() - curve.start().y()) * &dy;
        let endpoint = match compare_reals(&along, &Real::zero(), policy) {
            Some(Ordering::Less) => Some(curve.start()),
            Some(_) => match compare_reals(&along, &chord_length_squared, policy) {
                Some(Ordering::Greater) => Some(curve.end()),
                Some(_) => None,
                None => return Err(UncertaintyReason::Ordering),
            },
            None => return Err(UncertaintyReason::Ordering),
        };
        if let Some(endpoint) = endpoint {
            if !squared_distance_within(point, endpoint, max_error_squared, policy)? {
                return Ok(false);
            }
            continue;
        }
        let signed_area = orient2_real_expr(curve.start(), curve.end(), point);
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
    policy: &CurveContext,
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
        let (left, right) = curve.split_half(&CurveContext::STRICT).unwrap();

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
    fn flatten_certificate_evidence_actual_depth_used() {
        let policy = CurveContext::STRICT;
        let options = BezierFlatteningOptions::try_new(Real::one(), 8, &policy).unwrap();
        let curve = QuadraticBezier2::new(point(0, 0), point(1, 0), point(2, 0));

        let Classification::Decided(polyline) = curve.flatten_certified(&options, &policy) else {
            panic!("flat line-image quadratic should certify without subdivision");
        };

        assert_eq!(polyline.points(), &[point(0, 0), point(2, 0)]);
        assert_eq!(polyline.certificate().segment_count(), 1);
        assert_eq!(polyline.certificate().max_depth(), 0);
    }

    #[test]
    fn homogeneous_representation_flattens_finite_mixed_controls() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let half = (Real::one() / Real::from(2)).unwrap();
            let options = BezierFlatteningOptions::try_new(half.clone(), 12, &policy).unwrap();
            let curve = crate::RationalBezier2::try_new(
                vec![point(0, 0), point(1, 1), point(2, 0)],
                vec![Real::one(), -half, Real::one()],
            )
            .unwrap();
            let elevated = curve.elevated_to_degree(3).unwrap();
            for curve in [curve, elevated] {
                let span = BezierSubcurve2::Rational(curve);
                let Classification::Decided(polyline) = span.flatten_certified(&options, &policy)
                else {
                    panic!("a finite mixed-weight curve must admit local hulls");
                };
                assert_eq!(polyline.points().first(), Some(&point(0, 0)));
                assert_eq!(polyline.points().last(), Some(&point(2, 0)));
                assert!(polyline.points().contains(&point(1, -1)));
                assert!(polyline.certificate().max_depth() > 0);
            }
            let pole = BezierSubcurve2::Rational(
                crate::RationalBezier2::try_new(
                    vec![point(0, 0), point(1, 1), point(2, 0)],
                    vec![Real::one(), Real::from(-2), Real::one()],
                )
                .unwrap(),
            );
            assert!(matches!(
                pole.flatten_certified(&options, &policy),
                Classification::Uncertain(_)
            ));
        }
    }

    #[test]
    fn flatness_certificate_covers_collinear_overshoot() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let options = BezierFlatteningOptions::try_new(
                (Real::one() / Real::from(16)).unwrap(),
                12,
                &policy,
            )
            .unwrap();
            let curve = QuadraticBezier2::new(point(0, 0), point(4, 0), point(1, 0));
            let Classification::Decided(polyline) = curve.flatten_certified(&options, &policy)
            else {
                panic!("a retracing quadratic must admit certified finite chords");
            };
            assert!(polyline.points().iter().any(|point| {
                compare_reals(point.x(), &Real::from(2), &policy) == Some(Ordering::Greater)
            }));
            assert!(polyline.certificate().segment_count() > 1);
        }
    }
}
