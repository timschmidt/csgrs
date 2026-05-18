//! Certified flattening adapters for polynomial Bezier segments.
//!
//! Flattening is an output adapter, not a topology kernel. The code below only
//! emits a polyline after exact predicates certify that each Bezier sub-curve's
//! control hull is within the requested distance of its chord. This keeps the
//! branch boundary aligned with Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997). The recursive hull-to-chord test is
//! the standard Bezier flatness criterion discussed by Raph Levien,
//! "Flattening quadratic Beziers" (2019), with exact signs replacing floating
//! tolerances.

use std::cmp::Ordering;

use hyperreal::{Real, RealSign};

use crate::classify::{
    classify_oriented_line, compare_reals, is_zero, orient2d_real_expr, real_sign,
};
use crate::{
    Aabb2, Classification, CubicBezier2, CurveError, CurvePolicy, CurveResult, CurveString2,
    LineSeg2, LineSide, Point2, QuadraticBezier2, Segment2, UncertaintyReason,
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

/// Error metric used by certified Bezier-polyline simplification.
///
/// The current simplifier only removes vertices when exact predicates prove
/// that the polyline image is unchanged. Naming the metric now keeps the API
/// ready for later bounded simplifiers that may use Hausdorff or
/// Fréchet-like contracts, as discussed by Raph Levien, "Simplifying Bezier
/// paths" (2021), while keeping Yap's proof obligations explicit at the
/// approximation boundary; see Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierSimplificationErrorMetric {
    /// Exact distance between the source polyline image and simplified image.
    ExactPolylineImageDistance,
}

/// Proof status for a certified Bezier-polyline simplification bound.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierSimplificationBoundKind {
    /// The simplification error bound is proved exactly by symbolic predicates.
    ProvenExact,
}

/// Certificate attached to a simplified certified Bezier polyline.
///
/// The source range is half-open over the input polyline vertices passed to
/// [`CertifiedBezierPolyline2::simplify_exact_collinear`]. `error_bound` is
/// the simplification error relative to that input polyline image, while
/// `source_flattening_error` preserves the independent curve-to-polyline
/// flattening budget and `construction_policy` records the predicate policy
/// used to prove every removed vertex was redundant. Keeping these facts
/// separate follows Yap's exact geometric-computation discipline and avoids
/// treating a simplified approximation as a new exact curve.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierSimplificationCertificate {
    source_start: usize,
    source_end: usize,
    retained_vertex_count: usize,
    removed_vertex_count: usize,
    error_bound: Real,
    source_flattening_error: Real,
    source_flattening_max_depth: usize,
    construction_policy: CurvePolicy,
    metric: BezierSimplificationErrorMetric,
    bound_kind: BezierSimplificationBoundKind,
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

impl BezierSimplificationCertificate {
    /// Builds a proven zero-error collinear-simplification certificate.
    fn proven_exact_collinear(
        source_vertex_count: usize,
        retained_vertex_count: usize,
        source_flattening_error: Real,
        source_flattening_max_depth: usize,
        construction_policy: &CurvePolicy,
    ) -> Self {
        Self {
            source_start: 0,
            source_end: source_vertex_count,
            retained_vertex_count,
            removed_vertex_count: source_vertex_count.saturating_sub(retained_vertex_count),
            error_bound: Real::zero(),
            source_flattening_error,
            source_flattening_max_depth,
            construction_policy: construction_policy.clone(),
            metric: BezierSimplificationErrorMetric::ExactPolylineImageDistance,
            bound_kind: BezierSimplificationBoundKind::ProvenExact,
        }
    }

    /// Returns the half-open start index of the simplified source range.
    pub const fn source_start(&self) -> usize {
        self.source_start
    }

    /// Returns the half-open end index of the simplified source range.
    pub const fn source_end(&self) -> usize {
        self.source_end
    }

    /// Returns the number of vertices retained by the simplification.
    pub const fn retained_vertex_count(&self) -> usize {
        self.retained_vertex_count
    }

    /// Returns the number of source vertices removed by the simplification.
    pub const fn removed_vertex_count(&self) -> usize {
        self.removed_vertex_count
    }

    /// Returns the proven simplification error bound.
    pub const fn error_bound(&self) -> &Real {
        &self.error_bound
    }

    /// Returns the source curve-to-polyline flattening error.
    pub const fn source_flattening_error(&self) -> &Real {
        &self.source_flattening_error
    }

    /// Returns the source flattening recursion budget inherited by this product.
    pub const fn source_flattening_max_depth(&self) -> usize {
        self.source_flattening_max_depth
    }

    /// Returns the policy snapshot used to prove this simplification.
    pub const fn construction_policy(&self) -> &CurvePolicy {
        &self.construction_policy
    }

    /// Returns the simplification error metric.
    pub const fn metric(&self) -> BezierSimplificationErrorMetric {
        self.metric
    }

    /// Returns whether the simplification bound is proven exactly.
    pub const fn bound_kind(&self) -> BezierSimplificationBoundKind {
        self.bound_kind
    }
}

/// A polyline produced by certified Bezier flattening.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierPolyline2 {
    points: Vec<Point2>,
    certificate: BezierFlatteningCertificate,
    simplification_certificate: Option<BezierSimplificationCertificate>,
}

/// Display-only offset preview for a certified flattened Bezier polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct DisplayBezierOffsetPolyline2 {
    segments: Vec<LineSeg2>,
    source_certificate: BezierFlatteningCertificate,
    distance: Real,
}

/// Checked offset of a certified flattened Bezier polyline.
///
/// This is still an approximation-boundary product: the source is a certified
/// polyline view of a Bezier, not the analytic Bezier offset curve. Unlike
/// [`DisplayBezierOffsetPolyline2`], this adapter routes the polyline through
/// the crate's checked curve-string offset path, so self-contacting raw offset
/// output is reported as explicit uncertainty. That keeps the branch boundary
/// aligned with Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997). The primitive/join/trim staging follows Tiller and
/// Hanson, "Offsets of Two-Dimensional Profiles" (1984), and the rejection of
/// self-intersecting raw offsets follows Farouki and Neff, "Analytic
/// Properties of Plane Offset Curves" (1990).
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierPolylineOffset2 {
    curve: CurveString2,
    source_certificate: BezierFlatteningCertificate,
    distance: Real,
}

impl DisplayBezierOffsetPolyline2 {
    /// Returns the offset chord segments.
    pub fn segments(&self) -> &[LineSeg2] {
        &self.segments
    }

    /// Returns the flattening certificate for the source polyline.
    pub const fn source_certificate(&self) -> &BezierFlatteningCertificate {
        &self.source_certificate
    }

    /// Returns the signed display offset distance along each chord's left normal.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }
}

impl CertifiedBezierPolylineOffset2 {
    /// Returns the checked joined offset curve string.
    pub const fn curve(&self) -> &CurveString2 {
        &self.curve
    }

    /// Returns the flattening certificate for the source polyline.
    pub const fn source_certificate(&self) -> &BezierFlatteningCertificate {
        &self.source_certificate
    }

    /// Returns the signed checked offset distance along each chord's left normal.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }
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

    /// Returns the simplification certificate when this polyline came from a
    /// certified simplification adapter.
    pub const fn simplification_certificate(&self) -> Option<&BezierSimplificationCertificate> {
        self.simplification_certificate.as_ref()
    }

    /// Removes exactly collinear interior vertices without increasing error.
    ///
    /// This is a certified simplification adapter: it only drops a vertex when
    /// exact orientation proves the adjacent chord endpoints and the candidate
    /// vertex are collinear, and exact box containment proves the candidate
    /// lies between those endpoints. It is the zero-error subset of path
    /// simplification ideas discussed by Raph Levien, "Simplifying Bezier
    /// paths" (2021), with Yap's (1997) exact branch boundary replacing
    /// tolerance-based Ramer-Douglas-Peucker decisions.
    pub fn simplify_exact_collinear(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<CertifiedBezierPolyline2> {
        if self.points.len() <= 2 {
            return Classification::Decided(CertifiedBezierPolyline2 {
                points: self.points.clone(),
                certificate: self.certificate.clone(),
                simplification_certificate: Some(
                    BezierSimplificationCertificate::proven_exact_collinear(
                        self.points.len(),
                        self.points.len(),
                        self.certificate.max_error.clone(),
                        self.certificate.max_depth,
                        policy,
                    ),
                ),
            });
        }

        let mut simplified = Vec::with_capacity(self.points.len());
        simplified.push(self.points[0].clone());
        for index in 1..self.points.len() - 1 {
            let point = &self.points[index];
            let previous = simplified
                .last()
                .expect("simplified always contains the start vertex");
            let next = &self.points[index + 1];
            match can_remove_collinear_vertex(previous, point, next, policy) {
                Classification::Decided(true) => {}
                Classification::Decided(false) => simplified.push(point.clone()),
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        simplified.push(
            self.points
                .last()
                .expect("polyline contains at least two vertices")
                .clone(),
        );
        let segment_count = simplified.len().saturating_sub(1);
        Classification::Decided(CertifiedBezierPolyline2 {
            points: simplified,
            certificate: BezierFlatteningCertificate {
                max_error: self.certificate.max_error.clone(),
                segment_count,
                max_depth: self.certificate.max_depth,
            },
            simplification_certificate: Some(
                BezierSimplificationCertificate::proven_exact_collinear(
                    self.points.len(),
                    segment_count + 1,
                    self.certificate.max_error.clone(),
                    self.certificate.max_depth,
                    policy,
                ),
            ),
        })
    }

    /// Builds a display-only left-offset preview from the certified chords.
    ///
    /// This deliberately does not claim a Bezier offset topology result. It
    /// offsets each certified chord independently and leaves joins/trimming to
    /// higher layers. That matches the staged offset framing in Tiller and
    /// Hanson, "Offsets of Two-Dimensional Profiles" (1984), and the
    /// approximation caveats discussed by Raph Levien, "Parallel curves of
    /// cubic Beziers" (2022). In Yap's EGC terminology this is an approximate
    /// view attached to a certificate, not a combinatorial predicate.
    pub fn display_offset_left(&self, distance: Real) -> CurveResult<DisplayBezierOffsetPolyline2> {
        let mut segments = Vec::new();
        for window in self.points.windows(2) {
            let segment = match LineSeg2::try_new(window[0].clone(), window[1].clone()) {
                Ok(segment) => segment,
                Err(CurveError::ZeroLengthLine) => continue,
                Err(error) => return Err(error),
            };
            segments.push(segment.offset_left(distance.clone())?);
        }
        Ok(DisplayBezierOffsetPolyline2 {
            segments,
            source_certificate: self.certificate.clone(),
            distance,
        })
    }

    /// Builds a display-only right-offset preview from the certified chords.
    ///
    /// Right offsets are represented as negative signed left-normal distances,
    /// the same convention used by exact primitive Bezier line offsets. This is
    /// still only a display adapter over the certified flattened view: it keeps
    /// the source certificate but does not claim analytic Bezier offset
    /// topology. That separation follows Yap's EGC boundary and the
    /// Tiller-Hanson offset staging cited on [`Self::display_offset_left`].
    pub fn display_offset_right(
        &self,
        distance: Real,
    ) -> CurveResult<DisplayBezierOffsetPolyline2> {
        self.display_offset_left(-distance)
    }

    /// Builds a checked left-offset adapter from the certified flattened polyline.
    ///
    /// Each retained chord becomes an exact line segment, then the existing
    /// curve-string offset pipeline applies primitive line offsets, joins
    /// adjacent offset segments, and rejects self-contacting output. This is a
    /// certified approximation adapter, not an analytic Bezier offset: callers
    /// get the source flattening certificate plus an explicit uncertainty when
    /// the raw offset topology needs the later trim/rebuild stage described by
    /// Tiller-Hanson (1984) and Farouki-Neff (1990). Yap's exact-geometric
    /// computation discipline is preserved because all topology decisions are
    /// delegated to exact curve-string predicates.
    pub fn checked_offset_left(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CertifiedBezierPolylineOffset2>> {
        let source = certified_polyline_curve_string(self)?;
        match source.offset_left_checked(distance.clone(), policy)? {
            Classification::Decided(curve) => {
                Ok(Classification::Decided(CertifiedBezierPolylineOffset2 {
                    curve,
                    source_certificate: self.certificate.clone(),
                    distance,
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Builds a checked right-offset adapter from the certified flattened polyline.
    ///
    /// This uses the same exact curve-string predicate path as
    /// [`Self::checked_offset_left`] and stores the result as a negative signed
    /// left-normal distance. The returned product is therefore comparable to
    /// left offsets while remaining an approximation-boundary product of the
    /// flattened view, not an analytic Bezier offset curve.
    pub fn checked_offset_right(
        &self,
        distance: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<CertifiedBezierPolylineOffset2>> {
        self.checked_offset_left(-distance, policy)
    }
}

fn certified_polyline_curve_string(
    polyline: &CertifiedBezierPolyline2,
) -> CurveResult<CurveString2> {
    let mut segments = Vec::new();
    for window in polyline.points.windows(2) {
        match LineSeg2::try_new(window[0].clone(), window[1].clone()) {
            Ok(segment) => segments.push(Segment2::Line(segment)),
            Err(CurveError::ZeroLengthLine) => {}
            Err(error) => return Err(error),
        }
    }
    if segments.is_empty() {
        return Err(CurveError::ZeroLengthLine);
    }
    CurveString2::try_new(segments)
}

fn can_remove_collinear_vertex(
    previous: &Point2,
    point: &Point2,
    next: &Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    match classify_oriented_line(previous, next, point, policy) {
        Classification::Decided(LineSide::On) => {}
        Classification::Decided(_) => return Classification::Decided(false),
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    let envelope = match Aabb2::from_points([previous, next], policy) {
        Classification::Decided(bbox) => bbox,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    envelope.contains_point(point, policy)
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

trait FlattenableBezier: Clone {
    fn start(&self) -> &Point2;
    fn end(&self) -> &Point2;
    fn controls(&self) -> Vec<&Point2>;
    fn split_half(&self) -> (Self, Self);
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

    fn split_half(&self) -> (Self, Self) {
        let half = half();
        let p01 = self.start().lerp(self.control(), half.clone());
        let p12 = self.control().lerp(self.end(), half);
        let mid = midpoint_point(&p01, &p12);
        (
            QuadraticBezier2::new(self.start().clone(), p01, mid.clone()),
            QuadraticBezier2::new(mid, p12, self.end().clone()),
        )
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

    fn split_half(&self) -> (Self, Self) {
        let half = half();
        let p01 = self.start().lerp(self.control1(), half.clone());
        let p12 = self.control1().lerp(self.control2(), half.clone());
        let p23 = self.control2().lerp(self.end(), half);
        let p012 = midpoint_point(&p01, &p12);
        let p123 = midpoint_point(&p12, &p23);
        let mid = midpoint_point(&p012, &p123);
        (
            CubicBezier2::new(self.start().clone(), p01, p012, mid.clone()),
            CubicBezier2::new(mid, p123, p23, self.end().clone()),
        )
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
    let mut points = vec![curve.start().clone()];
    let max_error_squared = options.max_error() * options.max_error();
    if let Err(reason) = flatten_recursive(
        curve,
        &max_error_squared,
        options.max_depth(),
        0,
        policy,
        &mut points,
    ) {
        return Classification::Uncertain(reason);
    }
    let segment_count = points.len().saturating_sub(1);
    Classification::Decided(CertifiedBezierPolyline2 {
        points,
        certificate: BezierFlatteningCertificate {
            max_error: options.max_error().clone(),
            segment_count,
            max_depth: options.max_depth(),
        },
        simplification_certificate: None,
    })
}

fn flatten_recursive<C>(
    curve: C,
    max_error_squared: &Real,
    max_depth: usize,
    depth: usize,
    policy: &CurvePolicy,
    points: &mut Vec<Point2>,
) -> Result<(), UncertaintyReason>
where
    C: FlattenableBezier,
{
    if curve_is_flat(&curve, max_error_squared, policy)? {
        points.push(curve.end().clone());
        return Ok(());
    }
    if depth >= max_depth {
        return Err(UncertaintyReason::Unsupported);
    }
    let (left, right) = curve.split_half();
    flatten_recursive(
        left,
        max_error_squared,
        max_depth,
        depth + 1,
        policy,
        points,
    )?;
    flatten_recursive(
        right,
        max_error_squared,
        max_depth,
        depth + 1,
        policy,
        points,
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

fn midpoint_point(first: &Point2, second: &Point2) -> Point2 {
    first.lerp(second, half())
}

fn half() -> Real {
    (Real::one() / Real::from(2_i8)).expect("division by positive integer constant is defined")
}
