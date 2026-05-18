//! Certified fitting adapters for Bezier-derived boundary data.
//!
//! Fitting is an approximation boundary, not a topology shortcut. This module
//! starts with the zero-error case: a certified flattened Bezier polyline may
//! be fit back to a single exact line only when exact predicates prove every
//! retained vertex is collinear with, and between, the fitted endpoints. This
//! follows Yap's requirement that approximate views carry proof obligations;
//! see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997). Broader curve fitting should follow the error/certificate
//! discipline discussed by Raph Levien, "Fitting cubic Bezier curves" (2021).

use crate::classify::{classify_oriented_line, is_zero, real_sign};
use hyperreal::{Real, RealSign};

use crate::{
    Aabb2, BezierFlatteningCertificate, CertifiedBezierPolyline2, Classification, CubicBezier2,
    CurveError, CurvePolicy, CurveResult, LineSeg2, LineSide, Point2, QuadraticBezier2,
    RationalQuadraticBezier2, UncertaintyReason,
};

/// Error metric used by a certified Bezier fitting adapter.
///
/// The current fitting surface certifies the zero-error subset: every source
/// vertex or control point is proven exactly on the fitted primitive. The
/// metric is still named because later Levien-style fitting and simplification
/// passes need to report whether a bound is Hausdorff, Fréchet-like, moment
/// preserving, display-only, or another approximation contract. This follows
/// Yap's requirement that approximate/fitted views carry proof obligations at
/// their API boundary; see Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), and Raph Levien, "Fitting cubic
/// Bezier curves" (2021).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierFitErrorMetric {
    /// Exact Euclidean point-to-primitive distance with a zero bound.
    ExactEuclideanDistance,
}

/// Proof status for a certified Bezier fitting error bound.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierFitBoundKind {
    /// The bound is proved exactly by symbolic predicates.
    ProvenExact,
}

/// Certificate attached to a Bezier fitting product.
///
/// `source_range` is a half-open index range over the fitted source: flattened
/// polyline vertices for [`CertifiedBezierLineFit2`] and
/// [`CertifiedBezierPointFit2`], or Bezier/conic control points for
/// [`CertifiedBezierLineImage2`] and [`CertifiedBezierPointImage2`]. The fit
/// error bound is separate from `source_flattening_error`: a flattened-polyline
/// fit may be exact for the emitted vertices while still carrying the source
/// curve-to-polyline flattening budget. `construction_policy` records the
/// predicate policy used to prove the fit, and
/// `source_flattening_max_depth` records the upstream subdivision budget when
/// the source is a flattened Bezier. Keeping these facts explicit is the
/// certificate discipline requested by Yap's exact geometric computation model
/// and by Levien's fitting/simplification notes.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierFitCertificate {
    source_start: usize,
    source_end: usize,
    fit_error_bound: Real,
    source_flattening_error: Option<Real>,
    source_flattening_max_depth: Option<usize>,
    construction_policy: CurvePolicy,
    metric: BezierFitErrorMetric,
    bound_kind: BezierFitBoundKind,
}

/// A Bezier or conic segment certified to be one exact affine point.
///
/// This is the zero-dimensional companion to [`CertifiedBezierLineImage2`].
/// It succeeds only when every control point is certified equal to the endpoint
/// point, preserving Yap's exact branch boundary instead of treating a tiny
/// curve as degenerate by tolerance; see Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997). Rational conics also
/// require certified same-sign nonzero weights so the homogeneous denominator
/// cannot cross a projective boundary on the affine parameter interval. A
/// uniformly negative homogeneous lift is sign-normalized to the positive
/// case without changing the Euclidean image; see Farin, *Curves and Surfaces
/// for Computer-Aided Geometric Design* (5th ed., 2002).
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierPointImage2 {
    point: Point2,
    control_point_count: usize,
    fit_certificate: BezierFitCertificate,
}

/// A Bezier or conic segment certified to trace exactly one endpoint line segment.
///
/// This is a structural fit, not a sampled approximation: every control point
/// must be certified collinear with and inside the endpoint interval. For
/// rational quadratics, all weights must also be certified with the same
/// nonzero sign so the homogeneous lift can be sign-normalized before applying
/// the Euclidean convex-hull property. The certificate follows Yap's exact
/// geometric-computation model by preserving a proof obligation at the fitting
/// boundary; see Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997). The rational Bezier line-image condition follows
/// Farin, *Curves and Surfaces for Computer-Aided Geometric Design* (5th ed.,
/// 2002).
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineImage2 {
    line: LineSeg2,
    control_point_count: usize,
    fit_certificate: BezierFitCertificate,
}

/// Exact offset of a certified Bezier/conic line image.
///
/// The offset keeps the line-image fit certificate because the operation is
/// exact only after the source image has been proved to be a primitive line.
/// This keeps the proof object at the API boundary as required by Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997). General Bezier offsets remain a separate approximation/trimming
/// problem; see Tiller and Hanson, "Offsets of Two-Dimensional Profiles"
/// (1984).
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineImageOffset2 {
    line: LineSeg2,
    control_point_count: usize,
    distance: Real,
    fit_certificate: BezierFitCertificate,
}

/// A zero-error line fit recovered from a certified flattened Bezier polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineFit2 {
    line: LineSeg2,
    source_certificate: BezierFlatteningCertificate,
    fit_certificate: BezierFitCertificate,
}

/// A zero-error point fit recovered from a certified flattened Bezier polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierPointFit2 {
    point: Point2,
    source_certificate: BezierFlatteningCertificate,
    fit_certificate: BezierFitCertificate,
}

/// Exact offset of a certified zero-error line fit.
///
/// The offset is represented by a true line segment and retains both the
/// source flattening certificate and the zero-error fit certificate. This
/// follows Yap's certificate discipline at approximation boundaries; see Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997). The restriction to already-certified line fits avoids applying a
/// sampled offset algorithm as a topology decision; compare Tiller and Hanson,
/// "Offsets of Two-Dimensional Profiles" (1984).
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineOffset2 {
    line: LineSeg2,
    source_certificate: BezierFlatteningCertificate,
    distance: Real,
    fit_certificate: BezierFitCertificate,
}

impl CertifiedBezierLineOffset2 {
    /// Returns the exact offset line segment.
    pub const fn line(&self) -> &LineSeg2 {
        &self.line
    }

    /// Returns the flattening certificate of the source fit.
    pub const fn source_certificate(&self) -> &BezierFlatteningCertificate {
        &self.source_certificate
    }

    /// Returns the exact offset distance.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }

    /// Returns the fit certificate inherited by this exact primitive offset.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }
}

/// Certified result of attempting a zero-error line fit.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierLineFitRelation {
    /// Every retained source vertex is certified on the fitted line segment.
    Fit(CertifiedBezierLineFit2),
    /// At least one retained source vertex is certified off the fitted segment.
    NotLine,
}

/// Certified result of attempting a zero-error point fit.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierPointFitRelation {
    /// Every retained source vertex is certified equal to one point.
    Fit(CertifiedBezierPointFit2),
    /// At least one retained source vertex is certified different from the point.
    NotPoint,
}

/// Certified result of attempting to fit the Bezier/conic object itself to a line.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierLineImageFitRelation {
    /// Every control point is certified on the endpoint line segment.
    Fit(CertifiedBezierLineImage2),
    /// At least one control point is certified off the endpoint line segment.
    NotLine,
}

/// Certified result of attempting to fit the Bezier/conic object to one point.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierPointImageFitRelation {
    /// Every control point is certified equal to the endpoint point.
    Fit(CertifiedBezierPointImage2),
    /// At least one control point is certified different from the endpoint point.
    NotPoint,
}

impl BezierFitCertificate {
    /// Builds a proven exact fit certificate over a half-open source range.
    fn proven_exact(
        source_start: usize,
        source_end: usize,
        source_flattening_error: Option<Real>,
        source_flattening_max_depth: Option<usize>,
        construction_policy: &CurvePolicy,
    ) -> Self {
        Self {
            source_start,
            source_end,
            fit_error_bound: Real::zero(),
            source_flattening_error,
            source_flattening_max_depth,
            construction_policy: construction_policy.clone(),
            metric: BezierFitErrorMetric::ExactEuclideanDistance,
            bound_kind: BezierFitBoundKind::ProvenExact,
        }
    }

    /// Returns the half-open start index of the fitted source range.
    pub const fn source_start(&self) -> usize {
        self.source_start
    }

    /// Returns the half-open end index of the fitted source range.
    pub const fn source_end(&self) -> usize {
        self.source_end
    }

    /// Returns the proven fit-error bound.
    pub const fn fit_error_bound(&self) -> &Real {
        &self.fit_error_bound
    }

    /// Returns the source curve-to-polyline flattening error, when applicable.
    pub const fn source_flattening_error(&self) -> Option<&Real> {
        self.source_flattening_error.as_ref()
    }

    /// Returns the source flattening recursion budget, when the fit came from a
    /// certified flattened Bezier polyline.
    pub const fn source_flattening_max_depth(&self) -> Option<usize> {
        self.source_flattening_max_depth
    }

    /// Returns the policy snapshot used to prove this fit.
    pub const fn construction_policy(&self) -> &CurvePolicy {
        &self.construction_policy
    }

    /// Returns the error metric certified by this fit.
    pub const fn metric(&self) -> BezierFitErrorMetric {
        self.metric
    }

    /// Returns whether the fit bound is proven or only a weaker approximation contract.
    pub const fn bound_kind(&self) -> BezierFitBoundKind {
        self.bound_kind
    }
}

impl CertifiedBezierPointImage2 {
    /// Returns the exact point image traced by the Bezier.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the number of source Bezier control points covered by the fit.
    pub const fn control_point_count(&self) -> usize {
        self.control_point_count
    }

    /// Returns the certificate proving this exact point-image fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }
}

impl CertifiedBezierLineImage2 {
    /// Returns the exact endpoint line segment traced by the Bezier.
    pub const fn line(&self) -> &LineSeg2 {
        &self.line
    }

    /// Returns the number of source Bezier control points covered by the fit.
    pub const fn control_point_count(&self) -> usize {
        self.control_point_count
    }

    /// Returns the certificate proving this exact line-image fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }

    /// Offsets the certified line image exactly as a line primitive.
    ///
    /// This is a true offset only for the certified line-image case. General
    /// Bezier offsets still require the normal-curve and trimming analysis
    /// described by Tiller and Hanson, "Offsets of Two-Dimensional Profiles"
    /// (1984), and Raph Levien, "Parallel curves of cubic Beziers" (2022).
    pub fn offset_left_exact(
        &self,
        distance: Real,
    ) -> CurveResult<CertifiedBezierLineImageOffset2> {
        Ok(CertifiedBezierLineImageOffset2 {
            line: self.line.offset_left(distance.clone())?,
            control_point_count: self.control_point_count,
            distance,
            fit_certificate: self.fit_certificate.clone(),
        })
    }

    /// Offsets the certified line image exactly to the curve's right side.
    ///
    /// The retained certificate stores the result as a negative signed
    /// left-normal distance. This keeps left and right exact primitive offsets
    /// in one algebraic representation, matching the exact-boundary discipline
    /// advocated by Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997), while still avoiding any
    /// Tiller-Hanson-style sampled offset claim for non-line Bezier images.
    pub fn offset_right_exact(
        &self,
        distance: Real,
    ) -> CurveResult<CertifiedBezierLineImageOffset2> {
        self.offset_left_exact(-distance)
    }
}

impl CertifiedBezierLineImageOffset2 {
    /// Returns the exact offset line segment.
    pub const fn line(&self) -> &LineSeg2 {
        &self.line
    }

    /// Returns the number of source Bezier control points covered by the fit.
    pub const fn control_point_count(&self) -> usize {
        self.control_point_count
    }

    /// Returns the exact offset distance.
    pub const fn distance(&self) -> &Real {
        &self.distance
    }

    /// Returns the fit certificate inherited by this exact primitive offset.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }
}

impl CertifiedBezierLineFit2 {
    /// Returns the fitted exact line segment.
    pub const fn line(&self) -> &LineSeg2 {
        &self.line
    }

    /// Returns the flattening certificate of the source polyline.
    pub const fn source_certificate(&self) -> &BezierFlatteningCertificate {
        &self.source_certificate
    }

    /// Returns the certificate proving this zero-error flattened-polyline fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }

    /// Offsets the certified zero-error line fit exactly.
    ///
    /// This is a true primitive offset only because the fit has already been
    /// certified as one exact line. General Bezier offsets still require the
    /// staged cusp/inflection/normal analysis described by Tiller and Hanson,
    /// "Offsets of Two-Dimensional Profiles" (1984), and Raph Levien,
    /// "Parallel curves of cubic Beziers" (2022).
    pub fn offset_left_exact(&self, distance: Real) -> CurveResult<CertifiedBezierLineOffset2> {
        Ok(CertifiedBezierLineOffset2 {
            line: self.line.offset_left(distance.clone())?,
            source_certificate: self.source_certificate.clone(),
            distance,
            fit_certificate: self.fit_certificate.clone(),
        })
    }

    /// Offsets the certified zero-error line fit exactly to the right side.
    ///
    /// Right offsets are represented as negative signed left-normal distances
    /// in the returned certificate. The line fit itself has already proven a
    /// zero-error primitive image, so no free-form Bezier offset topology claim
    /// is introduced here; compare Tiller and Hanson, "Offsets of
    /// Two-Dimensional Profiles" (1984).
    pub fn offset_right_exact(&self, distance: Real) -> CurveResult<CertifiedBezierLineOffset2> {
        self.offset_left_exact(-distance)
    }
}

impl CertifiedBezierPointFit2 {
    /// Returns the fitted exact point.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the flattening certificate of the source polyline.
    pub const fn source_certificate(&self) -> &BezierFlatteningCertificate {
        &self.source_certificate
    }

    /// Returns the certificate proving this zero-error flattened-polyline fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }
}

impl QuadraticBezier2 {
    /// Fits this quadratic Bezier to one exact point when possible.
    pub fn fit_exact_point_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierPointImageFitRelation>> {
        fit_control_polygon_point_image(&self.control_points(), policy)
    }

    /// Fits this quadratic Bezier to its exact endpoint line image when possible.
    ///
    /// The fit succeeds only when the interior control point is certified on
    /// the endpoint segment. This avoids flattening before exact line offsets
    /// and keeps non-line curves behind explicit `NotLine` or uncertainty.
    pub fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        fit_control_polygon_line_image(&self.control_points(), policy)
    }
}

impl CubicBezier2 {
    /// Fits this cubic Bezier to one exact point when possible.
    pub fn fit_exact_point_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierPointImageFitRelation>> {
        fit_control_polygon_point_image(&self.control_points(), policy)
    }

    /// Fits this cubic Bezier to its exact endpoint line image when possible.
    pub fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        fit_control_polygon_line_image(&self.control_points(), policy)
    }
}

impl RationalQuadraticBezier2 {
    /// Fits this rational quadratic to one exact affine point when possible.
    ///
    /// Same-sign nonzero weights keep the homogeneous denominator away from
    /// zero over `[0, 1]`, so equal Euclidean controls certify a constant
    /// affine image after homogeneous sign normalization.
    pub fn fit_exact_point_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierPointImageFitRelation>> {
        match weights_known_same_nonzero_sign(self.weights().as_slice(), policy) {
            Some(true) => fit_control_polygon_point_image(&self.control_points(), policy),
            Some(false) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    /// Fits this rational quadratic to its exact endpoint line image when possible.
    ///
    /// A rational quadratic with certified same-sign nonzero weights and
    /// collinear controls has a Euclidean image inside the control hull after
    /// homogeneous sign normalization. If each control is also certified inside
    /// the endpoint box, this method returns a true endpoint line segment
    /// image. Mixed-sign or sign-ambiguous weights are explicit uncertainty
    /// because the endpoint segment image is not certified by the convex-hull
    /// theorem.
    pub fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        match weights_known_same_nonzero_sign(self.weights().as_slice(), policy) {
            Some(true) => fit_control_polygon_line_image(&self.control_points(), policy),
            Some(false) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
}

impl CertifiedBezierPolyline2 {
    /// Fits this certified polyline to one exact point when the fit has zero error.
    ///
    /// This is the flattened-polyline companion to
    /// [`QuadraticBezier2::fit_exact_point_image`]. It succeeds only when every
    /// retained vertex is certified equal to the first vertex, carrying the
    /// original flattening certificate across the fitting boundary. This follows
    /// Yap's exact-geometric-computation discipline for approximation products;
    /// see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
    /// 7.1-2 (1997).
    pub fn fit_exact_point(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierPointFitRelation>> {
        let Some(point) = self.points().first() else {
            return Err(CurveError::InsufficientVertices);
        };
        for vertex in self.points().iter().skip(1) {
            match is_zero(&point.distance_squared(vertex), policy) {
                Some(true) => {}
                Some(false) => {
                    return Ok(Classification::Decided(BezierPointFitRelation::NotPoint));
                }
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }

        Ok(Classification::Decided(BezierPointFitRelation::Fit(
            CertifiedBezierPointFit2 {
                point: point.clone(),
                source_certificate: self.certificate().clone(),
                fit_certificate: BezierFitCertificate::proven_exact(
                    0,
                    self.points().len(),
                    Some(self.certificate().max_error().clone()),
                    Some(self.certificate().max_depth()),
                    policy,
                ),
            },
        )))
    }

    /// Fits this certified polyline to one exact line when the fit has zero error.
    ///
    /// The method returns [`BezierLineFitRelation::NotLine`] when any retained
    /// vertex is proven off the fitted line or outside the endpoint interval.
    /// Uncertain orientation or containment is reported explicitly. This is a
    /// certified fitting product, not a least-squares or sampled tolerance fit.
    pub fn fit_exact_line(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineFitRelation>> {
        let Some(start) = self.points().first() else {
            return Err(CurveError::InsufficientVertices);
        };
        let Some(end) = self.points().last() else {
            return Err(CurveError::InsufficientVertices);
        };
        if is_zero(&start.distance_squared(end), policy) == Some(true) {
            return Err(CurveError::ZeroLengthLine);
        }
        let line = LineSeg2::try_new(start.clone(), end.clone())?;
        let envelope = match Aabb2::from_points([start, end], policy) {
            Classification::Decided(envelope) => envelope,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };

        for point in self
            .points()
            .iter()
            .skip(1)
            .take(self.points().len().saturating_sub(2))
        {
            match point_on_line_interval(start, end, &envelope, point, policy) {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Decided(BezierLineFitRelation::NotLine));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }

        Ok(Classification::Decided(BezierLineFitRelation::Fit(
            CertifiedBezierLineFit2 {
                line,
                source_certificate: self.certificate().clone(),
                fit_certificate: BezierFitCertificate::proven_exact(
                    0,
                    self.points().len(),
                    Some(self.certificate().max_error().clone()),
                    Some(self.certificate().max_depth()),
                    policy,
                ),
            },
        )))
    }
}

fn weights_known_same_nonzero_sign(weights: &[&Real], policy: &CurvePolicy) -> Option<bool> {
    let mut expected = None;
    for weight in weights {
        let sign = real_sign(weight, policy)?;
        match sign {
            RealSign::Positive | RealSign::Negative => {
                if let Some(expected) = expected {
                    if sign != expected {
                        return Some(false);
                    }
                } else {
                    expected = Some(sign);
                }
            }
            RealSign::Zero => return Some(false),
        }
    }
    Some(expected.is_some())
}

fn fit_control_polygon_point_image(
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<BezierPointImageFitRelation>> {
    let Some(point) = controls.first().copied() else {
        return Err(CurveError::InsufficientVertices);
    };
    for control in controls.iter().skip(1) {
        match is_zero(&point.distance_squared(control), policy) {
            Some(true) => {}
            Some(false) => {
                return Ok(Classification::Decided(
                    BezierPointImageFitRelation::NotPoint,
                ));
            }
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }

    Ok(Classification::Decided(BezierPointImageFitRelation::Fit(
        CertifiedBezierPointImage2 {
            point: point.clone(),
            control_point_count: controls.len(),
            fit_certificate: BezierFitCertificate::proven_exact(
                0,
                controls.len(),
                None,
                None,
                policy,
            ),
        },
    )))
}

fn fit_control_polygon_line_image(
    controls: &[&Point2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<BezierLineImageFitRelation>> {
    let Some(start) = controls.first().copied() else {
        return Err(CurveError::InsufficientVertices);
    };
    let Some(end) = controls.last().copied() else {
        return Err(CurveError::InsufficientVertices);
    };
    if is_zero(&start.distance_squared(end), policy) == Some(true) {
        return Err(CurveError::ZeroLengthLine);
    }
    let line = LineSeg2::try_new(start.clone(), end.clone())?;
    let envelope = match Aabb2::from_points([start, end], policy) {
        Classification::Decided(envelope) => envelope,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    for point in controls
        .iter()
        .skip(1)
        .take(controls.len().saturating_sub(2))
    {
        match point_on_line_interval(start, end, &envelope, point, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => {
                return Ok(Classification::Decided(BezierLineImageFitRelation::NotLine));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    Ok(Classification::Decided(BezierLineImageFitRelation::Fit(
        CertifiedBezierLineImage2 {
            line,
            control_point_count: controls.len(),
            fit_certificate: BezierFitCertificate::proven_exact(
                0,
                controls.len(),
                None,
                None,
                policy,
            ),
        },
    )))
}

fn point_on_line_interval(
    start: &Point2,
    end: &Point2,
    envelope: &Aabb2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    match classify_oriented_line(start, end, point, policy) {
        Classification::Decided(LineSide::On) => {}
        Classification::Decided(_) => return Classification::Decided(false),
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    }
    envelope.contains_point(point, policy)
}
