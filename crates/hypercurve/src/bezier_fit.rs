//! Certified zero-error fits for Bezier and rational-conic boundary data.
//!
//! This module keeps only the primitive cases needed by Bezier boolean and
//! offset work: a Bezier/conic image can be proven to be exactly one point or
//! exactly one endpoint line segment, and a certified flattened Bezier polyline
//! can be proven to have the same zero-error primitive image. These are proof
//! objects for exact branch decisions, not general fitting reports.

use hyperreal::{Real, RealSign};

use crate::classify::{classify_oriented_line, is_zero, real_sign};
use crate::{
    Aabb2, BezierFlatteningCertificate, CertifiedBezierPolyline2, Classification, CubicBezier2,
    CurveError, CurvePolicy, CurveResult, LineSeg2, LineSide, Point2, QuadraticBezier2,
    RationalBezier2, RationalQuadraticBezier2, UncertaintyReason,
};

/// Error metric used by a zero-error primitive fit.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierFitErrorMetric {
    /// Exact Euclidean point-to-primitive distance with a zero bound.
    ExactEuclideanDistance,
}

/// Proof status for a Bezier fitting error bound.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BezierFitBoundKind {
    /// The bound is proved exactly by symbolic predicates.
    ProvenExact,
}

/// Certificate attached to a zero-error Bezier primitive fit.
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
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierPointImage2 {
    point: Point2,
    control_point_count: usize,
    fit_certificate: BezierFitCertificate,
}

/// A Bezier or conic segment certified to trace exactly one endpoint line segment.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineImage2 {
    line: LineSeg2,
    control_point_count: usize,
    fit_certificate: BezierFitCertificate,
}

/// Exact offset of a certified Bezier/conic line image.
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
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineOffset2 {
    line: LineSeg2,
    source_certificate: BezierFlatteningCertificate,
    distance: Real,
    fit_certificate: BezierFitCertificate,
}

/// Result of attempting a zero-error line fit.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierLineFitRelation {
    /// Every retained source vertex is certified on the fitted line segment.
    Fit(CertifiedBezierLineFit2),
    /// At least one retained source vertex is certified off the fitted segment.
    NotLine,
}

/// Result of attempting a zero-error point fit.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierPointFitRelation {
    /// Every retained source vertex is certified equal to one point.
    Fit(CertifiedBezierPointFit2),
    /// At least one retained source vertex is certified different from the point.
    NotPoint,
}

/// Result of attempting to fit the Bezier/conic object itself to a line.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierLineImageFitRelation {
    /// Every control point is certified on the endpoint line segment.
    Fit(CertifiedBezierLineImage2),
    /// At least one control point is certified off the endpoint line segment.
    NotLine,
}

/// Result of attempting to fit the Bezier/conic object to one point.
#[derive(Clone, Debug, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum BezierPointImageFitRelation {
    /// Every control point is certified equal to the endpoint point.
    Fit(CertifiedBezierPointImage2),
    /// At least one control point is certified different from the endpoint point.
    NotPoint,
}

impl BezierFitCertificate {
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

    /// Returns the source flattening recursion budget, when applicable.
    pub const fn source_flattening_max_depth(&self) -> Option<usize> {
        self.source_flattening_max_depth
    }

    /// Returns the policy used to prove this fit.
    pub const fn construction_policy(&self) -> &CurvePolicy {
        &self.construction_policy
    }

    /// Returns the error metric certified by this fit.
    pub const fn metric(&self) -> BezierFitErrorMetric {
        self.metric
    }

    /// Returns whether the fit bound is proven exactly.
    pub const fn bound_kind(&self) -> BezierFitBoundKind {
        self.bound_kind
    }
}

impl CertifiedBezierPointImage2 {
    /// Returns the exact point image traced by the Bezier/conic.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the number of source control points covered by the fit.
    pub const fn control_point_count(&self) -> usize {
        self.control_point_count
    }

    /// Returns the certificate proving this exact point-image fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }
}

impl CertifiedBezierLineImage2 {
    /// Returns the exact endpoint line segment traced by the Bezier/conic.
    pub const fn line(&self) -> &LineSeg2 {
        &self.line
    }

    /// Returns the number of source control points covered by the fit.
    pub const fn control_point_count(&self) -> usize {
        self.control_point_count
    }

    /// Returns the certificate proving this exact line-image fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }

    /// Offsets the certified line image exactly as a line primitive.
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

    /// Returns the number of source control points covered by the fit.
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

    /// Returns the source flattening certificate.
    pub const fn source_certificate(&self) -> &BezierFlatteningCertificate {
        &self.source_certificate
    }

    /// Returns the certificate proving this zero-error fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }

    /// Offsets the certified zero-error line fit exactly.
    pub fn offset_left_exact(&self, distance: Real) -> CurveResult<CertifiedBezierLineOffset2> {
        Ok(CertifiedBezierLineOffset2 {
            line: self.line.offset_left(distance.clone())?,
            source_certificate: self.source_certificate.clone(),
            distance,
            fit_certificate: self.fit_certificate.clone(),
        })
    }

    /// Offsets the certified zero-error line fit exactly to the right side.
    pub fn offset_right_exact(&self, distance: Real) -> CurveResult<CertifiedBezierLineOffset2> {
        self.offset_left_exact(-distance)
    }
}

impl CertifiedBezierPointFit2 {
    /// Returns the fitted exact point.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the source flattening certificate.
    pub const fn source_certificate(&self) -> &BezierFlatteningCertificate {
        &self.source_certificate
    }

    /// Returns the certificate proving this zero-error fit.
    pub const fn fit_certificate(&self) -> &BezierFitCertificate {
        &self.fit_certificate
    }
}

impl CertifiedBezierLineOffset2 {
    /// Returns the exact offset line segment.
    pub const fn line(&self) -> &LineSeg2 {
        &self.line
    }

    /// Returns the source flattening certificate.
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

impl QuadraticBezier2 {
    /// Fits this quadratic Bezier to one exact point when possible.
    pub fn fit_exact_point_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierPointImageFitRelation>> {
        fit_control_polygon_point_image(&self.control_points(), policy)
    }

    /// Fits this quadratic Bezier to its exact endpoint line image when possible.
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
    /// Fits this rational quadratic conic to one exact affine point when possible.
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

    /// Fits this rational quadratic conic to its exact endpoint line image when possible.
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

impl RationalBezier2 {
    /// Fits this rational Bezier to its exact endpoint line image when possible.
    pub fn fit_exact_line_image(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierLineImageFitRelation>> {
        let weights = self.weights().iter().collect::<Vec<_>>();
        match weights_known_same_nonzero_sign(&weights, policy) {
            Some(true) => {
                let controls = self.control_points().iter().collect::<Vec<_>>();
                fit_control_polygon_line_image(&controls, policy)
            }
            Some(false) => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
    }
}

impl CertifiedBezierPolyline2 {
    /// Fits this certified polyline to one exact point when the fit has zero error.
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
