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
}

/// Exact offset of a certified Bezier/conic line image.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineImageOffset2 {
    line: LineSeg2,
    control_point_count: usize,
    distance: Real,
}

/// A zero-error line fit recovered from a certified flattened Bezier polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineFit2 {
    line: LineSeg2,
    source_certificate: BezierFlatteningCertificate,
}

/// A zero-error point fit recovered from a certified flattened Bezier polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierPointFit2 {
    point: Point2,
    source_certificate: BezierFlatteningCertificate,
}

/// Exact offset of a certified zero-error line fit.
#[derive(Clone, Debug, PartialEq)]
pub struct CertifiedBezierLineOffset2 {
    line: LineSeg2,
    source_certificate: BezierFlatteningCertificate,
    distance: Real,
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

impl CertifiedBezierPointImage2 {
    /// Returns the exact point image traced by the Bezier.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Returns the number of source Bezier control points covered by the fit.
    pub const fn control_point_count(&self) -> usize {
        self.control_point_count
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
        })
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
        })
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
