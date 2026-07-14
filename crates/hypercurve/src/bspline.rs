//! Exact polynomial and rational B-spline span extraction.
//!
//! This module is the first retained B-spline carrier in `hypercurve`.  It
//! keeps the authored control net, weights, and knot vector as exact [`Real`]
//! data, then extracts Bezier spans by exact Boehm knot insertion.  That
//! matches Yap's exact-geometric-computation rule from "Towards Exact
//! Geometric Computation" (1997): preserve the source object and move to
//! another representation only through replayable exact construction evidence.
//! Knot insertion follows Boehm, "Inserting new knots into B-spline curves"
//! (Computer-Aided Design, 1980), and the B-spline/Bezier span identities
//! follow de Boor, *A Practical Guide to Splines* (1978), and Farin, *Curves
//! and Surfaces for CAGD* (5th ed., 2002).

use std::cmp::Ordering;

use hyperreal::Real;

use crate::classify::{compare_reals, is_zero};
use crate::{
    Aabb2, Axis2, BezierSubcurve2, Classification, CubicBezier2, CurveError, CurvePolicy,
    CurveResult, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    RetainedCurveCacheSummary2, RetainedCurveFamily2, RetainedCurveIdentity2,
    RetainedCurveProfile2, RetainedEndpointEvidence2, RetainedParameterDomain1,
    RetainedTopologyStatus, RetainedTrimInterval1, SplinePeriodicity2, UncertaintyReason,
};

/// Exact polynomial B-spline curve in the plane.
///
/// Extraction accepts any positive degree. Linear, quadratic, and cubic spans
/// use specialized polynomial carriers; higher-degree spans use exact general
/// Beziers with unit weights, without approximation or degree reduction.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialBSplineCurve2 {
    degree: usize,
    control_points: Vec<Point2>,
    knots: Vec<Real>,
    periodicity: SplinePeriodicity2,
}

/// Exact Bezier extraction report for one polynomial B-spline.
///
/// The report keeps both the refined knot/control data and the emitted Bezier
/// spans so callers can audit the exact knot-insertion construction rather than
/// treating span conversion as an opaque adapter.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialBSplineBezierExtraction2 {
    degree: usize,
    refined_control_points: Vec<Point2>,
    refined_knots: Vec<Real>,
    spans: Vec<BezierSubcurve2>,
    inserted_knot_count: usize,
}

/// Exact quadratic NURBS curve in the plane.
///
/// This is the rational counterpart to [`PolynomialBSplineCurve2`] for the
/// family that can be consumed by the existing rational quadratic Bezier/conic
/// topology code.  The carrier stores affine control points, homogeneous
/// weights, and the authored knot vector exactly; extraction is performed by
/// Boehm insertion on homogeneous controls.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBSplineCurve2 {
    control_points: Vec<Point2>,
    weights: Vec<Real>,
    knots: Vec<Real>,
}

/// Exact rational Bezier extraction report for one quadratic NURBS curve.
///
/// The refined controls are affine rational Bezier controls.  Refined weights
/// are stored beside them so callers can audit the homogeneous knot-insertion
/// replay instead of accepting an unlabelled approximation.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalQuadraticBSplineBezierExtraction2 {
    refined_control_points: Vec<Point2>,
    refined_weights: Vec<Real>,
    refined_knots: Vec<Real>,
    spans: Vec<BezierSubcurve2>,
    inserted_knot_count: usize,
}

/// Exact rational B-spline/NURBS curve in the plane.
///
/// This retained carrier is the higher-degree counterpart to
/// [`RationalQuadraticBSplineCurve2`].  It stores affine controls, homogeneous
/// weights, and knots exactly, then extracts rational Bezier spans as retained
/// control nets instead of pretending that unsupported rational cubic and
/// higher-degree spans are native topology fragments.  This follows Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
/// 3-23 (1997): the exact object is preserved and any representational change
/// is report-bearing construction evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBSplineCurve2 {
    degree: usize,
    control_points: Vec<Point2>,
    weights: Vec<Real>,
    knots: Vec<Real>,
    periodicity: SplinePeriodicity2,
}

/// Exact rational Bezier extraction report for a retained NURBS curve.
///
/// The report exposes the refined homogeneous construction and the final
/// rational Bezier spans.  Callers that only support rational quadratics can
/// continue using [`RationalQuadraticBSplineCurve2`]; callers that need to
/// retain cubic or higher-degree NURBS evidence can use this type without
/// sampling or flattening the curve.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBSplineBezierExtraction2 {
    degree: usize,
    refined_control_points: Vec<Point2>,
    refined_weights: Vec<Real>,
    refined_knots: Vec<Real>,
    spans: Vec<RationalBezierSpan2>,
    inserted_knot_count: usize,
}

/// Native-topology audit report for a retained rational B-spline extraction.
///
/// This report is deliberately stronger than a direct `Vec<BezierSubcurve2>`:
/// every retained rational Bezier span contributes a status, and only spans
/// with [`RetainedTopologyStatus::NativeExact`] contribute a native subcurve.
/// Nonuniform rational cubics and higher-degree rational Beziers remain exact
/// native objects rather than disappearing behind a generic unsupported
/// return. This follows Yap's retained-object discipline, while the
/// degree/equal-weight promotion rules are the homogeneous Bezier identities
/// described by Farin, *Curves and Surfaces for CAGD* (5th ed., 2002).
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBSplineNativeTopologyReport2 {
    span_reports: Vec<RationalBezierSpanTopologyReport2>,
}

/// Native-topology audit report for one retained rational Bezier span.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierSpanTopologyReport2 {
    span_index: usize,
    degree: usize,
    knot_start: Real,
    knot_end: Real,
    status: RetainedTopologyStatus,
    decision_path: RationalBezierSpanTopologyPath2,
    native_subcurve: Option<BezierSubcurve2>,
}

/// Exact decision path used to classify one retained rational Bezier span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RationalBezierSpanTopologyPath2 {
    /// The retained span did not carry the expected `degree + 1` controls and weights.
    RetainedControlNetShapeMismatch,
    /// A degree-one rational span was elevated homogeneously to a native conic.
    NativeRationalLinearSpan,
    /// A degree-one rational span has a zero middle elevation weight.
    RetainedSingularLinearSpan,
    /// A degree-two rational span promoted directly to native conic topology.
    NativeRationalQuadraticSpan,
    /// A degree-three rational span promoted to a polynomial cubic because all weights match.
    NativeEqualWeightCubicSpan,
    /// An unequal-weight cubic or higher-degree span promoted without degree reduction.
    NativeGeneralRationalSpan,
}

/// Certified or retained monotonicity evidence for one extracted spline span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedSpanAxisMonotonicity {
    /// The span is certified monotone along this axis.
    CertifiedMonotone,
    /// Exact topology found interior extrema, so the span is not monotone.
    HasInteriorExtrema,
    /// The span is retained evidence and no exact monotone package exists yet.
    Unsupported,
}

/// Nonzero-weight evidence for a retained rational span.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RetainedSpanWeightDomainReport2 {
    weight_count: usize,
    certified_nonzero_count: usize,
    all_weights_certified_nonzero: bool,
}

/// Span-local facts produced from B-spline/NURBS Bezier extraction.
///
/// These facts are a retained CAD broad-phase package, not topology by
/// themselves.  Native Bezier/conic spans use their exact derivative-root
/// bounds and monotone predicates. Retained rational spans without native
/// topology expose conservative control-hull bounds plus explicit unsupported
/// monotone status. This follows the construction/predicate separation in
/// Chee Yap, "Towards Exact Geometric Computation", and keeps the span-local
/// Bernstein evidence required by Gerald Farin, "Curves and Surfaces for CAGD",
/// visible to callers.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedBSplineSpanFacts2 {
    span_index: usize,
    knot_start: Real,
    knot_end: Real,
    bounds: Aabb2,
    x_monotonicity: RetainedSpanAxisMonotonicity,
    y_monotonicity: RetainedSpanAxisMonotonicity,
    topology_status: RetainedTopologyStatus,
    weight_domain: Option<RetainedSpanWeightDomainReport2>,
}

/// Span-local fact report for one B-spline/NURBS extraction.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedBSplineSpanFactReport2 {
    span_facts: Vec<RetainedBSplineSpanFacts2>,
}

/// One exact rational Bezier span extracted from a retained NURBS curve.
///
/// `control_points` and `weights` have length `degree + 1`.  The endpoint knot
/// values are retained with the span so downstream code can keep the source
/// parameter interval attached to the Bezier evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierSpan2 {
    degree: usize,
    control_points: Vec<Point2>,
    weights: Vec<Real>,
    knot_start: Real,
    knot_end: Real,
}

impl PolynomialBSplineCurve2 {
    /// Constructs a polynomial B-spline of any positive degree.
    ///
    /// The knot vector must be nondecreasing, have length
    /// `control_points.len() + degree + 1`, and have endpoint multiplicity
    /// `degree + 1`.  All checks are exact comparisons through `policy`.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        Self::try_new_with_periodicity(
            degree,
            control_points,
            knots,
            SplinePeriodicity2::NonPeriodic,
            policy,
        )
    }

    pub(crate) fn try_new_with_periodicity(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let Some(order) = degree.checked_add(1) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(expected_knot_count) = control_points.len().checked_add(order) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if degree < 1 || control_points.len() < order || knots.len() != expected_knot_count {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        match validate_nondecreasing_knots(&knots, policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        if !has_positive_span(&knots, degree, control_points.len(), policy)? {
            return Err(CurveError::InvalidBSpline);
        }
        match validate_spline_periodicity(
            &knots,
            degree,
            control_points.len(),
            &periodicity,
            policy,
        )? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        Ok(Classification::Decided(Self {
            degree,
            control_points,
            knots,
            periodicity,
        }))
    }

    /// Returns the polynomial degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the retained control net.
    pub fn control_points(&self) -> &[Point2] {
        &self.control_points
    }

    /// Returns the retained knot vector.
    pub fn knots(&self) -> &[Real] {
        &self.knots
    }

    /// Returns the retained finite or periodic spline semantics.
    pub const fn periodicity(&self) -> &SplinePeriodicity2 {
        &self.periodicity
    }

    /// Extracts exact Bezier spans, preserving arbitrary polynomial degree.
    ///
    /// Each distinct interior knot is inserted until its multiplicity equals
    /// the spline degree.  The resulting control net can then be read in
    /// Bezier blocks over each nonzero knot span.  This is Boehm knot insertion
    /// used as an exact construction, not a numeric tessellation.
    pub fn extract_bezier_spans(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<PolynomialBSplineBezierExtraction2>> {
        let mut refined = BSplineWorkingCurve {
            degree: self.degree,
            control_points: self.control_points.clone(),
            knots: self.knots.clone(),
            inserted_knot_count: 0,
        };
        let break_knots = match distinct_bezier_break_knots(&refined.knots, self.degree, policy) {
            Classification::Decided(knots) => knots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for knot in break_knots {
            loop {
                let multiplicity = knot_multiplicity(&refined.knots, &knot, policy)?;
                if multiplicity >= self.degree {
                    break;
                }
                match refined.insert_knot(knot.clone(), policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let spans = match extract_refined_bezier_spans(&refined, policy)? {
            Classification::Decided(spans) => spans,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(
            PolynomialBSplineBezierExtraction2 {
                degree: self.degree,
                refined_control_points: refined.control_points,
                refined_knots: refined.knots,
                spans,
                inserted_knot_count: refined.inserted_knot_count,
            },
        ))
    }

    /// Builds a retained CAD-curve profile from exact B-spline evidence.
    ///
    /// The active domain is `[U[p], U[n+1]]`, the default trim is
    /// the whole domain, and the cache summary is produced by exact Boehm
    /// extraction.  No polyline preview or sampled geometry can promote the
    /// carrier: the topology status is native only because polynomial
    /// quadratic/cubic spans are exact native Bezier topology in this kernel.
    pub fn retained_curve_profile(
        &self,
        source_index: u64,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedCurveProfile2>> {
        self.retained_curve_profile_with_source_version(source_index, 0, policy)
    }

    /// Builds a retained CAD-curve profile with source version/revision evidence.
    pub fn retained_curve_profile_with_source_version(
        &self,
        source_index: u64,
        source_version: u64,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedCurveProfile2>> {
        let domain = match bspline_parameter_domain(
            &self.knots,
            self.degree,
            self.control_points.len(),
            policy,
        )? {
            Classification::Decided(domain) => domain,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let trim = match default_trim(&domain, policy)? {
            Classification::Decided(trim) => trim,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let extraction = match self.extract_bezier_spans(policy)? {
            Classification::Decided(extraction) => extraction,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let cache_summary = RetainedCurveCacheSummary2::new_with_source_version(
            source_version,
            self.control_points.len(),
            self.knots.len(),
            extraction.spans().len(),
            extraction.spans().len(),
            0,
        )?;
        Ok(Classification::Decided(RetainedCurveProfile2::new(
            RetainedCurveIdentity2::new_with_source_version(
                RetainedCurveFamily2::PolynomialBSpline,
                source_index,
                source_version,
            ),
            domain.clone(),
            trim,
            self.periodicity.clone(),
            RetainedTopologyStatus::NativeExact,
            endpoint_evidence(
                extraction
                    .spans()
                    .first()
                    .ok_or(CurveError::InvalidBSpline)?
                    .start(),
                extraction
                    .spans()
                    .last()
                    .ok_or(CurveError::InvalidBSpline)?
                    .end(),
                &domain,
            ),
            cache_summary,
        )?))
    }
}

impl PolynomialBSplineBezierExtraction2 {
    /// Returns the source spline degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the exact refined control net after knot insertion.
    pub fn refined_control_points(&self) -> &[Point2] {
        &self.refined_control_points
    }

    /// Returns the exact refined knot vector after knot insertion.
    pub fn refined_knots(&self) -> &[Real] {
        &self.refined_knots
    }

    /// Returns the extracted Bezier spans in parameter order.
    pub fn spans(&self) -> &[BezierSubcurve2] {
        &self.spans
    }

    /// Returns how many knots were inserted to produce the Bezier form.
    pub const fn inserted_knot_count(&self) -> usize {
        self.inserted_knot_count
    }

    /// Returns span-local bounds and monotonicity facts for extracted Bezier spans.
    pub fn span_fact_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedBSplineSpanFactReport2>> {
        native_span_fact_report(&self.spans, &self.refined_knots, self.degree, policy)
    }
}

impl RationalQuadraticBSplineCurve2 {
    /// Constructs a quadratic NURBS curve over its active knot domain.
    ///
    /// The control and weight arrays must have equal length, every input weight
    /// must be certified nonzero, and the knot vector must be nondecreasing.
    /// Mixed signs are allowed at construction because a
    /// projective NURBS carrier can represent them exactly; extraction rejects
    /// only spans whose refined homogeneous weight cannot be converted to an
    /// affine rational Bezier control.
    pub fn try_new(
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let degree = 2;
        if control_points.len() != weights.len()
            || control_points.len() < degree + 1
            || knots.len() != control_points.len() + degree + 1
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        for weight in &weights {
            match is_zero(weight, policy) {
                Some(false) => {}
                Some(true) => return Err(CurveError::ZeroRationalBezierWeight),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        match validate_nondecreasing_knots(&knots, policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        if !has_positive_span(&knots, degree, control_points.len(), policy)? {
            return Err(CurveError::InvalidBSpline);
        }
        Ok(Classification::Decided(Self {
            control_points,
            weights,
            knots,
        }))
    }

    /// Returns the retained affine control net.
    pub fn control_points(&self) -> &[Point2] {
        &self.control_points
    }

    /// Returns the retained homogeneous weights.
    pub fn weights(&self) -> &[Real] {
        &self.weights
    }

    /// Returns the retained knot vector.
    pub fn knots(&self) -> &[Real] {
        &self.knots
    }

    /// Extracts exact rational quadratic Bezier spans from this NURBS curve.
    ///
    /// Knot insertion is performed on homogeneous triples `(w*x, w*y, w)`.
    /// Only after every interior knot reaches multiplicity two does the method
    /// divide by each refined weight to produce affine rational Bezier controls.
    /// This is the rational Boehm/de Boor construction described by Farin
    /// (2002), kept as exact object replay in Yap's EGC sense.
    pub fn extract_bezier_spans(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalQuadraticBSplineBezierExtraction2>> {
        let mut refined = HomogeneousBSplineWorkingCurve {
            degree: 2,
            controls: self
                .control_points
                .iter()
                .zip(&self.weights)
                .map(|(point, weight)| HomogeneousControl2::from_affine(point, weight))
                .collect(),
            knots: self.knots.clone(),
            inserted_knot_count: 0,
        };
        let break_knots = match distinct_bezier_break_knots(&refined.knots, 2, policy) {
            Classification::Decided(knots) => knots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for knot in break_knots {
            loop {
                let multiplicity = knot_multiplicity(&refined.knots, &knot, policy)?;
                if multiplicity >= 2 {
                    break;
                }
                match refined.insert_knot(knot.clone(), policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        let extraction = match extract_refined_rational_quadratic_spans(&refined, policy)? {
            Classification::Decided(extraction) => extraction,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(Classification::Decided(extraction))
    }

    /// Builds a retained CAD-curve profile from exact quadratic NURBS evidence.
    ///
    /// Degree-two rational spans are exact native rational quadratic Beziers in
    /// the current kernel, so the profile status is native after homogeneous
    /// knot insertion certifies all refined weights.  This keeps the NURBS
    /// source domain and endpoint evidence attached to the native bridge.
    pub fn retained_curve_profile(
        &self,
        source_index: u64,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedCurveProfile2>> {
        self.retained_curve_profile_with_source_version(source_index, 0, policy)
    }

    /// Builds a retained CAD-curve profile with source version/revision evidence.
    pub fn retained_curve_profile_with_source_version(
        &self,
        source_index: u64,
        source_version: u64,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedCurveProfile2>> {
        let domain =
            match bspline_parameter_domain(&self.knots, 2, self.control_points.len(), policy)? {
                Classification::Decided(domain) => domain,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
        let trim = match default_trim(&domain, policy)? {
            Classification::Decided(trim) => trim,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let extraction = match self.extract_bezier_spans(policy)? {
            Classification::Decided(extraction) => extraction,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let cache_summary = RetainedCurveCacheSummary2::new_with_source_version(
            source_version,
            self.control_points.len(),
            self.knots.len(),
            extraction.spans().len(),
            extraction.spans().len(),
            0,
        )?;
        Ok(Classification::Decided(RetainedCurveProfile2::new(
            RetainedCurveIdentity2::new_with_source_version(
                RetainedCurveFamily2::RationalQuadraticBSpline,
                source_index,
                source_version,
            ),
            domain.clone(),
            trim,
            SplinePeriodicity2::NonPeriodic,
            RetainedTopologyStatus::NativeExact,
            endpoint_evidence(
                extraction
                    .spans()
                    .first()
                    .ok_or(CurveError::InvalidBSpline)?
                    .start(),
                extraction
                    .spans()
                    .last()
                    .ok_or(CurveError::InvalidBSpline)?
                    .end(),
                &domain,
            ),
            cache_summary,
        )?))
    }
}

impl RationalQuadraticBSplineBezierExtraction2 {
    /// Returns the exact refined affine control net.
    pub fn refined_control_points(&self) -> &[Point2] {
        &self.refined_control_points
    }

    /// Returns the exact refined homogeneous weights.
    pub fn refined_weights(&self) -> &[Real] {
        &self.refined_weights
    }

    /// Returns the exact refined knot vector.
    pub fn refined_knots(&self) -> &[Real] {
        &self.refined_knots
    }

    /// Returns extracted rational quadratic Bezier spans in parameter order.
    pub fn spans(&self) -> &[BezierSubcurve2] {
        &self.spans
    }

    /// Returns how many knots were inserted to produce the rational Bezier form.
    pub const fn inserted_knot_count(&self) -> usize {
        self.inserted_knot_count
    }

    /// Returns span-local bounds, monotonicity, and weight-domain facts.
    pub fn span_fact_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedBSplineSpanFactReport2>> {
        let mut report = match native_span_fact_report(&self.spans, &self.refined_knots, 2, policy)?
        {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut fact_index = 0_usize;
        let refined_control_count = self.refined_knots.len().saturating_sub(3);
        for knot_index in 2..refined_control_count {
            if compare_reals(
                &self.refined_knots[knot_index],
                &self.refined_knots[knot_index + 1],
                policy,
            ) != Some(Ordering::Less)
            {
                continue;
            }
            let Some(fact) = report.span_facts.get_mut(fact_index) else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            let start = knot_index - 2;
            fact.weight_domain = Some(weight_domain_report(
                &self.refined_weights[start..=knot_index],
                policy,
            )?);
            fact_index += 1;
        }
        if fact_index != report.span_facts.len() {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Ok(Classification::Decided(report))
    }
}

impl RationalBSplineCurve2 {
    /// Constructs a rational B-spline/NURBS curve of degree one or higher.
    ///
    /// The control and weight arrays must have equal length, every authored
    /// weight must be certified nonzero, and the knot vector must be
    /// nondecreasing and long enough for the selected degree.  The
    /// degree is not capped here because this carrier is retained evidence, not
    /// a promise that downstream topology can consume every extracted span.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        Self::try_new_with_periodicity(
            degree,
            control_points,
            weights,
            knots,
            SplinePeriodicity2::NonPeriodic,
            policy,
        )
    }

    pub(crate) fn try_new_with_periodicity(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        let Some(order) = degree.checked_add(1) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(expected_knot_count) = control_points.len().checked_add(order) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        if degree < 1
            || control_points.len() != weights.len()
            || control_points.len() < order
            || knots.len() != expected_knot_count
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        for weight in &weights {
            match is_zero(weight, policy) {
                Some(false) => {}
                Some(true) => return Err(CurveError::ZeroRationalBezierWeight),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        match validate_nondecreasing_knots(&knots, policy) {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        if !has_positive_span(&knots, degree, control_points.len(), policy)? {
            return Err(CurveError::InvalidBSpline);
        }
        match validate_spline_periodicity(
            &knots,
            degree,
            control_points.len(),
            &periodicity,
            policy,
        )? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        Ok(Classification::Decided(Self {
            degree,
            control_points,
            weights,
            knots,
            periodicity,
        }))
    }

    /// Returns the retained polynomial degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the retained affine control net.
    pub fn control_points(&self) -> &[Point2] {
        &self.control_points
    }

    /// Returns the retained homogeneous weights.
    pub fn weights(&self) -> &[Real] {
        &self.weights
    }

    /// Returns the retained knot vector.
    pub fn knots(&self) -> &[Real] {
        &self.knots
    }

    /// Returns the retained finite or periodic spline semantics.
    pub const fn periodicity(&self) -> &SplinePeriodicity2 {
        &self.periodicity
    }

    pub(crate) fn insert_knots(
        &self,
        knots: Vec<Real>,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<(Self, usize)>> {
        if knots.is_empty() {
            return Ok(Classification::Decided((self.clone(), 0)));
        }
        let mut refined = HomogeneousBSplineWorkingCurve {
            degree: self.degree,
            controls: self
                .control_points
                .iter()
                .zip(&self.weights)
                .map(|(point, weight)| HomogeneousControl2::from_affine(point, weight))
                .collect(),
            knots: self.knots.clone(),
            inserted_knot_count: 0,
        };
        for knot in knots {
            match refined.insert_knot(knot, policy)? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        if refined.inserted_knot_count == 0 {
            return Ok(Classification::Decided((self.clone(), 0)));
        }
        let inserted_knot_count = refined.inserted_knot_count;
        let (control_points, weights) = match refined_affine_controls(&refined, policy)? {
            Classification::Decided(values) => values,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match Self::try_new_with_periodicity(
            self.degree,
            control_points,
            weights,
            refined.knots,
            self.periodicity.clone(),
            policy,
        )? {
            Classification::Decided(curve) => {
                Ok(Classification::Decided((curve, inserted_knot_count)))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) fn remove_knot(
        &self,
        knot: Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<Self>>> {
        let knot_index = match exact_knot_index(&self.knots, &knot, policy)? {
            Some(index) => index,
            None => return Ok(Classification::Decided(None)),
        };
        let mut coarse_knots = self.knots.clone();
        coarse_knots.remove(knot_index);
        let coarse_control_count = self.control_points.len() - 1;
        let Some(span) = find_insertion_span(
            &coarse_knots,
            self.degree,
            coarse_control_count,
            &knot,
            policy,
        )?
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        };
        let multiplicity = knot_multiplicity(&coarse_knots, &knot, policy)?;
        if multiplicity >= self.degree {
            return Ok(Classification::Decided(None));
        }

        let fine_controls = self
            .control_points
            .iter()
            .zip(&self.weights)
            .map(|(point, weight)| HomogeneousControl2::from_affine(point, weight))
            .collect::<Vec<_>>();
        let mut coarse_controls = vec![fine_controls[0].clone(); coarse_control_count];
        let left_end = span - self.degree;
        coarse_controls[..=left_end].clone_from_slice(&fine_controls[..=left_end]);
        let blend_end = span - multiplicity;
        for index in left_end + 1..=blend_end {
            let denominator = &coarse_knots[index + self.degree] - &coarse_knots[index];
            let alpha = match (knot.clone() - &coarse_knots[index]) / denominator {
                Ok(alpha) => alpha,
                Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            };
            coarse_controls[index] =
                coarse_controls[index - 1].inverse_lerp(&fine_controls[index], &alpha)?;
        }
        match coarse_controls[blend_end].exact_eq(&fine_controls[blend_end + 1], policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        coarse_controls[blend_end + 1..].clone_from_slice(&fine_controls[blend_end + 2..]);

        let (control_points, weights) = match homogeneous_affine_controls(&coarse_controls, policy)?
        {
            Classification::Decided(values) => values,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let candidate = match Self::try_new_with_periodicity(
            self.degree,
            control_points,
            weights,
            coarse_knots,
            self.periodicity.clone(),
            policy,
        )? {
            Classification::Decided(curve) => curve,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let replayed = match candidate.insert_knots(vec![knot], policy)? {
            Classification::Decided((curve, 1)) => curve,
            Classification::Decided(_) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        match rational_bspline_exact_eq(self, &replayed, policy) {
            Classification::Decided(true) => Ok(Classification::Decided(Some(candidate))),
            Classification::Decided(false) => Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Extracts retained rational Bezier spans by exact homogeneous knot insertion.
    ///
    /// Each distinct interior knot is inserted until its multiplicity equals
    /// the degree.  The resulting homogeneous control net is converted back to
    /// affine controls only after every refined weight is certified nonzero.
    /// This is Boehm knot insertion on homogeneous coordinates, following
    /// Boehm, "Inserting new knots into B-spline curves" (1980), de Boor,
    /// *A Practical Guide to Splines* (1978), and Farin, *Curves and Surfaces
    /// for CAGD* (5th ed., 2002).
    pub fn extract_bezier_spans(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBSplineBezierExtraction2>> {
        let mut refined = HomogeneousBSplineWorkingCurve {
            degree: self.degree,
            controls: self
                .control_points
                .iter()
                .zip(&self.weights)
                .map(|(point, weight)| HomogeneousControl2::from_affine(point, weight))
                .collect(),
            knots: self.knots.clone(),
            inserted_knot_count: 0,
        };
        let break_knots = match distinct_bezier_break_knots(&refined.knots, self.degree, policy) {
            Classification::Decided(knots) => knots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for knot in break_knots {
            loop {
                let multiplicity = knot_multiplicity(&refined.knots, &knot, policy)?;
                if multiplicity >= self.degree {
                    break;
                }
                match refined.insert_knot(knot.clone(), policy)? {
                    Classification::Decided(()) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        extract_refined_rational_spans(&refined, policy)
    }

    /// Builds a retained CAD-curve profile from exact rational B-spline evidence.
    ///
    /// The profile records the source knot domain, whole-domain trim, retained
    /// periodicity, exact extracted endpoint evidence, and a span cache summary.
    /// Unequal-weight cubics and higher-degree spans promote to exact general
    /// rational Beziers rather than being degree-reduced or tessellated. This
    /// is the same retained-object boundary described by Yap (1997), applied
    /// to NURBS carrier admission.
    pub fn retained_curve_profile(
        &self,
        source_index: u64,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedCurveProfile2>> {
        self.retained_curve_profile_with_source_version(source_index, 0, policy)
    }

    /// Builds a retained CAD-curve profile with source version/revision evidence.
    pub fn retained_curve_profile_with_source_version(
        &self,
        source_index: u64,
        source_version: u64,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedCurveProfile2>> {
        let domain = match bspline_parameter_domain(
            &self.knots,
            self.degree,
            self.control_points.len(),
            policy,
        )? {
            Classification::Decided(domain) => domain,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let trim = match default_trim(&domain, policy)? {
            Classification::Decided(trim) => trim,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let extraction = match self.extract_bezier_spans(policy)? {
            Classification::Decided(extraction) => extraction,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let report = match extraction.native_topology_report(policy)? {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let native_span_count = report
            .span_reports()
            .iter()
            .filter(|span| span.status().is_native_exact())
            .count();
        let retained_span_count = report.span_reports().len() - native_span_count;
        let topology_status = if report.is_fully_native_exact() {
            RetainedTopologyStatus::NativeExact
        } else {
            RetainedTopologyStatus::Unsupported
        };
        let cache_summary = RetainedCurveCacheSummary2::new_with_source_version(
            source_version,
            self.control_points.len(),
            self.knots.len(),
            report.span_reports().len(),
            native_span_count,
            retained_span_count,
        )?;

        Ok(Classification::Decided(RetainedCurveProfile2::new(
            RetainedCurveIdentity2::new_with_source_version(
                RetainedCurveFamily2::RationalBSpline,
                source_index,
                source_version,
            ),
            domain.clone(),
            trim,
            self.periodicity.clone(),
            topology_status,
            endpoint_evidence(
                extraction
                    .spans()
                    .first()
                    .and_then(|span| span.control_points().first())
                    .ok_or(CurveError::InvalidBSpline)?,
                extraction
                    .spans()
                    .last()
                    .and_then(|span| span.control_points().last())
                    .ok_or(CurveError::InvalidBSpline)?,
                &domain,
            ),
            cache_summary,
        )?))
    }
}

impl RationalBSplineBezierExtraction2 {
    /// Returns the retained source degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the exact refined affine control net after knot insertion.
    pub fn refined_control_points(&self) -> &[Point2] {
        &self.refined_control_points
    }

    /// Returns the exact refined homogeneous weights after knot insertion.
    pub fn refined_weights(&self) -> &[Real] {
        &self.refined_weights
    }

    /// Returns the exact refined knot vector after knot insertion.
    pub fn refined_knots(&self) -> &[Real] {
        &self.refined_knots
    }

    /// Returns extracted retained rational Bezier spans in parameter order.
    pub fn spans(&self) -> &[RationalBezierSpan2] {
        &self.spans
    }

    /// Converts every retained rational Bezier span that has native topology.
    ///
    /// This is a conservative bridge from retained NURBS evidence into the
    /// existing Bezier/conic topology kernel. Degree-one spans are elevated
    /// homogeneously, degree-two spans are native rational quadratics,
    /// equal-weight cubics collapse to polynomial cubics, and every remaining
    /// span stays an exact general rational Bezier without sampling or degree
    /// reduction. This is the Yap EGC boundary applied to NURBS consumption:
    /// branch into topology only after an exact representation-preserving
    /// construction; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 3-23 (1997).  The homogeneous Bezier
    /// interpretation follows Farin, *Curves and Surfaces for CAGD* (5th ed.,
    /// 2002).
    pub fn native_subcurves(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Vec<BezierSubcurve2>>> {
        let report = match self.native_topology_report(policy)? {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if !report.is_fully_native_exact() {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
        Ok(Classification::Decided(report.into_native_subcurves()))
    }

    /// Returns a per-span native-topology status report.
    ///
    /// Use this when retained NURBS evidence and its exact representation path
    /// must be inspected without sampling or flattening any span.
    pub fn native_topology_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBSplineNativeTopologyReport2>> {
        let mut span_reports = Vec::with_capacity(self.spans.len());
        for (span_index, span) in self.spans.iter().enumerate() {
            match span.native_topology_report(span_index, policy)? {
                Classification::Decided(report) => span_reports.push(report),
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        Ok(Classification::Decided(
            RationalBSplineNativeTopologyReport2::new(span_reports)?,
        ))
    }

    /// Returns how many knots were inserted to produce Bezier form.
    pub const fn inserted_knot_count(&self) -> usize {
        self.inserted_knot_count
    }

    /// Returns span-local bounds, monotonicity, and weight-domain facts.
    ///
    /// Native polynomial and rational spans reuse exact bounds and
    /// monotonicity certificates. General rational spans first use their
    /// homogeneous derivative Bernstein coefficients as a sign fast path,
    /// then isolate derivative roots exactly when the coefficients are mixed.
    pub fn span_fact_report(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RetainedBSplineSpanFactReport2>> {
        let topology = match self.native_topology_report(policy)? {
            Classification::Decided(report) => report,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let mut facts = Vec::with_capacity(self.spans.len());
        for (span_index, span) in self.spans.iter().enumerate() {
            let topology_report = &topology.span_reports()[span_index];
            let (bounds, x_monotonicity, y_monotonicity) =
                if let Some(native) = topology_report.native_subcurve() {
                    let bounds = match subcurve_certified_bounds(native, policy) {
                        Classification::Decided(bounds) => bounds,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    (
                        bounds,
                        match subcurve_axis_monotonicity(native, Axis2::X, policy) {
                            Classification::Decided(monotonicity) => monotonicity,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        },
                        match subcurve_axis_monotonicity(native, Axis2::Y, policy) {
                            Classification::Decided(monotonicity) => monotonicity,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        },
                    )
                } else {
                    let bounds = match Aabb2::from_points(span.control_points(), policy) {
                        Classification::Decided(bounds) => bounds,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    };
                    (
                        bounds,
                        RetainedSpanAxisMonotonicity::Unsupported,
                        RetainedSpanAxisMonotonicity::Unsupported,
                    )
                };
            facts.push(RetainedBSplineSpanFacts2::new(
                span_index,
                span.knot_start.clone(),
                span.knot_end.clone(),
                bounds,
                x_monotonicity,
                y_monotonicity,
                topology_report.status(),
                Some(weight_domain_report(span.weights(), policy)?),
            )?);
        }
        Ok(Classification::Decided(
            RetainedBSplineSpanFactReport2::new(facts)?,
        ))
    }
}

impl RetainedSpanWeightDomainReport2 {
    /// Constructs a retained span weight-domain report.
    pub fn new(
        weight_count: usize,
        certified_nonzero_count: usize,
        all_weights_certified_nonzero: bool,
    ) -> CurveResult<Self> {
        validate_weight_domain_report(
            weight_count,
            certified_nonzero_count,
            all_weights_certified_nonzero,
        )?;
        Ok(Self {
            weight_count,
            certified_nonzero_count,
            all_weights_certified_nonzero,
        })
    }

    /// Returns the number of weights in the span.
    pub const fn weight_count(&self) -> usize {
        self.weight_count
    }

    /// Returns how many weights were certified nonzero.
    pub const fn certified_nonzero_count(&self) -> usize {
        self.certified_nonzero_count
    }

    /// Returns true when every span weight is certified nonzero.
    pub const fn all_weights_certified_nonzero(&self) -> bool {
        self.all_weights_certified_nonzero
    }
}

impl RetainedBSplineSpanFacts2 {
    /// Constructs one span-local facts record.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        span_index: usize,
        knot_start: Real,
        knot_end: Real,
        bounds: Aabb2,
        x_monotonicity: RetainedSpanAxisMonotonicity,
        y_monotonicity: RetainedSpanAxisMonotonicity,
        topology_status: RetainedTopologyStatus,
        weight_domain: Option<RetainedSpanWeightDomainReport2>,
    ) -> CurveResult<Self> {
        validate_span_fact_evidence(
            &knot_start,
            &knot_end,
            &bounds,
            topology_status,
            x_monotonicity,
            y_monotonicity,
            weight_domain.as_ref(),
        )?;
        Ok(Self {
            span_index,
            knot_start,
            knot_end,
            bounds,
            x_monotonicity,
            y_monotonicity,
            topology_status,
            weight_domain,
        })
    }

    /// Returns the span index in extraction order.
    pub const fn span_index(&self) -> usize {
        self.span_index
    }

    /// Returns the source knot interval.
    pub fn knot_interval(&self) -> (&Real, &Real) {
        (&self.knot_start, &self.knot_end)
    }

    /// Returns the certified or conservative span AABB.
    pub const fn bounds(&self) -> &Aabb2 {
        &self.bounds
    }

    /// Returns x-axis monotonicity evidence.
    pub const fn x_monotonicity(&self) -> RetainedSpanAxisMonotonicity {
        self.x_monotonicity
    }

    /// Returns y-axis monotonicity evidence.
    pub const fn y_monotonicity(&self) -> RetainedSpanAxisMonotonicity {
        self.y_monotonicity
    }

    /// Returns the span topology status.
    pub const fn topology_status(&self) -> RetainedTopologyStatus {
        self.topology_status
    }

    /// Returns rational weight-domain evidence when the span is rational.
    pub const fn weight_domain(&self) -> Option<&RetainedSpanWeightDomainReport2> {
        self.weight_domain.as_ref()
    }
}

impl RetainedBSplineSpanFactReport2 {
    /// Constructs a span-local fact report.
    pub fn new(span_facts: Vec<RetainedBSplineSpanFacts2>) -> CurveResult<Self> {
        validate_span_fact_report_evidence(&span_facts)?;
        Ok(Self { span_facts })
    }

    /// Returns facts in extraction order.
    pub fn span_facts(&self) -> &[RetainedBSplineSpanFacts2] {
        &self.span_facts
    }
}

impl RationalBSplineNativeTopologyReport2 {
    /// Constructs a rational B-spline topology report from per-span reports.
    pub fn new(span_reports: Vec<RationalBezierSpanTopologyReport2>) -> CurveResult<Self> {
        validate_span_topology_report_evidence(&span_reports)?;
        Ok(Self { span_reports })
    }

    /// Returns the per-span topology reports in source parameter order.
    pub fn span_reports(&self) -> &[RationalBezierSpanTopologyReport2] {
        &self.span_reports
    }

    /// Returns true when every retained span promoted to exact native topology.
    pub fn is_fully_native_exact(&self) -> bool {
        self.span_reports
            .iter()
            .all(|report| report.status().is_native_exact())
    }

    /// Consumes the report and returns only native subcurves.
    ///
    /// Call this only after [`Self::is_fully_native_exact`] succeeds. If a
    /// caller ignores that precondition, non-native spans are still not
    /// synthesized.
    pub fn into_native_subcurves(self) -> Vec<BezierSubcurve2> {
        self.span_reports
            .into_iter()
            .filter_map(|report| report.native_subcurve)
            .collect()
    }
}

impl RationalBezierSpanTopologyReport2 {
    /// Constructs one retained span topology report.
    pub fn new(
        span_index: usize,
        degree: usize,
        knot_start: Real,
        knot_end: Real,
        status: RetainedTopologyStatus,
        decision_path: RationalBezierSpanTopologyPath2,
        native_subcurve: Option<BezierSubcurve2>,
    ) -> CurveResult<Self> {
        validate_rational_span_topology_evidence(
            degree,
            &knot_start,
            &knot_end,
            status,
            decision_path,
            native_subcurve.as_ref(),
        )?;
        Ok(Self {
            span_index,
            degree,
            knot_start,
            knot_end,
            status,
            decision_path,
            native_subcurve,
        })
    }

    /// Returns the span index within the extraction report.
    pub const fn span_index(&self) -> usize {
        self.span_index
    }

    /// Returns the retained rational Bezier degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the source knot interval covered by this span.
    pub fn knot_interval(&self) -> (&Real, &Real) {
        (&self.knot_start, &self.knot_end)
    }

    /// Returns the span's topology-readiness status.
    pub const fn status(&self) -> RetainedTopologyStatus {
        self.status
    }

    /// Returns the exact decision path that produced this status.
    pub const fn decision_path(&self) -> RationalBezierSpanTopologyPath2 {
        self.decision_path
    }

    /// Returns the exact native subcurve when one exists.
    pub const fn native_subcurve(&self) -> Option<&BezierSubcurve2> {
        self.native_subcurve.as_ref()
    }
}

fn validate_weight_domain_report(
    weight_count: usize,
    certified_nonzero_count: usize,
    all_weights_certified_nonzero: bool,
) -> CurveResult<()> {
    if weight_count == 0 || certified_nonzero_count > weight_count {
        return Err(CurveError::Topology(
            "retained span weight report count evidence is inconsistent".into(),
        ));
    }
    if all_weights_certified_nonzero != (certified_nonzero_count == weight_count) {
        return Err(CurveError::Topology(
            "retained span weight report all-nonzero flag does not match certified count".into(),
        ));
    }
    Ok(())
}

fn validate_span_fact_evidence(
    knot_start: &Real,
    knot_end: &Real,
    bounds: &Aabb2,
    topology_status: RetainedTopologyStatus,
    x_monotonicity: RetainedSpanAxisMonotonicity,
    y_monotonicity: RetainedSpanAxisMonotonicity,
    weight_domain: Option<&RetainedSpanWeightDomainReport2>,
) -> CurveResult<()> {
    validate_positive_knot_interval(knot_start, knot_end)?;
    match bounds.has_valid_ordering(&CurvePolicy::certified()) {
        Classification::Decided(true) => {}
        Classification::Decided(false) => {
            return Err(CurveError::Topology(
                "retained span facts must carry a well-ordered bounding box".into(),
            ));
        }
        Classification::Uncertain(reason) => {
            return Err(CurveError::Topology(format!(
                "retained span fact bounds ordering is uncertified: {reason:?}"
            )));
        }
    }
    if !topology_status.is_native_exact()
        && (x_monotonicity != RetainedSpanAxisMonotonicity::Unsupported
            || y_monotonicity != RetainedSpanAxisMonotonicity::Unsupported)
    {
        return Err(CurveError::Topology(
            "non-native retained span facts must not claim certified monotonicity".into(),
        ));
    }
    if !topology_status.is_native_exact() && !topology_status.is_retained_evidence() {
        return Err(CurveError::Topology(
            "retained B-spline span facts must carry exact native or retained evidence status"
                .into(),
        ));
    }
    if topology_status.is_retained_evidence() && weight_domain.is_none() {
        return Err(CurveError::Topology(
            "retained non-native B-spline span facts must carry rational weight-domain evidence"
                .into(),
        ));
    }
    if topology_status.is_native_exact()
        && (x_monotonicity == RetainedSpanAxisMonotonicity::Unsupported
            || y_monotonicity == RetainedSpanAxisMonotonicity::Unsupported)
    {
        return Err(CurveError::Topology(
            "native retained span facts must carry exact monotonicity evidence".into(),
        ));
    }
    if topology_status.is_native_exact()
        && weight_domain.is_some_and(|domain| !domain.all_weights_certified_nonzero())
    {
        return Err(CurveError::Topology(
            "native retained rational span facts must carry all-nonzero weight evidence".into(),
        ));
    }
    Ok(())
}

fn validate_span_fact_report_evidence(span_facts: &[RetainedBSplineSpanFacts2]) -> CurveResult<()> {
    if span_facts.is_empty() {
        return Err(CurveError::Topology(
            "retained span fact report must carry at least one span".into(),
        ));
    }
    let policy = CurvePolicy::certified();
    for (expected_index, fact) in span_facts.iter().enumerate() {
        if fact.span_index() != expected_index {
            return Err(CurveError::Topology(
                "retained span fact report indices must be contiguous".into(),
            ));
        }
        if let Some(previous) = expected_index
            .checked_sub(1)
            .and_then(|index| span_facts.get(index))
        {
            validate_adjacent_knot_windows(
                previous.knot_interval().1,
                fact.knot_interval().0,
                &policy,
                "retained span fact report knot intervals must be contiguous",
            )?;
        }
    }
    Ok(())
}

fn validate_span_topology_report_evidence(
    span_reports: &[RationalBezierSpanTopologyReport2],
) -> CurveResult<()> {
    if span_reports.is_empty() {
        return Err(CurveError::Topology(
            "retained span topology report must carry at least one span".into(),
        ));
    }
    let degree = span_reports[0].degree();
    let policy = CurvePolicy::certified();
    for (expected_index, report) in span_reports.iter().enumerate() {
        if report.span_index() != expected_index {
            return Err(CurveError::Topology(
                "retained span topology report indices must be contiguous".into(),
            ));
        }
        if report.degree() != degree {
            return Err(CurveError::Topology(
                "retained span topology report degrees must match".into(),
            ));
        }
        if let Some(previous) = expected_index
            .checked_sub(1)
            .and_then(|index| span_reports.get(index))
        {
            validate_adjacent_knot_windows(
                previous.knot_interval().1,
                report.knot_interval().0,
                &policy,
                "retained span topology report knot intervals must be contiguous",
            )?;
        }
    }
    Ok(())
}

fn validate_rational_span_topology_evidence(
    degree: usize,
    knot_start: &Real,
    knot_end: &Real,
    status: RetainedTopologyStatus,
    decision_path: RationalBezierSpanTopologyPath2,
    native_subcurve: Option<&BezierSubcurve2>,
) -> CurveResult<()> {
    validate_positive_knot_interval(knot_start, knot_end)?;
    if degree < 1 {
        return Err(CurveError::Topology(
            "retained rational span topology report degree must be at least one".into(),
        ));
    }
    if !status.is_native_exact() && status != RetainedTopologyStatus::Unsupported {
        return Err(CurveError::Topology(
            "retained rational span topology report must carry exact native or unsupported evidence status"
                .into(),
        ));
    }
    let path_matches_status = match decision_path {
        RationalBezierSpanTopologyPath2::RetainedControlNetShapeMismatch => {
            status == RetainedTopologyStatus::Unsupported && native_subcurve.is_none()
        }
        RationalBezierSpanTopologyPath2::NativeRationalLinearSpan => {
            degree == 1
                && status.is_native_exact()
                && matches!(native_subcurve, Some(BezierSubcurve2::RationalQuadratic(_)))
        }
        RationalBezierSpanTopologyPath2::RetainedSingularLinearSpan => {
            degree == 1
                && status == RetainedTopologyStatus::Unsupported
                && native_subcurve.is_none()
        }
        RationalBezierSpanTopologyPath2::NativeRationalQuadraticSpan => {
            degree == 2
                && status.is_native_exact()
                && matches!(native_subcurve, Some(BezierSubcurve2::RationalQuadratic(_)))
        }
        RationalBezierSpanTopologyPath2::NativeEqualWeightCubicSpan => {
            degree == 3
                && status.is_native_exact()
                && matches!(native_subcurve, Some(BezierSubcurve2::Cubic(_)))
        }
        RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan => {
            degree >= 3
                && status.is_native_exact()
                && matches!(native_subcurve, Some(BezierSubcurve2::Rational(_)))
        }
    };
    if !path_matches_status {
        return Err(CurveError::Topology(
            "retained rational span topology path does not match status evidence".into(),
        ));
    }
    match (status.is_native_exact(), native_subcurve) {
        (true, Some(BezierSubcurve2::RationalQuadratic(_))) if degree == 1 || degree == 2 => Ok(()),
        (true, Some(BezierSubcurve2::Cubic(_))) if degree == 3 => Ok(()),
        (true, Some(BezierSubcurve2::Rational(_))) if degree >= 3 => Ok(()),
        (true, Some(_)) => Err(CurveError::Topology(
            "native rational span topology report subcurve does not match retained degree".into(),
        )),
        (true, None) => Err(CurveError::Topology(
            "native rational span topology report must carry a native subcurve".into(),
        )),
        (false, Some(_)) => Err(CurveError::Topology(
            "non-native rational span topology report must not carry a native subcurve".into(),
        )),
        (false, None) => Ok(()),
    }
}

fn validate_positive_knot_interval(knot_start: &Real, knot_end: &Real) -> CurveResult<()> {
    let policy = CurvePolicy::certified();
    if compare_reals(knot_start, knot_end, &policy) != Some(Ordering::Less) {
        return Err(CurveError::Topology(
            "retained B-spline span report must carry certified positive knot interval".into(),
        ));
    }
    Ok(())
}

fn validate_adjacent_knot_windows(
    previous_end: &Real,
    next_start: &Real,
    policy: &CurvePolicy,
    message: &str,
) -> CurveResult<()> {
    if compare_reals(previous_end, next_start, policy) != Some(Ordering::Equal) {
        return Err(CurveError::Topology(message.into()));
    }
    Ok(())
}

impl RationalBezierSpan2 {
    /// Returns the Bezier degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns exact affine control points for this retained rational span.
    pub fn control_points(&self) -> &[Point2] {
        &self.control_points
    }

    /// Returns exact homogeneous weights for this retained rational span.
    pub fn weights(&self) -> &[Real] {
        &self.weights
    }

    /// Returns the source knot interval covered by this Bezier span.
    pub fn knot_interval(&self) -> (&Real, &Real) {
        (&self.knot_start, &self.knot_end)
    }

    /// Converts this retained rational Bezier span into native topology when exact.
    ///
    /// Degree-one spans are elevated exactly in homogeneous coordinates and
    /// degree-two spans map directly to [`RationalQuadraticBezier2`]. A
    /// degree-three rational span maps to [`CubicBezier2`] when all homogeneous
    /// weights are exactly equal, because the rational Bezier denominator is
    /// then one common scale on the full parameter interval. Unequal-weight
    /// cubics and every higher degree map to exact [`RationalBezier2`] topology.
    pub fn native_subcurve(
        &self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<BezierSubcurve2>> {
        match self.native_topology_report(0, policy)? {
            Classification::Decided(report) => match report.native_subcurve {
                Some(subcurve) => Ok(Classification::Decided(subcurve)),
                None => Ok(Classification::Uncertain(UncertaintyReason::Unsupported)),
            },
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns the exact native-topology status for this retained rational span.
    pub fn native_topology_report(
        &self,
        span_index: usize,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBezierSpanTopologyReport2>> {
        if self.control_points.len() != self.degree + 1 || self.weights.len() != self.degree + 1 {
            return Ok(Classification::Decided(
                RationalBezierSpanTopologyReport2::new(
                    span_index,
                    self.degree,
                    self.knot_start.clone(),
                    self.knot_end.clone(),
                    RetainedTopologyStatus::Unsupported,
                    RationalBezierSpanTopologyPath2::RetainedControlNetShapeMismatch,
                    None,
                )?,
            ));
        }
        match self.degree {
            1 => {
                let weight_sum = &self.weights[0] + &self.weights[1];
                match is_zero(&weight_sum, policy) {
                    Some(true) => Ok(Classification::Decided(
                        RationalBezierSpanTopologyReport2::new(
                            span_index,
                            self.degree,
                            self.knot_start.clone(),
                            self.knot_end.clone(),
                            RetainedTopologyStatus::Unsupported,
                            RationalBezierSpanTopologyPath2::RetainedSingularLinearSpan,
                            None,
                        )?,
                    )),
                    Some(false) => {
                        let two = Real::from(2_i8);
                        let middle_weight = (&weight_sum / &two)?;
                        let middle_x = ((self.control_points[0].x() * &self.weights[0]
                            + self.control_points[1].x() * &self.weights[1])
                            / &weight_sum)?;
                        let middle_y = ((self.control_points[0].y() * &self.weights[0]
                            + self.control_points[1].y() * &self.weights[1])
                            / weight_sum)?;
                        let curve = RationalQuadraticBezier2::try_new(
                            self.control_points[0].clone(),
                            Point2::new(middle_x, middle_y),
                            self.control_points[1].clone(),
                            self.weights[0].clone(),
                            middle_weight,
                            self.weights[1].clone(),
                        )?;
                        Ok(Classification::Decided(
                            RationalBezierSpanTopologyReport2::new(
                                span_index,
                                self.degree,
                                self.knot_start.clone(),
                                self.knot_end.clone(),
                                RetainedTopologyStatus::NativeExact,
                                RationalBezierSpanTopologyPath2::NativeRationalLinearSpan,
                                Some(BezierSubcurve2::RationalQuadratic(curve)),
                            )?,
                        ))
                    }
                    None => Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
                }
            }
            2 => {
                let curve = RationalQuadraticBezier2::try_new(
                    self.control_points[0].clone(),
                    self.control_points[1].clone(),
                    self.control_points[2].clone(),
                    self.weights[0].clone(),
                    self.weights[1].clone(),
                    self.weights[2].clone(),
                )?;
                Ok(Classification::Decided(
                    RationalBezierSpanTopologyReport2::new(
                        span_index,
                        self.degree,
                        self.knot_start.clone(),
                        self.knot_end.clone(),
                        RetainedTopologyStatus::NativeExact,
                        RationalBezierSpanTopologyPath2::NativeRationalQuadraticSpan,
                        Some(BezierSubcurve2::RationalQuadratic(curve)),
                    )?,
                ))
            }
            3 => match weights_are_all_equal(&self.weights, policy) {
                Classification::Decided(true) => Ok(Classification::Decided(
                    RationalBezierSpanTopologyReport2::new(
                        span_index,
                        self.degree,
                        self.knot_start.clone(),
                        self.knot_end.clone(),
                        RetainedTopologyStatus::NativeExact,
                        RationalBezierSpanTopologyPath2::NativeEqualWeightCubicSpan,
                        Some(BezierSubcurve2::Cubic(CubicBezier2::new(
                            self.control_points[0].clone(),
                            self.control_points[1].clone(),
                            self.control_points[2].clone(),
                            self.control_points[3].clone(),
                        ))),
                    )?,
                )),
                Classification::Decided(false) | Classification::Uncertain(_) => {
                    general_rational_span_topology_report(self, span_index)
                }
            },
            _ => general_rational_span_topology_report(self, span_index),
        }
    }
}

fn general_rational_span_topology_report(
    span: &RationalBezierSpan2,
    span_index: usize,
) -> CurveResult<Classification<RationalBezierSpanTopologyReport2>> {
    let curve = crate::RationalBezier2::try_new(span.control_points.clone(), span.weights.clone())?;
    Ok(Classification::Decided(
        RationalBezierSpanTopologyReport2::new(
            span_index,
            span.degree,
            span.knot_start.clone(),
            span.knot_end.clone(),
            RetainedTopologyStatus::NativeExact,
            RationalBezierSpanTopologyPath2::NativeGeneralRationalSpan,
            Some(BezierSubcurve2::Rational(curve)),
        )?,
    ))
}

#[derive(Clone, Debug)]
struct BSplineWorkingCurve {
    degree: usize,
    control_points: Vec<Point2>,
    knots: Vec<Real>,
    inserted_knot_count: usize,
}

#[derive(Clone, Debug)]
struct HomogeneousControl2 {
    x: Real,
    y: Real,
    weight: Real,
}

#[derive(Clone, Debug)]
struct HomogeneousBSplineWorkingCurve {
    degree: usize,
    controls: Vec<HomogeneousControl2>,
    knots: Vec<Real>,
    inserted_knot_count: usize,
}

impl HomogeneousControl2 {
    fn from_affine(point: &Point2, weight: &Real) -> Self {
        Self {
            x: point.x() * weight,
            y: point.y() * weight,
            weight: weight.clone(),
        }
    }

    fn lerp(&self, other: &Self, t: Real) -> Self {
        let one_minus_t = Real::one() - &t;
        Self {
            x: (&self.x * &one_minus_t) + (&other.x * &t),
            y: (&self.y * &one_minus_t) + (&other.y * &t),
            weight: (&self.weight * &one_minus_t) + (&other.weight * &t),
        }
    }

    fn inverse_lerp(&self, blended: &Self, t: &Real) -> CurveResult<Self> {
        let one_minus_t = Real::one() - t;
        Ok(Self {
            x: ((blended.x.clone() - &self.x * &one_minus_t) / t.clone())?,
            y: ((blended.y.clone() - &self.y * &one_minus_t) / t.clone())?,
            weight: ((blended.weight.clone() - &self.weight * &one_minus_t) / t.clone())?,
        })
    }

    fn exact_eq(&self, other: &Self, policy: &CurvePolicy) -> Classification<bool> {
        for (first, second) in [
            (&self.x, &other.x),
            (&self.y, &other.y),
            (&self.weight, &other.weight),
        ] {
            match compare_reals(first, second, policy) {
                Some(Ordering::Equal) => {}
                Some(_) => return Classification::Decided(false),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        Classification::Decided(true)
    }

    fn to_affine(&self, policy: &CurvePolicy) -> CurveResult<Classification<(Point2, Real)>> {
        match is_zero(&self.weight, policy) {
            Some(false) => {}
            Some(true) => return Err(CurveError::ZeroRationalBezierWeight),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        let x = (&self.x / &self.weight)?;
        let y = (&self.y / &self.weight)?;
        Ok(Classification::Decided((
            Point2::new(x, y),
            self.weight.clone(),
        )))
    }
}

impl BSplineWorkingCurve {
    fn insert_knot(&mut self, knot: Real, policy: &CurvePolicy) -> CurveResult<Classification<()>> {
        let Some(span) = find_insertion_span(
            &self.knots,
            self.degree,
            self.control_points.len(),
            &knot,
            policy,
        )?
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        };
        let multiplicity = knot_multiplicity(&self.knots, &knot, policy)?;
        if multiplicity >= self.degree {
            return Ok(Classification::Decided(()));
        }

        let n = self.control_points.len() - 1;
        let p = self.degree;
        let mut new_points = vec![self.control_points[0].clone(); self.control_points.len() + 1];
        for (i, point) in new_points
            .iter_mut()
            .enumerate()
            .take(span.saturating_sub(p) + 1)
        {
            *point = self.control_points[i].clone();
        }
        let right_start = span - multiplicity + 1;
        new_points[right_start..=n + 1].clone_from_slice(&self.control_points[right_start - 1..=n]);
        for (i, point) in new_points
            .iter_mut()
            .enumerate()
            .take(span - multiplicity + 1)
            .skip(span - p + 1)
        {
            let denominator = &self.knots[i + p] - &self.knots[i];
            let alpha = match (knot.clone() - &self.knots[i]) / denominator {
                Ok(alpha) => alpha,
                Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            };
            *point = self.control_points[i - 1].lerp(&self.control_points[i], alpha);
        }

        self.knots.insert(span + 1, knot);
        self.control_points = new_points;
        self.inserted_knot_count += 1;
        Ok(Classification::Decided(()))
    }
}

impl HomogeneousBSplineWorkingCurve {
    fn insert_knot(&mut self, knot: Real, policy: &CurvePolicy) -> CurveResult<Classification<()>> {
        let Some(span) =
            find_insertion_span(&self.knots, self.degree, self.controls.len(), &knot, policy)?
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        };
        let multiplicity = knot_multiplicity(&self.knots, &knot, policy)?;
        if multiplicity >= self.degree {
            return Ok(Classification::Decided(()));
        }

        let n = self.controls.len() - 1;
        let p = self.degree;
        let mut new_controls = vec![self.controls[0].clone(); self.controls.len() + 1];
        for (i, control) in new_controls
            .iter_mut()
            .enumerate()
            .take(span.saturating_sub(p) + 1)
        {
            *control = self.controls[i].clone();
        }
        let right_start = span - multiplicity + 1;
        new_controls[right_start..=n + 1].clone_from_slice(&self.controls[right_start - 1..=n]);
        for (i, control) in new_controls
            .iter_mut()
            .enumerate()
            .take(span - multiplicity + 1)
            .skip(span - p + 1)
        {
            let denominator = &self.knots[i + p] - &self.knots[i];
            let alpha = match (knot.clone() - &self.knots[i]) / denominator {
                Ok(alpha) => alpha,
                Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            };
            *control = self.controls[i - 1].lerp(&self.controls[i], alpha);
        }

        self.knots.insert(span + 1, knot);
        self.controls = new_controls;
        self.inserted_knot_count += 1;
        Ok(Classification::Decided(()))
    }
}

fn validate_nondecreasing_knots(knots: &[Real], policy: &CurvePolicy) -> Classification<()> {
    for pair in knots.windows(2) {
        match compare_reals(&pair[0], &pair[1], policy) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => {
                return Classification::Uncertain(UncertaintyReason::Ordering);
            }
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(())
}

fn has_positive_span(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    policy: &CurvePolicy,
) -> CurveResult<bool> {
    for i in degree..control_count {
        if compare_reals(&knots[i], &knots[i + 1], policy) == Some(Ordering::Less) {
            return Ok(true);
        }
    }
    Ok(false)
}

fn validate_spline_periodicity(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    periodicity: &SplinePeriodicity2,
    policy: &CurvePolicy,
) -> CurveResult<Classification<()>> {
    let SplinePeriodicity2::Periodic { period } = periodicity else {
        return Ok(Classification::Decided(()));
    };
    match compare_reals(&Real::zero(), period, policy) {
        Some(Ordering::Less) => {}
        Some(_) => return Err(CurveError::InvalidPeriodicSpline),
        None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
    let domain_width = &knots[control_count] - &knots[degree];
    match compare_reals(&domain_width, period, policy) {
        Some(Ordering::Equal) => Ok(Classification::Decided(())),
        Some(_) => Err(CurveError::InvalidPeriodicSpline),
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

fn native_span_fact_report(
    spans: &[BezierSubcurve2],
    refined_knots: &[Real],
    degree: usize,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RetainedBSplineSpanFactReport2>> {
    let mut facts = Vec::with_capacity(spans.len());
    let mut span_index = 0_usize;
    let refined_control_count = refined_knots.len().saturating_sub(degree + 1);
    for knot_index in degree..refined_control_count {
        if compare_reals(
            &refined_knots[knot_index],
            &refined_knots[knot_index + 1],
            policy,
        ) != Some(Ordering::Less)
        {
            continue;
        }
        let Some(span) = spans.get(span_index) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let bounds = match subcurve_certified_bounds(span, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        facts.push(RetainedBSplineSpanFacts2::new(
            span_index,
            refined_knots[knot_index].clone(),
            refined_knots[knot_index + 1].clone(),
            bounds,
            match subcurve_axis_monotonicity(span, Axis2::X, policy) {
                Classification::Decided(monotonicity) => monotonicity,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            },
            match subcurve_axis_monotonicity(span, Axis2::Y, policy) {
                Classification::Decided(monotonicity) => monotonicity,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            },
            RetainedTopologyStatus::NativeExact,
            None,
        )?);
        span_index += 1;
    }
    if span_index != spans.len() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    Ok(Classification::Decided(
        RetainedBSplineSpanFactReport2::new(facts)?,
    ))
}

fn subcurve_certified_bounds(
    curve: &BezierSubcurve2,
    policy: &CurvePolicy,
) -> Classification<Aabb2> {
    match curve {
        BezierSubcurve2::Quadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Cubic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.certified_bounds(policy),
        BezierSubcurve2::Rational(curve) => curve.certified_bounds_classified(policy),
    }
}

fn subcurve_axis_monotonicity(
    curve: &BezierSubcurve2,
    axis: Axis2,
    policy: &CurvePolicy,
) -> Classification<RetainedSpanAxisMonotonicity> {
    let roots = match curve {
        BezierSubcurve2::Quadratic(curve) => curve.axis_monotone_parameters(axis, policy),
        BezierSubcurve2::Cubic(curve) => curve.axis_monotone_parameters(axis, policy),
        BezierSubcurve2::RationalQuadratic(curve) => curve.axis_monotone_parameters(axis, policy),
        BezierSubcurve2::Rational(curve) => {
            return match curve.axis_monotonicity_classified(axis, policy) {
                Ok(Classification::Decided(true)) => {
                    Classification::Decided(RetainedSpanAxisMonotonicity::CertifiedMonotone)
                }
                Ok(Classification::Decided(false)) => {
                    Classification::Decided(RetainedSpanAxisMonotonicity::HasInteriorExtrema)
                }
                Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                Err(CurveError::Real(_)) => Classification::Uncertain(UncertaintyReason::RealSign),
                Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
            };
        }
    };
    match roots {
        Classification::Decided(roots) if roots.is_empty() => {
            Classification::Decided(RetainedSpanAxisMonotonicity::CertifiedMonotone)
        }
        Classification::Decided(_) => {
            Classification::Decided(RetainedSpanAxisMonotonicity::HasInteriorExtrema)
        }
        Classification::Uncertain(reason) => Classification::Uncertain(reason),
    }
}

fn weight_domain_report(
    weights: &[Real],
    policy: &CurvePolicy,
) -> CurveResult<RetainedSpanWeightDomainReport2> {
    let mut certified_nonzero_count = 0_usize;
    for weight in weights {
        match is_zero(weight, policy) {
            Some(false) => certified_nonzero_count += 1,
            Some(true) => return Err(CurveError::ZeroRationalBezierWeight),
            None => {}
        }
    }
    RetainedSpanWeightDomainReport2::new(
        weights.len(),
        certified_nonzero_count,
        certified_nonzero_count == weights.len(),
    )
}

fn bspline_parameter_domain(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RetainedParameterDomain1>> {
    let Some(start) = knots.get(degree) else {
        return Err(CurveError::InvalidBSpline);
    };
    let Some(end) = knots.get(control_count) else {
        return Err(CurveError::InvalidBSpline);
    };
    RetainedParameterDomain1::try_new(start.clone(), end.clone(), policy)
}

fn default_trim(
    domain: &RetainedParameterDomain1,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RetainedTrimInterval1>> {
    RetainedTrimInterval1::try_new(domain.start().clone(), domain.end().clone(), domain, policy)
}

fn endpoint_evidence(
    start_point: &Point2,
    end_point: &Point2,
    domain: &RetainedParameterDomain1,
) -> RetainedEndpointEvidence2 {
    RetainedEndpointEvidence2::new(domain, start_point.clone(), end_point.clone())
}

fn distinct_bezier_break_knots(
    knots: &[Real],
    degree: usize,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    let mut result = Vec::new();
    for knot in &knots[degree..=knots.len() - degree - 1] {
        if result
            .last()
            .is_some_and(|last| compare_reals(last, knot, policy) == Some(Ordering::Equal))
        {
            continue;
        }
        result.push(knot.clone());
    }
    Classification::Decided(result)
}

fn knot_multiplicity(knots: &[Real], knot: &Real, policy: &CurvePolicy) -> CurveResult<usize> {
    let mut count = 0;
    for candidate in knots {
        match compare_reals(candidate, knot, policy) {
            Some(Ordering::Equal) => count += 1,
            Some(Ordering::Less | Ordering::Greater) => {}
            None => return Err(CurveError::InvalidBSpline),
        }
    }
    Ok(count)
}

fn weights_are_all_equal(weights: &[Real], policy: &CurvePolicy) -> Classification<bool> {
    let Some(first) = weights.first() else {
        return Classification::Uncertain(UncertaintyReason::Unsupported);
    };
    for weight in &weights[1..] {
        match compare_reals(first, weight, policy) {
            Some(Ordering::Equal) => {}
            Some(Ordering::Less | Ordering::Greater) => return Classification::Decided(false),
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(true)
}

fn find_insertion_span(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    knot: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Option<usize>> {
    let n = control_count - 1;
    if compare_reals(knot, &knots[n + 1], policy) == Some(Ordering::Equal) {
        return Ok(Some(if n + 1 < knots.len() - 1 { n + 1 } else { n }));
    }
    for span in degree..=n {
        let left = compare_reals(&knots[span], knot, policy);
        let right = compare_reals(knot, &knots[span + 1], policy);
        match (left, right) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less)) => {
                return Ok(Some(span));
            }
            (Some(_), Some(_)) => {}
            _ => return Ok(None),
        }
    }
    Ok(None)
}

fn extract_refined_bezier_spans(
    refined: &BSplineWorkingCurve,
    policy: &CurvePolicy,
) -> CurveResult<Classification<Vec<BezierSubcurve2>>> {
    let mut spans = Vec::new();
    let linear_half = if refined.degree == 1 {
        Some((Real::one() / Real::from(2_i8))?)
    } else {
        None
    };
    for knot_index in refined.degree..refined.control_points.len() {
        if compare_reals(
            &refined.knots[knot_index],
            &refined.knots[knot_index + 1],
            policy,
        ) != Some(Ordering::Less)
        {
            continue;
        }
        let start = knot_index - refined.degree;
        let controls = &refined.control_points[start..=knot_index];
        let span = match refined.degree {
            1 => BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                controls[0].clone(),
                controls[0].lerp(
                    &controls[1],
                    linear_half
                        .as_ref()
                        .expect("linear span extraction retained its elevation parameter")
                        .clone(),
                ),
                controls[1].clone(),
            )),
            2 => BezierSubcurve2::Quadratic(QuadraticBezier2::new(
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
            )),
            3 => BezierSubcurve2::Cubic(CubicBezier2::new(
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
                controls[3].clone(),
            )),
            _ => BezierSubcurve2::Rational(RationalBezier2::try_new(
                controls.to_vec(),
                vec![Real::one(); controls.len()],
            )?),
        };
        spans.push(span);
    }
    Ok(Classification::Decided(spans))
}

fn extract_refined_rational_quadratic_spans(
    refined: &HomogeneousBSplineWorkingCurve,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RationalQuadraticBSplineBezierExtraction2>> {
    let mut affine_controls = Vec::with_capacity(refined.controls.len());
    let mut weights = Vec::with_capacity(refined.controls.len());
    for control in &refined.controls {
        match control.to_affine(policy)? {
            Classification::Decided((point, weight)) => {
                affine_controls.push(point);
                weights.push(weight);
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }

    let mut spans = Vec::new();
    for knot_index in refined.degree..refined.controls.len() {
        if compare_reals(
            &refined.knots[knot_index],
            &refined.knots[knot_index + 1],
            policy,
        ) != Some(Ordering::Less)
        {
            continue;
        }
        let start = knot_index - refined.degree;
        let curve = RationalQuadraticBezier2::try_new(
            affine_controls[start].clone(),
            affine_controls[start + 1].clone(),
            affine_controls[start + 2].clone(),
            weights[start].clone(),
            weights[start + 1].clone(),
            weights[start + 2].clone(),
        )?;
        spans.push(BezierSubcurve2::RationalQuadratic(curve));
    }

    Ok(Classification::Decided(
        RationalQuadraticBSplineBezierExtraction2 {
            refined_control_points: affine_controls,
            refined_weights: weights,
            refined_knots: refined.knots.clone(),
            spans,
            inserted_knot_count: refined.inserted_knot_count,
        },
    ))
}

fn extract_refined_rational_spans(
    refined: &HomogeneousBSplineWorkingCurve,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RationalBSplineBezierExtraction2>> {
    let (affine_controls, weights) = match refined_affine_controls(refined, policy)? {
        Classification::Decided(refined) => refined,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };

    let mut spans = Vec::new();
    for knot_index in refined.degree..refined.controls.len() {
        if compare_reals(
            &refined.knots[knot_index],
            &refined.knots[knot_index + 1],
            policy,
        ) != Some(Ordering::Less)
        {
            continue;
        }
        let start = knot_index - refined.degree;
        spans.push(RationalBezierSpan2 {
            degree: refined.degree,
            control_points: affine_controls[start..=knot_index].to_vec(),
            weights: weights[start..=knot_index].to_vec(),
            knot_start: refined.knots[knot_index].clone(),
            knot_end: refined.knots[knot_index + 1].clone(),
        });
    }

    Ok(Classification::Decided(RationalBSplineBezierExtraction2 {
        degree: refined.degree,
        refined_control_points: affine_controls,
        refined_weights: weights,
        refined_knots: refined.knots.clone(),
        spans,
        inserted_knot_count: refined.inserted_knot_count,
    }))
}

fn refined_affine_controls(
    refined: &HomogeneousBSplineWorkingCurve,
    policy: &CurvePolicy,
) -> CurveResult<Classification<(Vec<Point2>, Vec<Real>)>> {
    homogeneous_affine_controls(&refined.controls, policy)
}

fn homogeneous_affine_controls(
    controls: &[HomogeneousControl2],
    policy: &CurvePolicy,
) -> CurveResult<Classification<(Vec<Point2>, Vec<Real>)>> {
    let mut affine_controls = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for control in controls {
        match control.to_affine(policy)? {
            Classification::Decided((point, weight)) => {
                affine_controls.push(point);
                weights.push(weight);
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
    }
    Ok(Classification::Decided((affine_controls, weights)))
}

fn exact_knot_index(
    knots: &[Real],
    knot: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Option<usize>> {
    for (index, candidate) in knots.iter().enumerate() {
        match compare_reals(candidate, knot, policy) {
            Some(Ordering::Equal) => return Ok(Some(index)),
            Some(_) => {}
            None => return Err(CurveError::InvalidBSpline),
        }
    }
    Ok(None)
}

fn rational_bspline_exact_eq(
    first: &RationalBSplineCurve2,
    second: &RationalBSplineCurve2,
    policy: &CurvePolicy,
) -> Classification<bool> {
    if first.degree != second.degree
        || first.control_points.len() != second.control_points.len()
        || first.knots.len() != second.knots.len()
        || first.periodicity != second.periodicity
    {
        return Classification::Decided(false);
    }
    for (first, second) in first.knots.iter().zip(&second.knots) {
        match compare_reals(first, second, policy) {
            Some(Ordering::Equal) => {}
            Some(_) => return Classification::Decided(false),
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    for ((first_point, first_weight), (second_point, second_weight)) in first
        .control_points
        .iter()
        .zip(&first.weights)
        .zip(second.control_points.iter().zip(&second.weights))
    {
        let first = HomogeneousControl2::from_affine(first_point, first_weight);
        let second = HomogeneousControl2::from_affine(second_point, second_weight);
        match first.exact_eq(&second, policy) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Classification::Decided(false),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(true)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(x: i32, y: i32) -> Point2 {
        Point2::new(x.into(), y.into())
    }

    fn periodic_controls() -> Vec<Point2> {
        vec![
            point(0, 0),
            point(2, 0),
            point(2, 2),
            point(0, 2),
            point(0, 0),
            point(2, 0),
        ]
    }

    fn periodic_knots() -> Vec<Real> {
        (-2..=6).map(Real::from).collect()
    }

    fn decided<T>(classification: Classification<T>) -> T {
        match classification {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("unexpected uncertainty: {reason:?}"),
        }
    }

    #[test]
    fn retained_periodicity_is_single_source_of_truth_for_profiles_and_knot_insertion() {
        let policy = CurvePolicy::certified();
        let periodicity = SplinePeriodicity2::Periodic {
            period: Real::from(4),
        };
        let polynomial = decided(
            PolynomialBSplineCurve2::try_new_with_periodicity(
                2,
                periodic_controls(),
                periodic_knots(),
                periodicity.clone(),
                &policy,
            )
            .unwrap(),
        );
        let polynomial_profile = decided(
            polynomial
                .retained_curve_profile_with_source_version(7, 3, &policy)
                .unwrap(),
        );
        assert_eq!(polynomial.periodicity(), &periodicity);
        assert_eq!(polynomial_profile.periodicity(), &periodicity);

        let rational = decided(
            RationalBSplineCurve2::try_new_with_periodicity(
                2,
                periodic_controls(),
                vec![Real::one(); 6],
                periodic_knots(),
                periodicity.clone(),
                &policy,
            )
            .unwrap(),
        );
        let (inserted, inserted_count) = decided(
            rational
                .insert_knots(vec![(Real::one() / Real::from(2)).unwrap()], &policy)
                .unwrap(),
        );
        assert_eq!(inserted_count, 1);
        assert_eq!(inserted.periodicity(), &periodicity);
        let inserted_profile = decided(
            inserted
                .retained_curve_profile_with_source_version(8, 4, &policy)
                .unwrap(),
        );
        assert_eq!(inserted_profile.periodicity(), &periodicity);
    }

    #[test]
    fn retained_periodicity_rejects_a_period_different_from_the_active_domain() {
        let result = PolynomialBSplineCurve2::try_new_with_periodicity(
            2,
            periodic_controls(),
            periodic_knots(),
            SplinePeriodicity2::Periodic {
                period: Real::from(5),
            },
            &CurvePolicy::certified(),
        );
        assert_eq!(result.unwrap_err(), CurveError::InvalidPeriodicSpline);
    }
}
