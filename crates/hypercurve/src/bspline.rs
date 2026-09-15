//! Exact polynomial and rational B-spline span extraction.
//!
//! This module is the first retained B-spline carrier in `hypercurve`.  It
//! keeps the authored control net, weights, and knot vector as exact [`Real`]
//! data, then extracts Bezier spans by exact Boehm knot insertion. This follows
//! the exact-geometric-computation rule: preserve the source object and change
//! representation only through replayable exact construction evidence.

use std::cmp::Ordering;
use std::sync::OnceLock;

use crate::HomogeneousControl2;
use crate::rational_bezier_general::project_homogeneous;

use hyperreal::Real;

use crate::classify::{compare_reals, is_zero};
use crate::{
    Aabb2, Axis2, BezierSubcurve2, Classification, CubicBezier2, CurveContext, CurveError,
    CurveResult, Point2, QuadraticBezier2, RationalBezier2, RationalQuadraticBezier2,
    SplinePeriodicity2, UncertaintyReason,
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

/// Exact Bezier extraction evidence for one polynomial B-spline.
///
/// The evidence keeps both the refined knot/control data and the emitted Bezier
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

/// Exact rational B-spline/NURBS curve in the plane.
///
/// One homogeneous control net represents every positive degree. Knot insertion,
/// removal, and Bezier extraction preserve these coefficients directly, including
/// zero or mixed control weights. An affine control view is available only when
/// every control can be projected to a finite point.
#[derive(Clone, Debug)]
pub struct RationalBSplineCurve2 {
    degree: usize,
    homogeneous_controls: Vec<HomogeneousControl2>,
    affine_control_points: OnceLock<Vec<Point2>>,
    weights: OnceLock<Vec<Real>>,
    knots: Vec<Real>,
    periodicity: SplinePeriodicity2,
}

impl PartialEq for RationalBSplineCurve2 {
    fn eq(&self, other: &Self) -> bool {
        self.degree == other.degree
            && self.homogeneous_controls == other.homogeneous_controls
            && self.knots == other.knots
            && self.periodicity == other.periodicity
    }
}

/// Exact rational Bezier extraction evidence for a retained NURBS curve.
///
/// The refined homogeneous control net and knot vector record exact insertion.
/// Each extracted span retains a shared rational Bezier evaluator and its source
/// knot interval; specialization to a conic or polynomial is optional.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBSplineBezierExtraction2 {
    degree: usize,
    refined_homogeneous_controls: Vec<HomogeneousControl2>,
    refined_knots: Vec<Real>,
    spans: Vec<RationalBezierSpan2>,
    inserted_knot_count: usize,
}

/// Certified or retained monotonicity evidence for one extracted spline span.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetainedSpanAxisMonotonicity {
    /// The span is certified monotone along this axis.
    CertifiedMonotone,
    /// Exact topology found interior extrema, so the span is not monotone.
    HasInteriorExtrema,
}

/// Span-local facts produced from B-spline/NURBS Bezier extraction.
///
/// Bounds and axis monotonicity are certified from the actual extracted curve.
/// Control-weight signs alone cannot certify the denominator domain. These
/// facts are produced only by extraction analysis, with uncertainty propagated
/// through the supplied context.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedBSplineSpanFacts2 {
    span_index: usize,
    knot_start: Real,
    knot_end: Real,
    bounds: Aabb2,
    x_monotonicity: RetainedSpanAxisMonotonicity,
    y_monotonicity: RetainedSpanAxisMonotonicity,
}

/// Span-local fact evidence for one B-spline/NURBS extraction.
#[derive(Clone, Debug, PartialEq)]
pub struct RetainedBSplineSpanFactEvidence2 {
    span_facts: Vec<RetainedBSplineSpanFacts2>,
}

/// One exact rational Bezier span extracted from a retained NURBS curve.
///
/// The evaluator owns homogeneous controls and reusable exact decisions. Its
/// local unit parameter maps to the retained source knot interval.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierSpan2 {
    curve: RationalBezier2,
    knot_start: Real,
    knot_end: Real,
}

impl PolynomialBSplineCurve2 {
    /// Constructs a polynomial B-spline of any positive degree.
    ///
    /// The knot vector must be nondecreasing, have length
    /// `control_points.len() + degree + 1`, and define a positive active domain.
    /// Clamped and unclamped knot vectors use exact comparisons through `policy`.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        validate_bspline_layout(degree, control_points.len(), knots.len())?;
        match validate_nondecreasing_knots(&knots, policy)? {
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<PolynomialBSplineBezierExtraction2>> {
        let mut refined = BSplineWorkingCurve {
            degree: self.degree,
            control_points: self.control_points.clone(),
            knots: self.knots.clone(),
            inserted_knot_count: 0,
        };
        match validate_nondecreasing_knots(&refined.knots, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let break_knots = match distinct_bezier_break_knots(&refined.knots, self.degree, policy)? {
            Classification::Decided(knots) => knots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for knot in break_knots {
            loop {
                let multiplicity = match knot_multiplicity(&refined.knots, &knot, policy) {
                    Classification::Decided(multiplicity) => multiplicity,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
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
    pub fn span_fact_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RetainedBSplineSpanFactEvidence2>> {
        native_span_fact_evidence(&self.spans, &self.refined_knots, self.degree, policy)
    }
}

impl RationalBSplineCurve2 {
    /// Constructs a rational B-spline/NURBS curve of degree one or higher.
    ///
    /// The control and weight arrays must have equal length, every authored
    /// weight must be certified nonzero, and the knot vector must be
    /// nondecreasing with `control_points.len() + degree + 1` entries. For
    /// controls at infinity, use [`Self::from_homogeneous_controls`].
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        validate_bspline_layout(degree, control_points.len(), knots.len())?;
        if control_points.len() != weights.len() {
            return Err(CurveError::InvalidBSpline);
        }
        for weight in &weights {
            match is_zero(weight, policy) {
                Some(false) => {}
                Some(true) => return Err(CurveError::ZeroRationalBezierWeight),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let controls = control_points
            .iter()
            .zip(&weights)
            .map(|(point, weight)| HomogeneousControl2::from_affine(point, weight.clone()))
            .collect();
        Ok(
            Self::from_homogeneous_with_periodicity(degree, controls, knots, periodicity, policy)?
                .map(|curve| {
                    let _ = curve.affine_control_points.set(control_points);
                    let _ = curve.weights.set(weights);
                    curve
                }),
        )
    }

    /// Retains exact homogeneous controls and the authored knot vector.
    /// Zero control weights do not imply a pole of the spline denominator.
    pub fn from_homogeneous_controls(
        degree: usize,
        controls: Vec<HomogeneousControl2>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        Self::from_homogeneous_with_periodicity(
            degree,
            controls,
            knots,
            SplinePeriodicity2::NonPeriodic,
            policy,
        )
    }

    pub(crate) fn from_homogeneous_with_periodicity(
        degree: usize,
        controls: Vec<HomogeneousControl2>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        validate_bspline_layout(degree, controls.len(), knots.len())?;
        match validate_nondecreasing_knots(&knots, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        if !has_positive_span(&knots, degree, controls.len(), policy)? {
            return Err(CurveError::InvalidBSpline);
        }
        match validate_spline_periodicity(&knots, degree, controls.len(), &periodicity, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        Ok(Classification::Decided(Self {
            degree,
            homogeneous_controls: controls,
            affine_control_points: OnceLock::new(),
            weights: OnceLock::new(),
            knots,
            periodicity,
        }))
    }

    /// Returns the retained polynomial degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the authoritative homogeneous Bernstein/de Boor controls.
    pub fn homogeneous_controls(&self) -> &[HomogeneousControl2] {
        &self.homogeneous_controls
    }

    pub(crate) fn project_endpoint_controls(
        &self,
        policy: &CurveContext,
    ) -> Classification<[Point2; 2]> {
        if let Some(points) = self.affine_control_points.get() {
            return Classification::Decided([
                points[0].clone(),
                points.last().expect("validated controls").clone(),
            ]);
        }
        let start = match project_homogeneous(&self.homogeneous_controls[0], policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        project_homogeneous(
            self.homogeneous_controls
                .last()
                .expect("validated controls"),
            policy,
        )
        .map(|end| [start, end])
    }

    /// Returns a finite affine authoring view when it can be certified.
    pub fn affine_control_points(&self) -> Option<&[Point2]> {
        if let Some(points) = self.affine_control_points.get() {
            return Some(points);
        }
        let mut points = Vec::with_capacity(self.homogeneous_controls.len());
        for control in &self.homogeneous_controls {
            match project_homogeneous(control, &CurveContext::STRICT) {
                Classification::Decided(point) => points.push(point),
                Classification::Uncertain(_) => return None,
            }
        }
        let _ = self.affine_control_points.set(points);
        self.affine_control_points.get().map(Vec::as_slice)
    }

    /// Returns the retained homogeneous control weights.
    pub fn weights(&self) -> &[Real] {
        self.weights.get_or_init(|| {
            self.homogeneous_controls
                .iter()
                .map(|control| control.weight.clone())
                .collect()
        })
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<(Self, usize)>> {
        if knots.is_empty() {
            return Ok(Classification::Decided((self.clone(), 0)));
        }
        let mut refined = HomogeneousBSplineWorkingCurve {
            degree: self.degree,
            controls: self.homogeneous_controls.clone(),
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
        match Self::from_homogeneous_with_periodicity(
            self.degree,
            refined.controls,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let knot_index = match exact_knot_index(&self.knots, &knot, policy) {
            Classification::Decided(Some(index)) => index,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let mut coarse_knots = self.knots.clone();
        coarse_knots.remove(knot_index);
        let coarse_control_count = self.homogeneous_controls.len() - 1;
        let span = match find_insertion_span(
            &coarse_knots,
            self.degree,
            coarse_control_count,
            &knot,
            policy,
        ) {
            Classification::Decided(Some(span)) => span,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let multiplicity = match knot_multiplicity(&coarse_knots, &knot, policy) {
            Classification::Decided(multiplicity) => multiplicity,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if multiplicity >= self.degree {
            return Ok(Classification::Decided(None));
        }

        let fine_controls = &self.homogeneous_controls;
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

        let candidate = match Self::from_homogeneous_with_periodicity(
            self.degree,
            coarse_controls,
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
    /// the degree. The refined coefficients remain homogeneous; only the two
    /// endpoints of each emitted span must project to finite points.
    pub fn extract_bezier_spans(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBSplineBezierExtraction2>> {
        let mut refined = HomogeneousBSplineWorkingCurve {
            degree: self.degree,
            controls: self.homogeneous_controls.clone(),
            knots: self.knots.clone(),
            inserted_knot_count: 0,
        };
        match validate_nondecreasing_knots(&refined.knots, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        let break_knots = match distinct_bezier_break_knots(&refined.knots, self.degree, policy)? {
            Classification::Decided(knots) => knots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        for knot in break_knots {
            loop {
                let multiplicity = match knot_multiplicity(&refined.knots, &knot, policy) {
                    Classification::Decided(multiplicity) => multiplicity,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
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
}

impl RationalBSplineBezierExtraction2 {
    /// Returns the retained source degree.
    pub const fn degree(&self) -> usize {
        self.degree
    }

    /// Returns the exact refined homogeneous control net.
    pub fn refined_homogeneous_controls(&self) -> &[HomogeneousControl2] {
        &self.refined_homogeneous_controls
    }

    /// Returns the exact refined knot vector.
    pub fn refined_knots(&self) -> &[Real] {
        &self.refined_knots
    }

    /// Returns exact rational Bezier spans in parameter order.
    pub fn spans(&self) -> &[RationalBezierSpan2] {
        &self.spans
    }

    /// Returns exact native spans, sharing the general rational evaluator.
    pub fn native_subcurves(&self, policy: &CurveContext) -> Vec<BezierSubcurve2> {
        self.spans
            .iter()
            .map(|span| span.native_subcurve(policy))
            .collect()
    }

    /// Returns the number of knots inserted during extraction.
    pub const fn inserted_knot_count(&self) -> usize {
        self.inserted_knot_count
    }

    /// Certifies bounds and monotonicity on every finite extracted span.
    pub fn span_fact_evidence(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RetainedBSplineSpanFactEvidence2>> {
        let mut facts = Vec::with_capacity(self.spans.len());
        for (span_index, span) in self.spans.iter().enumerate() {
            let native = span.native_subcurve(policy);
            let bounds = match subcurve_certified_bounds(&native, policy) {
                Classification::Decided(bounds) => bounds,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let monotone = |axis| subcurve_axis_monotonicity(&native, axis, policy);
            let x = match monotone(Axis2::X) {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let y = match monotone(Axis2::Y) {
                Classification::Decided(value) => value,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            let fact = match RetainedBSplineSpanFacts2::new(
                span_index,
                span.knot_start.clone(),
                span.knot_end.clone(),
                bounds,
                x,
                y,
                policy,
            )? {
                Classification::Decided(fact) => fact,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            facts.push(fact);
        }
        RetainedBSplineSpanFactEvidence2::new(facts, policy)
    }
}

impl RetainedBSplineSpanFacts2 {
    /// Constructs one span-local facts record.
    #[allow(clippy::too_many_arguments)]
    fn new(
        span_index: usize,
        knot_start: Real,
        knot_end: Real,
        bounds: Aabb2,
        x_monotonicity: RetainedSpanAxisMonotonicity,
        y_monotonicity: RetainedSpanAxisMonotonicity,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match validate_span_fact_evidence(&knot_start, &knot_end, &bounds, policy)? {
            Classification::Decided(()) => {}
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        }
        Ok(Classification::Decided(Self {
            span_index,
            knot_start,
            knot_end,
            bounds,
            x_monotonicity,
            y_monotonicity,
        }))
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
}

impl RetainedBSplineSpanFactEvidence2 {
    /// Constructs a span-local fact evidence.
    fn new(
        span_facts: Vec<RetainedBSplineSpanFacts2>,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match validate_span_fact_evidence_evidence(&span_facts, policy)? {
            Classification::Decided(()) => Ok(Classification::Decided(Self { span_facts })),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Returns facts in extraction order.
    pub fn span_facts(&self) -> &[RetainedBSplineSpanFacts2] {
        &self.span_facts
    }
}

fn validate_span_fact_evidence(
    knot_start: &Real,
    knot_end: &Real,
    bounds: &Aabb2,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match validate_positive_knot_interval(knot_start, knot_end, policy)? {
        Classification::Decided(()) => {}
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    }
    match bounds.has_valid_ordering(policy) {
        Classification::Decided(true) => Ok(Classification::Decided(())),
        Classification::Decided(false) => Err(CurveError::Topology(
            "spline span bounds must be ordered".into(),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn validate_span_fact_evidence_evidence(
    span_facts: &[RetainedBSplineSpanFacts2],
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    if span_facts.is_empty() {
        return Err(CurveError::Topology(
            "retained span fact evidence must carry at least one span".into(),
        ));
    }
    for (expected_index, fact) in span_facts.iter().enumerate() {
        if fact.span_index() != expected_index {
            return Err(CurveError::Topology(
                "retained span fact evidence indices must be contiguous".into(),
            ));
        }
        if let Some(previous) = expected_index
            .checked_sub(1)
            .and_then(|index| span_facts.get(index))
        {
            match validate_adjacent_knot_windows(
                previous.knot_interval().1,
                fact.knot_interval().0,
                policy,
                "retained span fact evidence knot intervals must be contiguous",
            )? {
                Classification::Decided(()) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
    }
    Ok(Classification::Decided(()))
}

fn validate_positive_knot_interval(
    knot_start: &Real,
    knot_end: &Real,
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    match compare_reals(knot_start, knot_end, policy) {
        Some(Ordering::Less) => Ok(Classification::Decided(())),
        Some(Ordering::Equal | Ordering::Greater) => Err(CurveError::Topology(
            "retained B-spline span evidence must carry certified positive knot interval".into(),
        )),
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

fn validate_adjacent_knot_windows(
    previous_end: &Real,
    next_start: &Real,
    policy: &CurveContext,
    message: &str,
) -> CurveResult<Classification<()>> {
    match compare_reals(previous_end, next_start, policy) {
        Some(Ordering::Equal) => Ok(Classification::Decided(())),
        Some(Ordering::Less | Ordering::Greater) => Err(CurveError::Topology(message.into())),
        None => Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
    }
}

impl RationalBezierSpan2 {
    /// Returns the exact homogeneous Bezier evaluator retained by this span.
    pub const fn curve(&self) -> &RationalBezier2 {
        &self.curve
    }

    /// Returns the source knot interval covered by this Bezier span.
    pub fn knot_interval(&self) -> (&Real, &Real) {
        (&self.knot_start, &self.knot_end)
    }

    /// Selects an exact polynomial/conic specialization when available.
    /// Every remaining span retains its original rational carrier and degree.
    pub fn native_subcurve(&self, policy: &CurveContext) -> BezierSubcurve2 {
        let strict = policy.strict_counterpart();
        if let Some(points) = self.curve.affine_control_points() {
            let weights = self.curve.weights();
            if self.curve.degree() == 2 {
                if let Ok(curve) = RationalQuadraticBezier2::try_new(
                    points[0].clone(),
                    points[1].clone(),
                    points[2].clone(),
                    weights[0].clone(),
                    weights[1].clone(),
                    weights[2].clone(),
                ) {
                    return BezierSubcurve2::RationalQuadratic(curve);
                }
            } else if self.curve.degree() == 3
                && weights_are_all_equal(weights, &strict) == Classification::Decided(true)
            {
                return BezierSubcurve2::Cubic(CubicBezier2::new(
                    points[0].clone(),
                    points[1].clone(),
                    points[2].clone(),
                    points[3].clone(),
                ));
            }
        }
        BezierSubcurve2::Rational(self.curve.clone())
    }
}

#[derive(Clone, Debug)]
struct BSplineWorkingCurve {
    degree: usize,
    control_points: Vec<Point2>,
    knots: Vec<Real>,
    inserted_knot_count: usize,
}

#[derive(Clone, Debug)]
struct HomogeneousBSplineWorkingCurve {
    degree: usize,
    controls: Vec<HomogeneousControl2>,
    knots: Vec<Real>,
    inserted_knot_count: usize,
}

impl HomogeneousControl2 {
    fn inverse_lerp(&self, blended: &Self, t: &Real) -> CurveResult<Self> {
        let one_minus_t = Real::one() - t;
        Ok(Self {
            x: ((blended.x.clone() - &self.x * &one_minus_t) / t.clone())?,
            y: ((blended.y.clone() - &self.y * &one_minus_t) / t.clone())?,
            weight: ((blended.weight.clone() - &self.weight * &one_minus_t) / t.clone())?,
        })
    }

    fn exact_eq(&self, other: &Self, policy: &CurveContext) -> Classification<bool> {
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
}

impl BSplineWorkingCurve {
    fn insert_knot(
        &mut self,
        knot: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let span = match find_insertion_span(
            &self.knots,
            self.degree,
            self.control_points.len(),
            &knot,
            policy,
        ) {
            Classification::Decided(Some(span)) => span,
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let multiplicity = match knot_multiplicity(&self.knots, &knot, policy) {
            Classification::Decided(multiplicity) => multiplicity,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if multiplicity >= self.degree {
            return Ok(Classification::Decided(()));
        }

        let p = self.degree;
        let affected_start = span - p + 1;
        let affected_end = span - multiplicity;
        self.control_points
            .insert(affected_end + 1, self.control_points[affected_end].clone());
        for i in (affected_start..=affected_end).rev() {
            let denominator = &self.knots[i + p] - &self.knots[i];
            let alpha = match (knot.clone() - &self.knots[i]) / denominator {
                Ok(alpha) => alpha,
                Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            };
            self.control_points[i] =
                self.control_points[i - 1].lerp(&self.control_points[i], alpha);
        }

        self.knots.insert(span + 1, knot);
        self.inserted_knot_count += 1;
        Ok(Classification::Decided(()))
    }
}

impl HomogeneousBSplineWorkingCurve {
    fn insert_knot(
        &mut self,
        knot: Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<()>> {
        let span =
            match find_insertion_span(&self.knots, self.degree, self.controls.len(), &knot, policy)
            {
                Classification::Decided(Some(span)) => span,
                Classification::Decided(None) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        let multiplicity = match knot_multiplicity(&self.knots, &knot, policy) {
            Classification::Decided(multiplicity) => multiplicity,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if multiplicity >= self.degree {
            return Ok(Classification::Decided(()));
        }

        let p = self.degree;
        let affected_start = span - p + 1;
        let affected_end = span - multiplicity;
        self.controls
            .insert(affected_end + 1, self.controls[affected_end].clone());
        for i in (affected_start..=affected_end).rev() {
            let denominator = &self.knots[i + p] - &self.knots[i];
            let alpha = match (knot.clone() - &self.knots[i]) / denominator {
                Ok(alpha) => alpha,
                Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            };
            self.controls[i] = self.controls[i - 1].lerp(&self.controls[i], &alpha);
        }

        self.knots.insert(span + 1, knot);
        self.inserted_knot_count += 1;
        Ok(Classification::Decided(()))
    }
}

#[derive(Clone, Copy)]
pub(crate) struct SelectedSpan {
    pub(crate) index: usize,
    pub(crate) location: SpanParameterLocation,
}

#[derive(Clone, Copy)]
pub(crate) enum SpanParameterLocation {
    Start,
    Interior,
    End,
}

/// Selects both incident spans from a certified, contiguous Bezier decomposition.
/// Extraction retains positive intervals in knot order. Their ends therefore
/// increase strictly, and each interior knot belongs to exactly two spans.
/// Reusing those facts avoids reclassifying every unrelated knot at evaluation.
pub(crate) fn select_span_indices<T>(
    spans: &[T],
    interval: impl Fn(&T) -> (&Real, &Real),
    parameter: &Real,
    family: crate::CurveFamily2,
    policy: &CurveContext,
) -> crate::ExactCurveResult<(SelectedSpan, SelectedSpan)> {
    let invalid = || {
        crate::ExactCurveError::invalid(
            crate::CurveOperation2::Evaluation,
            family,
            CurveError::InvalidCurveParameter,
        )
    };
    let compare = |left: &Real, right: &Real| {
        compare_reals(left, right, policy).ok_or_else(|| {
            crate::ExactCurveError::blocked(
                crate::CurveOperation2::Evaluation,
                family,
                UncertaintyReason::Ordering,
            )
        })
    };
    let (mut lower, mut upper) = (0, spans.len());
    let mut end_order = Ordering::Greater;
    while lower < upper {
        let middle = lower + (upper - lower) / 2;
        let order = compare(parameter, interval(&spans[middle]).1)?;
        if order == Ordering::Greater {
            lower = middle + 1;
        } else {
            upper = middle;
            end_order = order;
        }
    }
    if lower == spans.len() {
        return Err(invalid());
    }
    let location = if end_order == Ordering::Equal {
        SpanParameterLocation::End
    } else if lower == 0 {
        match compare(interval(&spans[0]).0, parameter)? {
            Ordering::Less => SpanParameterLocation::Interior,
            Ordering::Equal => SpanParameterLocation::Start,
            Ordering::Greater => return Err(invalid()),
        }
    } else {
        // The preceding end is below the parameter by the search invariant;
        // it is also this span's start by certified knot contiguity.
        SpanParameterLocation::Interior
    };
    let first = SelectedSpan {
        index: lower,
        location,
    };
    let last = if end_order == Ordering::Equal && lower + 1 < spans.len() {
        SelectedSpan {
            index: lower + 1,
            location: SpanParameterLocation::Start,
        }
    } else {
        first
    };
    Ok((first, last))
}

fn validate_bspline_layout(
    degree: usize,
    control_count: usize,
    knot_count: usize,
) -> CurveResult<()> {
    let valid = degree.checked_add(1).is_some_and(|order| {
        degree > 0 && control_count >= order && control_count.checked_add(order) == Some(knot_count)
    });
    if valid {
        Ok(())
    } else {
        Err(CurveError::InvalidBSpline)
    }
}

fn validate_nondecreasing_knots(
    knots: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<()>> {
    for pair in knots.windows(2) {
        match compare_reals(&pair[0], &pair[1], policy) {
            Some(Ordering::Less | Ordering::Equal) => {}
            Some(Ordering::Greater) => return Err(CurveError::InvalidBSpline),
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }
    }
    Ok(Classification::Decided(()))
}

fn has_positive_span(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    policy: &CurveContext,
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
    policy: &CurveContext,
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

fn native_span_fact_evidence(
    spans: &[BezierSubcurve2],
    refined_knots: &[Real],
    degree: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<RetainedBSplineSpanFactEvidence2>> {
    let mut facts = Vec::with_capacity(spans.len());
    let mut span_index = 0_usize;
    let refined_control_count = refined_knots.len().saturating_sub(degree + 1);
    for knot_index in degree..refined_control_count {
        match compare_reals(
            &refined_knots[knot_index],
            &refined_knots[knot_index + 1],
            policy,
        ) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal) => continue,
            Some(Ordering::Greater) => return Err(CurveError::InvalidBSpline),
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
        }
        let Some(span) = spans.get(span_index) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let bounds = match subcurve_certified_bounds(span, policy) {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let fact = match RetainedBSplineSpanFacts2::new(
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
            policy,
        )? {
            Classification::Decided(fact) => fact,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        facts.push(fact);
        span_index += 1;
    }
    if span_index != spans.len() {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    RetainedBSplineSpanFactEvidence2::new(facts, policy)
}

fn subcurve_certified_bounds(
    curve: &BezierSubcurve2,
    policy: &CurveContext,
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
    policy: &CurveContext,
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

fn distinct_bezier_break_knots(
    knots: &[Real],
    degree: usize,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<Real>>> {
    let mut result = Vec::new();
    for knot in &knots[degree..=knots.len() - degree - 1] {
        if let Some(last) = result.last() {
            match compare_reals(last, knot, policy) {
                Some(Ordering::Equal) => continue,
                Some(Ordering::Less) => {}
                Some(Ordering::Greater) => return Err(CurveError::InvalidBSpline),
                None => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
                }
            }
        }
        result.push(knot.clone());
    }
    Ok(Classification::Decided(result))
}

pub(crate) fn knot_multiplicity(
    knots: &[Real],
    knot: &Real,
    policy: &CurveContext,
) -> Classification<usize> {
    let lower = match knot_partition_point(knots, knot, false, policy) {
        Classification::Decided(lower) => lower,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let upper = match knot_partition_point(knots, knot, true, policy) {
        Classification::Decided(upper) => upper,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    Classification::Decided(upper - lower)
}

fn weights_are_all_equal(weights: &[Real], policy: &CurveContext) -> Classification<bool> {
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
    policy: &CurveContext,
) -> Classification<Option<usize>> {
    let n = control_count - 1;
    match compare_reals(knot, &knots[n + 1], policy) {
        Some(Ordering::Equal) => {
            return Classification::Decided(Some(if n + 1 < knots.len() - 1 { n + 1 } else { n }));
        }
        Some(Ordering::Less | Ordering::Greater) => {}
        None => return Classification::Uncertain(UncertaintyReason::Ordering),
    }
    let insertion = match knot_partition_point(knots, knot, true, policy) {
        Classification::Decided(insertion) => insertion,
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let Some(span) = insertion.checked_sub(1) else {
        return Classification::Decided(None);
    };
    Classification::Decided((degree..=n).contains(&span).then_some(span))
}

fn knot_partition_point(
    knots: &[Real],
    knot: &Real,
    include_equal: bool,
    policy: &CurveContext,
) -> Classification<usize> {
    let mut left = 0;
    let mut right = knots.len();
    while left < right {
        let middle = left + (right - left) / 2;
        match compare_reals(&knots[middle], knot, policy) {
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
            Some(ordering) => match ordering {
                Ordering::Less => left = middle + 1,
                Ordering::Equal if include_equal => left = middle + 1,
                Ordering::Equal | Ordering::Greater => right = middle,
            },
        }
    }
    Classification::Decided(left)
}

fn extract_refined_bezier_spans(
    refined: &BSplineWorkingCurve,
    policy: &CurveContext,
) -> CurveResult<Classification<Vec<BezierSubcurve2>>> {
    let mut spans = Vec::new();
    let linear_half = if refined.degree == 1 {
        Some((Real::one() / Real::from(2_i8))?)
    } else {
        None
    };
    for knot_index in refined.degree..refined.control_points.len() {
        match compare_reals(
            &refined.knots[knot_index],
            &refined.knots[knot_index + 1],
            policy,
        ) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal) => continue,
            Some(Ordering::Greater) => return Err(CurveError::InvalidBSpline),
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
        }
        let start = knot_index - refined.degree;
        let controls = &refined.control_points[start..=knot_index];
        let span = match refined.degree {
            1 => {
                // Degree elevation preserves the affine source parameter.
                // Keep that construction proof so incidence and corner work
                // can use the line carrier without reconstructing its image.
                // A collapsed span remains a valid constant Bezier.
                let curve = match crate::LineSeg2::try_new(controls[0].clone(), controls[1].clone())
                {
                    Ok(line) => QuadraticBezier2::from_line_segment(line),
                    Err(CurveError::ZeroLengthLine) => QuadraticBezier2::new(
                        controls[0].clone(),
                        controls[0].lerp(
                            &controls[1],
                            linear_half
                                .as_ref()
                                .expect("linear elevation parameter")
                                .clone(),
                        ),
                        controls[1].clone(),
                    ),
                    Err(cause) => return Err(cause),
                };
                BezierSubcurve2::Quadratic(curve)
            }
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

fn extract_refined_rational_spans(
    refined: &HomogeneousBSplineWorkingCurve,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBSplineBezierExtraction2>> {
    let mut spans = Vec::new();
    for knot_index in refined.degree..refined.controls.len() {
        match compare_reals(
            &refined.knots[knot_index],
            &refined.knots[knot_index + 1],
            policy,
        ) {
            Some(Ordering::Less) => {}
            Some(Ordering::Equal) => continue,
            Some(Ordering::Greater) => return Err(CurveError::InvalidBSpline),
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
        }
        let start = knot_index - refined.degree;
        let curve = match RationalBezier2::from_homogeneous_controls(
            refined.controls[start..=knot_index].to_vec(),
            policy,
        )? {
            Classification::Decided(curve) => curve,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        spans.push(RationalBezierSpan2 {
            curve,
            knot_start: refined.knots[knot_index].clone(),
            knot_end: refined.knots[knot_index + 1].clone(),
        });
    }

    Ok(Classification::Decided(RationalBSplineBezierExtraction2 {
        degree: refined.degree,
        refined_homogeneous_controls: refined.controls.clone(),
        refined_knots: refined.knots.clone(),
        spans,
        inserted_knot_count: refined.inserted_knot_count,
    }))
}

fn exact_knot_index(
    knots: &[Real],
    knot: &Real,
    policy: &CurveContext,
) -> Classification<Option<usize>> {
    for (index, candidate) in knots.iter().enumerate() {
        match compare_reals(candidate, knot, policy) {
            Some(Ordering::Equal) => return Classification::Decided(Some(index)),
            Some(_) => {}
            None => return Classification::Uncertain(UncertaintyReason::Ordering),
        }
    }
    Classification::Decided(None)
}

fn rational_bspline_exact_eq(
    first: &RationalBSplineCurve2,
    second: &RationalBSplineCurve2,
    policy: &CurveContext,
) -> Classification<bool> {
    if first.degree != second.degree
        || first.homogeneous_controls.len() != second.homogeneous_controls.len()
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
    for (first, second) in first
        .homogeneous_controls
        .iter()
        .zip(&second.homogeneous_controls)
    {
        match first.exact_eq(second, policy) {
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
    fn binary_knot_search_matches_complete_scan_on_large_repeated_vectors() {
        let policy = CurveContext::STRICT;
        let mut state = 0x9e37_79b9_u64;
        for case in 0..128_usize {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            let knot_count = 16 + usize::try_from(state % 240).unwrap();
            let degree = 3_usize;
            let control_count = knot_count - degree - 1;
            let n = control_count - 1;
            let mut value = -8_i32;
            let mut knots = Vec::with_capacity(knot_count);
            for _ in 0..knot_count {
                state = state
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1);
                value += i32::try_from(state % 3).unwrap();
                knots.push(Real::from(value));
            }

            for query_value in -10..=value + 2 {
                let query = Real::from(query_value);
                let expected_multiplicity = knots
                    .iter()
                    .filter(|existing| {
                        compare_reals(existing, &query, &policy) == Some(Ordering::Equal)
                    })
                    .count();
                assert_eq!(
                    decided(knot_multiplicity(&knots, &query, &policy)),
                    expected_multiplicity,
                    "case={case}; query={query_value}"
                );

                let expected_span =
                    if compare_reals(&query, &knots[n + 1], &policy) == Some(Ordering::Equal) {
                        Some(if n + 1 < knots.len() - 1 { n + 1 } else { n })
                    } else {
                        knots
                            .iter()
                            .position(|existing| {
                                compare_reals(existing, &query, &policy) == Some(Ordering::Greater)
                            })
                            .unwrap_or(knots.len())
                            .checked_sub(1)
                            .filter(|span| (degree..=n).contains(span))
                    };
                assert_eq!(
                    decided(find_insertion_span(
                        &knots,
                        degree,
                        control_count,
                        &query,
                        &policy,
                    )),
                    expected_span,
                    "case={case}; query={query_value}"
                );
            }
        }
    }

    #[test]
    fn retained_periodicity_survives_knot_insertion() {
        let policy = CurveContext::STRICT;
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
        assert_eq!(polynomial.periodicity(), &periodicity);

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
            &CurveContext::STRICT,
        );
        assert_eq!(result.unwrap_err(), CurveError::InvalidPeriodicSpline);
    }
}
