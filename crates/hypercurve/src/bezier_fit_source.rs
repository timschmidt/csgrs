//! Exact fitting-source reports for polynomial Bezier segments.
//!
//! Higher-order fitting should start from retained curve facts, not from a
//! sampled polyline. This module gathers the exact facts a later fitter needs:
//! structural control-polygon facts, monotone spans, cusp/inflection
//! classification, endpoint derivatives, area/moment integrals, length
//! enclosures, and already-certified primitive image fits. That follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997): approximation stages may propose fits, but certified geometric
//! facts remain the branch boundary. The area/moment and fitting motivation
//! follows Raph Levien, "Simplifying Bezier paths" (2021) and "Fitting cubic
//! Bezier curves" (2021), while the Bezier derivative and control-polygon
//! facts follow Farin, *Curves and Surfaces for Computer-Aided Geometric
//! Design* (5th ed., 2002).

use crate::{
    Aabb2, Bezier2Facts, BezierAreaMoments2, BezierCuspClassification, BezierDegree,
    BezierInflectionClassification, BezierLengthBounds2, BezierLineImageFitRelation,
    BezierMonotoneSpan, BezierPointImageFitRelation, Classification, CubicBezier2, CurveError,
    CurvePolicy, CurveResult, EndpointTangent2, QuadraticBezier2, Real,
};

use std::ops::Range;

/// Exact source facts prepared for a future polynomial Bezier fitting pass.
///
/// This report is intentionally a fact bundle, not a fitted curve. A later
/// bounded fitter can consume it to decide whether a source range has exact
/// primitive structure, monotone subranges, known cusps/inflections, exact
/// moment constraints, and certified metric enclosures before it generates any
/// approximate candidate. Keeping those contracts explicit is the Yap-style
/// separation between exact geometric objects and approximation adapters.
#[derive(Clone, Debug, PartialEq)]
pub struct BezierFitSourceReport2 {
    degree: BezierDegree,
    structural_facts: Bezier2Facts,
    control_hull: Classification<Aabb2>,
    monotone_spans: Classification<Vec<BezierMonotoneSpan>>,
    cusp_classification: Classification<BezierCuspClassification>,
    inflection_classification: Classification<BezierInflectionClassification>,
    length_bounds: BezierLengthBounds2,
    area_moments: BezierAreaMoments2,
    start_tangent: EndpointTangent2,
    end_tangent: EndpointTangent2,
    exact_line_image_fit: Classification<BezierLineImageFitRelation>,
    exact_point_image_fit: Classification<BezierPointImageFitRelation>,
}

/// Aggregate exact facts for a path-range of Bezier fitting sources.
///
/// This is the range-query counterpart to [`BezierFitSourceReport2`]. It
/// keeps cumulative length bounds, Green's-theorem area/moment totals, exact
/// primitive counts, higher-order-fit counts, and uncertainty counts in one
/// replayable object. Future simplifiers and bounded fitters can use this
/// report to reject or prioritize source ranges without re-integrating every
/// segment or silently sampling the path. The design follows Yap's exact
/// computation boundary and the area/moment range-query motivation in Raph
/// Levien, "Simplifying Bezier paths" (2021).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierFitSourceBatchReport2 {
    segment_count: usize,
    exact_primitive_sources: usize,
    higher_order_sources: usize,
    uncertain_sources: usize,
    non_exact_rational_sources: usize,
    undecided_monotone_sources: usize,
    total_length_lower: Real,
    total_length_upper: Real,
    total_signed_area: Real,
    total_x_moment: Real,
    total_y_moment: Real,
}

/// Prefix-sum table for exact Bezier fitting-source range reports.
///
/// This is the range-query data structure requested by the fitting and
/// simplification layer: exact primitive counts, uncertainty counts, length
/// intervals, area, and first moments can be queried by subtracting retained
/// prefix reports instead of re-walking or re-sampling the curve range. It
/// follows Yap's exact-geometric-computation discipline by keeping exact
/// source reports as the cached object facts, and it supports the moment/range
/// constraints discussed by Raph Levien, "Simplifying Bezier paths" (2021).
#[derive(Clone, Debug, PartialEq)]
pub struct BezierFitSourcePrefixSums2 {
    prefixes: Vec<BezierFitSourceBatchReport2>,
}

impl BezierFitSourceReport2 {
    /// Returns the polynomial degree of the source curve.
    pub const fn degree(&self) -> BezierDegree {
        self.degree
    }

    /// Returns cheap exact structural facts over the source control polygon.
    pub const fn structural_facts(&self) -> Bezier2Facts {
        self.structural_facts
    }

    /// Returns the certified control-hull box, or uncertainty from the policy boundary.
    pub const fn control_hull(&self) -> &Classification<Aabb2> {
        &self.control_hull
    }

    /// Returns certified x/y-monotone parameter spans, or explicit uncertainty.
    pub const fn monotone_spans(&self) -> &Classification<Vec<BezierMonotoneSpan>> {
        &self.monotone_spans
    }

    /// Returns the exact cusp classification used by future fitting stages.
    pub const fn cusp_classification(&self) -> &Classification<BezierCuspClassification> {
        &self.cusp_classification
    }

    /// Returns the exact inflection classification used by future fitting stages.
    pub const fn inflection_classification(
        &self,
    ) -> &Classification<BezierInflectionClassification> {
        &self.inflection_classification
    }

    /// Returns certified chord/control-polygon arc-length bounds.
    pub const fn length_bounds(&self) -> &BezierLengthBounds2 {
        &self.length_bounds
    }

    /// Returns exact Green's-theorem area and first-moment contributions.
    pub const fn area_moments(&self) -> &BezierAreaMoments2 {
        &self.area_moments
    }

    /// Returns exact first-derivative facts at `t = 0`.
    pub const fn start_tangent(&self) -> &EndpointTangent2 {
        &self.start_tangent
    }

    /// Returns exact first-derivative facts at `t = 1`.
    pub const fn end_tangent(&self) -> &EndpointTangent2 {
        &self.end_tangent
    }

    /// Returns the exact endpoint-line image fit attempt.
    pub const fn exact_line_image_fit(&self) -> &Classification<BezierLineImageFitRelation> {
        &self.exact_line_image_fit
    }

    /// Returns the exact collapsed-point image fit attempt.
    pub const fn exact_point_image_fit(&self) -> &Classification<BezierPointImageFitRelation> {
        &self.exact_point_image_fit
    }

    /// Returns true when the source has already certified an exact primitive image.
    pub fn has_exact_primitive_image(&self) -> bool {
        matches!(
            self.exact_point_image_fit,
            Classification::Decided(BezierPointImageFitRelation::Fit(_))
        ) || matches!(
            self.exact_line_image_fit,
            Classification::Decided(BezierLineImageFitRelation::Fit(_))
        )
    }

    /// Returns true when the source should continue to a bounded higher-order fitter.
    pub fn needs_higher_order_fit(&self) -> bool {
        matches!(
            self.exact_point_image_fit,
            Classification::Decided(BezierPointImageFitRelation::NotPoint)
        ) && matches!(
            self.exact_line_image_fit,
            Classification::Decided(BezierLineImageFitRelation::NotLine)
        )
    }

    /// Returns true when any carried predicate result is explicitly uncertain.
    pub fn has_uncertainty(&self) -> bool {
        self.control_hull.is_uncertain()
            || self.monotone_spans.is_uncertain()
            || self.cusp_classification.is_uncertain()
            || self.inflection_classification.is_uncertain()
            || self.exact_line_image_fit.is_uncertain()
            || self.exact_point_image_fit.is_uncertain()
    }
}

impl BezierFitSourceBatchReport2 {
    /// Returns an empty batch report.
    pub fn zero() -> Self {
        Self {
            segment_count: 0,
            exact_primitive_sources: 0,
            higher_order_sources: 0,
            uncertain_sources: 0,
            non_exact_rational_sources: 0,
            undecided_monotone_sources: 0,
            total_length_lower: Real::zero(),
            total_length_upper: Real::zero(),
            total_signed_area: Real::zero(),
            total_x_moment: Real::zero(),
            total_y_moment: Real::zero(),
        }
    }

    /// Builds a batch report from already-prepared fitting-source reports.
    pub fn from_reports<'a>(reports: impl IntoIterator<Item = &'a BezierFitSourceReport2>) -> Self {
        let mut batch = Self::zero();

        for report in reports {
            batch.add_report(report);
        }

        batch
    }

    /// Builds a batch report from quadratic Bezier sources.
    pub fn from_quadratics<'a>(
        curves: impl IntoIterator<Item = &'a QuadraticBezier2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(|curve| curve.fit_source_report(policy))
            .collect::<CurveResult<Vec<_>>>()
            .map(|reports| Self::from_reports(&reports))
    }

    /// Builds a batch report from cubic Bezier sources.
    pub fn from_cubics<'a>(
        curves: impl IntoIterator<Item = &'a CubicBezier2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(|curve| curve.fit_source_report(policy))
            .collect::<CurveResult<Vec<_>>>()
            .map(|reports| Self::from_reports(&reports))
    }

    /// Returns the number of source segments summarized.
    pub const fn segment_count(&self) -> usize {
        self.segment_count
    }

    /// Returns how many sources are already exact primitive images.
    pub const fn exact_primitive_sources(&self) -> usize {
        self.exact_primitive_sources
    }

    /// Returns how many sources should continue to bounded higher-order fitting.
    pub const fn higher_order_sources(&self) -> usize {
        self.higher_order_sources
    }

    /// Returns how many sources carried explicit uncertainty.
    pub const fn uncertain_sources(&self) -> usize {
        self.uncertain_sources
    }

    /// Returns true when every summarized source coordinate is exact rational.
    pub const fn all_sources_exact_rational(&self) -> bool {
        self.non_exact_rational_sources == 0
    }

    /// Returns true when every summarized source has decided monotone spans.
    pub const fn all_monotone_spans_decided(&self) -> bool {
        self.undecided_monotone_sources == 0
    }

    /// Returns how many sources did not expose all-exact-rational coordinates.
    pub const fn non_exact_rational_sources(&self) -> usize {
        self.non_exact_rational_sources
    }

    /// Returns how many sources did not expose decided monotone spans.
    pub const fn undecided_monotone_sources(&self) -> usize {
        self.undecided_monotone_sources
    }

    /// Returns the sum of certified arc-length lower bounds.
    pub const fn total_length_lower(&self) -> &Real {
        &self.total_length_lower
    }

    /// Returns the sum of certified arc-length upper bounds.
    pub const fn total_length_upper(&self) -> &Real {
        &self.total_length_upper
    }

    /// Returns the aggregate length interval width.
    pub fn total_length_width(&self) -> Real {
        &self.total_length_upper - &self.total_length_lower
    }

    /// Returns the exact signed-area total across all sources.
    pub const fn total_signed_area(&self) -> &Real {
        &self.total_signed_area
    }

    /// Returns the exact `integral integral x dA` total across all sources.
    pub const fn total_x_moment(&self) -> &Real {
        &self.total_x_moment
    }

    /// Returns the exact `integral integral y dA` total across all sources.
    pub const fn total_y_moment(&self) -> &Real {
        &self.total_y_moment
    }

    fn add_report(&mut self, report: &BezierFitSourceReport2) {
        self.segment_count += 1;
        self.total_length_lower = &self.total_length_lower + report.length_bounds().lower();
        self.total_length_upper = &self.total_length_upper + report.length_bounds().upper();
        self.total_signed_area = &self.total_signed_area + report.area_moments().signed_area();
        self.total_x_moment = &self.total_x_moment + report.area_moments().x_moment();
        self.total_y_moment = &self.total_y_moment + report.area_moments().y_moment();
        if !report.structural_facts().all_exact_rational() {
            self.non_exact_rational_sources += 1;
        }
        if report.monotone_spans().is_uncertain() {
            self.undecided_monotone_sources += 1;
        }
        if report.has_uncertainty() {
            self.uncertain_sources += 1;
        } else if report.has_exact_primitive_image() {
            self.exact_primitive_sources += 1;
        } else if report.needs_higher_order_fit() {
            self.higher_order_sources += 1;
        }
    }

    fn plus_report(&self, report: &BezierFitSourceReport2) -> Self {
        let mut next = self.clone();
        next.add_report(report);
        next
    }

    fn minus(&self, earlier: &Self) -> Self {
        Self {
            segment_count: self.segment_count - earlier.segment_count,
            exact_primitive_sources: self.exact_primitive_sources - earlier.exact_primitive_sources,
            higher_order_sources: self.higher_order_sources - earlier.higher_order_sources,
            uncertain_sources: self.uncertain_sources - earlier.uncertain_sources,
            non_exact_rational_sources: self.non_exact_rational_sources
                - earlier.non_exact_rational_sources,
            undecided_monotone_sources: self.undecided_monotone_sources
                - earlier.undecided_monotone_sources,
            total_length_lower: &self.total_length_lower - &earlier.total_length_lower,
            total_length_upper: &self.total_length_upper - &earlier.total_length_upper,
            total_signed_area: &self.total_signed_area - &earlier.total_signed_area,
            total_x_moment: &self.total_x_moment - &earlier.total_x_moment,
            total_y_moment: &self.total_y_moment - &earlier.total_y_moment,
        }
    }
}

impl BezierFitSourcePrefixSums2 {
    /// Builds prefix sums from already-prepared fitting-source reports.
    pub fn from_reports<'a>(reports: impl IntoIterator<Item = &'a BezierFitSourceReport2>) -> Self {
        let mut prefixes = vec![BezierFitSourceBatchReport2::zero()];
        for report in reports {
            let next = prefixes
                .last()
                .expect("prefix table always contains zero")
                .plus_report(report);
            prefixes.push(next);
        }
        Self { prefixes }
    }

    /// Builds fitting-source prefix sums from quadratic Bezier sources.
    pub fn from_quadratics<'a>(
        curves: impl IntoIterator<Item = &'a QuadraticBezier2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(|curve| curve.fit_source_report(policy))
            .collect::<CurveResult<Vec<_>>>()
            .map(|reports| Self::from_reports(&reports))
    }

    /// Builds fitting-source prefix sums from cubic Bezier sources.
    pub fn from_cubics<'a>(
        curves: impl IntoIterator<Item = &'a CubicBezier2>,
        policy: &CurvePolicy,
    ) -> CurveResult<Self> {
        curves
            .into_iter()
            .map(|curve| curve.fit_source_report(policy))
            .collect::<CurveResult<Vec<_>>>()
            .map(|reports| Self::from_reports(&reports))
    }

    /// Returns the number of source segments represented by the table.
    pub fn segment_count(&self) -> usize {
        self.prefixes.len().saturating_sub(1)
    }

    /// Returns all retained prefix reports, including the initial empty report.
    pub fn prefixes(&self) -> &[BezierFitSourceBatchReport2] {
        &self.prefixes
    }

    /// Returns the aggregate fitting-source report over a half-open segment range.
    pub fn range_report(&self, range: Range<usize>) -> CurveResult<BezierFitSourceBatchReport2> {
        if range.start > range.end || range.end > self.segment_count() {
            return Err(CurveError::InvalidBezierRange);
        }
        Ok(self.prefixes[range.end].minus(&self.prefixes[range.start]))
    }
}

impl QuadraticBezier2 {
    /// Builds an exact fitting-source report for this quadratic Bezier.
    ///
    /// The report collects facts that fitting and simplification algorithms
    /// usually approximate from samples: monotone decomposition, moments,
    /// length bounds, endpoint derivatives, and primitive image checks. It
    /// keeps each predicate result in its certified/uncertain form so later
    /// fitting code can follow Yap's exact-computation boundary.
    pub fn fit_source_report(&self, policy: &CurvePolicy) -> CurveResult<BezierFitSourceReport2> {
        Ok(BezierFitSourceReport2 {
            degree: BezierDegree::Quadratic,
            structural_facts: self.structural_facts(),
            control_hull: self.control_hull_box(policy),
            monotone_spans: self.monotone_spans(policy),
            cusp_classification: self.cusp_classification(policy),
            inflection_classification: Classification::Decided(self.inflection_classification()),
            length_bounds: self.length_bounds()?,
            area_moments: self.area_moments_contribution()?,
            start_tangent: self.endpoint_tangent(crate::BezierEndpoint::Start),
            end_tangent: self.endpoint_tangent(crate::BezierEndpoint::End),
            exact_line_image_fit: self.fit_exact_line_image(policy)?,
            exact_point_image_fit: self.fit_exact_point_image(policy)?,
        })
    }
}

impl CubicBezier2 {
    /// Builds an exact fitting-source report for this cubic Bezier.
    ///
    /// Cubic fitting is where Levien-style moment, arclength, cusp, and
    /// inflection constraints matter most. This report does not choose a cubic
    /// approximant; it preserves the exact facts and uncertainty needed by a
    /// future bounded fitter to validate candidates instead of trusting sample
    /// geometry.
    pub fn fit_source_report(&self, policy: &CurvePolicy) -> CurveResult<BezierFitSourceReport2> {
        Ok(BezierFitSourceReport2 {
            degree: BezierDegree::Cubic,
            structural_facts: self.structural_facts(),
            control_hull: self.control_hull_box(policy),
            monotone_spans: self.monotone_spans(policy),
            cusp_classification: self.cusp_classification(policy),
            inflection_classification: self.inflection_classification(policy),
            length_bounds: self.length_bounds()?,
            area_moments: self.area_moments_contribution()?,
            start_tangent: self.endpoint_tangent(crate::BezierEndpoint::Start),
            end_tangent: self.endpoint_tangent(crate::BezierEndpoint::End),
            exact_line_image_fit: self.fit_exact_line_image(policy)?,
            exact_point_image_fit: self.fit_exact_point_image(policy)?,
        })
    }
}
