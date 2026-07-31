//! Retained polynomial B-spline carrier with policy-isolated exact caches.

use std::cmp::Ordering;
use std::sync::Arc;

use crate::policy::{
    PolicyEvaluationCache, resolve_cached_evaluation, resolve_certified_operation,
};
use crate::spline_periodic::{expand_periodic_spline, wrap_periodic_parameter};
use crate::{
    BezierSubcurve2, Classification, CurveContext, CurveDerivative2, CurveError, CurveFamily2,
    CurveOperation2, CurveOutcome, CurveParameterSide2, ExactCurveError, ExactCurveResult,
    NurbsCurve2, Point2, PolynomialBSplineBezierExtraction2, PolynomialBSplineCurve2,
    RationalBezier2, Real, Similarity2, SplinePeriodicity2, UncertaintyReason,
};

type Cached<T> = Result<T, ExactCurveError>;

#[derive(Debug)]
struct PolynomialSplineData2 {
    retained: PolynomialBSplineCurve2,
    endpoints: SplineEndpoints2,
    decomposition: PolicyEvaluationCache<PolynomialSplineBezierDecomposition2>,
    rational_spans: PolicyEvaluationCache<Vec<RationalBezier2>>,
}

#[derive(Debug)]
enum SplineEndpoints2 {
    AuthoredControls,
    Extracted { start: Point2, end: Point2 },
}

/// Exact polynomial B-spline with retained source identity and decomposition.
///
/// Clones share the authored control net and lazy Boehm decomposition. One
/// decided exact decomposition is retained together with any strict blocker,
/// so approximate-first evaluation cannot certify a later strict request.
#[derive(Clone, Debug)]
pub struct PolynomialSplineCurve2 {
    data: Arc<PolynomialSplineData2>,
}

/// Exact Bezier decomposition retained by a [`PolynomialSplineCurve2`].
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialSplineBezierDecomposition2 {
    extraction: PolynomialBSplineBezierExtraction2,
    intervals: Vec<(Real, Real)>,
}

/// Borrowed polynomial Bezier span with source provenance.
#[derive(Clone, Copy, Debug)]
pub struct PolynomialSplineBezierSpanView2<'a> {
    span_index: usize,
    curve: &'a BezierSubcurve2,
    interval: &'a (Real, Real),
}

impl PolynomialSplineCurve2 {
    /// Constructs a polynomial B-spline of any positive degree.
    ///
    /// The outcome records any terminal decision consumed while validating
    /// knot ordering, endpoints, or a periodic seam.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::try_new_raw(degree, control_points, knots, attempt)
        })
    }

    pub(crate) fn try_new_raw(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded_with_policy(
            degree,
            control_points,
            knots,
            SplinePeriodicity2::NonPeriodic,
            policy,
        )
    }

    /// Constructs a periodic polynomial B-spline from one period of controls and knots.
    ///
    /// The outcome covers periodic expansion, carrier validation, and seam
    /// certification under the selected policy.
    pub fn try_new_periodic(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::try_new_periodic_raw(degree, control_points, period_knots, attempt)
        })
    }

    pub(crate) fn try_new_periodic_raw(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let expansion = expand_periodic_spline(
            degree,
            control_points,
            period_knots,
            CurveFamily2::PolynomialBSpline,
            policy,
        )?;
        Self::try_new_expanded_with_policy(
            degree,
            expansion.control_points,
            expansion.knots,
            SplinePeriodicity2::Periodic {
                period: expansion.period,
            },
            policy,
        )
    }

    fn try_new_expanded(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded_with_policy(
            degree,
            control_points,
            knots,
            periodicity,
            &CurveContext::STRICT,
        )
    }

    fn try_new_expanded_with_policy(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let valid_layout = degree
            .checked_add(1)
            .and_then(|order| {
                control_points
                    .len()
                    .checked_add(order)
                    .map(|knot_count| (order, knot_count))
            })
            .is_some_and(|(order, expected_knots)| {
                degree >= 1 && control_points.len() >= order && knots.len() == expected_knots
            });
        if !valid_layout {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::PolynomialBSpline,
                CurveError::InvalidBSpline,
            ));
        }
        let retained = exact_value(
            PolynomialBSplineCurve2::try_new_with_periodicity(
                degree,
                control_points,
                knots,
                periodicity,
                policy,
            ),
            CurveOperation2::Construction,
        )?;
        let decomposition = PolicyEvaluationCache::new();
        let endpoints = if has_clamped_endpoints(
            retained.knots(),
            retained.degree(),
            retained.control_points().len(),
            policy,
            CurveOperation2::Construction,
        )? {
            SplineEndpoints2::AuthoredControls
        } else {
            let extraction = exact_value(
                retained.extract_bezier_spans(policy),
                CurveOperation2::Construction,
            )?;
            let intervals = require_classification(
                source_intervals(&extraction, policy)?,
                CurveOperation2::Construction,
            )?;
            let start = extraction
                .spans()
                .first()
                .expect("validated spline has a positive span")
                .start()
                .clone();
            let end = extraction
                .spans()
                .last()
                .expect("validated spline has a positive span")
                .end()
                .clone();
            if !policy.permits_approximate_512() {
                decomposition.seed_certified(PolynomialSplineBezierDecomposition2 {
                    extraction,
                    intervals,
                });
            }
            SplineEndpoints2::Extracted { start, end }
        };
        let curve = Self {
            data: Arc::new(PolynomialSplineData2 {
                retained,
                endpoints,
                decomposition,
                rational_spans: PolicyEvaluationCache::new(),
            }),
        };
        curve.validate_periodic_seam(policy)?;
        Ok(curve)
    }

    #[cfg(feature = "svg")]
    pub(crate) fn try_new_expanded_with_periodicity(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded(degree, control_points, knots, periodicity)
    }

    /// Returns the polynomial degree.
    pub fn degree(&self) -> usize {
        self.data.retained.degree()
    }

    /// Returns the exact authored control net.
    pub fn control_points(&self) -> &[Point2] {
        self.data.retained.control_points()
    }

    /// Returns the exact authored knot vector.
    pub fn knots(&self) -> &[Real] {
        self.data.retained.knots()
    }

    /// Returns the exact active source-parameter domain `[U[p], U[n+1]]`.
    pub fn parameter_domain(&self) -> (&Real, &Real) {
        let knots = self.knots();
        (
            &knots[self.degree()],
            &knots[knots.len() - self.degree() - 1],
        )
    }

    /// Returns retained finite or periodic spline semantics.
    pub fn periodicity(&self) -> &SplinePeriodicity2 {
        self.data.retained.periodicity()
    }

    /// Returns the exact period when this spline is periodic.
    pub fn period(&self) -> Option<&Real> {
        self.periodicity().period()
    }

    /// Returns the exact active-domain start point.
    pub fn start(&self) -> &Point2 {
        match &self.data.endpoints {
            SplineEndpoints2::AuthoredControls => &self.data.retained.control_points()[0],
            SplineEndpoints2::Extracted { start, .. } => start,
        }
    }

    /// Returns the exact active-domain end point.
    pub fn end(&self) -> &Point2 {
        match &self.data.endpoints {
            SplineEndpoints2::AuthoredControls => self
                .data
                .retained
                .control_points()
                .last()
                .expect("validated polynomial spline has controls"),
            SplineEndpoints2::Extracted { end, .. } => end,
        }
    }

    /// Inserts one exact knot without changing the polynomial spline image.
    pub fn insert_knot(&self, knot: Real) -> ExactCurveResult<Self> {
        let refined = self
            .as_unit_weight_nurbs(&CurveContext::STRICT)?
            .insert_knot(knot)
            .map_err(|error| {
                remap_spline_family_operation(error, CurveOperation2::KnotInsertion)
            })?;
        if refined.control_points().len() == self.control_points().len() {
            return Ok(self.clone());
        }
        Self::from_unit_weight_nurbs(
            refined,
            CurveOperation2::KnotInsertion,
            &CurveContext::STRICT,
        )
    }

    /// Splits this polynomial spline exactly at a strict interior parameter.
    ///
    /// The returned [`CurveOutcome`] covers the unit-weight NURBS split and
    /// exact polynomial-carrier reconstruction.
    #[inline(always)]
    pub fn split_at(
        &self,
        parameter: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<(Self, Self)>> {
        resolve_certified_operation(policy, |attempt| self.split_at_raw(parameter, attempt))
    }

    pub(crate) fn split_at_raw(
        &self,
        parameter: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<(Self, Self)> {
        let (left, right) = self
            .as_unit_weight_nurbs(policy)?
            .split_at_raw(parameter, policy)
            .map_err(|error| remap_spline_family_operation(error, CurveOperation2::Subdivision))?;
        Ok((
            Self::from_unit_weight_nurbs(left, CurveOperation2::Subdivision, policy)?,
            Self::from_unit_weight_nurbs(right, CurveOperation2::Subdivision, policy)?,
        ))
    }

    /// Returns an exact polynomial subcurve over an ordered source range.
    ///
    /// The returned [`CurveOutcome`] records any `APPROXIMATE_512` terminal
    /// consumed throughout the complete reconstruction.
    #[inline(always)]
    pub fn subcurve(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        let (domain_start, domain_end) = self.parameter_domain();
        if &start == domain_start && &end == domain_end {
            return Ok(CurveOutcome::new(
                self.clone(),
                crate::CurveCertainty::Certified,
            ));
        }
        resolve_certified_operation(policy, |attempt| self.subcurve_raw(start, end, attempt))
    }

    pub(crate) fn subcurve_raw(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let subcurve = self
            .as_unit_weight_nurbs(policy)?
            .subcurve_raw(start, end, policy)
            .map_err(|error| remap_spline_family_operation(error, CurveOperation2::Subdivision))?;
        Self::from_unit_weight_nurbs(subcurve, CurveOperation2::Subdivision, policy)
    }

    /// Returns an exact finite subcurve with clamped endpoints.
    ///
    /// Periodic and unclamped polynomial splines are materialized in exact
    /// piecewise-Bézier B-spline form over the requested source interval.
    /// The returned [`CurveOutcome`] covers the complete exact materialization.
    #[inline(always)]
    pub fn clamped_subcurve(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.clamped_subcurve_raw(start, end, attempt)
        })
    }

    pub(crate) fn clamped_subcurve_raw(
        &self,
        start: Real,
        end: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let subcurve = self
            .as_unit_weight_nurbs(policy)?
            .clamped_subcurve_raw(start, end, policy)
            .map_err(|error| remap_spline_family_operation(error, CurveOperation2::Subdivision))?;
        Self::from_unit_weight_nurbs(subcurve, CurveOperation2::Subdivision, policy)
    }

    /// Returns the same polynomial spline image with traversal direction reversed.
    ///
    /// The control net is reversed and the knot vector is reflected through
    /// the authored domain midpoint, preserving both the domain and source.
    pub fn reversed(&self) -> ExactCurveResult<Self> {
        let (start, end) = self.parameter_domain();
        let knot_sum = start + end;
        let mut control_points = self.control_points().to_vec();
        control_points.reverse();
        let knots = self
            .knots()
            .iter()
            .rev()
            .map(|knot| &knot_sum - knot)
            .collect();
        Self::try_new_expanded(
            self.degree(),
            control_points,
            knots,
            self.periodicity().clone(),
        )
        .map_err(|error| remap_spline_operation(error, CurveOperation2::Reversal))
    }

    /// Applies an exact planar similarity while retaining periodicity and source.
    pub fn transform_similarity(&self, transform: &Similarity2) -> ExactCurveResult<Self> {
        Self::try_new_expanded(
            self.degree(),
            self.control_points()
                .iter()
                .map(|point| transform.transform_point(point))
                .collect(),
            self.knots().to_vec(),
            self.periodicity().clone(),
        )
        .map_err(|error| remap_spline_operation(error, CurveOperation2::Transformation))
    }

    /// Returns the shared exact Bezier decomposition and source intervals.
    pub fn bezier_decomposition(&self) -> ExactCurveResult<&PolynomialSplineBezierDecomposition2> {
        self.bezier_decomposition_for_operation(
            &CurveContext::STRICT,
            CurveOperation2::BezierDecomposition,
        )
    }

    pub(crate) fn bezier_decomposition_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&PolynomialSplineBezierDecomposition2>> {
        resolve_cached_evaluation(&self.data.decomposition, policy, |attempt| {
            let extraction = match map_classified_curve_result(
                self.data.retained.extract_bezier_spans(attempt),
                CurveOperation2::BezierDecomposition,
            )? {
                Classification::Decided(extraction) => extraction,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(source_intervals(&extraction, attempt)?.map(|intervals| {
                PolynomialSplineBezierDecomposition2 {
                    extraction,
                    intervals,
                }
            }))
        })
    }

    fn bezier_decomposition_for_operation(
        &self,
        policy: &CurveContext,
        operation: CurveOperation2,
    ) -> ExactCurveResult<&PolynomialSplineBezierDecomposition2> {
        require_classification(
            self.bezier_decomposition_with_policy(policy)
                .map_err(|error| remap_spline_operation(error, operation))?,
            operation,
        )
    }

    /// Iterates exact Bezier spans with source identity and knot intervals.
    pub fn bezier_spans(
        &self,
    ) -> ExactCurveResult<impl ExactSizeIterator<Item = PolynomialSplineBezierSpanView2<'_>>> {
        let decomposition = self.bezier_decomposition_for_operation(
            &CurveContext::STRICT,
            CurveOperation2::BezierDecomposition,
        )?;
        Ok(decomposition
            .spans()
            .iter()
            .zip(decomposition.intervals())
            .enumerate()
            .map(
                move |(span_index, (curve, interval))| PolynomialSplineBezierSpanView2 {
                    span_index,
                    curve,
                    interval,
                },
            ))
    }

    /// Evaluates the spline at an exact source-domain parameter.
    pub fn point_at(&self, parameter: &Real) -> ExactCurveResult<Point2> {
        self.point_at_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates an exact point with explicit knot-boundary side policy.
    pub fn point_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Point2> {
        self.point_at_side_with_policy(parameter, side, &CurveContext::STRICT)
    }

    pub(crate) fn point_at_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        if self.is_periodic_seam_parameter(parameter, policy)? {
            let (domain_start, domain_end) = self.parameter_domain();
            let left =
                self.point_at_canonical_side(domain_end, CurveParameterSide2::Left, policy)?;
            if side == CurveParameterSide2::Left {
                return Ok(left);
            }
            let right =
                self.point_at_canonical_side(domain_start, CurveParameterSide2::Right, policy)?;
            if side == CurveParameterSide2::Right {
                return Ok(right);
            }
            return matching_spline_point(left, right, policy);
        }
        self.point_at_canonical_side(parameter, side, policy)
    }

    /// Evaluates a periodic spline at any exactly wrappable parameter.
    pub fn point_at_wrapped(&self, parameter: &Real) -> ExactCurveResult<Point2> {
        self.point_at_wrapped_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates a periodic spline with explicit side selection at wrapped seams.
    pub fn point_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Point2> {
        let (start, end) = self.parameter_domain();
        let wrapped = wrap_periodic_parameter(
            parameter,
            start,
            end,
            self.periodicity(),
            side,
            CurveFamily2::PolynomialBSpline,
        )?;
        self.point_at_side(&wrapped, side)
    }

    fn point_at_canonical_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        let decomposition =
            self.bezier_decomposition_for_operation(policy, CurveOperation2::Evaluation)?;
        let (first, last) = select_span_indices(decomposition.intervals(), parameter, policy)?;
        let first_interval = &decomposition.intervals()[first.index];
        let first_point = evaluate_span(
            &decomposition.spans()[first.index],
            &first_interval.0,
            &first_interval.1,
            parameter,
            first.location,
            policy,
        )?;
        if first.index == last.index || side == CurveParameterSide2::Left {
            return Ok(first_point);
        }
        let last_interval = &decomposition.intervals()[last.index];
        let last_point = evaluate_span(
            &decomposition.spans()[last.index],
            &last_interval.0,
            &last_interval.1,
            parameter,
            last.location,
            policy,
        )?;
        if side == CurveParameterSide2::Right {
            return Ok(last_point);
        }
        matching_spline_point(first_point, last_point, policy)
    }

    /// Evaluates the exact first derivative in the authored knot parameter.
    pub fn derivative_at(&self, parameter: &Real) -> ExactCurveResult<CurveDerivative2> {
        self.derivative_at_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates an exact first derivative with explicit knot-boundary side policy.
    pub fn derivative_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<CurveDerivative2> {
        let mut derivatives =
            self.derivatives_at_side_with_policy(parameter, 1, side, &CurveContext::STRICT)?;
        Ok(derivatives.pop().expect("one derivative requested"))
    }

    /// Evaluates the first periodic derivative at any wrappable parameter.
    pub fn derivative_at_wrapped(&self, parameter: &Real) -> ExactCurveResult<CurveDerivative2> {
        self.derivative_at_wrapped_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates the first periodic derivative with explicit seam-side selection.
    pub fn derivative_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<CurveDerivative2> {
        let mut derivatives = self.derivatives_at_wrapped_side(parameter, 1, side)?;
        Ok(derivatives.pop().expect("one derivative requested"))
    }

    /// Evaluates exact derivatives through `max_order` in the authored knot parameter.
    pub fn derivatives_at(
        &self,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.derivatives_at_side(parameter, max_order, CurveParameterSide2::Automatic)
    }

    /// Evaluates exact derivatives with explicit knot-boundary side policy.
    pub fn derivatives_at_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.derivatives_at_side_with_policy(parameter, max_order, side, &CurveContext::STRICT)
    }

    pub(crate) fn derivatives_at_side_with_policy(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        if self.is_periodic_seam_parameter(parameter, policy)? {
            let (domain_start, domain_end) = self.parameter_domain();
            let left = self.derivatives_at_canonical_side(
                domain_end,
                max_order,
                CurveParameterSide2::Left,
                policy,
            )?;
            if side == CurveParameterSide2::Left {
                return Ok(left);
            }
            let right = self.derivatives_at_canonical_side(
                domain_start,
                max_order,
                CurveParameterSide2::Right,
                policy,
            )?;
            if side == CurveParameterSide2::Right {
                return Ok(right);
            }
            return matching_spline_derivatives(left, right, policy);
        }
        self.derivatives_at_canonical_side(parameter, max_order, side, policy)
    }

    /// Evaluates periodic derivatives through `max_order` at any wrappable parameter.
    pub fn derivatives_at_wrapped(
        &self,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        self.derivatives_at_wrapped_side(parameter, max_order, CurveParameterSide2::Automatic)
    }

    /// Evaluates periodic derivatives with explicit side selection at wrapped seams.
    pub fn derivatives_at_wrapped_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let (start, end) = self.parameter_domain();
        let wrapped = wrap_periodic_parameter(
            parameter,
            start,
            end,
            self.periodicity(),
            side,
            CurveFamily2::PolynomialBSpline,
        )?;
        self.derivatives_at_side(&wrapped, max_order, side)
    }

    fn derivatives_at_canonical_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let decomposition =
            self.bezier_decomposition_for_operation(policy, CurveOperation2::Evaluation)?;
        let (first, last) = select_span_indices(decomposition.intervals(), parameter, policy)?;
        let first_derivatives =
            self.derivatives_on_span(first.index, parameter, max_order, first.location, policy)?;
        if first.index == last.index || side == CurveParameterSide2::Left {
            return Ok(first_derivatives);
        }
        let last_derivatives =
            self.derivatives_on_span(last.index, parameter, max_order, last.location, policy)?;
        if side == CurveParameterSide2::Right {
            return Ok(last_derivatives);
        }
        matching_spline_derivatives(first_derivatives, last_derivatives, policy)
    }

    fn derivatives_on_span(
        &self,
        span_index: usize,
        parameter: &Real,
        max_order: usize,
        location: SpanParameterLocation,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let interval = &self
            .bezier_decomposition_for_operation(policy, CurveOperation2::Evaluation)?
            .intervals()[span_index];
        let local = match location {
            SpanParameterLocation::Start => Real::zero(),
            SpanParameterLocation::End => Real::one(),
            SpanParameterLocation::Interior => local_span_parameter(interval, parameter)?,
        };
        let evaluator =
            &self.rational_spans_for_operation(policy, CurveOperation2::Evaluation)?[span_index];
        let local_derivatives = if max_order == 1 {
            vec![exact_classification(
                evaluator.derivative_at_classified(&local, policy),
            )?]
        } else {
            exact_classification(evaluator.derivatives_at_classified(&local, max_order, policy))?
        };
        let inverse_width = (Real::one() / (&interval.1 - &interval.0)).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                CurveFamily2::PolynomialBSpline,
                cause.into(),
            )
        })?;
        let mut scale = Real::one();
        Ok(local_derivatives
            .into_iter()
            .map(|derivative| {
                scale *= &inverse_width;
                derivative.scaled(&scale)
            })
            .collect())
    }

    fn rational_spans_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&[RationalBezier2]>> {
        Ok(
            match resolve_cached_evaluation(&self.data.rational_spans, policy, |attempt| {
                let decomposition = match self.bezier_decomposition_with_policy(attempt)? {
                    Classification::Decided(decomposition) => decomposition,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                decomposition
                    .spans()
                    .iter()
                    .map(rationalize_subcurve)
                    .collect::<ExactCurveResult<Vec<_>>>()
                    .map(Classification::Decided)
            })? {
                Classification::Decided(spans) => Classification::Decided(spans.as_slice()),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    fn rational_spans_for_operation(
        &self,
        policy: &CurveContext,
        operation: CurveOperation2,
    ) -> ExactCurveResult<&[RationalBezier2]> {
        require_classification(
            self.rational_spans_with_policy(policy)
                .map_err(|error| remap_spline_operation(error, operation))?,
            operation,
        )
    }

    fn as_unit_weight_nurbs(&self, policy: &CurveContext) -> ExactCurveResult<NurbsCurve2> {
        let weights = vec![Real::one(); self.control_points().len()];
        let result = NurbsCurve2::try_new_expanded_with_periodicity_and_policy(
            self.degree(),
            self.control_points().to_vec(),
            weights,
            self.knots().to_vec(),
            self.periodicity().clone(),
            policy,
        );
        result
            .map_err(|error| remap_spline_family_operation(error, CurveOperation2::NativeTopology))
    }

    fn from_unit_weight_nurbs(
        curve: NurbsCurve2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let result = Self::try_new_expanded_with_policy(
            curve.degree(),
            curve.control_points().to_vec(),
            curve.knots().to_vec(),
            curve.periodicity().clone(),
            policy,
        );
        result.map_err(|error| remap_spline_operation(error, operation))
    }

    fn validate_periodic_seam(&self, policy: &CurveContext) -> ExactCurveResult<()> {
        if !self.periodicity().is_periodic() {
            return Ok(());
        }
        match (
            crate::classify::compare_reals(self.start().x(), self.end().x(), policy),
            crate::classify::compare_reals(self.start().y(), self.end().y(), policy),
        ) {
            (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(()),
            (Some(_), Some(_)) => Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::PolynomialBSpline,
                CurveError::PeriodicSplineSeamMismatch,
            )),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Construction,
                CurveFamily2::PolynomialBSpline,
                UncertaintyReason::RealSign,
            )),
        }
    }

    fn is_periodic_seam_parameter(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<bool> {
        if !self.periodicity().is_periodic() {
            return Ok(false);
        }
        let (start, end) = self.parameter_domain();
        match (
            crate::classify::compare_reals(parameter, start, policy),
            crate::classify::compare_reals(parameter, end, policy),
        ) {
            (Some(Ordering::Equal), _) | (_, Some(Ordering::Equal)) => Ok(true),
            (Some(_), Some(_)) => Ok(false),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::PolynomialBSpline,
                UncertaintyReason::Ordering,
            )),
        }
    }
}

impl PartialEq for PolynomialSplineCurve2 {
    fn eq(&self, other: &Self) -> bool {
        self.data.retained == other.data.retained
    }
}

impl PolynomialSplineBezierDecomposition2 {
    /// Returns the source spline degree.
    pub const fn degree(&self) -> usize {
        self.extraction.degree()
    }

    /// Returns the exact refined control net after knot insertion.
    pub fn refined_control_points(&self) -> &[Point2] {
        self.extraction.refined_control_points()
    }

    /// Returns the exact refined knot vector after knot insertion.
    pub fn refined_knots(&self) -> &[Real] {
        self.extraction.refined_knots()
    }

    /// Returns exact native Bezier spans in source-parameter order.
    pub fn spans(&self) -> &[BezierSubcurve2] {
        self.extraction.spans()
    }

    /// Returns source-parameter intervals corresponding one-to-one with spans.
    pub fn intervals(&self) -> &[(Real, Real)] {
        &self.intervals
    }

    /// Returns how many exact knot insertions produced Bezier form.
    pub const fn inserted_knot_count(&self) -> usize {
        self.extraction.inserted_knot_count()
    }
}

impl<'a> PolynomialSplineBezierSpanView2<'a> {
    /// Returns this span's stable index in source-parameter order.
    pub const fn span_index(self) -> usize {
        self.span_index
    }

    /// Returns the exact native polynomial Bezier curve.
    pub const fn curve(self) -> &'a BezierSubcurve2 {
        self.curve
    }

    /// Returns the exact source knot interval.
    pub fn knot_interval(self) -> (&'a Real, &'a Real) {
        (&self.interval.0, &self.interval.1)
    }
}

fn source_intervals(
    extraction: &PolynomialBSplineBezierExtraction2,
    policy: &CurveContext,
) -> ExactCurveResult<Classification<Vec<(Real, Real)>>> {
    let degree = extraction.degree();
    let knots = extraction.refined_knots();
    let end = knots.len().saturating_sub(degree + 1);
    let mut intervals = Vec::with_capacity(extraction.spans().len());
    for index in degree..end {
        match crate::classify::compare_reals(&knots[index], &knots[index + 1], policy) {
            Some(Ordering::Less) => {
                intervals.push((knots[index].clone(), knots[index + 1].clone()));
            }
            Some(Ordering::Equal) => {}
            Some(Ordering::Greater) => {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::BezierDecomposition,
                    CurveFamily2::PolynomialBSpline,
                    CurveError::InvalidBSpline,
                ));
            }
            None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
        }
    }
    if intervals.len() != extraction.spans().len() {
        return Err(ExactCurveError::invalid(
            CurveOperation2::BezierDecomposition,
            CurveFamily2::PolynomialBSpline,
            CurveError::Topology("B-spline span/interval count mismatch".into()),
        ));
    }
    Ok(Classification::Decided(intervals))
}

fn has_clamped_endpoints(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    policy: &CurveContext,
    operation: CurveOperation2,
) -> ExactCurveResult<bool> {
    match (
        crate::classify::compare_reals(&knots[0], &knots[degree], policy),
        crate::classify::compare_reals(
            knots.last().expect("validated spline has knots"),
            &knots[control_count],
            policy,
        ),
    ) {
        (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(true),
        (Some(_), Some(_)) => Ok(false),
        _ => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::PolynomialBSpline,
            UncertaintyReason::Ordering,
        )),
    }
}

fn evaluate_span(
    span: &BezierSubcurve2,
    start: &Real,
    end: &Real,
    parameter: &Real,
    location: SpanParameterLocation,
    policy: &CurveContext,
) -> ExactCurveResult<Point2> {
    match location {
        SpanParameterLocation::Start => return Ok(span.start().clone()),
        SpanParameterLocation::End => return Ok(span.end().clone()),
        SpanParameterLocation::Interior => {}
    }
    let local = ((parameter - start) / (end - start)).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            cause.into(),
        )
    })?;
    match span {
        BezierSubcurve2::Quadratic(curve) => Ok(curve.point_at(local)),
        BezierSubcurve2::Cubic(curve) => Ok(curve.point_at(local)),
        BezierSubcurve2::RationalQuadratic(curve) => match curve.point_at(local, policy) {
            Classification::Decided(point) => Ok(point),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::PolynomialBSpline,
                reason,
            )),
        },
        BezierSubcurve2::Rational(curve) => match curve.point_at_classified(&local, policy) {
            Classification::Decided(point) => Ok(point),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::PolynomialBSpline,
                reason,
            )),
        },
    }
}

fn rationalize_subcurve(curve: &BezierSubcurve2) -> Cached<RationalBezier2> {
    let (control_points, weights) = match curve {
        BezierSubcurve2::Quadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 3],
        ),
        BezierSubcurve2::Cubic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            vec![Real::one(); 4],
        ),
        BezierSubcurve2::RationalQuadratic(curve) => (
            curve.control_points().into_iter().cloned().collect(),
            curve.weights().into_iter().cloned().collect(),
        ),
        BezierSubcurve2::Rational(curve) => return Ok(curve.clone()),
    };
    RationalBezier2::try_new(control_points, weights).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::NativeTopology,
            CurveFamily2::PolynomialBSpline,
            cause,
        )
    })
}

#[derive(Clone, Copy)]
struct SelectedSpan {
    index: usize,
    location: SpanParameterLocation,
}

#[derive(Clone, Copy)]
enum SpanParameterLocation {
    Start,
    Interior,
    End,
}

fn select_span_indices(
    intervals: &[(Real, Real)],
    parameter: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<(SelectedSpan, SelectedSpan)> {
    let mut first = None;
    let mut last = None;
    for (span_index, (start, end)) in intervals.iter().enumerate() {
        let lower = crate::classify::compare_reals(start, parameter, policy);
        let upper = crate::classify::compare_reals(parameter, end, policy);
        match (lower, upper) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less | Ordering::Equal)) => {
                let selected = SelectedSpan {
                    index: span_index,
                    location: if lower == Some(Ordering::Equal) {
                        SpanParameterLocation::Start
                    } else if upper == Some(Ordering::Equal) {
                        SpanParameterLocation::End
                    } else {
                        SpanParameterLocation::Interior
                    },
                };
                first.get_or_insert(selected);
                last = Some(selected);
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    first.zip(last).ok_or_else(|| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            CurveError::InvalidCurveParameter,
        )
    })
}

fn local_span_parameter(interval: &(Real, Real), parameter: &Real) -> ExactCurveResult<Real> {
    ((parameter - &interval.0) / (&interval.1 - &interval.0)).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            cause.into(),
        )
    })
}

fn exact_classification<T>(classification: Classification<T>) -> ExactCurveResult<T> {
    match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            reason,
        )),
    }
}

fn matching_spline_derivatives(
    first: Vec<CurveDerivative2>,
    second: Vec<CurveDerivative2>,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<CurveDerivative2>> {
    debug_assert_eq!(first.len(), second.len());
    for (first_derivative, second_derivative) in first.iter().zip(&second) {
        match (
            crate::classify::compare_reals(first_derivative.dx(), second_derivative.dx(), policy),
            crate::classify::compare_reals(first_derivative.dy(), second_derivative.dy(), policy),
        ) {
            (Some(Ordering::Equal), Some(Ordering::Equal)) => {}
            (Some(_), Some(_)) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    UncertaintyReason::Boundary,
                ));
            }
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    UncertaintyReason::RealSign,
                ));
            }
        }
    }
    Ok(first)
}

fn matching_spline_point(
    first: Point2,
    second: Point2,
    policy: &CurveContext,
) -> ExactCurveResult<Point2> {
    match (
        crate::classify::compare_reals(first.x(), second.x(), policy),
        crate::classify::compare_reals(first.y(), second.y(), policy),
    ) {
        (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(first),
        (Some(_), Some(_)) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            UncertaintyReason::Boundary,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            UncertaintyReason::RealSign,
        )),
    }
}

fn exact_value<T>(
    result: crate::CurveResult<Classification<T>>,
    operation: CurveOperation2,
) -> ExactCurveResult<T> {
    match result {
        Ok(Classification::Decided(value)) => Ok(value),
        Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::PolynomialBSpline,
            reason,
        )),
        Err(cause) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::PolynomialBSpline,
            cause,
        )),
    }
}

fn map_classified_curve_result<T>(
    result: crate::CurveResult<Classification<T>>,
    operation: CurveOperation2,
) -> ExactCurveResult<Classification<T>> {
    result.map_err(|cause| {
        ExactCurveError::invalid(operation, CurveFamily2::PolynomialBSpline, cause)
    })
}

fn require_classification<T>(
    classification: Classification<T>,
    operation: CurveOperation2,
) -> ExactCurveResult<T> {
    match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::PolynomialBSpline,
            reason,
        )),
    }
}

fn remap_spline_operation(error: ExactCurveError, operation: CurveOperation2) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { family, cause, .. } => {
            ExactCurveError::invalid(operation, family, cause)
        }
        ExactCurveError::Blocked(blocker) => {
            ExactCurveError::blocked(operation, blocker.family(), blocker.reason())
        }
    }
}

fn remap_spline_family_operation(
    error: ExactCurveError,
    operation: CurveOperation2,
) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { cause, .. } => {
            ExactCurveError::invalid(operation, CurveFamily2::PolynomialBSpline, cause)
        }
        ExactCurveError::Blocked(blocker) => {
            ExactCurveError::blocked(operation, CurveFamily2::PolynomialBSpline, blocker.reason())
        }
    }
}

#[cfg(all(test, target_pointer_width = "64"))]
mod layout_tests {
    use super::{PolynomialSplineCurve2, PolynomialSplineData2};

    #[test]
    fn polynomial_spline_carrier_keeps_compact_policy_aware_storage() {
        assert_eq!(core::mem::size_of::<PolynomialSplineCurve2>(), 8);
        assert_eq!(core::mem::size_of::<PolynomialSplineData2>(), 288);
    }
}
