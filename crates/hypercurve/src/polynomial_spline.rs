//! Policy-free retained polynomial B-spline carrier.

use std::cell::OnceCell;
use std::cmp::Ordering;
use std::rc::Rc;

use crate::spline_periodic::{expand_periodic_spline, wrap_periodic_parameter};
use crate::{
    BezierSubcurve2, Classification, CurveDerivative2, CurveError, CurveFamily2, CurveOperation2,
    CurveParameterSide2, CurvePolicy, CurveSource2, ExactCurveError, ExactCurveResult, NurbsCurve2,
    Point2, PolynomialBSplineBezierExtraction2, PolynomialBSplineCurve2, RationalBezier2, Real,
    Similarity2, SplinePeriodicity2, UncertaintyReason,
};

type Cached<T> = Result<T, ExactCurveError>;

#[derive(Debug)]
struct PolynomialSplineData2 {
    retained: PolynomialBSplineCurve2,
    source: Option<CurveSource2>,
    endpoints: SplineEndpoints2,
    decomposition: OnceCell<Cached<PolynomialSplineBezierDecomposition2>>,
    rational_spans: OnceCell<Cached<Vec<RationalBezier2>>>,
}

#[derive(Debug)]
enum SplineEndpoints2 {
    AuthoredControls,
    Extracted { start: Point2, end: Point2 },
}

/// Exact polynomial B-spline with retained source identity and decomposition.
///
/// Clones share the authored control net and lazy Boehm decomposition. Both a
/// successful decomposition and a contextual failure are calculated once.
#[derive(Clone, Debug)]
pub struct PolynomialSplineCurve2 {
    data: Rc<PolynomialSplineData2>,
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
    source: Option<CurveSource2>,
    curve: &'a BezierSubcurve2,
    interval: &'a (Real, Real),
}

impl PolynomialSplineCurve2 {
    /// Constructs a polynomial B-spline of any positive degree.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_source(degree, control_points, knots, None)
    }

    /// Constructs a polynomial B-spline with stable source provenance.
    pub fn try_new_with_source(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        source: CurveSource2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_source(degree, control_points, knots, Some(source))
    }

    /// Constructs a periodic polynomial B-spline from one period of controls and knots.
    pub fn try_new_periodic(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
    ) -> ExactCurveResult<Self> {
        Self::try_new_periodic_with_optional_source(degree, control_points, period_knots, None)
    }

    /// Constructs a periodic polynomial B-spline with stable source provenance.
    pub fn try_new_periodic_with_source(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
        source: CurveSource2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_periodic_with_optional_source(
            degree,
            control_points,
            period_knots,
            Some(source),
        )
    }

    fn try_new_periodic_with_optional_source(
        degree: usize,
        control_points: Vec<Point2>,
        period_knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        let expansion = expand_periodic_spline(
            degree,
            control_points,
            period_knots,
            CurveFamily2::PolynomialBSpline,
            source,
        )?;
        Self::try_new_expanded(
            degree,
            expansion.control_points,
            expansion.knots,
            source,
            SplinePeriodicity2::Periodic {
                period: expansion.period,
            },
        )
    }

    fn try_new_with_optional_source(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded(
            degree,
            control_points,
            knots,
            source,
            SplinePeriodicity2::NonPeriodic,
        )
    }

    fn try_new_expanded(
        degree: usize,
        control_points: Vec<Point2>,
        knots: Vec<Real>,
        source: Option<CurveSource2>,
        periodicity: SplinePeriodicity2,
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
                source,
                CurveError::InvalidBSpline,
            ));
        }
        let retained = exact_value(
            PolynomialBSplineCurve2::try_new_with_periodicity(
                degree,
                control_points,
                knots,
                periodicity,
                &CurvePolicy::certified(),
            ),
            CurveOperation2::Construction,
            source,
        )?;
        let decomposition = OnceCell::new();
        let endpoints = if has_clamped_endpoints(
            retained.knots(),
            retained.degree(),
            retained.control_points().len(),
            source,
        )? {
            SplineEndpoints2::AuthoredControls
        } else {
            let extraction = exact_value(
                retained.extract_bezier_spans(&CurvePolicy::certified()),
                CurveOperation2::Construction,
                source,
            )?;
            let intervals = source_intervals(&extraction, source)?;
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
            decomposition
                .set(Ok(PolynomialSplineBezierDecomposition2 {
                    extraction,
                    intervals,
                }))
                .expect("new decomposition cache is empty");
            SplineEndpoints2::Extracted { start, end }
        };
        let curve = Self {
            data: Rc::new(PolynomialSplineData2 {
                retained,
                source,
                endpoints,
                decomposition,
                rational_spans: OnceCell::new(),
            }),
        };
        curve.validate_periodic_seam()?;
        Ok(curve)
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

    /// Returns retained source identity when supplied by the caller.
    pub fn source(&self) -> Option<CurveSource2> {
        self.data.source
    }

    /// Inserts one exact knot without changing the polynomial spline image.
    pub fn insert_knot(&self, knot: Real) -> ExactCurveResult<Self> {
        let refined = self
            .as_unit_weight_nurbs()?
            .insert_knot(knot)
            .map_err(|error| {
                remap_spline_family_operation(error, CurveOperation2::KnotInsertion)
            })?;
        if refined.control_points().len() == self.control_points().len() {
            return Ok(self.clone());
        }
        Self::from_unit_weight_nurbs(refined, CurveOperation2::KnotInsertion)
    }

    /// Splits this polynomial spline exactly at a strict interior parameter.
    pub fn split_at(&self, parameter: Real) -> ExactCurveResult<(Self, Self)> {
        let (left, right) = self
            .as_unit_weight_nurbs()?
            .split_at(parameter)
            .map_err(|error| remap_spline_family_operation(error, CurveOperation2::Subdivision))?;
        Ok((
            Self::from_unit_weight_nurbs(left, CurveOperation2::Subdivision)?,
            Self::from_unit_weight_nurbs(right, CurveOperation2::Subdivision)?,
        ))
    }

    /// Returns an exact polynomial subcurve over an ordered source range.
    pub fn subcurve(&self, start: Real, end: Real) -> ExactCurveResult<Self> {
        let subcurve = self
            .as_unit_weight_nurbs()?
            .subcurve(start, end)
            .map_err(|error| remap_spline_family_operation(error, CurveOperation2::Subdivision))?;
        Self::from_unit_weight_nurbs(subcurve, CurveOperation2::Subdivision)
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
            self.data.source,
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
            self.data.source,
            self.periodicity().clone(),
        )
        .map_err(|error| remap_spline_operation(error, CurveOperation2::Transformation))
    }

    /// Returns whether exact Bezier decomposition has already been retained.
    pub fn is_bezier_decomposition_cached(&self) -> bool {
        self.data.decomposition.get().is_some()
    }

    /// Returns whether reusable rational span evaluators are retained.
    pub fn is_rational_span_cache_cached(&self) -> bool {
        self.data.rational_spans.get().is_some()
    }

    /// Returns the shared exact Bezier decomposition and source intervals.
    pub fn bezier_decomposition(&self) -> ExactCurveResult<&PolynomialSplineBezierDecomposition2> {
        cached_result(&self.data.decomposition, || {
            let extraction = exact_value(
                self.data
                    .retained
                    .extract_bezier_spans(&CurvePolicy::certified()),
                CurveOperation2::BezierDecomposition,
                self.data.source,
            )?;
            let intervals = source_intervals(&extraction, self.data.source)?;
            Ok(PolynomialSplineBezierDecomposition2 {
                extraction,
                intervals,
            })
        })
    }

    /// Iterates exact Bezier spans with source identity and knot intervals.
    pub fn bezier_spans(
        &self,
    ) -> ExactCurveResult<impl ExactSizeIterator<Item = PolynomialSplineBezierSpanView2<'_>>> {
        let source = self.data.source;
        let decomposition = self.bezier_decomposition()?;
        Ok(decomposition
            .spans()
            .iter()
            .zip(decomposition.intervals())
            .enumerate()
            .map(
                move |(span_index, (curve, interval))| PolynomialSplineBezierSpanView2 {
                    span_index,
                    source,
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
        if self.is_periodic_seam_parameter(parameter)? {
            let (domain_start, domain_end) = self.parameter_domain();
            let left = self.point_at_canonical_side(domain_end, CurveParameterSide2::Left)?;
            if side == CurveParameterSide2::Left {
                return Ok(left);
            }
            let right = self.point_at_canonical_side(domain_start, CurveParameterSide2::Right)?;
            if side == CurveParameterSide2::Right {
                return Ok(right);
            }
            return matching_spline_point(left, right, self.data.source);
        }
        self.point_at_canonical_side(parameter, side)
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
            self.data.source,
        )?;
        self.point_at_side(&wrapped, side)
    }

    fn point_at_canonical_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Point2> {
        let decomposition = self.bezier_decomposition()?;
        let (first, last) =
            select_span_indices(decomposition.intervals(), parameter, self.data.source)?;
        let first_interval = &decomposition.intervals()[first];
        let first_point = evaluate_span(
            &decomposition.spans()[first],
            &first_interval.0,
            &first_interval.1,
            parameter,
            self.data.source,
        )?;
        if first == last || side == CurveParameterSide2::Left {
            return Ok(first_point);
        }
        let last_interval = &decomposition.intervals()[last];
        let last_point = evaluate_span(
            &decomposition.spans()[last],
            &last_interval.0,
            &last_interval.1,
            parameter,
            self.data.source,
        )?;
        if side == CurveParameterSide2::Right {
            return Ok(last_point);
        }
        matching_spline_point(first_point, last_point, self.data.source)
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
        let mut derivatives = self.derivatives_at_side(parameter, 1, side)?;
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
        if self.is_periodic_seam_parameter(parameter)? {
            let (domain_start, domain_end) = self.parameter_domain();
            let left = self.derivatives_at_canonical_side(
                domain_end,
                max_order,
                CurveParameterSide2::Left,
            )?;
            if side == CurveParameterSide2::Left {
                return Ok(left);
            }
            let right = self.derivatives_at_canonical_side(
                domain_start,
                max_order,
                CurveParameterSide2::Right,
            )?;
            if side == CurveParameterSide2::Right {
                return Ok(right);
            }
            return matching_spline_derivatives(left, right, self.data.source);
        }
        self.derivatives_at_canonical_side(parameter, max_order, side)
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
            self.data.source,
        )?;
        self.derivatives_at_side(&wrapped, max_order, side)
    }

    fn derivatives_at_canonical_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let decomposition = self.bezier_decomposition()?;
        let (first, last) =
            select_span_indices(decomposition.intervals(), parameter, self.data.source)?;
        let first_derivatives = self.derivatives_on_span(first, parameter, max_order)?;
        if first == last || side == CurveParameterSide2::Left {
            return Ok(first_derivatives);
        }
        let last_derivatives = self.derivatives_on_span(last, parameter, max_order)?;
        if side == CurveParameterSide2::Right {
            return Ok(last_derivatives);
        }
        matching_spline_derivatives(first_derivatives, last_derivatives, self.data.source)
    }

    fn derivatives_on_span(
        &self,
        span_index: usize,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let interval = &self.bezier_decomposition()?.intervals()[span_index];
        let local = local_span_parameter(interval, parameter, self.data.source)?;
        let evaluator = &self.rational_spans()?[span_index];
        let local_derivatives = if max_order == 1 {
            vec![exact_classification(
                evaluator.derivative_at_classified(&local, &CurvePolicy::certified()),
                self.data.source,
            )?]
        } else {
            exact_classification(
                evaluator.derivatives_at_classified(&local, max_order, &CurvePolicy::certified()),
                self.data.source,
            )?
        };
        let inverse_width = (Real::one() / (&interval.1 - &interval.0)).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                CurveFamily2::PolynomialBSpline,
                self.data.source,
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

    fn rational_spans(&self) -> ExactCurveResult<&[RationalBezier2]> {
        let spans = cached_result(&self.data.rational_spans, || {
            self.bezier_decomposition()?
                .spans()
                .iter()
                .map(|curve| rationalize_subcurve(curve, self.data.source))
                .collect()
        })?;
        Ok(spans)
    }

    fn as_unit_weight_nurbs(&self) -> ExactCurveResult<NurbsCurve2> {
        let weights = vec![Real::one(); self.control_points().len()];
        let result = NurbsCurve2::try_new_expanded_with_periodicity(
            self.degree(),
            self.control_points().to_vec(),
            weights,
            self.knots().to_vec(),
            self.source(),
            self.periodicity().clone(),
        );
        result
            .map_err(|error| remap_spline_family_operation(error, CurveOperation2::NativeTopology))
    }

    fn from_unit_weight_nurbs(
        curve: NurbsCurve2,
        operation: CurveOperation2,
    ) -> ExactCurveResult<Self> {
        let result = Self::try_new_expanded(
            curve.degree(),
            curve.control_points().to_vec(),
            curve.knots().to_vec(),
            curve.source(),
            curve.periodicity().clone(),
        );
        result.map_err(|error| remap_spline_operation(error, operation))
    }

    fn validate_periodic_seam(&self) -> ExactCurveResult<()> {
        if !self.periodicity().is_periodic() {
            return Ok(());
        }
        let policy = CurvePolicy::certified();
        match (
            crate::classify::compare_reals(self.start().x(), self.end().x(), &policy),
            crate::classify::compare_reals(self.start().y(), self.end().y(), &policy),
        ) {
            (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(()),
            (Some(_), Some(_)) => Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::PolynomialBSpline,
                self.data.source,
                CurveError::PeriodicSplineSeamMismatch,
            )),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Construction,
                CurveFamily2::PolynomialBSpline,
                self.data.source,
                UncertaintyReason::RealSign,
            )),
        }
    }

    fn is_periodic_seam_parameter(&self, parameter: &Real) -> ExactCurveResult<bool> {
        if !self.periodicity().is_periodic() {
            return Ok(false);
        }
        let (start, end) = self.parameter_domain();
        let policy = CurvePolicy::certified();
        match (
            crate::classify::compare_reals(parameter, start, &policy),
            crate::classify::compare_reals(parameter, end, &policy),
        ) {
            (Some(Ordering::Equal), _) | (_, Some(Ordering::Equal)) => Ok(true),
            (Some(_), Some(_)) => Ok(false),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::PolynomialBSpline,
                self.data.source,
                UncertaintyReason::Ordering,
            )),
        }
    }
}

impl PartialEq for PolynomialSplineCurve2 {
    fn eq(&self, other: &Self) -> bool {
        self.data.retained == other.data.retained && self.data.source == other.data.source
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

    /// Returns the owning curve source identity, when supplied.
    pub const fn source(self) -> Option<CurveSource2> {
        self.source
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<Vec<(Real, Real)>> {
    let policy = CurvePolicy::certified();
    let degree = extraction.degree();
    let knots = extraction.refined_knots();
    let end = knots.len().saturating_sub(degree + 1);
    let mut intervals = Vec::with_capacity(extraction.spans().len());
    for index in degree..end {
        match crate::classify::compare_reals(&knots[index], &knots[index + 1], &policy) {
            Some(Ordering::Less) => {
                intervals.push((knots[index].clone(), knots[index + 1].clone()));
            }
            Some(Ordering::Equal) => {}
            Some(Ordering::Greater) => {
                return Err(ExactCurveError::invalid(
                    CurveOperation2::BezierDecomposition,
                    CurveFamily2::PolynomialBSpline,
                    source,
                    CurveError::InvalidBSpline,
                ));
            }
            None => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::BezierDecomposition,
                    CurveFamily2::PolynomialBSpline,
                    source,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    if intervals.len() != extraction.spans().len() {
        return Err(ExactCurveError::invalid(
            CurveOperation2::BezierDecomposition,
            CurveFamily2::PolynomialBSpline,
            source,
            CurveError::Topology("B-spline span/interval count mismatch".into()),
        ));
    }
    Ok(intervals)
}

fn has_clamped_endpoints(
    knots: &[Real],
    degree: usize,
    control_count: usize,
    source: Option<CurveSource2>,
) -> ExactCurveResult<bool> {
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(&knots[0], &knots[degree], &policy),
        crate::classify::compare_reals(
            knots.last().expect("validated spline has knots"),
            &knots[control_count],
            &policy,
        ),
    ) {
        (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(true),
        (Some(_), Some(_)) => Ok(false),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Construction,
            CurveFamily2::PolynomialBSpline,
            source,
            UncertaintyReason::Ordering,
        )),
    }
}

fn evaluate_span(
    span: &BezierSubcurve2,
    start: &Real,
    end: &Real,
    parameter: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Point2> {
    let local = ((parameter - start) / (end - start)).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            source,
            cause.into(),
        )
    })?;
    match span {
        BezierSubcurve2::Quadratic(curve) => Ok(curve.point_at(local)),
        BezierSubcurve2::Cubic(curve) => Ok(curve.point_at(local)),
        BezierSubcurve2::RationalQuadratic(curve) => {
            match curve.point_at(local, &CurvePolicy::certified()) {
                Classification::Decided(point) => Ok(point),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    source,
                    reason,
                )),
            }
        }
        BezierSubcurve2::Rational(curve) => {
            match curve.point_at_classified(&local, &CurvePolicy::certified()) {
                Classification::Decided(point) => Ok(point),
                Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    source,
                    reason,
                )),
            }
        }
    }
}

fn rationalize_subcurve(
    curve: &BezierSubcurve2,
    source: Option<CurveSource2>,
) -> Cached<RationalBezier2> {
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
            source,
            cause,
        )
    })
}

fn select_span_indices(
    intervals: &[(Real, Real)],
    parameter: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<(usize, usize)> {
    let policy = CurvePolicy::certified();
    let mut first = None;
    let mut last = None;
    for (span_index, (start, end)) in intervals.iter().enumerate() {
        match (
            crate::classify::compare_reals(start, parameter, &policy),
            crate::classify::compare_reals(parameter, end, &policy),
        ) {
            (Some(Ordering::Less | Ordering::Equal), Some(Ordering::Less | Ordering::Equal)) => {
                first.get_or_insert(span_index);
                last = Some(span_index);
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    source,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    first.zip(last).ok_or_else(|| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            source,
            CurveError::InvalidCurveParameter,
        )
    })
}

fn local_span_parameter(
    interval: &(Real, Real),
    parameter: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Real> {
    ((parameter - &interval.0) / (&interval.1 - &interval.0)).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            source,
            cause.into(),
        )
    })
}

fn exact_classification<T>(
    classification: Classification<T>,
    source: Option<CurveSource2>,
) -> ExactCurveResult<T> {
    match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            source,
            reason,
        )),
    }
}

fn matching_spline_derivatives(
    first: Vec<CurveDerivative2>,
    second: Vec<CurveDerivative2>,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Vec<CurveDerivative2>> {
    debug_assert_eq!(first.len(), second.len());
    let policy = CurvePolicy::certified();
    for (first_derivative, second_derivative) in first.iter().zip(&second) {
        match (
            crate::classify::compare_reals(first_derivative.dx(), second_derivative.dx(), &policy),
            crate::classify::compare_reals(first_derivative.dy(), second_derivative.dy(), &policy),
        ) {
            (Some(Ordering::Equal), Some(Ordering::Equal)) => {}
            (Some(_), Some(_)) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    source,
                    UncertaintyReason::Boundary,
                ));
            }
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::PolynomialBSpline,
                    source,
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<Point2> {
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(first.x(), second.x(), &policy),
        crate::classify::compare_reals(first.y(), second.y(), &policy),
    ) {
        (Some(Ordering::Equal), Some(Ordering::Equal)) => Ok(first),
        (Some(_), Some(_)) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            source,
            UncertaintyReason::Boundary,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::PolynomialBSpline,
            source,
            UncertaintyReason::RealSign,
        )),
    }
}

fn cached_result<T>(
    cache: &OnceCell<Cached<T>>,
    initialize: impl FnOnce() -> Cached<T>,
) -> ExactCurveResult<&T> {
    match cache.get_or_init(initialize) {
        Ok(value) => Ok(value),
        Err(error) => Err(error.clone()),
    }
}

fn exact_value<T>(
    result: crate::CurveResult<Classification<T>>,
    operation: CurveOperation2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<T> {
    match result {
        Ok(Classification::Decided(value)) => Ok(value),
        Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::PolynomialBSpline,
            source,
            reason,
        )),
        Err(cause) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::PolynomialBSpline,
            source,
            cause,
        )),
    }
}

fn remap_spline_operation(error: ExactCurveError, operation: CurveOperation2) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid {
            family,
            source,
            cause,
            ..
        } => ExactCurveError::invalid(operation, family, source, cause),
        ExactCurveError::Blocked(blocker) => ExactCurveError::blocked(
            operation,
            blocker.family(),
            blocker.source(),
            blocker.reason(),
        ),
    }
}

fn remap_spline_family_operation(
    error: ExactCurveError,
    operation: CurveOperation2,
) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { source, cause, .. } => {
            ExactCurveError::invalid(operation, CurveFamily2::PolynomialBSpline, source, cause)
        }
        ExactCurveError::Blocked(blocker) => ExactCurveError::blocked(
            operation,
            CurveFamily2::PolynomialBSpline,
            blocker.source(),
            blocker.reason(),
        ),
    }
}
