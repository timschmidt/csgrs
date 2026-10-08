//! Retained NURBS carrier with policy-isolated exact decomposition caches.

use std::sync::{Arc, OnceLock};

use crate::policy::{
    BoundedPolicyResultCache, PolicyEvaluationCache, resolve_bounded_cached_result,
    resolve_cached_evaluation, resolve_certified_operation,
};
use crate::spline_periodic::{expand_periodic_spline, wrap_periodic_parameter};
use crate::{
    BezierSubcurve2, Classification, CurveContext, CurveDerivative2, CurveError, CurveFamily2,
    CurveOperation2, CurveOutcome, CurveParameterSide2, ExactCurveError, ExactCurveResult, Point2,
    RationalBSplineBezierExtraction2, RationalBSplineCurve2, RationalBezier2, RationalBezierSpan2,
    Real, Similarity2, SplinePeriodicity2, UncertaintyReason,
};

const MAX_RETAINED_KNOT_REFINEMENTS: usize = 8;
const MAX_RETAINED_KNOT_REMOVALS: usize = 8;
const MAX_RETAINED_DEGREE_ELEVATIONS: usize = 8;

#[derive(Debug)]
struct NurbsData2 {
    retained: RationalBSplineCurve2,
    endpoints: NurbsEndpoints2,
    decomposition: PolicyEvaluationCache<NurbsBezierDecomposition2>,
    native_subcurves: PolicyEvaluationCache<Vec<BezierSubcurve2>>,
    rational_spans: PolicyEvaluationCache<Vec<RationalBezier2>>,
    knot_refinements: BoundedPolicyResultCache<Vec<Real>, NurbsCurve2>,
    knot_removals: BoundedPolicyResultCache<Real, Option<NurbsCurve2>>,
    degree_elevations: BoundedPolicyResultCache<usize, NurbsDegreeElevation2>,
    elevated_curves: BoundedPolicyResultCache<usize, NurbsCurve2>,
}

#[derive(Debug)]
enum NurbsEndpoints2 {
    AuthoredControls,
    Extracted { start: Point2, end: Point2 },
}

/// Exact rational B-spline/NURBS curve with shared lazy caches.
///
/// Clones share the same immutable source carrier and lazy exact caches. The
/// homogeneous Boehm decomposition and native-topology promotion retain one
/// decided exact value together with any strict blocker, so approximate-first
/// evaluation cannot certify a later strict request.
#[derive(Clone, Debug)]
pub struct NurbsCurve2 {
    data: Arc<NurbsData2>,
}

/// Exact homogeneous Bezier decomposition retained by a [`NurbsCurve2`].
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsBezierDecomposition2 {
    extraction: RationalBSplineBezierExtraction2,
}

/// Borrowed exact NURBS Bezier span and its knot-span index.
#[derive(Clone, Copy, Debug)]
pub struct NurbsBezierSpanView2<'a> {
    span_index: usize,
    span: &'a RationalBezierSpan2,
}

/// Borrowed native topology promoted from one exact NURBS span.
#[derive(Clone, Copy, Debug)]
pub struct NurbsNativeSpanView2<'a> {
    source_span: NurbsBezierSpanView2<'a>,
    curve: &'a BezierSubcurve2,
}

/// Clone-shared exact degree elevation of every NURBS knot span.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsDegreeElevation2 {
    source_degree: usize,
    target_degree: usize,
    spans: Arc<[NurbsElevatedBezierSpan2]>,
}

/// One exact elevated rational Bezier span with its original knot interval.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsElevatedBezierSpan2 {
    span_index: usize,
    parameter_start: Real,
    parameter_end: Real,
    curve: RationalBezier2,
}

impl NurbsCurve2 {
    /// Constructs a degree-one-or-higher NURBS curve over its active knot domain.
    ///
    /// The outcome records any terminal decision consumed while validating
    /// weights, knot ordering, endpoints, or a periodic seam.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::try_new_raw(degree, control_points, weights, knots, attempt)
        })
    }

    pub(crate) fn try_new_raw(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_source_and_policy(
            degree,
            control_points,
            weights,
            knots,
            policy,
        )
    }

    /// Constructs a periodic NURBS from one period of controls and knot breaks.
    ///
    /// `period_knots` must contain exactly one more entry than the unique
    /// control count. Hypercurve extends the cyclic control and knot sequences
    /// exactly and certifies closure at the canonical seam. The outcome records
    /// any terminal decision consumed by that complete construction.
    pub fn try_new_periodic(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            Self::try_new_periodic_raw(degree, control_points, weights, period_knots, attempt)
        })
    }

    pub(crate) fn try_new_periodic_raw(
        degree: usize,
        control_points: Vec<Point2>,
        mut weights: Vec<Real>,
        period_knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if weights.len() != control_points.len() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::Nurbs,
                CurveError::InvalidPeriodicSpline,
            ));
        }
        let expansion = expand_periodic_spline(
            degree,
            control_points,
            period_knots,
            CurveFamily2::Nurbs,
            policy,
        )?;
        weights.extend_from_within(..degree);
        Self::try_new_expanded_with_policy(
            degree,
            expansion.control_points,
            weights,
            expansion.knots,
            SplinePeriodicity2::Periodic {
                period: expansion.period,
            },
            policy,
        )
    }

    fn try_new_with_optional_source_and_policy(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded_with_policy(
            degree,
            control_points,
            weights,
            knots,
            SplinePeriodicity2::NonPeriodic,
            policy,
        )
    }

    fn try_new_expanded_with_policy(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
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
                    .map(|knots| (order, knots))
            })
            .is_some_and(|(order, expected_knots)| {
                degree >= 1
                    && control_points.len() == weights.len()
                    && control_points.len() >= order
                    && knots.len() == expected_knots
            });
        if !valid_layout {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::Nurbs,
                CurveError::InvalidBSpline,
            ));
        }
        let retained = exact_value(
            RationalBSplineCurve2::try_new_with_periodicity(
                degree,
                control_points,
                weights,
                knots,
                periodicity,
                policy,
            ),
            CurveOperation2::Construction,
        )?;
        Self::from_retained(retained, None, policy)
    }

    #[cfg(feature = "svg")]
    pub(crate) fn try_new_expanded_with_periodicity(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded_with_policy(
            degree,
            control_points,
            weights,
            knots,
            periodicity,
            &CurveContext::STRICT,
        )
    }

    pub(crate) fn try_new_expanded_with_periodicity_and_policy(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        periodicity: SplinePeriodicity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded_with_policy(
            degree,
            control_points,
            weights,
            knots,
            periodicity,
            policy,
        )
    }

    fn from_retained(
        retained: RationalBSplineCurve2,
        preserved_endpoints: Option<(Point2, Point2)>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let decomposition = PolicyEvaluationCache::new();
        let endpoints = if let Some((start, end)) = preserved_endpoints {
            NurbsEndpoints2::Extracted { start, end }
        } else if has_clamped_endpoints(
            retained.knots(),
            retained.degree(),
            retained.control_points().len(),
            policy,
            CurveOperation2::Construction,
        )? {
            NurbsEndpoints2::AuthoredControls
        } else {
            let extraction = exact_value(
                retained.extract_bezier_spans(policy),
                CurveOperation2::Construction,
            )?;
            let start = extraction
                .spans()
                .first()
                .and_then(|span| span.control_points().first())
                .expect("validated NURBS has a positive span")
                .clone();
            let end = extraction
                .spans()
                .last()
                .and_then(|span| span.control_points().last())
                .expect("validated NURBS has a positive span")
                .clone();
            if !policy.permits_approximate_512() {
                decomposition.seed_certified(NurbsBezierDecomposition2 { extraction });
            }
            NurbsEndpoints2::Extracted { start, end }
        };
        let curve = Self {
            data: Arc::new(NurbsData2 {
                retained,
                endpoints,
                decomposition,
                native_subcurves: PolicyEvaluationCache::new(),
                rational_spans: PolicyEvaluationCache::new(),
                knot_refinements: OnceLock::new(),
                knot_removals: OnceLock::new(),
                degree_elevations: OnceLock::new(),
                elevated_curves: OnceLock::new(),
            }),
        };
        curve.validate_periodic_seam(policy)?;
        Ok(curve)
    }

    /// Returns the rational polynomial degree.
    pub fn degree(&self) -> usize {
        self.data.retained.degree()
    }

    /// Returns the exact affine control net.
    pub fn control_points(&self) -> &[Point2] {
        self.data.retained.control_points()
    }

    /// Returns exact homogeneous weights.
    pub fn weights(&self) -> &[Real] {
        self.data.retained.weights()
    }

    /// Returns the exact knot vector.
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

    /// Returns the exact period when this NURBS is periodic.
    pub fn period(&self) -> Option<&Real> {
        self.periodicity().period()
    }

    /// Inserts one exact knot with homogeneous Boehm refinement.
    ///
    /// The curve image, parameterization, and endpoints are preserved. If an
    /// interior knot already has full Bezier multiplicity, this returns a clone
    /// sharing the original carrier and caches.
    pub fn insert_knot(
        &self,
        knot: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        self.insert_knots(vec![knot], policy)
    }

    /// Inserts an ordered batch of exact knots in one homogeneous refinement pass.
    ///
    /// The working control net is projected and validated only once. Exact
    /// periodicity, endpoints, and parameterization are preserved. Repeated
    /// equal requests from any clone reuse a bounded retained result.
    pub fn insert_knots(
        &self,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        if knots.is_empty() {
            return Ok(CurveOutcome::new(
                self.clone(),
                crate::CurveCertainty::Certified,
            ));
        }
        resolve_certified_operation(policy, |attempt| {
            self.insert_knots_with_policy(knots, attempt)
        })
    }

    pub(crate) fn insert_knots_with_policy(
        &self,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if knots.is_empty() {
            return Ok(self.clone());
        }
        resolve_bounded_cached_result(
            &self.data.knot_refinements,
            knots,
            MAX_RETAINED_KNOT_REFINEMENTS,
            policy,
            |retained_knots, attempt| {
                self.insert_knots_uncached_with_policy(retained_knots.clone(), attempt)
            },
        )
    }

    /// Removes one exact interior knot occurrence when that preserves the curve.
    ///
    /// Removal is certified as the inverse of homogeneous Boehm insertion: the
    /// candidate control net is solved exactly, reinserted, and compared with
    /// every authored homogeneous control and knot. `None` means the requested
    /// knot is absent or is not exactly removable. Results are retained across
    /// clones, including negative results and blockers.
    pub fn remove_knot(
        &self,
        knot: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Option<Self>>> {
        resolve_certified_operation(policy, |attempt| {
            self.remove_knot_with_policy(knot, attempt)
        })
    }

    fn remove_knot_with_policy(
        &self,
        knot: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Self>> {
        resolve_bounded_cached_result(
            &self.data.knot_removals,
            knot,
            MAX_RETAINED_KNOT_REMOVALS,
            policy,
            |retained_knot, attempt| {
                validate_strict_interior(
                    self,
                    retained_knot,
                    CurveOperation2::KnotRemoval,
                    attempt,
                )?;
                self.remove_knot_uncached_with_policy(retained_knot.clone(), attempt)
            },
        )
    }

    /// Elevates every exact rational Bezier knot span to `target_degree`.
    ///
    /// The result retains the original knot intervals so callers can consume
    /// elevated homogeneous spans without changing the
    /// NURBS parameterization or inventing a less-continuous replacement knot
    /// vector. Equal requests and blockers are retained across clones.
    pub fn degree_elevation(
        &self,
        target_degree: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<NurbsDegreeElevation2>> {
        resolve_certified_operation(policy, |attempt| {
            self.degree_elevation_with_policy(target_degree, attempt)
        })
    }

    fn degree_elevation_with_policy(
        &self,
        target_degree: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<NurbsDegreeElevation2> {
        if target_degree < self.degree() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::Nurbs,
                CurveError::InvalidDegreeElevation,
            ));
        }
        resolve_bounded_cached_result(
            &self.data.degree_elevations,
            target_degree,
            MAX_RETAINED_DEGREE_ELEVATIONS,
            policy,
            |retained_degree, attempt| self.degree_elevation_uncached(*retained_degree, attempt),
        )
    }

    /// Returns an exact NURBS carrier elevated to `target_degree`.
    ///
    /// Every rational Bezier span is elevated homogeneously, adjacent span
    /// scales are aligned, and inverse knot insertion removes the extraction
    /// knots needed to restore the source continuity order. The resulting
    /// NURBS preserves the authored parameter domain, periodicity, source, and
    /// parameterized image. Equal requests and blockers are retained across
    /// clones.
    pub fn elevated_to_degree(
        &self,
        target_degree: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        if target_degree < self.degree() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::Nurbs,
                CurveError::InvalidDegreeElevation,
            ));
        }
        if target_degree == self.degree() {
            return Ok(CurveOutcome::new(
                self.clone(),
                crate::CurveCertainty::Certified,
            ));
        }
        resolve_certified_operation(policy, |attempt| {
            self.elevated_to_degree_with_policy(target_degree, attempt)
        })
    }

    fn elevated_to_degree_with_policy(
        &self,
        target_degree: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if target_degree == self.degree() {
            return Ok(self.clone());
        }
        resolve_bounded_cached_result(
            &self.data.elevated_curves,
            target_degree,
            MAX_RETAINED_DEGREE_ELEVATIONS,
            policy,
            |retained_degree, attempt| self.elevated_to_degree_uncached(*retained_degree, attempt),
        )
    }

    fn degree_elevation_uncached(
        &self,
        target_degree: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<NurbsDegreeElevation2> {
        let decomposition =
            self.bezier_decomposition_for_operation(policy, CurveOperation2::DegreeElevation)?;
        let rational_spans =
            self.rational_spans_for_operation(policy, CurveOperation2::DegreeElevation)?;
        let spans = decomposition
            .spans()
            .iter()
            .zip(rational_spans)
            .enumerate()
            .map(|(span_index, (source_span, rational_span))| {
                let curve = rational_span
                    .elevated_to_degree(target_degree)
                    .map_err(remap_degree_elevation_error)?;
                let (parameter_start, parameter_end) = source_span.knot_interval();
                Ok(NurbsElevatedBezierSpan2 {
                    span_index,
                    parameter_start: parameter_start.clone(),
                    parameter_end: parameter_end.clone(),
                    curve,
                })
            })
            .collect::<ExactCurveResult<Vec<_>>>()?;
        Ok(NurbsDegreeElevation2 {
            source_degree: self.degree(),
            target_degree,
            spans: spans.into(),
        })
    }

    fn elevated_to_degree_uncached(
        &self,
        target_degree: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let elevation = self.degree_elevation_with_policy(target_degree, policy)?;
        let (mut elevated, removable_knots) =
            self.piecewise_elevated_curve(&elevation, target_degree, policy)?;
        for (knot, removal_count) in removable_knots {
            for _ in 0..removal_count {
                elevated = elevated
                    .remove_knot_with_policy(knot.clone(), policy)
                    .map_err(|error| {
                        remap_nurbs_operation(error, CurveOperation2::DegreeElevation)
                    })?
                    .ok_or_else(|| {
                        ExactCurveError::invalid(
                            CurveOperation2::DegreeElevation,
                            CurveFamily2::Nurbs,
                            CurveError::InvalidDegreeElevation,
                        )
                    })?;
            }
        }
        Ok(elevated)
    }

    fn piecewise_elevated_curve(
        &self,
        elevation: &NurbsDegreeElevation2,
        target_degree: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<(Self, Vec<(Real, usize)>)> {
        let spans = elevation.spans();
        let mut span_weights = spans
            .iter()
            .map(|span| span.curve().weights().to_vec())
            .collect::<Vec<_>>();
        let mut multiplicities = Vec::with_capacity(spans.len().saturating_sub(1));
        for span_index in 1..spans.len() {
            let knot = spans[span_index].parameter_start.clone();
            let multiplicity = exact_nurbs_knot_multiplicity(
                self.knots(),
                &knot,
                CurveOperation2::DegreeElevation,
                policy,
            )?;
            if multiplicity <= self.degree() {
                exact_points_equal(
                    spans[span_index - 1].curve().end(),
                    spans[span_index].curve().start(),
                    CurveOperation2::DegreeElevation,
                    policy,
                )?;
                let scale = (span_weights[span_index - 1]
                    .last()
                    .expect("elevated span has weights")
                    / span_weights[span_index]
                        .first()
                        .expect("elevated span has weights"))
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::DegreeElevation,
                        CurveFamily2::Nurbs,
                        cause.into(),
                    )
                })?;
                for weight in &mut span_weights[span_index] {
                    *weight *= &scale;
                }
            }
            multiplicities.push((knot, multiplicity));
        }

        let mut control_points = Vec::new();
        let mut weights = Vec::new();
        let mut knots = Vec::new();
        let domain_start = spans
            .first()
            .expect("validated NURBS has a positive span")
            .parameter_start
            .clone();
        knots.extend(std::iter::repeat_n(domain_start, target_degree + 1));
        control_points.extend_from_slice(spans[0].curve().control_points());
        weights.extend_from_slice(&span_weights[0]);
        let mut removable_knots = Vec::new();
        for (span_index, (knot, source_multiplicity)) in multiplicities.iter().enumerate() {
            let discontinuous = *source_multiplicity == self.degree() + 1;
            knots.extend(std::iter::repeat_n(
                knot.clone(),
                if discontinuous {
                    target_degree + 1
                } else {
                    target_degree
                },
            ));
            let next_span = &spans[span_index + 1];
            let first_control = usize::from(!discontinuous);
            control_points.extend_from_slice(&next_span.curve().control_points()[first_control..]);
            weights.extend_from_slice(&span_weights[span_index + 1][first_control..]);
            removable_knots.push((
                knot.clone(),
                self.degree().saturating_sub(*source_multiplicity),
            ));
        }
        let domain_end = spans
            .last()
            .expect("validated NURBS has a positive span")
            .parameter_end
            .clone();
        knots.extend(std::iter::repeat_n(domain_end, target_degree + 1));
        let curve = Self::try_new_expanded_with_policy(
            target_degree,
            control_points,
            weights,
            knots,
            self.periodicity().clone(),
            policy,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::DegreeElevation))?;
        Ok((curve, removable_knots))
    }

    fn insert_knots_uncached_with_policy(
        &self,
        knots: Vec<Real>,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        let (retained, inserted_count) = exact_value(
            self.data.retained.insert_knots(knots, policy),
            CurveOperation2::KnotInsertion,
        )?;
        if inserted_count == 0 {
            return Ok(self.clone());
        }
        Self::from_retained(
            retained,
            Some((self.start().clone(), self.end().clone())),
            policy,
        )
    }

    fn remove_knot_uncached_with_policy(
        &self,
        knot: Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<Self>> {
        let retained = exact_value(
            self.data.retained.remove_knot(knot, policy),
            CurveOperation2::KnotRemoval,
        )?;
        retained
            .map(|retained| {
                Self::from_retained(
                    retained,
                    Some((self.start().clone(), self.end().clone())),
                    policy,
                )
                .map_err(|error| remap_nurbs_operation(error, CurveOperation2::KnotRemoval))
            })
            .transpose()
    }

    /// Splits this NURBS exactly at a strict interior knot-domain parameter.
    ///
    /// The returned [`CurveOutcome`] records whether parameter ordering, knot
    /// refinement, or reconstructed-carrier validation consumed the
    /// `APPROXIMATE_512` terminal.
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
        validate_strict_interior_parameter(self, &parameter, policy)?;
        let refined = self
            .insert_knots_with_policy(vec![parameter.clone(); self.degree()], policy)
            .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Subdivision))?;
        let equal_indices = refined
            .knots()
            .iter()
            .enumerate()
            .filter_map(|(index, knot)| {
                (crate::classify::compare_reals(knot, &parameter, policy)
                    == Some(std::cmp::Ordering::Equal))
                .then_some(index)
            })
            .collect::<Vec<_>>();
        if !matches!(equal_indices.len(), count if count == self.degree() || count == self.degree() + 1)
        {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Subdivision,
                CurveFamily2::Nurbs,
                UncertaintyReason::Ordering,
            ));
        }
        let first_knot = equal_indices[0];
        let last_knot = *equal_indices.last().expect("nonempty knot run");
        let right_start = last_knot - self.degree();
        let left_end = if equal_indices.len() == self.degree() {
            right_start
        } else {
            right_start - 1
        };
        let mut left_knots = refined.knots()[..first_knot].to_vec();
        left_knots.extend(std::iter::repeat_n(parameter.clone(), self.degree() + 1));
        let mut right_knots = vec![parameter; self.degree() + 1];
        right_knots.extend_from_slice(&refined.knots()[last_knot + 1..]);
        let left = Self::try_new_with_optional_source_and_policy(
            self.degree(),
            refined.control_points()[..=left_end].to_vec(),
            refined.weights()[..=left_end].to_vec(),
            left_knots,
            policy,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Subdivision))?;
        let right = Self::try_new_with_optional_source_and_policy(
            self.degree(),
            refined.control_points()[right_start..].to_vec(),
            refined.weights()[right_start..].to_vec(),
            right_knots,
            policy,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Subdivision))?;
        Ok((left, right))
    }

    /// Returns an exact NURBS subcurve over an ordered source-parameter range.
    ///
    /// One operation outcome covers range validation, every split, and exact
    /// reconstructed-carrier validation.
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
        validate_subcurve_range(self, &start, &end, policy)?;
        let (domain_start, domain_end) = self.parameter_domain();
        let starts_at_domain = crate::classify::compare_reals(&start, domain_start, policy)
            == Some(std::cmp::Ordering::Equal);
        let ends_at_domain = crate::classify::compare_reals(&end, domain_end, policy)
            == Some(std::cmp::Ordering::Equal);
        if starts_at_domain && ends_at_domain {
            return Ok(self.clone());
        }
        let through_end = if ends_at_domain {
            self.clone()
        } else {
            self.split_at_raw(end, policy)?.0
        };
        if starts_at_domain {
            Ok(through_end)
        } else {
            Ok(through_end.split_at_raw(start, policy)?.1)
        }
    }

    /// Returns an exact finite subcurve in clamped piecewise-Bézier NURBS form.
    ///
    /// This is the topology-ingestion form for a range cut from an unclamped or
    /// periodic carrier. It preserves the source parameter interval and exact
    /// rational image while replacing irrelevant exterior knots with clamped
    /// endpoints. Internal spans use full Bézier multiplicity; no fitting,
    /// sampling, or endpoint-only reconstruction is involved.
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
        validate_subcurve_range(self, &start, &end, policy)?;
        let subcurve = self.subcurve_raw(start, end, policy)?;
        if !subcurve.periodicity().is_periodic()
            && has_clamped_endpoints(
                subcurve.knots(),
                subcurve.degree(),
                subcurve.control_points().len(),
                policy,
                CurveOperation2::Subdivision,
            )?
        {
            return Ok(subcurve);
        }
        subcurve.clamped_piecewise_bezier_form(policy)
    }

    fn clamped_piecewise_bezier_form(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        let decomposition =
            self.bezier_decomposition_for_operation(policy, CurveOperation2::Subdivision)?;
        let spans = decomposition.spans();
        let first = spans.first().ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::Subdivision,
                CurveFamily2::Nurbs,
                CurveError::InvalidBSpline,
            )
        })?;
        let degree = self.degree();
        let mut control_points = first.control_points().to_vec();
        let mut weights = first.weights().to_vec();
        let mut knots = Vec::with_capacity(control_points.len() + degree + 1);
        knots.extend(std::iter::repeat_n(
            first.knot_interval().0.clone(),
            degree + 1,
        ));
        for span in &spans[1..] {
            let scale = (weights.last().expect("first exact NURBS span has weights")
                / &span.weights()[0])
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Subdivision,
                        CurveFamily2::Nurbs,
                        cause.into(),
                    )
                })?;
            control_points.extend(span.control_points().iter().skip(1).cloned());
            weights.extend(span.weights().iter().skip(1).map(|weight| weight * &scale));
            knots.extend(std::iter::repeat_n(span.knot_interval().0.clone(), degree));
        }
        knots.extend(std::iter::repeat_n(
            spans
                .last()
                .expect("exact NURBS decomposition is nonempty")
                .knot_interval()
                .1
                .clone(),
            degree + 1,
        ));
        Self::try_new_with_optional_source_and_policy(
            degree,
            control_points,
            weights,
            knots,
            policy,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Subdivision))
    }

    /// Returns the same NURBS image with traversal direction reversed.
    ///
    /// Controls and weights are reversed, while knots are reflected through
    /// the parameter-domain midpoint. The parameter domain is preserved exactly.
    pub fn reversed(&self, policy: &CurveContext) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| self.reversed_raw(attempt))
    }

    pub(crate) fn reversed_raw(&self, policy: &CurveContext) -> ExactCurveResult<Self> {
        let (start, end) = self.parameter_domain();
        let knot_sum = start + end;
        let mut control_points = self.control_points().to_vec();
        let mut weights = self.weights().to_vec();
        control_points.reverse();
        weights.reverse();
        let knots = self
            .knots()
            .iter()
            .rev()
            .map(|knot| &knot_sum - knot)
            .collect();
        Self::try_new_expanded_with_policy(
            self.degree(),
            control_points,
            weights,
            knots,
            self.periodicity().clone(),
            policy,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Reversal))
    }

    /// Applies an exact planar similarity while retaining periodicity.
    pub fn transform_similarity(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Self>> {
        resolve_certified_operation(policy, |attempt| {
            self.transform_similarity_raw(transform, attempt)
        })
    }

    pub(crate) fn transform_similarity_raw(
        &self,
        transform: &Similarity2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded_with_policy(
            self.degree(),
            self.control_points()
                .iter()
                .map(|point| transform.transform_point(point))
                .collect(),
            self.weights().to_vec(),
            self.knots().to_vec(),
            self.periodicity().clone(),
            policy,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Transformation))
    }

    /// Returns the exact active-domain start point.
    pub fn start(&self) -> &Point2 {
        match &self.data.endpoints {
            NurbsEndpoints2::AuthoredControls => &self.data.retained.control_points()[0],
            NurbsEndpoints2::Extracted { start, .. } => start,
        }
    }

    /// Returns the exact active-domain end point.
    pub fn end(&self) -> &Point2 {
        match &self.data.endpoints {
            NurbsEndpoints2::AuthoredControls => self
                .data
                .retained
                .control_points()
                .last()
                .expect("validated NURBS has controls"),
            NurbsEndpoints2::Extracted { end, .. } => end,
        }
    }

    /// Returns the shared exact homogeneous Bezier decomposition.
    pub fn bezier_decomposition(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&NurbsBezierDecomposition2>> {
        resolve_certified_operation(policy, |attempt| {
            self.bezier_decomposition_for_operation(attempt, CurveOperation2::BezierDecomposition)
        })
    }

    pub(crate) fn bezier_decomposition_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&NurbsBezierDecomposition2>> {
        resolve_cached_evaluation(&self.data.decomposition, policy, |attempt| {
            map_classified_curve_result(
                self.data.retained.extract_bezier_spans(attempt),
                CurveOperation2::BezierDecomposition,
            )
            .map(|decomposition| {
                decomposition.map(|extraction| NurbsBezierDecomposition2 { extraction })
            })
        })
    }

    fn bezier_decomposition_for_operation(
        &self,
        policy: &CurveContext,
        operation: CurveOperation2,
    ) -> ExactCurveResult<&NurbsBezierDecomposition2> {
        require_classification(
            self.bezier_decomposition_with_policy(policy)
                .map_err(|error| remap_nurbs_operation(error, operation))?,
            operation,
        )
    }

    /// Iterates exact retained Bezier spans with indices and knot intervals.
    pub fn bezier_spans(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<impl ExactSizeIterator<Item = NurbsBezierSpanView2<'_>>>>
    {
        resolve_certified_operation(policy, |attempt| {
            Ok(self
                .bezier_decomposition_for_operation(attempt, CurveOperation2::BezierDecomposition)?
                .spans()
                .iter()
                .enumerate()
                .map(move |(span_index, span)| NurbsBezierSpanView2 { span_index, span }))
        })
    }

    /// Returns native conic/polynomial Bezier spans when every span supports them.
    ///
    /// Linear rational spans are elevated exactly in homogeneous coordinates,
    /// quadratics use native conics, equal-weight cubics collapse to polynomial
    /// cubics, and all remaining spans use exact general rational Beziers.
    pub fn native_subcurves(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<&[BezierSubcurve2]>> {
        resolve_certified_operation(policy, |attempt| {
            self.native_subcurves_for_operation(attempt, CurveOperation2::NativeTopology)
        })
    }

    pub(crate) fn native_subcurves_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<&[BezierSubcurve2]>> {
        Ok(
            match resolve_cached_evaluation(&self.data.native_subcurves, policy, |attempt| {
                let decomposition = match self.bezier_decomposition_with_policy(attempt)? {
                    Classification::Decided(decomposition) => decomposition,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                map_classified_curve_result(
                    decomposition.extraction.native_subcurves(attempt),
                    CurveOperation2::NativeTopology,
                )
            })? {
                Classification::Decided(subcurves) => Classification::Decided(subcurves.as_slice()),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
        )
    }

    fn native_subcurves_for_operation(
        &self,
        policy: &CurveContext,
        operation: CurveOperation2,
    ) -> ExactCurveResult<&[BezierSubcurve2]> {
        require_classification(
            self.native_subcurves_with_policy(policy)
                .map_err(|error| remap_nurbs_operation(error, operation))?,
            operation,
        )
    }

    /// Iterates native promoted spans without losing their rational source span.
    pub fn native_spans(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<impl ExactSizeIterator<Item = NurbsNativeSpanView2<'_>>>>
    {
        resolve_certified_operation(policy, |attempt| {
            let decomposition =
                self.bezier_decomposition_for_operation(attempt, CurveOperation2::NativeTopology)?;
            let native =
                self.native_subcurves_for_operation(attempt, CurveOperation2::NativeTopology)?;
            debug_assert_eq!(decomposition.spans().len(), native.len());
            Ok(decomposition.spans().iter().zip(native).enumerate().map(
                move |(span_index, (span, curve))| NurbsNativeSpanView2 {
                    source_span: NurbsBezierSpanView2 { span_index, span },
                    curve,
                },
            ))
        })
    }

    /// Evaluates the NURBS at an exact source-domain parameter.
    ///
    /// The exact homogeneous Bezier decomposition is retained on first use.
    /// Evaluation then selects the source knot span and applies homogeneous de
    /// Casteljau interpolation without finite projection.
    pub fn point_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.point_at_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates an exact point with explicit knot-boundary side policy.
    pub fn point_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        resolve_certified_operation(policy, |attempt| {
            self.point_at_side_with_policy(parameter, side, attempt)
        })
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
            return matching_nurbs_point(left, right, policy);
        }
        self.point_at_canonical_side(parameter, side, policy)
    }

    /// Evaluates a periodic NURBS at any exactly wrappable parameter.
    pub fn point_at_wrapped(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        self.point_at_wrapped_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates a periodic NURBS with explicit side selection at wrapped seams.
    pub fn point_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Point2>> {
        resolve_certified_operation(policy, |attempt| {
            self.point_at_wrapped_side_with_policy(parameter, side, attempt)
        })
    }

    pub(crate) fn point_at_wrapped_side_with_policy(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        let (start, end) = self.parameter_domain();
        let wrapped = wrap_periodic_parameter(
            parameter,
            start,
            end,
            self.periodicity(),
            side,
            CurveFamily2::Nurbs,
            policy,
        )?;
        self.point_at_side_with_policy(&wrapped, side, policy)
    }

    fn point_at_canonical_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        let decomposition =
            self.bezier_decomposition_for_operation(policy, CurveOperation2::Evaluation)?;
        let (first, last) = select_span_indices(decomposition.spans(), parameter, policy)?;
        let first_point = self.point_on_span(first.index, parameter, first.location, policy)?;
        if first.index == last.index || side == CurveParameterSide2::Left {
            return Ok(first_point);
        }
        let last_point = self.point_on_span(last.index, parameter, last.location, policy)?;
        if side == CurveParameterSide2::Right {
            return Ok(last_point);
        }
        matching_nurbs_point(first_point, last_point, policy)
    }

    fn point_on_span(
        &self,
        span_index: usize,
        parameter: &Real,
        location: NurbsSpanParameterLocation,
        policy: &CurveContext,
    ) -> ExactCurveResult<Point2> {
        let curve =
            &self.rational_spans_for_operation(policy, CurveOperation2::Evaluation)?[span_index];
        match location {
            NurbsSpanParameterLocation::Start => return Ok(curve.start().clone()),
            NurbsSpanParameterLocation::End => return Ok(curve.end().clone()),
            NurbsSpanParameterLocation::Interior => {}
        }
        let decomposition =
            self.bezier_decomposition_for_operation(policy, CurveOperation2::Evaluation)?;
        let local = local_span_parameter(&decomposition.spans()[span_index], parameter)?;
        exact_classification(
            curve.point_at_classified(&local, policy),
            CurveOperation2::Evaluation,
        )
    }

    /// Evaluates the exact first derivative in the authored knot parameter.
    pub fn derivative_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.derivative_at_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates an exact first derivative with explicit knot-boundary side policy.
    pub fn derivative_at_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        resolve_certified_operation(policy, |attempt| {
            let mut derivatives =
                self.derivatives_at_side_with_policy(parameter, 1, side, attempt)?;
            Ok(derivatives.pop().expect("one derivative requested"))
        })
    }

    /// Evaluates the first periodic derivative at any wrappable parameter.
    pub fn derivative_at_wrapped(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        self.derivative_at_wrapped_side(parameter, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates the first periodic derivative with explicit seam-side selection.
    pub fn derivative_at_wrapped_side(
        &self,
        parameter: &Real,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveDerivative2>> {
        resolve_certified_operation(policy, |attempt| {
            let mut derivatives =
                self.derivatives_at_wrapped_side_with_policy(parameter, 1, side, attempt)?;
            Ok(derivatives.pop().expect("one derivative requested"))
        })
    }

    /// Evaluates exact derivatives through `max_order` in the authored knot parameter.
    ///
    /// The returned vector stores orders `1..=max_order`. Each local rational
    /// Bezier derivative is scaled by the corresponding power of the inverse
    /// source-span width, preserving the authored NURBS parameterization.
    pub fn derivatives_at(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.derivatives_at_side(parameter, max_order, CurveParameterSide2::Automatic, policy)
    }

    /// Evaluates exact derivatives with explicit knot-boundary side policy.
    pub fn derivatives_at_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.derivatives_at_side_with_policy(parameter, max_order, side, attempt)
        })
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
            return matching_nurbs_derivatives(left, right, policy);
        }
        self.derivatives_at_canonical_side(parameter, max_order, side, policy)
    }

    /// Evaluates periodic derivatives through `max_order` at any wrappable parameter.
    pub fn derivatives_at_wrapped(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        self.derivatives_at_wrapped_side(
            parameter,
            max_order,
            CurveParameterSide2::Automatic,
            policy,
        )
    }

    /// Evaluates periodic derivatives with explicit side selection at wrapped seams.
    pub fn derivatives_at_wrapped_side(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<Vec<CurveDerivative2>>> {
        resolve_certified_operation(policy, |attempt| {
            self.derivatives_at_wrapped_side_with_policy(parameter, max_order, side, attempt)
        })
    }

    pub(crate) fn derivatives_at_wrapped_side_with_policy(
        &self,
        parameter: &Real,
        max_order: usize,
        side: CurveParameterSide2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let (start, end) = self.parameter_domain();
        let wrapped = wrap_periodic_parameter(
            parameter,
            start,
            end,
            self.periodicity(),
            side,
            CurveFamily2::Nurbs,
            policy,
        )?;
        self.derivatives_at_side_with_policy(&wrapped, max_order, side, policy)
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
        let (first, last) = select_span_indices(decomposition.spans(), parameter, policy)?;
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
        matching_nurbs_derivatives(first_derivatives, last_derivatives, policy)
    }

    fn derivatives_on_span(
        &self,
        span_index: usize,
        parameter: &Real,
        max_order: usize,
        location: NurbsSpanParameterLocation,
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let decomposition =
            self.bezier_decomposition_for_operation(policy, CurveOperation2::Evaluation)?;
        let span = &decomposition.spans()[span_index];
        let local = match location {
            NurbsSpanParameterLocation::Start => Real::zero(),
            NurbsSpanParameterLocation::End => Real::one(),
            NurbsSpanParameterLocation::Interior => local_span_parameter(span, parameter)?,
        };
        let rational_span =
            &self.rational_spans_for_operation(policy, CurveOperation2::Evaluation)?[span_index];
        let local_derivatives = if max_order == 1 {
            vec![exact_classification(
                rational_span.derivative_at_classified(&local, policy),
                CurveOperation2::Evaluation,
            )?]
        } else {
            exact_classification(
                rational_span.derivatives_at_classified(&local, max_order, policy),
                CurveOperation2::Evaluation,
            )?
        };
        let (start, end) = span.knot_interval();
        let inverse_width = (Real::one() / (end - start)).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                CurveFamily2::Nurbs,
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
                    .map(|span| {
                        RationalBezier2::try_new(
                            span.control_points().to_vec(),
                            span.weights().to_vec(),
                        )
                        .map_err(|cause| {
                            ExactCurveError::invalid(
                                CurveOperation2::NativeTopology,
                                CurveFamily2::Nurbs,
                                cause,
                            )
                        })
                    })
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
                .map_err(|error| remap_nurbs_operation(error, operation))?,
            operation,
        )
    }

    fn validate_periodic_seam(&self, policy: &CurveContext) -> ExactCurveResult<()> {
        if !self.periodicity().is_periodic() {
            return Ok(());
        }
        match (
            crate::classify::compare_reals(self.start().x(), self.end().x(), policy),
            crate::classify::compare_reals(self.start().y(), self.end().y(), policy),
        ) {
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(()),
            (Some(_), Some(_)) => Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::Nurbs,
                CurveError::PeriodicSplineSeamMismatch,
            )),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Construction,
                CurveFamily2::Nurbs,
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
            (Some(std::cmp::Ordering::Equal), _) | (_, Some(std::cmp::Ordering::Equal)) => Ok(true),
            (Some(_), Some(_)) => Ok(false),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::Nurbs,
                UncertaintyReason::Ordering,
            )),
        }
    }
}

impl PartialEq for NurbsCurve2 {
    fn eq(&self, other: &Self) -> bool {
        self.data.retained == other.data.retained
    }
}

impl NurbsBezierDecomposition2 {
    /// Returns the retained NURBS degree.
    pub const fn degree(&self) -> usize {
        self.extraction.degree()
    }

    /// Returns the exact refined affine control net after knot insertion.
    pub fn refined_control_points(&self) -> &[Point2] {
        self.extraction.refined_control_points()
    }

    /// Returns the exact refined weights after knot insertion.
    pub fn refined_weights(&self) -> &[Real] {
        self.extraction.refined_weights()
    }

    /// Returns the exact refined knot vector after knot insertion.
    pub fn refined_knots(&self) -> &[Real] {
        self.extraction.refined_knots()
    }

    /// Returns retained rational Bezier spans in source-parameter order.
    pub fn spans(&self) -> &[RationalBezierSpan2] {
        self.extraction.spans()
    }

    /// Returns how many exact knot insertions produced Bezier form.
    pub const fn inserted_knot_count(&self) -> usize {
        self.extraction.inserted_knot_count()
    }
}

impl<'a> NurbsBezierSpanView2<'a> {
    /// Returns this span's stable index in source-parameter order.
    pub const fn span_index(self) -> usize {
        self.span_index
    }

    /// Returns the retained rational Bezier degree.
    pub const fn degree(self) -> usize {
        self.span.degree()
    }

    /// Returns exact affine controls for this rational span.
    pub fn control_points(self) -> &'a [Point2] {
        self.span.control_points()
    }

    /// Returns exact homogeneous weights for this rational span.
    pub fn weights(self) -> &'a [Real] {
        self.span.weights()
    }

    /// Returns the exact source knot interval.
    pub fn knot_interval(self) -> (&'a Real, &'a Real) {
        self.span.knot_interval()
    }

    /// Returns the retained low-level rational span evidence.
    pub const fn retained_span(self) -> &'a RationalBezierSpan2 {
        self.span
    }
}

impl<'a> NurbsNativeSpanView2<'a> {
    /// Returns the NURBS span from which the native curve was promoted.
    pub const fn source_span(self) -> NurbsBezierSpanView2<'a> {
        self.source_span
    }

    /// Returns the exact promoted native Bezier/conic curve.
    pub const fn curve(self) -> &'a BezierSubcurve2 {
        self.curve
    }
}

impl NurbsDegreeElevation2 {
    /// Returns the source NURBS degree.
    pub const fn source_degree(&self) -> usize {
        self.source_degree
    }

    /// Returns the exact elevated degree shared by every span.
    pub const fn target_degree(&self) -> usize {
        self.target_degree
    }

    /// Returns elevated spans in source knot order.
    pub fn spans(&self) -> &[NurbsElevatedBezierSpan2] {
        &self.spans
    }
}

impl NurbsElevatedBezierSpan2 {
    /// Returns the stable source span index.
    pub const fn span_index(&self) -> usize {
        self.span_index
    }

    /// Returns the exact source knot interval.
    pub fn parameter_interval(&self) -> (&Real, &Real) {
        (&self.parameter_start, &self.parameter_end)
    }

    /// Returns the exact elevated rational Bezier curve on local `[0, 1]`.
    pub const fn curve(&self) -> &RationalBezier2 {
        &self.curve
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
            CurveFamily2::Nurbs,
            reason,
        )),
        Err(cause) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::Nurbs,
            cause,
        )),
    }
}

fn map_classified_curve_result<T>(
    result: crate::CurveResult<Classification<T>>,
    operation: CurveOperation2,
) -> ExactCurveResult<Classification<T>> {
    result.map_err(|cause| ExactCurveError::invalid(operation, CurveFamily2::Nurbs, cause))
}

fn require_classification<T>(
    classification: Classification<T>,
    operation: CurveOperation2,
) -> ExactCurveResult<T> {
    match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            reason,
        )),
    }
}

fn remap_degree_elevation_error(error: ExactCurveError) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { cause, .. } => {
            ExactCurveError::invalid(CurveOperation2::DegreeElevation, CurveFamily2::Nurbs, cause)
        }
        ExactCurveError::Blocked(blocker) => ExactCurveError::blocked(
            CurveOperation2::DegreeElevation,
            CurveFamily2::Nurbs,
            blocker.reason(),
        ),
    }
}

fn validate_strict_interior_parameter(
    curve: &NurbsCurve2,
    parameter: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    validate_strict_interior(curve, parameter, CurveOperation2::Subdivision, policy)
}

fn validate_strict_interior(
    curve: &NurbsCurve2,
    parameter: &Real,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let (start, end) = curve.parameter_domain();
    match (
        crate::classify::compare_reals(start, parameter, policy),
        crate::classify::compare_reals(parameter, end, policy),
    ) {
        (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Less)) => Ok(()),
        (Some(_), Some(_)) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::Nurbs,
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            UncertaintyReason::Ordering,
        )),
    }
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
            knots.last().expect("validated NURBS has knots"),
            &knots[control_count],
            policy,
        ),
    ) {
        (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(true),
        (Some(_), Some(_)) => Ok(false),
        _ => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            UncertaintyReason::Ordering,
        )),
    }
}

fn validate_subcurve_range(
    curve: &NurbsCurve2,
    start: &Real,
    end: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let (domain_start, domain_end) = curve.parameter_domain();
    match (
        crate::classify::compare_reals(domain_start, start, policy),
        crate::classify::compare_reals(start, end, policy),
        crate::classify::compare_reals(end, domain_end, policy),
    ) {
        (
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            Some(std::cmp::Ordering::Less),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
        ) => Ok(()),
        (Some(_), Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            CurveFamily2::Nurbs,
            CurveError::InvalidCurveRange,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            CurveFamily2::Nurbs,
            UncertaintyReason::Ordering,
        )),
    }
}

fn exact_nurbs_knot_multiplicity(
    knots: &[Real],
    knot: &Real,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<usize> {
    let mut multiplicity = 0;
    for candidate in knots {
        match crate::classify::compare_reals(candidate, knot, policy) {
            Some(std::cmp::Ordering::Equal) => multiplicity += 1,
            Some(_) => {}
            None => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::Nurbs,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    Ok(multiplicity)
}

fn exact_points_equal(
    first: &Point2,
    second: &Point2,
    operation: CurveOperation2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    match (
        crate::classify::compare_reals(first.x(), second.x(), policy),
        crate::classify::compare_reals(first.y(), second.y(), policy),
    ) {
        (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(()),
        (Some(_), Some(_)) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::Nurbs,
            CurveError::InvalidDegreeElevation,
        )),
        _ => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            UncertaintyReason::RealSign,
        )),
    }
}

fn remap_nurbs_operation(error: ExactCurveError, operation: CurveOperation2) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { family, cause, .. } => {
            ExactCurveError::invalid(operation, family, cause)
        }
        ExactCurveError::Blocked(blocker) => {
            ExactCurveError::blocked(operation, blocker.family(), blocker.reason())
        }
    }
}

#[derive(Clone, Copy)]
struct SelectedNurbsSpan {
    index: usize,
    location: NurbsSpanParameterLocation,
}

#[derive(Clone, Copy)]
enum NurbsSpanParameterLocation {
    Start,
    Interior,
    End,
}

fn select_span_indices(
    spans: &[RationalBezierSpan2],
    parameter: &Real,
    policy: &CurveContext,
) -> ExactCurveResult<(SelectedNurbsSpan, SelectedNurbsSpan)> {
    let mut first = None;
    let mut last = None;
    for (span_index, span) in spans.iter().enumerate() {
        let (start, end) = span.knot_interval();
        let lower = crate::classify::compare_reals(start, parameter, policy);
        let upper = crate::classify::compare_reals(parameter, end, policy);
        match (lower, upper) {
            (
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => {
                let selected = SelectedNurbsSpan {
                    index: span_index,
                    location: if lower == Some(std::cmp::Ordering::Equal) {
                        NurbsSpanParameterLocation::Start
                    } else if upper == Some(std::cmp::Ordering::Equal) {
                        NurbsSpanParameterLocation::End
                    } else {
                        NurbsSpanParameterLocation::Interior
                    },
                };
                first.get_or_insert(selected);
                last = Some(selected);
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::Nurbs,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    first.zip(last).ok_or_else(|| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            CurveError::InvalidCurveParameter,
        )
    })
}

fn matching_nurbs_derivatives(
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
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => {}
            (Some(_), Some(_)) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::Nurbs,
                    UncertaintyReason::Boundary,
                ));
            }
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::Nurbs,
                    UncertaintyReason::RealSign,
                ));
            }
        }
    }
    Ok(first)
}

fn matching_nurbs_point(
    first: Point2,
    second: Point2,
    policy: &CurveContext,
) -> ExactCurveResult<Point2> {
    match (
        crate::classify::compare_reals(first.x(), second.x(), policy),
        crate::classify::compare_reals(first.y(), second.y(), policy),
    ) {
        (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(first),
        (Some(_), Some(_)) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            UncertaintyReason::Boundary,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            UncertaintyReason::RealSign,
        )),
    }
}

fn local_span_parameter(span: &RationalBezierSpan2, parameter: &Real) -> ExactCurveResult<Real> {
    let (start, end) = span.knot_interval();
    let width = end - start;
    ((parameter - start) / width).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            cause.into(),
        )
    })
}

fn exact_classification<T>(
    classification: Classification<T>,
    operation: CurveOperation2,
) -> ExactCurveResult<T> {
    match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            reason,
        )),
    }
}

#[cfg(all(test, target_pointer_width = "64"))]
mod layout_tests {
    use super::{NurbsCurve2, NurbsData2};

    #[test]
    fn nurbs_carrier_keeps_compact_policy_aware_storage() {
        assert_eq!(core::mem::size_of::<NurbsCurve2>(), 8);
        assert_eq!(core::mem::size_of::<NurbsData2>(), 512);
    }
}
