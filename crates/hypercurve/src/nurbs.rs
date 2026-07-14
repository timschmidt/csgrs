//! Policy-free retained NURBS carrier with shared exact decomposition caches.

use std::cell::{OnceCell, RefCell};
use std::rc::Rc;

use crate::spline_periodic::{expand_periodic_spline, wrap_periodic_parameter};
use crate::{
    BezierSubcurve2, Classification, CurveDerivative2, CurveError, CurveFamily2, CurveOperation2,
    CurveParameterSide2, CurvePolicy, CurveSource2, ExactCurveError, ExactCurveResult, Point2,
    RationalBSplineBezierExtraction2, RationalBSplineCurve2, RationalBezier2, RationalBezierSpan2,
    Real, Similarity2, SplinePeriodicity2, UncertaintyReason,
};

type Cached<T> = Result<T, ExactCurveError>;
const MAX_RETAINED_KNOT_REFINEMENTS: usize = 8;
const MAX_RETAINED_KNOT_REMOVALS: usize = 8;
const MAX_RETAINED_DEGREE_ELEVATIONS: usize = 8;

#[derive(Debug)]
struct NurbsData2 {
    retained: RationalBSplineCurve2,
    source: Option<CurveSource2>,
    endpoints: NurbsEndpoints2,
    decomposition: OnceCell<Cached<NurbsBezierDecomposition2>>,
    native_subcurves: OnceCell<Cached<Vec<BezierSubcurve2>>>,
    rational_spans: OnceCell<Cached<Vec<RationalBezier2>>>,
    knot_refinements: OnceCell<RefCell<Vec<(Vec<Real>, Cached<NurbsCurve2>)>>>,
    knot_removals: OnceCell<RefCell<Vec<(Real, Cached<Option<NurbsCurve2>>)>>>,
    degree_elevations: OnceCell<RefCell<Vec<(usize, Cached<NurbsDegreeElevation2>)>>>,
    elevated_curves: OnceCell<RefCell<Vec<(usize, Cached<NurbsCurve2>)>>>,
}

#[derive(Debug)]
enum NurbsEndpoints2 {
    AuthoredControls,
    Extracted { start: Point2, end: Point2 },
}

/// Exact rational B-spline/NURBS curve with retained source identity.
///
/// Clones share the same immutable source carrier and lazy exact caches. The
/// homogeneous Boehm decomposition and native-topology promotion therefore run
/// at most once for one curve object, including when the result is a blocker.
#[derive(Clone, Debug)]
pub struct NurbsCurve2 {
    data: Rc<NurbsData2>,
}

/// Exact homogeneous Bezier decomposition retained by a [`NurbsCurve2`].
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsBezierDecomposition2 {
    extraction: RationalBSplineBezierExtraction2,
}

/// Borrowed exact NURBS Bezier span with source provenance.
#[derive(Clone, Copy, Debug)]
pub struct NurbsBezierSpanView2<'a> {
    span_index: usize,
    source: Option<CurveSource2>,
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
    source: Option<CurveSource2>,
    spans: Rc<[NurbsElevatedBezierSpan2]>,
}

/// One exact elevated rational Bezier span with NURBS source provenance.
#[derive(Clone, Debug, PartialEq)]
pub struct NurbsElevatedBezierSpan2 {
    span_index: usize,
    parameter_start: Real,
    parameter_end: Real,
    curve: RationalBezier2,
}

impl NurbsCurve2 {
    /// Constructs a degree-one-or-higher NURBS curve over its active knot domain.
    pub fn try_new(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_source(degree, control_points, weights, knots, None)
    }

    /// Constructs a NURBS curve with stable source provenance.
    pub fn try_new_with_source(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        source: CurveSource2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_source(degree, control_points, weights, knots, Some(source))
    }

    /// Constructs a periodic NURBS from one period of controls and knot breaks.
    ///
    /// `period_knots` must contain exactly one more entry than the unique
    /// control count. Hypercurve extends the cyclic control and knot sequences
    /// exactly and certifies closure at the canonical seam.
    pub fn try_new_periodic(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        period_knots: Vec<Real>,
    ) -> ExactCurveResult<Self> {
        Self::try_new_periodic_with_optional_source(
            degree,
            control_points,
            weights,
            period_knots,
            None,
        )
    }

    /// Constructs a periodic NURBS with stable source provenance.
    pub fn try_new_periodic_with_source(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        period_knots: Vec<Real>,
        source: CurveSource2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_periodic_with_optional_source(
            degree,
            control_points,
            weights,
            period_knots,
            Some(source),
        )
    }

    fn try_new_periodic_with_optional_source(
        degree: usize,
        control_points: Vec<Point2>,
        mut weights: Vec<Real>,
        period_knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        if weights.len() != control_points.len() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::Nurbs,
                source,
                CurveError::InvalidPeriodicSpline,
            ));
        }
        let expansion = expand_periodic_spline(
            degree,
            control_points,
            period_knots,
            CurveFamily2::Nurbs,
            source,
        )?;
        weights.extend_from_within(..degree);
        Self::try_new_expanded(
            degree,
            expansion.control_points,
            weights,
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
        weights: Vec<Real>,
        knots: Vec<Real>,
        source: Option<CurveSource2>,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded(
            degree,
            control_points,
            weights,
            knots,
            source,
            SplinePeriodicity2::NonPeriodic,
        )
    }

    fn try_new_expanded(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
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
                source,
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
                &CurvePolicy::certified(),
            ),
            CurveOperation2::Construction,
            source,
        )?;
        Self::from_retained(retained, source, None)
    }

    pub(crate) fn try_new_expanded_with_periodicity(
        degree: usize,
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        knots: Vec<Real>,
        source: Option<CurveSource2>,
        periodicity: SplinePeriodicity2,
    ) -> ExactCurveResult<Self> {
        Self::try_new_expanded(degree, control_points, weights, knots, source, periodicity)
    }

    fn from_retained(
        retained: RationalBSplineCurve2,
        source: Option<CurveSource2>,
        preserved_endpoints: Option<(Point2, Point2)>,
    ) -> ExactCurveResult<Self> {
        let decomposition = OnceCell::new();
        let endpoints = if let Some((start, end)) = preserved_endpoints {
            NurbsEndpoints2::Extracted { start, end }
        } else if has_clamped_endpoints(
            retained.knots(),
            retained.degree(),
            retained.control_points().len(),
            source,
        )? {
            NurbsEndpoints2::AuthoredControls
        } else {
            let extraction = exact_value(
                retained.extract_bezier_spans(&CurvePolicy::certified()),
                CurveOperation2::Construction,
                source,
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
            decomposition
                .set(Ok(NurbsBezierDecomposition2 { extraction }))
                .expect("new decomposition cache is empty");
            NurbsEndpoints2::Extracted { start, end }
        };
        let curve = Self {
            data: Rc::new(NurbsData2 {
                retained,
                source,
                endpoints,
                decomposition,
                native_subcurves: OnceCell::new(),
                rational_spans: OnceCell::new(),
                knot_refinements: OnceCell::new(),
                knot_removals: OnceCell::new(),
                degree_elevations: OnceCell::new(),
                elevated_curves: OnceCell::new(),
            }),
        };
        curve.validate_periodic_seam()?;
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
    /// The curve image and authored source identity are preserved. If an
    /// interior knot already has full Bezier multiplicity, this returns a clone
    /// sharing the original carrier and caches.
    pub fn insert_knot(&self, knot: Real) -> ExactCurveResult<Self> {
        self.insert_knots(vec![knot])
    }

    /// Inserts an ordered batch of exact knots in one homogeneous refinement pass.
    ///
    /// The working control net is projected and validated only once. Exact
    /// source/version evidence, periodicity, endpoints, and parameterization
    /// are preserved. Repeated equal requests from any clone reuse a bounded
    /// retained result.
    pub fn insert_knots(&self, knots: Vec<Real>) -> ExactCurveResult<Self> {
        if knots.is_empty() {
            return Ok(self.clone());
        }
        let refinements = self
            .data
            .knot_refinements
            .get_or_init(|| RefCell::new(Vec::new()));
        if let Some((_, result)) = refinements
            .borrow()
            .iter()
            .find(|(retained_knots, _)| retained_knots == &knots)
        {
            return result.clone();
        }
        let result = self.insert_knots_uncached(knots.clone());
        let mut refinements = refinements.borrow_mut();
        if refinements.len() == MAX_RETAINED_KNOT_REFINEMENTS {
            let _ = refinements.remove(0);
        }
        refinements.push((knots, result.clone()));
        result
    }

    /// Returns whether this exact ordered refinement request is retained.
    pub fn is_knot_refinement_cached(&self, knots: &[Real]) -> bool {
        self.data.knot_refinements.get().is_some_and(|refinements| {
            refinements
                .borrow()
                .iter()
                .any(|(retained_knots, _)| retained_knots == knots)
        })
    }

    /// Removes one exact interior knot occurrence when that preserves the curve.
    ///
    /// Removal is certified as the inverse of homogeneous Boehm insertion: the
    /// candidate control net is solved exactly, reinserted, and compared with
    /// every authored homogeneous control and knot. `None` means the requested
    /// knot is absent or is not exactly removable. Results are retained across
    /// clones, including negative results and blockers.
    pub fn remove_knot(&self, knot: Real) -> ExactCurveResult<Option<Self>> {
        validate_strict_interior_knot(self, &knot)?;
        let removals = self
            .data
            .knot_removals
            .get_or_init(|| RefCell::new(Vec::new()));
        if let Some((_, result)) = removals
            .borrow()
            .iter()
            .find(|(retained_knot, _)| retained_knot == &knot)
        {
            return result.clone();
        }
        let result = self.remove_knot_uncached(knot.clone());
        let mut removals = removals.borrow_mut();
        if removals.len() == MAX_RETAINED_KNOT_REMOVALS {
            let _ = removals.remove(0);
        }
        removals.push((knot, result.clone()));
        result
    }

    /// Returns whether this exact knot-removal result or blocker is retained.
    pub fn is_knot_removal_cached(&self, knot: &Real) -> bool {
        self.data.knot_removals.get().is_some_and(|removals| {
            removals
                .borrow()
                .iter()
                .any(|(retained_knot, _)| retained_knot == knot)
        })
    }

    /// Elevates every exact rational Bezier knot span to `target_degree`.
    ///
    /// The result retains source knot intervals and source/version identity,
    /// so callers can consume elevated homogeneous spans without changing the
    /// NURBS parameterization or inventing a less-continuous replacement knot
    /// vector. Equal requests and blockers are retained across clones.
    pub fn degree_elevation(
        &self,
        target_degree: usize,
    ) -> ExactCurveResult<NurbsDegreeElevation2> {
        if target_degree < self.degree() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::Nurbs,
                self.data.source,
                CurveError::InvalidDegreeElevation,
            ));
        }
        let elevations = self
            .data
            .degree_elevations
            .get_or_init(|| RefCell::new(Vec::new()));
        if let Some((_, result)) = elevations
            .borrow()
            .iter()
            .find(|(retained_degree, _)| *retained_degree == target_degree)
        {
            return result.clone();
        }
        let result = self.degree_elevation_uncached(target_degree);
        let mut elevations = elevations.borrow_mut();
        if elevations.len() == MAX_RETAINED_DEGREE_ELEVATIONS {
            let _ = elevations.remove(0);
        }
        elevations.push((target_degree, result.clone()));
        result
    }

    /// Returns whether this exact target degree has retained spans or a blocker.
    pub fn is_degree_elevation_cached(&self, target_degree: usize) -> bool {
        self.data.degree_elevations.get().is_some_and(|elevations| {
            elevations
                .borrow()
                .iter()
                .any(|(retained_degree, _)| *retained_degree == target_degree)
        })
    }

    /// Returns an exact NURBS carrier elevated to `target_degree`.
    ///
    /// Every rational Bezier span is elevated homogeneously, adjacent span
    /// scales are aligned, and inverse knot insertion removes the extraction
    /// knots needed to restore the source continuity order. The resulting
    /// NURBS preserves the authored parameter domain, periodicity, source, and
    /// parameterized image. Equal requests and blockers are retained across
    /// clones.
    pub fn elevated_to_degree(&self, target_degree: usize) -> ExactCurveResult<Self> {
        if target_degree < self.degree() {
            return Err(ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::Nurbs,
                self.data.source,
                CurveError::InvalidDegreeElevation,
            ));
        }
        if target_degree == self.degree() {
            return Ok(self.clone());
        }
        let elevated_curves = self
            .data
            .elevated_curves
            .get_or_init(|| RefCell::new(Vec::new()));
        if let Some((_, result)) = elevated_curves
            .borrow()
            .iter()
            .find(|(retained_degree, _)| *retained_degree == target_degree)
        {
            return result.clone();
        }
        let result = self.elevated_to_degree_uncached(target_degree);
        let mut elevated_curves = elevated_curves.borrow_mut();
        if elevated_curves.len() == MAX_RETAINED_DEGREE_ELEVATIONS {
            let _ = elevated_curves.remove(0);
        }
        elevated_curves.push((target_degree, result.clone()));
        result
    }

    /// Returns whether this exact elevated carrier or blocker is retained.
    pub fn is_elevated_curve_cached(&self, target_degree: usize) -> bool {
        self.data.elevated_curves.get().is_some_and(|elevations| {
            elevations
                .borrow()
                .iter()
                .any(|(retained_degree, _)| *retained_degree == target_degree)
        })
    }

    fn degree_elevation_uncached(
        &self,
        target_degree: usize,
    ) -> ExactCurveResult<NurbsDegreeElevation2> {
        let decomposition = self.bezier_decomposition()?;
        let rational_spans = self.rational_spans()?;
        let spans = decomposition
            .spans()
            .iter()
            .zip(rational_spans)
            .enumerate()
            .map(|(span_index, (source_span, rational_span))| {
                let curve = rational_span
                    .elevated_to_degree(target_degree)
                    .map_err(|error| remap_degree_elevation_error(error, self.data.source))?;
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
            source: self.data.source,
            spans: spans.into(),
        })
    }

    fn elevated_to_degree_uncached(&self, target_degree: usize) -> ExactCurveResult<Self> {
        let elevation = self.degree_elevation(target_degree)?;
        let (mut elevated, removable_knots) =
            self.piecewise_elevated_curve(&elevation, target_degree)?;
        for (knot, removal_count) in removable_knots {
            for _ in 0..removal_count {
                elevated = elevated.remove_knot(knot.clone())?.ok_or_else(|| {
                    ExactCurveError::invalid(
                        CurveOperation2::DegreeElevation,
                        CurveFamily2::Nurbs,
                        self.data.source,
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
                self.data.source,
                CurveOperation2::DegreeElevation,
            )?;
            if multiplicity <= self.degree() {
                exact_points_equal(
                    spans[span_index - 1].curve().end(),
                    spans[span_index].curve().start(),
                    self.data.source,
                    CurveOperation2::DegreeElevation,
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
                        self.data.source,
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
        let curve = Self::try_new_expanded(
            target_degree,
            control_points,
            weights,
            knots,
            self.data.source,
            self.periodicity().clone(),
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::DegreeElevation))?;
        Ok((curve, removable_knots))
    }

    fn insert_knots_uncached(&self, knots: Vec<Real>) -> ExactCurveResult<Self> {
        let (retained, inserted_count) = exact_value(
            self.data
                .retained
                .insert_knots(knots, &CurvePolicy::certified()),
            CurveOperation2::KnotInsertion,
            self.data.source,
        )?;
        if inserted_count == 0 {
            return Ok(self.clone());
        }
        Self::from_retained(
            retained,
            self.data.source,
            Some((self.start().clone(), self.end().clone())),
        )
    }

    fn remove_knot_uncached(&self, knot: Real) -> ExactCurveResult<Option<Self>> {
        let retained = exact_value(
            self.data
                .retained
                .remove_knot(knot, &CurvePolicy::certified()),
            CurveOperation2::KnotRemoval,
            self.data.source,
        )?;
        retained
            .map(|retained| {
                Self::from_retained(
                    retained,
                    self.data.source,
                    Some((self.start().clone(), self.end().clone())),
                )
                .map_err(|error| remap_nurbs_operation(error, CurveOperation2::KnotRemoval))
            })
            .transpose()
    }

    /// Splits this NURBS exactly at a strict interior knot-domain parameter.
    pub fn split_at(&self, parameter: Real) -> ExactCurveResult<(Self, Self)> {
        validate_strict_interior_parameter(self, &parameter)?;
        let refined = self.insert_knots(vec![parameter.clone(); self.degree()])?;
        let policy = CurvePolicy::certified();
        let equal_indices = refined
            .knots()
            .iter()
            .enumerate()
            .filter_map(|(index, knot)| {
                (crate::classify::compare_reals(knot, &parameter, &policy)
                    == Some(std::cmp::Ordering::Equal))
                .then_some(index)
            })
            .collect::<Vec<_>>();
        if !matches!(equal_indices.len(), count if count == self.degree() || count == self.degree() + 1)
        {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Subdivision,
                CurveFamily2::Nurbs,
                self.data.source,
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
        let left = Self::try_new_with_optional_source(
            self.degree(),
            refined.control_points()[..=left_end].to_vec(),
            refined.weights()[..=left_end].to_vec(),
            left_knots,
            self.data.source,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Subdivision))?;
        let right = Self::try_new_with_optional_source(
            self.degree(),
            refined.control_points()[right_start..].to_vec(),
            refined.weights()[right_start..].to_vec(),
            right_knots,
            self.data.source,
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Subdivision))?;
        Ok((left, right))
    }

    /// Returns an exact NURBS subcurve over an ordered source-parameter range.
    pub fn subcurve(&self, start: Real, end: Real) -> ExactCurveResult<Self> {
        validate_subcurve_range(self, &start, &end)?;
        let (domain_start, domain_end) = self.parameter_domain();
        let policy = CurvePolicy::certified();
        let starts_at_domain = crate::classify::compare_reals(&start, domain_start, &policy)
            == Some(std::cmp::Ordering::Equal);
        let ends_at_domain = crate::classify::compare_reals(&end, domain_end, &policy)
            == Some(std::cmp::Ordering::Equal);
        if starts_at_domain && ends_at_domain {
            return Ok(self.clone());
        }
        let through_end = if ends_at_domain {
            self.clone()
        } else {
            self.split_at(end)?.0
        };
        if starts_at_domain {
            Ok(through_end)
        } else {
            Ok(through_end.split_at(start)?.1)
        }
    }

    /// Returns the same NURBS image with traversal direction reversed.
    ///
    /// Controls and weights are reversed, while knots are reflected through
    /// the authored domain midpoint. The source parameter domain and source
    /// provenance are preserved exactly.
    pub fn reversed(&self) -> ExactCurveResult<Self> {
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
        Self::try_new_expanded(
            self.degree(),
            control_points,
            weights,
            knots,
            self.data.source,
            self.periodicity().clone(),
        )
        .map_err(|error| remap_nurbs_operation(error, CurveOperation2::Reversal))
    }

    /// Applies an exact planar similarity while retaining periodicity and source.
    pub fn transform_similarity(&self, transform: &Similarity2) -> ExactCurveResult<Self> {
        Self::try_new_expanded(
            self.degree(),
            self.control_points()
                .iter()
                .map(|point| transform.transform_point(point))
                .collect(),
            self.weights().to_vec(),
            self.knots().to_vec(),
            self.data.source,
            self.periodicity().clone(),
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

    /// Returns retained source identity when supplied by the caller.
    pub fn source(&self) -> Option<CurveSource2> {
        self.data.source
    }

    /// Returns whether exact Bezier decomposition has already been retained.
    pub fn is_bezier_decomposition_cached(&self) -> bool {
        self.data.decomposition.get().is_some()
    }

    /// Returns whether reusable general-rational span evaluators are retained.
    pub fn is_rational_span_cache_cached(&self) -> bool {
        self.data.rational_spans.get().is_some()
    }

    /// Returns the shared exact homogeneous Bezier decomposition.
    pub fn bezier_decomposition(&self) -> ExactCurveResult<&NurbsBezierDecomposition2> {
        cached_result(&self.data.decomposition, || {
            exact_value(
                self.data
                    .retained
                    .extract_bezier_spans(&CurvePolicy::certified()),
                CurveOperation2::BezierDecomposition,
                self.data.source,
            )
            .map(|extraction| NurbsBezierDecomposition2 { extraction })
        })
    }

    /// Iterates exact retained Bezier spans with source identity and intervals.
    pub fn bezier_spans(
        &self,
    ) -> ExactCurveResult<impl ExactSizeIterator<Item = NurbsBezierSpanView2<'_>>> {
        let source = self.data.source;
        Ok(self.bezier_decomposition()?.spans().iter().enumerate().map(
            move |(span_index, span)| NurbsBezierSpanView2 {
                span_index,
                source,
                span,
            },
        ))
    }

    /// Returns native conic/polynomial Bezier spans when every span supports them.
    ///
    /// Linear rational spans are elevated exactly in homogeneous coordinates,
    /// quadratics use native conics, equal-weight cubics collapse to polynomial
    /// cubics, and all remaining spans use exact general rational Beziers.
    pub fn native_subcurves(&self) -> ExactCurveResult<&[BezierSubcurve2]> {
        let subcurves = cached_result(&self.data.native_subcurves, || {
            let decomposition = self.bezier_decomposition()?;
            exact_value(
                decomposition
                    .extraction
                    .native_subcurves(&CurvePolicy::certified()),
                CurveOperation2::NativeTopology,
                self.data.source,
            )
        })?;
        Ok(subcurves)
    }

    /// Iterates native promoted spans without losing their rational source span.
    pub fn native_spans(
        &self,
    ) -> ExactCurveResult<impl ExactSizeIterator<Item = NurbsNativeSpanView2<'_>>> {
        let source = self.data.source;
        let decomposition = self.bezier_decomposition()?;
        let native = self.native_subcurves()?;
        debug_assert_eq!(decomposition.spans().len(), native.len());
        Ok(decomposition.spans().iter().zip(native).enumerate().map(
            move |(span_index, (span, curve))| NurbsNativeSpanView2 {
                source_span: NurbsBezierSpanView2 {
                    span_index,
                    source,
                    span,
                },
                curve,
            },
        ))
    }

    /// Evaluates the NURBS at an exact source-domain parameter.
    ///
    /// The exact homogeneous Bezier decomposition is retained on first use.
    /// Evaluation then selects the source knot span and applies homogeneous de
    /// Casteljau interpolation without finite projection.
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
            return matching_nurbs_point(left, right, self.data.source);
        }
        self.point_at_canonical_side(parameter, side)
    }

    /// Evaluates a periodic NURBS at any exactly wrappable parameter.
    pub fn point_at_wrapped(&self, parameter: &Real) -> ExactCurveResult<Point2> {
        self.point_at_wrapped_side(parameter, CurveParameterSide2::Automatic)
    }

    /// Evaluates a periodic NURBS with explicit side selection at wrapped seams.
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
            CurveFamily2::Nurbs,
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
            select_span_indices(decomposition.spans(), parameter, self.data.source)?;
        let first_point = self.point_on_span(first, parameter)?;
        if first == last || side == CurveParameterSide2::Left {
            return Ok(first_point);
        }
        let last_point = self.point_on_span(last, parameter)?;
        if side == CurveParameterSide2::Right {
            return Ok(last_point);
        }
        matching_nurbs_point(first_point, last_point, self.data.source)
    }

    fn point_on_span(&self, span_index: usize, parameter: &Real) -> ExactCurveResult<Point2> {
        let decomposition = self.bezier_decomposition()?;
        let local = local_span_parameter(
            &decomposition.spans()[span_index],
            parameter,
            self.data.source,
        )?;
        let curve = &self.rational_spans()?[span_index];
        exact_classification(
            curve.point_at_classified(&local, &CurvePolicy::certified()),
            CurveOperation2::Evaluation,
            self.data.source,
        )
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
    ///
    /// The returned vector stores orders `1..=max_order`. Each local rational
    /// Bezier derivative is scaled by the corresponding power of the inverse
    /// source-span width, preserving the authored NURBS parameterization.
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
            return matching_nurbs_derivatives(left, right, self.data.source);
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
            CurveFamily2::Nurbs,
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
            select_span_indices(decomposition.spans(), parameter, self.data.source)?;
        let first_derivatives = self.derivatives_on_span(first, parameter, max_order)?;
        if first == last || side == CurveParameterSide2::Left {
            return Ok(first_derivatives);
        }
        let last_derivatives = self.derivatives_on_span(last, parameter, max_order)?;
        if side == CurveParameterSide2::Right {
            return Ok(last_derivatives);
        }
        matching_nurbs_derivatives(first_derivatives, last_derivatives, self.data.source)
    }

    fn derivatives_on_span(
        &self,
        span_index: usize,
        parameter: &Real,
        max_order: usize,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        let decomposition = self.bezier_decomposition()?;
        let span = &decomposition.spans()[span_index];
        let local = local_span_parameter(span, parameter, self.data.source)?;
        let rational_span = &self.rational_spans()?[span_index];
        let local_derivatives = if max_order == 1 {
            vec![exact_classification(
                rational_span.derivative_at_classified(&local, &CurvePolicy::certified()),
                CurveOperation2::Evaluation,
                self.data.source,
            )?]
        } else {
            exact_classification(
                rational_span.derivatives_at_classified(
                    &local,
                    max_order,
                    &CurvePolicy::certified(),
                ),
                CurveOperation2::Evaluation,
                self.data.source,
            )?
        };
        let (start, end) = span.knot_interval();
        let inverse_width = (Real::one() / (end - start)).map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Evaluation,
                CurveFamily2::Nurbs,
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
                .map(|span| {
                    RationalBezier2::try_new(
                        span.control_points().to_vec(),
                        span.weights().to_vec(),
                    )
                    .map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::NativeTopology,
                            CurveFamily2::Nurbs,
                            self.data.source,
                            cause,
                        )
                    })
                })
                .collect()
        })?;
        Ok(spans)
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
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(()),
            (Some(_), Some(_)) => Err(ExactCurveError::invalid(
                CurveOperation2::Construction,
                CurveFamily2::Nurbs,
                self.data.source,
                CurveError::PeriodicSplineSeamMismatch,
            )),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Construction,
                CurveFamily2::Nurbs,
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
            (Some(std::cmp::Ordering::Equal), _) | (_, Some(std::cmp::Ordering::Equal)) => Ok(true),
            (Some(_), Some(_)) => Ok(false),
            _ => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::Nurbs,
                self.data.source,
                UncertaintyReason::Ordering,
            )),
        }
    }
}

impl PartialEq for NurbsCurve2 {
    fn eq(&self, other: &Self) -> bool {
        self.data.retained == other.data.retained && self.data.source == other.data.source
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

    /// Returns the owning curve source identity, when supplied.
    pub const fn source(self) -> Option<CurveSource2> {
        self.source
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
    /// Returns source NURBS span provenance for the promoted curve.
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

    /// Returns stable source identity retained from the NURBS carrier.
    pub const fn source(&self) -> Option<CurveSource2> {
        self.source
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
            CurveFamily2::Nurbs,
            source,
            reason,
        )),
        Err(cause) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::Nurbs,
            source,
            cause,
        )),
    }
}

fn remap_degree_elevation_error(
    error: ExactCurveError,
    source: Option<CurveSource2>,
) -> ExactCurveError {
    match error {
        ExactCurveError::Invalid { cause, .. } => ExactCurveError::invalid(
            CurveOperation2::DegreeElevation,
            CurveFamily2::Nurbs,
            source,
            cause,
        ),
        ExactCurveError::Blocked(blocker) => ExactCurveError::blocked(
            CurveOperation2::DegreeElevation,
            CurveFamily2::Nurbs,
            source,
            blocker.reason(),
        ),
    }
}

fn validate_strict_interior_parameter(
    curve: &NurbsCurve2,
    parameter: &Real,
) -> ExactCurveResult<()> {
    validate_strict_interior(curve, parameter, CurveOperation2::Subdivision)
}

fn validate_strict_interior_knot(curve: &NurbsCurve2, knot: &Real) -> ExactCurveResult<()> {
    validate_strict_interior(curve, knot, CurveOperation2::KnotRemoval)
}

fn validate_strict_interior(
    curve: &NurbsCurve2,
    parameter: &Real,
    operation: CurveOperation2,
) -> ExactCurveResult<()> {
    let (start, end) = curve.parameter_domain();
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(start, parameter, &policy),
        crate::classify::compare_reals(parameter, end, &policy),
    ) {
        (Some(std::cmp::Ordering::Less), Some(std::cmp::Ordering::Less)) => Ok(()),
        (Some(_), Some(_)) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::Nurbs,
            curve.source(),
            CurveError::InvalidCurveParameter,
        )),
        _ => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            curve.source(),
            UncertaintyReason::Ordering,
        )),
    }
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
            knots.last().expect("validated NURBS has knots"),
            &knots[control_count],
            &policy,
        ),
    ) {
        (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(true),
        (Some(_), Some(_)) => Ok(false),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Construction,
            CurveFamily2::Nurbs,
            source,
            UncertaintyReason::Ordering,
        )),
    }
}

fn validate_subcurve_range(curve: &NurbsCurve2, start: &Real, end: &Real) -> ExactCurveResult<()> {
    let (domain_start, domain_end) = curve.parameter_domain();
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(domain_start, start, &policy),
        crate::classify::compare_reals(start, end, &policy),
        crate::classify::compare_reals(end, domain_end, &policy),
    ) {
        (
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            Some(std::cmp::Ordering::Less),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
        ) => Ok(()),
        (Some(_), Some(_), Some(_)) => Err(ExactCurveError::invalid(
            CurveOperation2::Subdivision,
            CurveFamily2::Nurbs,
            curve.source(),
            CurveError::InvalidCurveRange,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Subdivision,
            CurveFamily2::Nurbs,
            curve.source(),
            UncertaintyReason::Ordering,
        )),
    }
}

fn exact_nurbs_knot_multiplicity(
    knots: &[Real],
    knot: &Real,
    source: Option<CurveSource2>,
    operation: CurveOperation2,
) -> ExactCurveResult<usize> {
    let policy = CurvePolicy::certified();
    let mut multiplicity = 0;
    for candidate in knots {
        match crate::classify::compare_reals(candidate, knot, &policy) {
            Some(std::cmp::Ordering::Equal) => multiplicity += 1,
            Some(_) => {}
            None => {
                return Err(ExactCurveError::blocked(
                    operation,
                    CurveFamily2::Nurbs,
                    source,
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
    source: Option<CurveSource2>,
    operation: CurveOperation2,
) -> ExactCurveResult<()> {
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(first.x(), second.x(), &policy),
        crate::classify::compare_reals(first.y(), second.y(), &policy),
    ) {
        (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(()),
        (Some(_), Some(_)) => Err(ExactCurveError::invalid(
            operation,
            CurveFamily2::Nurbs,
            source,
            CurveError::InvalidDegreeElevation,
        )),
        _ => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            source,
            UncertaintyReason::RealSign,
        )),
    }
}

fn remap_nurbs_operation(error: ExactCurveError, operation: CurveOperation2) -> ExactCurveError {
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

fn select_span_indices(
    spans: &[RationalBezierSpan2],
    parameter: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<(usize, usize)> {
    let policy = CurvePolicy::certified();
    let mut first = None;
    let mut last = None;
    for (span_index, span) in spans.iter().enumerate() {
        let (start, end) = span.knot_interval();
        let lower = crate::classify::compare_reals(start, parameter, &policy);
        let upper = crate::classify::compare_reals(parameter, end, &policy);
        match (lower, upper) {
            (
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
                Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal),
            ) => {
                first.get_or_insert(span_index);
                last = Some(span_index);
            }
            (Some(_), Some(_)) => {}
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::Nurbs,
                    source,
                    UncertaintyReason::Ordering,
                ));
            }
        }
    }
    first.zip(last).ok_or_else(|| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            source,
            CurveError::InvalidCurveParameter,
        )
    })
}

fn matching_nurbs_derivatives(
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
            (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => {}
            (Some(_), Some(_)) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::Nurbs,
                    source,
                    UncertaintyReason::Boundary,
                ));
            }
            _ => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Evaluation,
                    CurveFamily2::Nurbs,
                    source,
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
    source: Option<CurveSource2>,
) -> ExactCurveResult<Point2> {
    let policy = CurvePolicy::certified();
    match (
        crate::classify::compare_reals(first.x(), second.x(), &policy),
        crate::classify::compare_reals(first.y(), second.y(), &policy),
    ) {
        (Some(std::cmp::Ordering::Equal), Some(std::cmp::Ordering::Equal)) => Ok(first),
        (Some(_), Some(_)) => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            source,
            UncertaintyReason::Boundary,
        )),
        _ => Err(ExactCurveError::blocked(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            source,
            UncertaintyReason::RealSign,
        )),
    }
}

fn local_span_parameter(
    span: &RationalBezierSpan2,
    parameter: &Real,
    source: Option<CurveSource2>,
) -> ExactCurveResult<Real> {
    let (start, end) = span.knot_interval();
    let width = end - start;
    ((parameter - start) / width).map_err(|cause| {
        ExactCurveError::invalid(
            CurveOperation2::Evaluation,
            CurveFamily2::Nurbs,
            source,
            cause.into(),
        )
    })
}

fn exact_classification<T>(
    classification: Classification<T>,
    operation: CurveOperation2,
    source: Option<CurveSource2>,
) -> ExactCurveResult<T> {
    match classification {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            operation,
            CurveFamily2::Nurbs,
            source,
            reason,
        )),
    }
}
