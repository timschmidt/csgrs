//! Native graph branches of a certified bivariate zero set.

use super::*;
use crate::UncertaintyReason;
use crate::bezier_algebraic_image::parameter_representation;
use crate::bezier_offset::BezierSimilarityPoint2;
use crate::polynomial_point::PolynomialPoint2;
use hyperlimit::Certainty;
use hypersolve::bivariate_topology::BivariateCurveTopology;
use hypersolve::{
    AlgebraicFiberRootIsolationConfig, AlgebraicFiberRootIsolationStatus,
    AlgebraicRootRepresentation, CurveResultantParameter,
    isolate_bivariate_fiber_roots_at_algebraic_parameter,
};
use std::cmp::Ordering;

/// The complete zero set of one admitted polynomial in a closed rectangle.
///
/// This is a curve collection, not a filled region. Every solver graph arc is
/// retained as a native [`Curve2`]; event roots without an incident arc are
/// retained separately. Singular contacts can join several branch endpoints.
/// The solver's finite-fiber restrictions and resource limits still apply.
#[derive(Clone, Debug)]
pub struct PolynomialCurveSet2 {
    source: Arc<PolynomialCurveSource2>,
    curves: Vec<Curve2>,
    isolated_points: Vec<CurvePoint2>,
}

#[derive(Debug, PartialEq)]
struct PolynomialCurveSource2 {
    topology: BivariateCurveTopology,
    fiber_config: AlgebraicFiberRootIsolationConfig,
}

/// One selected graph branch, its source chart, and its composed image map.
///
/// The parameter is the first coordinate of the source polynomial, including
/// after reversal and similarity transforms. Endpoints retain the selected
/// event fiber; interior points retain the branch's ordered fiber root.
/// Derivatives, rational Bézier promotion, and filled arrangements are not
/// supplied by this carrier and report their typed capability blockers.
#[derive(Clone, Debug, PartialEq)]
pub struct PolynomialCurveBranch2 {
    source: Arc<PolynomialCurveSource2>,
    slab: usize,
    root: usize,
    pub(super) range: CurveParameterRange2,
    endpoints: [CurvePoint2; 2],
    transform: Option<Similarity2>,
    pub(super) reversed: bool,
}

fn blocked(reason: UncertaintyReason) -> ExactCurveError {
    ExactCurveError::blocked(
        CurveOperation2::Evaluation,
        CurveFamily2::ImplicitPolynomial,
        reason,
    )
}

fn invalid(cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(
        CurveOperation2::Evaluation,
        CurveFamily2::ImplicitPolynomial,
        cause,
    )
}

fn decided<T>(value: Classification<T>) -> ExactCurveResult<T> {
    match value {
        Classification::Decided(value) => Ok(value),
        Classification::Uncertain(reason) => Err(blocked(reason)),
    }
}

fn parameter(root: &AlgebraicRootRepresentation) -> ExactCurveResult<BezierParameter2> {
    decided(
        BezierParameter2::from_algebraic_root_representation_unbounded(root, &CurveContext::STRICT)
            .map_err(invalid)?,
    )
}

impl PolynomialCurveSet2 {
    /// Consumes a solver-certified complete graph, replaying its point selectors.
    ///
    /// `fiber_config` controls subsequent point evaluation. Its subdivision and
    /// refinement limits do not bound total work, coefficient size, heap bytes,
    /// or cancellation latency. Failure publishes no partial curve collection.
    pub fn from_topology(
        topology: BivariateCurveTopology,
        fiber_config: AlgebraicFiberRootIsolationConfig,
    ) -> ExactCurveResult<Self> {
        Self::import_topology(topology, fiber_config)
            .map_err(|error| error.with_operation(CurveOperation2::Construction))
    }

    fn import_topology(
        topology: BivariateCurveTopology,
        fiber_config: AlgebraicFiberRootIsolationConfig,
    ) -> ExactCurveResult<Self> {
        let source = Arc::new(PolynomialCurveSource2 {
            topology,
            fiber_config,
        });
        let mut events = Vec::with_capacity(source.topology.events().len());
        for event in source.topology.events() {
            let base = parameter(event.parameter())?;
            let points = PolynomialPoint2::from_fiber(
                source.topology.equation().clone(),
                base.clone(),
                event.roots(),
            )?;
            events.push((CurveParameter2::from(base), points));
        }
        let mut incident: Vec<Vec<bool>> = events
            .iter()
            .map(|(_, points)| vec![false; points.len()])
            .collect();
        let mut curves = Vec::new();
        for (slab, cell) in source.topology.slabs().iter().enumerate() {
            for (root, arc) in cell.arcs().iter().enumerate() {
                incident[slab][arc.left_root()] = true;
                incident[slab + 1][arc.right_root()] = true;
                curves.push(Curve2::from_polynomial_branch(PolynomialCurveBranch2 {
                    source: Arc::clone(&source),
                    slab,
                    root,
                    range: CurveParameterRange2::new_validated(
                        events[slab].0.clone(),
                        events[slab + 1].0.clone(),
                    ),
                    endpoints: [
                        events[slab].1[arc.left_root()].clone(),
                        events[slab + 1].1[arc.right_root()].clone(),
                    ],
                    transform: None,
                    reversed: false,
                }));
            }
        }
        let isolated_points = events
            .into_iter()
            .zip(incident)
            .flat_map(|((_, points), incident)| {
                points
                    .into_iter()
                    .zip(incident)
                    .filter_map(|(point, incident)| (!incident).then_some(point))
            })
            .collect();
        Ok(Self {
            source,
            curves,
            isolated_points,
        })
    }

    /// Borrows the complete source graph and its defining equation.
    pub fn topology(&self) -> &BivariateCurveTopology {
        &self.source.topology
    }
    /// Borrows all native branches in projection-slab and root order.
    pub fn curves(&self) -> &[Curve2] {
        &self.curves
    }
    /// Borrows event points with no incident branch.
    pub fn isolated_points(&self) -> &[CurvePoint2] {
        &self.isolated_points
    }
    /// Whether the complete zero set is empty.
    pub fn is_empty(&self) -> bool {
        self.curves.is_empty() && self.isolated_points.is_empty()
    }
}

impl PolynomialCurveBranch2 {
    /// Controls for subsequent selected-fiber evaluation, retained on replay.
    pub fn evaluation_config(&self) -> AlgebraicFiberRootIsolationConfig {
        self.source.fiber_config
    }

    /// Whether traversal runs from the upper to the lower source parameter.
    pub const fn is_reversed(&self) -> bool {
        self.reversed
    }

    /// Complete source graph, shared by all restrictions and images.
    pub fn topology(&self) -> &BivariateCurveTopology {
        &self.source.topology
    }
    /// Index of the source projection slab.
    pub fn slab_index(&self) -> usize {
        self.slab
    }
    /// Ordered root index inside that open slab.
    pub fn root_index(&self) -> usize {
        self.root
    }
    /// Composed map from the polynomial's plane to this curve's plane.
    pub fn image_transform(&self) -> Option<&Similarity2> {
        self.transform.as_ref()
    }
    /// Whether two branches retain the same topology allocation.
    pub fn shares_source(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.source, &other.source)
    }

    pub(super) fn endpoint(&self, start: bool) -> CurvePoint2 {
        self.endpoints[usize::from(start == self.reversed)].clone()
    }

    pub(super) fn point_at(
        &self,
        value: &CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePoint2> {
        let compare =
            |boundary| decided(value.cmp_by_refinement(boundary, policy).map_err(invalid)?);
        let first = compare(self.range.start())?;
        let last = compare(self.range.end())?;
        if first.is_lt() || last.is_gt() {
            return Err(invalid(CurveError::InvalidCurveParameter));
        }
        if first.is_eq() {
            return Ok(self.endpoints[0].clone());
        }
        if last.is_eq() {
            return Ok(self.endpoints[1].clone());
        }
        let base = value
            .as_bezier_parameter()
            .ok_or_else(|| blocked(UncertaintyReason::Unsupported))?;
        let root = match base {
            BezierParameter2::Exact(value) => AlgebraicRootRepresentation::from_exact_value(value),
            BezierParameter2::Algebraic(value) => parameter_representation(value, policy),
        };
        let domain = &self.topology().domain().second;
        let report = isolate_bivariate_fiber_roots_at_algebraic_parameter(
            self.topology().equation(),
            CurveResultantParameter::First,
            &root,
            &domain[0],
            &domain[1],
            self.source.fiber_config,
            hyperlimit::PredicatePolicy::STRICT,
        );
        match report.status {
            AlgebraicFiberRootIsolationStatus::BudgetExhausted
            | AlgebraicFiberRootIsolationStatus::DepthLimit => {
                return Err(blocked(UncertaintyReason::BudgetExhausted));
            }
            AlgebraicFiberRootIsolationStatus::UnsupportedCoefficient => {
                return Err(blocked(UncertaintyReason::Unsupported));
            }
            _ => {}
        }
        if report.certainty != Certainty::Exact
            || report.status != AlgebraicFiberRootIsolationStatus::Isolated
        {
            return Err(blocked(UncertaintyReason::Predicate));
        }
        // Projection completeness proves every open fiber has the same ordered
        // simple roots. A contradictory census must never select a different arc.
        if report.intervals.len() != self.topology().slabs()[self.slab].arcs().len() {
            return Err(invalid(CurveError::InvalidBezierAlgebraicParameter));
        }
        let mut points = PolynomialPoint2::from_fiber(
            self.topology().equation().clone(),
            base.clone(),
            &report.intervals[self.root..=self.root],
        )?;
        let point = points
            .pop()
            .ok_or_else(|| invalid(CurveError::InvalidBezierAlgebraicParameter))?;
        Ok(self.map_point(point, policy))
    }

    fn map_point(&self, point: CurvePoint2, policy: &CurveContext) -> CurvePoint2 {
        match &self.transform {
            Some(transform) => CurvePoint2::from(BezierSimilarityPoint2::new(
                point,
                transform.clone(),
                policy,
            )),
            None => point,
        }
    }

    pub(super) fn reversed(&self) -> Self {
        let mut result = self.clone();
        result.reversed = !result.reversed;
        result
    }

    pub(super) fn transformed(&self, transform: &Similarity2, policy: &CurveContext) -> Self {
        let mut result = self.clone();
        result.transform = Some(
            self.transform
                .as_ref()
                .map_or_else(|| transform.clone(), |source| source.then(transform)),
        );
        result.endpoints = self.endpoints.clone().map(|point| {
            CurvePoint2::from(BezierSimilarityPoint2::new(
                point,
                transform.clone(),
                policy,
            ))
        });
        result
    }

    pub(super) fn restricted(
        &self,
        start: CurveParameter2,
        end: CurveParameter2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        if decided(start.cmp_by_refinement(&end, policy).map_err(invalid)?)? != Ordering::Less {
            return Err(invalid(CurveError::InvalidCurveRange));
        }
        let endpoints = [self.point_at(&start, policy)?, self.point_at(&end, policy)?];
        let mut result = self.clone();
        result.range = CurveParameterRange2::new_validated(start, end);
        result.endpoints = endpoints;
        Ok(result)
    }

    pub(super) fn bounds(&self) -> ExactCurveResult<Aabb2> {
        // The entire certified rectangle is conservative for every restriction.
        let domain = self.topology().domain();
        let mut corners = Vec::with_capacity(4);
        for x in &domain.first {
            for y in &domain.second {
                let point = Point2::new(x.clone(), y.clone());
                corners.push(
                    self.transform
                        .as_ref()
                        .map_or_else(|| point.clone(), |map| map.transform_point(&point)),
                );
            }
        }
        decided(Aabb2::from_points(&corners))
    }
}

impl Curve2 {
    /// Borrows a retained polynomial branch and its source/root identity.
    pub fn polynomial_branch(&self) -> Option<&PolynomialCurveBranch2> {
        match &self.data.carrier {
            CurveCarrier2::Polynomial(branch) => Some(branch),
            _ => None,
        }
    }

    pub(super) fn from_polynomial_branch(branch: PolynomialCurveBranch2) -> Self {
        Self {
            data: Arc::new(CurveData2 {
                carrier: CurveCarrier2::Polynomial(Box::new(branch)),
                lineage: None,
                parameter_domain: OnceLock::new(),
                native_bezier_fragments: PolicyEvaluationCache::new(),
                rational_evaluators: PolicyEvaluationCache::new(),
                bounds: OnceLock::new(),
            }),
        }
    }
}
