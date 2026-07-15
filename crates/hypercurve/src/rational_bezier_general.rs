//! Exact rational Bezier curves of arbitrary positive degree.

use std::cell::{OnceCell, RefCell};
use std::rc::Rc;

use hyperreal::{Real, RealSign, ZeroKnowledge};
#[cfg(feature = "predicates")]
use hypersolve::{
    AlgebraicRootRefinementComparisonConfig, compare_algebraic_root_representations_by_difference,
};
use hypersolve::{
    AlgebraicRootRepresentation, CurveIntersectionResultantConfig,
    CurveIntersectionResultantReport, CurveIntersectionResultantStatus, CurveResultantParameter,
    RationalParametricCurve2, resultant_rational_parametric_curve_intersection,
};

use crate::bezier_algebraic_image::{
    exact_real_algebraic_representation, rational_derivative_images_from_power_basis,
    rational_point_image_from_power_basis, rational_tangent_image_from_power_basis,
};
use crate::bezier_parameter::bernstein_to_power_coefficients;
use crate::bezier_topology::exact_line_contact_relation_from_bernstein_distances;
use crate::classify::{
    classify_oriented_line, compare_reals, in_closed_unit_interval, is_zero, orient2d_real_expr,
    real_sign,
};
use crate::intersect::oriented_param_range_overlap;
use crate::{
    Aabb2, Axis2, BezierArrangementGraph2, BezierLineContactRelation, BezierLineImageFitRelation,
    BezierParameter2, BezierParameterPolynomial, BezierParameterRange2,
    BezierSplitMaterialization2, Classification, CurveDerivative2, CurveError, CurveFamily2,
    CurveOperation2, CurvePolicy, CurveResult, ExactCurveError, ExactCurveResult, LineSeg2,
    LineSide, ParamRange, Point2, RationalBezierAlgebraicPointImage2,
    RationalBezierAlgebraicTangentImage2, UncertaintyReason,
};

/// Exact planar rational Bezier curve with an arbitrary positive degree.
///
/// Controls and weights are retained in affine form. Evaluation and splitting
/// operate in homogeneous coordinates, so unequal-weight cubic and
/// higher-degree NURBS spans do not need sampling or degree reduction.
#[derive(Clone, Debug)]
pub struct RationalBezier2 {
    data: Rc<RationalBezierData>,
}

#[derive(Debug)]
struct RationalBezierData {
    control_points: Vec<Point2>,
    weights: Vec<Real>,
    lineage: RationalBezierLineage,
    homogeneous_controls: OnceCell<Vec<HomogeneousPoint2>>,
    homogeneous_power_basis: OnceCell<RationalParametricCurve2>,
    x_derivative_numerator_bernstein: OnceCell<Option<Vec<Real>>>,
    y_derivative_numerator_bernstein: OnceCell<Option<Vec<Real>>>,
    x_axis_monotonicity: OnceCell<bool>,
    y_axis_monotonicity: OnceCell<bool>,
    degree_elevations: OnceCell<RefCell<Vec<ExactCurveResult<RationalBezier2>>>>,
}

#[derive(Clone, Debug)]
struct RationalBezierLineage {
    root: Rc<RationalBezierLineageRoot>,
    range: ParamRange,
}

#[derive(Debug, Default)]
struct RationalBezierLineageRoot {
    image_is_injective: OnceCell<bool>,
    implicit_quadratic_conic: OnceCell<Rc<[Real; 6]>>,
    quadratic_conic_parameter_frame: OnceCell<Rc<[HomogeneousPoint2; 3]>>,
}

#[derive(Clone, Debug)]
struct HomogeneousPoint2 {
    x: Real,
    y: Real,
    weight: Real,
}

#[derive(Clone, Debug)]
struct PolynomialGraph2 {
    axis: Axis2,
    origin: Real,
    scale: Real,
    dependent: Vec<Real>,
}

impl RationalBezierLineage {
    fn parameter_at(&self, local_parameter: &Real) -> Real {
        self.range.start() + local_parameter * (self.range.end() - self.range.start())
    }

    fn subrange(&self, start: &Real, end: &Real) -> Self {
        Self {
            root: Rc::clone(&self.root),
            range: ParamRange::new(self.parameter_at(start), self.parameter_at(end)),
        }
    }

    fn reversed(&self) -> Self {
        Self {
            root: Rc::clone(&self.root),
            range: ParamRange::new(self.range.end().clone(), self.range.start().clone()),
        }
    }
}

/// Exact parameter evidence for point incidence on a general rational Bezier.
#[derive(Clone, Debug, PartialEq)]
pub enum RationalBezierPointIncidence2 {
    /// Every parameter maps to the query point.
    EntireCurve,
    /// The complete ordered set of represented or isolated algebraic parameters.
    Parameters(Vec<BezierParameter2>),
}

/// Exact elimination candidates for two general rational Bezier curves.
///
/// Candidate lists are complete projections onto each parameter axis, but are
/// deliberately not paired: a resultant root becomes a topology event only
/// after exact replay proves that one parameter from each list maps to the
/// same affine point.
#[derive(Clone, Debug, PartialEq)]
pub enum RationalBezierIntersectionCandidates2 {
    /// At least one parameter projection has no root in the finite domains.
    NoIntersection,
    /// Both parameter projections contain all possible finite contacts.
    Candidates {
        /// Ordered represented or algebraically isolated first-curve parameters.
        first_parameters: Vec<BezierParameter2>,
        /// Ordered represented or algebraically isolated second-curve parameters.
        second_parameters: Vec<BezierParameter2>,
    },
    /// A resultant vanished identically, indicating a shared algebraic
    /// component or another elimination degeneracy that needs overlap replay.
    DegenerateResultant,
}

/// Exact affine point evidence retained for a rational Bezier contact.
#[derive(Clone, Debug, PartialEq)]
pub enum RationalBezierIntersectionPointEvidence2 {
    /// The contact point is represented directly by [`Real`] coordinates.
    Exact(Point2),
    /// The contact point is retained as exact algebraic coordinate images.
    Algebraic(RationalBezierAlgebraicPointImage2),
}

/// One exactly replayed parameter pair shared by two rational Bezier images.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierIntersectionContact2 {
    first_parameter: BezierParameter2,
    second_parameter: BezierParameter2,
    point: RationalBezierIntersectionPointEvidence2,
}

/// Relative parameter orientation of a certified shared rational-Bezier image.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RationalBezierOverlapOrientation2 {
    /// Both parameter domains traverse the shared image in the same direction.
    Same,
    /// The second parameter domain traverses the shared image in reverse.
    Reversed,
}

/// Certified complete-image overlap between two rational Bezier curves.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierIntersectionOverlap2 {
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
}

impl RationalBezierIntersectionOverlap2 {
    /// Returns the exact overlap range on the first curve.
    pub const fn first_range(&self) -> &BezierParameterRange2 {
        &self.first_range
    }

    /// Returns the exact overlap range on the second curve, oriented to match
    /// traversal of [`Self::first_range`].
    pub const fn second_range(&self) -> &BezierParameterRange2 {
        &self.second_range
    }

    /// Returns relative parameter orientation on the shared image.
    pub const fn orientation(&self) -> RationalBezierOverlapOrientation2 {
        self.orientation
    }
}

impl RationalBezierIntersectionContact2 {
    /// Returns the exact parameter on the first curve.
    pub const fn first_parameter(&self) -> &BezierParameter2 {
        &self.first_parameter
    }

    /// Returns the exact parameter on the second curve.
    pub const fn second_parameter(&self) -> &BezierParameter2 {
        &self.second_parameter
    }

    /// Returns retained affine point evidence from the first curve replay.
    pub const fn point(&self) -> &RationalBezierIntersectionPointEvidence2 {
        &self.point
    }
}

/// Exact replay status for rational Bezier resultant candidates.
#[derive(Clone, Debug, PartialEq)]
pub enum RationalBezierIntersectionContacts2 {
    /// Replay certified that the finite curve images do not meet.
    NoIntersection,
    /// Every resultant candidate pair was decided and these contacts remain.
    Contacts(Rc<[RationalBezierIntersectionContact2]>),
    /// Exact shared-component replay certified a positive-length full or
    /// partial shared image and retained both oriented parameter ranges.
    Overlap(RationalBezierIntersectionOverlap2),
    /// Some contacts were certified, but at least one candidate comparison
    /// remained unresolved under the exact algebraic comparison budget.
    Incomplete {
        /// Contacts already certified by exact replay.
        contacts: Rc<[RationalBezierIntersectionContact2]>,
        /// Complete unpaired resultant projections retained for later replay.
        candidates: RationalBezierIntersectionCandidates2,
    },
    /// A resultant vanished identically and overlap replay is required.
    DegenerateResultant,
}

#[derive(Debug)]
enum RationalBezierSharedComponentReplay {
    Overlap(RationalBezierIntersectionOverlap2),
    Contacts(Vec<(Real, Real)>),
    Unresolved,
}

/// Retained split topology derived from one completely replayed curve pair.
///
/// The contact collection is shared with the prepared pair. The two split
/// materializations preserve each contact parameter and its exact endpoint
/// images, so an arrangement can consume the result without rerunning
/// resultants or algebraic point comparison.
#[derive(Clone, Debug)]
pub struct RationalBezierIntersectionTopology2 {
    data: Rc<RationalBezierIntersectionTopologyData>,
}

#[derive(Debug)]
struct RationalBezierIntersectionTopologyData {
    contacts: Rc<[RationalBezierIntersectionContact2]>,
    first: BezierSplitMaterialization2,
    second: BezierSplitMaterialization2,
    arrangement: OnceCell<CurveResult<BezierArrangementGraph2>>,
}

impl RationalBezierIntersectionTopology2 {
    /// Returns all certified pair contacts in deterministic parameter order.
    pub fn contacts(&self) -> &[RationalBezierIntersectionContact2] {
        &self.data.contacts
    }

    /// Returns the first curve split at every certified contact parameter.
    pub fn first(&self) -> &BezierSplitMaterialization2 {
        &self.data.first
    }

    /// Returns the second curve split at every certified contact parameter.
    pub fn second(&self) -> &BezierSplitMaterialization2 {
        &self.data.second
    }

    /// Returns whether arrangement assembly has already been retained.
    pub fn is_arrangement_cached(&self) -> bool {
        self.data.arrangement.get().is_some()
    }

    /// Builds an arrangement graph once and returns a clone-shared fact view.
    pub fn arrangement_graph_view(&self) -> CurveResult<&BezierArrangementGraph2> {
        match self.data.arrangement.get_or_init(|| {
            BezierArrangementGraph2::from_split_materializations(&[
                self.data.first.clone(),
                self.data.second.clone(),
            ])
        }) {
            Ok(graph) => Ok(graph),
            Err(cause) => Err(cause.clone()),
        }
    }

    /// Returns an owned arrangement graph from the retained pair materializations.
    pub fn arrangement_graph(&self) -> CurveResult<BezierArrangementGraph2> {
        self.arrangement_graph_view().cloned()
    }
}

/// Clone-shared retained facts for one rational Bezier pair.
#[derive(Clone, Debug)]
pub struct PreparedRationalBezierIntersection2 {
    data: Rc<PreparedRationalBezierIntersectionData>,
}

#[derive(Debug)]
struct PreparedRationalBezierIntersectionData {
    first: RationalBezier2,
    second: RationalBezier2,
    policy: CurvePolicy,
    candidates: RationalBezierIntersectionCandidates2,
    contacts: OnceCell<CurveResult<Classification<RationalBezierIntersectionContacts2>>>,
    topology: OnceCell<CurveResult<Classification<RationalBezierIntersectionTopology2>>>,
}

impl PreparedRationalBezierIntersection2 {
    /// Returns the retained first operand.
    pub fn first(&self) -> &RationalBezier2 {
        &self.data.first
    }

    /// Returns the retained second operand.
    pub fn second(&self) -> &RationalBezier2 {
        &self.data.second
    }

    /// Returns the exact policy captured by preparation.
    pub fn policy(&self) -> &CurvePolicy {
        &self.data.policy
    }

    /// Returns the complete retained resultant projections.
    pub fn candidates(&self) -> &RationalBezierIntersectionCandidates2 {
        &self.data.candidates
    }

    /// Returns whether paired contact replay has already been retained.
    pub fn is_contact_replay_cached(&self) -> bool {
        self.data.contacts.get().is_some()
    }

    /// Returns whether contact-derived split topology has already been retained.
    pub fn is_topology_cached(&self) -> bool {
        self.data.topology.get().is_some()
    }

    /// Borrows the retained contact replay with typed failure context.
    pub fn try_contact_view(&self) -> ExactCurveResult<&RationalBezierIntersectionContacts2> {
        match self.retained_contacts_ref() {
            Ok(Classification::Decided(contacts)) => Ok(contacts),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                *reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                cause.clone(),
            )),
        }
    }

    /// Returns retained contacts with typed failure context.
    pub fn try_contacts(&self) -> ExactCurveResult<RationalBezierIntersectionContacts2> {
        self.try_contact_view().cloned()
    }

    /// Borrows retained contact-derived topology with typed failure context.
    pub fn try_topology_view(&self) -> ExactCurveResult<&RationalBezierIntersectionTopology2> {
        match self.retained_topology_ref() {
            Ok(Classification::Decided(topology)) => Ok(topology),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Arrangement,
                CurveFamily2::RationalBezier,
                None,
                *reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Arrangement,
                CurveFamily2::RationalBezier,
                None,
                cause.clone(),
            )),
        }
    }

    /// Returns retained contact-derived topology with typed failure context.
    pub fn try_topology(&self) -> ExactCurveResult<RationalBezierIntersectionTopology2> {
        self.try_topology_view().cloned()
    }

    fn retained_contacts_ref(
        &self,
    ) -> &CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        self.data.contacts.get_or_init(|| {
            self.data.first.replay_intersection_candidate_set(
                &self.data.second,
                &self.data.candidates,
                &self.data.policy,
            )
        })
    }

    fn retained_topology_ref(
        &self,
    ) -> &CurveResult<Classification<RationalBezierIntersectionTopology2>> {
        self.data.topology.get_or_init(|| {
            let contacts = match self.retained_contacts_ref() {
                Ok(Classification::Decided(
                    RationalBezierIntersectionContacts2::NoIntersection,
                )) => Rc::from([]),
                Ok(Classification::Decided(RationalBezierIntersectionContacts2::Contacts(
                    contacts,
                ))) => Rc::clone(contacts),
                Ok(Classification::Decided(RationalBezierIntersectionContacts2::Overlap(_))) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Ok(Classification::Decided(RationalBezierIntersectionContacts2::Incomplete {
                    ..
                })) => return Ok(Classification::Uncertain(UncertaintyReason::Predicate)),
                Ok(Classification::Decided(
                    RationalBezierIntersectionContacts2::DegenerateResultant,
                )) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                Ok(Classification::Uncertain(reason)) => {
                    return Ok(Classification::Uncertain(*reason));
                }
                Err(cause) => return Err(cause.clone()),
            };
            let first_parameters = contacts
                .iter()
                .map(|contact| contact.first_parameter().clone())
                .collect::<Vec<_>>();
            let second_parameters = contacts
                .iter()
                .map(|contact| contact.second_parameter().clone())
                .collect::<Vec<_>>();
            let first = match self
                .data
                .first
                .split_at_parameters(&first_parameters, &self.data.policy)?
            {
                Classification::Decided(first) => first,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let second = match self
                .data
                .second
                .split_at_parameters(&second_parameters, &self.data.policy)?
            {
                Classification::Decided(second) => second,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            Ok(Classification::Decided(
                RationalBezierIntersectionTopology2 {
                    data: Rc::new(RationalBezierIntersectionTopologyData {
                        contacts,
                        first,
                        second,
                        arrangement: OnceCell::new(),
                    }),
                },
            ))
        })
    }
}

#[derive(Clone, Debug)]
struct CandidatePointReplay {
    evidence: RationalBezierIntersectionPointEvidence2,
    x: AlgebraicRootRepresentation,
    y: AlgebraicRootRepresentation,
}

#[derive(Debug)]
enum ResultantParameterProjection {
    Empty,
    Parameters(Vec<BezierParameter2>),
    Degenerate,
}

const MAX_RATIONAL_INTERSECTION_RESULTANT_DEGREE: usize = 128;
const RATIONAL_INTERSECTION_RESULTANT_PRECISION: i32 = -128;

impl PartialEq for RationalBezier2 {
    fn eq(&self, other: &Self) -> bool {
        self.control_points() == other.control_points() && self.weights() == other.weights()
    }
}

impl RationalBezier2 {
    /// Constructs an exact positive-degree rational Bezier curve.
    pub fn try_new(control_points: Vec<Point2>, weights: Vec<Real>) -> CurveResult<Self> {
        Self::try_new_with_lineage(
            control_points,
            weights,
            RationalBezierLineage {
                root: Rc::new(RationalBezierLineageRoot::default()),
                range: ParamRange::new(Real::zero(), Real::one()),
            },
        )
    }

    fn try_new_with_lineage(
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        lineage: RationalBezierLineage,
    ) -> CurveResult<Self> {
        if control_points.len() < 2 || control_points.len() != weights.len() {
            return Err(CurveError::InvalidRationalBezier);
        }
        if weights
            .iter()
            .any(|weight| weight.zero_status() == ZeroKnowledge::Zero)
        {
            return Err(CurveError::ZeroRationalBezierWeight);
        }
        Ok(Self {
            data: Rc::new(RationalBezierData {
                control_points,
                weights,
                lineage,
                homogeneous_controls: OnceCell::new(),
                homogeneous_power_basis: OnceCell::new(),
                x_derivative_numerator_bernstein: OnceCell::new(),
                y_derivative_numerator_bernstein: OnceCell::new(),
                x_axis_monotonicity: OnceCell::new(),
                y_axis_monotonicity: OnceCell::new(),
                degree_elevations: OnceCell::new(),
            }),
        })
    }

    /// Returns the polynomial degree of the homogeneous Bernstein curve.
    pub fn degree(&self) -> usize {
        self.control_points().len() - 1
    }

    /// Returns exact affine controls in Bernstein order.
    pub fn control_points(&self) -> &[Point2] {
        &self.data.control_points
    }

    /// Returns exact homogeneous weights in Bernstein order.
    pub fn weights(&self) -> &[Real] {
        &self.data.weights
    }

    /// Returns the exact parameter range in the root curve's source domain.
    pub fn source_parameter_range(&self) -> &ParamRange {
        &self.data.lineage.range
    }

    /// Returns whether elevation to `target_degree` has already been retained.
    pub fn is_degree_elevation_cached(&self, target_degree: usize) -> bool {
        if target_degree == self.degree() {
            return true;
        }
        let Some(offset) = target_degree.checked_sub(self.degree()) else {
            return false;
        };
        self.data
            .degree_elevations
            .get()
            .is_some_and(|elevations| elevations.borrow().len() >= offset)
    }

    /// Elevates this rational Bezier exactly to `target_degree`.
    ///
    /// Elevation is performed in homogeneous Bernstein coordinates. Repeated
    /// calls and clones reuse every intermediate elevated degree. The public
    /// parameterization and retained source lineage are unchanged.
    pub fn elevated_to_degree(&self, target_degree: usize) -> ExactCurveResult<Self> {
        let source_degree = self.degree();
        if target_degree < source_degree {
            return Err(ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::RationalBezier,
                None,
                CurveError::InvalidDegreeElevation,
            ));
        }
        if target_degree == source_degree {
            return Ok(self.clone());
        }
        let elevation_count = target_degree.checked_sub(source_degree).ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::RationalBezier,
                None,
                CurveError::InvalidDegreeElevation,
            )
        })?;
        let elevations = self
            .data
            .degree_elevations
            .get_or_init(|| RefCell::new(Vec::new()));
        while elevations.borrow().len() < elevation_count {
            let source = {
                let retained = elevations.borrow();
                match retained.last() {
                    Some(Ok(curve)) => Ok(curve.clone()),
                    Some(Err(error)) => Err(error.clone()),
                    None => Ok(self.clone()),
                }
            };
            let elevated = source.and_then(|curve| curve.elevate_once_uncached());
            elevations.borrow_mut().push(elevated);
        }
        elevations.borrow()[elevation_count - 1].clone()
    }

    fn elevate_once_uncached(&self) -> ExactCurveResult<Self> {
        let target_degree = self.degree().checked_add(1).ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::RationalBezier,
                None,
                CurveError::InvalidDegreeElevation,
            )
        })?;
        let denominator = u64::try_from(target_degree).map(Real::from).map_err(|_| {
            ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::RationalBezier,
                None,
                CurveError::InvalidDegreeElevation,
            )
        })?;
        let source = self.homogeneous_controls();
        let mut homogeneous = Vec::with_capacity(source.len() + 1);
        homogeneous.push(source[0].clone());
        for index in 1..target_degree {
            let numerator = u64::try_from(index).map(Real::from).map_err(|_| {
                ExactCurveError::invalid(
                    CurveOperation2::DegreeElevation,
                    CurveFamily2::RationalBezier,
                    None,
                    CurveError::InvalidDegreeElevation,
                )
            })?;
            let alpha = (numerator / &denominator).map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::DegreeElevation,
                    CurveFamily2::RationalBezier,
                    None,
                    cause.into(),
                )
            })?;
            homogeneous.push(source[index].lerp(&source[index - 1], &alpha));
        }
        homogeneous.push(source[source.len() - 1].clone());

        let policy = CurvePolicy::certified();
        let mut control_points = Vec::with_capacity(homogeneous.len());
        let mut weights = Vec::with_capacity(homogeneous.len());
        for point in homogeneous {
            match project_homogeneous(&point, &policy) {
                Classification::Decided(control) => control_points.push(control),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::DegreeElevation,
                        CurveFamily2::RationalBezier,
                        None,
                        reason,
                    ));
                }
            }
            weights.push(point.weight);
        }
        Self::try_new_with_lineage(control_points, weights, self.data.lineage.clone()).map_err(
            |cause| {
                ExactCurveError::invalid(
                    CurveOperation2::DegreeElevation,
                    CurveFamily2::RationalBezier,
                    None,
                    cause,
                )
            },
        )
    }

    /// Returns whether the clone-shared homogeneous control net has been computed.
    pub fn is_homogeneous_control_net_cached(&self) -> bool {
        self.data.homogeneous_controls.get().is_some()
    }

    /// Returns whether the clone-shared homogeneous power basis has been computed.
    pub fn is_homogeneous_power_basis_cached(&self) -> bool {
        self.data.homogeneous_power_basis.get().is_some()
    }

    /// Returns the exact start point.
    pub fn start(&self) -> &Point2 {
        &self.control_points()[0]
    }

    /// Returns the exact end point.
    pub fn end(&self) -> &Point2 {
        self.control_points()
            .last()
            .expect("validated rational Bezier has controls")
    }

    /// Evaluates this curve from its clone-shared homogeneous power basis.
    ///
    /// The basis is constructed exactly once from the Bernstein controls. Horner
    /// evaluation then avoids allocating and mutating a de Casteljau work vector
    /// on every repeated point query.
    pub fn point_at(&self, parameter: &Real, policy: &CurvePolicy) -> ExactCurveResult<Point2> {
        match self.point_at_classified(parameter, policy) {
            Classification::Decided(point) => Ok(point),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
        }
    }

    pub(crate) fn point_at_classified(
        &self,
        parameter: &Real,
        policy: &CurvePolicy,
    ) -> Classification<Point2> {
        if in_closed_unit_interval(parameter, policy) != Some(true) {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        let Ok(power_basis) = self.homogeneous_power_basis() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        project_homogeneous(
            &HomogeneousPoint2 {
                x: evaluate_power_polynomial(&power_basis.x_numerator, parameter),
                y: evaluate_power_polynomial(&power_basis.y_numerator, parameter),
                weight: evaluate_power_polynomial(&power_basis.weight, parameter),
            },
            policy,
        )
    }

    /// Evaluates the exact affine derivative with respect to the Bezier parameter.
    pub fn derivative_at(
        &self,
        parameter: &Real,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<CurveDerivative2> {
        match self.derivative_at_classified(parameter, policy) {
            Classification::Decided(derivative) => Ok(derivative),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
        }
    }

    pub(crate) fn derivative_at_classified(
        &self,
        parameter: &Real,
        policy: &CurvePolicy,
    ) -> Classification<CurveDerivative2> {
        if in_closed_unit_interval(parameter, policy) != Some(true) {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        let Ok(power_basis) = self.homogeneous_power_basis() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let (x, dx) =
            evaluate_power_polynomial_value_and_derivative(&power_basis.x_numerator, parameter);
        let (y, dy) =
            evaluate_power_polynomial_value_and_derivative(&power_basis.y_numerator, parameter);
        let (weight, dweight) =
            evaluate_power_polynomial_value_and_derivative(&power_basis.weight, parameter);
        match is_zero(&weight, policy) {
            Some(false) => {}
            Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
        let denominator = &weight * &weight;
        let Ok(dx) = (&dx * &weight - &x * &dweight) / &denominator else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        let Ok(dy) = (&dy * &weight - &y * &dweight) / denominator else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        Classification::Decided(CurveDerivative2::new(dx, dy))
    }

    /// Evaluates exact affine derivatives through `max_order` at one parameter.
    ///
    /// The returned vector stores orders `1..=max_order`. Homogeneous
    /// numerator and denominator derivatives are evaluated together from the
    /// retained power basis, then the quotient recurrence computes every
    /// affine order from the preceding values. Rational derivatives are not
    /// truncated at the Bezier degree: a nonconstant denominator can produce
    /// nonzero derivatives of arbitrarily high order.
    pub fn derivatives_at(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.derivatives_at_classified(parameter, max_order, policy) {
            Classification::Decided(derivatives) => Ok(derivatives),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
        }
    }

    pub(crate) fn derivatives_at_classified(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurvePolicy,
    ) -> Classification<Vec<CurveDerivative2>> {
        match self.affine_derivative_values_at(parameter, max_order, policy) {
            Classification::Decided(values) => Classification::Decided(
                values
                    .into_iter()
                    .skip(1)
                    .map(|(dx, dy)| CurveDerivative2::new(dx, dy))
                    .collect(),
            ),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Evaluates the affine point at an isolated algebraic parameter.
    ///
    /// The clone-shared homogeneous power basis is transformed through the
    /// exact rational-image package, preserving represented algebraic
    /// coordinates and denominator validation instead of sampling the
    /// parameter interval.
    pub fn point_at_algebraic_parameter(
        &self,
        parameter: &crate::BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        let power_basis = self.homogeneous_power_basis()?;
        rational_point_image_from_power_basis(
            parameter,
            power_basis.x_numerator.clone(),
            power_basis.y_numerator.clone(),
            power_basis.weight.clone(),
            policy,
        )
    }

    /// Evaluates the affine tangent at an isolated algebraic parameter.
    pub fn tangent_at_algebraic_parameter(
        &self,
        parameter: &crate::BezierAlgebraicParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
        let power_basis = self.homogeneous_power_basis()?;
        rational_tangent_image_from_power_basis(
            parameter,
            power_basis.x_numerator.clone(),
            power_basis.y_numerator.clone(),
            power_basis.weight.clone(),
            policy,
        )
    }

    /// Evaluates exact affine derivative images through `max_order` at an
    /// isolated algebraic parameter.
    ///
    /// The returned vector stores orders `1..=max_order`. All orders are
    /// constructed in one quotient-recurrence pass, reusing each preceding
    /// numerator and denominator power rather than rebuilding lower-order
    /// derivatives. An order-`k` coordinate is represented as `A_k/D^(k+1)`.
    pub fn derivatives_at_algebraic_parameter(
        &self,
        parameter: &crate::BezierAlgebraicParameter2,
        max_order: usize,
        policy: &CurvePolicy,
    ) -> CurveResult<Vec<RationalBezierAlgebraicTangentImage2>> {
        let power_basis = self.homogeneous_power_basis()?;
        rational_derivative_images_from_power_basis(
            parameter,
            power_basis.x_numerator.clone(),
            power_basis.y_numerator.clone(),
            power_basis.weight.clone(),
            policy,
            max_order,
        )
    }

    /// Returns a conservative exact control-hull bound when all weights share a sign.
    pub fn certified_bounds(&self, policy: &CurvePolicy) -> ExactCurveResult<Aabb2> {
        match self.certified_bounds_classified(policy) {
            Classification::Decided(bounds) => Ok(bounds),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Classification,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
        }
    }

    pub(crate) fn certified_bounds_classified(
        &self,
        policy: &CurvePolicy,
    ) -> Classification<Aabb2> {
        match self.common_weight_sign(policy) {
            Classification::Decided(_) => Aabb2::from_points(self.control_points().iter(), policy),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    /// Certifies whether one coordinate is monotone on the full parameter domain.
    ///
    /// The quotient derivative numerator `N'D - ND'` is formed directly in
    /// Bernstein form. A one-signed coefficient sequence proves monotonicity
    /// without constructing roots. Mixed-sign sequences use exact root
    /// isolation: an odd-multiplicity interior derivative root proves an
    /// extremum, while endpoint roots and even-multiplicity stationary points
    /// do not change monotonicity.
    pub fn axis_is_monotone(&self, axis: Axis2, policy: &CurvePolicy) -> ExactCurveResult<bool> {
        match self.axis_monotonicity_classified(axis, policy) {
            Ok(Classification::Decided(monotone)) => Ok(monotone),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Classification,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Classification,
                CurveFamily2::RationalBezier,
                None,
                cause,
            )),
        }
    }

    pub(crate) fn axis_monotonicity_classified(
        &self,
        axis: Axis2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        let cache = match axis {
            Axis2::X => &self.data.x_axis_monotonicity,
            Axis2::Y => &self.data.y_axis_monotonicity,
        };
        if let Some(monotone) = cache.get() {
            return Ok(Classification::Decided(*monotone));
        }
        let result = self.compute_axis_is_monotone(axis, policy)?;
        if let Classification::Decided(monotone) = result {
            let _ = cache.set(monotone);
        }
        Ok(result)
    }

    fn compute_axis_is_monotone(
        &self,
        axis: Axis2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        if let Classification::Uncertain(reason) = self.common_weight_sign(policy) {
            return Ok(Classification::Uncertain(reason));
        }
        let Some(coefficients) = self.axis_derivative_numerator_bernstein(axis) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut has_positive = false;
        let mut has_negative = false;
        let mut first_nonzero = None;
        let mut last_nonzero = None;
        for coefficient in coefficients {
            let Some(sign) = real_sign(coefficient, policy) else {
                return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
            };
            has_positive |= sign == RealSign::Positive;
            has_negative |= sign == RealSign::Negative;
            if sign != RealSign::Zero {
                first_nonzero.get_or_insert(sign);
                last_nonzero = Some(sign);
            }
        }
        if !has_positive || !has_negative {
            return Ok(Classification::Decided(true));
        }
        if first_nonzero != last_nonzero {
            return Ok(Classification::Decided(false));
        }
        let polynomial = match BezierParameterPolynomial::try_new_bernstein_basis(
            coefficients.to_vec(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let roots = match polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        for root in roots {
            if root
                .as_exact()
                .is_some_and(|root| root == &Real::zero() || root == &Real::one())
            {
                continue;
            }
            match polynomial.changes_sign_at_root(&root, policy)? {
                Classification::Decided(true) => return Ok(Classification::Decided(false)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(true))
    }

    /// Classifies exact contacts with an infinite supporting line.
    ///
    /// The affine line predicate is represented by the homogeneous Bernstein
    /// numerator `w_i orient(line, P_i)`. Same-sign weights certify that the
    /// denominator has no affine pole. Every finite root remains a
    /// [`BezierParameter2`], including isolated irrational roots, and contact
    /// kind is certified from exact root-multiplicity parity.
    pub fn relation_to_line_with_contacts(
        &self,
        line: &LineSeg2,
        policy: &CurvePolicy,
    ) -> Classification<BezierLineContactRelation> {
        if let Classification::Uncertain(reason) = self.common_weight_sign(policy) {
            return Classification::Uncertain(reason);
        }
        let weighted_distances = self
            .control_points()
            .iter()
            .zip(self.weights())
            .map(|(point, weight)| orient2d_real_expr(line.start(), line.end(), point) * weight)
            .collect::<Vec<_>>();
        let sides = self
            .control_points()
            .iter()
            .map(|point| classify_oriented_line(line.start(), line.end(), point, policy))
            .collect::<Vec<_>>();
        if sides
            .iter()
            .all(|side| matches!(side, Classification::Decided(LineSide::On)))
        {
            return Classification::Decided(BezierLineContactRelation::OnSupportingLine);
        }
        for side in [LineSide::Left, LineSide::Right] {
            if sides.iter().all(
                |candidate| matches!(candidate, Classification::Decided(value) if *value == side),
            ) {
                return Classification::Decided(BezierLineContactRelation::ControlHullDisjoint {
                    side,
                });
            }
        }

        exact_line_contact_relation_from_bernstein_distances(weighted_distances, policy)
    }

    /// Returns complete exact point-incidence parameter evidence.
    ///
    /// The two homogeneous equations `Nx - xW = 0` and `Ny - yW = 0`
    /// reuse the curve's clone-shared power basis. Their polynomial GCD
    /// contains exactly the common parameter roots, which are returned as
    /// represented values or validated singleton Sturm isolators.
    pub fn point_incidence(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<RationalBezierPointIncidence2> {
        match self.point_incidence_classified(point, policy) {
            Ok(Classification::Decided(incidence)) => Ok(incidence),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                cause,
            )),
        }
    }

    fn point_incidence_classified(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBezierPointIncidence2>> {
        if let Classification::Uncertain(reason) = self.common_weight_sign(policy) {
            return Ok(Classification::Uncertain(reason));
        }
        if self.has_certified_injective_axis(policy) {
            for (parameter, endpoint) in [(Real::zero(), self.start()), (Real::one(), self.end())] {
                if is_zero(&endpoint.distance_squared(point), policy) == Some(true) {
                    return Ok(Classification::Decided(
                        RationalBezierPointIncidence2::Parameters(vec![BezierParameter2::Exact(
                            parameter,
                        )]),
                    ));
                }
            }
        }
        let x = match self.point_axis_polynomial(point.x(), Axis2::X, policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(error) => return Err(error),
        };
        let y = match self.point_axis_polynomial(point.y(), Axis2::Y, policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(error) => return Err(error),
        };
        let polynomial = match (x, y) {
            (None, None) => {
                return Ok(Classification::Decided(
                    RationalBezierPointIncidence2::EntireCurve,
                ));
            }
            (Some(polynomial), None) | (None, Some(polynomial)) => polynomial,
            (Some(first), Some(second)) => match first.greatest_common_divisor(&second, policy)? {
                Classification::Decided(Some(polynomial)) => polynomial,
                Classification::Decided(None) => {
                    return Ok(Classification::Decided(
                        RationalBezierPointIncidence2::Parameters(Vec::new()),
                    ));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        match polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(parameters) => Ok(Classification::Decided(
                RationalBezierPointIncidence2::Parameters(parameters),
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Classifies whether `point` lies on this finite rational Bezier.
    pub fn contains_point(&self, point: &Point2, policy: &CurvePolicy) -> ExactCurveResult<bool> {
        self.point_incidence(point, policy)
            .map(|incidence| match incidence {
                RationalBezierPointIncidence2::EntireCurve => true,
                RationalBezierPointIncidence2::Parameters(parameters) => !parameters.is_empty(),
            })
    }

    pub(crate) fn contains_point_classified(
        &self,
        point: &Point2,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        match self.point_incidence_classified(point, policy) {
            Ok(classification) => classification.map(|incidence| match incidence {
                RationalBezierPointIncidence2::EntireCurve => true,
                RationalBezierPointIncidence2::Parameters(parameters) => !parameters.is_empty(),
            }),
            Err(CurveError::Real(_)) => Classification::Uncertain(UncertaintyReason::RealSign),
            Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        }
    }

    /// Returns exact resultant candidates for all finite curve contacts.
    ///
    /// Homogeneous coordinate equations are eliminated in each parameter
    /// direction. Roots are retained as represented or algebraically isolated
    /// [`BezierParameter2`] values. The two projections are not paired or
    /// accepted as contacts until a later exact replay proves equal images.
    pub fn intersection_candidates(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<RationalBezierIntersectionCandidates2> {
        match self.intersection_candidates_classified(other, policy) {
            Ok(Classification::Decided(candidates)) => Ok(candidates),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                cause,
            )),
        }
    }

    /// Prepares one pair with typed failure context.
    pub fn try_prepare_intersection(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<PreparedRationalBezierIntersection2> {
        match self.prepare_intersection_classified(other, policy) {
            Ok(Classification::Decided(prepared)) => Ok(prepared),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                cause,
            )),
        }
    }

    fn prepare_intersection_classified(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<PreparedRationalBezierIntersection2>> {
        match self.intersection_candidates_classified(other, policy)? {
            Classification::Decided(candidates) => Ok(Classification::Decided(
                PreparedRationalBezierIntersection2 {
                    data: Rc::new(PreparedRationalBezierIntersectionData {
                        first: self.clone(),
                        second: other.clone(),
                        policy: policy.clone(),
                        candidates,
                        contacts: OnceCell::new(),
                        topology: OnceCell::new(),
                    }),
                },
            )),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    /// Replays all resultant projections into exact paired contacts.
    ///
    /// The result distinguishes complete contact sets from partial algebraic
    /// replay. No raw resultant root is accepted as a contact without exact
    /// equality of both constructed affine coordinates.
    pub fn intersection_contacts(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<RationalBezierIntersectionContacts2> {
        match self.intersection_contacts_classified(other, policy) {
            Ok(Classification::Decided(contacts)) => Ok(contacts),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                None,
                cause,
            )),
        }
    }

    pub(crate) fn intersection_contacts_classified(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        if matches!(
            self.shares_implicit_quadratic_conic(other, policy),
            Classification::Decided(true)
        ) {
            return self.replay_intersection_candidate_set(
                other,
                &RationalBezierIntersectionCandidates2::DegenerateResultant,
                policy,
            );
        }
        let candidates = match self.intersection_candidates_classified(other, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.replay_intersection_candidate_set(other, &candidates, policy)
    }

    fn replay_intersection_candidate_set(
        &self,
        other: &Self,
        candidates: &RationalBezierIntersectionCandidates2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        match candidates {
            RationalBezierIntersectionCandidates2::NoIntersection => Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )),
            RationalBezierIntersectionCandidates2::DegenerateResultant => {
                match self.image_overlap(other, policy) {
                    Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
                        overlap,
                    )) => Ok(Classification::Decided(
                        RationalBezierIntersectionContacts2::Overlap(overlap),
                    )),
                    Classification::Decided(RationalBezierSharedComponentReplay::Contacts(
                        contacts,
                    )) => {
                        let mut replayed = Vec::with_capacity(contacts.len());
                        for (first_parameter, second_parameter) in contacts {
                            let point = match self.point_at_classified(&first_parameter, policy) {
                                Classification::Decided(point) => point,
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            };
                            replayed.push(RationalBezierIntersectionContact2 {
                                first_parameter: BezierParameter2::Exact(first_parameter),
                                second_parameter: BezierParameter2::Exact(second_parameter),
                                point: RationalBezierIntersectionPointEvidence2::Exact(point),
                            });
                        }
                        Ok(Classification::Decided(if replayed.is_empty() {
                            RationalBezierIntersectionContacts2::NoIntersection
                        } else {
                            RationalBezierIntersectionContacts2::Contacts(replayed.into())
                        }))
                    }
                    Classification::Decided(RationalBezierSharedComponentReplay::Unresolved) => {
                        Ok(Classification::Decided(
                            RationalBezierIntersectionContacts2::DegenerateResultant,
                        ))
                    }
                    Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
                }
            }
            RationalBezierIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            } => self.replay_intersection_candidates(
                other,
                first_parameters,
                second_parameters,
                policy,
            ),
        }
    }

    fn intersection_candidates_classified(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBezierIntersectionCandidates2>> {
        // Same-sign control-hull bounds are only a rejection accelerator. An
        // unavailable sign or ordering certificate must fall through to the
        // homogeneous resultant, whose affine replay independently rejects
        // projective poles and out-of-domain roots.
        if matches!(self.common_weight_sign(policy), Classification::Decided(_))
            && matches!(other.common_weight_sign(policy), Classification::Decided(_))
            && let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
                self.certified_bounds_classified(policy),
                other.certified_bounds_classified(policy),
            )
            && matches!(
                first_bounds.overlaps(&second_bounds, policy),
                Classification::Decided(false)
            )
        {
            return Ok(Classification::Decided(
                RationalBezierIntersectionCandidates2::NoIntersection,
            ));
        }

        self.intersection_candidates_after_overlapping_bounds(other, policy)
    }

    fn intersection_candidates_after_overlapping_bounds(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBezierIntersectionCandidates2>> {
        match self.lineage_overlap(other, policy) {
            Classification::Decided(Some(_)) => {
                return Ok(Classification::Decided(
                    RationalBezierIntersectionCandidates2::DegenerateResultant,
                ));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
        let config = CurveIntersectionResultantConfig {
            min_precision: RATIONAL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_RATIONAL_INTERSECTION_RESULTANT_DEGREE,
        };
        let first = resultant_rational_parametric_curve_intersection(
            self.homogeneous_power_basis()?,
            other.homogeneous_power_basis()?,
            CurveResultantParameter::First,
            config,
        );
        let second = resultant_rational_parametric_curve_intersection(
            self.homogeneous_power_basis()?,
            other.homogeneous_power_basis()?,
            CurveResultantParameter::Second,
            config,
        );
        let first = match resultant_parameter_projection(first, policy)? {
            Classification::Decided(projection) => projection,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let second = match resultant_parameter_projection(second, policy)? {
            Classification::Decided(projection) => projection,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(match (first, second) {
            (ResultantParameterProjection::Empty, _) | (_, ResultantParameterProjection::Empty) => {
                RationalBezierIntersectionCandidates2::NoIntersection
            }
            (ResultantParameterProjection::Degenerate, _)
            | (_, ResultantParameterProjection::Degenerate) => {
                RationalBezierIntersectionCandidates2::DegenerateResultant
            }
            (
                ResultantParameterProjection::Parameters(first_parameters),
                ResultantParameterProjection::Parameters(second_parameters),
            ) => RationalBezierIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            },
        }))
    }

    /// Splits this curve exactly at one represented parameter.
    pub fn split_at_exact(
        &self,
        parameter: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<(Self, Self)>> {
        if in_closed_unit_interval(parameter, policy) != Some(true) {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        self.retain_root_image_injectivity(policy);
        if self.degree() == 2 {
            let _ = self.implicit_quadratic_conic(policy);
        }
        let mut level = self.homogeneous_controls().to_vec();
        let mut left = Vec::with_capacity(level.len());
        let mut right = Vec::with_capacity(level.len());
        left.push(level[0].clone());
        right.push(
            level
                .last()
                .expect("validated rational Bezier has controls")
                .clone(),
        );
        while level.len() > 1 {
            let next = level
                .windows(2)
                .map(|pair| pair[0].lerp(&pair[1], parameter))
                .collect::<Vec<_>>();
            left.push(next[0].clone());
            right.push(next.last().expect("de Casteljau level is nonempty").clone());
            level = next;
        }
        right.reverse();
        let left_lineage = self.data.lineage.subrange(&Real::zero(), parameter);
        let right_lineage = self.data.lineage.subrange(parameter, &Real::one());
        let left = match from_homogeneous(left, left_lineage, policy)? {
            Classification::Decided(curve) => curve,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let right = match from_homogeneous(right, right_lineage, policy)? {
            Classification::Decided(curve) => curve,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided((left, right)))
    }

    /// Materializes the exact subcurve over an ordered represented range.
    pub fn subcurve_between_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Self>> {
        if in_closed_unit_interval(start, policy) != Some(true)
            || in_closed_unit_interval(end, policy) != Some(true)
        {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        match crate::classify::compare_reals(start, end, policy) {
            Some(std::cmp::Ordering::Greater) | None => {
                return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
            }
            Some(std::cmp::Ordering::Equal) => {
                let point = match self.point_at_classified(start, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                self.retain_root_image_injectivity(policy);
                return Self::try_new_with_lineage(
                    vec![point; self.control_points().len()],
                    vec![Real::one(); self.weights().len()],
                    self.data.lineage.subrange(start, end),
                )
                .map(Classification::Decided);
            }
            Some(std::cmp::Ordering::Less) => {}
        }
        if crate::classify::compare_reals(start, &Real::zero(), policy)
            == Some(std::cmp::Ordering::Equal)
            && crate::classify::compare_reals(end, &Real::one(), policy)
                == Some(std::cmp::Ordering::Equal)
        {
            return Ok(Classification::Decided(self.clone()));
        }
        let (left, _) = match self.split_at_exact(end, policy)? {
            Classification::Decided(split) => split,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        if crate::classify::compare_reals(start, &Real::zero(), policy)
            == Some(std::cmp::Ordering::Equal)
        {
            return Ok(Classification::Decided(left));
        }
        let local_start = (start / end)?;
        match left.split_at_exact(&local_start, policy)? {
            Classification::Decided((_, middle)) => Ok(Classification::Decided(middle)),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    pub(crate) fn endpoint_derivatives(
        &self,
        at_end: bool,
        max_order: usize,
        policy: &CurvePolicy,
    ) -> Classification<Vec<(Real, Real)>> {
        let parameter = if at_end { Real::one() } else { Real::zero() };
        self.affine_derivative_values_at(&parameter, max_order, policy)
    }

    fn affine_derivative_values_at(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurvePolicy,
    ) -> Classification<Vec<(Real, Real)>> {
        if in_closed_unit_interval(parameter, policy) != Some(true) {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        let Ok(power_basis) = self.homogeneous_power_basis() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some(numerator_x) =
            evaluate_power_polynomial_derivatives(&power_basis.x_numerator, parameter, max_order)
        else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some(numerator_y) =
            evaluate_power_polynomial_derivatives(&power_basis.y_numerator, parameter, max_order)
        else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some(denominator) =
            evaluate_power_polynomial_derivatives(&power_basis.weight, parameter, max_order)
        else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        match is_zero(&denominator[0], policy) {
            Some(false) => {}
            Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }

        let Some(value_count) = max_order.checked_add(1) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let mut derivatives: Vec<(Real, Real)> = Vec::new();
        if derivatives.try_reserve_exact(value_count).is_err() {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        }
        for derivative_order in 0..=max_order {
            let mut x = numerator_x[derivative_order].clone();
            let mut y = numerator_y[derivative_order].clone();
            for denominator_order in 1..=derivative_order {
                let Some(coefficient) = checked_binomial(derivative_order, denominator_order)
                else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let coefficient = Real::from(coefficient);
                let previous = &derivatives[derivative_order - denominator_order];
                x -= &coefficient * &denominator[denominator_order] * &previous.0;
                y -= &coefficient * &denominator[denominator_order] * &previous.1;
            }
            let Ok(x) = x / &denominator[0] else {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            };
            let Ok(y) = y / &denominator[0] else {
                return Classification::Uncertain(UncertaintyReason::Boundary);
            };
            derivatives.push((x, y));
        }
        Classification::Decided(derivatives)
    }

    /// Returns this curve with traversal direction reversed.
    pub fn reversed(&self) -> Self {
        let mut control_points = self.control_points().to_vec();
        let mut weights = self.weights().to_vec();
        control_points.reverse();
        weights.reverse();
        Self::try_new_with_lineage(control_points, weights, self.data.lineage.reversed())
            .expect("reversing a valid rational Bezier is valid")
    }

    fn homogeneous_controls(&self) -> &[HomogeneousPoint2] {
        self.data.homogeneous_controls.get_or_init(|| {
            self.control_points()
                .iter()
                .zip(self.weights())
                .map(|(point, weight)| HomogeneousPoint2 {
                    x: point.x() * weight,
                    y: point.y() * weight,
                    weight: weight.clone(),
                })
                .collect()
        })
    }

    fn homogeneous_power_basis(&self) -> CurveResult<&RationalParametricCurve2> {
        if let Some(power_basis) = self.data.homogeneous_power_basis.get() {
            return Ok(power_basis);
        }
        let homogeneous = self.homogeneous_controls();
        let x = bernstein_to_power_coefficients(
            homogeneous.iter().map(|point| point.x.clone()).collect(),
        )?;
        let y = bernstein_to_power_coefficients(
            homogeneous.iter().map(|point| point.y.clone()).collect(),
        )?;
        let weight = bernstein_to_power_coefficients(
            homogeneous
                .iter()
                .map(|point| point.weight.clone())
                .collect(),
        )?;
        let _ = self
            .data
            .homogeneous_power_basis
            .set(RationalParametricCurve2::new(x, y, weight));
        Ok(self
            .data
            .homogeneous_power_basis
            .get()
            .expect("homogeneous power basis was initialized"))
    }

    fn axis_derivative_numerator_bernstein(&self, axis: Axis2) -> Option<&[Real]> {
        let cache = match axis {
            Axis2::X => &self.data.x_derivative_numerator_bernstein,
            Axis2::Y => &self.data.y_derivative_numerator_bernstein,
        };
        cache
            .get_or_init(|| self.compute_axis_derivative_numerator_bernstein(axis))
            .as_deref()
    }

    fn compute_axis_derivative_numerator_bernstein(&self, axis: Axis2) -> Option<Vec<Real>> {
        let degree = self.degree();
        let derivative_degree = degree.checked_sub(1)?;
        let product_degree = degree.checked_add(derivative_degree)?;
        let degree_scale = Real::from(u64::try_from(degree).ok()?);
        let weighted_coordinates = self
            .homogeneous_controls()
            .iter()
            .map(|point| {
                match axis {
                    Axis2::X => &point.x,
                    Axis2::Y => &point.y,
                }
                .clone()
            })
            .collect::<Vec<_>>();
        let coordinate_derivative = weighted_coordinates
            .windows(2)
            .map(|pair| &degree_scale * (&pair[1] - &pair[0]))
            .collect::<Vec<_>>();
        let weight_derivative = self
            .weights()
            .windows(2)
            .map(|pair| &degree_scale * (&pair[1] - &pair[0]))
            .collect::<Vec<_>>();
        let mut coefficients = Vec::with_capacity(product_degree + 1);
        for product_index in 0..=product_degree {
            let mut coefficient = Real::zero();
            let derivative_start = product_index.saturating_sub(degree);
            let derivative_end = derivative_degree.min(product_index);
            for (derivative_index, derivative_coordinate) in coordinate_derivative
                .iter()
                .enumerate()
                .take(derivative_end + 1)
                .skip(derivative_start)
            {
                let coordinate_index = product_index - derivative_index;
                let scale = checked_binomial(derivative_degree, derivative_index)?
                    .checked_mul(checked_binomial(degree, coordinate_index)?)?;
                coefficient +=
                    Real::from(scale) * derivative_coordinate * &self.weights()[coordinate_index];
            }
            let coordinate_start = product_index.saturating_sub(derivative_degree);
            let coordinate_end = degree.min(product_index);
            for (coordinate_index, weighted_coordinate) in weighted_coordinates
                .iter()
                .enumerate()
                .take(coordinate_end + 1)
                .skip(coordinate_start)
            {
                let derivative_index = product_index - coordinate_index;
                let scale = checked_binomial(degree, coordinate_index)?
                    .checked_mul(checked_binomial(derivative_degree, derivative_index)?)?;
                coefficient -=
                    Real::from(scale) * weighted_coordinate * &weight_derivative[derivative_index];
            }
            let basis_scale = Real::from(checked_binomial(product_degree, product_index)?);
            coefficients.push((coefficient / basis_scale).ok()?);
        }
        Some(coefficients)
    }

    fn point_axis_polynomial(
        &self,
        target: &Real,
        axis: Axis2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<BezierParameterPolynomial>>> {
        let power_basis = self.homogeneous_power_basis()?;
        let coordinate = match axis {
            Axis2::X => &power_basis.x_numerator,
            Axis2::Y => &power_basis.y_numerator,
        };
        let coefficients = coordinate
            .iter()
            .zip(&power_basis.weight)
            .map(|(coordinate, weight)| coordinate - target * weight)
            .collect::<Vec<_>>();
        if coefficients
            .iter()
            .all(|control| is_zero(control, policy) == Some(true))
        {
            return Ok(Classification::Decided(None));
        }
        BezierParameterPolynomial::try_new_power_basis(coefficients, policy)
            .map(|polynomial| polynomial.map(Some))
    }

    fn replay_intersection_candidates(
        &self,
        other: &Self,
        first_parameters: &[BezierParameter2],
        second_parameters: &[BezierParameter2],
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        let first_replays = first_parameters
            .iter()
            .map(|parameter| self.candidate_point_replay(parameter, policy))
            .collect::<CurveResult<Vec<_>>>()?;
        let second_replays = second_parameters
            .iter()
            .map(|parameter| other.candidate_point_replay(parameter, policy))
            .collect::<CurveResult<Vec<_>>>()?;
        let mut incomplete =
            first_replays.iter().any(Option::is_none) || second_replays.iter().any(Option::is_none);
        let mut contacts = Vec::new();
        for (first_index, first_replay) in first_replays.iter().enumerate() {
            let Some(first_replay) = first_replay else {
                continue;
            };
            for (second_index, second_replay) in second_replays.iter().enumerate() {
                let Some(second_replay) = second_replay else {
                    continue;
                };
                match candidate_points_equal(first_replay, second_replay, policy) {
                    Some(true) => contacts.push(RationalBezierIntersectionContact2 {
                        first_parameter: first_parameters[first_index].clone(),
                        second_parameter: second_parameters[second_index].clone(),
                        point: first_replay.evidence.clone(),
                    }),
                    Some(false) => {}
                    None => match self.parameter_pair_same_point_by_incidence(
                        other,
                        &first_parameters[first_index],
                        &second_parameters[second_index],
                        policy,
                    )? {
                        Classification::Decided(true) => {
                            contacts.push(RationalBezierIntersectionContact2 {
                                first_parameter: first_parameters[first_index].clone(),
                                second_parameter: second_parameters[second_index].clone(),
                                point: first_replay.evidence.clone(),
                            });
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(_) => incomplete = true,
                    },
                }
            }
        }
        if incomplete {
            return Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::Incomplete {
                    contacts: contacts.into(),
                    candidates: RationalBezierIntersectionCandidates2::Candidates {
                        first_parameters: first_parameters.to_vec(),
                        second_parameters: second_parameters.to_vec(),
                    },
                },
            ));
        }
        if contacts.is_empty() {
            Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            ))
        } else {
            Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::Contacts(contacts.into()),
            ))
        }
    }

    fn candidate_point_replay(
        &self,
        parameter: &BezierParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Option<CandidatePointReplay>> {
        match parameter {
            BezierParameter2::Exact(parameter) => {
                let point = match self.point_at_classified(parameter, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(_) => return Ok(None),
                };
                Ok(Some(CandidatePointReplay {
                    x: exact_real_algebraic_representation(point.x()),
                    y: exact_real_algebraic_representation(point.y()),
                    evidence: RationalBezierIntersectionPointEvidence2::Exact(point),
                }))
            }
            BezierParameter2::Algebraic(parameter) => {
                let image = self.point_at_algebraic_parameter(parameter, policy)?;
                let (Some(x), Some(y)) = (
                    image.x().and_then(|coordinate| coordinate.representation()),
                    image.y().and_then(|coordinate| coordinate.representation()),
                ) else {
                    return Ok(None);
                };
                Ok(Some(CandidatePointReplay {
                    x: x.clone(),
                    y: y.clone(),
                    evidence: RationalBezierIntersectionPointEvidence2::Algebraic(image),
                }))
            }
        }
    }

    fn parameter_pair_same_point_by_incidence(
        &self,
        other: &Self,
        first: &BezierParameter2,
        second: &BezierParameter2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        let (point, curve, target) = match (first.as_exact(), second.as_exact()) {
            (Some(parameter), _) => {
                let point = match self.point_at_classified(parameter, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (point, other, second)
            }
            (None, Some(parameter)) => {
                let point = match other.point_at_classified(parameter, policy) {
                    Classification::Decided(point) => point,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                (point, self, first)
            }
            (None, None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        };
        match curve.point_incidence_classified(&point, policy)? {
            Classification::Decided(RationalBezierPointIncidence2::EntireCurve) => {
                Ok(Classification::Decided(true))
            }
            Classification::Decided(RationalBezierPointIncidence2::Parameters(parameters)) => {
                let mut uncertain = None;
                for parameter in parameters {
                    match parameter.same_value(target, policy)? {
                        Classification::Decided(true) => {
                            return Ok(Classification::Decided(true));
                        }
                        Classification::Decided(false) => {}
                        Classification::Uncertain(reason) => uncertain = Some(reason),
                    }
                }
                Ok(uncertain.map_or(Classification::Decided(false), Classification::Uncertain))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    fn common_weight_sign(&self, policy: &CurvePolicy) -> Classification<RealSign> {
        let Some(first) = real_sign(&self.weights()[0], policy) else {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        };
        if first == RealSign::Zero {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        }
        for weight in &self.weights()[1..] {
            match real_sign(weight, policy) {
                Some(sign) if sign == first => {}
                Some(RealSign::Zero) => {
                    return Classification::Uncertain(UncertaintyReason::Boundary);
                }
                Some(_) => return Classification::Uncertain(UncertaintyReason::Boundary),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        Classification::Decided(first)
    }

    fn same_projective_control_net(
        &self,
        other: &Self,
        reversed: bool,
        policy: &CurvePolicy,
    ) -> Option<bool> {
        if self.degree() != other.degree() {
            return Some(false);
        }
        let degree = self.degree();
        let other_base = if reversed { degree } else { 0 };
        for index in 0..=degree {
            let other_index = if reversed { degree - index } else { index };
            if !is_zero(
                &self.control_points()[index]
                    .distance_squared(&other.control_points()[other_index]),
                policy,
            )? || !is_zero(
                &(&self.weights()[index] * &other.weights()[other_base]
                    - &other.weights()[other_index] * &self.weights()[0]),
                policy,
            )? {
                return Some(false);
            }
        }
        Some(true)
    }

    fn same_projective_control_net_degree_aligned(
        &self,
        other: &Self,
        reversed: bool,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        let comparison = match self.degree().cmp(&other.degree()) {
            std::cmp::Ordering::Equal => self.same_projective_control_net(other, reversed, policy),
            std::cmp::Ordering::Less => match self.elevated_to_degree(other.degree()) {
                Ok(elevated) => elevated.same_projective_control_net(other, reversed, policy),
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Classification::Uncertain(blocker.reason());
                }
                Err(ExactCurveError::Invalid {
                    cause: CurveError::Real(_),
                    ..
                }) => return Classification::Uncertain(UncertaintyReason::RealSign),
                Err(ExactCurveError::Invalid { .. }) => {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                }
            },
            std::cmp::Ordering::Greater => match other.elevated_to_degree(self.degree()) {
                Ok(elevated) => self.same_projective_control_net(&elevated, reversed, policy),
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Classification::Uncertain(blocker.reason());
                }
                Err(ExactCurveError::Invalid {
                    cause: CurveError::Real(_),
                    ..
                }) => return Classification::Uncertain(UncertaintyReason::RealSign),
                Err(ExactCurveError::Invalid { .. }) => {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                }
            },
        };
        comparison.map_or_else(
            || Classification::Uncertain(UncertaintyReason::RealSign),
            Classification::Decided,
        )
    }

    fn image_overlap(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Classification<RationalBezierSharedComponentReplay> {
        match self.lineage_overlap(other, policy) {
            Classification::Decided(Some(overlap)) => {
                return Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
                    overlap,
                ));
            }
            Classification::Decided(None) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        if matches!(
            self.shares_implicit_quadratic_conic(other, policy),
            Classification::Decided(true)
        ) {
            return self.partial_image_overlap(other, policy);
        }
        match self.same_projective_control_net_degree_aligned(other, false, policy) {
            Classification::Decided(true) => {
                return Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
                    RationalBezierIntersectionOverlap2 {
                        first_range: BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                        second_range: BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                        orientation: RationalBezierOverlapOrientation2::Same,
                    },
                ));
            }
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        match self.same_projective_control_net_degree_aligned(other, true, policy) {
            Classification::Decided(true) => Classification::Decided(
                RationalBezierSharedComponentReplay::Overlap(RationalBezierIntersectionOverlap2 {
                    first_range: BezierParameterRange2::from_exact(Real::zero(), Real::one()),
                    second_range: BezierParameterRange2::from_exact(Real::one(), Real::zero()),
                    orientation: RationalBezierOverlapOrientation2::Reversed,
                }),
            ),
            Classification::Decided(false) => self.partial_image_overlap(other, policy),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        }
    }

    fn lineage_overlap(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Classification<Option<RationalBezierIntersectionOverlap2>> {
        if !Rc::ptr_eq(&self.data.lineage.root, &other.data.lineage.root) {
            return Classification::Decided(None);
        }
        self.retain_root_image_injectivity(policy);
        other.retain_root_image_injectivity(policy);
        if self.data.lineage.root.image_is_injective.get() != Some(&true) {
            return Classification::Decided(None);
        }

        oriented_param_range_overlap(&self.data.lineage.range, &other.data.lineage.range, policy)
            .map(|overlap| {
                overlap.map(|overlap| RationalBezierIntersectionOverlap2 {
                    first_range: BezierParameterRange2::from_exact(
                        overlap.first.start().clone(),
                        overlap.first.end().clone(),
                    ),
                    second_range: BezierParameterRange2::from_exact(
                        overlap.second.start().clone(),
                        overlap.second.end().clone(),
                    ),
                    orientation: if overlap.same_orientation {
                        RationalBezierOverlapOrientation2::Same
                    } else {
                        RationalBezierOverlapOrientation2::Reversed
                    },
                })
            })
    }

    fn retain_root_image_injectivity(&self, policy: &CurvePolicy) {
        if self.data.lineage.root.image_is_injective.get().is_some() {
            return;
        }
        let range = &self.data.lineage.range;
        let covers_root_domain = (compare_reals(range.start(), &Real::zero(), policy)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(range.end(), &Real::one(), policy) == Some(std::cmp::Ordering::Equal))
            || (compare_reals(range.start(), &Real::one(), policy)
                == Some(std::cmp::Ordering::Equal)
                && compare_reals(range.end(), &Real::zero(), policy)
                    == Some(std::cmp::Ordering::Equal));
        if covers_root_domain && self.has_certified_injective_axis(policy) {
            let _ = self.data.lineage.root.image_is_injective.set(true);
        }
    }

    fn partial_image_overlap(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Classification<RationalBezierSharedComponentReplay> {
        let shared_quadratic_conic = match self.shares_implicit_quadratic_conic(other, policy) {
            Classification::Decided(shared) => shared,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        if !shared_quadratic_conic {
            match self.certified_line_image_overlap(other, policy) {
                Classification::Decided(Some(overlap)) => {
                    return Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
                        overlap,
                    ));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => return Classification::Uncertain(reason),
            }
        }
        let mut contacts = Vec::with_capacity(4);
        for (first_parameter, point) in [
            (Real::zero(), self.start().clone()),
            (Real::one(), self.end().clone()),
        ] {
            if shared_quadratic_conic {
                match shared_conic_endpoint_parameters(self, &first_parameter, other, policy) {
                    Classification::Decided(Some(parameters)) => {
                        for second_parameter in parameters {
                            push_unique_parameter_overlap_contact(
                                &mut contacts,
                                BezierParameter2::Exact(first_parameter.clone()),
                                second_parameter,
                            );
                        }
                        continue;
                    }
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
            match other.point_incidence_classified(&point, policy) {
                Err(CurveError::Real(_)) => {
                    return Classification::Uncertain(UncertaintyReason::RealSign);
                }
                Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
                Ok(Classification::Decided(RationalBezierPointIncidence2::Parameters(
                    parameters,
                ))) => {
                    for second_parameter in parameters {
                        push_unique_parameter_overlap_contact(
                            &mut contacts,
                            BezierParameter2::Exact(first_parameter.clone()),
                            second_parameter,
                        );
                    }
                }
                Ok(Classification::Decided(RationalBezierPointIncidence2::EntireCurve)) => {
                    return Classification::Decided(
                        RationalBezierSharedComponentReplay::Unresolved,
                    );
                }
                Ok(Classification::Uncertain(reason)) => {
                    return Classification::Uncertain(reason);
                }
            }
        }
        for (second_parameter, point) in [
            (Real::zero(), other.start().clone()),
            (Real::one(), other.end().clone()),
        ] {
            if shared_quadratic_conic {
                match shared_conic_endpoint_parameters(other, &second_parameter, self, policy) {
                    Classification::Decided(Some(parameters)) => {
                        for first_parameter in parameters {
                            push_unique_parameter_overlap_contact(
                                &mut contacts,
                                first_parameter,
                                BezierParameter2::Exact(second_parameter.clone()),
                            );
                        }
                        continue;
                    }
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
            match self.point_incidence_classified(&point, policy) {
                Err(CurveError::Real(_)) => {
                    return Classification::Uncertain(UncertaintyReason::RealSign);
                }
                Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
                Ok(Classification::Decided(RationalBezierPointIncidence2::Parameters(
                    parameters,
                ))) => {
                    for first_parameter in parameters {
                        push_unique_parameter_overlap_contact(
                            &mut contacts,
                            first_parameter,
                            BezierParameter2::Exact(second_parameter.clone()),
                        );
                    }
                }
                Ok(Classification::Decided(RationalBezierPointIncidence2::EntireCurve)) => {
                    return Classification::Decided(
                        RationalBezierSharedComponentReplay::Unresolved,
                    );
                }
                Ok(Classification::Uncertain(reason)) => {
                    return Classification::Uncertain(reason);
                }
            }
        }

        if shared_quadratic_conic {
            match overlap_from_parameter_contacts(&contacts, policy) {
                Classification::Decided(Some(overlap)) => {
                    return Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
                        overlap,
                    ));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Classification::Uncertain(reason);
                }
            }
        }

        match self.certified_polynomial_graph_component(other, policy) {
            Ok(Classification::Decided(true)) => {
                match overlap_from_parameter_contacts(&contacts, policy) {
                    Classification::Decided(Some(overlap)) => {
                        return Classification::Decided(
                            RationalBezierSharedComponentReplay::Overlap(overlap),
                        );
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
            Ok(Classification::Decided(false)) => {}
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        }

        let mut overlap = None;
        for first_index in 0..contacts.len() {
            for second_index in first_index + 1..contacts.len() {
                let candidate = match self.overlap_between_contacts(
                    other,
                    &contacts[first_index],
                    &contacts[second_index],
                    policy,
                ) {
                    Classification::Decided(candidate) => candidate,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
                let Some(candidate) = candidate else {
                    continue;
                };
                if overlap.is_some() {
                    return Classification::Decided(
                        RationalBezierSharedComponentReplay::Unresolved,
                    );
                }
                overlap = Some(candidate);
            }
        }
        if let Some(overlap) = overlap {
            return Classification::Decided(RationalBezierSharedComponentReplay::Overlap(overlap));
        }
        if self.has_certified_injective_axis(policy) && other.has_certified_injective_axis(policy) {
            let represented = contacts
                .iter()
                .map(|(first, second)| {
                    Some((first.as_exact()?.clone(), second.as_exact()?.clone()))
                })
                .collect::<Option<Vec<_>>>();
            represented.map_or_else(
                || Classification::Decided(RationalBezierSharedComponentReplay::Unresolved),
                |contacts| {
                    Classification::Decided(RationalBezierSharedComponentReplay::Contacts(contacts))
                },
            )
        } else {
            Classification::Decided(RationalBezierSharedComponentReplay::Unresolved)
        }
    }

    fn certified_line_image_overlap(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Classification<Option<RationalBezierIntersectionOverlap2>> {
        let (first_line, second_line) = match (
            self.fit_exact_line_image(policy),
            other.fit_exact_line_image(policy),
        ) {
            (
                Ok(Classification::Decided(BezierLineImageFitRelation::Fit(first))),
                Ok(Classification::Decided(BezierLineImageFitRelation::Fit(second))),
            ) => (first, second),
            (Ok(Classification::Uncertain(reason)), _)
            | (_, Ok(Classification::Uncertain(reason))) => {
                return Classification::Uncertain(reason);
            }
            (Err(CurveError::Real(_)), _) | (_, Err(CurveError::Real(_))) => {
                return Classification::Uncertain(UncertaintyReason::RealSign);
            }
            (Err(_), _) | (_, Err(_)) => {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            }
            _ => return Classification::Decided(None),
        };
        if !self.has_certified_injective_axis(policy) || !other.has_certified_injective_axis(policy)
        {
            return Classification::Decided(None);
        }
        let intersection = match first_line.line().intersect_line(second_line.line(), policy) {
            Ok(intersection) => intersection,
            Err(CurveError::Real(_)) => {
                return Classification::Uncertain(UncertaintyReason::RealSign);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let crate::LineLineIntersection::Overlap { segment, .. } = intersection else {
            return match intersection {
                crate::LineLineIntersection::Uncertain { reason } => {
                    Classification::Uncertain(reason)
                }
                _ => Classification::Decided(None),
            };
        };
        let first_start = match unique_point_incidence_parameter(self, segment.start(), policy) {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => return Classification::Decided(None),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let first_end = match unique_point_incidence_parameter(self, segment.end(), policy) {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => return Classification::Decided(None),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let second_start = match unique_point_incidence_parameter(other, segment.start(), policy) {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => return Classification::Decided(None),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let second_end = match unique_point_incidence_parameter(other, segment.end(), policy) {
            Classification::Decided(Some(parameter)) => parameter,
            Classification::Decided(None) => return Classification::Decided(None),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let first_order = match first_start.cmp_by_interval(&first_end, policy) {
            Ok(Classification::Decided(ordering)) => ordering,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        if first_order.is_eq() {
            return Classification::Decided(None);
        }
        let (first_start, first_end, second_start, second_end) = if first_order.is_lt() {
            (first_start, first_end, second_start, second_end)
        } else {
            (first_end, first_start, second_end, second_start)
        };
        let second_order = match second_start.cmp_by_interval(&second_end, policy) {
            Ok(Classification::Decided(ordering)) => ordering,
            Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        if second_order.is_eq() {
            return Classification::Decided(None);
        }
        Classification::Decided(Some(RationalBezierIntersectionOverlap2 {
            first_range: BezierParameterRange2::new_validated(first_start, first_end),
            second_range: BezierParameterRange2::new_validated(second_start, second_end),
            orientation: if second_order.is_lt() {
                RationalBezierOverlapOrientation2::Same
            } else {
                RationalBezierOverlapOrientation2::Reversed
            },
        }))
    }

    pub(crate) fn has_certified_injective_axis(&self, policy: &CurvePolicy) -> bool {
        for axis in [Axis2::X, Axis2::Y] {
            if self.has_certified_injective_axis_on(axis, policy) {
                return true;
            }
        }
        false
    }

    fn has_certified_injective_axis_on(&self, axis: Axis2, policy: &CurvePolicy) -> bool {
        if !matches!(
            self.axis_monotonicity_classified(axis, policy),
            Ok(Classification::Decided(true))
        ) {
            return false;
        }
        let (start, end) = match axis {
            Axis2::X => (self.start().x(), self.end().x()),
            Axis2::Y => (self.start().y(), self.end().y()),
        };
        // A one-signed Bernstein derivative with distinct endpoint coordinates
        // is strictly monotone on the open domain.
        compare_reals(start, end, policy).is_some_and(|ordering| !ordering.is_eq())
    }

    fn certified_polynomial_graph_component(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        for (base, candidate) in [(self, other), (other, self)] {
            for axis in [Axis2::X, Axis2::Y] {
                let graph = match base.polynomial_graph(axis, policy)? {
                    Classification::Decided(Some(graph)) => graph,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                if !candidate.has_certified_injective_axis_on(axis, policy) {
                    continue;
                }
                match graph.contains_curve(candidate, policy)? {
                    Classification::Decided(true) => {
                        return Ok(Classification::Decided(true));
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        Ok(Classification::Decided(false))
    }

    fn shares_implicit_quadratic_conic(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> Classification<bool> {
        let first = match self.implicit_quadratic_conic(policy) {
            Classification::Decided(Some(coefficients)) => coefficients,
            Classification::Decided(None) => return Classification::Decided(false),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let second = match other.implicit_quadratic_conic(policy) {
            Classification::Decided(Some(coefficients)) => coefficients,
            Classification::Decided(None) => return Classification::Decided(false),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let mut uncertain = false;
        for first_index in 0..first.len() {
            for second_index in first_index + 1..first.len() {
                match is_zero(
                    &(&first[first_index] * &second[second_index]
                        - &first[second_index] * &second[first_index]),
                    policy,
                ) {
                    Some(true) => {}
                    Some(false) => return Classification::Decided(false),
                    None => uncertain = true,
                }
            }
        }
        if uncertain {
            Classification::Uncertain(UncertaintyReason::RealSign)
        } else {
            Classification::Decided(true)
        }
    }

    fn implicit_quadratic_conic(&self, policy: &CurvePolicy) -> Classification<Option<&[Real; 6]>> {
        if self.degree() != 2 {
            return Classification::Decided(None);
        }
        self.retain_quadratic_conic_parameter_frame(policy);
        if let Some(coefficients) = self.data.lineage.root.implicit_quadratic_conic.get() {
            return Classification::Decided(Some(coefficients));
        }
        let controls = quadratic_conic_parameter_frame(self);
        let first = homogeneous_control_vector(&controls[0]);
        let middle = homogeneous_control_vector(&controls[1]);
        let last = homogeneous_control_vector(&controls[2]);
        let lambda_0 = cross3(&middle, &last);
        let lambda_1 = cross3(&last, &first);
        let lambda_2 = cross3(&first, &middle);
        let determinant = dot3(&first, &lambda_0);
        match is_zero(&determinant, policy) {
            Some(false) => {}
            Some(true) => return Classification::Decided(None),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
        let two = Real::from(2_i8);
        let four = Real::from(4_i8);
        let coefficients = [
            &lambda_1[0] * &lambda_1[0] - &four * &lambda_0[0] * &lambda_2[0],
            &two * &lambda_1[0] * &lambda_1[1]
                - &four * (&lambda_0[0] * &lambda_2[1] + &lambda_0[1] * &lambda_2[0]),
            &lambda_1[1] * &lambda_1[1] - &four * &lambda_0[1] * &lambda_2[1],
            &two * &lambda_1[0] * &lambda_1[2]
                - &four * (&lambda_0[0] * &lambda_2[2] + &lambda_0[2] * &lambda_2[0]),
            &two * &lambda_1[1] * &lambda_1[2]
                - &four * (&lambda_0[1] * &lambda_2[2] + &lambda_0[2] * &lambda_2[1]),
            &lambda_1[2] * &lambda_1[2] - &four * &lambda_0[2] * &lambda_2[2],
        ];
        let _ = self
            .data
            .lineage
            .root
            .implicit_quadratic_conic
            .set(Rc::new(coefficients));
        Classification::Decided(Some(
            self.data
                .lineage
                .root
                .implicit_quadratic_conic
                .get()
                .expect("decided implicit conic was retained"),
        ))
    }

    fn retain_quadratic_conic_parameter_frame(&self, policy: &CurvePolicy) {
        let root = &self.data.lineage.root;
        if self.degree() != 2 || root.quadratic_conic_parameter_frame.get().is_some() {
            return;
        }
        let range = self.source_parameter_range();
        let forward = compare_reals(range.start(), &Real::zero(), policy)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(range.end(), &Real::one(), policy) == Some(std::cmp::Ordering::Equal);
        let reversed = compare_reals(range.start(), &Real::one(), policy)
            == Some(std::cmp::Ordering::Equal)
            && compare_reals(range.end(), &Real::zero(), policy) == Some(std::cmp::Ordering::Equal);
        if !forward && !reversed {
            return;
        }
        let controls = self.homogeneous_controls();
        let frame = if forward {
            [
                controls[0].clone(),
                controls[1].clone(),
                controls[2].clone(),
            ]
        } else {
            [
                controls[2].clone(),
                controls[1].clone(),
                controls[0].clone(),
            ]
        };
        let _ = root.quadratic_conic_parameter_frame.set(Rc::new(frame));
    }

    fn polynomial_graph(
        &self,
        axis: Axis2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<Option<PolynomialGraph2>>> {
        let basis = self.homogeneous_power_basis()?;
        if !matches!(self.common_weight_sign(policy), Classification::Decided(_)) {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        if basis.weight.is_empty() || is_zero(&basis.weight[0], policy) != Some(false) {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        for coefficient in basis.weight.iter().skip(1) {
            match is_zero(coefficient, policy) {
                Some(true) => {}
                Some(false) => return Ok(Classification::Decided(None)),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let (axis_numerator, dependent_numerator) = match axis {
            Axis2::X => (&basis.x_numerator, &basis.y_numerator),
            Axis2::Y => (&basis.y_numerator, &basis.x_numerator),
        };
        let origin = (&axis_numerator[0] / &basis.weight[0])?;
        let scale = if axis_numerator.len() > 1 {
            (&axis_numerator[1] / &basis.weight[0])?
        } else {
            Real::zero()
        };
        match is_zero(&scale, policy) {
            Some(false) => {}
            Some(true) => return Ok(Classification::Decided(None)),
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        }
        for coefficient in axis_numerator.iter().skip(2) {
            match is_zero(coefficient, policy) {
                Some(true) => {}
                Some(false) => return Ok(Classification::Decided(None)),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        let dependent = dependent_numerator
            .iter()
            .map(|coefficient| coefficient / &basis.weight[0])
            .collect::<Result<Vec<_>, _>>()?;
        let dependent = match trim_power_polynomial(dependent, policy) {
            Classification::Decided(dependent) => dependent,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        Ok(Classification::Decided(Some(PolynomialGraph2 {
            axis,
            origin,
            scale,
            dependent,
        })))
    }

    fn overlap_between_contacts(
        &self,
        other: &Self,
        first_contact: &(BezierParameter2, BezierParameter2),
        second_contact: &(BezierParameter2, BezierParameter2),
        policy: &CurvePolicy,
    ) -> Classification<Option<RationalBezierIntersectionOverlap2>> {
        let (
            Some(first_exact),
            Some(second_exact),
            Some(other_first_exact),
            Some(other_second_exact),
        ) = (
            first_contact.0.as_exact(),
            second_contact.0.as_exact(),
            first_contact.1.as_exact(),
            second_contact.1.as_exact(),
        )
        else {
            return Classification::Decided(None);
        };
        let Some(first_order) = compare_reals(first_exact, second_exact, policy) else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        let Some(second_order) = compare_reals(other_first_exact, other_second_exact, policy)
        else {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        };
        if first_order.is_eq() || second_order.is_eq() {
            return Classification::Decided(None);
        }
        let (first_start, first_end) = if first_order.is_lt() {
            (first_exact, second_exact)
        } else {
            (second_exact, first_exact)
        };
        let (second_start, second_end) = if second_order.is_lt() {
            (other_first_exact, other_second_exact)
        } else {
            (other_second_exact, other_first_exact)
        };
        let first_subcurve = match self.subcurve_between_exact(first_start, first_end, policy) {
            Ok(Classification::Decided(curve)) => curve,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let second_subcurve = match other.subcurve_between_exact(second_start, second_end, policy) {
            Ok(Classification::Decided(curve)) => curve,
            Ok(Classification::Uncertain(reason)) => {
                return Classification::Uncertain(reason);
            }
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
        let reversed = first_order != second_order;
        match first_subcurve.same_projective_control_net_degree_aligned(
            &second_subcurve,
            reversed,
            policy,
        ) {
            Classification::Decided(true) => {}
            Classification::Decided(false) => return Classification::Decided(None),
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
        let orientation = if reversed {
            RationalBezierOverlapOrientation2::Reversed
        } else {
            RationalBezierOverlapOrientation2::Same
        };
        let second_range = if reversed {
            ParamRange::new(second_end.clone(), second_start.clone())
        } else {
            ParamRange::new(second_start.clone(), second_end.clone())
        };
        Classification::Decided(Some(RationalBezierIntersectionOverlap2 {
            first_range: BezierParameterRange2::from_exact(first_start.clone(), first_end.clone()),
            second_range: BezierParameterRange2::from_exact(
                second_range.start().clone(),
                second_range.end().clone(),
            ),
            orientation,
        }))
    }
}

impl PolynomialGraph2 {
    fn contains_curve(
        &self,
        curve: &RationalBezier2,
        policy: &CurvePolicy,
    ) -> CurveResult<Classification<bool>> {
        if !matches!(curve.common_weight_sign(policy), Classification::Decided(_)) {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        let basis = curve.homogeneous_power_basis()?;
        let (axis_numerator, dependent_numerator) = match self.axis {
            Axis2::X => (&basis.x_numerator, &basis.y_numerator),
            Axis2::Y => (&basis.y_numerator, &basis.x_numerator),
        };
        let axis_offset = subtract_power_polynomials(
            axis_numerator,
            &scale_power_polynomial(&basis.weight, &self.origin),
        );
        let scaled_weight = scale_power_polynomial(&basis.weight, &self.scale);
        let degree = self.dependent.len() - 1;
        let Some(axis_powers) = power_polynomial_sequence(&axis_offset, degree) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(weight_powers) = power_polynomial_sequence(&scaled_weight, degree) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut substituted = vec![Real::zero()];
        for (power, coefficient) in self.dependent.iter().enumerate() {
            let Some(term) =
                multiply_power_polynomials(&axis_powers[power], &weight_powers[degree - power])
            else {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            };
            add_scaled_power_polynomial(&mut substituted, &term, coefficient);
        }
        let Some(left) = multiply_power_polynomials(dependent_numerator, &weight_powers[degree])
        else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let Some(right) = multiply_power_polynomials(&basis.weight, &substituted) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let coefficient_count = left.len().max(right.len());
        for index in 0..coefficient_count {
            let difference = left.get(index).cloned().unwrap_or_else(Real::zero)
                - right.get(index).cloned().unwrap_or_else(Real::zero);
            match is_zero(&difference, policy) {
                Some(true) => {}
                Some(false) => return Ok(Classification::Decided(false)),
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
        }
        Ok(Classification::Decided(true))
    }
}

fn shared_conic_endpoint_parameters(
    source: &RationalBezier2,
    source_parameter: &Real,
    target: &RationalBezier2,
    policy: &CurvePolicy,
) -> Classification<Option<Vec<BezierParameter2>>> {
    if source.degree() != 2 || target.degree() != 2 {
        return Classification::Decided(None);
    }

    let source_controls = quadratic_conic_parameter_frame(source);
    let source_root_parameter = source.data.lineage.parameter_at(source_parameter);
    let one_minus = Real::one() - &source_root_parameter;
    let source_coefficients = [
        &one_minus * &one_minus,
        Real::from(2_i8) * &source_root_parameter * &one_minus,
        &source_root_parameter * &source_root_parameter,
    ];
    let homogeneous_point = [
        &source_controls[0].x * &source_coefficients[0]
            + &source_controls[1].x * &source_coefficients[1]
            + &source_controls[2].x * &source_coefficients[2],
        &source_controls[0].y * &source_coefficients[0]
            + &source_controls[1].y * &source_coefficients[1]
            + &source_controls[2].y * &source_coefficients[2],
        &source_controls[0].weight * &source_coefficients[0]
            + &source_controls[1].weight * &source_coefficients[1]
            + &source_controls[2].weight * &source_coefficients[2],
    ];

    let controls = quadratic_conic_parameter_frame(target);
    let first = homogeneous_control_vector(&controls[0]);
    let middle = homogeneous_control_vector(&controls[1]);
    let last = homogeneous_control_vector(&controls[2]);
    let coordinates = [
        dot3(&homogeneous_point, &cross3(&middle, &last)),
        dot3(&homogeneous_point, &cross3(&last, &first)),
        dot3(&homogeneous_point, &cross3(&first, &middle)),
    ];

    // The caller has already certified the shared implicit conic. In the
    // target's retained root frame these coordinates are proportional to
    // ((1-t)^2, 2t(1-t), t^2), which recovers its root parameter directly.
    let two = Real::from(2_i8);
    let first_denominator = &two * &coordinates[0] + &coordinates[1];
    let root_parameter = match is_zero(&first_denominator, policy) {
        Some(false) => match &coordinates[1] / &first_denominator {
            Ok(parameter) => parameter,
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        },
        Some(true) => {
            let second_denominator = &coordinates[1] + &two * &coordinates[2];
            match is_zero(&second_denominator, policy) {
                Some(false) => match (&two * &coordinates[2]) / second_denominator {
                    Ok(parameter) => parameter,
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                },
                // Both denominators vanish only at the omitted projective
                // parameter at infinity, not on this finite curve interval.
                Some(true) => return Classification::Decided(None),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    };
    let local_denominator =
        target.source_parameter_range().end() - target.source_parameter_range().start();
    let parameter =
        match (&root_parameter - target.source_parameter_range().start()) / local_denominator {
            Ok(parameter) => parameter,
            Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
        };
    match in_closed_unit_interval(&parameter, policy) {
        Some(true) => Classification::Decided(Some(vec![BezierParameter2::Exact(parameter)])),
        Some(false) => Classification::Decided(None),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
    }
}

fn quadratic_conic_parameter_frame(curve: &RationalBezier2) -> &[HomogeneousPoint2; 3] {
    curve
        .data
        .lineage
        .root
        .quadratic_conic_parameter_frame
        .get()
        .map(Rc::as_ref)
        .unwrap_or_else(|| {
            curve
                .homogeneous_controls()
                .try_into()
                .expect("quadratic curve has three homogeneous controls")
        })
}

fn push_unique_parameter_overlap_contact(
    contacts: &mut Vec<(BezierParameter2, BezierParameter2)>,
    first: BezierParameter2,
    second: BezierParameter2,
) {
    if contacts
        .iter()
        .any(|contact| contact.0 == first && contact.1 == second)
    {
        return;
    }
    contacts.push((first, second));
}

fn overlap_from_parameter_contacts(
    contacts: &[(BezierParameter2, BezierParameter2)],
    policy: &CurvePolicy,
) -> Classification<Option<RationalBezierIntersectionOverlap2>> {
    let [first, second] = contacts else {
        return Classification::Decided(None);
    };
    let first_order = match first.0.cmp_by_interval(&second.0, policy) {
        Ok(Classification::Decided(ordering)) => ordering,
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if first_order.is_eq() {
        return Classification::Decided(None);
    }
    let (first_start, first_end, second_start, second_end) = if first_order.is_lt() {
        (&first.0, &second.0, &first.1, &second.1)
    } else {
        (&second.0, &first.0, &second.1, &first.1)
    };
    let second_order = match second_start.cmp_by_interval(second_end, policy) {
        Ok(Classification::Decided(ordering)) => ordering,
        Ok(Classification::Uncertain(reason)) => return Classification::Uncertain(reason),
        Err(_) => return Classification::Uncertain(UncertaintyReason::Unsupported),
    };
    if second_order.is_eq() {
        return Classification::Decided(None);
    }
    Classification::Decided(Some(RationalBezierIntersectionOverlap2 {
        first_range: BezierParameterRange2::new_validated(first_start.clone(), first_end.clone()),
        second_range: BezierParameterRange2::new_validated(
            second_start.clone(),
            second_end.clone(),
        ),
        orientation: if second_order.is_lt() {
            RationalBezierOverlapOrientation2::Same
        } else {
            RationalBezierOverlapOrientation2::Reversed
        },
    }))
}

fn homogeneous_control_vector(control: &HomogeneousPoint2) -> [Real; 3] {
    [control.x.clone(), control.y.clone(), control.weight.clone()]
}

fn cross3(first: &[Real; 3], second: &[Real; 3]) -> [Real; 3] {
    [
        &first[1] * &second[2] - &first[2] * &second[1],
        &first[2] * &second[0] - &first[0] * &second[2],
        &first[0] * &second[1] - &first[1] * &second[0],
    ]
}

fn dot3(first: &[Real; 3], second: &[Real; 3]) -> Real {
    &first[0] * &second[0] + &first[1] * &second[1] + &first[2] * &second[2]
}

fn trim_power_polynomial(
    mut coefficients: Vec<Real>,
    policy: &CurvePolicy,
) -> Classification<Vec<Real>> {
    while coefficients.len() > 1 {
        match is_zero(coefficients.last().expect("nonempty polynomial"), policy) {
            Some(true) => {
                coefficients.pop();
            }
            Some(false) => break,
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
    }
    if coefficients.is_empty() {
        coefficients.push(Real::zero());
    }
    Classification::Decided(coefficients)
}

fn scale_power_polynomial(coefficients: &[Real], scale: &Real) -> Vec<Real> {
    coefficients
        .iter()
        .map(|coefficient| coefficient * scale)
        .collect()
}

fn subtract_power_polynomials(left: &[Real], right: &[Real]) -> Vec<Real> {
    let coefficient_count = left.len().max(right.len());
    (0..coefficient_count)
        .map(|index| {
            left.get(index).cloned().unwrap_or_else(Real::zero)
                - right.get(index).cloned().unwrap_or_else(Real::zero)
        })
        .collect()
}

fn add_scaled_power_polynomial(target: &mut Vec<Real>, source: &[Real], scale: &Real) {
    if target.len() < source.len() {
        target.resize_with(source.len(), Real::zero);
    }
    for (target, source) in target.iter_mut().zip(source) {
        *target = &*target + source * scale;
    }
}

fn multiply_power_polynomials(left: &[Real], right: &[Real]) -> Option<Vec<Real>> {
    let coefficient_count = left.len().checked_add(right.len())?.checked_sub(1)?;
    let mut product = vec![Real::zero(); coefficient_count];
    for (left_index, left) in left.iter().enumerate() {
        for (right_index, right) in right.iter().enumerate() {
            product[left_index + right_index] += left * right;
        }
    }
    Some(product)
}

fn power_polynomial_sequence(base: &[Real], max_power: usize) -> Option<Vec<Vec<Real>>> {
    let mut powers = Vec::new();
    powers.try_reserve_exact(max_power.checked_add(1)?).ok()?;
    powers.push(vec![Real::one()]);
    for power in 1..=max_power {
        powers.push(multiply_power_polynomials(&powers[power - 1], base)?);
    }
    Some(powers)
}

fn unique_point_incidence_parameter(
    curve: &RationalBezier2,
    point: &Point2,
    policy: &CurvePolicy,
) -> Classification<Option<BezierParameter2>> {
    match curve.point_incidence_classified(point, policy) {
        Err(CurveError::Real(_)) => Classification::Uncertain(UncertaintyReason::RealSign),
        Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
        Ok(Classification::Decided(RationalBezierPointIncidence2::Parameters(mut parameters))) => {
            if parameters.len() == 1 {
                let parameter = parameters.pop().expect("length checked above");
                match parameter.promote_represented_rational_root(policy) {
                    Ok(Classification::Decided(parameter)) => {
                        Classification::Decided(Some(parameter))
                    }
                    Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
                    Err(CurveError::Real(_)) => {
                        Classification::Uncertain(UncertaintyReason::RealSign)
                    }
                    Err(_) => Classification::Uncertain(UncertaintyReason::Unsupported),
                }
            } else if parameters.is_empty() {
                Classification::Decided(None)
            } else {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
        }
        Ok(Classification::Decided(RationalBezierPointIncidence2::EntireCurve)) => {
            Classification::Uncertain(UncertaintyReason::Unsupported)
        }
        Ok(Classification::Uncertain(reason)) => Classification::Uncertain(reason),
    }
}

fn candidate_points_equal(
    first: &CandidatePointReplay,
    second: &CandidatePointReplay,
    policy: &CurvePolicy,
) -> Option<bool> {
    match algebraic_coordinates_equal(&first.x, &second.x, policy) {
        Some(false) => return Some(false),
        Some(true) => {}
        None => return None,
    }
    algebraic_coordinates_equal(&first.y, &second.y, policy)
}

fn algebraic_coordinates_equal(
    first: &AlgebraicRootRepresentation,
    second: &AlgebraicRootRepresentation,
    policy: &CurvePolicy,
) -> Option<bool> {
    if let (Some(first), Some(second)) = (
        first.exact_rational_witness(),
        second.exact_rational_witness(),
    ) {
        return compare_reals(first, second, policy).map(|ordering| ordering.is_eq());
    }
    compare_algebraic_coordinates(first, second, policy)
}

#[cfg(feature = "predicates")]
fn compare_algebraic_coordinates(
    first: &AlgebraicRootRepresentation,
    second: &AlgebraicRootRepresentation,
    policy: &CurvePolicy,
) -> Option<bool> {
    let report = compare_algebraic_root_representations_by_difference(
        first,
        second,
        AlgebraicRootRefinementComparisonConfig {
            policy: policy.predicate_policy,
            ..AlgebraicRootRefinementComparisonConfig::default()
        },
    );
    report.comparison.ordering.map(|ordering| ordering.is_eq())
}

#[cfg(not(feature = "predicates"))]
fn compare_algebraic_coordinates(
    _first: &AlgebraicRootRepresentation,
    _second: &AlgebraicRootRepresentation,
    _policy: &CurvePolicy,
) -> Option<bool> {
    None
}

fn resultant_parameter_projection(
    report: CurveIntersectionResultantReport,
    policy: &CurvePolicy,
) -> CurveResult<Classification<ResultantParameterProjection>> {
    match report.status {
        CurveIntersectionResultantStatus::Constructed => {}
        CurveIntersectionResultantStatus::UndecidedCoefficient => {
            return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
        }
        CurveIntersectionResultantStatus::DegreeBoundExceeded
        | CurveIntersectionResultantStatus::EmptyCoordinatePolynomial
        | CurveIntersectionResultantStatus::ResultantError
        | CurveIntersectionResultantStatus::InterpolationDivisionFailed
        | CurveIntersectionResultantStatus::InvalidHomogeneousWeight => {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        }
    }
    if report
        .resultant_coefficients
        .iter()
        .all(|coefficient| is_zero(coefficient, policy) == Some(true))
    {
        return Ok(Classification::Decided(
            ResultantParameterProjection::Degenerate,
        ));
    }
    if report
        .resultant_coefficients
        .iter()
        .all(|coefficient| is_zero(coefficient, policy) != Some(false))
    {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    }
    let polynomial = match BezierParameterPolynomial::try_new_power_basis(
        report.resultant_coefficients,
        policy,
    )? {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    match polynomial.isolate_unit_interval_roots(policy)? {
        Classification::Decided(parameters) if parameters.is_empty() => {
            Ok(Classification::Decided(ResultantParameterProjection::Empty))
        }
        Classification::Decided(parameters) => Ok(Classification::Decided(
            ResultantParameterProjection::Parameters(parameters),
        )),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn evaluate_power_polynomial(coefficients: &[Real], parameter: &Real) -> Real {
    coefficients
        .iter()
        .rev()
        .fold(Real::zero(), |accumulator, coefficient| {
            (accumulator * parameter) + coefficient
        })
}

fn evaluate_power_polynomial_derivatives(
    coefficients: &[Real],
    parameter: &Real,
    max_order: usize,
) -> Option<Vec<Real>> {
    let value_count = max_order.checked_add(1)?;
    let mut derivatives = Vec::new();
    derivatives.try_reserve_exact(value_count).ok()?;
    derivatives.resize(value_count, Real::zero());
    for coefficient in coefficients.iter().rev() {
        for order in (1..=max_order).rev() {
            let scale = Real::from(u64::try_from(order).ok()?);
            derivatives[order] = &derivatives[order] * parameter + &scale * &derivatives[order - 1];
        }
        derivatives[0] = &derivatives[0] * parameter + coefficient;
    }
    Some(derivatives)
}

fn evaluate_power_polynomial_value_and_derivative(
    coefficients: &[Real],
    parameter: &Real,
) -> (Real, Real) {
    coefficients.iter().rev().fold(
        (Real::zero(), Real::zero()),
        |(value, derivative), coefficient| {
            (
                &value * parameter + coefficient,
                derivative * parameter + value,
            )
        },
    )
}

fn checked_binomial(n: usize, k: usize) -> Option<u64> {
    let k = k.min(n.checked_sub(k)?);
    (0..k).try_fold(1_u64, |result, index| {
        let numerator = u64::try_from(n.checked_sub(index)?).ok()?;
        let denominator = u64::try_from(index.checked_add(1)?).ok()?;
        result
            .checked_mul(numerator)
            .map(|value| value / denominator)
    })
}

impl HomogeneousPoint2 {
    fn lerp(&self, other: &Self, parameter: &Real) -> Self {
        let one_minus = Real::one() - parameter;
        Self {
            x: (&self.x * &one_minus) + (&other.x * parameter),
            y: (&self.y * &one_minus) + (&other.y * parameter),
            weight: (&self.weight * &one_minus) + (&other.weight * parameter),
        }
    }
}

fn project_homogeneous(point: &HomogeneousPoint2, policy: &CurvePolicy) -> Classification<Point2> {
    match is_zero(&point.weight, policy) {
        Some(true) => return Classification::Uncertain(UncertaintyReason::Boundary),
        Some(false) => {}
        None => return Classification::Uncertain(UncertaintyReason::RealSign),
    }
    let Ok(x) = &point.x / &point.weight else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    let Ok(y) = &point.y / &point.weight else {
        return Classification::Uncertain(UncertaintyReason::Boundary);
    };
    Classification::Decided(Point2::new(x, y))
}

fn from_homogeneous(
    controls: Vec<HomogeneousPoint2>,
    lineage: RationalBezierLineage,
    policy: &CurvePolicy,
) -> CurveResult<Classification<RationalBezier2>> {
    let mut points = Vec::with_capacity(controls.len());
    let mut weights = Vec::with_capacity(controls.len());
    for control in controls {
        let point = match project_homogeneous(&control, policy) {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        points.push(point);
        weights.push(control.weight);
    }
    RationalBezier2::try_new_with_lineage(points, weights, lineage).map(Classification::Decided)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clones_share_retained_axis_derivative_numerators() {
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(0.into(), 0.into()),
                Point2::new(1.into(), 2.into()),
                Point2::new(3.into(), 1.into()),
            ],
            vec![1.into(), 2.into(), 1.into()],
        )
        .unwrap();
        let clone = curve.clone();

        assert!(curve.data.x_derivative_numerator_bernstein.get().is_none());
        assert!(curve.data.x_axis_monotonicity.get().is_none());
        assert!(matches!(
            clone.axis_is_monotone(Axis2::X, &CurvePolicy::certified()),
            Ok(true)
        ));
        assert!(curve.data.x_derivative_numerator_bernstein.get().is_some());
        assert!(clone.data.x_derivative_numerator_bernstein.get().is_some());
        assert_eq!(curve.data.x_axis_monotonicity.get(), Some(&true));
        assert_eq!(clone.data.x_axis_monotonicity.get(), Some(&true));
        assert!(curve.data.y_derivative_numerator_bernstein.get().is_none());
        assert!(curve.data.y_axis_monotonicity.get().is_none());
    }

    #[test]
    fn implicit_conic_certificate_is_parameterization_independent_and_shared() {
        let weight = (Real::from(2_i8).sqrt().unwrap() / Real::from(2_i8)).unwrap();
        let controls = vec![
            Point2::new(1.into(), 0.into()),
            Point2::new(1.into(), 1.into()),
            Point2::new(0.into(), 1.into()),
        ];
        let first =
            RationalBezier2::try_new(controls.clone(), vec![1.into(), weight.clone(), 1.into()])
                .unwrap();
        let second = RationalBezier2::try_new(
            controls,
            vec![1.into(), Real::from(2_i8) * weight, 4.into()],
        )
        .unwrap();
        let policy = CurvePolicy::certified();

        let shared = first.shares_implicit_quadratic_conic(&second, &policy);
        assert!(
            matches!(shared, Classification::Decided(true)),
            "{shared:?}; first={:?}; second={:?}",
            first.data.lineage.root.implicit_quadratic_conic.get(),
            second.data.lineage.root.implicit_quadratic_conic.get()
        );
        assert!(
            first
                .data
                .lineage
                .root
                .implicit_quadratic_conic
                .get()
                .is_some()
        );
        assert!(
            second
                .data
                .lineage
                .root
                .implicit_quadratic_conic
                .get()
                .is_some()
        );
        assert!(matches!(
            first.point_at_classified(&Real::zero(), &policy),
            Classification::Decided(_)
        ));
        assert!(matches!(
            first.point_at_classified(&Real::one(), &policy),
            Classification::Decided(_)
        ));
        assert!(matches!(
            shared_conic_endpoint_parameters(&first, &Real::zero(), &second, &policy),
            Classification::Decided(Some(_))
        ));
        assert!(matches!(
            shared_conic_endpoint_parameters(&first, &Real::one(), &second, &policy),
            Classification::Decided(Some(_))
        ));
        assert!(matches!(
            overlap_from_parameter_contacts(
                &[
                    (
                        BezierParameter2::Exact(Real::zero()),
                        BezierParameter2::Exact(Real::zero())
                    ),
                    (
                        BezierParameter2::Exact(Real::one()),
                        BezierParameter2::Exact(Real::one())
                    )
                ],
                &policy
            ),
            Classification::Decided(Some(_))
        ));
        let replay = first.partial_image_overlap(&second, &policy);
        assert!(
            matches!(
                replay,
                Classification::Decided(RationalBezierSharedComponentReplay::Overlap(_))
            ),
            "{replay:?}"
        );

        let first_trimmed = match first
            .subcurve_between_exact(
                &Real::zero(),
                &(Real::from(3_i8) / Real::from(4_i8)).unwrap(),
                &policy,
            )
            .unwrap()
        {
            Classification::Decided(curve) => curve,
            Classification::Uncertain(reason) => panic!("first trim blocked: {reason:?}"),
        };
        let second_trimmed = match second
            .subcurve_between_exact(
                &(Real::one() / Real::from(4_i8)).unwrap(),
                &Real::one(),
                &policy,
            )
            .unwrap()
        {
            Classification::Decided(curve) => curve,
            Classification::Uncertain(reason) => panic!("second trim blocked: {reason:?}"),
        };
        let shared = first_trimmed.shares_implicit_quadratic_conic(&second_trimmed, &policy);
        assert!(
            matches!(shared, Classification::Decided(true)),
            "{shared:?}"
        );
        let first_mappings = [Real::zero(), Real::one()].map(|parameter| {
            let parameters = shared_conic_endpoint_parameters(
                &first_trimmed,
                &parameter,
                &second_trimmed,
                &policy,
            );
            assert!(
                matches!(parameters, Classification::Decided(_)),
                "first endpoint mapping: {parameters:?}"
            );
            parameters
        });
        let second_mappings = [Real::zero(), Real::one()].map(|parameter| {
            let parameters = shared_conic_endpoint_parameters(
                &second_trimmed,
                &parameter,
                &first_trimmed,
                &policy,
            );
            assert!(
                matches!(parameters, Classification::Decided(_)),
                "second endpoint mapping: {parameters:?}"
            );
            parameters
        });
        let replay = first_trimmed.partial_image_overlap(&second_trimmed, &policy);
        assert!(
            matches!(
                replay,
                Classification::Decided(RationalBezierSharedComponentReplay::Overlap(_))
            ),
            "trimmed replay: {replay:?}; first mappings: {first_mappings:?}; second mappings: {second_mappings:?}"
        );
    }
}
