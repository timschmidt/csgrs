//! Exact rational Bezier curves of arbitrary positive degree.

use std::cmp::Ordering;
use std::sync::Arc;
use std::sync::{Mutex, OnceLock};

use hyperreal::Rational as HyperRational;
use hyperreal::{Real, RealSign, ZeroKnowledge};
use hypersolve::{
    AlgebraicPolynomialValueInterval, AlgebraicRootRationalImageStatus, AlgebraicRootRationalMap,
    resultant_univariate_polynomials,
};
use hypersolve::{
    AlgebraicRootRepresentation, BivariatePolynomial, CurveIntersectionResultantConfig,
    CurveIntersectionResultantReport, CurveIntersectionResultantStatus, CurveResultantParameter,
    RationalParametricCurve2, compose_univariate_polynomial_linear_fractional,
    divide_bivariate_polynomial_exact, resultant_bivariate_polynomial_system,
    resultant_rational_parametric_curve_intersection,
};

use crate::bezier_algebraic_image::{
    compare_algebraic_representations_with_policy, exact_real_algebraic_representation,
    parameter_representation, rational_derivative_images_from_power_basis,
    rational_point_image_from_power_basis,
};
use crate::bezier_parameter::{BezierParameterRefinement2, bernstein_to_power_coefficients};
use crate::bezier_topology::{
    exact_line_contact_relation_from_bernstein_distances,
    exact_quadratic_line_contact_relation_with_certified_crossing,
    polynomial_roots_in_unit_interval_with_endpoints,
};
use crate::classify::{
    classify_oriented_line, compare_reals, in_closed_unit_interval, is_zero, orient2_real_expr,
    real_sign,
};
use crate::intersect::{circle_relation_from_supports, oriented_param_range_overlap};
use crate::{
    Aabb2, Axis2, BezierArrangementGraph2, BezierLineContactKind, BezierLineContactRelation,
    BezierLineCrossingDirection, BezierLineImageFitRelation, BezierParameter2,
    BezierParameterPolynomial, BezierParameterRange2, BezierParameterRayDirection2,
    BezierSplitMaterialization2, BezierSubcurve2, CircleCircleRelation, Classification,
    CurveContext, CurveDerivative2, CurveError, CurveFamily2, CurveOperation2, CurveResult,
    ExactCurveError, ExactCurveResult, LineSeg2, LineSide, ParamRange, Point2,
    RationalBezierAlgebraicPointImage2, RationalBezierAlgebraicTangentImage2,
    RationalQuadraticBezier2, UncertaintyReason,
};
use crate::{BezierAlgebraicParameter2, BezierParameterInterval};

/// Exact planar rational Bezier curve with an arbitrary positive degree.
///
/// Controls and weights are retained in affine form. Evaluation and splitting
/// operate in homogeneous coordinates, so unequal-weight cubic and
/// higher-degree NURBS spans do not need sampling or degree reduction.
#[derive(Clone, Debug)]
pub struct RationalBezier2 {
    data: Arc<RationalBezierData>,
}

#[derive(Debug)]
struct RationalBezierData {
    control_points: Vec<Point2>,
    weights: Vec<Real>,
    exact_line_image: Option<LineSeg2>,
    lineage: RationalBezierLineage,
    homogeneous_controls: OnceLock<Vec<HomogeneousPoint2>>,
    homogeneous_power_basis: OnceLock<RationalParametricCurve2>,
    x_derivative_numerator_bernstein: OnceLock<Option<Vec<Real>>>,
    y_derivative_numerator_bernstein: OnceLock<Option<Vec<Real>>>,
    x_axis_monotonicity: OnceLock<bool>,
    y_axis_monotonicity: OnceLock<bool>,
    degree_elevations: OnceLock<Mutex<Vec<ExactCurveResult<RationalBezier2>>>>,
}

#[derive(Clone, Debug)]
struct RationalBezierLineage {
    root: Arc<RationalBezierLineageRoot>,
    range: ParamRange,
}

#[derive(Debug, Default)]
struct RationalBezierLineageRoot {
    image_is_injective: OnceLock<bool>,
    implicit_quadratic_conic: OnceLock<Arc<[Real; 6]>>,
    circular_conic: OnceLock<Arc<crate::rational_bezier::RationalQuadraticCircle2>>,
    quadratic_conic_parameter_frame: OnceLock<Arc<[HomogeneousPoint2; 3]>>,
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
            root: Arc::clone(&self.root),
            range: ParamRange::new(self.parameter_at(start), self.parameter_at(end)),
        }
    }

    fn reversed(&self) -> Self {
        Self {
            root: Arc::clone(&self.root),
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

/// Exact affine point evidence retained for a curve contact.
#[derive(Clone, Debug, PartialEq)]
pub enum RationalBezierIntersectionPointEvidence2 {
    /// The contact point is represented directly by [`Real`] coordinates.
    Exact(Point2),
    /// The contact point is retained as exact algebraic point evidence.
    ///
    /// A retained rational-expression status may defer coordinate images
    /// while preserving the exact source curve and parameter.
    Algebraic(RationalBezierAlgebraicPointImage2),
    /// A unique nonparallel intersection of two retained algebraic chords.
    ///
    /// The four endpoint fields remain separate and are refined only when a
    /// coordinate comparison or enclosure is requested.
    AlgebraicChordPair(crate::BezierAlgebraicChordPairPoint2),
    /// A selected algebraic-circle contact with a certified axis-aligned
    /// retained chord.  Both selected fields and the square-root branch remain
    /// exact until a terminal predicate policy permits approximation.
    AlgebraicCuspChord(crate::BezierAlgebraicCuspChordPoint2),
    /// An exact affine derivative of a retained selected-circle/axis-chord
    /// contact, such as one endpoint of an axis-aligned parallel.
    AlgebraicCuspChordDerived(crate::BezierAlgebraicCuspChordDerivedPoint2),
    /// One endpoint displaced along an exact unit normal or tangent of a
    /// retained algebraic chord whose normalized direction spans selected
    /// endpoint fields.
    AlgebraicChordParallel(crate::BezierAlgebraicChordParallelPoint2),
    /// One exact point on an analytic Bezier parallel at a retained source
    /// parameter. The normalized direction is evaluated only by predicates.
    AnalyticParallel(crate::BezierAnalyticParallelPoint2),
    /// One exact retained point transported by a certified planar similarity.
    ///
    /// The source evidence remains correlated and is evaluated lazily rather
    /// than flattened into independently reconstructed coordinates.
    Similarity(crate::BezierSimilarityPoint2),
}

impl RationalBezierIntersectionPointEvidence2 {
    /// Returns the represented point when this evidence has native coordinates.
    pub const fn as_exact(&self) -> Option<&Point2> {
        match self {
            Self::Exact(point) => Some(point),
            Self::Algebraic(_) => None,
            Self::AlgebraicChordPair(_) => None,
            Self::AlgebraicCuspChord(_) => None,
            Self::AlgebraicCuspChordDerived(_) => None,
            Self::AlgebraicChordParallel(_) => None,
            Self::AnalyticParallel(_) => None,
            Self::Similarity(_) => None,
        }
    }

    /// Returns a constant-time positive identity certificate for retained
    /// point evidence. Distinct storage may still describe the same point and
    /// must continue through the exact geometric predicates.
    pub(crate) fn shares_storage(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Exact(first), Self::Exact(second)) => first.shares_storage(second),
            (Self::Algebraic(first), Self::Algebraic(second)) => first.shares_storage(second),
            (Self::AlgebraicChordPair(_), Self::AlgebraicChordPair(_)) => false,
            (Self::AlgebraicCuspChord(first), Self::AlgebraicCuspChord(second)) => {
                first.shares_storage(second)
            }
            (Self::AlgebraicCuspChordDerived(first), Self::AlgebraicCuspChordDerived(second)) => {
                first.shares_storage(second)
            }
            (Self::AlgebraicChordParallel(first), Self::AlgebraicChordParallel(second)) => {
                first.shares_storage(second)
            }
            (Self::AnalyticParallel(first), Self::AnalyticParallel(second)) => {
                first.shares_storage(second)
            }
            (Self::Similarity(first), Self::Similarity(second)) => first.shares_storage(second),
            _ => false,
        }
    }

    /// Returns the retained algebraic image, when present.
    pub const fn as_algebraic(&self) -> Option<&RationalBezierAlgebraicPointImage2> {
        match self {
            Self::Exact(_) => None,
            Self::Algebraic(point) => Some(point),
            Self::AlgebraicChordPair(_) => None,
            Self::AlgebraicCuspChord(_) => None,
            Self::AlgebraicCuspChordDerived(_) => None,
            Self::AlgebraicChordParallel(_) => None,
            Self::AnalyticParallel(_) => None,
            Self::Similarity(_) => None,
        }
    }

    /// Returns retained correlated chord-pair point evidence, when present.
    pub const fn as_algebraic_chord_pair(&self) -> Option<&crate::BezierAlgebraicChordPairPoint2> {
        match self {
            Self::AlgebraicChordPair(point) => Some(point),
            Self::Exact(_)
            | Self::Algebraic(_)
            | Self::AlgebraicCuspChord(_)
            | Self::AlgebraicCuspChordDerived(_)
            | Self::AlgebraicChordParallel(_)
            | Self::AnalyticParallel(_)
            | Self::Similarity(_) => None,
        }
    }

    /// Returns retained correlated cusp/chord point evidence, when present.
    pub const fn as_algebraic_cusp_chord(&self) -> Option<&crate::BezierAlgebraicCuspChordPoint2> {
        match self {
            Self::AlgebraicCuspChord(point) => Some(point),
            Self::Exact(_)
            | Self::Algebraic(_)
            | Self::AlgebraicChordPair(_)
            | Self::AlgebraicCuspChordDerived(_)
            | Self::AlgebraicChordParallel(_)
            | Self::AnalyticParallel(_)
            | Self::Similarity(_) => None,
        }
    }

    /// Returns a derived correlated cusp/chord point, when present.
    pub const fn as_algebraic_cusp_chord_derived(
        &self,
    ) -> Option<&crate::BezierAlgebraicCuspChordDerivedPoint2> {
        match self {
            Self::AlgebraicCuspChordDerived(point) => Some(point),
            Self::Exact(_)
            | Self::Algebraic(_)
            | Self::AlgebraicChordPair(_)
            | Self::AlgebraicCuspChord(_)
            | Self::AlgebraicChordParallel(_)
            | Self::AnalyticParallel(_)
            | Self::Similarity(_) => None,
        }
    }

    /// Returns a retained algebraic-chord parallel endpoint, when present.
    pub const fn as_algebraic_chord_parallel(
        &self,
    ) -> Option<&crate::BezierAlgebraicChordParallelPoint2> {
        match self {
            Self::AlgebraicChordParallel(point) => Some(point),
            Self::Exact(_)
            | Self::Algebraic(_)
            | Self::AlgebraicChordPair(_)
            | Self::AlgebraicCuspChord(_)
            | Self::AlgebraicCuspChordDerived(_)
            | Self::AnalyticParallel(_)
            | Self::Similarity(_) => None,
        }
    }

    /// Returns a retained analytic-parallel point, when present.
    pub const fn as_analytic_parallel(&self) -> Option<&crate::BezierAnalyticParallelPoint2> {
        match self {
            Self::AnalyticParallel(point) => Some(point),
            Self::Exact(_)
            | Self::Algebraic(_)
            | Self::AlgebraicChordPair(_)
            | Self::AlgebraicCuspChord(_)
            | Self::AlgebraicCuspChordDerived(_)
            | Self::AlgebraicChordParallel(_)
            | Self::Similarity(_) => None,
        }
    }

    /// Compares two retained affine points without materializing an algebraic
    /// coordinate or sampling either isolating interval.
    ///
    /// Exact points use the canonical [`Point2`] predicate. Algebraic images
    /// first reuse shared parametric provenance and disjoint source bounds,
    /// then compare represented coordinate roots. Correlated chord contacts
    /// retain their defining supports and refine enclosures without composing
    /// endpoint fields. Any predicate that remains unproved stays explicit
    /// under `policy`.
    pub(crate) fn same_point(&self, other: &Self, policy: &CurveContext) -> Classification<bool> {
        if self.shares_storage(other) {
            return Classification::Decided(true);
        }
        match (self, other) {
            (Self::Exact(first), Self::Exact(second)) => {
                match is_zero(&first.distance_squared(second), policy) {
                    Some(equal) => Classification::Decided(equal),
                    None => Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            (Self::Algebraic(first), Self::Algebraic(second)) => {
                if let Some(classification) =
                    first.same_injective_parametric_source_point(second, policy)
                {
                    return classification;
                }
                if let (
                    Some(Classification::Decided(first_bounds)),
                    Some(Classification::Decided(second_bounds)),
                ) = (
                    first.parametric_source_bounds(policy),
                    second.parametric_source_bounds(policy),
                ) && first_bounds.overlaps(&second_bounds, policy)
                    == Classification::Decided(false)
                {
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "contact-point-equality",
                        "source-bounds-disjoint",
                    );
                    return Classification::Decided(false);
                }
                if let Ok(Some(classification)) = first.same_retained_rational_point(second, policy)
                {
                    return classification;
                }
                let (Some(first), Some(second)) = (first.resolved(policy), second.resolved(policy))
                else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let (Some(first_x), Some(first_y), Some(second_x), Some(second_y)) = (
                    first.x().and_then(|image| image.representation()),
                    first.y().and_then(|image| image.representation()),
                    second.x().and_then(|image| image.representation()),
                    second.y().and_then(|image| image.representation()),
                ) else {
                    return if first == second {
                        Classification::Decided(true)
                    } else {
                        Classification::Uncertain(UncertaintyReason::Unsupported)
                    };
                };
                match (
                    crate::bezier_arrangement::represented_roots_equal(first_x, second_x, policy),
                    crate::bezier_arrangement::represented_roots_equal(first_y, second_y, policy),
                ) {
                    (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                    _ => Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            (Self::Exact(exact), Self::Algebraic(algebraic))
            | (Self::Algebraic(algebraic), Self::Exact(exact)) => {
                if let (Ok(x), Ok(y)) = (
                    algebraic.coordinate_order_to_real(true, exact.x(), policy),
                    algebraic.coordinate_order_to_real(false, exact.y(), policy),
                ) {
                    match (x, y) {
                        (
                            Classification::Decided(std::cmp::Ordering::Equal),
                            Classification::Decided(std::cmp::Ordering::Equal),
                        ) => return Classification::Decided(true),
                        (
                            Classification::Decided(
                                std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                            ),
                            _,
                        )
                        | (
                            _,
                            Classification::Decided(
                                std::cmp::Ordering::Less | std::cmp::Ordering::Greater,
                            ),
                        ) => return Classification::Decided(false),
                        _ => {}
                    }
                }
                let Some(algebraic) = algebraic.resolved(policy) else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let (Some(x), Some(y)) = (
                    algebraic.x().and_then(|image| image.representation()),
                    algebraic.y().and_then(|image| image.representation()),
                ) else {
                    return Classification::Uncertain(UncertaintyReason::Unsupported);
                };
                let exact_x = exact_real_algebraic_representation(exact.x());
                let exact_y = exact_real_algebraic_representation(exact.y());
                match (
                    crate::bezier_arrangement::represented_roots_equal(x, &exact_x, policy),
                    crate::bezier_arrangement::represented_roots_equal(y, &exact_y, policy),
                ) {
                    (Some(x_equal), Some(y_equal)) => Classification::Decided(x_equal && y_equal),
                    _ => Classification::Uncertain(UncertaintyReason::RealSign),
                }
            }
            (Self::AlgebraicChordPair(first), Self::AlgebraicChordPair(second)) => {
                first.same_point(second, policy)
            }
            (Self::AlgebraicChordPair(point), other) | (other, Self::AlgebraicChordPair(point)) => {
                point.same_point_evidence(other, policy)
            }
            (Self::AlgebraicCuspChord(first), Self::AlgebraicCuspChord(second)) => {
                first.same_point_evidence(&Self::AlgebraicCuspChord(second.clone()), policy)
            }
            (Self::AlgebraicCuspChord(point), other) | (other, Self::AlgebraicCuspChord(point)) => {
                point.same_point_evidence(other, policy)
            }
            (Self::AlgebraicCuspChordDerived(first), Self::AlgebraicCuspChordDerived(second)) => {
                first.same_point(second, policy)
            }
            (Self::AlgebraicCuspChordDerived(point), other)
            | (other, Self::AlgebraicCuspChordDerived(point)) => {
                point.same_point_evidence(other, policy)
            }
            (Self::AlgebraicChordParallel(point), other)
            | (other, Self::AlgebraicChordParallel(point)) => {
                point.same_point_evidence(other, policy)
            }
            (Self::AnalyticParallel(point), other) | (other, Self::AnalyticParallel(point)) => {
                point.same_point_evidence(other, policy)
            }
            (Self::Similarity(point), other) | (other, Self::Similarity(point)) => {
                point.same_point_evidence(other, policy)
            }
        }
    }
}

impl From<Point2> for RationalBezierIntersectionPointEvidence2 {
    fn from(point: Point2) -> Self {
        Self::Exact(point)
    }
}

/// One exactly replayed parameter pair shared by two rational Bezier images.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierIntersectionContact2 {
    first_parameter: BezierParameter2,
    second_parameter: BezierParameter2,
    point: RationalBezierIntersectionPointEvidence2,
    certified_transverse: bool,
    tangent_cross_sign: Option<RealSign>,
}

/// Relative parameter orientation of a certified shared rational-Bezier image.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RationalBezierOverlapOrientation2 {
    /// Both parameter domains traverse the shared image in the same direction.
    Same,
    /// The second parameter domain traverses the shared image in reverse.
    Reversed,
}

/// Certified positive-length image overlap between two rational Bezier curves.
///
/// The oriented parameter ranges bound the overlap closure. The endpoint
/// inclusion flags distinguish ordinary closed shared images from strict
/// branch selections that exclude one or both paired boundary points.
#[derive(Clone, Debug, PartialEq)]
pub struct RationalBezierIntersectionOverlap2 {
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
    endpoint_inclusion: [bool; 2],
}

#[derive(Clone, Debug)]
pub(crate) enum RationalBezierOverlapParameterCorrespondence2 {
    Identity,
    UnitComplement,
    EndpointProjective {
        second_to_first_scale: Real,
        reversed: bool,
    },
    RangeProjective {
        second_to_first_scale: Real,
        reversed: bool,
    },
    General {
        first: RationalBezier2,
        second: RationalBezier2,
        unresolved: Option<UncertaintyReason>,
    },
}

enum RationalBezierEndpointParameterRelation2 {
    Affine,
    Projective(Real),
}

impl RationalBezierIntersectionOverlap2 {
    pub(crate) fn from_certified_parameters(
        first_start: BezierParameter2,
        first_end: BezierParameter2,
        second_start: BezierParameter2,
        second_end: BezierParameter2,
        orientation: RationalBezierOverlapOrientation2,
        endpoint_inclusion: [bool; 2],
    ) -> Self {
        Self {
            first_range: BezierParameterRange2::new_validated(first_start, first_end),
            second_range: BezierParameterRange2::new_validated(second_start, second_end),
            orientation,
            endpoint_inclusion,
        }
    }

    /// Returns the exact oriented closure bounds on the first curve.
    pub const fn first_range(&self) -> &BezierParameterRange2 {
        &self.first_range
    }

    /// Returns the exact oriented closure bounds on the second curve, arranged
    /// to match traversal of [`Self::first_range`].
    pub const fn second_range(&self) -> &BezierParameterRange2 {
        &self.second_range
    }

    /// Returns relative parameter orientation on the shared image.
    pub const fn orientation(&self) -> RationalBezierOverlapOrientation2 {
        self.orientation
    }

    /// Returns whether the paired starts of both oriented ranges belong to the overlap.
    pub const fn includes_start(&self) -> bool {
        self.endpoint_inclusion[0]
    }

    /// Returns whether the paired ends of both oriented ranges belong to the overlap.
    pub const fn includes_end(&self) -> bool {
        self.endpoint_inclusion[1]
    }
}

impl RationalBezierOverlapParameterCorrespondence2 {
    fn new(first: &RationalBezier2, second: &RationalBezier2, policy: &CurveContext) -> Self {
        let mut unresolved = None;
        if first.degree() == second.degree() {
            for reversed in [false, true] {
                match first.endpoint_parameter_relation(second, reversed, policy) {
                    Classification::Decided(Some(
                        RationalBezierEndpointParameterRelation2::Affine,
                    )) => {
                        return if reversed {
                            Self::UnitComplement
                        } else {
                            Self::Identity
                        };
                    }
                    Classification::Decided(Some(
                        RationalBezierEndpointParameterRelation2::Projective(second_to_first_scale),
                    )) => {
                        return Self::EndpointProjective {
                            second_to_first_scale,
                            reversed,
                        };
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => unresolved = Some(reason),
                }
            }
        } else {
            for reversed in [false, true] {
                match first.same_projective_control_net_degree_aligned(second, reversed, policy) {
                    Classification::Decided(true) => {
                        return if reversed {
                            Self::UnitComplement
                        } else {
                            Self::Identity
                        };
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => unresolved = Some(reason),
                }
            }
        }
        Self::General {
            first: first.clone(),
            second: second.clone(),
            unresolved,
        }
    }

    /// Maps one parameter between two rational carriers that are known by the
    /// caller to represent the same local geometric point.
    ///
    /// This uses the global endpoint-projective fast paths when available and
    /// otherwise falls through to the exact conic/graph/injective-coordinate
    /// point-incidence authority. Unlike [`Self::for_overlap`], it does not
    /// require an already materialized overlap range and therefore also serves
    /// compact mapped cuts retained by another exact carrier.
    pub(crate) fn map_parameter_between_curves(
        source: &RationalBezier2,
        target: &RationalBezier2,
        parameter: &BezierParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        match Self::new(source, target, policy) {
            Self::Identity => Ok(Classification::Decided(Some(parameter.clone()))),
            Self::UnitComplement => Ok(Classification::Decided(Some(parameter.unit_complement()))),
            Self::EndpointProjective {
                second_to_first_scale,
                reversed,
            } => endpoint_projective_parameter_image(
                parameter,
                &second_to_first_scale,
                reversed,
                true,
                policy,
            ),
            Self::General {
                first,
                second,
                unresolved,
            } => match first.image_overlap(&second, policy) {
                Classification::Decided(RationalBezierSharedComponentReplay::Overlap(overlap)) => {
                    let correspondence = Self::for_overlap(&first, &second, &overlap, policy);
                    correspondence.map_first_to_second(
                        parameter,
                        overlap.first_range(),
                        overlap.second_range(),
                        policy,
                    )
                }
                Classification::Decided(RationalBezierSharedComponentReplay::Contacts(_)) => {
                    Ok(Classification::Decided(None))
                }
                Classification::Decided(RationalBezierSharedComponentReplay::Unresolved) => Ok(
                    Classification::Uncertain(unresolved.unwrap_or(UncertaintyReason::Unsupported)),
                ),
                Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
            },
            Self::RangeProjective { .. } => {
                unreachable!("a range-projective correspondence requires an authored overlap range")
            }
        }
    }

    pub(crate) fn for_overlap(
        first: &RationalBezier2,
        second: &RationalBezier2,
        overlap: &RationalBezierIntersectionOverlap2,
        policy: &CurveContext,
    ) -> Self {
        let fallback = Self::new(first, second, policy);
        if !matches!(fallback, Self::General { .. }) {
            return fallback;
        }
        let (Some(first_start), Some(first_end)) = (
            overlap.first_range().start().as_exact(),
            overlap.first_range().end().as_exact(),
        ) else {
            return fallback;
        };
        let reversed = overlap.orientation() == RationalBezierOverlapOrientation2::Reversed;
        let (second_start, second_end) = if reversed {
            (
                overlap.second_range().end().as_exact(),
                overlap.second_range().start().as_exact(),
            )
        } else {
            (
                overlap.second_range().start().as_exact(),
                overlap.second_range().end().as_exact(),
            )
        };
        let (Some(second_start), Some(second_end)) = (second_start, second_end) else {
            return fallback;
        };
        let first_subcurve = match first.subcurve_between_exact(first_start, first_end, policy) {
            Ok(Classification::Decided(curve)) => curve,
            Ok(Classification::Uncertain(_)) | Err(_) => return fallback,
        };
        let second_subcurve = match second.subcurve_between_exact(second_start, second_end, policy)
        {
            Ok(Classification::Decided(curve)) => curve,
            Ok(Classification::Uncertain(_)) | Err(_) => return fallback,
        };
        let second_to_first_scale =
            match first_subcurve.endpoint_parameter_relation(&second_subcurve, reversed, policy) {
                Classification::Decided(Some(RationalBezierEndpointParameterRelation2::Affine)) => {
                    Real::one()
                }
                Classification::Decided(Some(
                    RationalBezierEndpointParameterRelation2::Projective(scale),
                )) => scale,
                Classification::Decided(None) | Classification::Uncertain(_) => return fallback,
            };
        Self::RangeProjective {
            second_to_first_scale,
            reversed,
        }
    }

    pub(crate) const fn projective_reversal(&self) -> Option<bool> {
        match self {
            Self::Identity => Some(false),
            Self::UnitComplement => Some(true),
            Self::EndpointProjective { .. }
            | Self::RangeProjective { .. }
            | Self::General { .. } => None,
        }
    }

    pub(crate) fn map_first_to_second(
        &self,
        parameter: &BezierParameter2,
        first_range: &BezierParameterRange2,
        second_range: &BezierParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        match self {
            Self::Identity => Ok(Classification::Decided(Some(parameter.clone()))),
            Self::UnitComplement => Ok(Classification::Decided(Some(parameter.unit_complement()))),
            Self::EndpointProjective {
                second_to_first_scale,
                reversed,
            } => endpoint_projective_parameter_image(
                parameter,
                second_to_first_scale,
                *reversed,
                true,
                policy,
            ),
            Self::RangeProjective {
                second_to_first_scale,
                reversed,
            } => range_projective_parameter_image(
                parameter,
                first_range,
                second_range,
                second_to_first_scale,
                *reversed,
                true,
                policy,
            ),
            Self::General {
                first,
                second,
                unresolved,
            } => overlap_parameter_on_curve(first, second, parameter, *unresolved, policy),
        }
    }

    pub(crate) fn map_second_to_first(
        &self,
        parameter: &BezierParameter2,
        first_range: &BezierParameterRange2,
        second_range: &BezierParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        match self {
            Self::Identity => Ok(Classification::Decided(Some(parameter.clone()))),
            Self::UnitComplement => Ok(Classification::Decided(Some(parameter.unit_complement()))),
            Self::EndpointProjective {
                second_to_first_scale,
                reversed,
            } => endpoint_projective_parameter_image(
                parameter,
                second_to_first_scale,
                *reversed,
                false,
                policy,
            ),
            Self::RangeProjective {
                second_to_first_scale,
                reversed,
            } => range_projective_parameter_image(
                parameter,
                first_range,
                second_range,
                second_to_first_scale,
                *reversed,
                false,
                policy,
            ),
            Self::General {
                first,
                second,
                unresolved,
            } => overlap_parameter_on_curve(second, first, parameter, *unresolved, policy),
        }
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

    /// Returns whether retained simple-root evidence certifies a transverse contact.
    pub const fn is_certified_transverse(&self) -> bool {
        self.certified_transverse
    }

    /// Returns the certified sign of the first tangent crossed with the second.
    pub const fn tangent_cross_sign(&self) -> Option<RealSign> {
        self.tangent_cross_sign
    }
}

/// Exact replay status for rational Bezier resultant candidates.
#[derive(Clone, Debug, PartialEq)]
pub enum RationalBezierIntersectionContacts2 {
    /// Replay certified that the finite curve images do not meet.
    NoIntersection,
    /// Every resultant candidate pair was decided and these contacts remain.
    Contacts(Arc<[RationalBezierIntersectionContact2]>),
    /// Exact shared-component replay certified a positive-length full or
    /// partial shared image and retained both oriented parameter ranges.
    Overlap(RationalBezierIntersectionOverlap2),
    /// The complete set contains both isolated contacts and a positive-length
    /// shared image. This occurs, for example, when overlapping retained
    /// subranges of one non-injective carrier also meet across distinct
    /// branches of that carrier.
    ContactsAndOverlap {
        /// Isolated contacts outside the same-source overlap correspondence.
        contacts: Arc<[RationalBezierIntersectionContact2]>,
        /// Certified positive-length shared image.
        overlap: RationalBezierIntersectionOverlap2,
    },
    /// Some contacts were certified, but at least one candidate comparison
    /// remained unresolved under the exact algebraic comparison budget.
    Incomplete {
        /// Contacts already certified by exact replay.
        contacts: Arc<[RationalBezierIntersectionContact2]>,
        /// Complete unpaired resultant projections retained for later replay.
        candidates: RationalBezierIntersectionCandidates2,
    },
    /// A resultant vanished identically and overlap replay is required.
    DegenerateResultant,
}

impl RationalBezierIntersectionContacts2 {
    /// Returns the completely replayed isolated contacts retained by this result.
    pub fn isolated_contacts(&self) -> &[RationalBezierIntersectionContact2] {
        match self {
            Self::Contacts(contacts)
            | Self::Incomplete { contacts, .. }
            | Self::ContactsAndOverlap { contacts, .. } => contacts,
            Self::NoIntersection | Self::Overlap(_) | Self::DegenerateResultant => &[],
        }
    }

    /// Returns the certified positive-length overlap, when present.
    pub const fn overlap(&self) -> Option<&RationalBezierIntersectionOverlap2> {
        match self {
            Self::Overlap(overlap) | Self::ContactsAndOverlap { overlap, .. } => Some(overlap),
            Self::NoIntersection
            | Self::Contacts(_)
            | Self::Incomplete { .. }
            | Self::DegenerateResultant => None,
        }
    }
}

#[derive(Debug)]
enum RationalBezierSharedComponentReplay {
    Overlap(RationalBezierIntersectionOverlap2),
    Contacts(Vec<(Real, Real)>),
    Unresolved,
}

/// Retained split topology derived from one completely replayed curve pair.
///
/// The contact collection is shared with the retained pair. The two split
/// materializations preserve each contact parameter and its exact endpoint
/// images, so an arrangement can consume the result without rerunning
/// resultants or algebraic point comparison.
#[derive(Clone, Debug)]
pub struct RationalBezierIntersectionTopology2 {
    data: Arc<RationalBezierIntersectionTopologyData>,
}

#[derive(Debug)]
struct RationalBezierIntersectionTopologyData {
    contacts: Arc<[RationalBezierIntersectionContact2]>,
    first: BezierSplitMaterialization2,
    second: BezierSplitMaterialization2,
    arrangement: OnceLock<CurveResult<BezierArrangementGraph2>>,
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

#[derive(Debug)]
pub(crate) struct RationalBezierIntersectionContext {
    data: RationalBezierIntersectionContextData,
}

#[derive(Debug)]
struct RationalBezierIntersectionContextData {
    first: RationalBezier2,
    second: RationalBezier2,
    policy: CurveContext,
    candidates: RationalBezierIntersectionCandidates2,
    contacts: OnceLock<CurveResult<Classification<RationalBezierIntersectionContacts2>>>,
}

impl RationalBezierIntersectionContext {
    pub(crate) fn try_new(
        first: &RationalBezier2,
        second: &RationalBezier2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_circle_relation(first, second, policy, None, None, None)
    }

    pub(crate) fn try_new_with_circle_relation(
        first: &RationalBezier2,
        second: &RationalBezier2,
        policy: &CurveContext,
        circle_relation: Option<&CircleCircleRelation>,
        first_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
        second_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
    ) -> ExactCurveResult<Self> {
        match first.intersection_context_classified(
            second,
            policy,
            circle_relation,
            first_circle_parameters,
            second_circle_parameters,
        ) {
            Ok(Classification::Decided(context)) => Ok(context),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    fn try_contact_view(&self) -> ExactCurveResult<&RationalBezierIntersectionContacts2> {
        match self.contacts_ref() {
            Ok(Classification::Decided(contacts)) => Ok(contacts),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                *reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause.clone(),
            )),
        }
    }

    pub(crate) fn try_contacts(&self) -> ExactCurveResult<RationalBezierIntersectionContacts2> {
        self.try_contact_view().cloned()
    }

    pub(crate) fn overlap_parameter_correspondence(
        &self,
        overlap: &RationalBezierIntersectionOverlap2,
    ) -> RationalBezierOverlapParameterCorrespondence2 {
        RationalBezierOverlapParameterCorrespondence2::for_overlap(
            &self.data.first,
            &self.data.second,
            overlap,
            &self.data.policy,
        )
    }

    fn try_topology(&self) -> ExactCurveResult<RationalBezierIntersectionTopology2> {
        match self.build_topology() {
            Ok(Classification::Decided(topology)) => Ok(topology),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Arrangement,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Arrangement,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    fn contacts_ref(&self) -> &CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        self.data.contacts.get_or_init(|| {
            self.data.first.replay_intersection_candidate_set(
                &self.data.second,
                &self.data.candidates,
                &self.data.policy,
            )
        })
    }

    fn build_topology(&self) -> CurveResult<Classification<RationalBezierIntersectionTopology2>> {
        let contacts = match self.contacts_ref() {
            Ok(Classification::Decided(RationalBezierIntersectionContacts2::NoIntersection)) => {
                Arc::from([])
            }
            Ok(Classification::Decided(RationalBezierIntersectionContacts2::Contacts(
                contacts,
            ))) => Arc::clone(contacts),
            Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::Overlap(_)
                | RationalBezierIntersectionContacts2::ContactsAndOverlap { .. },
            )) => {
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
                data: Arc::new(RationalBezierIntersectionTopologyData {
                    contacts,
                    first,
                    second,
                    arrangement: OnceLock::new(),
                }),
            },
        ))
    }
}

#[derive(Clone, Debug)]
struct CandidatePointReplay {
    evidence: RationalBezierIntersectionPointEvidence2,
    x: AlgebraicRootRepresentation,
    y: AlgebraicRootRepresentation,
}

#[derive(Debug)]
pub(crate) enum ResultantParameterProjection {
    Empty,
    Parameters(Vec<BezierParameter2>),
    /// Parameters isolated directly in the caller's selected algebraic fiber.
    /// Unlike an ordinary quotient norm, these need no conjugate-root replay.
    SelectedParameters(Vec<BezierParameter2>),
    Degenerate,
}

const MAX_RATIONAL_INTERSECTION_RESULTANT_DEGREE: usize = 128;
const RATIONAL_INTERSECTION_RESULTANT_PRECISION: i32 = -128;
const MAX_QUOTIENT_RING_RATIONAL_IMAGE_DEGREE: usize = 12;
const MAX_RETAINED_EVALUATION_POWER_DEGREE: usize = 256;

fn rational_self_intersection_residual_system(
    basis: &RationalParametricCurve2,
) -> Option<[BivariatePolynomial; 2]> {
    let diagonal = BivariatePolynomial::new(vec![
        vec![Real::zero(), Real::one()],
        vec![Real::from(-1_i8)],
    ]);
    let x = rational_coordinate_parameter_difference(
        &basis.x_numerator,
        &basis.weight,
        &basis.x_numerator,
        &basis.weight,
    );
    let y = rational_coordinate_parameter_difference(
        &basis.y_numerator,
        &basis.weight,
        &basis.y_numerator,
        &basis.weight,
    );
    Some([
        divide_bivariate_polynomial_exact(&x, &diagonal)?,
        divide_bivariate_polynomial_exact(&y, &diagonal)?,
    ])
}

fn rational_coordinate_parameter_difference(
    first_numerator: &[Real],
    first_weight: &[Real],
    second_numerator: &[Real],
    second_weight: &[Real],
) -> BivariatePolynomial {
    let first_coefficient_count = first_numerator.len().max(first_weight.len());
    let second_coefficient_count = second_numerator.len().max(second_weight.len());
    let mut coefficients =
        vec![vec![Real::zero(); second_coefficient_count]; first_coefficient_count];
    for (first_power, row) in coefficients.iter_mut().enumerate() {
        let first_numerator = first_numerator
            .get(first_power)
            .cloned()
            .unwrap_or_else(Real::zero);
        let first_weight = first_weight
            .get(first_power)
            .cloned()
            .unwrap_or_else(Real::zero);
        for (second_power, coefficient) in row.iter_mut().enumerate() {
            let second_numerator = second_numerator
                .get(second_power)
                .cloned()
                .unwrap_or_else(Real::zero);
            let second_weight = second_weight
                .get(second_power)
                .cloned()
                .unwrap_or_else(Real::zero);
            *coefficient = &first_numerator * second_weight - &first_weight * second_numerator;
        }
    }
    BivariatePolynomial::new(coefficients)
}

fn rational_retained_lineage_residual_system(
    first: &RationalBezier2,
    second: &RationalBezier2,
) -> CurveResult<Option<[BivariatePolynomial; 2]>> {
    let first_basis = first.homogeneous_power_basis()?;
    let second_basis = second.homogeneous_power_basis()?;
    let first_range = first.source_parameter_range();
    let second_range = second.source_parameter_range();
    let first_delta = first_range.end() - first_range.start();
    let second_delta = second_range.end() - second_range.start();
    let same_source_parameter = BivariatePolynomial::new(vec![
        vec![second_range.start() - first_range.start(), second_delta],
        vec![-first_delta],
    ]);
    let x = rational_coordinate_parameter_difference(
        &first_basis.x_numerator,
        &first_basis.weight,
        &second_basis.x_numerator,
        &second_basis.weight,
    );
    let y = rational_coordinate_parameter_difference(
        &first_basis.y_numerator,
        &first_basis.weight,
        &second_basis.y_numerator,
        &second_basis.weight,
    );
    let Some(x) = divide_bivariate_polynomial_exact(&x, &same_source_parameter) else {
        return Ok(None);
    };
    let Some(y) = divide_bivariate_polynomial_exact(&y, &same_source_parameter) else {
        return Ok(None);
    };
    Ok(Some([x, y]))
}

fn project_retained_lineage_residual_system(
    equations: &[BivariatePolynomial; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierIntersectionCandidates2>> {
    let project = |parameter| {
        resultant_parameter_projection(
            resultant_bivariate_polynomial_system(
                &equations[0],
                &equations[1],
                parameter,
                CurveIntersectionResultantConfig {
                    min_precision: RATIONAL_INTERSECTION_RESULTANT_PRECISION,
                    max_resultant_degree: MAX_RATIONAL_INTERSECTION_RESULTANT_DEGREE,
                },
            ),
            policy,
        )
    };
    let first = match project(CurveResultantParameter::First)? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    let second = match project(CurveResultantParameter::Second)? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
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
            ResultantParameterProjection::Parameters(first_parameters)
            | ResultantParameterProjection::SelectedParameters(first_parameters),
            ResultantParameterProjection::Parameters(second_parameters)
            | ResultantParameterProjection::SelectedParameters(second_parameters),
        ) => RationalBezierIntersectionCandidates2::Candidates {
            first_parameters,
            second_parameters,
        },
    }))
}

fn project_symmetric_self_intersection_system(
    equations: &[BivariatePolynomial; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierIntersectionCandidates2>> {
    let report = resultant_bivariate_polynomial_system(
        &equations[0],
        &equations[1],
        CurveResultantParameter::First,
        CurveIntersectionResultantConfig {
            min_precision: RATIONAL_INTERSECTION_RESULTANT_PRECISION,
            max_resultant_degree: MAX_RATIONAL_INTERSECTION_RESULTANT_DEGREE,
        },
    );
    let projection = match resultant_parameter_projection(report, policy)? {
        Classification::Decided(projection) => projection,
        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
    };
    Ok(Classification::Decided(match projection {
        ResultantParameterProjection::Empty => {
            RationalBezierIntersectionCandidates2::NoIntersection
        }
        ResultantParameterProjection::Degenerate => {
            RationalBezierIntersectionCandidates2::DegenerateResultant
        }
        ResultantParameterProjection::Parameters(parameters)
        | ResultantParameterProjection::SelectedParameters(parameters) => {
            RationalBezierIntersectionCandidates2::Candidates {
                first_parameters: parameters.clone(),
                second_parameters: parameters,
            }
        }
    }))
}

fn rational_tangent_cross_polynomial(
    basis: &RationalParametricCurve2,
) -> Option<BivariatePolynomial> {
    rational_pair_tangent_cross_polynomial(basis, basis)
}

fn rational_pair_tangent_cross_polynomial(
    first: &RationalParametricCurve2,
    second: &RationalParametricCurve2,
) -> Option<BivariatePolynomial> {
    let first_tangent_x = rational_coordinate_tangent_numerator(&first.x_numerator, &first.weight)?;
    let first_tangent_y = rational_coordinate_tangent_numerator(&first.y_numerator, &first.weight)?;
    let second_tangent_x =
        rational_coordinate_tangent_numerator(&second.x_numerator, &second.weight)?;
    let second_tangent_y =
        rational_coordinate_tangent_numerator(&second.y_numerator, &second.weight)?;
    let first_coefficient_count = first_tangent_x.len().max(first_tangent_y.len());
    let second_coefficient_count = second_tangent_x.len().max(second_tangent_y.len());
    let mut coefficients =
        vec![vec![Real::zero(); second_coefficient_count]; first_coefficient_count];
    for (first_power, row) in coefficients.iter_mut().enumerate() {
        let first_x = first_tangent_x
            .get(first_power)
            .cloned()
            .unwrap_or_else(Real::zero);
        let first_y = first_tangent_y
            .get(first_power)
            .cloned()
            .unwrap_or_else(Real::zero);
        for (second_power, coefficient) in row.iter_mut().enumerate() {
            let second_x = second_tangent_x
                .get(second_power)
                .cloned()
                .unwrap_or_else(Real::zero);
            let second_y = second_tangent_y
                .get(second_power)
                .cloned()
                .unwrap_or_else(Real::zero);
            *coefficient = &first_x * second_y - &first_y * second_x;
        }
    }
    Some(BivariatePolynomial::new(coefficients))
}

fn rational_coordinate_tangent_numerator(numerator: &[Real], weight: &[Real]) -> Option<Vec<Real>> {
    let numerator_derivative = derivative_power_polynomial(numerator)?;
    let weight_derivative = derivative_power_polynomial(weight)?;
    let first = multiply_power_polynomials(&numerator_derivative, weight)?;
    let second = multiply_power_polynomials(numerator, &weight_derivative)?;
    Some(subtract_power_polynomials(&first, &second))
}

fn derivative_power_polynomial(coefficients: &[Real]) -> Option<Vec<Real>> {
    if coefficients.len() <= 1 {
        return Some(vec![Real::zero()]);
    }
    coefficients
        .iter()
        .enumerate()
        .skip(1)
        .map(|(power, coefficient)| Some(Real::from(u64::try_from(power).ok()?) * coefficient))
        .collect()
}

fn retain_unordered_rational_self_contacts(
    replayed: RationalBezierIntersectionContacts2,
    basis: &RationalParametricCurve2,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
    let tangent_cross = rational_tangent_cross_polynomial(basis);
    retain_rational_contact_tangent_cross_signs(replayed, tangent_cross.as_ref(), policy)
}

fn retain_rational_contact_tangent_cross_signs(
    replayed: RationalBezierIntersectionContacts2,
    tangent_cross: Option<&BivariatePolynomial>,
    policy: &CurveContext,
) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
    let retain = |contacts: &Arc<[RationalBezierIntersectionContact2]>|
     -> CurveResult<Classification<Arc<[RationalBezierIntersectionContact2]>>> {
        let mut retained = Vec::with_capacity(contacts.len());
        for contact in contacts.iter() {
            let tangent_cross_sign = match tangent_cross {
                Some(tangent_cross) => {
                    crate::bezier_offset::bivariate_parameter_pair_strict_sign_by_refinement(
                        tangent_cross,
                        &contact.first_parameter,
                        &contact.second_parameter,
                        policy,
                    )?
                }
                None => None,
            };
            let mut contact = contact.clone();
            contact.certified_transverse |= matches!(
                tangent_cross_sign,
                Some(RealSign::Positive | RealSign::Negative)
            );
            contact.tangent_cross_sign = tangent_cross_sign;
            retained.push(contact);
        }
        Ok(Classification::Decided(Arc::from(retained)))
    };
    let result = match replayed {
        RationalBezierIntersectionContacts2::Contacts(contacts) => match retain(&contacts)? {
            Classification::Decided(contacts) if contacts.is_empty() => {
                RationalBezierIntersectionContacts2::NoIntersection
            }
            Classification::Decided(contacts) => {
                RationalBezierIntersectionContacts2::Contacts(contacts)
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        },
        RationalBezierIntersectionContacts2::Incomplete {
            contacts,
            candidates,
        } => match retain(&contacts)? {
            Classification::Decided(contacts) => RationalBezierIntersectionContacts2::Incomplete {
                contacts,
                candidates,
            },
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        },
        RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap } => {
            match retain(&contacts)? {
                Classification::Decided(contacts) => {
                    RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap }
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        result => result,
    };
    Ok(Classification::Decided(result))
}

fn append_complete_rational_contacts(
    replayed: RationalBezierIntersectionContacts2,
    additional: Vec<RationalBezierIntersectionContact2>,
) -> RationalBezierIntersectionContacts2 {
    if additional.is_empty() {
        return replayed;
    }
    let append = |contacts: Arc<[RationalBezierIntersectionContact2]>| {
        contacts
            .iter()
            .cloned()
            .chain(additional.iter().cloned())
            .collect::<Arc<[_]>>()
    };
    match replayed {
        RationalBezierIntersectionContacts2::NoIntersection => {
            RationalBezierIntersectionContacts2::Contacts(additional.into())
        }
        RationalBezierIntersectionContacts2::Contacts(contacts) => {
            RationalBezierIntersectionContacts2::Contacts(append(contacts))
        }
        RationalBezierIntersectionContacts2::Incomplete {
            contacts,
            candidates,
        } => RationalBezierIntersectionContacts2::Incomplete {
            contacts: append(contacts),
            candidates,
        },
        RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap } => {
            RationalBezierIntersectionContacts2::ContactsAndOverlap {
                contacts: append(contacts),
                overlap,
            }
        }
        replayed @ (RationalBezierIntersectionContacts2::Overlap(_)
        | RationalBezierIntersectionContacts2::DegenerateResultant) => replayed,
    }
}

impl PartialEq for RationalBezier2 {
    fn eq(&self, other: &Self) -> bool {
        self.control_points() == other.control_points() && self.weights() == other.weights()
    }
}

impl From<RationalQuadraticBezier2> for RationalBezier2 {
    fn from(curve: RationalQuadraticBezier2) -> Self {
        let control_points = curve.control_points().into_iter().cloned().collect();
        let weights = curve.weights().into_iter().cloned().collect();
        let implicit_quadratic_conic = curve.retained_implicit_quadratic_conic().cloned();
        let circular_conic = curve.retained_circular_conic().cloned();
        match implicit_quadratic_conic {
            Some(implicit_quadratic_conic) => Self::try_new_with_implicit_quadratic_conic(
                control_points,
                weights,
                implicit_quadratic_conic,
                circular_conic,
            ),
            None => Self::try_new(control_points, weights),
        }
        .expect("validated rational-quadratic controls remain valid after promotion")
    }
}

impl RationalBezier2 {
    pub(crate) fn shares_retained_data(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.data, &other.data)
    }

    pub(crate) fn retained_implicit_quadratic_conic(&self) -> Option<&Arc<[Real; 6]>> {
        self.data.lineage.root.implicit_quadratic_conic.get()
    }

    pub(crate) fn retained_circular_conic(
        &self,
    ) -> Option<&Arc<crate::rational_bezier::RationalQuadraticCircle2>> {
        self.data.lineage.root.circular_conic.get()
    }

    /// Recovers finite parameters for a point already certified on this
    /// curve's retained circular support.
    pub(crate) fn retained_circle_point_parameters(
        &self,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<BezierParameter2>>> {
        if point == self.start() {
            return Ok(Classification::Decided(vec![BezierParameter2::Exact(
                Real::zero(),
            )]));
        }
        if point == self.end() {
            return Ok(Classification::Decided(vec![BezierParameter2::Exact(
                Real::one(),
            )]));
        }
        // Circle-relation callers have already certified support incidence.
        // Recovering the retained projective parameter is therefore both the
        // direct proof and the finite-domain test; a Cartesian bounds proof
        // would only repeat exact coordinate work before the same inverse.
        if self.degree() == 2
            || self
                .data
                .lineage
                .root
                .quadratic_conic_parameter_frame
                .get()
                .is_some()
        {
            return Ok(
                match quadratic_conic_point_parameters(point, self, policy) {
                    Classification::Decided(Some(parameters)) => {
                        Classification::Decided(parameters)
                    }
                    Classification::Decided(None) => Classification::Decided(Vec::new()),
                    Classification::Uncertain(reason) => Classification::Uncertain(reason),
                },
            );
        }
        if let Classification::Decided(bounds) = self.certified_bounds_classified(policy)
            && matches!(
                bounds.contains_point(point, policy),
                Classification::Decided(false)
            )
        {
            return Ok(Classification::Decided(Vec::new()));
        }
        Ok(match self.point_incidence_classified(point, policy)? {
            Classification::Decided(RationalBezierPointIncidence2::Parameters(parameters)) => {
                Classification::Decided(parameters)
            }
            Classification::Decided(RationalBezierPointIncidence2::EntireCurve) => {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            }
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        })
    }

    pub(crate) fn try_from_subcurve(curve: &BezierSubcurve2) -> CurveResult<Self> {
        match curve {
            BezierSubcurve2::Quadratic(curve) => {
                let control_points = curve.control_points().into_iter().cloned().collect();
                let weights = vec![Real::one(); 3];
                match curve.retained_exact_line_image() {
                    Some(line) => {
                        Self::try_new_with_exact_line_image(control_points, weights, line.clone())
                    }
                    None => Self::try_new(control_points, weights),
                }
            }
            BezierSubcurve2::Cubic(curve) => Self::try_new(
                curve.control_points().into_iter().cloned().collect(),
                vec![Real::one(); 4],
            ),
            BezierSubcurve2::RationalQuadratic(curve) => Ok(Self::from(curve.clone())),
            BezierSubcurve2::Rational(curve) => Ok(curve.clone()),
        }
    }

    /// Constructs an exact positive-degree rational Bezier curve.
    pub fn try_new(control_points: Vec<Point2>, weights: Vec<Real>) -> CurveResult<Self> {
        Self::try_new_with_lineage(
            control_points,
            weights,
            RationalBezierLineage {
                root: Arc::new(RationalBezierLineageRoot::default()),
                range: ParamRange::new(Real::zero(), Real::one()),
            },
        )
    }

    pub(crate) fn try_new_with_exact_line_image(
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        exact_line_image: LineSeg2,
    ) -> CurveResult<Self> {
        Self::try_new_with_lineage_and_exact_line_image(
            control_points,
            weights,
            RationalBezierLineage {
                root: Arc::new(RationalBezierLineageRoot::default()),
                range: ParamRange::new(Real::zero(), Real::one()),
            },
            Some(exact_line_image),
        )
    }

    pub(crate) fn try_new_with_implicit_quadratic_conic(
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        implicit_quadratic_conic: Arc<[Real; 6]>,
        circular_conic: Option<Arc<crate::rational_bezier::RationalQuadraticCircle2>>,
    ) -> CurveResult<Self> {
        let root = Arc::new(RationalBezierLineageRoot::default());
        let _ = root.implicit_quadratic_conic.set(implicit_quadratic_conic);
        if let Some(circular_conic) = circular_conic {
            let _ = root.circular_conic.set(circular_conic);
        }
        Self::try_new_with_lineage(
            control_points,
            weights,
            RationalBezierLineage {
                root,
                range: ParamRange::new(Real::zero(), Real::one()),
            },
        )
    }

    fn try_new_with_lineage(
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        lineage: RationalBezierLineage,
    ) -> CurveResult<Self> {
        Self::try_new_with_lineage_and_exact_line_image(control_points, weights, lineage, None)
    }

    fn try_new_with_lineage_and_exact_line_image(
        control_points: Vec<Point2>,
        weights: Vec<Real>,
        lineage: RationalBezierLineage,
        exact_line_image: Option<LineSeg2>,
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
            data: Arc::new(RationalBezierData {
                control_points,
                weights,
                exact_line_image,
                lineage,
                homogeneous_controls: OnceLock::new(),
                homogeneous_power_basis: OnceLock::new(),
                x_derivative_numerator_bernstein: OnceLock::new(),
                y_derivative_numerator_bernstein: OnceLock::new(),
                x_axis_monotonicity: OnceLock::new(),
                y_axis_monotonicity: OnceLock::new(),
                degree_elevations: OnceLock::new(),
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

    pub(crate) fn retained_exact_line_image(&self) -> Option<&LineSeg2> {
        self.data.exact_line_image.as_ref()
    }

    pub(crate) fn exact_linear_parameterization_line(&self) -> Option<LineSeg2> {
        if let Some(line) = self.retained_exact_line_image() {
            return Some(line.clone());
        }
        if self.weights().iter().any(|weight| weight != &Real::one()) {
            return None;
        }
        let line = LineSeg2::try_new(self.start().clone(), self.end().clone()).ok()?;
        match self.degree() {
            1 => Some(line),
            2 => {
                let half =
                    (Real::one() / Real::from(2_i8)).expect("two is a nonzero exact denominator");
                (self.control_points()[1] == line.point_at(half)).then_some(line)
            }
            _ => None,
        }
    }

    /// Returns exact homogeneous weights in Bernstein order.
    pub fn weights(&self) -> &[Real] {
        &self.data.weights
    }

    /// Returns the exact parameter range in the root curve's source domain.
    pub fn source_parameter_range(&self) -> &ParamRange {
        &self.data.lineage.range
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
                CurveError::InvalidDegreeElevation,
            ));
        }
        if target_degree == source_degree {
            return Ok(self.clone());
        }
        if source_degree == 2 {
            self.retain_quadratic_conic_parameter_frame(&CurveContext::STRICT);
        }
        let elevation_count = target_degree.checked_sub(source_degree).ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::RationalBezier,
                CurveError::InvalidDegreeElevation,
            )
        })?;
        let elevations = self
            .data
            .degree_elevations
            .get_or_init(|| Mutex::new(Vec::new()));
        while elevations
            .lock()
            .expect("rational Bézier degree elevation cache mutex poisoned")
            .len()
            < elevation_count
        {
            let source = {
                let retained = elevations
                    .lock()
                    .expect("rational Bézier degree elevation cache mutex poisoned");
                match retained.last() {
                    Some(Ok(curve)) => Ok(curve.clone()),
                    Some(Err(error)) => Err(error.clone()),
                    None => Ok(self.clone()),
                }
            };
            let elevated = source.and_then(|curve| curve.elevate_once_uncached());
            elevations
                .lock()
                .expect("rational Bézier degree elevation cache mutex poisoned")
                .push(elevated);
        }
        elevations
            .lock()
            .expect("rational Bézier degree elevation cache mutex poisoned")[elevation_count - 1]
            .clone()
    }

    fn elevate_once_uncached(&self) -> ExactCurveResult<Self> {
        let target_degree = self.degree().checked_add(1).ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::RationalBezier,
                CurveError::InvalidDegreeElevation,
            )
        })?;
        let denominator = u64::try_from(target_degree).map(Real::from).map_err(|_| {
            ExactCurveError::invalid(
                CurveOperation2::DegreeElevation,
                CurveFamily2::RationalBezier,
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
                    CurveError::InvalidDegreeElevation,
                )
            })?;
            let alpha = (numerator / &denominator).map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::DegreeElevation,
                    CurveFamily2::RationalBezier,
                    cause.into(),
                )
            })?;
            homogeneous.push(source[index].lerp(&source[index - 1], &alpha));
        }
        homogeneous.push(source[source.len() - 1].clone());

        let policy = CurveContext::STRICT;
        let mut control_points = Vec::with_capacity(homogeneous.len());
        let mut weights = Vec::with_capacity(homogeneous.len());
        for point in homogeneous {
            match project_homogeneous(&point, &policy) {
                Classification::Decided(control) => control_points.push(control),
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::DegreeElevation,
                        CurveFamily2::RationalBezier,
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
                    cause,
                )
            },
        )
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
    pub fn point_at(&self, parameter: &Real, policy: &CurveContext) -> ExactCurveResult<Point2> {
        match self.point_at_classified(parameter, policy) {
            Classification::Decided(point) => Ok(point),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        }
    }

    pub(crate) fn point_at_classified(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> Classification<Point2> {
        if in_closed_unit_interval(parameter, policy) != Some(true) {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        self.point_at_affine_classified(parameter, policy)
    }

    /// Evaluates any finite affine parameter without imposing the authored
    /// unit-domain restriction.
    ///
    /// Callers must separately prove that the parameter belongs to the
    /// intended pole-partitioned projective cell. Projection still rejects a
    /// zero or undecidable homogeneous weight, so this cannot turn a point at
    /// infinity into affine geometry.
    pub(crate) fn point_at_affine_classified(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> Classification<Point2> {
        if parameter.zero_status() == ZeroKnowledge::Zero {
            return Classification::Decided(self.start().clone());
        }
        if (Real::one() - parameter).zero_status() == ZeroKnowledge::Zero {
            return Classification::Decided(self.end().clone());
        }
        if self.degree() > MAX_RETAINED_EVALUATION_POWER_DEGREE
            && self.data.homogeneous_power_basis.get().is_none()
        {
            return match self.homogeneous_bernstein_value(parameter, policy) {
                Classification::Decided(point) => project_homogeneous(&point, policy),
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            };
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

    fn homogeneous_bernstein_value(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> Classification<HomogeneousPoint2> {
        if is_zero(parameter, policy) == Some(true) {
            return Classification::Decided(self.homogeneous_controls()[0].clone());
        }
        let one_minus_parameter = Real::one() - parameter;
        if is_zero(&one_minus_parameter, policy) == Some(true) {
            return Classification::Decided(
                self.homogeneous_controls()
                    .last()
                    .expect("validated rational Bezier has controls")
                    .clone(),
            );
        }
        if is_zero(&one_minus_parameter, policy) != Some(false) {
            return self.homogeneous_de_casteljau_value(parameter);
        }
        let Ok(parameter_ratio) = parameter / &one_minus_parameter else {
            return self.homogeneous_de_casteljau_value(parameter);
        };
        let mut basis = real_nonnegative_integer_power(&one_minus_parameter, self.degree());
        let controls = self.homogeneous_controls();
        let mut value = controls[0].scaled(&basis);
        for (index, control) in controls.iter().enumerate().skip(1) {
            let Ok(numerator) = u64::try_from(self.degree() - index + 1) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let Ok(denominator) = u64::try_from(index) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            basis = basis * &parameter_ratio * Real::from(numerator);
            let Ok(next_basis) = basis / Real::from(denominator) else {
                return self.homogeneous_de_casteljau_value(parameter);
            };
            basis = next_basis;
            value.add_scaled(control, &basis);
        }
        Classification::Decided(value)
    }

    fn homogeneous_de_casteljau_value(
        &self,
        parameter: &Real,
    ) -> Classification<HomogeneousPoint2> {
        let mut level = self.homogeneous_controls().to_vec();
        let one_minus_parameter = Real::one() - parameter;
        for next_len in (1..level.len()).rev() {
            for index in 0..next_len {
                level[index] = level[index].lerp_with_complement(
                    &level[index + 1],
                    parameter,
                    &one_minus_parameter,
                );
            }
        }
        Classification::Decided(level.remove(0))
    }

    /// Evaluates the exact affine derivative with respect to the Bezier parameter.
    pub fn derivative_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveDerivative2> {
        match self.derivative_at_classified(parameter, policy) {
            Classification::Decided(derivative) => Ok(derivative),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        }
    }

    pub(crate) fn derivative_at_classified(
        &self,
        parameter: &Real,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> ExactCurveResult<Vec<CurveDerivative2>> {
        match self.derivatives_at_classified(parameter, max_order, policy) {
            Classification::Decided(derivatives) => Ok(derivatives),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Evaluation,
                CurveFamily2::RationalBezier,
                reason,
            )),
        }
    }

    pub(crate) fn derivatives_at_classified(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicPointImage2> {
        if let Some(image) = parameter.cached_rational_bezier_point_image(self) {
            return Ok(image);
        }
        let power_basis = self.homogeneous_power_basis()?;
        let image = rational_point_image_from_power_basis(
            parameter,
            power_basis.x_numerator.clone(),
            power_basis.y_numerator.clone(),
            power_basis.weight.clone(),
            policy,
        )?;
        if image.status() == crate::BezierAlgebraicImageStatus::Transformed {
            parameter.retain_rational_bezier_point_image(self, image.clone());
        }
        Ok(image)
    }

    /// Evaluates the affine tangent at an isolated algebraic parameter.
    pub fn tangent_at_algebraic_parameter(
        &self,
        parameter: &crate::BezierAlgebraicParameter2,
        policy: &CurveContext,
    ) -> CurveResult<RationalBezierAlgebraicTangentImage2> {
        Ok(self
            .derivatives_at_algebraic_parameter(parameter, 1, policy)?
            .pop()
            .expect("one requested rational derivative image"))
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
        policy: &CurveContext,
    ) -> CurveResult<Vec<RationalBezierAlgebraicTangentImage2>> {
        if let Some(images) = parameter.cached_rational_bezier_derivative_images(self, max_order) {
            return Ok(images);
        }
        let power_basis = self.homogeneous_power_basis()?;
        let images = rational_derivative_images_from_power_basis(
            parameter,
            power_basis.x_numerator.clone(),
            power_basis.y_numerator.clone(),
            power_basis.weight.clone(),
            policy,
            max_order,
        )?;
        if images
            .iter()
            .all(|image| image.status() == crate::BezierAlgebraicImageStatus::Transformed)
        {
            parameter.retain_rational_bezier_derivative_images(self, images.clone());
        }
        Ok(images)
    }

    /// Returns a conservative exact control-hull bound when all weights share a sign.
    pub fn certified_bounds(&self, policy: &CurveContext) -> ExactCurveResult<Aabb2> {
        match self.certified_bounds_classified(policy) {
            Classification::Decided(bounds) => Ok(bounds),
            Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
                CurveOperation2::Classification,
                CurveFamily2::RationalBezier,
                reason,
            )),
        }
    }

    pub(crate) fn certified_bounds_classified(
        &self,
        policy: &CurveContext,
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
    pub fn axis_is_monotone(&self, axis: Axis2, policy: &CurveContext) -> ExactCurveResult<bool> {
        match self.axis_monotonicity_classified(axis, policy) {
            Ok(Classification::Decided(monotone)) => Ok(monotone),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Classification,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Classification,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    pub(crate) fn axis_monotonicity_classified(
        &self,
        axis: Axis2,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if let Classification::Uncertain(reason) = self.common_weight_sign(policy) {
            return Ok(Classification::Uncertain(reason));
        }
        if self.control_polygon_certifies_axis_monotone(axis, policy) {
            return Ok(Classification::Decided(true));
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
        policy: &CurveContext,
    ) -> Classification<BezierLineContactRelation> {
        let weight_sign = match self.common_weight_sign(policy) {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let control_sides = self
            .control_points()
            .iter()
            .map(|point| classify_oriented_line(line.start(), line.end(), point, policy))
            .collect::<Vec<_>>();
        for side in [LineSide::Left, LineSide::Right] {
            if control_sides.iter().all(
                |candidate| matches!(candidate, Classification::Decided(value) if *value == side),
            ) {
                return Classification::Decided(BezierLineContactRelation::ControlHullDisjoint {
                    side,
                });
            }
        }
        if control_sides
            .iter()
            .all(|side| matches!(side, Classification::Decided(LineSide::On)))
        {
            return Classification::Decided(BezierLineContactRelation::OnSupportingLine);
        }
        if self.degree() == 2
            && let Some(circle) = self.data.lineage.root.circular_conic.get()
        {
            let (line_dx, line_dy) = line.delta();
            let (from_center_x, from_center_y) = line.start().delta_from(&circle.center);
            let quadratic = Real::dot2_refs([&line_dx, &line_dy], [&line_dx, &line_dy]);
            let half_linear =
                Real::dot2_refs([&from_center_x, &from_center_y], [&line_dx, &line_dy]);
            let one = Real::one();
            let constant = Real::signed_product_sum(
                [true, true, false],
                [
                    [&from_center_x, &from_center_x],
                    [&from_center_y, &from_center_y],
                    [&circle.radius_squared, &one],
                ],
            );
            let discriminant = Real::signed_product_sum(
                [true, false],
                [[&half_linear, &half_linear], [&quadratic, &constant]],
            );
            let (line_parameters, kind) = match real_sign(&discriminant, policy) {
                Some(RealSign::Negative) => {
                    return Classification::Decided(BezierLineContactRelation::NoContact);
                }
                Some(RealSign::Zero) => {
                    let Ok(parameter) = (-half_linear) / &quadratic else {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    };
                    (vec![parameter], BezierLineContactKind::Tangent)
                }
                Some(RealSign::Positive) => {
                    let Ok(root) = discriminant.sqrt() else {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    };
                    let negative_half_linear = -half_linear;
                    let Ok(first) = (&negative_half_linear - &root) / &quadratic else {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    };
                    let Ok(second) = (negative_half_linear + root) / quadratic else {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    };
                    (vec![first, second], BezierLineContactKind::Crossing)
                }
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            };
            let mut contacts = Vec::with_capacity(line_parameters.len());
            for line_parameter in line_parameters {
                let point = Point2::new(
                    line.start().x() + &line_dx * &line_parameter,
                    line.start().y() + &line_dy * &line_parameter,
                );
                let parameters = match quadratic_conic_point_parameters(&point, self, policy) {
                    Classification::Decided(Some(parameters)) => parameters,
                    Classification::Decided(None) => continue,
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                };
                for parameter in parameters {
                    let crossing_direction = if kind == BezierLineContactKind::Crossing {
                        let BezierParameter2::Exact(exact) = &parameter else {
                            return Classification::Uncertain(UncertaintyReason::Unsupported);
                        };
                        let Classification::Decided(derivative) =
                            self.derivative_at_classified(exact, policy)
                        else {
                            return Classification::Uncertain(UncertaintyReason::RealSign);
                        };
                        let signed_derivative = Real::signed_product_sum(
                            [true, false],
                            [[&line_dx, derivative.dy()], [&line_dy, derivative.dx()]],
                        );
                        match real_sign(&signed_derivative, policy) {
                            Some(RealSign::Positive) => {
                                Some(BezierLineCrossingDirection::NegativeToPositive)
                            }
                            Some(RealSign::Negative) => {
                                Some(BezierLineCrossingDirection::PositiveToNegative)
                            }
                            Some(RealSign::Zero) | None => {
                                return Classification::Uncertain(UncertaintyReason::RealSign);
                            }
                        }
                    } else {
                        None
                    };
                    let Ok(contact) =
                        crate::BezierLineContact::with_crossing_direction_and_line_parameter(
                            parameter,
                            kind,
                            crossing_direction,
                            line_parameter.clone(),
                            policy,
                        )
                    else {
                        return Classification::Uncertain(UncertaintyReason::Ordering);
                    };
                    contacts.push(contact);
                }
            }
            if contacts.len() == 2 {
                match contacts[0]
                    .parameter()
                    .cmp_by_interval(contacts[1].parameter(), policy)
                {
                    Ok(Classification::Decided(Ordering::Greater)) => contacts.swap(0, 1),
                    Ok(Classification::Decided(_)) => {}
                    Ok(Classification::Uncertain(reason)) => {
                        return Classification::Uncertain(reason);
                    }
                    Err(_) => return Classification::Uncertain(UncertaintyReason::Ordering),
                }
            }
            return Classification::Decided(if contacts.is_empty() {
                BezierLineContactRelation::NoContact
            } else {
                BezierLineContactRelation::Contacts { contacts }
            });
        }
        let weighted_distances = self
            .control_points()
            .iter()
            .enumerate()
            .zip(self.weights())
            .map(|((index, point), weight)| {
                if (index == 0 || index + 1 == self.control_points().len())
                    && (point == line.start()
                        || point == line.end()
                        || is_zero(&point.distance_squared(line.start()), policy) == Some(true)
                        || is_zero(&point.distance_squared(line.end()), policy) == Some(true))
                {
                    Real::zero()
                } else {
                    let normalized_weight = if weight_sign == RealSign::Negative {
                        -weight.clone()
                    } else {
                        weight.clone()
                    };
                    orient2_real_expr(line.start(), line.end(), point) * normalized_weight
                }
            })
            .collect::<Vec<_>>();

        exact_line_contact_relation_from_bernstein_distances(weighted_distances, policy)
    }

    pub(crate) fn relation_to_line_with_certified_crossing(
        &self,
        line: &LineSeg2,
        parameter: &Real,
        crossing_direction: BezierLineCrossingDirection,
        policy: &CurveContext,
    ) -> Classification<BezierLineContactRelation> {
        if self.degree() != 2 || self.retained_circular_conic().is_none() {
            return self.relation_to_line_with_contacts(line, policy);
        }
        let weight_sign = match self.common_weight_sign(policy) {
            Classification::Decided(sign) => sign,
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        };
        let weighted_distances = self
            .control_points()
            .iter()
            .enumerate()
            .zip(self.weights())
            .map(|((index, point), weight)| {
                if (index == 0 || index + 1 == self.control_points().len())
                    && (point == line.start()
                        || point == line.end()
                        || is_zero(&point.distance_squared(line.start()), policy) == Some(true)
                        || is_zero(&point.distance_squared(line.end()), policy) == Some(true))
                {
                    Real::zero()
                } else {
                    let normalized_weight = if weight_sign == RealSign::Negative {
                        -weight.clone()
                    } else {
                        weight.clone()
                    };
                    orient2_real_expr(line.start(), line.end(), point) * normalized_weight
                }
            })
            .collect::<Vec<_>>();
        let Ok(distances) = <Vec<Real> as TryInto<[Real; 3]>>::try_into(weighted_distances) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        exact_quadratic_line_contact_relation_with_certified_crossing(
            distances,
            parameter,
            crossing_direction,
            policy,
        )
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
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezierPointIncidence2> {
        match self.point_incidence_classified(point, policy) {
            Ok(Classification::Decided(incidence)) => Ok(incidence),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    pub(crate) fn point_incidence_classified(
        &self,
        point: &Point2,
        policy: &CurveContext,
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
    pub fn contains_point(&self, point: &Point2, policy: &CurveContext) -> ExactCurveResult<bool> {
        self.point_incidence(point, policy)
            .map(|incidence| match incidence {
                RationalBezierPointIncidence2::EntireCurve => true,
                RationalBezierPointIncidence2::Parameters(parameters) => !parameters.is_empty(),
            })
    }

    pub(crate) fn contains_point_classified(
        &self,
        point: &Point2,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezierIntersectionCandidates2> {
        match self.intersection_candidates_classified(other, policy) {
            Ok(Classification::Decided(candidates)) => Ok(candidates),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    /// Computes exact contact-derived split topology immediately.
    pub fn intersection_topology(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezierIntersectionTopology2> {
        RationalBezierIntersectionContext::try_new(self, other, policy)?.try_topology()
    }

    fn intersection_context_classified(
        &self,
        other: &Self,
        policy: &CurveContext,
        circle_relation: Option<&CircleCircleRelation>,
        first_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
        second_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
    ) -> CurveResult<Classification<RationalBezierIntersectionContext>> {
        if self == other {
            let overlap = RationalBezierIntersectionOverlap2 {
                first_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                second_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                orientation: RationalBezierOverlapOrientation2::Same,
                endpoint_inclusion: [true, true],
            };
            let contacts = RationalBezierIntersectionContacts2::Overlap(overlap);
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if self
            .control_points()
            .iter()
            .rev()
            .eq(other.control_points().iter())
            && self.weights().iter().rev().eq(other.weights().iter())
        {
            let overlap = RationalBezierIntersectionOverlap2 {
                first_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::zero()),
                    BezierParameter2::Exact(Real::one()),
                ),
                second_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(Real::one()),
                    BezierParameter2::Exact(Real::zero()),
                ),
                orientation: RationalBezierOverlapOrientation2::Reversed,
                endpoint_inclusion: [true, true],
            };
            let contacts = RationalBezierIntersectionContacts2::Overlap(overlap);
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if self.certified_bounds_are_disjoint(other, policy) {
            let contacts = OnceLock::new();
            let _ = contacts.set(Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates: RationalBezierIntersectionCandidates2::NoIntersection,
                    contacts,
                },
            }));
        }
        if let Some(contacts) = self.retained_lineage_intersection_contacts(other, policy)? {
            let contacts = match contacts {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if let Some(contacts) = self.circular_conic_intersection_contacts(
            other,
            policy,
            circle_relation,
            first_circle_parameters,
            second_circle_parameters,
        )? {
            let contacts = match contacts {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if let Some(contacts) = self.retained_linear_image_contacts(other, policy)? {
            let contacts = match contacts {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if let Some(Classification::Decided(contacts)) =
            self.certified_linear_image_contacts(other, policy)?
        {
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }

        // A line image can arrive degree-elevated through several native
        // carriers. Align it with the nonlinear operand before elimination so
        // the resultant does not retain parameterization-only base factors.
        if self.degree() < other.degree()
            && matches!(
                self.fit_exact_line_image(policy)?,
                Classification::Decided(BezierLineImageFitRelation::Fit(_))
            )
        {
            let elevated = match self.elevated_to_degree(other.degree()) {
                Ok(elevated) => elevated,
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Ok(Classification::Uncertain(blocker.reason()));
                }
                Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            };
            return elevated.intersection_context_classified(other, policy, None, None, None);
        }
        if other.degree() < self.degree()
            && matches!(
                other.fit_exact_line_image(policy)?,
                Classification::Decided(BezierLineImageFitRelation::Fit(_))
            )
        {
            let elevated = match other.elevated_to_degree(self.degree()) {
                Ok(elevated) => elevated,
                Err(ExactCurveError::Blocked(blocker)) => {
                    return Ok(Classification::Uncertain(blocker.reason()));
                }
                Err(ExactCurveError::Invalid { cause, .. }) => return Err(cause),
            };
            return self.intersection_context_classified(&elevated, policy, None, None, None);
        }

        let line_image_contacts =
            if let Some(contacts) = self.exact_line_image_intersection_contacts(other, policy)? {
                Some(contacts)
            } else {
                other
                    .exact_line_image_intersection_contacts(self, policy)?
                    .map(|contacts| contacts.map(reverse_rational_intersection_contacts))
            };
        if let Some(Classification::Decided(contacts)) = line_image_contacts {
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        if matches!(
            self.shares_implicit_quadratic_conic(other, policy),
            Classification::Decided(true)
        ) {
            match self.replay_intersection_candidate_set(
                other,
                &RationalBezierIntersectionCandidates2::DegenerateResultant,
                policy,
            )? {
                Classification::Decided(
                    RationalBezierIntersectionContacts2::DegenerateResultant,
                ) => {}
                Classification::Decided(contacts) => {
                    let candidates = intersection_candidates_from_contacts(&contacts);
                    let contact_cache = OnceLock::new();
                    let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
                    return Ok(Classification::Decided(RationalBezierIntersectionContext {
                        data: RationalBezierIntersectionContextData {
                            first: self.clone(),
                            second: other.clone(),
                            policy: *policy,
                            candidates,
                            contacts: contact_cache,
                        },
                    }));
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let special =
            if let Some(contacts) = self.implicit_conic_intersection_contacts(other, policy)? {
                Some(contacts)
            } else {
                other
                    .implicit_conic_intersection_contacts(self, policy)?
                    .map(|contacts| contacts.map(reverse_rational_intersection_contacts))
            };
        if let Some(Classification::Decided(contacts)) = special {
            let candidates = intersection_candidates_from_contacts(&contacts);
            let contact_cache = OnceLock::new();
            let _ = contact_cache.set(Ok(Classification::Decided(contacts)));
            return Ok(Classification::Decided(RationalBezierIntersectionContext {
                data: RationalBezierIntersectionContextData {
                    first: self.clone(),
                    second: other.clone(),
                    policy: *policy,
                    candidates,
                    contacts: contact_cache,
                },
            }));
        }
        // Bounds were already checked before the implicit-conic fast path.
        // Continue directly so an overlapping pair is not boxed and compared
        // a second time before resultant construction.
        match self.intersection_candidates_after_bounds_check(other, policy)? {
            Classification::Decided(candidates) => {
                Ok(Classification::Decided(RationalBezierIntersectionContext {
                    data: RationalBezierIntersectionContextData {
                        first: self.clone(),
                        second: other.clone(),
                        policy: *policy,
                        candidates,
                        contacts: OnceLock::new(),
                    },
                }))
            }
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        }
    }

    fn retained_linear_image_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let (Some(first), Some(second)) = (
            self.exact_linear_parameterization_line(),
            other.exact_linear_parameterization_line(),
        ) else {
            return Ok(None);
        };
        Ok(Some(match first.intersect_line(&second, policy)? {
            crate::LineLineIntersection::None => {
                Classification::Decided(RationalBezierIntersectionContacts2::NoIntersection)
            }
            crate::LineLineIntersection::Point {
                point,
                a_param,
                b_param,
                kind,
            } => {
                Classification::Decided(RationalBezierIntersectionContacts2::Contacts(Arc::from([
                    RationalBezierIntersectionContact2 {
                        first_parameter: BezierParameter2::Exact(a_param),
                        second_parameter: BezierParameter2::Exact(b_param),
                        point: RationalBezierIntersectionPointEvidence2::Exact(point),
                        certified_transverse: kind == crate::IntersectionKind::Crossing,
                        tangent_cross_sign: None,
                    },
                ])))
            }
            crate::LineLineIntersection::Overlap { .. } => return Ok(None),
            crate::LineLineIntersection::Uncertain { reason } => Classification::Uncertain(reason),
        }))
    }

    fn certified_linear_image_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let first = match self.fit_exact_line_image(policy)? {
            Classification::Decided(BezierLineImageFitRelation::Fit(first)) => first,
            Classification::Decided(BezierLineImageFitRelation::NotLine) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let second = match other.fit_exact_line_image(policy)? {
            Classification::Decided(BezierLineImageFitRelation::Fit(second)) => second,
            Classification::Decided(BezierLineImageFitRelation::NotLine) => return Ok(None),
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        Ok(match first.line().intersect_line(second.line(), policy)? {
            crate::LineLineIntersection::None => Some(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )),
            crate::LineLineIntersection::Point { point, kind, .. } => {
                let unique_parameter =
                    |curve: &Self| match unique_point_incidence_parameter(curve, &point, policy) {
                        Classification::Decided(Some(parameter)) => Ok(parameter),
                        Classification::Decided(None) => Err(UncertaintyReason::Predicate),
                        Classification::Uncertain(reason) => Err(reason),
                    };
                let first_parameter = match unique_parameter(self) {
                    Ok(parameter) => parameter,
                    Err(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                };
                let second_parameter = match unique_parameter(other) {
                    Ok(parameter) => parameter,
                    Err(reason) => return Ok(Some(Classification::Uncertain(reason))),
                };
                Some(Classification::Decided(
                    RationalBezierIntersectionContacts2::Contacts(Arc::from([
                        RationalBezierIntersectionContact2 {
                            first_parameter,
                            second_parameter,
                            point: RationalBezierIntersectionPointEvidence2::Exact(point),
                            certified_transverse: kind == crate::IntersectionKind::Crossing,
                            tangent_cross_sign: None,
                        },
                    ])),
                ))
            }
            crate::LineLineIntersection::Overlap { .. } => {
                match self.certified_line_image_overlap(other, policy) {
                    Classification::Decided(Some(overlap)) => Some(Classification::Decided(
                        RationalBezierIntersectionContacts2::Overlap(overlap),
                    )),
                    Classification::Decided(None) => None,
                    Classification::Uncertain(reason) => Some(Classification::Uncertain(reason)),
                }
            }
            crate::LineLineIntersection::Uncertain { reason } => {
                Some(Classification::Uncertain(reason))
            }
        })
    }

    /// Replays all resultant projections into exact paired contacts.
    ///
    /// The result distinguishes complete contact sets from partial algebraic
    /// replay. No raw resultant root is accepted as a contact without exact
    /// equality of both constructed affine coordinates.
    pub fn intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezierIntersectionContacts2> {
        match self.intersection_contacts_classified(other, policy) {
            Ok(Classification::Decided(contacts)) => Ok(contacts),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    pub(crate) fn intersection_contacts_classified(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        if self.certified_bounds_are_disjoint(other, policy) {
            return Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            ));
        }
        if let Some(contacts) = self.retained_lineage_intersection_contacts(other, policy)? {
            return Ok(contacts);
        }
        if let Some(Classification::Decided(contacts)) =
            self.certified_linear_image_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }

        // A symbolic quadratic conic is better served by its implicit
        // equation than by direct Bernstein line-side signs. The latter can
        // exhaust the sign budget before the exact algebraic replay that is
        // specifically able to retain the symbolic coefficient field.
        if self.degree() == 2
            && self
                .weights()
                .iter()
                .any(|weight| weight.exact_rational_ref().is_none())
            && let Some(Classification::Decided(contacts)) =
                self.implicit_conic_intersection_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }
        if other.degree() == 2
            && other
                .weights()
                .iter()
                .any(|weight| weight.exact_rational_ref().is_none())
            && let Some(Classification::Decided(contacts)) =
                other.implicit_conic_intersection_contacts(self, policy)?
        {
            return Ok(Classification::Decided(
                reverse_rational_intersection_contacts(contacts),
            ));
        }

        // A line-image shortcut can be unable to classify a transcendental
        // conic even though the implicit-conic route below can replay the
        // contact exactly. Treat only decided shortcut results as terminal.
        if let Some(Classification::Decided(contacts)) =
            self.exact_line_image_intersection_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }
        if let Some(Classification::Decided(contacts)) =
            other.exact_line_image_intersection_contacts(self, policy)?
        {
            return Ok(Classification::Decided(
                reverse_rational_intersection_contacts(contacts),
            ));
        }

        if let Some(Classification::Decided(contacts)) =
            self.implicit_conic_intersection_contacts(other, policy)?
        {
            return Ok(Classification::Decided(contacts));
        }
        if let Some(Classification::Decided(contacts)) =
            other.implicit_conic_intersection_contacts(self, policy)?
        {
            return Ok(Classification::Decided(
                reverse_rational_intersection_contacts(contacts),
            ));
        }
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
        // Bounds were already checked before the implicit-conic fast path.
        // Continue directly so an overlapping or inconclusive pair is not
        // boxed and compared a second time before resultant construction.
        let candidates = match self.intersection_candidates_after_bounds_check(other, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        self.replay_intersection_candidate_set(other, &candidates, policy)
    }

    /// Returns every unordered off-diagonal self-contact of this curve.
    ///
    /// Both homogeneous coordinate-equality equations contain the universal
    /// `u - t` identity component. The self-contact authority divides that
    /// component exactly before elimination, projects the resulting symmetric
    /// system once, and reuses ordinary affine contact replay. A remaining
    /// positive-dimensional correspondence is reported as a degenerate
    /// resultant instead of being mistaken for isolated contacts.
    pub fn self_intersection_contacts(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<RationalBezierIntersectionContacts2> {
        match self.self_intersection_contacts_classified(policy) {
            Ok(Classification::Decided(contacts)) => Ok(contacts),
            Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                reason,
            )),
            Err(cause) => Err(ExactCurveError::invalid(
                CurveOperation2::Intersection,
                CurveFamily2::RationalBezier,
                cause,
            )),
        }
    }

    pub(crate) fn self_intersection_contacts_classified(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        self.self_intersection_contacts_with_point_evidence_classified(policy, &mut |_| Ok(None))
    }

    pub(crate) fn self_intersection_contacts_with_point_evidence_classified(
        &self,
        policy: &CurveContext,
        fallback_point_evidence: &mut dyn FnMut(
            &BezierParameter2,
        ) -> CurveResult<
            Option<RationalBezierIntersectionPointEvidence2>,
        >,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        if self.has_certified_injective_axis(policy) {
            return Ok(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            ));
        }
        if let Classification::Uncertain(reason) = self.common_weight_sign(policy) {
            return Ok(Classification::Uncertain(reason));
        }
        let basis = self.homogeneous_power_basis()?;
        let Some(equations) = rational_self_intersection_residual_system(basis) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let candidates = match project_symmetric_self_intersection_system(&equations, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let replayed = match &candidates {
            RationalBezierIntersectionCandidates2::NoIntersection => {
                RationalBezierIntersectionContacts2::NoIntersection
            }
            RationalBezierIntersectionCandidates2::DegenerateResultant => {
                RationalBezierIntersectionContacts2::DegenerateResultant
            }
            RationalBezierIntersectionCandidates2::Candidates {
                first_parameters,
                second_parameters,
            } => match self.replay_intersection_candidates_with_pair_filter(
                self,
                first_parameters,
                second_parameters,
                true,
                Some(&equations),
                Some(fallback_point_evidence),
                policy,
            )? {
                Classification::Decided(replayed) => replayed,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            },
        };
        retain_unordered_rational_self_contacts(replayed, basis, policy)
    }

    fn retained_lineage_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        if !Arc::ptr_eq(&self.data.lineage.root, &other.data.lineage.root)
            || self == other
            || (self
                .control_points()
                .iter()
                .rev()
                .eq(other.control_points().iter())
                && self.weights().iter().rev().eq(other.weights().iter()))
        {
            return Ok(None);
        }
        self.retain_root_image_injectivity(policy);
        other.retain_root_image_injectivity(policy);
        if self.data.lineage.root.image_is_injective.get() == Some(&true) {
            return Ok(None);
        }

        let source_overlap = match self.retained_source_parameter_overlap(other, policy) {
            Classification::Decided(overlap) => overlap,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let Some(equations) = rational_retained_lineage_residual_system(self, other)? else {
            return Ok(Some(Classification::Uncertain(
                UncertaintyReason::Unsupported,
            )));
        };
        let candidates = match project_retained_lineage_residual_system(&equations, policy)? {
            Classification::Decided(candidates) => candidates,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let replayed = if matches!(
            candidates,
            RationalBezierIntersectionCandidates2::DegenerateResultant
        ) {
            RationalBezierIntersectionContacts2::DegenerateResultant
        } else {
            match self.replay_intersection_candidate_set(other, &candidates, policy)? {
                Classification::Decided(replayed) => replayed,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        };
        let replayed = match self.remove_same_source_parameter_contacts(other, replayed, policy)? {
            Classification::Decided(replayed) => replayed,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let replayed = if replayed.isolated_contacts().is_empty() {
            replayed
        } else {
            let tangent_cross = rational_pair_tangent_cross_polynomial(
                self.homogeneous_power_basis()?,
                other.homogeneous_power_basis()?,
            );
            match retain_rational_contact_tangent_cross_signs(
                replayed,
                tangent_cross.as_ref(),
                policy,
            )? {
                Classification::Decided(replayed) => replayed,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        };

        let replayed = if source_overlap.is_none() {
            let identity_contacts = match self.retained_lineage_touch_contacts(other, policy)? {
                Classification::Decided(contacts) => contacts,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            append_complete_rational_contacts(replayed, identity_contacts)
        } else {
            replayed
        };
        let result = match (source_overlap, replayed) {
            (None, replayed) => replayed,
            (Some(overlap), RationalBezierIntersectionContacts2::NoIntersection) => {
                RationalBezierIntersectionContacts2::Overlap(overlap)
            }
            (Some(overlap), RationalBezierIntersectionContacts2::Contacts(contacts)) => {
                RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap }
            }
            (
                Some(_),
                RationalBezierIntersectionContacts2::Incomplete { .. }
                | RationalBezierIntersectionContacts2::DegenerateResultant,
            ) => RationalBezierIntersectionContacts2::DegenerateResultant,
            (Some(_), RationalBezierIntersectionContacts2::Overlap(_))
            | (Some(_), RationalBezierIntersectionContacts2::ContactsAndOverlap { .. }) => {
                unreachable!("residual replay cannot produce an image overlap")
            }
        };
        Ok(Some(Classification::Decided(result)))
    }

    fn remove_same_source_parameter_contacts(
        &self,
        other: &Self,
        replayed: RationalBezierIntersectionContacts2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        let first_range = self.source_parameter_range();
        let second_range = other.source_parameter_range();
        let numerator = vec![
            first_range.start() - second_range.start(),
            first_range.end() - first_range.start(),
        ];
        let denominator = vec![second_range.end() - second_range.start()];
        if is_zero(&denominator[0], policy) != Some(false) {
            return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
        }
        let retain = |contacts: Arc<[RationalBezierIntersectionContact2]>|
         -> CurveResult<Classification<Arc<[RationalBezierIntersectionContact2]>>> {
            let mut retained = Vec::with_capacity(contacts.len());
            for contact in contacts.iter() {
                match rational_parameter_image_matches(
                    contact.first_parameter(),
                    contact.second_parameter(),
                    &numerator,
                    &denominator,
                    policy,
                )? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => retained.push(contact.clone()),
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            Ok(Classification::Decided(retained.into()))
        };
        Ok(match replayed {
            RationalBezierIntersectionContacts2::Contacts(contacts) => match retain(contacts)? {
                Classification::Decided(contacts) if contacts.is_empty() => {
                    Classification::Decided(RationalBezierIntersectionContacts2::NoIntersection)
                }
                Classification::Decided(contacts) => {
                    Classification::Decided(RationalBezierIntersectionContacts2::Contacts(contacts))
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
            RationalBezierIntersectionContacts2::Incomplete {
                contacts,
                candidates,
            } => match retain(contacts)? {
                Classification::Decided(contacts) => {
                    Classification::Decided(RationalBezierIntersectionContacts2::Incomplete {
                        contacts,
                        candidates,
                    })
                }
                Classification::Uncertain(reason) => Classification::Uncertain(reason),
            },
            replayed => Classification::Decided(replayed),
        })
    }

    fn retained_lineage_touch_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Vec<RationalBezierIntersectionContact2>>> {
        let mut contacts = Vec::with_capacity(1);
        for (first_parameter, first_source) in [
            (Real::zero(), self.source_parameter_range().start()),
            (Real::one(), self.source_parameter_range().end()),
        ] {
            for (second_parameter, second_source) in [
                (Real::zero(), other.source_parameter_range().start()),
                (Real::one(), other.source_parameter_range().end()),
            ] {
                match compare_reals(first_source, second_source, policy) {
                    Some(Ordering::Equal) => {
                        let point = match self.point_at_classified(&first_parameter, policy) {
                            Classification::Decided(point) => point,
                            Classification::Uncertain(reason) => {
                                return Ok(Classification::Uncertain(reason));
                            }
                        };
                        contacts.push(RationalBezierIntersectionContact2 {
                            first_parameter: BezierParameter2::Exact(first_parameter.clone()),
                            second_parameter: BezierParameter2::Exact(second_parameter.clone()),
                            point: RationalBezierIntersectionPointEvidence2::Exact(point),
                            certified_transverse: false,
                            tangent_cross_sign: None,
                        });
                    }
                    Some(_) => {}
                    None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
                }
            }
        }
        Ok(Classification::Decided(contacts))
    }

    fn implicit_conic_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let conic = match self.implicit_quadratic_conic(policy) {
            Classification::Decided(Some(conic)) => conic,
            Classification::Decided(None) => return Ok(None),
            Classification::Uncertain(_) => return Ok(None),
        };
        // Exact degree elevation preserves the authored local parameter, but
        // carrying its redundant higher-degree basis into rational root-image
        // transport can force dozens of unnecessary isolator refinements.
        // Recover an exact linear homogeneous representative only for the
        // allocation-light all-rational case. Every inverse-elevation step is
        // replayed below; mixed and symbolic Real carriers retain the general
        // certified path unchanged.
        let reduced_other = other.exact_linear_homogeneous_representative(policy)?;
        let parameter_curve = reduced_other.as_ref().unwrap_or(other);
        let other_basis = parameter_curve.homogeneous_power_basis()?;
        let Some(substituted) = substitute_implicit_conic(conic, other_basis) else {
            return Ok(None);
        };
        let polynomial = match BezierParameterPolynomial::try_new_power_basis(substituted, policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
            Err(CurveError::InvalidBezierPolynomial) => return Ok(None),
            Err(error) => return Err(error),
        };
        let other_parameters = match polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        if other_parameters.is_empty() {
            return Ok(Some(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )));
        }
        let simple_roots = polynomial.simple_root_classifications(&other_parameters, policy)?;
        let parameter_map = match conic_parameter_map(self, parameter_curve, policy)? {
            Classification::Decided(parameter_map) => parameter_map,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let primary_parameter_candidate = match conic_parameter_candidate(
            polynomial.coefficients(),
            &parameter_map.primary,
            policy,
        )? {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let mut contacts = Vec::with_capacity(other_parameters.len());
        for (parameter, simple_root) in other_parameters.iter().zip(simple_roots) {
            // The quadratic frame is nonsingular and both rational
            // denominators have a certified common sign. Consequently a
            // simple root of the cleared implicit substitution has nonzero
            // directional derivative, which is exactly transversality of the
            // two regular affine images. Multiple or undecided roots retain
            // the existing tangent-based fallback.
            let certified_transverse = matches!(simple_root, Classification::Decided(true));
            let mapped = conic_parameter_from_curve_parameter(
                &parameter_map,
                &primary_parameter_candidate,
                polynomial.coefficients(),
                parameter,
                reduced_other.is_some(),
                policy,
            )?;
            match mapped {
                Classification::Decided(Some(conic_parameter)) => {
                    let point = match parameter {
                        BezierParameter2::Exact(_) => {
                            match exact_contact_point_evidence(other, parameter, policy)? {
                                Some(point) => point,
                                None => {
                                    let Some(point) = exact_contact_point_evidence(
                                        self,
                                        &conic_parameter,
                                        policy,
                                    )?
                                    else {
                                        return Ok(Some(Classification::Uncertain(
                                            UncertaintyReason::Predicate,
                                        )));
                                    };
                                    point
                                }
                            }
                        }
                        BezierParameter2::Algebraic(parameter) => {
                            RationalBezierIntersectionPointEvidence2::Algebraic(
                                RationalBezierAlgebraicPointImage2::from_parametric_source(
                                    other.clone(),
                                    parameter.clone(),
                                    policy,
                                ),
                            )
                        }
                    };
                    contacts.push(RationalBezierIntersectionContact2 {
                        first_parameter: conic_parameter,
                        second_parameter: parameter.clone(),
                        point,
                        certified_transverse,
                        tangent_cross_sign: None,
                    });
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            }
        }
        if contacts.is_empty() {
            return Ok(Some(Classification::Decided(
                RationalBezierIntersectionContacts2::NoIntersection,
            )));
        }
        Ok(Some(Classification::Decided(
            RationalBezierIntersectionContacts2::Contacts(contacts.into()),
        )))
    }

    fn exact_linear_homogeneous_representative(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Self>> {
        if self.degree() <= 1
            || self
                .weights()
                .iter()
                .any(|value| value.exact_rational_ref().is_none())
            || self.control_points().iter().any(|point| {
                point.x().exact_rational_ref().is_none() || point.y().exact_rational_ref().is_none()
            })
        {
            return Ok(None);
        }
        let frame = match exact_linear_homogeneous_reduction(self.homogeneous_controls(), policy) {
            Classification::Decided(Some(frame)) => frame,
            Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
        };
        let mut controls = Vec::with_capacity(2);
        let mut weights = Vec::with_capacity(2);
        for point in frame {
            let projected = match project_homogeneous(&point, policy) {
                Classification::Decided(projected) => projected,
                Classification::Uncertain(_) => return Ok(None),
            };
            controls.push(projected);
            weights.push(point.weight);
        }
        Ok(Self::try_new(controls, weights).ok())
    }

    fn exact_line_image_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let line = match other.fit_exact_line_image(policy)? {
            Classification::Decided(BezierLineImageFitRelation::Fit(fit)) => fit,
            Classification::Decided(BezierLineImageFitRelation::NotLine) => return Ok(None),
            Classification::Uncertain(_) => return Ok(None),
        };
        if matches!(
            self.fit_exact_line_image(policy)?,
            Classification::Decided(BezierLineImageFitRelation::Fit(_))
        ) {
            return Ok(None);
        }
        if self.degree() == 2
            && let Some(circle) = self.data.lineage.root.circular_conic.get()
        {
            let (line_dx, line_dy) = line.line().delta();
            let (from_center_x, from_center_y) = line.line().start().delta_from(&circle.center);
            let one = Real::one();
            let start_residual = Real::signed_product_sum(
                [true, true, false],
                [
                    [&from_center_x, &from_center_x],
                    [&from_center_y, &from_center_y],
                    [&circle.radius_squared, &one],
                ],
            );
            let (end_from_center_x, end_from_center_y) =
                line.line().end().delta_from(&circle.center);
            let end_residual = Real::signed_product_sum(
                [true, true, false],
                [
                    [&end_from_center_x, &end_from_center_x],
                    [&end_from_center_y, &end_from_center_y],
                    [&circle.radius_squared, &one],
                ],
            );
            let matches_curve_endpoint = |point: &Point2| {
                [self.start(), self.end()].into_iter().any(|endpoint| {
                    point == endpoint
                        || is_zero(&point.distance_squared(endpoint), policy) == Some(true)
                })
            };
            let point_on_curve = |point: &Point2| {
                matches_curve_endpoint(point)
                    || matches!(
                        self.point_incidence_classified(point, policy),
                        Ok(Classification::Decided(
                            RationalBezierPointIncidence2::Parameters(parameters)
                        )) if !parameters.is_empty()
                    )
            };
            let start_matches_curve_endpoint = point_on_curve(line.line().start());
            let end_matches_curve_endpoint = point_on_curve(line.line().end());
            let start_value =
                if start_matches_curve_endpoint || is_zero(&start_residual, policy) == Some(true) {
                    Real::zero()
                } else {
                    start_residual.clone()
                };
            let end_value =
                if end_matches_curve_endpoint || is_zero(&end_residual, policy) == Some(true) {
                    Real::zero()
                } else {
                    end_residual
                };
            for (point, line_parameter, radius_x, radius_y, residual, at_start) in [
                (
                    line.line().start(),
                    Real::zero(),
                    &from_center_x,
                    &from_center_y,
                    &start_value,
                    true,
                ),
                (
                    line.line().end(),
                    Real::one(),
                    &end_from_center_x,
                    &end_from_center_y,
                    &end_value,
                    false,
                ),
            ] {
                let radial_direction = Real::dot2_refs([radius_x, radius_y], [&line_dx, &line_dy]);
                let inward_parameter_direction = if at_start {
                    radial_direction.clone()
                } else {
                    -radial_direction.clone()
                };
                let certified_orthogonal = is_zero(&radial_direction, policy) == Some(true)
                    || (is_zero(&(radius_x - radius_y), policy) == Some(true)
                        && is_zero(&(&line_dx + &line_dy), policy) == Some(true))
                    || (is_zero(&(radius_x + radius_y), policy) == Some(true)
                        && is_zero(&(&line_dx - &line_dy), policy) == Some(true))
                    || (is_zero(radius_x, policy) == Some(true)
                        && is_zero(&line_dy, policy) == Some(true))
                    || (is_zero(radius_y, policy) == Some(true)
                        && is_zero(&line_dx, policy) == Some(true));
                let direction_moves_outside = matches!(
                    real_sign(&inward_parameter_direction, policy),
                    Some(RealSign::Positive)
                );
                let mut curve_parameters =
                    [(self.start(), Real::zero()), (self.end(), one.clone())]
                        .into_iter()
                        .filter_map(|(endpoint, parameter)| {
                            (point == endpoint
                                || is_zero(&point.distance_squared(endpoint), policy) == Some(true))
                            .then_some(BezierParameter2::Exact(parameter))
                        })
                        .collect::<Vec<_>>();
                if curve_parameters.is_empty()
                    && let Classification::Decided(RationalBezierPointIncidence2::Parameters(
                        parameters,
                    )) = self.point_incidence_classified(point, policy)?
                {
                    curve_parameters = parameters;
                }
                if curve_parameters.is_empty()
                    && let Classification::Decided(Some(parameters)) =
                        quadratic_conic_point_parameters(point, self, policy)
                {
                    for parameter in parameters {
                        let BezierParameter2::Exact(exact) = &parameter else {
                            continue;
                        };
                        if matches!(
                            self.point_at_classified(exact, policy),
                            Classification::Decided(image)
                                if is_zero(&image.distance_squared(point), policy) == Some(true)
                        ) {
                            curve_parameters.push(parameter);
                        }
                    }
                }
                let point_is_on_full_circle = is_zero(residual, policy) == Some(true);
                if (point_is_on_full_circle || !curve_parameters.is_empty())
                    && (certified_orthogonal || direction_moves_outside)
                {
                    if let [curve_parameter] = curve_parameters.as_slice() {
                        return Ok(Some(Classification::Decided(
                            RationalBezierIntersectionContacts2::Contacts(Arc::from([
                                RationalBezierIntersectionContact2 {
                                    first_parameter: curve_parameter.clone(),
                                    second_parameter: BezierParameter2::Exact(line_parameter),
                                    point: RationalBezierIntersectionPointEvidence2::Exact(
                                        point.clone(),
                                    ),
                                    certified_transverse: direction_moves_outside,
                                    tangent_cross_sign: None,
                                },
                            ])),
                        )));
                    }
                    if point_is_on_full_circle && certified_orthogonal {
                        let hull = Aabb2::from_points(self.control_points(), policy);
                        if matches!(
                            hull,
                            Classification::Decided(bounds)
                                if matches!(
                                    bounds.contains_point(point, policy),
                                    Classification::Decided(false)
                                )
                        ) {
                            return Ok(Some(Classification::Decided(
                                RationalBezierIntersectionContacts2::NoIntersection,
                            )));
                        }
                    }
                }
            }
            let half_linear =
                Real::dot2_refs([&from_center_x, &from_center_y], [&line_dx, &line_dy]);
            let quadratic = Real::dot2_refs([&line_dx, &line_dy], [&line_dx, &line_dy]);
            let roots = polynomial_roots_in_unit_interval_with_endpoints(
                start_value.clone(),
                Real::from(2_i8) * half_linear,
                quadratic,
                &start_value,
                &end_value,
                policy,
            );
            if let Classification::Decided(roots) = roots {
                let mut replayed = Vec::with_capacity(roots.len());
                for line_parameter in roots {
                    let point = Point2::new(
                        line.line().start().x() + &line_dx * &line_parameter,
                        line.line().start().y() + &line_dy * &line_parameter,
                    );
                    let endpoint_parameters =
                        [(self.start(), Real::zero()), (self.end(), one.clone())]
                            .into_iter()
                            .filter_map(|(endpoint, parameter)| {
                                (point == *endpoint
                                    || is_zero(&point.distance_squared(endpoint), policy)
                                        == Some(true))
                                .then_some(BezierParameter2::Exact(parameter))
                            })
                            .collect::<Vec<_>>();
                    let curve_parameters = if endpoint_parameters.is_empty() {
                        match quadratic_conic_point_parameters(&point, self, policy) {
                            Classification::Decided(Some(parameters)) => parameters,
                            Classification::Decided(None) => continue,
                            Classification::Uncertain(reason) => {
                                return Ok(Some(Classification::Uncertain(reason)));
                            }
                        }
                    } else {
                        endpoint_parameters
                    };
                    let (radius_x, radius_y) = point.delta_from(&circle.center);
                    let certified_transverse = match is_zero(
                        &Real::dot2_refs([&radius_x, &radius_y], [&line_dx, &line_dy]),
                        policy,
                    ) {
                        Some(value) => !value,
                        None => {
                            return Ok(Some(Classification::Uncertain(
                                UncertaintyReason::RealSign,
                            )));
                        }
                    };
                    for curve_parameter in curve_parameters {
                        replayed.push(RationalBezierIntersectionContact2 {
                            first_parameter: curve_parameter,
                            second_parameter: BezierParameter2::Exact(line_parameter.clone()),
                            point: RationalBezierIntersectionPointEvidence2::Exact(point.clone()),
                            certified_transverse,
                            tangent_cross_sign: None,
                        });
                    }
                }
                return Ok(Some(Classification::Decided(if replayed.is_empty() {
                    RationalBezierIntersectionContacts2::NoIntersection
                } else {
                    RationalBezierIntersectionContacts2::Contacts(replayed.into())
                })));
            }
        }
        let supporting_contacts = match self.relation_to_line_with_contacts(line.line(), policy) {
            Classification::Decided(relation) => relation,
            Classification::Uncertain(reason) => {
                return Ok(Some(Classification::Uncertain(reason)));
            }
        };
        let contacts = match supporting_contacts {
            BezierLineContactRelation::ControlHullDisjoint { .. }
            | BezierLineContactRelation::NoContact => Vec::new(),
            BezierLineContactRelation::OnSupportingLine => return Ok(None),
            BezierLineContactRelation::Contacts { contacts } => contacts,
        };
        let parameter_graph = [Axis2::X, Axis2::Y].into_iter().find_map(|axis| {
            match other.polynomial_graph(axis, policy).ok()? {
                Classification::Decided(Some(graph)) => Some(graph),
                Classification::Decided(None) | Classification::Uncertain(_) => None,
            }
        });
        let Some(parameter_graph) = parameter_graph else {
            return Ok(None);
        };
        let basis = self.homogeneous_power_basis()?;
        let axis_numerator = match parameter_graph.axis {
            Axis2::X => &basis.x_numerator,
            Axis2::Y => &basis.y_numerator,
        };
        let parameter_numerator = subtract_power_polynomials(
            axis_numerator,
            &scale_power_polynomial(&basis.weight, &parameter_graph.origin),
        );
        let parameter_denominator = scale_power_polynomial(&basis.weight, &parameter_graph.scale);
        let mut replayed = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let source_parameter = contact.parameter().clone();
            let parameter = match source_parameter
                .clone()
                .promote_represented_rational_root(policy)?
            {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(_) => source_parameter,
            };
            let Some(parameter_value) = parameter.as_exact() else {
                let root = parameter_root_representation(&parameter, policy);
                let candidate = match conic_parameter_candidate(
                    &root.polynomial_coefficients,
                    &(parameter_numerator.clone(), parameter_denominator.clone()),
                    policy,
                )? {
                    Classification::Decided(candidate) => candidate,
                    Classification::Uncertain(_) => return Ok(None),
                };
                let mapped = match rational_image_parameter(&root, &candidate, policy)? {
                    Classification::Decided(mapped) => Classification::Decided(mapped),
                    Classification::Uncertain(_) => {
                        real_coefficient_rational_image_parameter(&parameter, &candidate, policy)?
                    }
                };
                let Classification::Decided(mapped) = mapped else {
                    return Ok(None);
                };
                let Some(mapped) = mapped else {
                    continue;
                };
                let point = match exact_contact_point_evidence(other, &mapped, policy)? {
                    Some(point) => Some(point),
                    None => exact_contact_point_evidence(self, &parameter, policy)?,
                };
                let Some(point) = point else {
                    return Ok(None);
                };
                replayed.push(RationalBezierIntersectionContact2 {
                    first_parameter: parameter,
                    second_parameter: mapped,
                    point,
                    certified_transverse: contact.kind() == BezierLineContactKind::Crossing,
                    tangent_cross_sign: None,
                });
                continue;
            };
            let point = match self.point_at_classified(parameter_value, policy) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    return Ok(Some(Classification::Uncertain(reason)));
                }
            };
            let other_parameter = if other.exact_linear_parameterization_line().is_some() {
                let coordinate = match parameter_graph.axis {
                    Axis2::X => point.x(),
                    Axis2::Y => point.y(),
                };
                let mapped = ((coordinate - &parameter_graph.origin) / &parameter_graph.scale)?;
                match in_closed_unit_interval(&mapped, policy) {
                    Some(true) => BezierParameter2::Exact(mapped),
                    Some(false) => continue,
                    None => {
                        return Ok(Some(Classification::Uncertain(UncertaintyReason::Ordering)));
                    }
                }
            } else {
                match unique_point_incidence_parameter(other, &point, policy) {
                    Classification::Decided(Some(parameter)) => parameter,
                    Classification::Decided(None) => {
                        return Ok(Some(Classification::Uncertain(
                            UncertaintyReason::Predicate,
                        )));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Some(Classification::Uncertain(reason)));
                    }
                }
            };
            replayed.push(RationalBezierIntersectionContact2 {
                first_parameter: parameter,
                second_parameter: other_parameter,
                point: RationalBezierIntersectionPointEvidence2::Exact(point),
                certified_transverse: contact.kind() == BezierLineContactKind::Crossing,
                tangent_cross_sign: None,
            });
        }
        Ok(Some(Classification::Decided(if replayed.is_empty() {
            RationalBezierIntersectionContacts2::NoIntersection
        } else {
            RationalBezierIntersectionContacts2::Contacts(replayed.into())
        })))
    }

    fn circular_conic_intersection_contacts(
        &self,
        other: &Self,
        policy: &CurveContext,
        circle_relation: Option<&CircleCircleRelation>,
        first_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
        second_circle_parameters: Option<&[Classification<Arc<[BezierParameter2]>>]>,
    ) -> CurveResult<Option<Classification<RationalBezierIntersectionContacts2>>> {
        let (Some(first), Some(second)) = (
            self.data.lineage.root.circular_conic.get(),
            other.data.lineage.root.circular_conic.get(),
        ) else {
            return Ok(None);
        };
        let computed_relation;
        let circle_relation = match circle_relation {
            Some(relation) => relation,
            None => {
                computed_relation = circle_relation_from_supports(
                    &first.center,
                    &first.radius_squared,
                    &second.center,
                    &second.radius_squared,
                    policy,
                )?;
                &computed_relation
            }
        };
        let (points, certified_transverse) = match circle_relation {
            CircleCircleRelation::Coincident => return Ok(None),
            CircleCircleRelation::Disjoint => {
                return Ok(Some(Classification::Decided(
                    RationalBezierIntersectionContacts2::NoIntersection,
                )));
            }
            CircleCircleRelation::Tangent { point } => (vec![point.clone()], false),
            CircleCircleRelation::Secant {
                first_point,
                second_point,
            } => (vec![first_point.clone(), second_point.clone()], true),
            CircleCircleRelation::Uncertain { reason } => {
                return Ok(Some(Classification::Uncertain(*reason)));
            }
        };
        let mut contacts = Vec::with_capacity(points.len());
        for (point_index, point) in points.into_iter().enumerate() {
            let first_parameters =
                match first_circle_parameters.and_then(|parameters| parameters.get(point_index)) {
                    Some(Classification::Decided(parameters)) => Arc::clone(parameters),
                    Some(Classification::Uncertain(reason)) => {
                        return Ok(Some(Classification::Uncertain(*reason)));
                    }
                    None => match self.retained_circle_point_parameters(&point, policy)? {
                        Classification::Decided(parameters) => Arc::from(parameters),
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    },
                };
            if first_parameters.is_empty() {
                continue;
            }
            let second_parameters =
                match second_circle_parameters.and_then(|parameters| parameters.get(point_index)) {
                    Some(Classification::Decided(parameters)) => Arc::clone(parameters),
                    Some(Classification::Uncertain(reason)) => {
                        return Ok(Some(Classification::Uncertain(*reason)));
                    }
                    None => match other.retained_circle_point_parameters(&point, policy)? {
                        Classification::Decided(parameters) => Arc::from(parameters),
                        Classification::Uncertain(reason) => {
                            return Ok(Some(Classification::Uncertain(reason)));
                        }
                    },
                };
            if second_parameters.is_empty() {
                continue;
            }
            for first_parameter in first_parameters.iter() {
                for second_parameter in second_parameters.iter() {
                    contacts.push(RationalBezierIntersectionContact2 {
                        first_parameter: first_parameter.clone(),
                        second_parameter: second_parameter.clone(),
                        point: RationalBezierIntersectionPointEvidence2::Exact(point.clone()),
                        certified_transverse,
                        tangent_cross_sign: None,
                    });
                }
            }
        }
        Ok(Some(Classification::Decided(if contacts.is_empty() {
            RationalBezierIntersectionContacts2::NoIntersection
        } else {
            RationalBezierIntersectionContacts2::Contacts(contacts.into())
        })))
    }

    fn replay_intersection_candidate_set(
        &self,
        other: &Self,
        candidates: &RationalBezierIntersectionCandidates2,
        policy: &CurveContext,
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
                                certified_transverse: false,
                                tangent_cross_sign: None,
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

    pub(crate) fn intersection_candidates_classified(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionCandidates2>> {
        // Same-sign control-hull bounds are only a rejection accelerator. An
        // unavailable sign or ordering certificate must fall through to the
        // homogeneous resultant, whose affine replay independently rejects
        // projective poles and out-of-domain roots.
        if self.certified_bounds_are_disjoint(other, policy) {
            return Ok(Classification::Decided(
                RationalBezierIntersectionCandidates2::NoIntersection,
            ));
        }

        self.intersection_candidates_after_bounds_check(other, policy)
    }

    fn certified_bounds_are_disjoint(&self, other: &Self, policy: &CurveContext) -> bool {
        let (Classification::Decided(first_bounds), Classification::Decided(second_bounds)) = (
            self.certified_bounds_classified(policy),
            other.certified_bounds_classified(policy),
        ) else {
            return false;
        };
        matches!(
            first_bounds.overlaps(&second_bounds, policy),
            Classification::Decided(false)
        )
    }

    fn intersection_candidates_after_bounds_check(
        &self,
        other: &Self,
        policy: &CurveContext,
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
        for reversed in [false, true] {
            if self.same_projective_control_net(other, reversed, policy) == Some(true) {
                return Ok(Classification::Decided(
                    RationalBezierIntersectionCandidates2::DegenerateResultant,
                ));
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
                ResultantParameterProjection::Parameters(first_parameters)
                | ResultantParameterProjection::SelectedParameters(first_parameters),
                ResultantParameterProjection::Parameters(second_parameters)
                | ResultantParameterProjection::SelectedParameters(second_parameters),
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
        policy: &CurveContext,
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
        let one_minus_parameter = Real::one() - parameter;
        let is_midpoint = compare_reals(parameter, &one_minus_parameter, policy)
            == Some(std::cmp::Ordering::Equal);
        left.push(level[0].clone());
        right.push(
            level
                .last()
                .expect("validated rational Bezier has controls")
                .clone(),
        );
        for next_len in (1..level.len()).rev() {
            for index in 0..next_len {
                let interpolated = if is_midpoint {
                    level[index].midpoint(&level[index + 1], parameter)
                } else {
                    level[index].lerp_with_complement(
                        &level[index + 1],
                        parameter,
                        &one_minus_parameter,
                    )
                };
                level[index] = interpolated;
            }
            left.push(level[0].clone());
            right.push(level[next_len - 1].clone());
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
        policy: &CurveContext,
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

    /// Materializes the exact rational image over any ordered finite affine
    /// parameter range whose denominator has no zero.
    ///
    /// Exterior corner reconstruction uses this after the incident-ray solver
    /// has selected one pole-partitioned component. Reparameterization creates
    /// a fresh unit-domain lineage: an injectivity fact proved only on the
    /// authored source interval must not leak onto its projective extension.
    /// A finite rational image can nevertheless acquire zero or mixed-sign
    /// intermediate Bernstein weights under extrapolation. Exact homogeneous
    /// degree elevation is repeated until every weight has the common sign
    /// guaranteed by the pole-free denominator, avoiding a second carrier for
    /// controls at infinity.
    pub(crate) fn subcurve_between_affine_exact(
        &self,
        start: &Real,
        end: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        match compare_reals(start, end, policy) {
            Some(Ordering::Greater) => return Err(CurveError::InvalidBezierRange),
            Some(_) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        }

        let span = end - start;
        let transformed_weight = match compose_univariate_polynomial_linear_fractional(
            &self.homogeneous_power_basis()?.weight,
            &span,
            start,
            &Real::zero(),
            &Real::one(),
            policy.predicate_policy(),
        ) {
            Some(coefficients) => coefficients,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let transformed_weight =
            match BezierParameterPolynomial::try_new_power_basis(transformed_weight, policy)? {
                Classification::Decided(polynomial) => polynomial,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
        match transformed_weight.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) if roots.is_empty() => {}
            Classification::Decided(_) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
            }
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }

        let start_at_one = match compare_reals(start, &Real::one(), policy) {
            Some(Ordering::Equal) => true,
            Some(_) => false,
            None => return Ok(Classification::Uncertain(UncertaintyReason::Ordering)),
        };
        let mut controls = affine_homogeneous_subcurve_controls(
            self.homogeneous_controls(),
            start,
            end,
            start_at_one,
            policy,
        )?;
        loop {
            match homogeneous_controls_common_weight_sign(&controls, policy) {
                Classification::Decided(Some(_)) => break,
                Classification::Decided(None) => {
                    controls = elevate_homogeneous_controls_once(&controls)?;
                }
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }

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

        let root = Arc::new(RationalBezierLineageRoot::default());
        if let Some(implicit) = self.data.lineage.root.implicit_quadratic_conic.get() {
            let _ = root.implicit_quadratic_conic.set(Arc::clone(implicit));
        }
        if let Some(circle) = self.data.lineage.root.circular_conic.get() {
            let _ = root.circular_conic.set(Arc::clone(circle));
        }
        let exact_line_image = self.data.exact_line_image.as_ref().and_then(|_| {
            LineSeg2::try_new(
                points
                    .first()
                    .expect("positive-degree curve has a start")
                    .clone(),
                points
                    .last()
                    .expect("positive-degree curve has an end")
                    .clone(),
            )
            .ok()
        });
        Self::try_new_with_lineage_and_exact_line_image(
            points,
            weights,
            RationalBezierLineage {
                root,
                range: ParamRange::new(Real::zero(), Real::one()),
            },
            exact_line_image,
        )
        .map(Classification::Decided)
    }

    pub(crate) fn endpoint_derivatives(
        &self,
        at_end: bool,
        max_order: usize,
        policy: &CurveContext,
    ) -> Classification<Vec<(Real, Real)>> {
        let parameter = if at_end { Real::one() } else { Real::zero() };
        self.affine_derivative_values_at_with_endpoint(&parameter, max_order, Some(at_end), policy)
    }

    fn affine_derivative_values_at(
        &self,
        parameter: &Real,
        max_order: usize,
        policy: &CurveContext,
    ) -> Classification<Vec<(Real, Real)>> {
        self.affine_derivative_values_at_with_endpoint(parameter, max_order, None, policy)
    }

    fn affine_derivative_values_at_with_endpoint(
        &self,
        parameter: &Real,
        max_order: usize,
        endpoint: Option<bool>,
        policy: &CurveContext,
    ) -> Classification<Vec<(Real, Real)>> {
        if in_closed_unit_interval(parameter, policy) != Some(true) {
            return Classification::Uncertain(UncertaintyReason::Ordering);
        }
        let Ok(power_basis) = self.homogeneous_power_basis() else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let evaluate = |coefficients: &[Real]| match endpoint {
            Some(at_end) => {
                evaluate_power_polynomial_endpoint_derivatives(coefficients, at_end, max_order)
            }
            None => evaluate_power_polynomial_derivatives(coefficients, parameter, max_order),
        };
        let Some(numerator_x) = evaluate(&power_basis.x_numerator) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some(numerator_y) = evaluate(&power_basis.y_numerator) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let Some(denominator) = evaluate(&power_basis.weight) else {
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

    pub(crate) fn homogeneous_power_basis(&self) -> CurveResult<&RationalParametricCurve2> {
        if let Some(power_basis) = self.data.homogeneous_power_basis.get() {
            return Ok(power_basis);
        }
        let x = bernstein_to_power_coefficients(
            self.control_points()
                .iter()
                .zip(self.weights())
                .map(|(point, weight)| point.x() * weight)
                .collect(),
        )?;
        let y = bernstein_to_power_coefficients(
            self.control_points()
                .iter()
                .zip(self.weights())
                .map(|(point, weight)| point.y() * weight)
                .collect(),
        )?;
        let weight = bernstein_to_power_coefficients(self.weights().to_vec())?;
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
                let scale = exact_binomial_product(
                    derivative_degree,
                    derivative_index,
                    degree,
                    coordinate_index,
                )?;
                let product_difference = derivative_coordinate * &self.weights()[coordinate_index]
                    - &weighted_coordinates[coordinate_index]
                        * &weight_derivative[derivative_index];
                coefficient += scale * product_difference;
            }
            let basis_scale = exact_binomial(product_degree, product_index)?;
            coefficients.push((coefficient / basis_scale).ok()?);
        }
        Some(coefficients)
    }

    fn point_axis_polynomial(
        &self,
        target: &Real,
        axis: Axis2,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        self.replay_intersection_candidates_with_pair_filter(
            other,
            first_parameters,
            second_parameters,
            false,
            None,
            None,
            policy,
        )
    }

    fn replay_intersection_candidates_with_pair_filter(
        &self,
        other: &Self,
        first_parameters: &[BezierParameter2],
        second_parameters: &[BezierParameter2],
        unordered_self_pairs: bool,
        pair_equations: Option<&[BivariatePolynomial; 2]>,
        mut certified_pair_point_evidence: Option<
            &mut dyn FnMut(
                &BezierParameter2,
            )
                -> CurveResult<Option<RationalBezierIntersectionPointEvidence2>>,
        >,
        policy: &CurveContext,
    ) -> CurveResult<Classification<RationalBezierIntersectionContacts2>> {
        if !unordered_self_pairs
            && let Some(contacts) =
                self.replay_candidates_through_polynomial_graph(other, first_parameters, policy)?
        {
            return Ok(Classification::Decided(if contacts.is_empty() {
                RationalBezierIntersectionContacts2::NoIntersection
            } else {
                RationalBezierIntersectionContacts2::Contacts(contacts.into())
            }));
        }
        // Candidate image intervals are useful observations, but they are not
        // sufficient pair-inequality certificates. In particular, separately
        // evaluated resultant roots once produced disjoint enclosures for a
        // real rational/cubic contact. Replay every candidate pair through the
        // exact algebraic coordinate comparison instead.
        let mut first_replays = (0..first_parameters.len())
            .map(|_| None)
            .collect::<Vec<Option<Option<CandidatePointReplay>>>>();
        let mut second_replays = (0..second_parameters.len())
            .map(|_| None)
            .collect::<Vec<Option<Option<CandidatePointReplay>>>>();
        let first_simple_roots = first_parameters
            .iter()
            .map(|parameter| candidate_parameter_is_simple_root(parameter, policy))
            .collect::<CurveResult<Vec<_>>>()?;
        let second_simple_roots = second_parameters
            .iter()
            .map(|parameter| candidate_parameter_is_simple_root(parameter, policy))
            .collect::<CurveResult<Vec<_>>>()?;
        let mut pair_replay_cache =
            crate::bezier_offset::BivariateParameterPairReplayCache::default();
        let mut incomplete = false;
        let mut contacts = Vec::new();
        for first_index in 0..first_parameters.len() {
            'second_parameter: for second_index in 0..second_parameters.len() {
                if unordered_self_pairs {
                    match first_parameters[first_index]
                        .cmp_by_refinement(&second_parameters[second_index], policy)?
                    {
                        Classification::Decided(Ordering::Less) => {}
                        Classification::Decided(Ordering::Equal | Ordering::Greater) => continue,
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                let projected_pair = if let Some(equations) = pair_equations {
                    let replay = crate::bezier_offset::replay_projected_bivariate_parameter_pair(
                        equations,
                        &first_parameters[first_index],
                        &second_parameters[second_index],
                        policy,
                        CurveIntersectionResultantConfig {
                            min_precision: RATIONAL_INTERSECTION_RESULTANT_PRECISION,
                            max_resultant_degree: MAX_RATIONAL_INTERSECTION_RESULTANT_DEGREE,
                        },
                        &mut pair_replay_cache,
                    )?;
                    match replay {
                        Classification::Decided(false) => continue 'second_parameter,
                        Classification::Decided(true) => Some(true),
                        Classification::Uncertain(_) => None,
                    }
                } else {
                    None
                };
                if projected_pair == Some(true) {
                    let point = match exact_contact_point_evidence(
                        self,
                        &first_parameters[first_index],
                        policy,
                    )? {
                        Some(point) => Some(point),
                        None => match certified_pair_point_evidence.as_deref_mut() {
                            Some(fallback) => fallback(&first_parameters[first_index])?,
                            None => None,
                        },
                    };
                    let Some(point) = point else {
                        incomplete = true;
                        continue;
                    };
                    contacts.push(RationalBezierIntersectionContact2 {
                        first_parameter: first_parameters[first_index].clone(),
                        second_parameter: second_parameters[second_index].clone(),
                        point,
                        certified_transverse: first_simple_roots[first_index]
                            && second_simple_roots[second_index],
                        tangent_cross_sign: None,
                    });
                    continue;
                }
                if first_replays[first_index].is_none() {
                    first_replays[first_index] =
                        Some(self.candidate_point_replay(&first_parameters[first_index], policy)?);
                }
                if second_replays[second_index].is_none() {
                    second_replays[second_index] = Some(
                        other.candidate_point_replay(&second_parameters[second_index], policy)?,
                    );
                }
                let (Some(first_replay), Some(second_replay)) = (
                    first_replays[first_index].as_ref().and_then(Option::as_ref),
                    second_replays[second_index]
                        .as_ref()
                        .and_then(Option::as_ref),
                ) else {
                    incomplete = true;
                    continue;
                };
                match candidate_points_equal(first_replay, second_replay, policy) {
                    Some(true) => contacts.push(RationalBezierIntersectionContact2 {
                        first_parameter: first_parameters[first_index].clone(),
                        second_parameter: second_parameters[second_index].clone(),
                        point: first_replay.evidence.clone(),
                        // A first-order root in both parameter projections
                        // excludes tangency and singular projection at this
                        // matched isolated contact.
                        certified_transverse: first_simple_roots[first_index]
                            && second_simple_roots[second_index],
                        tangent_cross_sign: None,
                    }),
                    Some(false) => {}
                    None => {
                        // Coordinate-image representations carry validated
                        // isolating intervals for the actual replayed image
                        // roots. Unlike the former independently evaluated
                        // parameter bounds, disjoint represented-root
                        // intervals are a sound certificate that this
                        // Cartesian candidate pair is not one contact.
                        if candidate_point_representations_disjoint(
                            first_replay,
                            second_replay,
                            policy,
                        ) {
                            continue;
                        }
                        match self.parameter_pair_same_point_by_incidence(
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
                                    certified_transverse: first_simple_roots[first_index]
                                        && second_simple_roots[second_index],
                                    tangent_cross_sign: None,
                                });
                            }
                            Classification::Decided(false) => {}
                            Classification::Uncertain(_) => incomplete = true,
                        }
                    }
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

    fn replay_candidates_through_polynomial_graph(
        &self,
        other: &Self,
        first_parameters: &[BezierParameter2],
        policy: &CurveContext,
    ) -> CurveResult<Option<Vec<RationalBezierIntersectionContact2>>> {
        if first_parameters.is_empty() {
            return Ok(None);
        }
        if !matches!(self.common_weight_sign(policy), Classification::Decided(_)) {
            return Ok(None);
        }
        let graph = [Axis2::X, Axis2::Y].into_iter().find_map(|axis| {
            match other.polynomial_graph(axis, policy).ok()? {
                Classification::Decided(Some(graph)) => Some(graph),
                Classification::Decided(None) | Classification::Uncertain(_) => None,
            }
        });
        let Some(graph) = graph else {
            return Ok(None);
        };
        // The graph coordinate is affine and injective in `other`'s complex
        // parameter. Every finite resultant root of `self` therefore has at
        // most one matching parameter on `other`, obtained by this exact
        // rational image. A real mapped value in `[0, 1]` is the resultant's
        // finite contact; an out-of-range value is a certified non-contact.
        let basis = self.homogeneous_power_basis()?;
        let coordinate = match graph.axis {
            Axis2::X => &basis.x_numerator,
            Axis2::Y => &basis.y_numerator,
        };
        let numerator = subtract_power_polynomials(
            coordinate,
            &scale_power_polynomial(&basis.weight, &graph.origin),
        );
        let denominator = scale_power_polynomial(&basis.weight, &graph.scale);
        let mut contacts = Vec::with_capacity(first_parameters.len());
        for first_parameter in first_parameters {
            let root = parameter_root_representation(first_parameter, policy);
            let candidate = match conic_parameter_candidate(
                &root.polynomial_coefficients,
                &(numerator.clone(), denominator.clone()),
                policy,
            )? {
                Classification::Decided(candidate) => candidate,
                Classification::Uncertain(_) => return Ok(None),
            };
            let mapped = match real_coefficient_rational_image_parameter(
                first_parameter,
                &candidate,
                policy,
            )? {
                Classification::Decided(Some(mapped)) => mapped,
                Classification::Decided(None) => continue,
                Classification::Uncertain(_) => {
                    match rational_image_parameter(&root, &candidate, policy)? {
                        Classification::Decided(Some(mapped)) => mapped,
                        Classification::Decided(None) => continue,
                        Classification::Uncertain(_) => return Ok(None),
                    }
                }
            };
            let Some(point) = exact_contact_point_evidence(other, &mapped, policy)? else {
                return Ok(None);
            };
            contacts.push(RationalBezierIntersectionContact2 {
                first_parameter: first_parameter.clone(),
                second_parameter: mapped,
                point,
                certified_transverse: false,
                tangent_cross_sign: None,
            });
        }
        Ok(Some(contacts))
    }

    fn candidate_point_replay(
        &self,
        parameter: &BezierParameter2,
        policy: &CurveContext,
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
                let source = BezierParameter2::Algebraic(parameter.clone());
                let mut refinement = BezierParameterRefinement2::new(&source, policy);
                for refinement_steps in [16, 32, 64, 128] {
                    let refined = refinement.refine_to(refinement_steps);
                    let BezierParameter2::Algebraic(refined) = refined else {
                        return self.candidate_point_replay(refined, policy);
                    };
                    let image = self.point_at_algebraic_parameter(refined, policy)?;
                    let (Some(x), Some(y)) = (
                        image.x().and_then(|coordinate| coordinate.representation()),
                        image.y().and_then(|coordinate| coordinate.representation()),
                    ) else {
                        continue;
                    };
                    return Ok(Some(CandidatePointReplay {
                        x: x.clone(),
                        y: y.clone(),
                        evidence: RationalBezierIntersectionPointEvidence2::Algebraic(image),
                    }));
                }
                Ok(None)
            }
        }
    }

    fn parameter_pair_same_point_by_incidence(
        &self,
        other: &Self,
        first: &BezierParameter2,
        second: &BezierParameter2,
        policy: &CurveContext,
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

    pub(crate) fn common_weight_sign(&self, policy: &CurveContext) -> Classification<RealSign> {
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
        policy: &CurveContext,
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

    fn endpoint_parameter_relation(
        &self,
        other: &Self,
        reversed: bool,
        policy: &CurveContext,
    ) -> Classification<Option<RationalBezierEndpointParameterRelation2>> {
        if self.degree() != other.degree() {
            return Classification::Decided(None);
        }
        let degree = self.degree();
        let other_base = if reversed { degree } else { 0 };
        for index in 0..=degree {
            let other_index = if reversed { degree - index } else { index };
            match is_zero(
                &self.control_points()[index]
                    .distance_squared(&other.control_points()[other_index]),
                policy,
            ) {
                Some(true) => {}
                Some(false) => return Classification::Decided(None),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        let mut affine_unresolved = false;
        let mut affine = true;
        for index in 0..=degree {
            let other_index = if reversed { degree - index } else { index };
            let difference = &self.weights()[index] * &other.weights()[other_base]
                - &other.weights()[other_index] * &self.weights()[0];
            match is_zero(&difference, policy) {
                Some(true) => {}
                Some(false) => {
                    affine = false;
                    affine_unresolved = false;
                    break;
                }
                None => affine_unresolved = true,
            }
        }
        if affine && !affine_unresolved {
            return Classification::Decided(Some(RationalBezierEndpointParameterRelation2::Affine));
        }
        if !matches!(self.common_weight_sign(policy), Classification::Decided(_))
            || !matches!(other.common_weight_sign(policy), Classification::Decided(_))
        {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        }
        let other_first = if reversed { degree - 1 } else { 1 };
        let scale_numerator = &other.weights()[other_first] * &self.weights()[0];
        let scale_denominator = &self.weights()[1] * &other.weights()[other_base];
        let Ok(scale) = scale_numerator / scale_denominator else {
            return Classification::Uncertain(UncertaintyReason::Boundary);
        };
        match real_sign(&scale, policy) {
            Some(RealSign::Positive) => {}
            Some(_) if affine_unresolved => {
                return Classification::Uncertain(UncertaintyReason::RealSign);
            }
            Some(_) => return Classification::Decided(None),
            None => return Classification::Uncertain(UncertaintyReason::RealSign),
        }
        let mut scale_power = Real::one();
        for index in 0..=degree {
            let other_index = if reversed { degree - index } else { index };
            let difference = &other.weights()[other_index] * &self.weights()[0]
                - &scale_power * &self.weights()[index] * &other.weights()[other_base];
            match is_zero(&difference, policy) {
                Some(true) => {}
                Some(false) if affine_unresolved => {
                    return Classification::Uncertain(UncertaintyReason::RealSign);
                }
                Some(false) => return Classification::Decided(None),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
            scale_power *= &scale;
        }
        Classification::Decided(Some(RationalBezierEndpointParameterRelation2::Projective(
            scale,
        )))
    }

    pub(crate) fn same_projective_control_net_degree_aligned(
        &self,
        other: &Self,
        reversed: bool,
        policy: &CurveContext,
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
        policy: &CurveContext,
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
        if self.degree() == other.degree() {
            for reversed in [false, true] {
                match self.endpoint_parameter_relation(other, reversed, policy) {
                    Classification::Decided(Some(_)) => {
                        return complete_rational_bezier_image_overlap(reversed);
                    }
                    Classification::Decided(None) => {}
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
        } else {
            for reversed in [false, true] {
                match self.same_projective_control_net_degree_aligned(other, reversed, policy) {
                    Classification::Decided(true) => {
                        return complete_rational_bezier_image_overlap(reversed);
                    }
                    Classification::Decided(false) => {}
                    Classification::Uncertain(reason) => {
                        return Classification::Uncertain(reason);
                    }
                }
            }
        }
        self.partial_image_overlap(other, policy)
    }

    fn lineage_overlap(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Classification<Option<RationalBezierIntersectionOverlap2>> {
        if !Arc::ptr_eq(&self.data.lineage.root, &other.data.lineage.root) {
            return Classification::Decided(None);
        }
        self.retain_root_image_injectivity(policy);
        other.retain_root_image_injectivity(policy);
        if self.data.lineage.root.image_is_injective.get() != Some(&true) {
            return Classification::Decided(None);
        }

        self.retained_source_parameter_overlap(other, policy)
    }

    fn retained_source_parameter_overlap(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> Classification<Option<RationalBezierIntersectionOverlap2>> {
        if !Arc::ptr_eq(&self.data.lineage.root, &other.data.lineage.root) {
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
                    endpoint_inclusion: [true, true],
                })
            })
    }

    fn retain_root_image_injectivity(&self, policy: &CurveContext) {
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
        policy: &CurveContext,
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
                    Classification::Decided(None) => {}
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
                    Classification::Decided(None) => {}
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
        policy: &CurveContext,
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
            endpoint_inclusion: [true, true],
        }))
    }

    pub(crate) fn has_certified_injective_axis(&self, policy: &CurveContext) -> bool {
        if self.data.lineage.root.image_is_injective.get() == Some(&true) {
            return true;
        }
        let injective = [Axis2::X, Axis2::Y]
            .into_iter()
            .any(|axis| self.has_certified_injective_axis_on(axis, policy));
        if injective {
            let _ = self.data.lineage.root.image_is_injective.set(true);
        }
        injective
    }

    pub(crate) fn derivative_is_certified_nonzero_at(
        &self,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if in_closed_unit_interval(parameter, policy) != Some(true) {
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        if let Classification::Uncertain(reason) = self.common_weight_sign(policy) {
            return Ok(Classification::Uncertain(reason));
        }
        let mut uncertainty = None;
        for axis in [Axis2::X, Axis2::Y] {
            match self.axis_derivative_is_certified_nonzero_at(axis, parameter, policy)? {
                Classification::Decided(true) => return Ok(Classification::Decided(true)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    uncertainty.get_or_insert(reason);
                }
            }
        }
        Ok(uncertainty.map_or(Classification::Decided(false), Classification::Uncertain))
    }

    fn axis_derivative_is_certified_nonzero_at(
        &self,
        axis: Axis2,
        parameter: &Real,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        let Some(coefficients) = self.axis_derivative_numerator_bernstein(axis) else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let mut positive = false;
        let mut negative = false;
        let mut coefficients_decided = true;
        for coefficient in coefficients {
            match real_sign(coefficient, policy) {
                Some(RealSign::Positive) => positive = true,
                Some(RealSign::Negative) => negative = true,
                Some(RealSign::Zero) => {}
                None => coefficients_decided = false,
            }
        }
        if coefficients_decided && positive != negative {
            let at_start = compare_reals(parameter, &Real::zero(), policy);
            let at_end = compare_reals(parameter, &Real::one(), policy);
            if at_start == Some(Ordering::Equal) {
                return Ok(Classification::Decided(
                    real_sign(&coefficients[0], policy) != Some(RealSign::Zero),
                ));
            }
            if at_end == Some(Ordering::Equal) {
                return Ok(Classification::Decided(
                    real_sign(
                        coefficients
                            .last()
                            .expect("a rational derivative has Bernstein coefficients"),
                        policy,
                    ) != Some(RealSign::Zero),
                ));
            }
            if matches!(at_start, Some(Ordering::Greater)) && matches!(at_end, Some(Ordering::Less))
            {
                return Ok(Classification::Decided(true));
            }
            return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
        }
        if coefficients_decided && !positive && !negative {
            return Ok(Classification::Decided(false));
        }
        let polynomial = match BezierParameterPolynomial::try_new_bernstein_basis(
            coefficients.to_vec(),
            policy,
        )? {
            Classification::Decided(polynomial) => polynomial,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let roots = match polynomial.isolate_unit_interval_roots(policy)? {
            Classification::Decided(roots) => roots,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let parameter = BezierParameter2::Exact(parameter.clone());
        for root in roots {
            match parameter.same_value(&root, policy)? {
                Classification::Decided(true) => return Ok(Classification::Decided(false)),
                Classification::Decided(false) => {}
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        Ok(Classification::Decided(true))
    }

    pub(crate) fn has_certified_injective_axis_on(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> bool {
        let (start, end) = match axis {
            Axis2::X => (self.start().x(), self.end().x()),
            Axis2::Y => (self.start().y(), self.end().y()),
        };
        if self.control_polygon_certifies_axis_monotone(axis, policy)
            && compare_reals(start, end, policy).is_some_and(|ordering| !ordering.is_eq())
        {
            return true;
        }
        if !matches!(
            self.axis_monotonicity_classified(axis, policy),
            Ok(Classification::Decided(true))
        ) {
            return false;
        }
        // A one-signed Bernstein derivative with distinct endpoint coordinates
        // is strictly monotone on the open domain.
        compare_reals(start, end, policy).is_some_and(|ordering| !ordering.is_eq())
    }

    fn control_polygon_certifies_axis_monotone(&self, axis: Axis2, policy: &CurveContext) -> bool {
        if !matches!(self.common_weight_sign(policy), Classification::Decided(_)) {
            return false;
        }
        let mut direction = None;
        for pair in self.control_points().windows(2) {
            let first = match axis {
                Axis2::X => pair[0].x(),
                Axis2::Y => pair[0].y(),
            };
            let second = match axis {
                Axis2::X => pair[1].x(),
                Axis2::Y => pair[1].y(),
            };
            let Some(ordering) = compare_reals(first, second, policy) else {
                return false;
            };
            if ordering.is_eq() {
                continue;
            }
            if direction.is_some_and(|direction| direction != ordering) {
                return false;
            }
            direction = Some(ordering);
        }
        true
    }

    fn certified_polynomial_graph_component(
        &self,
        other: &Self,
        policy: &CurveContext,
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
        policy: &CurveContext,
    ) -> Classification<bool> {
        if let (Some(first), Some(second)) = (
            self.data.lineage.root.circular_conic.get(),
            other.data.lineage.root.circular_conic.get(),
        ) && (first == second
            || (is_zero(&first.center.distance_squared(&second.center), policy) == Some(true)
                && is_zero(&(&first.radius_squared - &second.radius_squared), policy)
                    == Some(true)))
        {
            return Classification::Decided(true);
        }
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
        if first == second {
            return Classification::Decided(true);
        }
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

    fn implicit_quadratic_conic(
        &self,
        policy: &CurveContext,
    ) -> Classification<Option<&[Real; 6]>> {
        if let Some(coefficients) = self.data.lineage.root.implicit_quadratic_conic.get() {
            return Classification::Decided(Some(coefficients));
        }
        if self.degree() != 2 {
            return Classification::Decided(None);
        }
        self.retain_quadratic_conic_parameter_frame(policy);
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
            .set(Arc::new(coefficients));
        Classification::Decided(Some(
            self.data
                .lineage
                .root
                .implicit_quadratic_conic
                .get()
                .expect("decided implicit conic was retained"),
        ))
    }

    fn retain_quadratic_conic_parameter_frame(&self, policy: &CurveContext) {
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
        let _ = root.quadratic_conic_parameter_frame.set(Arc::new(frame));
    }

    pub(crate) fn retained_quadratic_representative(
        &self,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<RationalQuadraticBezier2>>> {
        if self.degree() == 2 {
            self.retain_quadratic_conic_parameter_frame(policy);
        }
        let range = self.source_parameter_range();
        let forward = range.start() == &Real::zero() && range.end() == &Real::one();
        let reversed = range.start() == &Real::one() && range.end() == &Real::zero();
        let retained_frame = self.data.lineage.root.quadratic_conic_parameter_frame.get();
        let retained_subcurve_frame;
        let structural_frame;
        let ordered = if let Some(frame) = retained_frame
            && forward
        {
            [&frame[0], &frame[1], &frame[2]]
        } else if let Some(frame) = retained_frame
            && reversed
        {
            [&frame[2], &frame[1], &frame[0]]
        } else if let Some(frame) = retained_frame {
            // The root frame and retained source range already certify this
            // subcurve, so evaluate its quadratic blossom instead of re-proving reduction.
            retained_subcurve_frame = [
                quadratic_homogeneous_blossom(frame, range.start(), range.start()),
                quadratic_homogeneous_blossom(frame, range.start(), range.end()),
                quadratic_homogeneous_blossom(frame, range.end(), range.end()),
            ];
            [
                &retained_subcurve_frame[0],
                &retained_subcurve_frame[1],
                &retained_subcurve_frame[2],
            ]
        } else {
            structural_frame =
                match exact_quadratic_homogeneous_reduction(self.homogeneous_controls(), policy) {
                    Classification::Decided(Some(frame)) => frame,
                    Classification::Decided(None) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
            [
                &structural_frame[0],
                &structural_frame[1],
                &structural_frame[2],
            ]
        };
        let mut controls = Vec::with_capacity(3);
        for point in ordered {
            match project_homogeneous(point, policy) {
                Classification::Decided(point) => controls.push(point),
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            }
        }
        let representative = RationalQuadraticBezier2::try_new(
            controls[0].clone(),
            controls[1].clone(),
            controls[2].clone(),
            ordered[0].weight.clone(),
            ordered[1].weight.clone(),
            ordered[2].weight.clone(),
        )?
        .with_retained_conic_provenance(
            self.data
                .lineage
                .root
                .implicit_quadratic_conic
                .get()
                .cloned(),
            self.data.lineage.root.circular_conic.get().cloned(),
        );
        Ok(Classification::Decided(Some(representative)))
    }

    fn polynomial_graph(
        &self,
        axis: Axis2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<PolynomialGraph2>>> {
        if let Some(line) = self.exact_linear_parameterization_line() {
            let (axis_start, axis_end, dependent_start, dependent_end) = match axis {
                Axis2::X => (
                    line.start().x(),
                    line.end().x(),
                    line.start().y(),
                    line.end().y(),
                ),
                Axis2::Y => (
                    line.start().y(),
                    line.end().y(),
                    line.start().x(),
                    line.end().x(),
                ),
            };
            let scale = axis_end - axis_start;
            return Ok(match is_zero(&scale, policy) {
                Some(false) => Classification::Decided(Some(PolynomialGraph2 {
                    axis,
                    origin: axis_start.clone(),
                    scale,
                    dependent: vec![dependent_start.clone(), dependent_end - dependent_start],
                })),
                Some(true) => Classification::Decided(None),
                None => Classification::Uncertain(UncertaintyReason::RealSign),
            });
        }
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
        policy: &CurveContext,
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
        let shares_control_net = if first_subcurve.degree() == second_subcurve.degree() {
            first_subcurve
                .endpoint_parameter_relation(&second_subcurve, reversed, policy)
                .map(|relation| relation.is_some())
        } else {
            first_subcurve.same_projective_control_net_degree_aligned(
                &second_subcurve,
                reversed,
                policy,
            )
        };
        match shares_control_net {
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
            endpoint_inclusion: [true, true],
        }))
    }
}

impl PolynomialGraph2 {
    fn contains_curve(
        &self,
        curve: &RationalBezier2,
        policy: &CurveContext,
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
    policy: &CurveContext,
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
    quadratic_conic_homogeneous_point_parameters(&homogeneous_point, target, policy)
}

fn quadratic_conic_point_parameters(
    point: &Point2,
    target: &RationalBezier2,
    policy: &CurveContext,
) -> Classification<Option<Vec<BezierParameter2>>> {
    quadratic_conic_homogeneous_point_parameters(
        &[point.x().clone(), point.y().clone(), Real::one()],
        target,
        policy,
    )
}

fn quadratic_conic_homogeneous_point_parameters(
    homogeneous_point: &[Real; 3],
    target: &RationalBezier2,
    policy: &CurveContext,
) -> Classification<Option<Vec<BezierParameter2>>> {
    let controls = quadratic_conic_parameter_frame(target);
    let first = homogeneous_control_vector(&controls[0]);
    let middle = homogeneous_control_vector(&controls[1]);
    let last = homogeneous_control_vector(&controls[2]);
    let coordinates = [
        dot3(homogeneous_point, &cross3(&middle, &last)),
        dot3(homogeneous_point, &cross3(&last, &first)),
        dot3(homogeneous_point, &cross3(&first, &middle)),
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
        .map(Arc::as_ref)
        .unwrap_or_else(|| {
            curve
                .homogeneous_controls()
                .try_into()
                .expect("quadratic curve has three homogeneous controls")
        })
}

fn quadratic_homogeneous_blossom(
    frame: &[HomogeneousPoint2; 3],
    first: &Real,
    second: &Real,
) -> HomogeneousPoint2 {
    let one_minus_first = Real::one() - first;
    let one_minus_second = Real::one() - second;
    let first_scale = &one_minus_first * &one_minus_second;
    let middle_scale = &one_minus_first * second + first * &one_minus_second;
    let last_scale = first * second;
    let mut point = frame[0].scaled(&first_scale);
    point.add_scaled(&frame[1], &middle_scale);
    point.add_scaled(&frame[2], &last_scale);
    point
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
    policy: &CurveContext,
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
        endpoint_inclusion: [true, true],
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

fn substitute_implicit_conic(
    conic: &[Real; 6],
    curve: &RationalParametricCurve2,
) -> Option<Vec<Real>> {
    let terms = [
        (&curve.x_numerator, &curve.x_numerator, &conic[0]),
        (&curve.x_numerator, &curve.y_numerator, &conic[1]),
        (&curve.y_numerator, &curve.y_numerator, &conic[2]),
        (&curve.x_numerator, &curve.weight, &conic[3]),
        (&curve.y_numerator, &curve.weight, &conic[4]),
        (&curve.weight, &curve.weight, &conic[5]),
    ];
    let mut substituted = vec![Real::zero()];
    for (left, right, scale) in terms {
        let product = multiply_power_polynomials(left, right)?;
        add_scaled_power_polynomial(&mut substituted, &product, scale);
    }
    Some(substituted)
}

fn homogeneous_linear_form(
    curve: &RationalParametricCurve2,
    coefficients: &[Real; 3],
) -> Vec<Real> {
    let mut form = vec![Real::zero()];
    for (coordinate, scale) in [
        (&curve.x_numerator, &coefficients[0]),
        (&curve.y_numerator, &coefficients[1]),
        (&curve.weight, &coefficients[2]),
    ] {
        add_scaled_power_polynomial(&mut form, coordinate, scale);
    }
    form
}

fn add_power_polynomials(left: &[Real], right: &[Real]) -> Vec<Real> {
    let mut sum = left.to_vec();
    add_scaled_power_polynomial(&mut sum, right, &Real::one());
    sum
}

fn parameter_root_representation(
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> AlgebraicRootRepresentation {
    match parameter {
        BezierParameter2::Exact(parameter) => exact_real_algebraic_representation(parameter),
        BezierParameter2::Algebraic(parameter) => parameter_representation(parameter, policy),
    }
}

fn complete_rational_bezier_image_overlap(
    reversed: bool,
) -> Classification<RationalBezierSharedComponentReplay> {
    Classification::Decided(RationalBezierSharedComponentReplay::Overlap(
        RationalBezierIntersectionOverlap2 {
            first_range: BezierParameterRange2::from_exact(Real::zero(), Real::one()),
            second_range: if reversed {
                BezierParameterRange2::from_exact(Real::one(), Real::zero())
            } else {
                BezierParameterRange2::from_exact(Real::zero(), Real::one())
            },
            orientation: if reversed {
                RationalBezierOverlapOrientation2::Reversed
            } else {
                RationalBezierOverlapOrientation2::Same
            },
            endpoint_inclusion: [true, true],
        },
    ))
}

fn endpoint_projective_parameter_image(
    parameter: &BezierParameter2,
    second_to_first_scale: &Real,
    reversed: bool,
    first_to_second: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let one = Real::one();
    let zero = Real::zero();
    let (numerator, denominator) = match (reversed, first_to_second) {
        (false, true) => (
            [zero, one.clone()],
            [second_to_first_scale.clone(), &one - second_to_first_scale],
        ),
        (false, false) => (
            [Real::zero(), second_to_first_scale.clone()],
            [one.clone(), second_to_first_scale - &one],
        ),
        (true, _) => (
            [
                second_to_first_scale.clone(),
                -second_to_first_scale.clone(),
            ],
            [second_to_first_scale.clone(), &one - second_to_first_scale],
        ),
    };
    projective_parameter_image(parameter, &numerator, &denominator, policy)
}

fn range_projective_parameter_image(
    parameter: &BezierParameter2,
    first_range: &BezierParameterRange2,
    second_range: &BezierParameterRange2,
    second_to_first_scale: &Real,
    reversed: bool,
    first_to_second: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let (Some(first_start), Some(first_end)) =
        (first_range.start().as_exact(), first_range.end().as_exact())
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let (second_start, second_end) = if reversed {
        (
            second_range.end().as_exact(),
            second_range.start().as_exact(),
        )
    } else {
        (
            second_range.start().as_exact(),
            second_range.end().as_exact(),
        )
    };
    let (Some(second_start), Some(second_end)) = (second_start, second_end) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let second_span = second_end - second_start;
    let denominator = [
        second_to_first_scale * first_end - first_start,
        Real::one() - second_to_first_scale,
    ];
    let aligned_numerator = if reversed {
        [second_to_first_scale * first_end, -second_to_first_scale]
    } else {
        [-first_start.clone(), Real::one()]
    };
    let numerator = [
        second_start * &denominator[0] + &second_span * &aligned_numerator[0],
        second_start * &denominator[1] + second_span * &aligned_numerator[1],
    ];
    if first_to_second {
        projective_parameter_image(parameter, &numerator, &denominator, policy)
    } else {
        let inverse_numerator = [numerator[0].clone(), -denominator[0].clone()];
        let inverse_denominator = [-numerator[1].clone(), denominator[1].clone()];
        projective_parameter_image(parameter, &inverse_numerator, &inverse_denominator, policy)
    }
}

fn projective_parameter_image(
    parameter: &BezierParameter2,
    numerator: &[Real; 2],
    denominator: &[Real; 2],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    if let Some(parameter) = parameter.as_exact() {
        let numerator = &numerator[0] + &numerator[1] * parameter;
        let denominator = &denominator[0] + &denominator[1] * parameter;
        let mapped = match numerator / denominator {
            Ok(mapped) => mapped,
            Err(_) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
        };
        return Ok(match in_closed_unit_interval(&mapped, policy) {
            Some(true) => Classification::Decided(Some(BezierParameter2::Exact(mapped))),
            Some(false) => Classification::Decided(None),
            None => Classification::Uncertain(UncertaintyReason::Ordering),
        });
    }
    let root = parameter_root_representation(parameter, policy);
    let candidate = match conic_parameter_candidate(
        &root.polynomial_coefficients,
        &(numerator.to_vec(), denominator.to_vec()),
        policy,
    )? {
        Classification::Decided(candidate) => candidate,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    conic_parameter_from_candidates(std::slice::from_ref(&candidate), parameter, policy)
}

fn overlap_parameter_on_curve(
    source: &RationalBezier2,
    target: &RationalBezier2,
    source_parameter: &BezierParameter2,
    mut unresolved: Option<UncertaintyReason>,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let has_conic_parameter_frame = target.degree() == 2
        || target
            .data
            .lineage
            .root
            .quadratic_conic_parameter_frame
            .get()
            .is_some();
    let target_is_conic = if has_conic_parameter_frame {
        match target.implicit_quadratic_conic(policy) {
            Classification::Decided(Some(_)) => true,
            Classification::Decided(None) => false,
            Classification::Uncertain(reason) => {
                unresolved = Some(reason);
                false
            }
        }
    } else {
        false
    };
    if target_is_conic {
        match conic_parameter_map(target, source, policy)? {
            Classification::Decided(parameter_map) => {
                let root = parameter_root_representation(source_parameter, policy);
                match conic_parameter_candidate(
                    &root.polynomial_coefficients,
                    &parameter_map.primary,
                    policy,
                )? {
                    Classification::Decided(primary) => {
                        return conic_parameter_from_curve_parameter(
                            &parameter_map,
                            &primary,
                            &root.polynomial_coefficients,
                            source_parameter,
                            false,
                            policy,
                        );
                    }
                    Classification::Uncertain(reason) => unresolved = Some(reason),
                }
            }
            Classification::Uncertain(reason) => unresolved = Some(reason),
        }
    }

    for axis in [Axis2::X, Axis2::Y] {
        let graph = match target.polynomial_graph(axis, policy)? {
            Classification::Decided(Some(graph)) => graph,
            Classification::Decided(None) => continue,
            Classification::Uncertain(reason) => {
                unresolved = Some(reason);
                continue;
            }
        };
        let basis = source.homogeneous_power_basis()?;
        let coordinate = match axis {
            Axis2::X => &basis.x_numerator,
            Axis2::Y => &basis.y_numerator,
        };
        let numerator = subtract_power_polynomials(
            coordinate,
            &scale_power_polynomial(&basis.weight, &graph.origin),
        );
        let denominator = scale_power_polynomial(&basis.weight, &graph.scale);
        let root = parameter_root_representation(source_parameter, policy);
        let candidate = match conic_parameter_candidate(
            &root.polynomial_coefficients,
            &(numerator, denominator),
            policy,
        )? {
            Classification::Decided(candidate) => candidate,
            Classification::Uncertain(reason) => {
                unresolved = Some(reason);
                continue;
            }
        };
        return conic_parameter_from_candidates(&[candidate], source_parameter, policy);
    }

    if let Some(parameter) = source_parameter.as_exact() {
        match source.point_at_classified(parameter, policy) {
            Classification::Decided(point) => {
                return Ok(unique_point_incidence_parameter(target, &point, policy));
            }
            Classification::Uncertain(reason) => unresolved = Some(reason),
        }
    }
    for axis in [Axis2::X, Axis2::Y] {
        if !target.has_certified_injective_axis_on(axis, policy) {
            continue;
        }
        match overlap_parameter_through_injective_axis(
            source,
            target,
            source_parameter,
            axis,
            policy,
        )? {
            Classification::Decided(parameter) => {
                return Ok(Classification::Decided(parameter));
            }
            Classification::Uncertain(reason) => unresolved = Some(reason),
        }
    }
    Ok(Classification::Uncertain(
        unresolved.unwrap_or(UncertaintyReason::Unsupported),
    ))
}

fn overlap_parameter_through_injective_axis(
    source: &RationalBezier2,
    target: &RationalBezier2,
    source_parameter: &BezierParameter2,
    axis: Axis2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let source_basis = source.homogeneous_power_basis()?;
    let source_coordinate = match axis {
        Axis2::X => &source_basis.x_numerator,
        Axis2::Y => &source_basis.y_numerator,
    };
    let source_root = parameter_root_representation(source_parameter, policy);
    let coordinate_map = AlgebraicRootRationalMap::new(
        &source_root.polynomial_coefficients,
        source_coordinate,
        &source_basis.weight,
        policy.predicate_policy(),
    );
    let coordinate_image = coordinate_map.transform(&source_root);
    if coordinate_image.status != AlgebraicRootRationalImageStatus::Transformed {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some(coordinate_image) = coordinate_image.representation.as_ref() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };

    let target_basis = target.homogeneous_power_basis()?;
    let target_coordinate = match axis {
        Axis2::X => &target_basis.x_numerator,
        Axis2::Y => &target_basis.y_numerator,
    };
    let Some(preimage_coefficients) = rational_map_preimage_polynomial(
        &coordinate_image.polynomial_coefficients,
        target_coordinate,
        &target_basis.weight,
    ) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
    };
    let polynomial =
        match BezierParameterPolynomial::try_new_power_basis(preimage_coefficients, policy) {
            Ok(Classification::Decided(polynomial)) => polynomial,
            Ok(Classification::Uncertain(reason)) => {
                return Ok(Classification::Uncertain(reason));
            }
            Err(CurveError::InvalidBezierPolynomial) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
            Err(error) => return Err(error),
        };
    let parameters = match polynomial.isolate_unit_interval_roots(policy)? {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut matched = None;
    let mut unresolved = false;
    for parameter in parameters {
        let Some(replay) = target.candidate_point_replay(&parameter, policy)? else {
            unresolved = true;
            continue;
        };
        let candidate_coordinate = match axis {
            Axis2::X => &replay.x,
            Axis2::Y => &replay.y,
        };
        match algebraic_coordinates_equal(coordinate_image, candidate_coordinate, policy) {
            Some(true) if matched.is_none() => matched = Some(parameter),
            Some(true) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
            Some(false) => {}
            None => unresolved = true,
        }
    }
    if let Some(parameter) = matched {
        return match parameter.promote_represented_rational_root(policy)? {
            Classification::Decided(parameter) => Ok(Classification::Decided(Some(parameter))),
            Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
        };
    }
    Ok(if unresolved {
        Classification::Uncertain(UncertaintyReason::Predicate)
    } else {
        Classification::Decided(None)
    })
}

fn rational_map_preimage_polynomial(
    image_polynomial: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<Vec<Real>> {
    let degree = image_polynomial.len().checked_sub(1)?;
    let numerator_powers = power_polynomial_sequence(numerator, degree)?;
    let denominator_powers = power_polynomial_sequence(denominator, degree)?;
    let mut preimage = vec![Real::zero()];
    for (power, coefficient) in image_polynomial.iter().enumerate() {
        let term = multiply_power_polynomials(
            &numerator_powers[power],
            &denominator_powers[degree - power],
        )?;
        add_scaled_power_polynomial(&mut preimage, &term, coefficient);
    }
    Some(preimage)
}

struct ConicParameterMap2 {
    primary: (Vec<Real>, Vec<Real>),
    coordinates: [Vec<Real>; 3],
    range_start: Real,
    range_span: Real,
}

struct ConicParameterCandidate2 {
    map: AlgebraicRootRationalMap,
    numerator: Vec<Real>,
    denominator: Vec<Real>,
    image_polynomial: OnceLock<Option<BezierParameterPolynomial>>,
    image_parameters: OnceLock<CurveResult<Classification<Vec<BezierParameter2>>>>,
    quotient_matrices: OnceLock<Option<QuotientRingRationalMapMatrices>>,
    quotient_power: OnceLock<Option<Vec<Real>>>,
}

fn conic_parameter_map(
    conic: &RationalBezier2,
    curve: &RationalBezier2,
    policy: &CurveContext,
) -> CurveResult<Classification<ConicParameterMap2>> {
    let controls = quadratic_conic_parameter_frame(conic);
    let first = homogeneous_control_vector(&controls[0]);
    let middle = homogeneous_control_vector(&controls[1]);
    let last = homogeneous_control_vector(&controls[2]);
    let lambda_0 = cross3(&middle, &last);
    if is_zero(&dot3(&first, &lambda_0), policy) != Some(false) {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let lambda_1 = cross3(&last, &first);
    let lambda_2 = cross3(&first, &middle);
    let basis = curve.homogeneous_power_basis()?;
    let coordinate_0 = homogeneous_linear_form(basis, &lambda_0);
    let coordinate_1 = homogeneous_linear_form(basis, &lambda_1);
    let coordinate_2 = homogeneous_linear_form(basis, &lambda_2);
    let two = Real::from(2_i8);
    let twice_coordinate_2 = scale_power_polynomial(&coordinate_2, &two);
    let coordinate_sum = add_power_polynomials(
        &add_power_polynomials(&coordinate_0, &coordinate_1),
        &coordinate_2,
    );
    let right_numerator = add_power_polynomials(&coordinate_1, &twice_coordinate_2);
    let range = conic.source_parameter_range();
    let range_start = range.start().clone();
    let span = range.end() - range.start();
    let primary = localize_conic_parameter_candidate(
        right_numerator,
        scale_power_polynomial(&coordinate_sum, &two),
        &range_start,
        &span,
    );
    Ok(Classification::Decided(ConicParameterMap2 {
        primary,
        coordinates: [coordinate_0, coordinate_1, coordinate_2],
        range_start,
        range_span: span,
    }))
}

fn conic_parameter_from_curve_parameter(
    parameter_map: &ConicParameterMap2,
    primary_candidate: &ConicParameterCandidate2,
    source_polynomial: &[Real],
    curve_parameter: &BezierParameter2,
    prefer_exact_image_polynomial: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    if prefer_exact_image_polynomial {
        match real_coefficient_rational_image_parameter(curve_parameter, primary_candidate, policy)?
        {
            Classification::Decided(Some(parameter)) => {
                return Ok(Classification::Decided(Some(parameter)));
            }
            Classification::Decided(None) | Classification::Uncertain(_) => {}
        }
    }
    let primary = conic_parameter_from_candidates(
        std::slice::from_ref(primary_candidate),
        curve_parameter,
        policy,
    )?;
    let primary_absent = match primary {
        Classification::Decided(Some(parameter)) => {
            return Ok(Classification::Decided(Some(parameter)));
        }
        // For an algebraic source, the retained image route reports `None`
        // only after proving the primary image is disjoint from the target
        // interval. Every nonsingular conic chart represents the same
        // parameter, so rebuilding the two fallback charts cannot recover an
        // in-range value. Exact-source evaluation also uses `None` for a
        // chart pole and must retain the fallback search below.
        Classification::Decided(None) if curve_parameter.as_exact().is_none() => {
            return Ok(Classification::Decided(None));
        }
        Classification::Decided(None) => true,
        Classification::Uncertain(_) => false,
    };

    let [coordinate_0, coordinate_1, coordinate_2] = &parameter_map.coordinates;
    let two = Real::from(2_i8);
    let twice_coordinate_0 = scale_power_polynomial(coordinate_0, &two);
    let twice_coordinate_2 = scale_power_polynomial(coordinate_2, &two);
    let fallback_candidate_polynomials = [
        localize_conic_parameter_candidate(
            coordinate_1.clone(),
            add_power_polynomials(&twice_coordinate_0, coordinate_1),
            &parameter_map.range_start,
            &parameter_map.range_span,
        ),
        localize_conic_parameter_candidate(
            twice_coordinate_2.clone(),
            add_power_polynomials(coordinate_1, &twice_coordinate_2),
            &parameter_map.range_start,
            &parameter_map.range_span,
        ),
    ];
    let mut fallback_candidates = Vec::with_capacity(fallback_candidate_polynomials.len());
    for candidate in &fallback_candidate_polynomials {
        match conic_parameter_candidate(source_polynomial, candidate, policy)? {
            Classification::Decided(candidate) => fallback_candidates.push(candidate),
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        }
    }
    match conic_parameter_from_candidates(&fallback_candidates, curve_parameter, policy)? {
        Classification::Decided(Some(parameter)) => Ok(Classification::Decided(Some(parameter))),
        Classification::Decided(None) => Ok(Classification::Decided(None)),
        Classification::Uncertain(_) if primary_absent => Ok(Classification::Decided(None)),
        Classification::Uncertain(reason) => Ok(Classification::Uncertain(reason)),
    }
}

fn localize_conic_parameter_candidate(
    numerator: Vec<Real>,
    denominator: Vec<Real>,
    range_start: &Real,
    range_span: &Real,
) -> (Vec<Real>, Vec<Real>) {
    (
        subtract_power_polynomials(
            &numerator,
            &scale_power_polynomial(&denominator, range_start),
        ),
        scale_power_polynomial(&denominator, range_span),
    )
}

fn conic_parameter_from_candidates(
    candidates: &[ConicParameterCandidate2],
    curve_parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    if curve_parameter.as_exact().is_some() {
        // An implicit conic can meet the other curve's projective extension at
        // a parameter where one rational chart has a zero denominator. Try
        // every chart and treat a chart's exact pole as absence, not global
        // predicate uncertainty.
        let mut uncertain = None;
        for candidate in candidates {
            match real_coefficient_rational_image_parameter(curve_parameter, candidate, policy)? {
                Classification::Decided(Some(parameter)) => {
                    return Ok(Classification::Decided(Some(parameter)));
                }
                Classification::Decided(None) => {}
                Classification::Uncertain(reason) => uncertain = Some(reason),
            }
        }
        return Ok(uncertain.map_or(Classification::Decided(None), Classification::Uncertain));
    }

    let refinement_steps: &[usize] = if matches!(curve_parameter, BezierParameter2::Exact(_)) {
        &[0]
    } else {
        &[2, 4, 8, 16, 32, 64, 128]
    };
    let mut certified_absent = vec![false; candidates.len()];
    let mut refinement = BezierParameterRefinement2::new(curve_parameter, policy);
    for &max_refinement_steps in refinement_steps {
        let refined_curve_parameter = refinement.refine_to(max_refinement_steps);
        let root = parameter_root_representation(refined_curve_parameter, policy);
        for (candidate_index, candidate) in candidates.iter().enumerate() {
            if certified_absent[candidate_index] {
                continue;
            }
            match rational_image_parameter(&root, candidate, policy)? {
                Classification::Decided(Some(parameter)) => {
                    return Ok(Classification::Decided(Some(parameter)));
                }
                Classification::Decided(None) => certified_absent[candidate_index] = true,
                Classification::Uncertain(_) => {}
            }
        }
        if certified_absent.iter().all(|absent| *absent) {
            return Ok(Classification::Decided(None));
        }
    }
    let mut refinement = BezierParameterRefinement2::new(curve_parameter, policy);
    for &max_refinement_steps in refinement_steps {
        let refined_curve_parameter = refinement.refine_to(max_refinement_steps);
        for (candidate_index, candidate) in candidates.iter().enumerate() {
            if certified_absent[candidate_index] {
                continue;
            }
            match real_coefficient_rational_image_parameter(
                refined_curve_parameter,
                candidate,
                policy,
            )? {
                Classification::Decided(Some(parameter)) => {
                    return Ok(Classification::Decided(Some(parameter)));
                }
                Classification::Decided(None) => certified_absent[candidate_index] = true,
                Classification::Uncertain(_) => {}
            }
        }
        if certified_absent.iter().all(|absent| *absent) {
            return Ok(Classification::Decided(None));
        }
    }
    Ok(Classification::Uncertain(UncertaintyReason::Predicate))
}

fn rational_map_image_polynomial(
    source_polynomial: &[Real],
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> Option<BezierParameterPolynomial> {
    if let Some(coefficients) = quotient_ring_rational_map_image_polynomial(
        source_polynomial,
        numerator,
        denominator,
        policy,
    ) && let Ok(Classification::Decided(polynomial)) =
        BezierParameterPolynomial::try_new_power_basis(coefficients, policy)
    {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "conic-rational-image-fallback",
            "quotient-ring-resultant",
        );
        return Some(polynomial);
    }
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "conic-rational-image-fallback",
        "sampled-bareiss-resultant",
    );
    let source_degree = source_polynomial.len().checked_sub(1)?;
    let mut samples = Vec::with_capacity(source_degree + 1);
    for sample in 0..=source_degree {
        let value = Real::from(i64::try_from(sample).ok()?);
        let relation =
            subtract_power_polynomials(numerator, &scale_power_polynomial(denominator, &value));
        let resultant = resultant_univariate_polynomials(
            source_polynomial,
            &relation,
            RATIONAL_INTERSECTION_RESULTANT_PRECISION,
        )
        .ok()?
        .resultant;
        samples.push(resultant);
    }
    let coefficients = interpolate_exact_real_samples(&samples)?;
    match BezierParameterPolynomial::try_new_power_basis(coefficients, policy).ok()? {
        Classification::Decided(polynomial) => Some(polynomial),
        Classification::Uncertain(_) => None,
    }
}

fn quotient_ring_rational_map_image_coefficients(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<Vec<Real>> {
    let matrices = quotient_ring_rational_map_matrices(source, numerator, denominator)?;
    determinant_linear_power_polynomial(&matrices.numerator, &matrices.denominator, matrices.degree)
}

fn quotient_ring_rational_map_image_polynomial(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> Option<Vec<Real>> {
    match trim_power_polynomial(
        quotient_ring_rational_map_image_coefficients(source, numerator, denominator)?,
        policy,
    ) {
        Classification::Decided(polynomial) => Some(polynomial),
        Classification::Uncertain(_) => None,
    }
}

struct QuotientRingRationalMapMatrices {
    degree: usize,
    numerator: Vec<Real>,
    denominator: Vec<Real>,
}

fn quotient_ring_rational_map_matrices(
    source: &[Real],
    numerator: &[Real],
    denominator: &[Real],
) -> Option<QuotientRingRationalMapMatrices> {
    let degree = source.len().checked_sub(1)?;
    if degree == 0
        || degree > MAX_QUOTIENT_RING_RATIONAL_IMAGE_DEGREE
        || numerator.is_empty()
        || denominator.is_empty()
    {
        return None;
    }
    let leading = source.last()?;
    let inverse_leading = (!leading.structural_facts().exact_rational)
        .then(|| leading.inverse_ref())
        .transpose()
        .ok()?;
    let numerator_matrix =
        quotient_multiplication_matrix(source, numerator, inverse_leading.as_ref())?;
    let denominator_matrix =
        quotient_multiplication_matrix(source, denominator, inverse_leading.as_ref())?;
    Some(QuotientRingRationalMapMatrices {
        degree,
        numerator: numerator_matrix,
        denominator: denominator_matrix,
    })
}

fn determinant_linear_power_polynomial(
    constants: &[Real],
    negative_linear_coefficients: &[Real],
    degree: usize,
) -> Option<Vec<Real>> {
    let matrix_entries = degree.checked_mul(degree)?;
    if constants.len() != matrix_entries || negative_linear_coefficients.len() != matrix_entries {
        return None;
    }
    // The determinant of multiplication by n(x) - y*d(x) in R[x]/(source)
    // is its exact norm, hence the required resultant up to one nonzero scale.
    // Subset expansion visits each partial column set once and keeps the matrix
    // entries as linear polynomials in y.
    let state_count = 1_usize.checked_shl(u32::try_from(degree).ok()?)?;
    let mut partials = vec![None; state_count];
    partials[0] = Some(vec![Real::one()]);
    for mask in 0..state_count {
        let row = usize::try_from(mask.count_ones()).ok()?;
        if row == degree {
            continue;
        }
        let Some(partial) = partials[mask].take() else {
            continue;
        };
        for column in 0..degree {
            let column_bit = 1_usize.checked_shl(u32::try_from(column).ok()?)?;
            if mask & column_bit != 0 {
                continue;
            }
            let entry_index = row * degree + column;
            let negative = (mask >> (column + 1)).count_ones() % 2 != 0;
            let next = partials[mask | column_bit]
                .get_or_insert_with(|| vec![Real::zero(); partial.len() + 1]);
            for (power, coefficient) in partial.iter().enumerate() {
                let constant = coefficient * &constants[entry_index];
                let linear = coefficient * &negative_linear_coefficients[entry_index];
                if negative {
                    next[power] -= constant;
                    next[power + 1] += linear;
                } else {
                    next[power] += constant;
                    next[power + 1] -= linear;
                }
            }
        }
    }
    partials.pop()?
}

#[derive(Clone)]
#[cfg_attr(test, derive(Debug, PartialEq))]
struct CertifiedRationalInterval {
    lower: HyperRational,
    upper: HyperRational,
}

impl CertifiedRationalInterval {
    fn zero() -> Self {
        Self::point(HyperRational::zero())
    }

    fn point(value: HyperRational) -> Self {
        Self {
            lower: value.clone(),
            upper: value,
        }
    }

    fn add_assign(&mut self, other: &Self) {
        self.lower = &self.lower + &other.lower;
        self.upper = &self.upper + &other.upper;
    }

    fn subtract(&self, other: &Self) -> Self {
        Self {
            lower: &self.lower - &other.upper,
            upper: &self.upper - &other.lower,
        }
    }

    fn subtract_assign(&mut self, other: &Self) {
        self.lower = &self.lower - &other.upper;
        self.upper = &self.upper - &other.lower;
    }

    fn multiply_scalar(&self, scalar: &HyperRational) -> Self {
        if scalar >= &HyperRational::zero() {
            Self {
                lower: &self.lower * scalar,
                upper: &self.upper * scalar,
            }
        } else {
            Self {
                lower: &self.upper * scalar,
                upper: &self.lower * scalar,
            }
        }
    }

    fn multiply(&self, other: &Self) -> Self {
        let zero = HyperRational::zero();
        if self.lower >= zero {
            if other.lower >= zero {
                Self {
                    lower: &self.lower * &other.lower,
                    upper: &self.upper * &other.upper,
                }
            } else if other.upper <= zero {
                Self {
                    lower: &self.upper * &other.lower,
                    upper: &self.lower * &other.upper,
                }
            } else {
                Self {
                    lower: &self.upper * &other.lower,
                    upper: &self.upper * &other.upper,
                }
            }
        } else if self.upper <= zero {
            if other.lower >= zero {
                Self {
                    lower: &self.lower * &other.upper,
                    upper: &self.upper * &other.lower,
                }
            } else if other.upper <= zero {
                Self {
                    lower: &self.upper * &other.upper,
                    upper: &self.lower * &other.lower,
                }
            } else {
                Self {
                    lower: &self.lower * &other.upper,
                    upper: &self.lower * &other.lower,
                }
            }
        } else if other.lower >= zero {
            Self {
                lower: &self.lower * &other.upper,
                upper: &self.upper * &other.upper,
            }
        } else if other.upper <= zero {
            Self {
                lower: &self.upper * &other.lower,
                upper: &self.lower * &other.lower,
            }
        } else {
            let lower_left = &self.lower * &other.upper;
            let lower_right = &self.upper * &other.lower;
            let upper_left = &self.lower * &other.lower;
            let upper_right = &self.upper * &other.upper;
            Self {
                lower: if lower_left < lower_right {
                    lower_left
                } else {
                    lower_right
                },
                upper: if upper_left > upper_right {
                    upper_left
                } else {
                    upper_right
                },
            }
        }
    }

    fn sign(&self) -> Option<RealSign> {
        if self.lower > HyperRational::zero() {
            Some(RealSign::Positive)
        } else if self.upper < HyperRational::zero() {
            Some(RealSign::Negative)
        } else if self.lower == HyperRational::zero() && self.upper == HyperRational::zero() {
            Some(RealSign::Zero)
        } else {
            None
        }
    }
}

fn determinant_local_bernstein_signs_from_enclosures(
    matrices: &QuotientRingRationalMapMatrices,
    lower: &HyperRational,
    upper: &HyperRational,
    precision: i32,
) -> Option<(Vec<RealSign>, RealSign)> {
    let enclose = |value: &Real| {
        value
            .certified_dyadic_interval(precision)
            .map(|interval| CertifiedRationalInterval {
                lower: interval[0].clone(),
                upper: interval[1].clone(),
            })
    };
    let numerator = matrices
        .numerator
        .iter()
        .map(enclose)
        .collect::<Option<Vec<_>>>()?;
    let denominator = matrices
        .denominator
        .iter()
        .map(enclose)
        .collect::<Option<Vec<_>>>()?;
    let span = upper - lower;
    let constants = numerator
        .iter()
        .zip(&denominator)
        .map(|(numerator, denominator)| numerator.subtract(&denominator.multiply_scalar(lower)))
        .collect::<Vec<_>>();
    let linear = denominator
        .iter()
        .map(|coefficient| coefficient.multiply_scalar(&span))
        .collect::<Vec<_>>();

    let state_count = 1_usize.checked_shl(u32::try_from(matrices.degree).ok()?)?;
    let mut partials = vec![None; state_count];
    partials[0] = Some(vec![CertifiedRationalInterval::point(HyperRational::one())]);
    for mask in 0..state_count {
        let row = usize::try_from(mask.count_ones()).ok()?;
        if row == matrices.degree {
            continue;
        }
        let Some(partial) = partials[mask].take() else {
            continue;
        };
        for column in 0..matrices.degree {
            let column_bit = 1_usize.checked_shl(u32::try_from(column).ok()?)?;
            if mask & column_bit != 0 {
                continue;
            }
            let entry_index = row * matrices.degree + column;
            let negative = (mask >> (column + 1)).count_ones() % 2 != 0;
            let next = partials[mask | column_bit]
                .get_or_insert_with(|| vec![CertifiedRationalInterval::zero(); partial.len() + 1]);
            for (power, coefficient) in partial.iter().enumerate() {
                let constant = coefficient.multiply(&constants[entry_index]);
                let linear = coefficient.multiply(&linear[entry_index]);
                if negative {
                    next[power].subtract_assign(&constant);
                    next[power + 1].add_assign(&linear);
                } else {
                    next[power].add_assign(&constant);
                    next[power + 1].subtract_assign(&linear);
                }
            }
        }
    }
    let power = partials.pop()??;
    let leading_power_sign = match power.last()?.sign()? {
        RealSign::Positive => RealSign::Positive,
        RealSign::Negative => RealSign::Negative,
        RealSign::Zero => return None,
    };
    let mut signs = Vec::with_capacity(matrices.degree + 1);
    for index in 0..=matrices.degree {
        let mut coefficient = CertifiedRationalInterval::zero();
        for (power_index, power_coefficient) in power.iter().enumerate().take(index + 1) {
            let numerator = checked_binomial(index, power_index)?;
            let denominator = checked_binomial(matrices.degree, power_index)?;
            let weight =
                HyperRational::fraction(i64::try_from(numerator).ok()?, denominator).ok()?;
            coefficient.add_assign(&power_coefficient.multiply_scalar(&weight));
        }
        signs.push(coefficient.sign()?);
    }
    Some((signs, leading_power_sign))
}

fn quotient_multiplication_matrix(
    source: &[Real],
    relation: &[Real],
    inverse_leading: Option<&Real>,
) -> Option<Vec<Real>> {
    let degree = source.len().checked_sub(1)?;
    let leading = source.last()?;
    let mut matrix = vec![Real::zero(); degree.checked_mul(degree)?];
    for column in 0..degree {
        let mut remainder = vec![Real::zero(); relation.len().checked_add(column)?];
        remainder[column..].clone_from_slice(relation);
        while remainder.len() > degree {
            let coefficient = remainder.pop()?;
            let shift = remainder.len().checked_sub(degree)?;
            let factor = if let Some(inverse) = inverse_leading {
                coefficient * inverse
            } else {
                (coefficient / leading).ok()?
            };
            for (index, source_coefficient) in source[..degree].iter().enumerate() {
                remainder[shift + index] -= &factor * source_coefficient;
            }
        }
        remainder.resize(degree, Real::zero());
        for (row, coefficient) in remainder.into_iter().enumerate() {
            matrix[row * degree + column] = coefficient;
        }
    }
    Some(matrix)
}

fn interpolate_exact_real_samples(samples: &[Real]) -> Option<Vec<Real>> {
    let mut polynomial = vec![Real::zero(); samples.len()];
    for (sample_index, sample) in samples.iter().enumerate() {
        let mut basis = vec![Real::one()];
        let mut denominator = Real::one();
        let sample_value = Real::from(i64::try_from(sample_index).ok()?);
        for other_index in 0..samples.len() {
            if sample_index == other_index {
                continue;
            }
            let other_value = Real::from(i64::try_from(other_index).ok()?);
            basis = multiply_power_polynomial_by_linear_factor(basis, -other_value.clone());
            denominator *= &sample_value - other_value;
        }
        let scale = (sample / denominator).ok()?;
        add_scaled_power_polynomial(&mut polynomial, &basis, &scale);
    }
    Some(polynomial)
}

fn multiply_power_polynomial_by_linear_factor(polynomial: Vec<Real>, constant: Real) -> Vec<Real> {
    let mut product = vec![Real::zero(); polynomial.len() + 1];
    for (index, coefficient) in polynomial.into_iter().enumerate() {
        product[index] += &coefficient * &constant;
        product[index + 1] += coefficient;
    }
    product
}

fn locally_certified_rational_image_parameter(
    source_parameter: &BezierParameter2,
    candidate: &ConicParameterCandidate2,
    policy: &CurveContext,
) -> CurveResult<Option<Classification<Option<BezierParameter2>>>> {
    let Some(matrices) = candidate
        .quotient_matrices
        .get_or_init(|| {
            let BezierParameter2::Algebraic(source) = source_parameter else {
                return None;
            };
            quotient_ring_rational_map_matrices(
                source.polynomial().coefficients(),
                &candidate.numerator,
                &candidate.denominator,
            )
        })
        .as_ref()
    else {
        return Ok(None);
    };
    let mut refinement = BezierParameterRefinement2::new(source_parameter, policy);
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256] {
        let refined = refinement.refine_to(refinement_steps);
        let source_interval = match refined.known_interval(policy)? {
            Classification::Decided(interval) => ExactRealInterval {
                lower: interval.start().clone(),
                upper: interval.end().clone(),
            },
            Classification::Uncertain(_) => continue,
        };
        let Some(image_interval) = evaluate_rational_map_interval(
            &candidate.numerator,
            &candidate.denominator,
            &source_interval,
            policy,
        ) else {
            continue;
        };
        if compare_reals(&image_interval.upper, &Real::zero(), policy) == Some(Ordering::Less)
            || compare_reals(&image_interval.lower, &Real::one(), policy) == Some(Ordering::Greater)
        {
            return Ok(Some(Classification::Decided(None)));
        }
        if !matches!(
            compare_reals(&image_interval.lower, &Real::zero(), policy),
            Some(Ordering::Greater | Ordering::Equal)
        ) || !matches!(
            compare_reals(&image_interval.upper, &Real::one(), policy),
            Some(Ordering::Less | Ordering::Equal)
        ) || compare_reals(&image_interval.lower, &image_interval.upper, policy)
            != Some(Ordering::Less)
        {
            continue;
        }

        let enclosure_precision = match refinement_steps {
            0..=4 => -4,
            5..=8 => -6,
            9..=16 => -8,
            17..=32 => -12,
            33..=64 => -16,
            65..=128 => -24,
            _ => -32,
        };
        let Some(lower_enclosure) = image_interval
            .lower
            .certified_dyadic_interval(enclosure_precision)
        else {
            continue;
        };
        let Some(upper_enclosure) = image_interval
            .upper
            .certified_dyadic_interval(enclosure_precision)
        else {
            continue;
        };
        let lower = if lower_enclosure[0] < HyperRational::zero() {
            HyperRational::zero()
        } else {
            lower_enclosure[0].clone()
        };
        let upper = if upper_enclosure[1] > HyperRational::one() {
            HyperRational::one()
        } else {
            upper_enclosure[1].clone()
        };
        if lower >= upper {
            continue;
        }

        // Localize `det(M_N - u M_D)` to the rational target enclosure and
        // propagate certified dyadic coefficient intervals through the small
        // determinant. This avoids first materializing a combinatorial `Real`
        // expression merely to ask for the signs of its Bernstein controls.
        let Some((signs, _leading_power_sign)) = [-16, -32, -64, -128, -256, -512]
            .into_iter()
            .find_map(|precision| {
                determinant_local_bernstein_signs_from_enclosures(
                    matrices, &lower, &upper, precision,
                )
            })
        else {
            continue;
        };
        let first_sign = signs[0];
        let last_sign = signs[signs.len() - 1];
        if first_sign == RealSign::Zero || last_sign == RealSign::Zero {
            continue;
        }
        let mut previous = None;
        let mut variations = 0_usize;
        for sign in signs {
            if sign == RealSign::Zero {
                continue;
            }
            if previous.is_some_and(|previous| previous != sign) {
                variations += 1;
            }
            previous = Some(sign);
        }
        if variations != 1 {
            continue;
        }

        let interval =
            match BezierParameterInterval::try_new(Real::new(lower), Real::new(upper), policy)? {
                Classification::Decided(interval) => interval,
                Classification::Uncertain(_) => continue,
            };
        let Some(global_power) = candidate
            .quotient_power
            .get_or_init(|| {
                determinant_linear_power_polynomial(
                    &matrices.numerator,
                    &matrices.denominator,
                    matrices.degree,
                )
            })
            .as_ref()
        else {
            continue;
        };
        let Some(parameter) = BezierAlgebraicParameter2::from_certified_simple_power_basis(
            global_power.clone(),
            interval,
        ) else {
            continue;
        };
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "conic-rational-image-fallback",
            "local-bernstein-resultant",
        );
        return Ok(Some(Classification::Decided(Some(
            BezierParameter2::Algebraic(parameter),
        ))));
    }
    Ok(None)
}

fn real_coefficient_rational_image_parameter(
    source_parameter: &BezierParameter2,
    candidate: &ConicParameterCandidate2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    if let Some(source) = source_parameter.as_exact() {
        let numerator = evaluate_power_polynomial(&candidate.numerator, source);
        let denominator = evaluate_power_polynomial(&candidate.denominator, source);
        match is_zero(&denominator, policy) {
            Some(true) => return Ok(Classification::Decided(None)),
            Some(false) => {}
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let value = (numerator / denominator)?;
        return match BezierParameter2::exact(value, policy) {
            Ok(Classification::Decided(parameter)) => Ok(Classification::Decided(Some(parameter))),
            Err(CurveError::InvalidBezierParameter) => Ok(Classification::Decided(None)),
            Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
            Err(error) => Err(error),
        };
    }

    let BezierParameter2::Algebraic(source_algebraic) = source_parameter else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    if let Some(result) =
        locally_certified_rational_image_parameter(source_parameter, candidate, policy)?
    {
        return Ok(result);
    }
    let Some(image_polynomial) = candidate.image_polynomial.get_or_init(|| {
        #[cfg(feature = "dispatch-trace")]
        hyperreal::dispatch_trace::record(
            "hypercurve",
            "conic-rational-image-fallback",
            "construct-image-polynomial",
        );
        rational_map_image_polynomial(
            source_algebraic.polynomial().coefficients(),
            &candidate.numerator,
            &candidate.denominator,
            policy,
        )
    }) else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    let image_parameters = match candidate
        .image_parameters
        .get_or_init(|| image_polynomial.isolate_unit_interval_roots(policy))
    {
        Ok(Classification::Decided(parameters)) => parameters,
        Ok(Classification::Uncertain(reason)) => {
            return Ok(Classification::Uncertain(*reason));
        }
        Err(error) => return Err(error.clone()),
    };
    if image_parameters.is_empty() {
        return Ok(Classification::Decided(None));
    }

    let mut refinement = BezierParameterRefinement2::new(source_parameter, policy);
    for refinement_steps in [0, 2, 4, 8, 16, 32, 64] {
        let refined = refinement.refine_to(refinement_steps);
        let source_interval = match refined.known_interval(policy)? {
            Classification::Decided(interval) => ExactRealInterval {
                lower: interval.start().clone(),
                upper: interval.end().clone(),
            },
            Classification::Uncertain(reason) => {
                return Ok(Classification::Uncertain(reason));
            }
        };
        let Some(image_interval) = evaluate_rational_map_interval(
            &candidate.numerator,
            &candidate.denominator,
            &source_interval,
            policy,
        ) else {
            continue;
        };
        if compare_reals(&image_interval.upper, &Real::zero(), policy) == Some(Ordering::Less)
            || compare_reals(&image_interval.lower, &Real::one(), policy) == Some(Ordering::Greater)
        {
            return Ok(Classification::Decided(None));
        }

        let mut containing = image_parameters.iter().filter(|parameter| {
            let Ok(Classification::Decided(interval)) = parameter.known_interval(policy) else {
                return false;
            };
            matches!(
                compare_reals(interval.start(), &image_interval.lower, policy),
                Some(Ordering::Less | Ordering::Equal)
            ) && matches!(
                compare_reals(&image_interval.upper, interval.end(), policy),
                Some(Ordering::Less | Ordering::Equal)
            )
        });
        let Some(parameter) = containing.next() else {
            continue;
        };
        if containing.next().is_none() {
            return Ok(Classification::Decided(Some(parameter.clone())));
        }
    }
    Ok(Classification::Uncertain(UncertaintyReason::Predicate))
}

#[derive(Clone)]
struct ExactRealInterval {
    lower: Real,
    upper: Real,
}

fn evaluate_rational_map_interval(
    numerator: &[Real],
    denominator: &[Real],
    parameter: &ExactRealInterval,
    policy: &CurveContext,
) -> Option<ExactRealInterval> {
    let numerator = evaluate_power_polynomial_interval(numerator, parameter, policy)?;
    let denominator = evaluate_power_polynomial_interval(denominator, parameter, policy)?;
    let reciprocal = reciprocal_interval(&denominator, policy)?;
    multiply_intervals(&numerator, &reciprocal, policy)
}

fn evaluate_power_polynomial_interval(
    coefficients: &[Real],
    parameter: &ExactRealInterval,
    policy: &CurveContext,
) -> Option<ExactRealInterval> {
    let mut value = ExactRealInterval {
        lower: Real::zero(),
        upper: Real::zero(),
    };
    for coefficient in coefficients.iter().rev() {
        value = multiply_intervals(&value, parameter, policy)?;
        value.lower += coefficient;
        value.upper += coefficient;
    }
    Some(value)
}

fn reciprocal_interval(
    interval: &ExactRealInterval,
    policy: &CurveContext,
) -> Option<ExactRealInterval> {
    let lower_sign = compare_reals(&interval.lower, &Real::zero(), policy)?;
    let upper_sign = compare_reals(&interval.upper, &Real::zero(), policy)?;
    if lower_sign != Ordering::Greater && upper_sign != Ordering::Less {
        return None;
    }
    let mut endpoints = [
        (Real::one() / &interval.lower).ok()?,
        (Real::one() / &interval.upper).ok()?,
    ];
    sort_reals(&mut endpoints, policy)?;
    Some(ExactRealInterval {
        lower: endpoints[0].clone(),
        upper: endpoints[1].clone(),
    })
}

fn multiply_intervals(
    left: &ExactRealInterval,
    right: &ExactRealInterval,
    policy: &CurveContext,
) -> Option<ExactRealInterval> {
    let mut products = [
        &left.lower * &right.lower,
        &left.lower * &right.upper,
        &left.upper * &right.lower,
        &left.upper * &right.upper,
    ];
    sort_reals(&mut products, policy)?;
    Some(ExactRealInterval {
        lower: products[0].clone(),
        upper: products[3].clone(),
    })
}

fn sort_reals(values: &mut [Real], policy: &CurveContext) -> Option<()> {
    for index in 1..values.len() {
        let mut cursor = index;
        while cursor > 0 {
            if compare_reals(&values[cursor], &values[cursor - 1], policy)? != Ordering::Less {
                break;
            }
            values.swap(cursor, cursor - 1);
            cursor -= 1;
        }
    }
    Some(())
}

pub(crate) fn exact_contact_point_evidence(
    curve: &RationalBezier2,
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<Option<RationalBezierIntersectionPointEvidence2>> {
    match parameter {
        BezierParameter2::Exact(parameter) => {
            Ok(match curve.point_at_classified(parameter, policy) {
                Classification::Decided(point) => {
                    Some(RationalBezierIntersectionPointEvidence2::Exact(point))
                }
                Classification::Uncertain(_) => None,
            })
        }
        BezierParameter2::Algebraic(parameter) => {
            let image = curve.point_at_algebraic_parameter(parameter, policy)?;
            Ok(match image.status() {
                crate::BezierAlgebraicImageStatus::Transformed
                | crate::BezierAlgebraicImageStatus::RetainedRationalExpression => {
                    Some(RationalBezierIntersectionPointEvidence2::Algebraic(image))
                }
                crate::BezierAlgebraicImageStatus::XImageFailed
                | crate::BezierAlgebraicImageStatus::YImageFailed => {
                    Some(RationalBezierIntersectionPointEvidence2::Algebraic(
                        // The coordinate-image package is deliberately bounded.
                        // Preserve the exact curve/selected-parameter expression
                        // when one coordinate resultant exceeds that budget;
                        // consumers resolve or refine it only when a predicate
                        // actually needs Cartesian coordinates.
                        RationalBezierAlgebraicPointImage2::from_parametric_source(
                            curve.clone(),
                            parameter.clone(),
                            policy,
                        ),
                    ))
                }
                crate::BezierAlgebraicImageStatus::InvalidParameterEvidence => None,
            })
        }
    }
}

pub(crate) fn rational_parameter_image_matches(
    source: &BezierParameter2,
    target: &BezierParameter2,
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<bool>> {
    match source {
        BezierParameter2::Exact(source) => {
            let denominator = evaluate_power_polynomial(denominator, source);
            match real_sign(&denominator, policy) {
                Some(RealSign::Zero) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Some(RealSign::Positive | RealSign::Negative) => {}
                None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
            }
            let image = (evaluate_power_polynomial(numerator, source) / denominator)?;
            BezierParameter2::Exact(image).same_value(target, policy)
        }
        BezierParameter2::Algebraic(source) => {
            let candidate = match conic_parameter_candidate(
                source.polynomial().coefficients(),
                &(numerator.to_vec(), denominator.to_vec()),
                policy,
            )? {
                Classification::Decided(candidate) => candidate,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            {
                let target_interval = match target.known_interval(policy)? {
                    Classification::Decided(interval) => interval,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                };
                let evidence = candidate.map.transform_in_interval(
                    &parameter_representation(source, policy),
                    &AlgebraicPolynomialValueInterval {
                        lower: target_interval.start().clone(),
                        upper: target_interval.end().clone(),
                    },
                );
                if evidence.status == AlgebraicRootRationalImageStatus::ImageIntervalDisjoint {
                    return Ok(Classification::Decided(false));
                }
                if evidence.status != AlgebraicRootRationalImageStatus::Transformed {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                }
                let Some(image) = evidence.representation.as_ref() else {
                    return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
                };
                Ok(algebraic_coordinates_equal(
                    image,
                    &parameter_root_representation(target, policy),
                    policy,
                )
                .map_or(
                    Classification::Uncertain(UncertaintyReason::Predicate),
                    Classification::Decided,
                ))
            }
        }
    }
}

/// Operation-scoped exact rational map with policy-isolated algebraic proof caches.
pub(crate) struct RationalParameterImageMap2 {
    coefficients: (Vec<Real>, Vec<Real>),
    candidates: Vec<(Vec<Real>, ConicParameterCandidate2)>,
    policy: CurveContext,
}

impl RationalParameterImageMap2 {
    pub(crate) fn new(numerator: Vec<Real>, denominator: Vec<Real>, policy: &CurveContext) -> Self {
        Self {
            coefficients: (numerator, denominator),
            candidates: Vec::new(),
            policy: *policy,
        }
    }

    pub(crate) fn image(
        &mut self,
        source: &BezierParameter2,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        let strict_policy = self.policy.strict_counterpart();
        if let Some(source) = source.as_exact() {
            let strict = exact_rational_parameter_image(
                source,
                &self.coefficients.0,
                &self.coefficients.1,
                true,
                &strict_policy,
            )?;
            if strict.is_decided() || !self.policy.permits_approximate_512() {
                return Ok(strict);
            }
            return exact_rational_parameter_image(
                source,
                &self.coefficients.0,
                &self.coefficients.1,
                true,
                &self.policy,
            );
        }
        let BezierParameter2::Algebraic(source) = source else {
            return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
        };
        let source_polynomial = source.polynomial().coefficients();
        let candidate_index = if let Some(index) = self
            .candidates
            .iter()
            .position(|(polynomial, _)| polynomial == source_polynomial)
        {
            index
        } else {
            let candidate = match conic_parameter_candidate(
                source_polynomial,
                &self.coefficients,
                &strict_policy,
            )? {
                Classification::Decided(candidate) => candidate,
                Classification::Uncertain(reason) if !self.policy.permits_approximate_512() => {
                    return Ok(Classification::Uncertain(reason));
                }
                Classification::Uncertain(_) => match conic_parameter_candidate(
                    source_polynomial,
                    &self.coefficients,
                    &self.policy,
                )? {
                    Classification::Decided(candidate) => candidate,
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                },
            };
            self.candidates
                .push((source_polynomial.to_vec(), candidate));
            self.candidates.len() - 1
        };
        let strict = real_coefficient_rational_image_parameter(
            &BezierParameter2::Algebraic(source.clone()),
            &self.candidates[candidate_index].1,
            &strict_policy,
        )?;
        if strict.is_decided() || !self.policy.permits_approximate_512() {
            return Ok(strict);
        }
        real_coefficient_rational_image_parameter(
            &BezierParameter2::Algebraic(source.clone()),
            &self.candidates[candidate_index].1,
            &self.policy,
        )
    }

    /// Maps to any finite affine parameter. The caller owns the projective
    /// cell and placement checks; unlike [`Self::image`], this does not clip
    /// the image to the authored unit segment.
    pub(crate) fn image_unbounded(
        &self,
        source: &BezierParameter2,
    ) -> CurveResult<Classification<Option<BezierParameter2>>> {
        let strict_policy = self.policy.strict_counterpart();
        let strict = rational_parameter_image_unbounded(
            source,
            &self.coefficients.0,
            &self.coefficients.1,
            &strict_policy,
        )?;
        if strict.is_decided() || !self.policy.permits_approximate_512() {
            return Ok(strict);
        }
        rational_parameter_image_unbounded(
            source,
            &self.coefficients.0,
            &self.coefficients.1,
            &self.policy,
        )
    }
}

fn exact_rational_parameter_image(
    source: &Real,
    numerator: &[Real],
    denominator: &[Real],
    unit_domain: bool,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let denominator_value = evaluate_power_polynomial(denominator, source);
    match real_sign(&denominator_value, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {}
        Some(RealSign::Zero) => return Ok(Classification::Decided(None)),
        None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
    }
    let value = (evaluate_power_polynomial(numerator, source) / denominator_value)?;
    if !unit_domain {
        return Ok(Classification::Decided(Some(BezierParameter2::Exact(
            value,
        ))));
    }
    match BezierParameter2::exact(value, policy) {
        Ok(Classification::Decided(parameter)) => Ok(Classification::Decided(Some(parameter))),
        Err(CurveError::InvalidBezierParameter) => Ok(Classification::Decided(None)),
        Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
        Err(error) => Err(error),
    }
}

fn rational_parameter_image_unbounded(
    source: &BezierParameter2,
    numerator: &[Real],
    denominator: &[Real],
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let BezierParameter2::Algebraic(source) = source else {
        return exact_rational_parameter_image(
            source
                .as_exact()
                .expect("a non-algebraic Bezier parameter is exact"),
            numerator,
            denominator,
            false,
            policy,
        );
    };
    let map = AlgebraicRootRationalMap::new(
        source.polynomial().coefficients(),
        numerator,
        denominator,
        policy.predicate_policy(),
    );
    let evidence = map.transform(&parameter_representation(source, policy));
    if evidence.status == AlgebraicRootRationalImageStatus::CertifiedZeroDenominator {
        return Ok(Classification::Decided(None));
    }
    if evidence.status != AlgebraicRootRationalImageStatus::Transformed {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some(representation) = evidence.representation.as_ref() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    BezierParameter2::from_algebraic_root_representation_unbounded(representation, policy)
        .map(|parameter| parameter.map(Some))
}

fn conic_parameter_candidate(
    source_polynomial: &[Real],
    candidate: &(Vec<Real>, Vec<Real>),
    policy: &CurveContext,
) -> CurveResult<Classification<ConicParameterCandidate2>> {
    let mut numerator = match trim_power_polynomial(candidate.0.clone(), policy) {
        Classification::Decided(numerator) => numerator,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut denominator = match trim_power_polynomial(candidate.1.clone(), policy) {
        Classification::Decided(denominator) => denominator,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    if numerator
        .iter()
        .chain(&denominator)
        .any(|coefficient| coefficient.exact_rational_ref().is_none())
        && let Some(scale) = numerator
            .iter()
            .chain(&denominator)
            .find(|coefficient| is_zero(coefficient, policy) == Some(false))
            .cloned()
    {
        let normalized_numerator = numerator
            .iter()
            .map(|coefficient| coefficient.clone() / scale.clone())
            .collect::<Result<Vec<_>, _>>()?;
        let normalized_denominator = denominator
            .iter()
            .map(|coefficient| coefficient.clone() / scale.clone())
            .collect::<Result<Vec<_>, _>>()?;
        if normalized_numerator
            .iter()
            .chain(&normalized_denominator)
            .all(|coefficient| coefficient.exact_rational_ref().is_some())
        {
            numerator = normalized_numerator;
            denominator = normalized_denominator;
        }
    }
    {
        Ok(Classification::Decided(ConicParameterCandidate2 {
            map: AlgebraicRootRationalMap::new(
                source_polynomial,
                &numerator,
                &denominator,
                policy.predicate_policy(),
            ),
            numerator,
            denominator,
            image_polynomial: OnceLock::new(),
            image_parameters: OnceLock::new(),
            quotient_matrices: OnceLock::new(),
            quotient_power: OnceLock::new(),
        }))
    }
}

fn rational_image_parameter(
    source: &AlgebraicRootRepresentation,
    candidate: &ConicParameterCandidate2,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameter2>>> {
    let zero = Real::zero();
    let one = Real::one();
    let evidence = candidate.map.transform_in_interval(
        source,
        &AlgebraicPolynomialValueInterval {
            lower: zero.clone(),
            upper: one.clone(),
        },
    );
    #[cfg(feature = "dispatch-trace")]
    hyperreal::dispatch_trace::record(
        "hypercurve",
        "conic-rational-image",
        match evidence.status {
            AlgebraicRootRationalImageStatus::Transformed => "transformed",
            AlgebraicRootRationalImageStatus::ImageIntervalDisjoint => "interval-disjoint",
            AlgebraicRootRationalImageStatus::InvalidEvidence => "invalid-evidence",
            AlgebraicRootRationalImageStatus::InvalidNumeratorPolynomial => "invalid-numerator",
            AlgebraicRootRationalImageStatus::InvalidDenominatorPolynomial => "invalid-denominator",
            AlgebraicRootRationalImageStatus::CertifiedZeroDenominator => "zero-denominator",
            AlgebraicRootRationalImageStatus::DenominatorMayContainZero => {
                "denominator-may-contain-zero"
            }
            AlgebraicRootRationalImageStatus::NumeratorImageFailed => "numerator-image-failed",
            AlgebraicRootRationalImageStatus::DenominatorImageFailed => "denominator-image-failed",
            AlgebraicRootRationalImageStatus::QuotientConstructionFailed => {
                "quotient-construction-failed"
            }
            AlgebraicRootRationalImageStatus::InvalidTransformedEvidence => {
                "invalid-transformed-evidence"
            }
            AlgebraicRootRationalImageStatus::Undecided => "undecided",
        },
    );
    if evidence.status == AlgebraicRootRationalImageStatus::ImageIntervalDisjoint {
        return Ok(Classification::Decided(None));
    }
    if evidence.status != AlgebraicRootRationalImageStatus::Transformed {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    let Some(representation) = evidence.representation.as_ref() else {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    };
    let lower_zero = compare_reals(&representation.interval.lower, &zero, policy);
    let upper_zero = compare_reals(&representation.interval.upper, &zero, policy);
    let lower_one = compare_reals(&representation.interval.lower, &one, policy);
    let upper_one = compare_reals(&representation.interval.upper, &one, policy);
    let (Some(lower_zero), Some(upper_zero), Some(lower_one), Some(upper_one)) =
        (lower_zero, upper_zero, lower_one, upper_one)
    else {
        return Ok(Classification::Uncertain(UncertaintyReason::Ordering));
    };
    if upper_zero == Ordering::Less || lower_one == Ordering::Greater {
        return Ok(Classification::Decided(None));
    }
    if lower_zero == Ordering::Less || upper_one == Ordering::Greater {
        return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
    }
    match BezierParameter2::from_algebraic_root_representation(representation, policy) {
        Ok(Classification::Decided(parameter)) => Ok(Classification::Decided(Some(parameter))),
        Ok(Classification::Uncertain(reason)) => Ok(Classification::Uncertain(reason)),
        Err(CurveError::InvalidBezierParameter) => {
            Ok(Classification::Uncertain(UncertaintyReason::Predicate))
        }
        Err(error) => Err(error),
    }
}

fn reverse_rational_intersection_contacts(
    contacts: RationalBezierIntersectionContacts2,
) -> RationalBezierIntersectionContacts2 {
    match contacts {
        RationalBezierIntersectionContacts2::NoIntersection => {
            RationalBezierIntersectionContacts2::NoIntersection
        }
        RationalBezierIntersectionContacts2::Contacts(contacts) => {
            RationalBezierIntersectionContacts2::Contacts(
                contacts
                    .iter()
                    .map(|contact| RationalBezierIntersectionContact2 {
                        first_parameter: contact.second_parameter.clone(),
                        second_parameter: contact.first_parameter.clone(),
                        point: contact.point.clone(),
                        certified_transverse: contact.certified_transverse,
                        tangent_cross_sign: contact.tangent_cross_sign.map(negated_real_sign),
                    })
                    .collect::<Vec<_>>()
                    .into(),
            )
        }
        RationalBezierIntersectionContacts2::Overlap(overlap) => {
            RationalBezierIntersectionContacts2::Overlap(RationalBezierIntersectionOverlap2 {
                first_range: overlap.second_range,
                second_range: overlap.first_range,
                orientation: overlap.orientation,
                endpoint_inclusion: overlap.endpoint_inclusion,
            })
        }
        RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap } => {
            RationalBezierIntersectionContacts2::ContactsAndOverlap {
                contacts: contacts
                    .iter()
                    .map(|contact| RationalBezierIntersectionContact2 {
                        first_parameter: contact.second_parameter.clone(),
                        second_parameter: contact.first_parameter.clone(),
                        point: contact.point.clone(),
                        certified_transverse: contact.certified_transverse,
                        tangent_cross_sign: contact.tangent_cross_sign.map(negated_real_sign),
                    })
                    .collect::<Vec<_>>()
                    .into(),
                overlap: RationalBezierIntersectionOverlap2 {
                    first_range: overlap.second_range,
                    second_range: overlap.first_range,
                    orientation: overlap.orientation,
                    endpoint_inclusion: overlap.endpoint_inclusion,
                },
            }
        }
        RationalBezierIntersectionContacts2::Incomplete {
            contacts,
            candidates,
        } => RationalBezierIntersectionContacts2::Incomplete {
            contacts: contacts
                .iter()
                .map(|contact| RationalBezierIntersectionContact2 {
                    first_parameter: contact.second_parameter.clone(),
                    second_parameter: contact.first_parameter.clone(),
                    point: contact.point.clone(),
                    certified_transverse: contact.certified_transverse,
                    tangent_cross_sign: contact.tangent_cross_sign.map(negated_real_sign),
                })
                .collect::<Vec<_>>()
                .into(),
            candidates: match candidates {
                RationalBezierIntersectionCandidates2::Candidates {
                    first_parameters,
                    second_parameters,
                } => RationalBezierIntersectionCandidates2::Candidates {
                    first_parameters: second_parameters,
                    second_parameters: first_parameters,
                },
                candidates => candidates,
            },
        },
        RationalBezierIntersectionContacts2::DegenerateResultant => {
            RationalBezierIntersectionContacts2::DegenerateResultant
        }
    }
}

const fn negated_real_sign(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Positive => RealSign::Negative,
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
    }
}

fn intersection_candidates_from_contacts(
    contacts: &RationalBezierIntersectionContacts2,
) -> RationalBezierIntersectionCandidates2 {
    match contacts {
        RationalBezierIntersectionContacts2::NoIntersection => {
            RationalBezierIntersectionCandidates2::NoIntersection
        }
        RationalBezierIntersectionContacts2::Contacts(contacts) => {
            RationalBezierIntersectionCandidates2::Candidates {
                first_parameters: contacts
                    .iter()
                    .map(|contact| contact.first_parameter.clone())
                    .collect(),
                second_parameters: contacts
                    .iter()
                    .map(|contact| contact.second_parameter.clone())
                    .collect(),
            }
        }
        RationalBezierIntersectionContacts2::Incomplete { candidates, .. } => candidates.clone(),
        RationalBezierIntersectionContacts2::Overlap(_)
        | RationalBezierIntersectionContacts2::ContactsAndOverlap { .. }
        | RationalBezierIntersectionContacts2::DegenerateResultant => {
            RationalBezierIntersectionCandidates2::DegenerateResultant
        }
    }
}

fn trim_power_polynomial(
    mut coefficients: Vec<Real>,
    policy: &CurveContext,
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
    policy: &CurveContext,
) -> Classification<Option<BezierParameter2>> {
    if point == curve.start() {
        return Classification::Decided(Some(BezierParameter2::Exact(Real::zero())));
    }
    if point == curve.end() {
        return Classification::Decided(Some(BezierParameter2::Exact(Real::one())));
    }
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
    policy: &CurveContext,
) -> Option<bool> {
    match algebraic_coordinates_equal(&first.x, &second.x, policy) {
        Some(false) => return Some(false),
        Some(true) => {}
        None => return None,
    }
    algebraic_coordinates_equal(&first.y, &second.y, policy)
}

fn candidate_point_representations_disjoint(
    first: &CandidatePointReplay,
    second: &CandidatePointReplay,
    policy: &CurveContext,
) -> bool {
    represented_root_intervals_disjoint(&first.x, &second.x, policy)
        || represented_root_intervals_disjoint(&first.y, &second.y, policy)
}

fn represented_root_intervals_disjoint(
    first: &AlgebraicRootRepresentation,
    second: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> bool {
    compare_reals(&first.interval.upper, &second.interval.lower, policy)
        .is_some_and(|ordering| ordering.is_lt())
        || compare_reals(&second.interval.upper, &first.interval.lower, policy)
            .is_some_and(|ordering| ordering.is_lt())
}

fn candidate_parameter_is_simple_root(
    parameter: &BezierParameter2,
    policy: &CurveContext,
) -> CurveResult<bool> {
    let BezierParameter2::Algebraic(algebraic) = parameter else {
        return Ok(false);
    };
    let classifications = algebraic
        .polynomial()
        .simple_root_classifications(std::slice::from_ref(parameter), policy)?;
    Ok(matches!(
        classifications.first(),
        Some(Classification::Decided(true))
    ))
}

fn algebraic_coordinates_equal(
    first: &AlgebraicRootRepresentation,
    second: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<bool> {
    if let (Some(first), Some(second)) = (
        first.exact_rational_witness(),
        second.exact_rational_witness(),
    ) {
        return compare_reals(first, second, policy).map(|ordering| ordering.is_eq());
    }
    compare_algebraic_coordinates(first, second, policy)
}

fn compare_algebraic_coordinates(
    first: &AlgebraicRootRepresentation,
    second: &AlgebraicRootRepresentation,
    policy: &CurveContext,
) -> Option<bool> {
    compare_algebraic_representations_with_policy(first, second, policy)
        .map(|ordering| ordering.is_eq())
}

fn resultant_parameter_polynomial(
    evidence: CurveIntersectionResultantReport,
    policy: &CurveContext,
) -> CurveResult<Classification<Option<BezierParameterPolynomial>>> {
    match evidence.status {
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
    if evidence
        .resultant_coefficients
        .iter()
        .all(|coefficient| is_zero(coefficient, policy) == Some(true))
    {
        return Ok(Classification::Decided(None));
    }
    if evidence
        .resultant_coefficients
        .iter()
        .all(|coefficient| is_zero(coefficient, policy) != Some(false))
    {
        return Ok(Classification::Uncertain(UncertaintyReason::RealSign));
    }
    // A resultant is only a projection carrier: its multiplicities do not
    // encode geometric multiplicities, which are established by bivariate
    // replay. Remove repeated factors exactly before root isolation so squared
    // eliminants do not inflate Sturm construction, especially after mapping
    // an unbounded incident ray to a compact chart. STRICT owns this algebraic
    // reduction under both public policies; retaining the original polynomial
    // remains exact when the GCD cannot be certified.
    let coefficients = hypersolve::square_free_part(
        evidence.resultant_coefficients.clone(),
        hypersolve::PredicatePolicy::STRICT,
    )
    .unwrap_or(evidence.resultant_coefficients);
    Ok(
        match BezierParameterPolynomial::try_new_power_basis(coefficients, policy)? {
            Classification::Decided(polynomial) => Classification::Decided(Some(polynomial)),
            Classification::Uncertain(reason) => Classification::Uncertain(reason),
        },
    )
}

pub(crate) fn resultant_parameter_projection(
    evidence: CurveIntersectionResultantReport,
    policy: &CurveContext,
) -> CurveResult<Classification<ResultantParameterProjection>> {
    let polynomial = match resultant_parameter_polynomial(evidence, policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => {
            return Ok(Classification::Decided(
                ResultantParameterProjection::Degenerate,
            ));
        }
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

/// Projects one resultant onto the authored unit span plus its open regular
/// endpoint ray. The caller supplies the first source pole or speed-zero
/// barrier, so every retained exterior root stays in the same analytic cell.
pub(crate) fn resultant_parameter_projection_with_incident_ray(
    evidence: CurveIntersectionResultantReport,
    anchor: &Real,
    direction: BezierParameterRayDirection2,
    barrier: Option<&BezierParameter2>,
    policy: &CurveContext,
) -> CurveResult<Classification<ResultantParameterProjection>> {
    let polynomial = match resultant_parameter_polynomial(evidence, policy)? {
        Classification::Decided(Some(polynomial)) => polynomial,
        Classification::Decided(None) => {
            return Ok(Classification::Decided(
                ResultantParameterProjection::Degenerate,
            ));
        }
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let mut parameters = match polynomial.isolate_unit_interval_roots(policy)? {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    let exterior = match polynomial.isolate_incident_ray_roots(anchor, direction, policy)? {
        Classification::Decided(parameters) => parameters,
        Classification::Uncertain(reason) => {
            return Ok(Classification::Uncertain(reason));
        }
    };
    for parameter in exterior {
        if let Some(barrier) = barrier {
            let ordering = match parameter.cmp_by_refinement(barrier, policy)? {
                Classification::Decided(ordering) => ordering,
                Classification::Uncertain(reason) => {
                    return Ok(Classification::Uncertain(reason));
                }
            };
            let before_barrier = match direction {
                BezierParameterRayDirection2::Decreasing => ordering == Ordering::Greater,
                BezierParameterRayDirection2::Increasing => ordering == Ordering::Less,
            };
            if !before_barrier {
                continue;
            }
        }
        parameters.push(parameter);
    }
    Ok(Classification::Decided(if parameters.is_empty() {
        ResultantParameterProjection::Empty
    } else {
        ResultantParameterProjection::Parameters(parameters)
    }))
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

fn evaluate_power_polynomial_endpoint_derivatives(
    coefficients: &[Real],
    at_end: bool,
    max_order: usize,
) -> Option<Vec<Real>> {
    let value_count = max_order.checked_add(1)?;
    let mut derivatives = vec![Real::zero(); value_count];
    if !at_end {
        let mut factorial = 1_u64;
        for (order, derivative) in derivatives.iter_mut().enumerate() {
            if order > 1 {
                factorial = factorial.checked_mul(u64::try_from(order).ok()?)?;
            }
            if let Some(coefficient) = coefficients.get(order) {
                *derivative = if factorial == 1 {
                    coefficient.clone()
                } else {
                    Real::from(factorial) * coefficient
                };
            }
        }
        return Some(derivatives);
    }

    for coefficient in coefficients.iter().rev() {
        for order in (1..=max_order).rev() {
            let scale = Real::from(u64::try_from(order).ok()?);
            derivatives[order] = &derivatives[order] + &scale * &derivatives[order - 1];
        }
        derivatives[0] = &derivatives[0] + coefficient;
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

fn exact_binomial(n: usize, k: usize) -> Option<Real> {
    if let Some(value) = checked_binomial(n, k) {
        return Some(Real::from(value));
    }
    let k = k.min(n.checked_sub(k)?);
    let mut result = Real::one();
    for index in 0..k {
        result *= Real::from(u64::try_from(n.checked_sub(index)?).ok()?);
        result = (result / Real::from(u64::try_from(index.checked_add(1)?).ok()?)).ok()?;
    }
    Some(result)
}

fn exact_binomial_product(
    first_n: usize,
    first_k: usize,
    second_n: usize,
    second_k: usize,
) -> Option<Real> {
    if let (Some(first), Some(second)) = (
        checked_binomial(first_n, first_k),
        checked_binomial(second_n, second_k),
    ) && let Some(product) = first.checked_mul(second)
    {
        return Some(Real::from(product));
    }
    Some(exact_binomial(first_n, first_k)? * exact_binomial(second_n, second_k)?)
}

fn exact_quadratic_homogeneous_reduction(
    source: &[HomogeneousPoint2],
    policy: &CurveContext,
) -> Classification<Option<[HomogeneousPoint2; 3]>> {
    let reduced = match exact_homogeneous_degree_reduction(source, 3, policy) {
        Classification::Decided(Some(reduced)) => reduced,
        Classification::Decided(None) => return Classification::Decided(None),
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let Ok(frame) = <Vec<HomogeneousPoint2> as TryInto<[HomogeneousPoint2; 3]>>::try_into(reduced)
    else {
        return Classification::Decided(None);
    };
    Classification::Decided(Some(frame))
}

fn exact_linear_homogeneous_reduction(
    source: &[HomogeneousPoint2],
    policy: &CurveContext,
) -> Classification<Option<[HomogeneousPoint2; 2]>> {
    let reduced = match exact_homogeneous_degree_reduction(source, 2, policy) {
        Classification::Decided(Some(reduced)) => reduced,
        Classification::Decided(None) => return Classification::Decided(None),
        Classification::Uncertain(reason) => return Classification::Uncertain(reason),
    };
    let Ok(frame) = <Vec<HomogeneousPoint2> as TryInto<[HomogeneousPoint2; 2]>>::try_into(reduced)
    else {
        return Classification::Decided(None);
    };
    Classification::Decided(Some(frame))
}

fn exact_homogeneous_degree_reduction(
    source: &[HomogeneousPoint2],
    target_control_count: usize,
    policy: &CurveContext,
) -> Classification<Option<Vec<HomogeneousPoint2>>> {
    if target_control_count < 2 || source.len() < target_control_count {
        return Classification::Decided(None);
    }
    let mut current = source.to_vec();
    while current.len() > target_control_count {
        let degree = current.len() - 1;
        let Ok(degree_u64) = u64::try_from(degree) else {
            return Classification::Uncertain(UncertaintyReason::Unsupported);
        };
        let degree_real = Real::from(degree_u64);
        let mut reduced = Vec::with_capacity(degree);
        reduced.push(current[0].clone());
        for index in 1..degree {
            let Ok(index_u64) = u64::try_from(index) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let Ok(remaining_u64) = u64::try_from(degree - index) else {
                return Classification::Uncertain(UncertaintyReason::Unsupported);
            };
            let index_real = Real::from(index_u64);
            let remaining = Real::from(remaining_u64);
            let previous = &reduced[index - 1];
            let candidate = HomogeneousPoint2 {
                x: match (&degree_real * &current[index].x - &index_real * &previous.x) / &remaining
                {
                    Ok(value) => value,
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                },
                y: match (&degree_real * &current[index].y - &index_real * &previous.y) / &remaining
                {
                    Ok(value) => value,
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                },
                weight: match (&degree_real * &current[index].weight
                    - &index_real * &previous.weight)
                    / &remaining
                {
                    Ok(value) => value,
                    Err(_) => {
                        return Classification::Uncertain(UncertaintyReason::Unsupported);
                    }
                },
            };
            reduced.push(candidate);
        }
        let expected_end = reduced
            .last()
            .expect("positive-degree inverse elevation has an endpoint");
        for residual in [
            &expected_end.x - &current[degree].x,
            &expected_end.y - &current[degree].y,
            &expected_end.weight - &current[degree].weight,
        ] {
            match is_zero(&residual, policy) {
                Some(true) => {}
                Some(false) => return Classification::Decided(None),
                None => return Classification::Uncertain(UncertaintyReason::RealSign),
            }
        }
        current = reduced;
    }
    Classification::Decided(Some(current))
}

impl HomogeneousPoint2 {
    fn scaled(&self, scale: &Real) -> Self {
        Self {
            x: &self.x * scale,
            y: &self.y * scale,
            weight: &self.weight * scale,
        }
    }

    fn add_scaled(&mut self, other: &Self, scale: &Real) {
        self.x = &self.x + &other.x * scale;
        self.y = &self.y + &other.y * scale;
        self.weight = &self.weight + &other.weight * scale;
    }

    fn midpoint(&self, other: &Self, half: &Real) -> Self {
        Self {
            x: Real::dot2_refs([&self.x, &other.x], [half, half]),
            y: Real::dot2_refs([&self.y, &other.y], [half, half]),
            weight: Real::dot2_refs([&self.weight, &other.weight], [half, half]),
        }
    }

    fn lerp(&self, other: &Self, parameter: &Real) -> Self {
        let one_minus = Real::one() - parameter;
        self.lerp_with_complement(other, parameter, &one_minus)
    }

    fn lerp_with_complement(
        &self,
        other: &Self,
        parameter: &Real,
        one_minus_parameter: &Real,
    ) -> Self {
        Self {
            x: Real::dot2_refs([&self.x, &other.x], [one_minus_parameter, parameter]),
            y: Real::dot2_refs([&self.y, &other.y], [one_minus_parameter, parameter]),
            weight: Real::dot2_refs(
                [&self.weight, &other.weight],
                [one_minus_parameter, parameter],
            ),
        }
    }
}

fn split_homogeneous_controls(
    source: &[HomogeneousPoint2],
    parameter: &Real,
) -> (Vec<HomogeneousPoint2>, Vec<HomogeneousPoint2>) {
    let mut level = source.to_vec();
    let mut left = Vec::with_capacity(level.len());
    let mut right = Vec::with_capacity(level.len());
    left.push(level[0].clone());
    right.push(
        level
            .last()
            .expect("positive-degree homogeneous curve has controls")
            .clone(),
    );
    for next_len in (1..level.len()).rev() {
        for index in 0..next_len {
            level[index] = level[index].lerp(&level[index + 1], parameter);
        }
        left.push(level[0].clone());
        right.push(level[next_len - 1].clone());
    }
    right.reverse();
    (left, right)
}

fn affine_homogeneous_subcurve_controls(
    source: &[HomogeneousPoint2],
    start: &Real,
    end: &Real,
    start_at_one: bool,
    policy: &CurveContext,
) -> CurveResult<Vec<HomogeneousPoint2>> {
    if compare_reals(start, end, policy) == Some(Ordering::Equal) {
        let (left, _) = split_homogeneous_controls(source, start);
        let point = left
            .last()
            .expect("a homogeneous evaluation has one terminal point")
            .clone();
        return Ok(vec![point; source.len()]);
    }
    if !start_at_one {
        let (_, right) = split_homogeneous_controls(source, start);
        let local_end = ((end - start) / (Real::one() - start))?;
        let (range, _) = split_homogeneous_controls(&right, &local_end);
        return Ok(range);
    }
    let (left, _) = split_homogeneous_controls(source, end);
    let local_start = (start / end)?;
    let (_, range) = split_homogeneous_controls(&left, &local_start);
    Ok(range)
}

fn homogeneous_controls_common_weight_sign(
    controls: &[HomogeneousPoint2],
    policy: &CurveContext,
) -> Classification<Option<RealSign>> {
    let mut common = None;
    for control in controls {
        let Some(sign) = real_sign(&control.weight, policy) else {
            return Classification::Uncertain(UncertaintyReason::RealSign);
        };
        match (common, sign) {
            (_, RealSign::Zero) => return Classification::Decided(None),
            (None, sign) => common = Some(sign),
            (Some(expected), sign) if sign == expected => {}
            (Some(_), _) => return Classification::Decided(None),
        }
    }
    Classification::Decided(common)
}

fn elevate_homogeneous_controls_once(
    source: &[HomogeneousPoint2],
) -> CurveResult<Vec<HomogeneousPoint2>> {
    let target_degree = source.len();
    let denominator =
        Real::from(u64::try_from(target_degree).map_err(|_| CurveError::InvalidDegreeElevation)?);
    let mut elevated = Vec::with_capacity(source.len() + 1);
    elevated.push(source[0].clone());
    for index in 1..target_degree {
        let numerator =
            Real::from(u64::try_from(index).map_err(|_| CurveError::InvalidDegreeElevation)?);
        let alpha = (numerator / &denominator)?;
        elevated.push(source[index].lerp(&source[index - 1], &alpha));
    }
    elevated.push(
        source
            .last()
            .expect("positive-degree homogeneous curve has an end")
            .clone(),
    );
    Ok(elevated)
}

fn real_nonnegative_integer_power(base: &Real, mut exponent: usize) -> Real {
    let mut result = Real::one();
    let mut factor = base.clone();
    while exponent != 0 {
        if !exponent.is_multiple_of(2) {
            result *= &factor;
        }
        exponent /= 2;
        if exponent != 0 {
            factor = &factor * &factor;
        }
    }
    result
}

fn project_homogeneous(point: &HomogeneousPoint2, policy: &CurveContext) -> Classification<Point2> {
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
    policy: &CurveContext,
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

    fn exact_f64(value: f64) -> Real {
        Real::try_from(value).expect("finite binary rational")
    }

    #[test]
    fn unit_weight_degree_one_curve_exposes_its_exact_line_parameterization() {
        let start = Point2::new(Real::from(-2_i8), Real::from(3_i8));
        let end = Point2::new(Real::from(5_i8), Real::from(-7_i8));
        let curve = RationalBezier2::try_new(
            vec![start.clone(), end.clone()],
            vec![Real::one(), Real::one()],
        )
        .unwrap();

        assert_eq!(
            curve.exact_linear_parameterization_line(),
            Some(LineSeg2::try_new(start, end).unwrap())
        );
    }

    #[test]
    fn affine_subcurve_elevates_zero_intermediate_weight_without_a_pole() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let two_thirds = (Real::from(2_i8) / Real::from(3_i8)).unwrap();
        let source = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 1),
                Point2::from_values(2, 0),
            ],
            vec![Real::one(), half, Real::one()],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(extended) = source
                .subcurve_between_affine_exact(&Real::zero(), &Real::from(2_i8), &policy)
                .unwrap()
            else {
                panic!("the pole-free affine extension must materialize");
            };
            assert_eq!(extended.degree(), 3);
            assert_eq!(extended.start(), &Point2::from_values(0, 0));
            assert_eq!(
                extended.end(),
                &Point2::new(Real::from(2_i8), -two_thirds.clone())
            );
            assert_eq!(
                extended.common_weight_sign(&policy),
                Classification::Decided(RealSign::Positive)
            );
        }
    }

    #[test]
    fn affine_subcurve_rejects_a_crossed_projective_pole() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let source = RationalBezier2::try_new(
            vec![Point2::from_values(0, 0), Point2::from_values(1, 0)],
            vec![Real::one(), half],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert_eq!(
                source
                    .subcurve_between_affine_exact(&Real::zero(), &Real::from(3_i8), &policy,)
                    .unwrap(),
                Classification::Uncertain(UncertaintyReason::Boundary)
            );
        }
    }

    #[test]
    fn affine_subcurve_materializes_a_finite_rational_parabola_extension() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let quarter = (Real::one() / Real::from(4_i8)).unwrap();
        let three_halves = (Real::from(3_i8) / Real::from(2_i8)).unwrap();
        let source = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::new(half.clone(), Real::zero()),
                Point2::from_values(1, 1),
            ],
            vec![Real::one(), half, quarter],
        )
        .unwrap();
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(extended) = source
                .subcurve_between_affine_exact(&Real::zero(), &three_halves, &policy)
                .unwrap()
            else {
                panic!("the pre-pole rational parabola interval must materialize");
            };
            assert_eq!(extended.end(), &Point2::from_values(3, 9));
        }
    }

    #[test]
    fn deferred_parametric_point_reuses_source_polynomials_for_exact_equality() {
        let policy = CurveContext::STRICT;
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(
            vec![-half, Real::zero(), Real::one()],
            &policy,
        )
        .unwrap() else {
            panic!("quadratic parameter polynomial was not certified");
        };
        let Classification::Decided(interval) =
            BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap()
        else {
            panic!("unit parameter interval was not certified");
        };
        let Classification::Decided(parameter) =
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap()
        else {
            panic!("positive quadratic parameter was not isolated");
        };
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(Real::from(-1_i8), Real::zero()),
                Point2::new(Real::zero(), Real::one()),
                Point2::new(Real::one(), Real::from(2_i8)),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();
        let deferred = RationalBezierAlgebraicPointImage2::from_parametric_source(
            curve.clone(),
            parameter.clone(),
            &policy,
        );
        let resolved = curve
            .point_at_algebraic_parameter(&parameter, &policy)
            .unwrap();

        assert_eq!(
            deferred
                .same_retained_rational_point(&resolved, &policy)
                .unwrap(),
            Some(Classification::Decided(true))
        );
    }

    #[test]
    fn exact_contact_evidence_retains_a_bounded_high_degree_image_failure() {
        let policy = CurveContext::STRICT;
        let mut coefficients = vec![Real::zero(); 42];
        coefficients[0] = (Real::from(-1_i8) / Real::from(2_i8)).unwrap();
        coefficients[41] = Real::one();
        let Classification::Decided(polynomial) =
            BezierParameterPolynomial::try_new_power_basis(coefficients, &policy).unwrap()
        else {
            panic!("degree-41 parameter polynomial was not certified");
        };
        let Classification::Decided(interval) =
            BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap()
        else {
            panic!("degree-41 root interval was not certified");
        };
        let Classification::Decided(parameter) =
            BezierAlgebraicParameter2::try_isolate(polynomial, interval, &policy).unwrap()
        else {
            panic!("positive degree-41 parameter was not isolated");
        };
        let curve = RationalBezier2::try_new(
            vec![
                Point2::from_values(0, 0),
                Point2::from_values(1, 2),
                Point2::from_values(3, -1),
            ],
            vec![Real::one(), Real::from(2_i8), Real::from(3_i8)],
        )
        .unwrap();
        let bounded = curve
            .point_at_algebraic_parameter(&parameter, &policy)
            .unwrap();
        assert!(matches!(
            bounded.status(),
            crate::BezierAlgebraicImageStatus::XImageFailed
                | crate::BezierAlgebraicImageStatus::YImageFailed
        ));

        let evidence = exact_contact_point_evidence(
            &curve,
            &BezierParameter2::Algebraic(parameter.clone()),
            &policy,
        )
        .unwrap()
        .expect("the exact parametric source must survive a bounded image failure");
        let RationalBezierIntersectionPointEvidence2::Algebraic(retained) = evidence else {
            panic!("the high-degree contact must remain algebraic");
        };
        assert_eq!(
            retained.status(),
            crate::BezierAlgebraicImageStatus::RetainedRationalExpression
        );
        assert_eq!(retained.retained_parameter(), Some(&parameter));
    }

    #[test]
    fn certified_rational_interval_sign_products_match_four_corner_enclosure() {
        for first_lower in -3_i64..=3 {
            for first_upper in first_lower..=3 {
                let first = CertifiedRationalInterval {
                    lower: HyperRational::new(first_lower),
                    upper: HyperRational::new(first_upper),
                };
                for second_lower in -3_i64..=3 {
                    for second_upper in second_lower..=3 {
                        let second = CertifiedRationalInterval {
                            lower: HyperRational::new(second_lower),
                            upper: HyperRational::new(second_upper),
                        };
                        let products = [
                            &first.lower * &second.lower,
                            &first.lower * &second.upper,
                            &first.upper * &second.lower,
                            &first.upper * &second.upper,
                        ];
                        let mut expected_lower = products[0].clone();
                        let mut expected_upper = products[0].clone();
                        for product in products.into_iter().skip(1) {
                            if product < expected_lower {
                                expected_lower = product.clone();
                            }
                            if product > expected_upper {
                                expected_upper = product;
                            }
                        }

                        let product = first.multiply(&second);
                        assert_eq!(product.lower, expected_lower);
                        assert_eq!(product.upper, expected_upper);
                        for scalar in -3_i64..=3 {
                            assert_eq!(
                                first.multiply_scalar(&HyperRational::new(scalar)),
                                first.multiply(&CertifiedRationalInterval::point(
                                    HyperRational::new(scalar),
                                ))
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn off_diagonal_self_contacts_share_one_rational_authority() {
        let controls = vec![
            Point2::new(Real::from(9_i8), Real::zero()),
            Point2::new(Real::from(-7_i8), Real::from(3_i8)),
            Point2::new(Real::from(-7_i8), Real::from(-10_i8)),
            Point2::new(Real::from(9_i8), Real::from(9_i8)),
        ];
        let fixtures = [
            (
                vec![Real::one(); 4],
                (Real::one() / Real::from(4_i8)).unwrap(),
                (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
            ),
            (
                vec![
                    Real::one(),
                    Real::from(2_i8),
                    Real::from(4_i8),
                    Real::from(8_i8),
                ],
                (Real::one() / Real::from(7_i8)).unwrap(),
                (Real::from(3_i8) / Real::from(5_i8)).unwrap(),
            ),
        ];

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (weights, expected_first, expected_second) in &fixtures {
                let curve = RationalBezier2::try_new(controls.clone(), weights.clone()).unwrap();
                let RationalBezierIntersectionContacts2::Contacts(contacts) =
                    curve.self_intersection_contacts(&policy).unwrap()
                else {
                    panic!("isolated rational loop contact was not completely replayed");
                };
                assert_eq!(contacts.len(), 1);
                let contact = &contacts[0];
                assert!(matches!(
                    contact
                        .first_parameter()
                        .same_value(&BezierParameter2::Exact(expected_first.clone()), &policy)
                        .unwrap(),
                    Classification::Decided(true)
                ));
                assert!(matches!(
                    contact
                        .second_parameter()
                        .same_value(&BezierParameter2::Exact(expected_second.clone()), &policy)
                        .unwrap(),
                    Classification::Decided(true)
                ));
                assert!(contact.is_certified_transverse());
                assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Negative));
            }

            let algebraic = RationalBezier2::try_new(
                vec![
                    Point2::new(Real::from(3_i8), Real::zero()),
                    Point2::new(Real::from(-5_i8), Real::one()),
                    Point2::new(Real::from(-5_i8), Real::from(-6_i8)),
                    Point2::new(Real::from(3_i8), Real::from(3_i8)),
                ],
                vec![Real::one(); 4],
            )
            .unwrap();
            let RationalBezierIntersectionContacts2::Contacts(contacts) =
                algebraic.self_intersection_contacts(&policy).unwrap()
            else {
                panic!("algebraic-parameter loop contact was not completely replayed");
            };
            assert_eq!(contacts.len(), 1);
            assert!(matches!(
                contacts[0].first_parameter(),
                BezierParameter2::Algebraic(_)
            ));
            assert!(matches!(
                contacts[0].second_parameter(),
                BezierParameter2::Algebraic(_)
            ));
            assert_eq!(contacts[0].tangent_cross_sign(), Some(RealSign::Negative));
        }
    }

    #[test]
    fn retained_noninjective_subranges_replay_cross_branch_contacts() {
        let controls = vec![
            Point2::new(Real::from(9_i8), Real::zero()),
            Point2::new(Real::from(-7_i8), Real::from(3_i8)),
            Point2::new(Real::from(-7_i8), Real::from(-10_i8)),
            Point2::new(Real::from(9_i8), Real::from(9_i8)),
        ];
        let ratio = |numerator: i8, denominator: i8| {
            (Real::from(numerator) / Real::from(denominator)).unwrap()
        };

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for weights in [
                vec![Real::one(); 4],
                vec![
                    Real::one(),
                    Real::from(2_i8),
                    Real::from(4_i8),
                    Real::from(8_i8),
                ],
            ] {
                let curve = RationalBezier2::try_new(controls.clone(), weights).unwrap();
                let Classification::Decided((left, _)) =
                    curve.split_at_exact(&ratio(49, 100), &policy).unwrap()
                else {
                    panic!("retained lower split was not decided");
                };
                let Classification::Decided((_, right)) =
                    curve.split_at_exact(&ratio(51, 100), &policy).unwrap()
                else {
                    panic!("retained upper split was not decided");
                };
                let RationalBezierIntersectionContacts2::Contacts(contacts) =
                    left.intersection_contacts(&right, &policy).unwrap()
                else {
                    panic!("disjoint retained branches did not replay isolated contacts");
                };
                assert_eq!(contacts.len(), 1);
                assert!(contacts[0].is_certified_transverse());
            }

            let curve = RationalBezier2::try_new(controls.clone(), vec![Real::one(); 4]).unwrap();
            let Classification::Decided((left, right)) =
                curve.split_at_exact(&ratio(1, 2), &policy).unwrap()
            else {
                panic!("retained midpoint split was not decided");
            };
            let RationalBezierIntersectionContacts2::Contacts(contacts) =
                left.intersection_contacts(&right, &policy).unwrap()
            else {
                panic!("touching retained branches did not replay all point contacts");
            };
            assert_eq!(contacts.len(), 2, "crossing plus shared split endpoint");

            let Classification::Decided(middle) = curve
                .subcurve_between_exact(&ratio(1, 10), &ratio(9, 10), &policy)
                .unwrap()
            else {
                panic!("retained middle subrange was not decided");
            };
            let RationalBezierIntersectionContacts2::ContactsAndOverlap { contacts, overlap } =
                curve.intersection_contacts(&middle, &policy).unwrap()
            else {
                panic!("overlapping retained branches lost their isolated contacts");
            };
            assert_eq!(contacts.len(), 2);
            assert_eq!(
                overlap.orientation(),
                RationalBezierOverlapOrientation2::Same
            );
        }
    }

    #[test]
    fn self_contact_authority_distinguishes_injective_and_retraced_quadratics() {
        let injective = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::one()),
                Point2::new(Real::from(2_i8), Real::zero()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();
        let retraced = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::zero()),
                Point2::new(Real::zero(), Real::zero()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            assert!(matches!(
                injective.self_intersection_contacts(&policy).unwrap(),
                RationalBezierIntersectionContacts2::NoIntersection
            ));
            assert!(matches!(
                retraced.self_intersection_contacts(&policy).unwrap(),
                RationalBezierIntersectionContacts2::DegenerateResultant
            ));
        }
    }

    #[test]
    fn exact_degree_elevated_line_recovers_linear_parameter_transport() {
        let third = (Real::one() / Real::from(3_i8)).unwrap();
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let elevated = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), half.clone()),
                Point2::new(third.clone(), half.clone()),
                Point2::new(&third + &third, half.clone()),
                Point2::new(Real::one(), half),
            ],
            vec![Real::one(); 4],
        )
        .unwrap();
        let policy = CurveContext::STRICT;
        let reduced = elevated
            .exact_linear_homogeneous_representative(&policy)
            .unwrap()
            .expect("exact inverse elevation recovers the linear carrier");

        assert_eq!(reduced.degree(), 1);
        assert_eq!(reduced.start(), elevated.start());
        assert_eq!(reduced.end(), elevated.end());

        let symbolic_weights =
            RationalBezier2::try_new(elevated.control_points().to_vec(), vec![Real::pi(); 4])
                .unwrap();
        assert!(
            symbolic_weights
                .exact_linear_homogeneous_representative(&policy)
                .unwrap()
                .is_none(),
            "nonrational Real carriers retain the certified general path"
        );
    }

    #[test]
    fn endpoint_projective_cubic_correspondence_maps_both_orientations() {
        let controls = vec![
            Point2::new(Real::zero(), Real::zero()),
            Point2::new(Real::from(7_i8), Real::from(-5_i8)),
            Point2::new(Real::from(8_i8), Real::from(-4_i8)),
            Point2::new(Real::from(3_i8), Real::from(3_i8)),
        ];
        let first = RationalBezier2::try_new(controls.clone(), vec![Real::one(); 4]).unwrap();
        let same = RationalBezier2::try_new(
            controls.clone(),
            vec![
                Real::one(),
                Real::from(2_i8),
                Real::from(4_i8),
                Real::from(8_i8),
            ],
        )
        .unwrap();
        let reversed = RationalBezier2::try_new(
            controls.into_iter().rev().collect(),
            vec![
                Real::from(8_i8),
                Real::from(4_i8),
                Real::from(2_i8),
                Real::one(),
            ],
        )
        .unwrap();
        let unit = BezierParameterRange2::from_exact(Real::zero(), Real::one());
        let first_parameter = BezierParameter2::Exact((Real::one() / Real::from(3_i8)).unwrap());
        let same_expected = (Real::one() / Real::from(5_i8)).unwrap();
        let reversed_expected = (Real::from(4_i8) / Real::from(5_i8)).unwrap();

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (second, expected) in [(&same, &same_expected), (&reversed, &reversed_expected)] {
                assert!(matches!(
                    first.image_overlap(second, &policy),
                    Classification::Decided(RationalBezierSharedComponentReplay::Overlap(_))
                ));
                let correspondence =
                    RationalBezierOverlapParameterCorrespondence2::new(&first, second, &policy);
                let Classification::Decided(Some(mapped)) = correspondence
                    .map_first_to_second(&first_parameter, &unit, &unit, &policy)
                    .unwrap()
                else {
                    panic!("projective correspondence did not map the first parameter");
                };
                assert_eq!(mapped.as_exact(), Some(expected));
                let Classification::Decided(Some(round_trip)) = correspondence
                    .map_second_to_first(&mapped, &unit, &unit, &policy)
                    .unwrap()
                else {
                    panic!("projective correspondence did not map the second parameter");
                };
                assert_eq!(round_trip, first_parameter);
            }
        }
    }

    #[test]
    fn range_projective_correspondence_maps_and_inverts_oriented_ranges() {
        let first_range = BezierParameterRange2::from_exact(
            (Real::one() / Real::from(4_i8)).unwrap(),
            (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
        );
        let second_low = (Real::one() / Real::from(5_i8)).unwrap();
        let second_high = (Real::from(4_i8) / Real::from(5_i8)).unwrap();
        let scale = Real::from(2_i8);

        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for (reversed, second_range) in [
                (
                    false,
                    BezierParameterRange2::from_exact(second_low.clone(), second_high.clone()),
                ),
                (
                    true,
                    BezierParameterRange2::from_exact(second_high.clone(), second_low.clone()),
                ),
            ] {
                for (first, expected_second) in [
                    (first_range.start(), second_range.start()),
                    (first_range.end(), second_range.end()),
                ] {
                    let Classification::Decided(Some(mapped)) = range_projective_parameter_image(
                        first,
                        &first_range,
                        &second_range,
                        &scale,
                        reversed,
                        true,
                        &policy,
                    )
                    .unwrap() else {
                        panic!("range projective map did not map an endpoint");
                    };
                    assert_eq!(mapped, expected_second.clone());
                    let Classification::Decided(Some(round_trip)) =
                        range_projective_parameter_image(
                            &mapped,
                            &first_range,
                            &second_range,
                            &scale,
                            reversed,
                            false,
                            &policy,
                        )
                        .unwrap()
                    else {
                        panic!("range projective inverse did not map an endpoint");
                    };
                    assert_eq!(round_trip, first.clone());
                }
            }
        }
    }

    #[test]
    fn shared_demo_conic_cubic_contacts_are_complete() {
        let conic = RationalBezier2::try_new(
            vec![
                Point2::new(exact_f64(14.600491094738247), exact_f64(-20.78282692939043)),
                Point2::new(exact_f64(18.91), exact_f64(6.2)),
                Point2::new(exact_f64(20.150000000000002), exact_f64(6.820000000000001)),
            ],
            vec![Real::one(), exact_f64(0.36), Real::one()],
        )
        .unwrap();
        let cubic = RationalBezier2::try_new(
            vec![
                Point2::new(exact_f64(-24.8), exact_f64(-18.6)),
                Point2::new(exact_f64(-12.4), exact_f64(-20.77)),
                Point2::new(exact_f64(12.4), exact_f64(-20.77)),
                Point2::new(exact_f64(24.8), exact_f64(-18.6)),
            ],
            vec![Real::one(); 4],
        )
        .unwrap();
        let policy = CurveContext::STRICT;

        let contacts = conic
            .implicit_conic_intersection_contacts(&cubic, &policy)
            .unwrap();
        assert!(
            matches!(
                contacts,
                Some(Classification::Decided(
                    RationalBezierIntersectionContacts2::Contacts(_)
                        | RationalBezierIntersectionContacts2::NoIntersection
                ))
            ),
            "{contacts:#?}"
        );
    }

    #[test]
    fn shared_demo_cubic_pair_contacts_are_complete() {
        let first = RationalBezier2::try_new(
            vec![
                Point2::new(exact_f64(4.03), exact_f64(-4.03)),
                Point2::new(exact_f64(7.184295191466809), exact_f64(-46.13835323035717)),
                Point2::new(exact_f64(10.85), exact_f64(5.89)),
                Point2::new(exact_f64(14.600491094738247), exact_f64(-20.78282692939043)),
            ],
            vec![Real::one(); 4],
        )
        .unwrap();
        let second = RationalBezier2::try_new(
            vec![
                Point2::new(exact_f64(-24.8), exact_f64(-18.6)),
                Point2::new(exact_f64(-12.4), exact_f64(-20.77)),
                Point2::new(exact_f64(12.4), exact_f64(-20.77)),
                Point2::new(exact_f64(24.8), exact_f64(-18.6)),
            ],
            vec![Real::one(); 4],
        )
        .unwrap();
        let policy = CurveContext::STRICT;
        let contacts = first.intersection_contacts(&second, &policy).unwrap();
        let RationalBezierIntersectionContacts2::Contacts(contacts) = contacts else {
            panic!("shared cubic pair should produce complete contacts");
        };
        assert_eq!(contacts.len(), 3);
    }

    #[test]
    fn implicit_conic_route_replays_quadratic_line_contact() {
        let conic = RationalBezier2::try_new(
            vec![
                Point2::new(11.into(), 7.into()),
                Point2::new(20.into(), 4.into()),
                Point2::new(29.into(), 7.into()),
            ],
            vec![
                Real::one(),
                (Real::one() / Real::from(2_i8)).unwrap(),
                Real::one(),
            ],
        )
        .unwrap();
        let line = RationalBezier2::try_new(
            vec![
                Point2::new(20.into(), (-11).into()),
                Point2::new(20.into(), (Real::from(-1_i8) / Real::from(2_i8)).unwrap()),
                Point2::new(20.into(), 10.into()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();
        let policy = CurveContext::STRICT;

        let contacts = conic.intersection_contacts(&line, &policy).unwrap();
        let RationalBezierIntersectionContacts2::Contacts(ref contacts) = contacts else {
            panic!("quadratic line should meet the rational conic: {contacts:#?}");
        };
        assert_eq!(contacts.len(), 1);
        assert!(contacts[0].is_certified_transverse());
        assert_eq!(
            contacts[0].first_parameter().as_exact(),
            Some(&(Real::one() / Real::from(2_i8)).unwrap())
        );
        assert_eq!(
            contacts[0].second_parameter().as_exact(),
            Some(&(Real::from(17_i8) / Real::from(21_i8)).unwrap())
        );
    }

    #[test]
    fn exact_line_image_route_replays_algebraic_conic_contact() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let conic = RationalBezier2::try_new(
            vec![
                Point2::new(17.into(), 7.into()),
                Point2::new(Real::from(51_i8) * &half, 4.into()),
                Point2::new(34.into(), 7.into()),
            ],
            vec![Real::one(), half, Real::one()],
        )
        .unwrap();
        let line = RationalBezier2::try_new(
            vec![
                Point2::new(25.into(), (-11).into()),
                Point2::new(25.into(), (Real::from(-1_i8) / Real::from(2_i8)).unwrap()),
                Point2::new(25.into(), 10.into()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();
        let policy = CurveContext::STRICT;

        let contacts = conic.intersection_contacts(&line, &policy).unwrap();
        let RationalBezierIntersectionContacts2::Contacts(ref contacts) = contacts else {
            panic!("algebraic line/conic contact was lost: {contacts:#?}");
        };
        assert_eq!(contacts.len(), 1);
    }

    #[test]
    fn clones_share_retained_axis_derivative_numerators() {
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(0.into(), 0.into()),
                Point2::new(1.into(), 2.into()),
                Point2::new(0.into(), 1.into()),
                Point2::new(1.into(), 0.into()),
            ],
            vec![1.into(); 4],
        )
        .unwrap();
        let clone = curve.clone();

        assert!(curve.data.x_derivative_numerator_bernstein.get().is_none());
        assert!(curve.data.x_axis_monotonicity.get().is_none());
        assert!(matches!(
            clone.axis_is_monotone(Axis2::X, &CurveContext::STRICT),
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
    fn endpoint_derivative_specialization_matches_general_horner_recurrence() {
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(0.into(), 1.into()),
                Point2::new(2.into(), 4.into()),
                Point2::new(5.into(), (-1).into()),
                Point2::new(7.into(), 3.into()),
            ],
            vec![2.into(), 3.into(), 5.into(), 7.into()],
        )
        .unwrap();
        let policy = CurveContext::STRICT;

        for (at_end, parameter) in [(false, Real::zero()), (true, Real::one())] {
            assert_eq!(
                curve.endpoint_derivatives(at_end, 3, &policy),
                curve.affine_derivative_values_at(&parameter, 3, &policy)
            );
        }
    }

    #[test]
    fn conic_dual_coordinate_sum_maps_endpoints_without_a_pole() {
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(0.into(), 0.into()),
                Point2::new(1.into(), 2.into()),
                Point2::new(3.into(), 0.into()),
            ],
            vec![2.into(), 3.into(), 5.into()],
        )
        .unwrap();
        let policy = CurveContext::STRICT;
        let Classification::Decided(parameter_map) =
            conic_parameter_map(&curve, &curve, &policy).unwrap()
        else {
            panic!("nonsingular rational quadratic did not produce a parameter map");
        };
        let (numerator, denominator) = &parameter_map.primary;
        let third = (Real::one() / Real::from(3_i8)).unwrap();

        for parameter in [Real::zero(), third, Real::one()] {
            let image = (evaluate_power_polynomial(numerator, &parameter)
                / evaluate_power_polynomial(denominator, &parameter))
            .unwrap();
            assert_eq!(image, parameter);
        }
    }

    #[test]
    fn conic_parameter_primary_map_defers_fallback_image_polynomial() {
        let policy = CurveContext::STRICT;
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(
            vec![-half, Real::zero(), Real::one()],
            &policy,
        )
        .unwrap() else {
            panic!("quadratic source polynomial was not certified");
        };
        let Classification::Decided(interval) =
            crate::BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap()
        else {
            panic!("unit interval was not certified");
        };
        let Classification::Decided(parameter) =
            crate::BezierAlgebraicParameter2::try_isolate(polynomial.clone(), interval, &policy)
                .unwrap()
        else {
            panic!("positive quadratic root was not isolated");
        };
        let Classification::Decided(candidate) = conic_parameter_candidate(
            polynomial.coefficients(),
            &(vec![Real::zero(), Real::one()], vec![Real::one()]),
            &policy,
        )
        .unwrap() else {
            panic!("identity conic parameter map was not constructed");
        };

        assert!(candidate.image_polynomial.get().is_none());
        assert!(matches!(
            conic_parameter_from_candidates(
                std::slice::from_ref(&candidate),
                &BezierParameter2::Algebraic(parameter),
                &policy,
            )
            .unwrap(),
            Classification::Decided(Some(_))
        ));
        assert!(
            candidate.image_polynomial.get().is_none(),
            "the primary exact map must not construct its unused fallback polynomial"
        );
    }

    #[test]
    fn rational_parameter_image_map_reuses_quotient_authority_across_isolators() {
        let third = (Real::one() / Real::from(3_i8)).unwrap();
        let two_thirds = Real::from(2_i8) * &third;
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let Classification::Decided(polynomial) =
                BezierParameterPolynomial::try_new_power_basis(
                    vec![&third * &two_thirds, -Real::one(), Real::one()],
                    &policy,
                )
                .unwrap()
            else {
                panic!("the two-root source polynomial must be certified");
            };
            let make_parameter = |lower: Real, upper: Real| {
                let Classification::Decided(interval) =
                    crate::BezierParameterInterval::try_new(lower, upper, &policy).unwrap()
                else {
                    panic!("the source isolating interval must be certified");
                };
                let Classification::Decided(parameter) =
                    crate::BezierAlgebraicParameter2::try_isolate(
                        polynomial.clone(),
                        interval,
                        &policy,
                    )
                    .unwrap()
                else {
                    panic!("the source root must be isolated");
                };
                BezierParameter2::Algebraic(parameter)
            };
            let first = make_parameter(
                (Real::one() / Real::from(4_i8)).unwrap(),
                (Real::one() / Real::from(2_i8)).unwrap(),
            );
            let second = make_parameter(
                (Real::one() / Real::from(2_i8)).unwrap(),
                (Real::from(3_i8) / Real::from(4_i8)).unwrap(),
            );
            let mut map = RationalParameterImageMap2::new(
                vec![Real::zero(), Real::one()],
                vec![Real::one()],
                &policy,
            );

            for source in [&first, &second] {
                let Classification::Decided(Some(image)) = map.image(source).unwrap() else {
                    panic!("the identity rational map must retain each exact root");
                };
                assert!(matches!(
                    image.cmp_by_refinement(source, &policy).unwrap(),
                    Classification::Decided(Ordering::Equal)
                ));
            }
            assert_eq!(map.candidates.len(), 1);
            let candidate = &map.candidates[0].1;
            assert!(
                candidate
                    .quotient_matrices
                    .get()
                    .is_some_and(Option::is_some)
            );
            assert!(candidate.quotient_power.get().is_some_and(Option::is_some));
        }
    }

    #[test]
    fn conic_parameter_refines_primary_map_before_constructing_fallback() {
        let policy = CurveContext::STRICT;
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let Classification::Decided(polynomial) = BezierParameterPolynomial::try_new_power_basis(
            vec![-half.clone(), Real::zero(), Real::one()],
            &policy,
        )
        .unwrap() else {
            panic!("quadratic source polynomial was not certified");
        };
        let Classification::Decided(interval) =
            crate::BezierParameterInterval::try_new(Real::zero(), Real::one(), &policy).unwrap()
        else {
            panic!("unit interval was not certified");
        };
        let Classification::Decided(parameter) =
            crate::BezierAlgebraicParameter2::try_isolate(polynomial.clone(), interval, &policy)
                .unwrap()
        else {
            panic!("positive quadratic root was not isolated");
        };
        let Classification::Decided(candidate) = conic_parameter_candidate(
            polynomial.coefficients(),
            &(vec![-half, Real::one()], vec![Real::zero(), Real::one()]),
            &policy,
        )
        .unwrap() else {
            panic!("rational conic parameter map was not constructed");
        };

        assert!(candidate.image_polynomial.get().is_none());
        assert!(matches!(
            conic_parameter_from_candidates(
                std::slice::from_ref(&candidate),
                &BezierParameter2::Algebraic(parameter),
                &policy,
            )
            .unwrap(),
            Classification::Decided(Some(_))
        ));
        assert!(
            candidate.image_polynomial.get().is_none(),
            "source refinement must certify the primary map before constructing its fallback"
        );
    }

    #[test]
    fn quotient_ring_rational_image_retains_nonrational_source_coefficients() {
        let policy = CurveContext::STRICT;
        let pi = Real::pi();
        let coefficients = quotient_ring_rational_map_image_polynomial(
            &[-pi.clone(), Real::zero(), Real::one()],
            &[Real::zero(), Real::one()],
            &[Real::one(), Real::one()],
            &policy,
        )
        .expect("the quotient-ring image polynomial must be constructed exactly");
        let expected = [-pi.clone(), Real::from(2_i8) * &pi, Real::one() - pi];

        assert_eq!(coefficients.len(), expected.len());
        for (coefficient, expected) in coefficients.iter().zip(expected) {
            assert_eq!(
                compare_reals(coefficient, &expected, &policy),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn quotient_ring_rational_image_reuses_nonrational_source_scale() {
        let policy = CurveContext::STRICT;
        let pi = Real::pi();
        let numerator = [Real::zero(), Real::one()];
        let denominator = [Real::one(), Real::one()];
        let monic = quotient_ring_rational_map_image_polynomial(
            &[-pi.clone(), Real::zero(), Real::one()],
            &numerator,
            &denominator,
            &policy,
        )
        .expect("the monic quotient-ring image must remain exact");
        let pi_squared = &pi * &pi;
        let scaled = quotient_ring_rational_map_image_polynomial(
            &[-pi_squared, Real::zero(), pi],
            &numerator,
            &denominator,
            &policy,
        )
        .expect("a nonrational source scale must remain exact");

        assert_eq!(scaled.len(), monic.len());
        for (scaled, monic) in scaled.iter().zip(monic) {
            assert_eq!(
                compare_reals(scaled, &monic, &policy),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn quotient_ring_rational_image_matches_exact_resultant_samples() {
        let policy = CurveContext::STRICT;
        let source = [Real::from(-2_i8), Real::zero(), Real::zero(), Real::one()];
        let numerator = [Real::one(), Real::zero(), Real::one()];
        let denominator = [Real::from(2_i8), Real::one()];
        let coefficients =
            quotient_ring_rational_map_image_polynomial(&source, &numerator, &denominator, &policy)
                .expect("the cubic quotient-ring image polynomial must be exact");

        for sample in 0..=3 {
            let value = Real::from(sample);
            let relation = subtract_power_polynomials(
                &numerator,
                &scale_power_polynomial(&denominator, &value),
            );
            let sampled = resultant_univariate_polynomials(
                &source,
                &relation,
                RATIONAL_INTERSECTION_RESULTANT_PRECISION,
            )
            .unwrap()
            .resultant;
            assert_eq!(
                compare_reals(
                    &evaluate_power_polynomial(&coefficients, &value),
                    &sampled,
                    &policy,
                ),
                Some(Ordering::Equal)
            );
        }
    }

    #[test]
    fn implicit_conic_contacts_retain_source_parameter_point_image_first() {
        let half = (Real::one() / Real::from(2_i8)).unwrap();
        let third = (Real::one() / Real::from(3_i8)).unwrap();
        let two_thirds = Real::from(2_i8) * &third;
        let conic = RationalBezier2::try_new(
            vec![
                Point2::new(0.into(), 0.into()),
                Point2::new(half.clone(), 0.into()),
                Point2::new(1.into(), 1.into()),
            ],
            vec![1.into(); 3],
        )
        .unwrap();
        let line = RationalBezier2::try_new(
            vec![
                Point2::new(0.into(), half.clone()),
                Point2::new(third, half.clone()),
                Point2::new(two_thirds, half.clone()),
                Point2::new(1.into(), half),
            ],
            vec![1.into(); 4],
        )
        .unwrap();

        let policy = CurveContext::STRICT;
        let Some(Classification::Decided(RationalBezierIntersectionContacts2::Contacts(contacts))) =
            conic
                .implicit_conic_intersection_contacts(&line, &policy)
                .unwrap()
        else {
            panic!("parabola and horizontal line did not produce their algebraic contact");
        };
        let [contact] = contacts.as_ref() else {
            panic!("parabola and horizontal line should have one finite contact");
        };
        let BezierParameter2::Algebraic(source_parameter) = contact.second_parameter() else {
            panic!("horizontal-line contact parameter should remain algebraic");
        };
        assert!(
            source_parameter
                .cached_rational_bezier_point_image(&line)
                .is_none()
        );
        let BezierParameter2::Algebraic(conic_parameter) = contact.first_parameter() else {
            panic!("parabola contact parameter should remain algebraic");
        };
        assert!(
            conic_parameter
                .cached_rational_bezier_point_image(&conic)
                .is_none()
        );
        let RationalBezierIntersectionPointEvidence2::Algebraic(point_image) = contact.point()
        else {
            panic!("implicit-conic contact did not retain algebraic point evidence");
        };
        assert_eq!(
            point_image.status(),
            crate::BezierAlgebraicImageStatus::RetainedRationalExpression
        );
        assert!(point_image.x().is_none());
        assert!(point_image.y().is_none());
        assert_eq!(point_image.retained_parameter(), Some(source_parameter));
        assert!(point_image.parameter().is_valid());
        let unresolved_clone = point_image.clone();
        let resolved = point_image
            .resolved(&policy)
            .expect("retained exact point source must resolve");
        assert_eq!(
            resolved.status(),
            crate::BezierAlgebraicImageStatus::Transformed
        );
        assert!(resolved.x().is_some());
        assert!(resolved.y().is_some());
        assert_eq!(point_image, &unresolved_clone);
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
        let policy = CurveContext::STRICT;

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

    #[test]
    fn high_degree_point_evaluation_uses_exact_bounded_memory_bernstein_path() {
        let degree = MAX_RETAINED_EVALUATION_POWER_DEGREE + 1;
        let curve = RationalBezier2::try_new(
            (0..=degree)
                .map(|index| {
                    Point2::new(
                        Real::from(u64::try_from(index).unwrap()),
                        Real::from(u64::try_from(index % 7).unwrap()),
                    )
                })
                .collect(),
            vec![Real::one(); degree + 1],
        )
        .unwrap();
        let parameter = (Real::one() / Real::from(2_u8)).unwrap();
        let policy = CurveContext::STRICT;

        let point = curve.point_at(&parameter, &policy).unwrap();
        let expected_x = (Real::from(u64::try_from(degree).unwrap()) / Real::from(2_u8)).unwrap();
        assert_eq!(
            compare_reals(point.x(), &expected_x, &policy),
            Some(std::cmp::Ordering::Equal)
        );
        assert!(curve.data.homogeneous_power_basis.get().is_none());
    }

    #[test]
    fn derivative_nonzero_certificate_respects_a_zero_endpoint_coefficient() {
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::zero()),
            ],
            vec![Real::one(); 3],
        )
        .unwrap();
        let policy = CurveContext::STRICT;
        assert_eq!(
            curve
                .derivative_is_certified_nonzero_at(&Real::zero(), &policy)
                .unwrap(),
            Classification::Decided(false)
        );
        assert_eq!(
            curve
                .derivative_is_certified_nonzero_at(
                    &(Real::one() / Real::from(2_u8)).unwrap(),
                    &policy,
                )
                .unwrap(),
            Classification::Decided(true)
        );
    }

    #[test]
    fn negative_common_weights_preserve_geometric_line_crossing_direction() {
        let curve = RationalBezier2::try_new(
            vec![
                Point2::new(Real::from(-1_i8), Real::zero()),
                Point2::new(Real::zero(), Real::zero()),
                Point2::new(Real::one(), Real::zero()),
            ],
            vec![Real::from(-1_i8); 3],
        )
        .unwrap();
        let line = LineSeg2::try_new(
            Point2::new(Real::zero(), Real::from(-1_i8)),
            Point2::new(Real::zero(), Real::one()),
        )
        .unwrap();
        let relation = curve.relation_to_line_with_contacts(&line, &CurveContext::STRICT);
        let Classification::Decided(BezierLineContactRelation::Contacts { contacts }) = relation
        else {
            panic!("the negative-weight line crossing must be decided");
        };
        assert_eq!(contacts.len(), 1);
        let half = (Real::one() / Real::from(2_u8)).unwrap();
        assert_eq!(contacts[0].parameter().as_exact(), Some(&half));
        assert_eq!(
            contacts[0].crossing_direction(),
            Some(BezierLineCrossingDirection::PositiveToNegative)
        );
    }

    #[test]
    fn high_degree_bernstein_evaluation_matches_de_casteljau_with_varying_weights() {
        let degree = MAX_RETAINED_EVALUATION_POWER_DEGREE + 1;
        let curve = RationalBezier2::try_new(
            (0..=degree)
                .map(|index| {
                    Point2::new(
                        Real::from(u64::try_from(index % 19).unwrap()),
                        Real::from(u64::try_from(index % 11).unwrap()),
                    )
                })
                .collect(),
            (0..=degree)
                .map(|index| Real::from(u64::try_from(index % 5 + 1).unwrap()))
                .collect(),
        )
        .unwrap();
        let parameter = (Real::one() / Real::from(3_u8)).unwrap();
        let policy = CurveContext::STRICT;

        let actual = curve.point_at(&parameter, &policy).unwrap();
        let expected = match curve.homogeneous_de_casteljau_value(&parameter) {
            Classification::Decided(value) => match project_homogeneous(&value, &policy) {
                Classification::Decided(point) => point,
                Classification::Uncertain(reason) => {
                    panic!("de Casteljau projection blocked: {reason:?}")
                }
            },
            Classification::Uncertain(reason) => {
                panic!("de Casteljau evaluation blocked: {reason:?}")
            }
        };
        assert_eq!(
            compare_reals(actual.x(), expected.x(), &policy),
            Some(std::cmp::Ordering::Equal)
        );
        assert_eq!(
            compare_reals(actual.y(), expected.y(), &policy),
            Some(std::cmp::Ordering::Equal)
        );
        assert!(curve.data.homogeneous_power_basis.get().is_none());
    }
}
