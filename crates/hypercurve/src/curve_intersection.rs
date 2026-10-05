//! Top-level exact curve-pair intersection with retained parameter intervals.

#[path = "curve_support_intersection.rs"]
mod curve_support_intersection;

use std::sync::Arc;
use std::sync::OnceLock;

use hyperreal::Real;

use crate::classify::compare_reals;
use crate::intersect::{circle_relation_from_supports, oriented_param_range_overlap};
use crate::policy::resolve_certified_operation;
use crate::rational_bezier::RationalQuadraticCircle2;
use crate::rational_bezier_general::{
    RationalBezierIntersectionContext, RationalBezierOverlapParameterCorrespondence2,
};
use crate::{
    ArcArcIntersection, BezierParameter2, BezierParameterRange2, CircleCircleRelation,
    CircularArc2, Classification, Curve2, CurveContext, CurveError, CurveGeometry2,
    CurveOperation2, CurveOutcome, CurveParameter2, CurveParameterRange2, CurvePoint2, CurveResult,
    CurveSpanRange2, ExactCurveError, ExactCurveResult, LineArcIntersection,
    LineArcIntersectionPoint, LineArcOrder, LineLineIntersection, ParamRange, Point2,
    RationalBezier2, RationalBezierIntersectionContacts2, UncertaintyReason,
};

/// Exact location in a curve's retained span chart.
///
/// The local parameter keeps its selected-root or geometric authority. The
/// span chart maps it into the authored curve parameter only when requested.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveLocation2 {
    span_index: usize,
    span_range: CurveSpanRange2,
    local_parameter: CurveParameter2,
}

/// Every exact preimage of one point on a curve.
#[derive(Clone, Debug, PartialEq)]
pub enum CurvePointLocations2 {
    /// The curve is constant at the point, so every parameter maps to it.
    EntireCurve,
    /// The complete ordered set of distinct parameter locations. A point
    /// visited more than once keeps one location per visit; a continuous
    /// spline seam is reported once.
    Locations(Vec<CurveLocation2>),
}

/// One exact top-level curve contact with parameters on both operands.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionContact2 {
    first: CurveLocation2,
    second: CurveLocation2,
    point: CurvePoint2,
    certified_transverse: bool,
    tangent_cross_sign: Option<hyperreal::RealSign>,
}

/// One connected closed set of parameters on a retained support chart.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveParameterSet2 {
    /// One exact parameter, retaining its selected-root or geometric authority.
    Single(CurveParameter2),
    /// Every parameter in one nondegenerate closed range.
    Range(CurveParameterRange2),
}

impl CurveParameterSet2 {
    pub(crate) fn boundaries(&self) -> impl Iterator<Item = &CurveParameter2> {
        let (start, end) = match self {
            Self::Single(parameter) => (parameter, None),
            Self::Range(range) => (range.start(), Some(range.end())),
        };
        std::iter::once(start).chain(end)
    }
}

/// A complete Cartesian parameter component whose image is one exact point.
///
/// At least one operand contributes a range. This evidence distinguishes a
/// collapsed trace from an isolated contact or a positive-length image overlap.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionParameterComponent2 {
    first_span_index: usize,
    second_span_index: usize,
    first_parameters: CurveParameterSet2,
    second_parameters: CurveParameterSet2,
    point: CurvePoint2,
}

impl CurveIntersectionParameterComponent2 {
    /// Returns the retained span index on the first curve.
    pub const fn first_span_index(&self) -> usize {
        self.first_span_index
    }

    /// Returns the retained span index on the second curve.
    pub const fn second_span_index(&self) -> usize {
        self.second_span_index
    }

    /// Returns the complete local parameter set on the first span.
    pub const fn first_parameters(&self) -> &CurveParameterSet2 {
        &self.first_parameters
    }

    /// Returns the complete local parameter set on the second span.
    pub const fn second_parameters(&self) -> &CurveParameterSet2 {
        &self.second_parameters
    }

    /// Returns the exact point shared by every pair in this component.
    pub const fn point(&self) -> &CurvePoint2 {
        &self.point
    }
}

/// Relative parameter orientation of a certified shared curve image.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CurveOverlapOrientation2 {
    /// Both parameter domains traverse the shared image in the same direction.
    Same,
    /// The second parameter domain traverses the shared image in reverse.
    Reversed,
}

/// Certified positive-length overlap between two retained curve spans.
///
/// The oriented ranges bound the overlap closure. Endpoint inclusion remains
/// explicit because a strict exact branch predicate can select an open end.
#[derive(Clone, Debug)]
pub struct CurveIntersectionOverlap2 {
    pub(crate) first_span_index: usize,
    pub(crate) second_span_index: usize,
    pub(crate) first_range: CurveParameterRange2,
    pub(crate) second_range: CurveParameterRange2,
    pub(crate) orientation: CurveOverlapOrientation2,
    pub(crate) endpoint_inclusion: [bool; 2],
    pub(crate) parameter_correspondence: CurveOverlapCorrespondence2,
}

/// The complete support correspondence retains its original chart intervals.
/// Clipping an overlap changes its active domain, never the map's basis.
#[derive(Clone, Debug)]
pub(crate) struct RationalCurveOverlap2 {
    source: RationalBezierOverlapParameterCorrespondence2,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
}

impl RationalCurveOverlap2 {
    fn new(
        source: RationalBezierOverlapParameterCorrespondence2,
        overlap: &crate::RationalBezierIntersectionOverlap2,
    ) -> Self {
        Self {
            source,
            first_range: overlap.first_range().clone(),
            second_range: overlap.second_range().clone(),
        }
    }

    pub(crate) fn clipped_ranges(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        self.source
            .clipped_ranges(&self.first_range, &self.second_range, first, second, policy)
    }
}

/// Circle overlap transport shared by open-curve and region operations.
/// Each case retains the kernel's original parameter and selected-field authority.
#[derive(Clone, Debug)]
pub(crate) enum CurveCircleOverlap2 {
    Pair(crate::bezier_offset::BezierAlgebraicCuspSemicirclePairOverlap2),
    Mapped(crate::bezier_offset::BezierAlgebraicCuspSemicircleMappedOverlap2),
    Selected(crate::bezier_offset::BezierAlgebraicCuspSemicircleSelectedFiberRationalOverlap2),
}

impl CurveCircleOverlap2 {
    pub(crate) fn orientation(&self) -> CurveOverlapOrientation2 {
        match self {
            Self::Pair(source) => source.orientation(),
            Self::Mapped(source) => source.orientation(),
            Self::Selected(source) => source.orientation(),
        }
    }

    pub(crate) fn parameter_ranges(&self) -> (CurveParameterRange2, CurveParameterRange2) {
        match self {
            Self::Pair(source) => source.parameter_ranges(),
            Self::Mapped(source) => source.parameter_ranges(),
            Self::Selected(source) => source.parameter_ranges(),
        }
    }

    fn map_parameter(
        &self,
        parameter: &CurveParameter2,
        forward: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        match self {
            Self::Pair(source) => {
                let parameter = parameter
                    .as_algebraic_cusp()
                    .ok_or(CurveError::InvalidCurveParameter)?;
                Ok(Classification::Decided(Some(
                    CurveParameter2::from_algebraic_cusp(source.map_parameter(parameter, forward)),
                )))
            }
            Self::Mapped(source) => source.map_parameter(parameter, forward, policy),
            Self::Selected(source) => source.map_parameter(parameter, forward, policy),
        }
    }

    pub(crate) fn clipped_ranges(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        // Both ranges pair corresponding endpoints; independently sorting the
        // second range would erase the map's orientation.
        let (first_overlap, second_overlap) = self.parameter_ranges();
        crate::bezier_split::clip_corresponding_parameter_ranges(
            &first_overlap,
            &second_overlap,
            first,
            second,
            policy,
            |parameter| self.map_parameter(parameter, true, policy),
            |parameter| self.map_parameter(parameter, false, policy),
        )
    }

    /// Replays the closed-set contact when `clipped_ranges` found no positive
    /// span. A monotone component can then meet the active domains only at an
    /// endpoint of the first clipped interval. Forward transport preserves the
    /// original selected parameters without constructing inverse cuts.
    pub(crate) fn singleton_contact(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<[CurveParameter2; 2]>>> {
        use crate::bezier_split::CurveParameterDomain2;
        let (first_overlap, second_overlap) = self.parameter_ranges();
        for parameter in [
            first.start(),
            first.end(),
            first_overlap.start(),
            first_overlap.end(),
        ] {
            let contains = |range, parameter| {
                CurveParameterDomain2::new(range, None).contains_finite_parameter(parameter, policy)
            };
            let mut admitted = true;
            for range in [&first_overlap, first] {
                match contains(range, parameter)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        admitted = false;
                        break;
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if !admitted {
                continue;
            }
            let mapped = match self.map_parameter(parameter, true, policy)? {
                Classification::Decided(Some(mapped)) => mapped,
                Classification::Decided(None) => continue,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            for range in [&second_overlap, second] {
                match contains(range, &mapped)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        admitted = false;
                        break;
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            if admitted {
                return Ok(Classification::Decided(Some([parameter.clone(), mapped])));
            }
        }
        Ok(Classification::Decided(None))
    }
}

/// Exact transport retained by a shared-image component. Each variant owns
/// the support evidence required to restrict the correspondence again.
#[derive(Clone, Debug)]
pub(crate) enum CurveOverlapCorrespondence2 {
    Rational {
        source: RationalCurveOverlap2,
        swapped: bool,
    },
    ParameterComponent {
        source: crate::bezier_offset::BezierParameterComponentOverlap2,
        swapped: bool,
    },
    Circle {
        source: CurveCircleOverlap2,
        swapped: bool,
    },
    ChordRational {
        source: Arc<crate::bezier_offset::BezierAlgebraicChordRationalOverlap2>,
        chord_first: bool,
    },
    Chords {
        first: crate::BezierAlgebraicChord2,
        second: crate::BezierAlgebraicChord2,
        first_range: CurveParameterRange2,
        second_range: CurveParameterRange2,
    },
}

impl CurveOverlapCorrespondence2 {
    /// Transports one exact parameter through the retained correspondence.
    /// Both curve intersection and family selection use this same authority.
    pub(crate) fn map_parameter(
        &self,
        parameter: &CurveParameter2,
        forward: bool,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<CurveParameter2>>> {
        match self {
            Self::Rational { source, swapped } => {
                if forward != *swapped {
                    source.source.map_first_to_second_region_parameter(
                        parameter,
                        &source.first_range,
                        &source.second_range,
                        policy,
                    )
                } else {
                    source.source.map_second_to_first_region_parameter(
                        parameter,
                        &source.first_range,
                        &source.second_range,
                        policy,
                    )
                }
            }
            Self::ParameterComponent { source, swapped } => source.map_curve_parameter(
                if forward != *swapped {
                    hypersolve::CurveResultantParameter::First
                } else {
                    hypersolve::CurveResultantParameter::Second
                },
                parameter,
                policy,
            ),
            Self::Circle { source, swapped } => {
                source.map_parameter(parameter, forward != *swapped, policy)
            }
            Self::ChordRational {
                source,
                chord_first,
            } => {
                if forward == *chord_first {
                    source.source_parameter_at_chord_parameter(
                        parameter
                            .as_algebraic_chord()
                            .ok_or(CurveError::InvalidCurveParameter)?,
                        policy,
                    )
                } else {
                    Ok(source
                        .chord_parameter_at_source_parameter(parameter, policy)?
                        .map(|parameter| parameter.map(CurveParameter2::from_algebraic_chord)))
                }
            }
            Self::Chords { first, second, .. } => {
                let target = if forward { second } else { first };
                let parameter = parameter
                    .as_algebraic_chord()
                    .ok_or(CurveError::InvalidCurveParameter)?;
                target
                    .parameter_at_certified_support_point(parameter.point().clone(), policy)
                    .map(|parameter| {
                        Classification::Decided(Some(CurveParameter2::from_algebraic_chord(
                            parameter,
                        )))
                    })
            }
        }
    }

    /// The native line or shared-lineage kernel has certified an affine map.
    fn affine(first: &ParamRange, second: &ParamRange) -> Self {
        Self::Rational {
            source: RationalCurveOverlap2 {
                source: RationalBezierOverlapParameterCorrespondence2::RangeProjective {
                    second_to_first_scale: Real::one(),
                    reversed: false,
                },
                first_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(first.start().clone()),
                    BezierParameter2::Exact(first.end().clone()),
                ),
                second_range: BezierParameterRange2::new_validated(
                    BezierParameter2::Exact(second.start().clone()),
                    BezierParameter2::Exact(second.end().clone()),
                ),
            },
            swapped: false,
        }
    }

    /// Retains transport for a component already certified by a native kernel.
    pub(crate) fn for_rational_ranges(
        first: &RationalBezier2,
        second: &RationalBezier2,
        first_range: &BezierParameterRange2,
        second_range: &BezierParameterRange2,
        orientation: CurveOverlapOrientation2,
        policy: &CurveContext,
    ) -> Self {
        let overlap = crate::RationalBezierIntersectionOverlap2::from_certified_parameters(
            first_range.start().clone(),
            first_range.end().clone(),
            second_range.start().clone(),
            second_range.end().clone(),
            orientation,
            [true, true],
        );
        Self::rational(
            RationalBezierOverlapParameterCorrespondence2::for_overlap(
                first, second, &overlap, policy,
            ),
            &overlap,
        )
    }

    pub(crate) fn rational(
        source: RationalBezierOverlapParameterCorrespondence2,
        overlap: &crate::RationalBezierIntersectionOverlap2,
    ) -> Self {
        Self::Rational {
            source: RationalCurveOverlap2::new(source, overlap),
            swapped: false,
        }
    }

    pub(crate) fn clipped_ranges(
        &self,
        first_range: &CurveParameterRange2,
        second_range: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<(CurveParameterRange2, CurveParameterRange2)>>> {
        let swapped = match self {
            Self::Rational { swapped, .. }
            | Self::Circle { swapped, .. }
            | Self::ParameterComponent { swapped, .. } => *swapped,
            _ => false,
        };
        let (first_range, second_range) = if swapped {
            (second_range, first_range)
        } else {
            (first_range, second_range)
        };
        let ranges = match self {
            Self::Rational { source, .. } => {
                source.clipped_ranges(first_range, second_range, policy)
            }
            Self::ParameterComponent { source, .. } => {
                source.clipped_ranges(first_range, second_range, policy)
            }
            Self::Circle { source, .. } => source.clipped_ranges(first_range, second_range, policy),
            Self::ChordRational {
                source,
                chord_first,
            } => {
                let (chord_range, source_range) = if *chord_first {
                    (first_range, second_range)
                } else {
                    (second_range, first_range)
                };
                Ok(source
                    .clipped_ranges(chord_range, source_range, policy)?
                    .map(|ranges| {
                        ranges.map(|(chord, source)| {
                            if *chord_first {
                                (chord, source)
                            } else {
                                (source, chord)
                            }
                        })
                    }))
            }
            Self::Chords {
                first,
                second,
                first_range: first_overlap,
                second_range: second_overlap,
            } => {
                let map = |parameter: &CurveParameter2, chord: &crate::BezierAlgebraicChord2| {
                    let parameter = parameter
                        .as_algebraic_chord()
                        .ok_or(CurveError::InvalidCurveParameter)?;
                    chord
                        .parameter_at_certified_support_point(parameter.point().clone(), policy)
                        .map(|p| {
                            Classification::Decided(Some(CurveParameter2::from_algebraic_chord(p)))
                        })
                };
                crate::bezier_split::clip_corresponding_parameter_ranges(
                    first_overlap,
                    second_overlap,
                    first_range,
                    second_range,
                    policy,
                    |p| map(p, second),
                    |p| map(p, first),
                )
            }
        }?;
        Ok(ranges.map(|ranges| {
            ranges.map(|(first, second)| {
                if swapped {
                    (second, first)
                } else {
                    (first, second)
                }
            })
        }))
    }
}

impl PartialEq for CurveIntersectionOverlap2 {
    fn eq(&self, other: &Self) -> bool {
        self.first_span_index == other.first_span_index
            && self.second_span_index == other.second_span_index
            && self.first_range == other.first_range
            && self.second_range == other.second_range
            && self.orientation == other.orientation
            && self.endpoint_inclusion == other.endpoint_inclusion
    }
}

/// Complete unpaired parameter projections for a curve intersection query.
///
/// Each root retains its exact scalar or algebraic authority in the original
/// operand chart. Projection alone does not prove incidence: replay must pair
/// roots, reject excluded poles and select the intended geometric branches.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum CurveIntersectionCandidates2 {
    /// At least one projection has no root in the queried parameter domains.
    NoIntersection,
    /// Both projections contain every possible isolated contact parameter.
    Candidates {
        /// Ordered represented or algebraically isolated first-operand parameters.
        first_parameters: Vec<BezierParameter2>,
        /// Ordered represented or algebraically isolated second-operand parameters.
        second_parameters: Vec<BezierParameter2>,
    },
    /// Elimination requires shared-component or other degenerate replay.
    DegenerateResultant,
}

impl CurveIntersectionCandidates2 {
    pub(crate) fn swapped(mut self) -> Self {
        if let Self::Candidates {
            first_parameters,
            second_parameters,
        } = &mut self
        {
            std::mem::swap(first_parameters, second_parameters);
        }
        self
    }
}

/// Reason one promoted span pair did not produce complete contact topology.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveIntersectionPairBlockerKind2 {
    /// A required predicate remained undecided under the active policy.
    Uncertain(UncertaintyReason),
    /// Candidate replay retained some contacts but not a complete pairing.
    IncompleteReplay,
    /// Elimination found a shared algebraic component requiring overlap ownership.
    SharedComponent,
}

/// Blocker for one pair of promoted spans.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionPairBlocker2 {
    first_span_index: usize,
    second_span_index: usize,
    kind: CurveIntersectionPairBlockerKind2,
}

/// Retained top-level curve intersection result.
#[derive(Clone, Debug)]
pub struct CurveIntersectionResult2 {
    data: Arc<CurveIntersectionResultData>,
}

/// Clone-shared exact curve pieces for one complete curve pair.
#[derive(Clone, Debug)]
pub struct CurveIntersectionTopology2 {
    data: Arc<CurveIntersectionTopologyData>,
}

#[derive(Debug)]
struct CurveIntersectionTopologyData {
    result: CurveIntersectionResult2,
    first: Arc<[Curve2]>,
    second: Arc<[Curve2]>,
}

#[derive(Debug)]
struct CurveIntersectionResultData {
    span_pair_count: usize,
    contacts: Arc<[CurveIntersectionContact2]>,
    overlaps: Arc<[CurveIntersectionOverlap2]>,
    blockers: Arc<[CurveIntersectionPairBlocker2]>,
    parameter_components: Option<Arc<[CurveIntersectionParameterComponent2]>>,
}

#[derive(Debug)]
pub(crate) struct CurveIntersectionContext {
    data: CurveIntersectionContextData,
}

#[derive(Debug, Default)]
pub(crate) struct CurveIntersectionBatchCache {
    circular_support_relations: Vec<CircularSupportRelationCacheEntry>,
    circular_point_parameters: Vec<CircularPointParameterCacheEntry>,
    unit_parallel_self_intersections: Vec<Arc<UnitParallelSelfIntersections>>,
}

/// Whole-unit discovery is reusable across finite restrictions, but never
/// supplies their admission proof. The batch owns this lazy, policy-local work.
#[derive(Debug)]
struct UnitParallelSelfIntersections {
    source: crate::BezierParallel2,
    policy: CurveContext,
    result: OnceLock<CurveResult<Classification<crate::BezierParallelPairIntersectionSet2>>>,
}

impl UnitParallelSelfIntersections {
    fn result(&self) -> CurveResult<Classification<crate::BezierParallelPairIntersectionSet2>> {
        self.result
            .get_or_init(|| self.source.unit_self_intersections(&self.policy))
            .clone()
    }
}

#[derive(Debug)]
struct CircularSupportRelationCacheEntry {
    first: RationalQuadraticCircle2,
    second: RationalQuadraticCircle2,
    relation: CircleCircleRelation,
}

#[derive(Clone, Copy)]
struct CircularSupportRef<'a> {
    center: &'a Point2,
    radius_squared: &'a Real,
}

#[derive(Debug)]
struct CircularPointParameterCacheEntry {
    curve: RationalBezier2,
    point: Point2,
    parameters: Classification<Arc<[BezierParameter2]>>,
}

impl CurveIntersectionBatchCache {
    fn unit_parallel_self_intersections(
        &mut self,
        source: crate::BezierParallel2,
        policy: &CurveContext,
    ) -> Arc<UnitParallelSelfIntersections> {
        if let Some(cached) = self
            .unit_parallel_self_intersections
            .iter()
            .find(|cached| cached.source == source && cached.policy == *policy)
        {
            return Arc::clone(cached);
        }
        let cached = Arc::new(UnitParallelSelfIntersections {
            source,
            policy: *policy,
            result: OnceLock::new(),
        });
        self.unit_parallel_self_intersections
            .push(Arc::clone(&cached));
        cached
    }

    fn circular_support_relation(
        &mut self,
        first_curve: &Curve2,
        second_curve: &Curve2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Option<CircleCircleRelation>> {
        let (Some(first), Some(second)) = (
            retained_curve_circular_support(first_curve),
            retained_curve_circular_support(second_curve),
        ) else {
            return Ok(None);
        };
        if let Some(entry) = self.circular_support_relations.iter().find(|entry| {
            entry.first.center == *first.center
                && entry.first.radius_squared == *first.radius_squared
                && entry.second.center == *second.center
                && entry.second.radius_squared == *second.radius_squared
        }) {
            return Ok(Some(entry.relation.clone()));
        }
        let relation = circle_relation_from_supports(
            first.center,
            first.radius_squared,
            second.center,
            second.radius_squared,
            policy,
        )
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Intersection, first_curve.family(), cause)
        })?;
        self.circular_support_relations
            .push(CircularSupportRelationCacheEntry {
                first: RationalQuadraticCircle2 {
                    center: first.center.clone(),
                    radius_squared: first.radius_squared.clone(),
                    tangent_contacts: None,
                },
                second: RationalQuadraticCircle2 {
                    center: second.center.clone(),
                    radius_squared: second.radius_squared.clone(),
                    tangent_contacts: None,
                },
                relation: relation.clone(),
            });
        Ok(Some(relation))
    }

    fn circular_point_parameters(
        &mut self,
        curve: &RationalBezier2,
        point: &Point2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Arc<[BezierParameter2]>>> {
        if let Some(entry) = self.circular_point_parameters.iter().find(|entry| {
            entry.curve.shares_retained_data(curve) && entry.point.shares_storage(point)
        }) {
            return Ok(entry.parameters.clone());
        }
        let parameters = curve
            .retained_circle_point_parameters(point, policy)?
            .map(Arc::<[BezierParameter2]>::from);
        self.circular_point_parameters
            .push(CircularPointParameterCacheEntry {
                curve: curve.clone(),
                point: point.clone(),
                parameters: parameters.clone(),
            });
        Ok(parameters)
    }

    fn circular_parameter_table(
        &mut self,
        curves: &[RationalBezier2],
        relation: &CircleCircleRelation,
        policy: &CurveContext,
    ) -> CurveResult<Option<Vec<Vec<Classification<Arc<[BezierParameter2]>>>>>> {
        let mut table = Vec::with_capacity(curves.len());
        for curve in curves {
            let mut parameters = Vec::with_capacity(2);
            match relation {
                CircleCircleRelation::Tangent { point } => {
                    parameters.push(self.circular_point_parameters(curve, point, policy)?);
                }
                CircleCircleRelation::Secant {
                    first_point,
                    second_point,
                } => {
                    parameters.push(self.circular_point_parameters(curve, first_point, policy)?);
                    parameters.push(self.circular_point_parameters(curve, second_point, policy)?);
                }
                CircleCircleRelation::Coincident
                | CircleCircleRelation::Disjoint
                | CircleCircleRelation::Uncertain { .. } => return Ok(None),
            }
            table.push(parameters);
        }
        Ok(Some(table))
    }
}

fn retained_curve_circular_support(curve: &Curve2) -> Option<CircularSupportRef<'_>> {
    match curve.geometry() {
        Some(CurveGeometry2::CircularArc(curve))
            if curve.endpoints_on_stored_circle_are_certified() =>
        {
            Some(CircularSupportRef {
                center: curve.center(),
                radius_squared: curve.radius_squared_ref(),
            })
        }
        Some(CurveGeometry2::RationalQuadraticBezier(curve)) => curve
            .retained_circular_conic()
            .map(|support| CircularSupportRef {
                center: &support.center,
                radius_squared: &support.radius_squared,
            }),
        Some(CurveGeometry2::RationalBezier(curve)) => {
            curve
                .retained_circular_conic()
                .map(|support| CircularSupportRef {
                    center: &support.center,
                    radius_squared: &support.radius_squared,
                })
        }
        _ => None,
    }
}

#[derive(Debug)]
struct CurveIntersectionContextData {
    first: Curve2,
    second: Curve2,
    policy: CurveContext,
    span_pair_count: usize,
    dispatch: CurveIntersectionDispatch,
    result: OnceLock<ExactCurveResult<CurveIntersectionResult2>>,
}

#[derive(Debug)]
enum CurveIntersectionDispatch {
    SupportSelf(Option<Arc<UnitParallelSelfIntersections>>),
    SupportEvidence(CurveIntersectionResult2),
    RationalPairs(Vec<PreparedRationalPair>),
    NativeLine(LineLineIntersection),
    NativeLineArc {
        order: LineArcOrder,
        arc: CircularArc2,
        relation: LineArcIntersection,
    },
    NativeArcPoints {
        first_arc: CircularArc2,
        second_arc: CircularArc2,
        points: Vec<Point2>,
    },
    NativeCoincidentArcs {
        first_arc: CircularArc2,
        second_arc: CircularArc2,
    },
}

enum NativeArcIntersectionDispatch {
    Points {
        first_arc: CircularArc2,
        second_arc: CircularArc2,
        points: Vec<Point2>,
    },
    Coincident {
        first_arc: CircularArc2,
        second_arc: CircularArc2,
    },
}

#[derive(Debug)]
enum PreparedRationalPair {
    Rational(RationalBezierIntersectionContext),
    RetainedLineageOverlap {
        first_range: ParamRange,
        second_range: ParamRange,
        orientation: CurveOverlapOrientation2,
    },
    Blocked(UncertaintyReason),
}

fn prepare_rational_pairs(
    first_curve: &Curve2,
    second_curve: &Curve2,
    first_evaluators: &[RationalBezier2],
    second_evaluators: &[RationalBezier2],
    policy: &CurveContext,
    circle_relation: Option<&CircleCircleRelation>,
    mut batch_cache: Option<&mut CurveIntersectionBatchCache>,
) -> ExactCurveResult<Vec<PreparedRationalPair>> {
    let first_fragments =
        first_curve.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let second_fragments = second_curve
        .native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let first_circle_parameters = match (circle_relation, batch_cache.as_deref_mut()) {
        (Some(relation), Some(cache)) => cache
            .circular_parameter_table(first_evaluators, relation, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Intersection, first_curve.family(), cause)
            })?,
        _ => None,
    };
    let second_circle_parameters = match (circle_relation, batch_cache) {
        (Some(relation), Some(cache)) => cache
            .circular_parameter_table(second_evaluators, relation, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(
                    CurveOperation2::Intersection,
                    second_curve.family(),
                    cause,
                )
            })?,
        _ => None,
    };
    let shares_injective_lineage = first_curve.shares_certified_parameter_lineage(second_curve);
    let mut pairs = Vec::with_capacity(first_evaluators.len() * second_evaluators.len());
    for (first_span_index, first) in first_evaluators.iter().enumerate() {
        for (second_span_index, second) in second_evaluators.iter().enumerate() {
            let retained_overlap = if shares_injective_lineage {
                let (first_start, first_end) = first_fragments[first_span_index].parameter_range();
                let (second_start, second_end) =
                    second_fragments[second_span_index].parameter_range();
                oriented_param_range_overlap(
                    &ParamRange::new(
                        first_curve.lineage_parameter_at(first_start)?,
                        first_curve.lineage_parameter_at(first_end)?,
                    ),
                    &ParamRange::new(
                        second_curve.lineage_parameter_at(second_start)?,
                        second_curve.lineage_parameter_at(second_end)?,
                    ),
                    policy,
                )
            } else {
                Classification::Decided(None)
            };
            let retained_overlap = match retained_overlap {
                Classification::Decided(overlap) => overlap,
                Classification::Uncertain(reason) => {
                    pairs.push(PreparedRationalPair::Blocked(reason));
                    continue;
                }
            };
            if let Some(overlap) = retained_overlap {
                pairs.push(PreparedRationalPair::RetainedLineageOverlap {
                    first_range: overlap.first,
                    second_range: overlap.second,
                    orientation: if overlap.same_orientation {
                        CurveOverlapOrientation2::Same
                    } else {
                        CurveOverlapOrientation2::Reversed
                    },
                });
                continue;
            }
            let state = match RationalBezierIntersectionContext::try_new_with_circle_relation(
                first,
                second,
                policy,
                circle_relation,
                first_circle_parameters
                    .as_ref()
                    .map(|parameters| parameters[first_span_index].as_slice()),
                second_circle_parameters
                    .as_ref()
                    .map(|parameters| parameters[second_span_index].as_slice()),
            ) {
                Ok(intersection) => PreparedRationalPair::Rational(intersection),
                Err(ExactCurveError::Blocked(blocker)) => {
                    PreparedRationalPair::Blocked(blocker.reason())
                }
                Err(ExactCurveError::Invalid { cause, .. }) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        first_curve.family(),
                        cause,
                    ));
                }
            };
            pairs.push(state);
        }
    }
    Ok(pairs)
}

fn has_native_point_image_span(curve: &Curve2, policy: &CurveContext) -> ExactCurveResult<bool> {
    let strict = policy.strict_counterpart();
    Ok(curve
        .native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?
        .iter()
        .any(|fragment| {
            matches!(
                fragment.native_curve().point_image(&strict),
                Classification::Decided(Some(_))
            )
        }))
}

fn certified_singleton_aabb_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionResult2>> {
    let (Ok(first_bounds), Ok(second_bounds)) = (first.bounds(), second.bounds()) else {
        return Ok(None);
    };
    let point = match first_bounds.singleton_intersection(second_bounds) {
        Classification::Decided(Some(point)) => point,
        Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
    };
    // A singleton image intersection can have many parameter preimages.
    // Use the existing exact incidence authority for every authored span;
    // its injectivity certificate still admits the cheap endpoint case.
    // Point images use the common component publisher. An unresolved or
    // constant incidence never certifies a missing visit.
    let locations = |curve: &Curve2| -> ExactCurveResult<Option<Vec<CurveLocation2>>> {
        match curve.span_point_locations(&point, CurveOperation2::Intersection, policy)? {
            Classification::Decided(Some(locations)) => Ok(Some(locations)),
            Classification::Decided(None) | Classification::Uncertain(_) => Ok(None),
        }
    };
    let Some(first_locations) = locations(first)? else {
        return Ok(None);
    };
    let Some(second_locations) = locations(second)? else {
        return Ok(None);
    };
    let mut contacts = Vec::new();
    let point = CurvePoint2::from(point);
    for first in first_locations {
        for second in &second_locations {
            let contact = CurveIntersectionContact2 {
                first: first.clone(),
                second: second.clone(),
                point: point.clone(),
                certified_transverse: false,
                tangent_cross_sign: None,
            };
            match matching_contact_index(&contacts, &contact, policy) {
                Classification::Decided(None) => contacts.push(contact),
                Classification::Decided(Some(_)) => {}
                Classification::Uncertain(_) => return Ok(None),
            }
        }
    }
    Ok(Some(CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count: first
                .native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?
                .len()
                * second
                    .native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?
                    .len(),
            contacts: contacts.into(),
            overlaps: Arc::from([]),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    }))
}

fn native_line_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<LineLineIntersection>> {
    let (Some(first_line), Some(second_line)) = (
        first.geometry().and_then(affine_line_image),
        second.geometry().and_then(affine_line_image),
    ) else {
        return Ok(None);
    };
    first_line
        .intersect_line_with_policy(second_line, policy)
        .map(Some)
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Intersection, first.family(), cause)
        })
}

fn affine_line_image(geometry: &CurveGeometry2) -> Option<&crate::LineSeg2> {
    match geometry {
        CurveGeometry2::Line(line) => Some(line),
        // A line promoted with `QuadraticBezier2::from_line_segment` is exact
        // degree elevation: its local parameter is still the affine segment
        // parameter. Retaining that fact lets the canonical arrangement use
        // the native line solver without restoring a second region engine.
        CurveGeometry2::QuadraticBezier(curve) => curve.retained_exact_line_image(),
        _ => None,
    }
}

fn native_line_arc_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<(LineArcOrder, CircularArc2, LineArcIntersection)>> {
    let (order, line, arc) = if let (Some(line), Some(arc)) = (
        first.geometry().and_then(affine_line_image),
        materialized_circular_arc(second, policy)?,
    ) {
        (LineArcOrder::LineThenArc, line, arc)
    } else if let (Some(arc), Some(line)) = (
        materialized_circular_arc(first, policy)?,
        second.geometry().and_then(affine_line_image),
    ) {
        (LineArcOrder::ArcThenLine, line, arc)
    } else {
        return Ok(None);
    };
    let arc_curve = match order {
        LineArcOrder::LineThenArc => second,
        LineArcOrder::ArcThenLine => first,
    };
    if let Some(relation) = retained_tangent_line_arc_contact(arc_curve, line, &arc) {
        return Ok(Some((order, arc, relation)));
    }
    line.intersect_arc_with_policy(&arc, policy)
        .map(|relation| Some((order, arc, relation)))
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Intersection, first.family(), cause)
        })
}

fn retained_tangent_line_arc_contact(
    arc_curve: &Curve2,
    line: &crate::LineSeg2,
    arc: &CircularArc2,
) -> Option<LineArcIntersection> {
    let circle = match arc_curve.geometry() {
        Some(CurveGeometry2::RationalQuadraticBezier(curve)) => curve.retained_circular_conic(),
        Some(CurveGeometry2::RationalBezier(curve)) => curve.retained_circular_conic(),
        _ => None,
    }?;
    let contacts = circle.tangent_contacts.as_deref()?;
    for contact in contacts {
        let crate::rational_bezier::RationalQuadraticCircleTangentContact2::Line {
            line: certified_line,
            point,
        } = contact
        else {
            continue;
        };
        let same_line = certified_line == line
            || (certified_line.start() == line.end() && certified_line.end() == line.start());
        if !same_line {
            continue;
        }
        let line_param = if point == line.start() {
            Real::zero()
        } else if point == line.end() {
            Real::one()
        } else {
            continue;
        };
        let arc_param = if point == arc.start() {
            Real::zero()
        } else if point == arc.end() {
            Real::one()
        } else {
            continue;
        };
        return Some(LineArcIntersection::Point(LineArcIntersectionPoint {
            point: point.clone(),
            line_param,
            arc_param,
            kind: crate::IntersectionKind::Endpoint,
        }));
    }
    None
}

fn build_native_line_evidence(
    first: &Curve2,
    second: &Curve2,
    relation: &LineLineIntersection,
    policy: &CurveContext,
    span_pair_count: usize,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let first_fragment =
        &first.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?[0];
    let second_fragment =
        &second.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?[0];
    let contact =
        |first_parameter: Real, second_parameter: Real, point: Point2| CurveIntersectionContact2 {
            first: CurveLocation2 {
                span_index: 0,
                span_range: first_fragment.span_range().clone(),
                local_parameter: first_parameter.into(),
            },
            second: CurveLocation2 {
                span_index: 0,
                span_range: second_fragment.span_range().clone(),
                local_parameter: second_parameter.into(),
            },
            point: CurvePoint2::from(point),
            certified_transverse: false,
            tangent_cross_sign: None,
        };
    let (contacts, overlaps) = match relation {
        LineLineIntersection::None => (Vec::new(), Vec::new()),
        LineLineIntersection::Point {
            point,
            a_param,
            b_param,
            ..
        } => (
            vec![contact(a_param.clone(), b_param.clone(), point.clone())],
            Vec::new(),
        ),
        LineLineIntersection::Overlap {
            segment,
            a_range,
            b_range,
        } => {
            let orientation = match compare_reals(b_range.start(), b_range.end(), policy) {
                Some(std::cmp::Ordering::Less) => CurveOverlapOrientation2::Same,
                Some(std::cmp::Ordering::Greater) => CurveOverlapOrientation2::Reversed,
                Some(std::cmp::Ordering::Equal) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        first.family(),
                        CurveError::DegenerateOverlapRange,
                    ));
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Intersection,
                        first.family(),
                        UncertaintyReason::Ordering,
                    ));
                }
            };
            let contacts = if parameter_range_covers_unit(a_range, first, policy)?
                && parameter_range_covers_unit(b_range, first, policy)?
            {
                Vec::new()
            } else {
                vec![
                    contact(
                        a_range.start().clone(),
                        b_range.start().clone(),
                        segment.start().clone(),
                    ),
                    contact(
                        a_range.end().clone(),
                        b_range.end().clone(),
                        segment.end().clone(),
                    ),
                ]
            };
            (
                contacts,
                vec![CurveIntersectionOverlap2 {
                    first_span_index: 0,
                    second_span_index: 0,
                    first_range: CurveParameterRange2::new_validated(
                        a_range.start().clone().into(),
                        a_range.end().clone().into(),
                    ),
                    second_range: CurveParameterRange2::new_validated(
                        b_range.start().clone().into(),
                        b_range.end().clone().into(),
                    ),
                    orientation,
                    endpoint_inclusion: [true, true],
                    parameter_correspondence: CurveOverlapCorrespondence2::affine(a_range, b_range),
                }],
            )
        }
        LineLineIntersection::Uncertain { reason } => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                first.family(),
                *reason,
            ));
        }
    };
    Ok(CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count,
            contacts: contacts.into(),
            overlaps: overlaps.into(),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    })
}

fn build_native_line_arc_evidence(
    first: &Curve2,
    second: &Curve2,
    order: LineArcOrder,
    arc: &CircularArc2,
    relation: &LineArcIntersection,
    policy: &CurveContext,
    span_pair_count: usize,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let mut contacts = Vec::new();
    match relation {
        LineArcIntersection::None => {}
        LineArcIntersection::Point(hit) => {
            append_native_line_arc_contact(&mut contacts, first, second, order, arc, hit, policy)?;
        }
        LineArcIntersection::TwoPoints {
            first: first_hit,
            second: second_hit,
        } => {
            append_native_line_arc_contact(
                &mut contacts,
                first,
                second,
                order,
                arc,
                first_hit,
                policy,
            )?;
            append_native_line_arc_contact(
                &mut contacts,
                first,
                second,
                order,
                arc,
                second_hit,
                policy,
            )?;
        }
        LineArcIntersection::Uncertain { reason } => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                first.family(),
                *reason,
            ));
        }
    }
    Ok(CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count,
            contacts: contacts.into(),
            overlaps: Arc::from([]),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    })
}

fn append_native_line_arc_contact(
    contacts: &mut Vec<CurveIntersectionContact2>,
    first: &Curve2,
    second: &Curve2,
    order: LineArcOrder,
    arc: &CircularArc2,
    hit: &LineArcIntersectionPoint,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let (line, arc_curve) = match order {
        LineArcOrder::LineThenArc => (first, second),
        LineArcOrder::ArcThenLine => (second, first),
    };
    let line_fragment =
        &line.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?[0];
    let arc_fragments =
        arc_curve.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let arc_evaluators =
        arc_curve.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    let arc_span_indices = arc_span_indices_for_point(arc_curve, arc, &hit.point, policy)?;
    let contact_count = contacts.len();
    for arc_span_index in arc_span_indices {
        let line_parameter = CurveLocation2 {
            span_index: 0,
            span_range: line_fragment.span_range().clone(),
            local_parameter: hit.line_param.clone().into(),
        };
        let arc_parameter = CurveLocation2 {
            span_index: arc_span_index,
            span_range: arc_fragments[arc_span_index].span_range().clone(),
            local_parameter: native_arc_span_parameter(
                arc_curve,
                &arc_evaluators[arc_span_index],
                &hit.point,
                policy,
            )?
            .into(),
        };
        let (first_parameter, second_parameter) = match order {
            LineArcOrder::LineThenArc => (line_parameter, arc_parameter),
            LineArcOrder::ArcThenLine => (arc_parameter, line_parameter),
        };
        let candidate = CurveIntersectionContact2 {
            first: first_parameter,
            second: second_parameter,
            point: CurvePoint2::from(hit.point.clone()),
            certified_transverse: false,
            tangent_cross_sign: None,
        };
        match matching_contact_index(contacts, &candidate, policy) {
            Classification::Decided(Some(_)) => {}
            Classification::Decided(None) => contacts.push(candidate),
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    arc_curve.family(),
                    reason,
                ));
            }
        }
    }
    if contacts.len() == contact_count {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Intersection,
            arc_curve.family(),
            UncertaintyReason::Predicate,
        ));
    }
    Ok(())
}

fn parameter_range_covers_unit(
    range: &ParamRange,
    curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let (lower, upper) = match compare_reals(range.start(), range.end(), policy) {
        Some(std::cmp::Ordering::Less) => (range.start(), range.end()),
        Some(std::cmp::Ordering::Greater) => (range.end(), range.start()),
        Some(std::cmp::Ordering::Equal) => return Ok(false),
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                curve.family(),
                UncertaintyReason::Ordering,
            ));
        }
    };
    Ok(
        compare_reals(lower, &Real::zero(), policy) == Some(std::cmp::Ordering::Equal)
            && compare_reals(upper, &Real::one(), policy) == Some(std::cmp::Ordering::Equal),
    )
}

fn native_arc_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
    batch_cache: Option<&mut CurveIntersectionBatchCache>,
) -> ExactCurveResult<Option<NativeArcIntersectionDispatch>> {
    let (Some(first_arc), Some(second_arc)) = (
        materialized_circular_arc(first, policy)?,
        materialized_circular_arc(second, policy)?,
    ) else {
        return Ok(None);
    };
    let relation = match batch_cache
        .map(|cache| cache.circular_support_relation(first, second, policy))
        .transpose()?
        .flatten()
    {
        Some(relation) => relation,
        None => first_arc
            .circle_relation_with_policy(&second_arc, policy)
            .map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Intersection, first.family(), cause)
            })?,
    };
    let candidates = match relation {
        CircleCircleRelation::Coincident => {
            return Ok(Some(NativeArcIntersectionDispatch::Coincident {
                first_arc,
                second_arc,
            }));
        }
        CircleCircleRelation::Disjoint => {
            return Ok(Some(NativeArcIntersectionDispatch::Points {
                first_arc,
                second_arc,
                points: Vec::new(),
            }));
        }
        CircleCircleRelation::Tangent { point } => vec![point],
        CircleCircleRelation::Secant {
            first_point,
            second_point,
        } => vec![first_point, second_point],
        CircleCircleRelation::Uncertain { .. } => {
            return Ok(None);
        }
    };

    let mut points = Vec::with_capacity(candidates.len());
    for point in candidates {
        match (
            first_arc.contains_sweep_point_with_policy(&point, policy),
            second_arc.contains_sweep_point_with_policy(&point, policy),
        ) {
            (Classification::Decided(true), Classification::Decided(true)) => points.push(point),
            (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {}
            (Classification::Uncertain(_), _) | (_, Classification::Uncertain(_)) => {
                return Ok(None);
            }
        }
    }
    Ok(Some(NativeArcIntersectionDispatch::Points {
        first_arc,
        second_arc,
        points,
    }))
}

fn materialized_circular_arc(
    curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CircularArc2>> {
    let arc = match curve.geometry() {
        Some(CurveGeometry2::CircularArc(arc)) => return Ok(Some(arc.clone())),
        Some(CurveGeometry2::RationalQuadraticBezier(conic))
            if conic.retained_circular_conic().is_some() =>
        {
            crate::arc_bezier::rational_quadratic_circular_arc(conic, policy)
        }
        Some(CurveGeometry2::RationalBezier(conic))
            if conic.retained_circular_conic().is_some() =>
        {
            crate::arc_bezier::rational_bezier_circular_arc(conic, policy)
        }
        _ => return Ok(None),
    }
    .map_err(|cause| {
        ExactCurveError::invalid(CurveOperation2::Intersection, curve.family(), cause)
    })?;
    Ok(match arc {
        Classification::Decided(arc) => arc,
        Classification::Uncertain(_) => None,
    })
}

fn build_native_arc_evidence(
    first: &Curve2,
    second: &Curve2,
    first_arc: &CircularArc2,
    second_arc: &CircularArc2,
    points: &[Point2],
    policy: &CurveContext,
    span_pair_count: usize,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let first_fragments =
        first.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let second_fragments =
        second.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let first_evaluators =
        first.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    let second_evaluators =
        second.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    let mut contacts = Vec::new();
    for point in points {
        let first_span_indices = arc_span_indices_for_point(first, first_arc, point, policy)?;
        let second_span_indices = arc_span_indices_for_point(second, second_arc, point, policy)?;
        let contact_count = contacts.len();
        for &first_span_index in &first_span_indices {
            for &second_span_index in &second_span_indices {
                let candidate = CurveIntersectionContact2 {
                    first: CurveLocation2 {
                        span_index: first_span_index,
                        span_range: first_fragments[first_span_index].span_range().clone(),
                        local_parameter: native_arc_span_parameter(
                            first,
                            &first_evaluators[first_span_index],
                            point,
                            policy,
                        )?
                        .into(),
                    },
                    second: CurveLocation2 {
                        span_index: second_span_index,
                        span_range: second_fragments[second_span_index].span_range().clone(),
                        local_parameter: native_arc_span_parameter(
                            second,
                            &second_evaluators[second_span_index],
                            point,
                            policy,
                        )?
                        .into(),
                    },
                    point: CurvePoint2::from(point.clone()),
                    certified_transverse: false,
                    tangent_cross_sign: None,
                };
                match matching_contact_index(&contacts, &candidate, policy) {
                    Classification::Decided(Some(_)) => {}
                    Classification::Decided(None) => contacts.push(candidate),
                    Classification::Uncertain(reason) => {
                        return Err(ExactCurveError::blocked(
                            CurveOperation2::Intersection,
                            first.family(),
                            reason,
                        ));
                    }
                }
            }
        }
        if contacts.len() == contact_count {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                first.family(),
                UncertaintyReason::Predicate,
            ));
        }
    }
    Ok(CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count,
            contacts: contacts.into(),
            overlaps: Arc::from([]),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    })
}

fn build_native_coincident_arc_evidence(
    first: &Curve2,
    second: &Curve2,
    first_arc: &CircularArc2,
    second_arc: &CircularArc2,
    policy: &CurveContext,
    span_pair_count: usize,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let first_fragments =
        first.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let second_fragments =
        second.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let first_evaluators =
        first.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    let second_evaluators =
        second.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    let mut contacts = Vec::new();
    let mut overlaps = Vec::new();

    for (first_span_index, first_fragment) in first_fragments.iter().enumerate() {
        let (first_start, first_end) = first_fragment.native_curve().endpoints();
        let first_span = if first_fragments.len() == 1 {
            first_arc.clone()
        } else {
            CircularArc2::new_with_certified_radius(
                first_start.clone(),
                first_end.clone(),
                first_arc.center().clone(),
                first_arc.radius_squared(),
                first_arc.is_clockwise(),
                None,
            )
        };
        for (second_span_index, second_fragment) in second_fragments.iter().enumerate() {
            let (second_start, second_end) = second_fragment.native_curve().endpoints();
            let second_span = if second_fragments.len() == 1 {
                second_arc.clone()
            } else {
                CircularArc2::new_with_certified_radius(
                    second_start.clone(),
                    second_end.clone(),
                    first_arc.center().clone(),
                    first_arc.radius_squared(),
                    second_arc.is_clockwise(),
                    None,
                )
            };
            let relation = first_span
                .intersect_arc_with_policy(&second_span, policy)
                .map_err(|cause| native_arc_parameter_error(first, cause))?;
            match relation {
                ArcArcIntersection::None => {}
                ArcArcIntersection::Point(hit) => append_native_arc_span_contact(
                    &mut contacts,
                    first,
                    second,
                    first_span_index,
                    second_span_index,
                    &hit.point,
                    policy,
                )?,
                ArcArcIntersection::TwoPoints {
                    first: first_hit,
                    second: second_hit,
                } => {
                    append_native_arc_span_contact(
                        &mut contacts,
                        first,
                        second,
                        first_span_index,
                        second_span_index,
                        &first_hit.point,
                        policy,
                    )?;
                    append_native_arc_span_contact(
                        &mut contacts,
                        first,
                        second,
                        first_span_index,
                        second_span_index,
                        &second_hit.point,
                        policy,
                    )?;
                }
                ArcArcIntersection::Overlap { segment, .. } => {
                    let first_range = native_arc_overlap_range(
                        first,
                        &first_evaluators[first_span_index],
                        &segment,
                        policy,
                    )?;
                    let second_range = native_arc_overlap_range(
                        second,
                        &second_evaluators[second_span_index],
                        &segment,
                        policy,
                    )?;
                    let orientation = match second_range
                        .start()
                        .cmp_by_interval(second_range.end(), policy)
                        .map_err(|cause| native_arc_parameter_error(second, cause))?
                    {
                        Classification::Decided(std::cmp::Ordering::Less) => {
                            CurveOverlapOrientation2::Same
                        }
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            CurveOverlapOrientation2::Reversed
                        }
                        Classification::Decided(std::cmp::Ordering::Equal) => {
                            return Err(native_arc_parameter_error(
                                second,
                                CurveError::DegenerateOverlapRange,
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            return Err(ExactCurveError::blocked(
                                CurveOperation2::Intersection,
                                second.family(),
                                reason,
                            ));
                        }
                    };
                    if !(bezier_parameter_range_covers_unit(&first_range, first, policy)?
                        && bezier_parameter_range_covers_unit(&second_range, second, policy)?)
                    {
                        append_native_arc_span_contact(
                            &mut contacts,
                            first,
                            second,
                            first_span_index,
                            second_span_index,
                            segment.start(),
                            policy,
                        )?;
                        append_native_arc_span_contact(
                            &mut contacts,
                            first,
                            second,
                            first_span_index,
                            second_span_index,
                            segment.end(),
                            policy,
                        )?;
                    }
                    let correspondence = CurveOverlapCorrespondence2::for_rational_ranges(
                        &first_evaluators[first_span_index],
                        &second_evaluators[second_span_index],
                        &first_range,
                        &second_range,
                        orientation,
                        policy,
                    );
                    overlaps.push(CurveIntersectionOverlap2 {
                        first_span_index,
                        second_span_index,
                        first_range: CurveParameterRange2::from_bezier_range(first_range),
                        second_range: CurveParameterRange2::from_bezier_range(second_range),
                        orientation,
                        endpoint_inclusion: [true, true],
                        parameter_correspondence: correspondence,
                    });
                }
                ArcArcIntersection::Uncertain { reason } => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Intersection,
                        first.family(),
                        reason,
                    ));
                }
            }
        }
    }

    Ok(CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count,
            contacts: contacts.into(),
            overlaps: overlaps.into(),
            blockers: Arc::from([]),
            parameter_components: None,
        }),
    })
}

#[allow(clippy::too_many_arguments)]
fn append_native_arc_span_contact(
    contacts: &mut Vec<CurveIntersectionContact2>,
    first: &Curve2,
    second: &Curve2,
    first_span_index: usize,
    second_span_index: usize,
    point: &Point2,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let first_fragments =
        first.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let second_fragments =
        second.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let first_evaluators =
        first.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    let second_evaluators =
        second.rational_evaluators_for_operation(policy, CurveOperation2::Intersection)?;
    let candidate = CurveIntersectionContact2 {
        first: CurveLocation2 {
            span_index: first_span_index,
            span_range: first_fragments[first_span_index].span_range().clone(),
            local_parameter: native_arc_span_parameter(
                first,
                &first_evaluators[first_span_index],
                point,
                policy,
            )?
            .into(),
        },
        second: CurveLocation2 {
            span_index: second_span_index,
            span_range: second_fragments[second_span_index].span_range().clone(),
            local_parameter: native_arc_span_parameter(
                second,
                &second_evaluators[second_span_index],
                point,
                policy,
            )?
            .into(),
        },
        point: CurvePoint2::from(point.clone()),
        certified_transverse: false,
        tangent_cross_sign: None,
    };
    match matching_contact_index(contacts, &candidate, policy) {
        Classification::Decided(Some(_)) => {}
        Classification::Decided(None) => contacts.push(candidate),
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                first.family(),
                reason,
            ));
        }
    }
    Ok(())
}

fn native_arc_overlap_range(
    curve: &Curve2,
    evaluator: &RationalBezier2,
    overlap: &CircularArc2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierParameterRange2> {
    let start = native_arc_span_parameter(curve, evaluator, overlap.start(), policy)?;
    let end = native_arc_span_parameter(curve, evaluator, overlap.end(), policy)?;
    match BezierParameterRange2::try_new(start, end, policy)
        .map_err(|cause| native_arc_parameter_error(curve, cause))?
    {
        Classification::Decided(range) => Ok(range),
        Classification::Uncertain(reason) => Err(ExactCurveError::blocked(
            CurveOperation2::Intersection,
            curve.family(),
            reason,
        )),
    }
}

fn bezier_parameter_range_covers_unit(
    range: &BezierParameterRange2,
    curve: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<bool> {
    let order = range
        .start()
        .cmp_by_interval(range.end(), policy)
        .map_err(|cause| native_arc_parameter_error(curve, cause))?;
    let (lower, upper) = match order {
        Classification::Decided(std::cmp::Ordering::Less) => (range.start(), range.end()),
        Classification::Decided(std::cmp::Ordering::Greater) => (range.end(), range.start()),
        Classification::Decided(std::cmp::Ordering::Equal) => return Ok(false),
        Classification::Uncertain(reason) => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                curve.family(),
                reason,
            ));
        }
    };
    let zero = BezierParameter2::Exact(Real::zero());
    let one = BezierParameter2::Exact(Real::one());
    let lower_is_zero = lower
        .same_value(&zero, policy)
        .map_err(|cause| native_arc_parameter_error(curve, cause))?;
    let upper_is_one = upper
        .same_value(&one, policy)
        .map_err(|cause| native_arc_parameter_error(curve, cause))?;
    match (lower_is_zero, upper_is_one) {
        (Classification::Decided(lower), Classification::Decided(upper)) => Ok(lower && upper),
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => Err(
            ExactCurveError::blocked(CurveOperation2::Intersection, curve.family(), reason),
        ),
    }
}

fn arc_span_indices_for_point(
    curve: &Curve2,
    arc: &CircularArc2,
    point: &Point2,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<usize>> {
    let fragments =
        curve.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    if fragments.len() == 1 {
        return Ok(vec![0]);
    }
    let mut indices = Vec::new();
    for (span_index, fragment) in fragments.iter().enumerate() {
        let (start, end) = fragment.native_curve().endpoints();
        let span = CircularArc2::new_with_certified_radius(
            start,
            end,
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        );
        match span.contains_sweep_point_with_policy(point, policy) {
            Classification::Decided(true) => indices.push(span_index),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    curve.family(),
                    reason,
                ));
            }
        }
    }
    Ok(indices)
}

fn native_arc_span_parameter(
    curve: &Curve2,
    span: &RationalBezier2,
    point: &Point2,
    policy: &CurveContext,
) -> ExactCurveResult<BezierParameter2> {
    if span.degree() != 2 {
        let mut parameters = match span
            .retained_circle_point_parameters(point, policy)
            .map_err(|cause| native_arc_parameter_error(curve, cause))?
        {
            Classification::Decided(parameters) => parameters,
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    curve.family(),
                    reason,
                ));
            }
        };
        if parameters.len() == 1 {
            return Ok(parameters.pop().expect("one retained circle parameter"));
        }
        return Err(if parameters.is_empty() {
            native_arc_parameter_error(
                curve,
                CurveError::Topology(
                    "retained circular parameterization omitted a certified arc point".into(),
                ),
            )
        } else {
            ExactCurveError::blocked(
                CurveOperation2::Intersection,
                curve.family(),
                UncertaintyReason::Boundary,
            )
        });
    }
    if crate::classify::is_zero(&span.start().distance_squared(point), policy) == Some(true) {
        return Ok(BezierParameter2::Exact(Real::zero()));
    }
    if crate::classify::is_zero(&span.end().distance_squared(point), policy) == Some(true) {
        return Ok(BezierParameter2::Exact(Real::one()));
    }

    let controls = span.homogeneous_controls();
    let relative = |control: &crate::HomogeneousControl2| {
        (
            control.x() - point.x() * control.weight(),
            control.y() - point.y() * control.weight(),
        )
    };
    let p0 = relative(&controls[0]);
    let p1 = relative(&controls[1]);
    let p2 = relative(&controls[2]);
    let beta2_scaled = &p0.0 * &p1.1 - &p0.1 * &p1.0;
    let beta0_scaled = &p1.0 * &p2.1 - &p1.1 * &p2.0;
    let ratio_squared = (beta2_scaled / beta0_scaled)
        .map_err(|cause| native_arc_parameter_error(curve, cause.into()))?;
    let ratio = ratio_squared
        .sqrt()
        .map_err(|cause| native_arc_parameter_error(curve, cause.into()))?;
    let parameter = (&ratio / (Real::one() + &ratio))
        .map_err(|cause| native_arc_parameter_error(curve, cause.into()))?;
    Ok(BezierParameter2::Exact(parameter))
}

fn native_arc_parameter_error(curve: &Curve2, cause: CurveError) -> ExactCurveError {
    ExactCurveError::invalid(CurveOperation2::Intersection, curve.family(), cause)
}

impl Curve2 {
    /// Computes exact contact, overlap, and blocker evidence against another
    /// curve immediately.
    ///
    /// The returned [`CurveOutcome`] records whether the complete promotion,
    /// dispatch, and replay consumed the `APPROXIMATE_512` terminal.
    pub fn intersect_curve(
        &self,
        other: &Self,
    ) -> crate::ExactCurveResult<CurveIntersectionResult2> {
        self.intersect_curve_with_policy(other, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::intersect_curve`] under an explicit predicate policy.
    pub(crate) fn intersect_curve_with_policy(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveIntersectionResult2>> {
        resolve_certified_operation(policy, |attempt| self.intersect_curve_raw(other, attempt))
    }

    /// Computes exact off-diagonal self-contact, retracing, and blocker
    /// evidence immediately.
    ///
    /// Both sides of the result address this curve. Each unordered contact
    /// and retracing component is reported once; shared span joints are the
    /// identity, not contacts, while a closed seam joins distinct parameters
    /// and is reported.
    pub fn self_intersections(&self) -> crate::ExactCurveResult<CurveIntersectionResult2> {
        self.self_intersections_with_policy(&crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::self_intersections`] under an explicit predicate policy.
    pub(crate) fn self_intersections_with_policy(
        &self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveIntersectionResult2>> {
        resolve_certified_operation(policy, |attempt| {
            CurveIntersectionContext::new_self(
                self,
                attempt,
                &mut CurveIntersectionBatchCache::default(),
            )
            .result()
        })
    }

    pub(crate) fn intersect_curve_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveIntersectionResult2> {
        CurveIntersectionContext::try_new(self, other, policy)?.result()
    }

    /// Computes exact split topology against another curve immediately and
    /// reports any consumed terminal decision once.
    pub fn intersection_topology(
        &self,
        other: &Self,
    ) -> crate::ExactCurveResult<CurveIntersectionTopology2> {
        self.intersection_topology_with_policy(other, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::intersection_topology`] under an explicit predicate policy.
    pub(crate) fn intersection_topology_with_policy(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveIntersectionTopology2>> {
        resolve_certified_operation(policy, |attempt| {
            self.intersection_topology_raw(other, attempt)
        })
    }

    pub(crate) fn intersection_topology_raw(
        &self,
        other: &Self,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveIntersectionTopology2> {
        CurveIntersectionContext::try_new(self, other, policy)?.topology()
    }
}

/// Intersects two curves with identical native single-span Bezier geometry.
///
/// Equal exact definitions are the same point set in the same chart, so they
/// meet in the identity overlap of the whole domain and in the curve's own
/// off-diagonal self contacts. This is a positive certificate from shared
/// definition: it needs no implicitization, whose coefficients can be nested
/// surds (for example a conic cut at irrational parameters) that refinement
/// cannot decide. A quadratic, or a positive-weight rational quadratic, with a
/// nondegenerate control triangle is an injective conic arc and has no self
/// contacts; other geometries take their self-intersection query. Distinct
/// definitions decline and keep the general dispatch.
fn identical_native_bezier_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionResult2>> {
    let (Some(geometry), Some(other)) = (first.geometry(), second.geometry()) else {
        return Ok(None);
    };
    let conic_controls = match geometry {
        CurveGeometry2::QuadraticBezier(curve) => Some(curve.control_points()),
        CurveGeometry2::RationalQuadraticBezier(curve) => curve
            .weights()
            .iter()
            .all(|weight| {
                crate::classify::real_sign(weight, &policy.strict_counterpart())
                    == Some(hyperreal::RealSign::Positive)
            })
            .then(|| curve.control_points()),
        CurveGeometry2::CubicBezier(_) | CurveGeometry2::RationalBezier(_) => None,
        CurveGeometry2::Line(_)
        | CurveGeometry2::CircularArc(_)
        | CurveGeometry2::PolynomialBSpline(_)
        | CurveGeometry2::Nurbs(_) => return Ok(None),
    };
    // A constant image meets itself in a whole parameter fiber, not an
    // overlap; its point-image dispatch owns that result.
    if geometry != other || has_native_point_image_span(first, policy)? {
        return Ok(None);
    }
    let injective_conic = conic_controls.is_some_and(|[start, control, end]| {
        let cross = (control.x() - start.x()) * (end.y() - start.y())
            - (control.y() - start.y()) * (end.x() - start.x());
        matches!(
            crate::classify::real_sign(&cross, &policy.strict_counterpart()),
            Some(hyperreal::RealSign::Positive | hyperreal::RealSign::Negative)
        )
    });
    let (contacts, mut overlaps, blockers, parameter_components) = if injective_conic {
        (Arc::from([]), Vec::new(), Arc::from([]), None)
    } else {
        let own = CurveIntersectionContext::new_self(
            first,
            policy,
            &mut CurveIntersectionBatchCache::default(),
        )
        .result()?;
        // Retracing self overlaps would need their correspondence transported
        // to both orders; leave those curves to the general dispatch.
        if !own.data.overlaps.is_empty() {
            return Ok(None);
        }
        // A self contact is unordered; a pair reports both ordered visits.
        let contacts = own
            .data
            .contacts
            .iter()
            .flat_map(|contact| {
                let swapped = CurveIntersectionContact2 {
                    first: contact.second.clone(),
                    second: contact.first.clone(),
                    point: contact.point.clone(),
                    certified_transverse: contact.certified_transverse,
                    tangent_cross_sign: contact.tangent_cross_sign.map(|sign| match sign {
                        hyperreal::RealSign::Positive => hyperreal::RealSign::Negative,
                        hyperreal::RealSign::Negative => hyperreal::RealSign::Positive,
                        hyperreal::RealSign::Zero => hyperreal::RealSign::Zero,
                    }),
                };
                [contact.clone(), swapped]
            })
            .collect::<Vec<_>>();
        (
            Arc::from(contacts),
            Vec::new(),
            own.data.blockers.clone(),
            own.data.parameter_components.clone(),
        )
    };
    let domain = ParamRange::new(Real::zero(), Real::one());
    overlaps.push(CurveIntersectionOverlap2 {
        first_span_index: 0,
        second_span_index: 0,
        first_range: CurveParameterRange2::from_bezier_range(BezierParameterRange2::new_validated(
            BezierParameter2::Exact(Real::zero()),
            BezierParameter2::Exact(Real::one()),
        )),
        second_range: CurveParameterRange2::from_bezier_range(
            BezierParameterRange2::new_validated(
                BezierParameter2::Exact(Real::zero()),
                BezierParameter2::Exact(Real::one()),
            ),
        ),
        orientation: crate::CurveOverlapOrientation2::Same,
        endpoint_inclusion: [true, true],
        parameter_correspondence: CurveOverlapCorrespondence2::affine(&domain, &domain),
    });
    Ok(Some(CurveIntersectionResult2 {
        data: Arc::new(CurveIntersectionResultData {
            span_pair_count: 1,
            contacts,
            overlaps: overlaps.into(),
            blockers,
            parameter_components,
        }),
    }))
}

impl CurveIntersectionContext {
    pub(crate) fn try_new(
        first: &Curve2,
        second: &Curve2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_batch_cache(first, second, policy, None)
    }

    /// Retains one prepared support query; algebra is evaluated only on demand.
    pub(crate) fn new_self(
        curve: &Curve2,
        policy: &CurveContext,
        batch_cache: &mut CurveIntersectionBatchCache,
    ) -> Self {
        let unit_parallel = curve.retained_fragment().and_then(|fragment| {
            match crate::curve_support::CurveSupport2::from_fragment(fragment) {
                crate::curve_support::CurveSupport2::Parallel(source) => {
                    Some(batch_cache.unit_parallel_self_intersections(source, policy))
                }
                _ => None,
            }
        });
        Self {
            data: CurveIntersectionContextData {
                first: curve.clone(),
                second: curve.clone(),
                policy: *policy,
                span_pair_count: 1,
                dispatch: CurveIntersectionDispatch::SupportSelf(unit_parallel),
                result: OnceLock::new(),
            },
        }
    }

    pub(crate) fn try_new_with_batch_cache(
        first: &Curve2,
        second: &Curve2,
        policy: &CurveContext,
        batch_cache: &mut CurveIntersectionBatchCache,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_batch_cache(first, second, policy, Some(batch_cache))
    }

    fn try_new_with_optional_batch_cache(
        first: &Curve2,
        second: &Curve2,
        policy: &CurveContext,
        mut batch_cache: Option<&mut CurveIntersectionBatchCache>,
    ) -> ExactCurveResult<Self> {
        if first.geometry().is_none() || second.geometry().is_none() {
            let result = curve_support_intersection::intersect(first, second, policy, None)?;
            return Ok(Self {
                data: CurveIntersectionContextData {
                    first: first.clone(),
                    second: second.clone(),
                    policy: *policy,
                    span_pair_count: result.span_pair_count(),
                    dispatch: CurveIntersectionDispatch::SupportEvidence(result),
                    result: OnceLock::new(),
                },
            });
        }
        if let Some(result) = identical_native_bezier_intersection(first, second, policy)? {
            return Ok(Self {
                data: CurveIntersectionContextData {
                    first: first.clone(),
                    second: second.clone(),
                    policy: *policy,
                    span_pair_count: result.span_pair_count(),
                    dispatch: CurveIntersectionDispatch::SupportEvidence(result),
                    result: OnceLock::new(),
                },
            });
        }
        let (span_pair_count, dispatch) = match native_line_intersection(first, second, policy)? {
            Some(relation) => (1, CurveIntersectionDispatch::NativeLine(relation)),
            None => {
                if let Some((order, arc, relation)) =
                    native_line_arc_intersection(first, second, policy)?
                {
                    let span_pair_count = first
                        .native_bezier_fragments_for_operation(
                            policy,
                            CurveOperation2::Intersection,
                        )?
                        .len()
                        * second
                            .native_bezier_fragments_for_operation(
                                policy,
                                CurveOperation2::Intersection,
                            )?
                            .len();
                    (
                        span_pair_count,
                        CurveIntersectionDispatch::NativeLineArc {
                            order,
                            arc,
                            relation,
                        },
                    )
                } else {
                    match native_arc_intersection(
                        first,
                        second,
                        policy,
                        batch_cache.as_deref_mut(),
                    )? {
                        Some(native) => {
                            let span_pair_count = first
                                .native_bezier_fragments_for_operation(
                                    policy,
                                    CurveOperation2::Intersection,
                                )?
                                .len()
                                * second
                                    .native_bezier_fragments_for_operation(
                                        policy,
                                        CurveOperation2::Intersection,
                                    )?
                                    .len();
                            let dispatch = match native {
                                NativeArcIntersectionDispatch::Points {
                                    first_arc,
                                    second_arc,
                                    points,
                                } => CurveIntersectionDispatch::NativeArcPoints {
                                    first_arc,
                                    second_arc,
                                    points,
                                },
                                NativeArcIntersectionDispatch::Coincident {
                                    first_arc,
                                    second_arc,
                                } => CurveIntersectionDispatch::NativeCoincidentArcs {
                                    first_arc,
                                    second_arc,
                                },
                            };
                            (span_pair_count, dispatch)
                        }
                        None => {
                            if has_native_point_image_span(first, policy)?
                                || has_native_point_image_span(second, policy)?
                            {
                                let result = curve_support_intersection::intersect(
                                    first, second, policy, None,
                                )?;
                                (
                                    result.span_pair_count(),
                                    CurveIntersectionDispatch::SupportEvidence(result),
                                )
                            } else if let Some(result) =
                                certified_singleton_aabb_intersection(first, second, policy)?
                            {
                                (
                                    result.span_pair_count(),
                                    CurveIntersectionDispatch::SupportEvidence(result),
                                )
                            } else {
                                let first_evaluators = first.rational_evaluators_for_operation(
                                    policy,
                                    CurveOperation2::Intersection,
                                )?;
                                let second_evaluators = second.rational_evaluators_for_operation(
                                    policy,
                                    CurveOperation2::Intersection,
                                )?;
                                let span_pair_count =
                                    first_evaluators.len() * second_evaluators.len();
                                let circle_relation = match batch_cache.as_deref_mut() {
                                    Some(cache) => {
                                        cache.circular_support_relation(first, second, policy)?
                                    }
                                    None => None,
                                };
                                let dispatch = CurveIntersectionDispatch::RationalPairs(
                                    prepare_rational_pairs(
                                        first,
                                        second,
                                        first_evaluators,
                                        second_evaluators,
                                        policy,
                                        circle_relation.as_ref(),
                                        batch_cache,
                                    )?,
                                );
                                (span_pair_count, dispatch)
                            }
                        }
                    }
                }
            }
        };
        Ok(Self {
            data: CurveIntersectionContextData {
                first: first.clone(),
                second: second.clone(),
                policy: *policy,
                span_pair_count,
                dispatch,
                result: OnceLock::new(),
            },
        })
    }

    pub(crate) fn result(&self) -> ExactCurveResult<CurveIntersectionResult2> {
        self.result_view().cloned()
    }

    pub(crate) fn result_view(&self) -> ExactCurveResult<&CurveIntersectionResult2> {
        match self.data.result.get_or_init(|| self.build_evidence()) {
            Ok(result) => Ok(result),
            Err(error) => Err(error.clone()),
        }
    }

    pub(crate) fn topology(&self) -> ExactCurveResult<CurveIntersectionTopology2> {
        self.build_topology()
    }

    fn build_evidence(&self) -> ExactCurveResult<CurveIntersectionResult2> {
        if let CurveIntersectionDispatch::SupportSelf(unit_parallel) = &self.data.dispatch {
            return curve_support_intersection::self_intersections(
                &self.data.first,
                &self.data.policy,
                unit_parallel.as_deref(),
            );
        }
        if let CurveIntersectionDispatch::SupportEvidence(result) = &self.data.dispatch {
            return Ok(result.clone());
        }
        if let CurveIntersectionDispatch::NativeLine(relation) = &self.data.dispatch {
            return build_native_line_evidence(
                &self.data.first,
                &self.data.second,
                relation,
                &self.data.policy,
                self.data.span_pair_count,
            );
        }
        if let CurveIntersectionDispatch::NativeLineArc {
            order,
            arc,
            relation,
        } = &self.data.dispatch
        {
            return build_native_line_arc_evidence(
                &self.data.first,
                &self.data.second,
                *order,
                arc,
                relation,
                &self.data.policy,
                self.data.span_pair_count,
            );
        }
        if let CurveIntersectionDispatch::NativeArcPoints {
            first_arc,
            second_arc,
            points,
        } = &self.data.dispatch
        {
            return build_native_arc_evidence(
                &self.data.first,
                &self.data.second,
                first_arc,
                second_arc,
                points,
                &self.data.policy,
                self.data.span_pair_count,
            );
        }
        if let CurveIntersectionDispatch::NativeCoincidentArcs {
            first_arc,
            second_arc,
        } = &self.data.dispatch
        {
            return build_native_coincident_arc_evidence(
                &self.data.first,
                &self.data.second,
                first_arc,
                second_arc,
                &self.data.policy,
                self.data.span_pair_count,
            );
        }
        let CurveIntersectionDispatch::RationalPairs(pairs) = &self.data.dispatch else {
            unreachable!("native dispatch returned before common span replay")
        };
        curve_support_intersection::intersect(
            &self.data.first,
            &self.data.second,
            &self.data.policy,
            Some(pairs),
        )
    }

    fn build_topology(&self) -> ExactCurveResult<CurveIntersectionTopology2> {
        let result = self.result_view()?.clone();
        if let Some(blocker) = result.blockers().first() {
            let reason = match blocker.kind() {
                CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                CurveIntersectionPairBlockerKind2::IncompleteReplay => UncertaintyReason::Predicate,
                CurveIntersectionPairBlockerKind2::SharedComponent => UncertaintyReason::Boundary,
            };
            return Err(ExactCurveError::blocked(
                CurveOperation2::Arrangement,
                self.data.first.family(),
                reason,
            ));
        }
        let first_parameters = result
            .contacts()
            .iter()
            .map(|contact| {
                (
                    contact.first().span_index(),
                    contact.first().local_parameter().clone(),
                )
            })
            .chain(result.overlaps().iter().flat_map(|overlap| {
                [
                    (
                        overlap.first_span_index(),
                        overlap.first_range().start().clone(),
                    ),
                    (
                        overlap.first_span_index(),
                        overlap.first_range().end().clone(),
                    ),
                ]
            }));
        let first_parameters =
            first_parameters.chain(result.parameter_components().iter().flat_map(|component| {
                component
                    .first_parameters()
                    .boundaries()
                    .map(|parameter| (component.first_span_index(), parameter.clone()))
            }));
        let first = split_curve(&self.data.first, first_parameters, &self.data.policy)?;
        let second_parameters = result
            .contacts()
            .iter()
            .map(|contact| {
                (
                    contact.second().span_index(),
                    contact.second().local_parameter().clone(),
                )
            })
            .chain(result.overlaps().iter().flat_map(|overlap| {
                [
                    (
                        overlap.second_span_index(),
                        overlap.second_range().start().clone(),
                    ),
                    (
                        overlap.second_span_index(),
                        overlap.second_range().end().clone(),
                    ),
                ]
            }));
        let second_parameters =
            second_parameters.chain(result.parameter_components().iter().flat_map(|component| {
                component
                    .second_parameters()
                    .boundaries()
                    .map(|parameter| (component.second_span_index(), parameter.clone()))
            }));
        let second = split_curve(&self.data.second, second_parameters, &self.data.policy)?;
        Ok(CurveIntersectionTopology2 {
            data: Arc::new(CurveIntersectionTopologyData {
                result,
                first: first.into(),
                second: second.into(),
            }),
        })
    }
}

impl Curve2 {
    /// Locates every exact parameter at which this curve passes through a point.
    ///
    /// Each span replays its exact incidence equations; roots remain
    /// represented values or certified isolators, and no parameter is
    /// rounded. Distinct visits of the same point are all reported. The
    /// query point must have represented coordinates; a retained algebraic
    /// point or a generated carrier without a rational span evaluator reports
    /// an unsupported blocker rather than an empty result.
    pub fn point_locations(
        &self,
        point: &CurvePoint2,
    ) -> crate::ExactCurveResult<CurvePointLocations2> {
        self.point_locations_with_policy(point, &crate::policy::principal_context())
            .map(crate::CurveOutcome::into_value)
    }

    /// [`Self::point_locations`] under an explicit predicate policy.
    pub(crate) fn point_locations_with_policy(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurvePointLocations2>> {
        resolve_certified_operation(policy, |attempt| self.point_locations_raw(point, attempt))
    }

    pub(crate) fn point_locations_raw(
        &self,
        point: &CurvePoint2,
        policy: &CurveContext,
    ) -> ExactCurveResult<CurvePointLocations2> {
        let blocked =
            |reason| ExactCurveError::blocked(CurveOperation2::Intersection, self.family(), reason);
        let Some(point) = point.coordinates() else {
            return Err(blocked(UncertaintyReason::Unsupported));
        };
        let spans = match self.span_point_locations(point, CurveOperation2::Intersection, policy)? {
            Classification::Decided(Some(locations)) => locations,
            Classification::Decided(None) => return Ok(CurvePointLocations2::EntireCurve),
            Classification::Uncertain(reason) => return Err(blocked(reason)),
        };
        // A continuous spline seam is the end of one span and the start of
        // the next; both describe one authored parameter.
        let mut locations: Vec<CurveLocation2> = Vec::with_capacity(spans.len());
        for location in spans {
            let parameter = location.parameter(policy).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Intersection, self.family(), cause)
            })?;
            let parameter = match parameter {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => return Err(blocked(reason)),
            };
            let mut duplicate = false;
            if let Some(previous_location) = locations.last() {
                let previous = match previous_location.parameter(policy).map_err(|cause| {
                    ExactCurveError::invalid(CurveOperation2::Intersection, self.family(), cause)
                })? {
                    Classification::Decided(parameter) => parameter,
                    Classification::Uncertain(reason) => return Err(blocked(reason)),
                };
                duplicate = previous_location.span_index != location.span_index
                    && match previous.same_value(&parameter, policy).map_err(|cause| {
                        ExactCurveError::invalid(
                            CurveOperation2::Intersection,
                            self.family(),
                            cause,
                        )
                    })? {
                        Classification::Decided(same) => same,
                        Classification::Uncertain(reason) => return Err(blocked(reason)),
                    };
            }
            if !duplicate {
                locations.push(location);
            }
        }
        Ok(CurvePointLocations2::Locations(locations))
    }

    /// Returns per-span incidence locations, or `None` when every span is
    /// constant at the point. Authored spans replay their rational incidence
    /// equations; generated parallels, chords, and selected circles use their
    /// own exact incidence authorities on their retained source ranges.
    fn span_point_locations(
        &self,
        point: &Point2,
        operation: CurveOperation2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Classification<Option<Vec<CurveLocation2>>>> {
        let invalid = |cause| ExactCurveError::invalid(operation, self.family(), cause);
        if self.geometry().is_some() {
            // Authored curves reuse their cached native spans and evaluators.
            let fragments = self.native_bezier_fragments_for_operation(policy, operation)?;
            let evaluators = self.rational_evaluators_for_operation(policy, operation)?;
            let mut locations = Vec::new();
            let mut constant_spans = 0;
            for (span_index, (fragment, evaluator)) in fragments.iter().zip(evaluators).enumerate()
            {
                match evaluator
                    .point_incidence_on_range(point, &CurveParameterRange2::unit(), policy)
                    .map_err(invalid)?
                {
                    Classification::Decided(crate::RationalBezierPointIncidence2::Parameters(
                        parameters,
                    )) => {
                        locations.extend(parameters.into_iter().map(|parameter| CurveLocation2 {
                            span_index,
                            span_range: fragment.span_range().clone(),
                            local_parameter: parameter.into(),
                        }))
                    }
                    Classification::Decided(crate::RationalBezierPointIncidence2::EntireCurve) => {
                        constant_spans += 1;
                    }
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            return Ok(if constant_spans == 0 {
                Classification::Decided(Some(locations))
            } else if constant_spans == fragments.len() {
                Classification::Decided(None)
            } else {
                Classification::Uncertain(UncertaintyReason::Unsupported)
            });
        }
        let spans = self.source_spans(policy, operation)?;
        let query = CurvePoint2::from(point.clone());
        let mut locations = Vec::new();
        let mut constant_spans = 0;
        for (span_index, span) in spans.iter().enumerate() {
            let chart = span.chart();
            let parameters: Vec<CurveParameter2> = match &span.fragment {
                crate::BezierSplitFragment2::Materialized { curve, .. } => {
                    let rational = RationalBezier2::try_from_subcurve(curve).map_err(invalid)?;
                    match rational
                        .point_incidence_on_range(point, &CurveParameterRange2::unit(), policy)
                        .map_err(invalid)?
                    {
                        Classification::Decided(
                            crate::RationalBezierPointIncidence2::Parameters(parameters),
                        ) => parameters.into_iter().map(Into::into).collect(),
                        Classification::Decided(
                            crate::RationalBezierPointIncidence2::EntireCurve,
                        ) => {
                            constant_spans += 1;
                            continue;
                        }
                        Classification::Uncertain(reason) => {
                            return Ok(Classification::Uncertain(reason));
                        }
                    }
                }
                fragment => {
                    let range = fragment.curve_region_parameter_range();
                    match crate::curve_support::CurveSupport2::from_fragment(fragment) {
                        crate::curve_support::CurveSupport2::Bezier(curve) => {
                            let rational =
                                RationalBezier2::try_from_subcurve(&curve).map_err(invalid)?;
                            match rational
                                .point_incidence_on_range(point, &range, policy)
                                .map_err(invalid)?
                            {
                                Classification::Decided(
                                    crate::RationalBezierPointIncidence2::Parameters(parameters),
                                ) => parameters.into_iter().map(Into::into).collect(),
                                Classification::Decided(
                                    crate::RationalBezierPointIncidence2::EntireCurve,
                                ) => {
                                    constant_spans += 1;
                                    continue;
                                }
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        crate::curve_support::CurveSupport2::Parallel(parallel) => {
                            match parallel
                                .point_incidence(point, &range, policy)
                                .map_err(invalid)?
                            {
                                Classification::Decided(
                                    crate::BezierParallelIncidence2::Parameters(parameters),
                                ) => parameters.into_iter().map(Into::into).collect(),
                                Classification::Decided(
                                    crate::BezierParallelIncidence2::EntireCurve,
                                ) => {
                                    constant_spans += 1;
                                    continue;
                                }
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        crate::curve_support::CurveSupport2::Line(chord) => {
                            match chord.point_parameter(&query, policy).map_err(invalid)? {
                                Classification::Decided(parameter) => parameter
                                    .into_iter()
                                    .map(CurveParameter2::from_algebraic_chord)
                                    .collect(),
                                Classification::Uncertain(reason) => {
                                    return Ok(Classification::Uncertain(reason));
                                }
                            }
                        }
                        crate::curve_support::CurveSupport2::Circle(circle) => {
                            let incidence = circle
                                .semicircle()
                                .retained_point_incidence_sign(&query, policy)
                                .map_err(invalid)?;
                            if let Classification::Uncertain(reason) = incidence {
                                return Ok(Classification::Uncertain(reason));
                            }
                            if incidence != Classification::Decided(crate::RealSign::Zero) {
                                Vec::new()
                            } else {
                                let parameter = match circle
                                    .semicircle()
                                    .parameter_at_certified_incident_point(&query, policy)
                                    .map_err(invalid)?
                                {
                                    Classification::Decided(parameter) => parameter,
                                    Classification::Uncertain(reason) => {
                                        return Ok(Classification::Uncertain(reason));
                                    }
                                };
                                // A point on the complementary half lies off
                                // this fragment's angular range.
                                let inside = match parameter.as_algebraic_cusp() {
                                    Some(cusp) if !parameter.is_algebraic_cusp_complement() => {
                                        match circle
                                            .certified_incident_point_evidence_location(
                                                cusp, &query, policy,
                                            )
                                            .map_err(invalid)?
                                        {
                                            Classification::Decided(location) => !matches!(
                                                location,
                                                crate::bezier_offset::BezierAlgebraicCuspSemicircleIncidentLocation2::Exterior
                                            ),
                                            Classification::Uncertain(reason) => {
                                                return Ok(Classification::Uncertain(reason));
                                            }
                                        }
                                    }
                                    _ => false,
                                };
                                if inside { vec![parameter] } else { Vec::new() }
                            }
                        }
                    }
                }
            };
            locations.extend(parameters.into_iter().map(|parameter| CurveLocation2 {
                span_index,
                span_range: chart.clone(),
                local_parameter: parameter,
            }));
        }
        if constant_spans == 0 {
            Ok(Classification::Decided(Some(locations)))
        } else if constant_spans == spans.len() {
            Ok(Classification::Decided(None))
        } else {
            // A partly constant spline has a positive-dimensional preimage
            // on some spans only; it is not a finite location set.
            Ok(Classification::Uncertain(UncertaintyReason::Unsupported))
        }
    }
}

impl CurveLocation2 {
    pub(crate) fn new(
        span_index: usize,
        span_range: CurveSpanRange2,
        local_parameter: CurveParameter2,
    ) -> Self {
        Self {
            span_index,
            span_range,
            local_parameter,
        }
    }

    /// Returns the promoted span index used by top-level dispatch.
    pub const fn span_index(&self) -> usize {
        self.span_index
    }

    /// Returns the promoted span's public parameter interval.
    pub const fn span_range(&self) -> &CurveSpanRange2 {
        &self.span_range
    }

    /// Returns the exact parameter in the retained support's local chart.
    pub const fn local_parameter(&self) -> &CurveParameter2 {
        &self.local_parameter
    }

    /// Returns the exact authored curve parameter without requiring a scalar
    /// payload or discarding selected-root evidence.
    ///
    /// Identity charts reuse the existing authority. Other affine charts are
    /// replayed only on demand, under certified parameter-transform predicates.
    pub fn parameter(&self, policy: &CurveContext) -> CurveResult<Classification<CurveParameter2>> {
        let (start, end) = self.span_range.endpoints();
        if start == &Real::zero() && end == &Real::one() {
            return Ok(Classification::Decided(self.local_parameter.clone()));
        }
        self.local_parameter
            .affine_image_unbounded(&(end - start), start, policy)
    }
}

impl CurveIntersectionContact2 {
    /// Returns parameter evidence on the first top-level curve.
    pub const fn first(&self) -> &CurveLocation2 {
        &self.first
    }

    /// Returns parameter evidence on the second top-level curve.
    pub const fn second(&self) -> &CurveLocation2 {
        &self.second
    }

    /// Returns exact affine point evidence retained by candidate replay.
    pub const fn point(&self) -> &CurvePoint2 {
        &self.point
    }

    /// Returns whether retained exact evidence certifies a transverse contact.
    pub const fn is_certified_transverse(&self) -> bool {
        self.certified_transverse
    }

    /// Returns the certified sign of the first traversal tangent crossed with the second.
    pub const fn tangent_cross_sign(&self) -> Option<hyperreal::RealSign> {
        self.tangent_cross_sign
    }
}

impl CurveIntersectionPairBlocker2 {
    /// Returns the promoted span index on the first curve.
    pub const fn first_span_index(&self) -> usize {
        self.first_span_index
    }

    /// Returns the promoted span index on the second curve.
    pub const fn second_span_index(&self) -> usize {
        self.second_span_index
    }

    /// Returns the retained blocker kind and exact replay evidence.
    pub const fn kind(&self) -> &CurveIntersectionPairBlockerKind2 {
        &self.kind
    }
}

impl CurveIntersectionOverlap2 {
    /// Restricts this positive-length component to two closed local domains.
    ///
    /// Each endpoint pair must belong to the corresponding operand span. Either
    /// pair may descend. The result
    /// keeps this component's traversal, open endpoints and original transport
    /// evidence; wider limits never enlarge it. A singleton restriction has no
    /// positive-length component and returns `None`.
    pub fn restrict(
        &self,
        first: [CurveParameter2; 2],
        second: [CurveParameter2; 2],
        policy: &CurveContext,
    ) -> CurveResult<CurveOutcome<Classification<Option<Self>>>> {
        let [first_start, first_end] = first;
        let [second_start, second_end] = second;
        let first = CurveParameterRange2::new_validated(first_start, first_end);
        let second = CurveParameterRange2::new_validated(second_start, second_end);
        resolve_certified_operation(policy, |attempt| {
            for range in [&first, &second] {
                match range.start().cmp_by_refinement(range.end(), attempt)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        return Ok(Classification::Decided(None));
                    }
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
            self.restrict_raw(&first, &second, attempt)
        })
    }

    pub(crate) fn restrict_raw(
        &self,
        first: &CurveParameterRange2,
        second: &CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Option<Self>>> {
        let clip = |active, limit| {
            crate::bezier_split::intersect_parameter_ranges(active, limit, policy).map(|result| {
                result.map(|bounds| {
                    bounds.map(|[start, end]| CurveParameterRange2::new_validated(start, end))
                })
            })
        };
        let first = match clip(&self.first_range, first)? {
            Classification::Decided(Some(range)) => range,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let second = match clip(&self.second_range, second)? {
            Classification::Decided(Some(range)) => range,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let same_bounds = |a: &CurveParameterRange2, b: &CurveParameterRange2| {
            a == b || (a.start() == b.end() && a.end() == b.start())
        };
        if same_bounds(&first, &self.first_range) && same_bounds(&second, &self.second_range) {
            return Ok(Classification::Decided(Some(self.clone())));
        }
        let source = &self.parameter_correspondence;
        let (mut first, mut second) = match source.clipped_ranges(&first, &second, policy)? {
            Classification::Decided(Some(ranges)) => ranges,
            Classification::Decided(None) => return Ok(Classification::Decided(None)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let order = match self
            .first_range
            .start()
            .cmp_by_refinement(self.first_range.end(), policy)?
        {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let clipped_order = match first.start().cmp_by_refinement(first.end(), policy)? {
            Classification::Decided(order) => order,
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        if order != clipped_order {
            first = CurveParameterRange2::new_validated(first.end().clone(), first.start().clone());
            second =
                CurveParameterRange2::new_validated(second.end().clone(), second.start().clone());
        }
        Ok(self.with_paired_ranges(first, second, policy)?.map(Some))
    }

    /// Publishes certified paired subranges without rebasing the original map.
    pub(crate) fn with_paired_ranges(
        &self,
        first_range: CurveParameterRange2,
        second_range: CurveParameterRange2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<Self>> {
        let mut endpoint_inclusion = [true, true];
        for (index, parameter) in [first_range.start(), first_range.end()]
            .into_iter()
            .enumerate()
        {
            for (boundary, included) in [
                (self.first_range.start(), self.includes_start()),
                (self.first_range.end(), self.includes_end()),
            ] {
                if included {
                    continue;
                }
                match parameter.cmp_by_refinement(boundary, policy)? {
                    Classification::Decided(std::cmp::Ordering::Equal) => {
                        endpoint_inclusion[index] = false
                    }
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
            }
        }
        Ok(Classification::Decided(Self {
            first_range,
            second_range,
            endpoint_inclusion,
            ..self.clone()
        }))
    }

    /// Returns the promoted span index on the first curve.
    pub const fn first_span_index(&self) -> usize {
        self.first_span_index
    }

    /// Returns the promoted span index on the second curve.
    pub const fn second_span_index(&self) -> usize {
        self.second_span_index
    }

    /// Returns the exact local closure bounds, following the first curve's traversal.
    pub const fn first_range(&self) -> &CurveParameterRange2 {
        &self.first_range
    }

    /// Returns the paired exact local closure bounds on the second span.
    ///
    /// The two ranges traverse corresponding points in the same order.
    pub const fn second_range(&self) -> &CurveParameterRange2 {
        &self.second_range
    }

    /// Returns relative traversal orientation on the shared image.
    pub const fn orientation(&self) -> CurveOverlapOrientation2 {
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

impl CurveIntersectionResult2 {
    /// Returns the number of promoted span pairs considered by exact dispatch.
    pub fn span_pair_count(&self) -> usize {
        self.data.span_pair_count
    }

    /// Returns all certified contacts in deterministic promoted-span order.
    pub fn contacts(&self) -> &[CurveIntersectionContact2] {
        &self.data.contacts
    }

    /// Returns all span pairs that still require exact topology work.
    pub fn blockers(&self) -> &[CurveIntersectionPairBlocker2] {
        &self.data.blockers
    }

    /// Returns all certified positive-length span overlaps.
    pub fn overlaps(&self) -> &[CurveIntersectionOverlap2] {
        &self.data.overlaps
    }

    /// Returns complete parameter components with a single point image.
    pub fn parameter_components(&self) -> &[CurveIntersectionParameterComponent2] {
        self.data.parameter_components.as_deref().unwrap_or(&[])
    }

    /// Returns true when every promoted span pair was completely replayed.
    pub fn is_complete(&self) -> bool {
        self.data.blockers.is_empty()
    }

    /// Returns true when complete replay certified no intersection.
    pub fn is_disjoint(&self) -> bool {
        self.is_complete()
            && self.data.contacts.is_empty()
            && self.data.overlaps.is_empty()
            && self.parameter_components().is_empty()
    }
}

impl CurveIntersectionTopology2 {
    /// Returns the complete contact result that generated this topology.
    pub fn result(&self) -> &CurveIntersectionResult2 {
        &self.data.result
    }

    /// Returns the first curve's exact pieces in traversal order.
    pub fn first(&self) -> &[Curve2] {
        &self.data.first
    }

    /// Returns the second curve's exact pieces in traversal order.
    pub fn second(&self) -> &[Curve2] {
        &self.data.second
    }
}

pub(crate) fn split_curve(
    curve: &Curve2,
    parameters: impl Iterator<Item = (usize, CurveParameter2)>,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<Curve2>> {
    let parameters = parameters.collect::<Vec<_>>();
    if parameters.is_empty() {
        return Ok(vec![curve.clone()]);
    }
    let spans = curve_support_intersection::spans(curve, policy)
        .map_err(|error| error.with_operation(CurveOperation2::Arrangement))?;
    let mut cuts = Vec::with_capacity(parameters.len());
    for (span_index, local_parameter) in parameters {
        let span = spans.get(span_index).ok_or_else(|| {
            ExactCurveError::invalid(
                CurveOperation2::Arrangement,
                curve.family(),
                CurveError::Topology(
                    "an intersection cut references an unknown source span".into(),
                ),
            )
        })?;
        let location = CurveLocation2 {
            span_index,
            span_range: span.chart.clone(),
            local_parameter,
        };
        cuts.push(
            match location.parameter(policy).map_err(|cause| {
                ExactCurveError::invalid(CurveOperation2::Arrangement, curve.family(), cause)
            })? {
                Classification::Decided(parameter) => parameter,
                Classification::Uncertain(reason) => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Arrangement,
                        curve.family(),
                        reason,
                    ));
                }
            },
        );
    }
    curve
        .split_at_parameters(cuts, policy)
        .map(|pieces| pieces.into_iter().map(|(_, curve)| curve).collect())
        .map_err(|error| error.with_operation(CurveOperation2::Arrangement))
}

fn matching_contact_index(
    contacts: &[CurveIntersectionContact2],
    candidate: &CurveIntersectionContact2,
    policy: &CurveContext,
) -> Classification<Option<usize>> {
    for (index, existing) in contacts.iter().enumerate() {
        match same_contact(existing, candidate, policy) {
            Classification::Decided(true) => return Classification::Decided(Some(index)),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => return Classification::Uncertain(reason),
        }
    }
    Classification::Decided(None)
}

fn same_contact(
    first: &CurveIntersectionContact2,
    second: &CurveIntersectionContact2,
    policy: &CurveContext,
) -> Classification<bool> {
    let first_parameter = same_curve_parameter(&first.first, &second.first, policy);
    if first_parameter == Classification::Decided(false) {
        return Classification::Decided(false);
    }
    let second_parameter = same_curve_parameter(&first.second, &second.second, policy);
    match (first_parameter, second_parameter) {
        (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {
            Classification::Decided(false)
        }
        (Classification::Decided(true), Classification::Decided(true)) => {
            Classification::Decided(true)
        }
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Classification::Uncertain(reason)
        }
    }
}

fn same_curve_parameter(
    first: &CurveLocation2,
    second: &CurveLocation2,
    policy: &CurveContext,
) -> Classification<bool> {
    if first == second {
        return Classification::Decided(true);
    }
    if first.span_index == second.span_index && first.span_range == second.span_range {
        return first
            .local_parameter
            .same_value(&second.local_parameter, policy)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let (first_start, first_end) = first.span_range.endpoints();
    let (second_start, second_end) = second.span_range.endpoints();
    if compare_reals(first_start, first_end, policy) == Some(std::cmp::Ordering::Less)
        && compare_reals(second_start, second_end, policy) == Some(std::cmp::Ordering::Less)
        && (matches!(
            compare_reals(first_end, second_start, policy),
            Some(std::cmp::Ordering::Less)
        ) || matches!(
            compare_reals(second_end, first_start, policy),
            Some(std::cmp::Ordering::Less)
        ))
    {
        return Classification::Decided(false);
    }
    let (Ok(Classification::Decided(first)), Ok(Classification::Decided(second))) =
        (first.parameter(policy), second.parameter(policy))
    else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    first
        .same_value(&second, policy)
        .unwrap_or(Classification::Uncertain(UncertaintyReason::Ordering))
}

#[cfg(test)]
mod native_dispatch_tests {
    use super::*;
    use crate::{CircularArc2, LineSeg2, QuadraticBezier2};

    fn point(x: i8, y: i8) -> Point2 {
        Point2::from_values(x, y)
    }

    #[test]
    fn degree_elevated_affine_lines_keep_native_pair_dispatch() {
        let horizontal = Curve2::from(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(point(0, 0), point(4, 0)).unwrap(),
        ));
        let vertical = Curve2::from(QuadraticBezier2::from_line_segment(
            LineSeg2::try_new(point(2, -2), point(2, 2)).unwrap(),
        ));
        let context =
            CurveIntersectionContext::try_new(&horizontal, &vertical, &CurveContext::STRICT)
                .unwrap();

        assert!(matches!(
            context.data.dispatch,
            CurveIntersectionDispatch::NativeLine(_)
        ));
        let result = context.result().unwrap();
        assert!(result.is_complete());
        assert_eq!(result.contacts().len(), 1);
        assert!(result.overlaps().is_empty());
        assert_eq!(
            result.contacts()[0]
                .first()
                .parameter(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided((Real::one() / Real::from(2_i8)).unwrap().into())
        );
        assert_eq!(
            result.contacts()[0]
                .second()
                .parameter(&CurveContext::STRICT)
                .unwrap(),
            Classification::Decided((Real::one() / Real::from(2_i8)).unwrap().into())
        );
    }

    #[test]
    fn batch_cache_reuses_one_circle_support_relation_across_spans() {
        let first =
            CircularArc2::try_from_center(point(2, 0), point(2, 0), point(0, 0), false).unwrap();
        let second =
            CircularArc2::try_from_center(point(3, 0), point(3, 0), point(1, 0), false).unwrap();
        let first = first
            .rational_bezier_decomposition_with_policy(&CurveContext::STRICT)
            .unwrap()
            .into_value();
        let second = second
            .rational_bezier_decomposition_with_policy(&CurveContext::STRICT)
            .unwrap()
            .into_value();
        let mut cache = CurveIntersectionBatchCache::default();
        for first_span in first.spans() {
            for second_span in second.spans() {
                let context = CurveIntersectionContext::try_new_with_batch_cache(
                    &Curve2::from(first_span.curve().clone()),
                    &Curve2::from(second_span.curve().clone()),
                    &CurveContext::STRICT,
                    &mut cache,
                )
                .unwrap();
                let result = context.result().unwrap();
                assert!(result.is_complete(), "{:?}", result.blockers());
            }
        }
        assert_eq!(cache.circular_support_relations.len(), 1);
    }
}

#[cfg(test)]
mod point_component_dispatch_tests {
    use super::*;
    use crate::{
        CubicBezier2, CurveCertainty, CurvePath2, LineSeg2, QuadraticBezier2,
        RationalQuadraticBezier2,
    };

    fn p(x: i32, y: i32) -> Point2 {
        Point2::from_values(x, y)
    }
    fn exact<T: std::fmt::Debug>(value: Classification<T>) -> T {
        match value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("{reason:?}"),
        }
    }
    fn certified<T>(value: CurveOutcome<T>) -> T {
        assert_eq!(value.certainty, CurveCertainty::Certified);
        value.value
    }
    fn query(first: &Curve2, second: &Curve2, policy: &CurveContext) -> CurveIntersectionResult2 {
        let result = certified(first.intersect_curve_with_policy(second, policy).unwrap());
        assert!(result.is_complete(), "{:?}", result.blockers());
        result
    }
    fn constants(policy: &CurveContext) -> Vec<Curve2> {
        vec![
            QuadraticBezier2::new(p(0, 0), p(0, 0), p(0, 0)).into(),
            CubicBezier2::new(p(0, 0), p(0, 0), p(0, 0), p(0, 0)).into(),
            RationalQuadraticBezier2::try_new(
                p(0, 0),
                p(0, 0),
                p(0, 0),
                1.into(),
                2.into(),
                3.into(),
            )
            .unwrap()
            .into(),
            RationalBezier2::try_new(vec![p(0, 0); 5], vec![Real::one(); 5])
                .unwrap()
                .into(),
            certified(
                Curve2::try_polynomial_bspline_with_policy(
                    1,
                    vec![p(0, 0); 3],
                    [2, 2, 3, 4, 4].into_iter().map(Real::from).collect(),
                    policy,
                )
                .unwrap(),
            ),
            certified(
                Curve2::try_nurbs_with_policy(
                    1,
                    vec![p(0, 0); 3],
                    vec![1.into(), 2.into(), 3.into()],
                    [2, 2, 3, 4, 4].into_iter().map(Real::from).collect(),
                    policy,
                )
                .unwrap(),
            ),
        ]
    }

    #[test]
    fn native_constant_images_retain_complete_parameter_fibers() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let crossing = Curve2::from(LineSeg2::try_new(p(-1, 0), p(1, 0)).unwrap());
            let ending = Curve2::from(LineSeg2::try_new(p(0, 0), p(1, 0)).unwrap());
            let excluded = Curve2::from(LineSeg2::try_new(p(-1, 1), p(1, 1)).unwrap());
            for constant in constants(&policy) {
                let count = certified(
                    constant
                        .native_bezier_fragments_with_policy(&policy)
                        .unwrap(),
                )
                .len();
                for constant in [
                    constant.clone(),
                    certified(constant.reversed_with_policy(&policy).unwrap()),
                ] {
                    for (other, parameter) in [
                        (&crossing, (Real::one() / Real::from(2)).unwrap()),
                        (&ending, Real::zero()),
                    ] {
                        for swapped in [false, true] {
                            let (a, b) = if swapped {
                                (other, &constant)
                            } else {
                                (&constant, other)
                            };
                            let result = query(a, b, &policy);
                            assert_eq!(result.parameter_components().len(), count);
                            assert!(result.contacts().is_empty() && result.overlaps().is_empty());
                            for (index, component) in
                                result.parameter_components().iter().enumerate()
                            {
                                let (free, fixed, span) = if swapped {
                                    (
                                        component.second_parameters(),
                                        component.first_parameters(),
                                        component.second_span_index(),
                                    )
                                } else {
                                    (
                                        component.first_parameters(),
                                        component.second_parameters(),
                                        component.first_span_index(),
                                    )
                                };
                                assert_eq!(span, index);
                                assert!(matches!(free, CurveParameterSet2::Range(_)));
                                let CurveParameterSet2::Single(fixed) = fixed else {
                                    panic!("one point on the nonconstant line")
                                };
                                assert_eq!(fixed.scalar(), Some(&parameter));
                                assert_eq!(
                                    certified(
                                        component.point().coincides_with(&p(0, 0).into(), &policy)
                                    ),
                                    Classification::Decided(true)
                                );
                            }
                        }
                    }
                    assert!(query(&constant, &excluded, &policy).is_disjoint());
                    let result = query(&constant, &constant, &policy);
                    assert_eq!(result.parameter_components().len(), count * count);
                    assert!(result.contacts().is_empty() && result.overlaps().is_empty());
                    assert!(
                        result
                            .parameter_components()
                            .iter()
                            .all(|component| matches!(
                                (component.first_parameters(), component.second_parameters()),
                                (CurveParameterSet2::Range(_), CurveParameterSet2::Range(_))
                            ))
                    );
                    let path = CurvePath2::try_new(vec![crossing.clone()]).unwrap();
                    let cutter = CurvePath2::try_new(vec![constant.clone()]).unwrap();
                    let topology = certified(
                        path.intersection_topology_with_policy(&cutter, &policy)
                            .unwrap(),
                    );
                    assert_eq!(topology.result().parameter_components().len(), count);
                    assert_eq!(topology.first()[0].curves().len(), 2);
                }
            }
        }
    }

    #[test]
    fn native_endpoint_shortcut_does_not_erase_retraced_or_spline_visits() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let line = Curve2::from(LineSeg2::try_new(p(-1, 0), p(0, 0)).unwrap());
            let sources = [
                (
                    Curve2::from(QuadraticBezier2::new(p(0, 0), p(1, 0), p(0, 0))),
                    Real::one(),
                ),
                (
                    certified(
                        Curve2::try_polynomial_bspline_with_policy(
                            1,
                            vec![p(0, 0), p(1, 0), p(0, 0)],
                            [0, 0, 1, 2, 2].into_iter().map(Real::from).collect(),
                            &policy,
                        )
                        .unwrap(),
                    ),
                    Real::from(2),
                ),
            ];
            for (source, end) in sources {
                for swapped in [false, true] {
                    let (a, b) = if swapped {
                        (&line, &source)
                    } else {
                        (&source, &line)
                    };
                    let result = query(a, b, &policy);
                    assert_eq!(result.contacts().len(), 2);
                    assert!(
                        result.overlaps().is_empty() && result.parameter_components().is_empty()
                    );
                    let parameters = result
                        .contacts()
                        .iter()
                        .map(|contact| {
                            let (source, line) = if swapped {
                                (contact.second(), contact.first())
                            } else {
                                (contact.first(), contact.second())
                            };
                            assert_eq!(line.local_parameter().scalar(), Some(&Real::one()));
                            exact(source.parameter(&policy).unwrap())
                                .scalar()
                                .unwrap()
                                .clone()
                        })
                        .collect::<Vec<_>>();
                    assert!(parameters.iter().any(|p| p == &Real::zero()));
                    assert!(parameters.iter().any(|p| p == &end));
                    assert_eq!(
                        certified(source.start().coincides_with(&source.end(), &policy)),
                        Classification::Decided(true)
                    );
                }
            }
        }
    }

    #[test]
    fn native_point_components_preserve_homogeneous_pole_domains() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let source = exact(
                RationalBezier2::from_homogeneous_controls_with_policy(
                    [-1, 0, 1]
                        .into_iter()
                        .map(|weight| {
                            crate::HomogeneousControl2::new(
                                Real::zero(),
                                Real::zero(),
                                Real::from(weight),
                            )
                        })
                        .collect(),
                    &policy,
                )
                .unwrap(),
            );
            let source = Curve2::from(source);
            let line = Curve2::from(LineSeg2::try_new(p(-1, 0), p(1, 0)).unwrap());
            let result = certified(source.intersect_curve_with_policy(&line, &policy).unwrap());
            assert!(!result.is_complete());
            assert!(result.parameter_components().is_empty());
            let finite = certified(
                source
                    .subcurve_with_policy(
                        Real::zero().into(),
                        (Real::one() / Real::from(4)).unwrap().into(),
                        &policy,
                    )
                    .unwrap(),
            );
            let result = query(&finite, &line, &policy);
            assert_eq!(result.parameter_components().len(), 1);
        }
    }
    #[test]
    fn singleton_incidence_merges_spline_seams_but_keeps_distinct_visits() {
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            let line = Curve2::from(LineSeg2::try_new(p(-1, 0), p(0, 0)).unwrap());
            for count in [3, 5] {
                let controls = (0..count)
                    .map(|index| p(if index % 2 == 0 { 1 } else { 0 }, 0))
                    .collect::<Vec<_>>();
                let knots = std::iter::once(0)
                    .chain(0..count)
                    .chain(std::iter::once(count - 1))
                    .map(Real::from)
                    .collect();
                let source = certified(
                    Curve2::try_polynomial_bspline_with_policy(1, controls, knots, &policy)
                        .unwrap(),
                );
                for swapped in [false, true] {
                    let (a, b) = if swapped {
                        (&line, &source)
                    } else {
                        (&source, &line)
                    };
                    let result = query(a, b, &policy);
                    assert_eq!(result.contacts().len(), (count / 2) as usize);
                    assert_eq!(result.span_pair_count(), (count - 1) as usize);
                    for visit in (1..count).step_by(2) {
                        assert_eq!(
                            result
                                .contacts()
                                .iter()
                                .filter(|contact| {
                                    let location = if swapped {
                                        contact.second()
                                    } else {
                                        contact.first()
                                    };
                                    exact(location.parameter(&policy).unwrap()).scalar()
                                        == Some(&Real::from(visit))
                                })
                                .count(),
                            1
                        );
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod overlap_restriction_tests {
    use super::*;
    use crate::{CurveCertainty, LineSeg2};

    fn exact<T: std::fmt::Debug>(value: CurveOutcome<Classification<T>>) -> T {
        assert_eq!(value.certainty, CurveCertainty::Certified);
        match value.value {
            Classification::Decided(value) => value,
            Classification::Uncertain(reason) => panic!("{reason:?}"),
        }
    }

    fn check_native_restriction(first: &Curve2, second: &Curve2, policy: &CurveContext) {
        let result = first.intersect_curve_with_policy(second, policy).unwrap();
        assert_eq!(result.certainty, CurveCertainty::Certified);
        let result = result.value;
        assert!(result.is_complete());
        assert!(!result.overlaps().is_empty());
        let first_spans = first
            .native_bezier_fragments_with_policy(policy)
            .unwrap()
            .into_value();
        let second_spans = second
            .native_bezier_fragments_with_policy(policy)
            .unwrap()
            .into_value();
        for overlap in result.overlaps() {
            let mid = match overlap
                .first_range()
                .strict_interior_scalar(policy)
                .unwrap()
            {
                Classification::Decided(mid) => mid,
                Classification::Uncertain(reason) => panic!("{reason:?}"),
            };
            let limit = CurveParameterRange2::new_validated(
                overlap.first_range().start().clone(),
                mid.into(),
            );
            let mut open = overlap.clone();
            open.endpoint_inclusion = [false, false];
            let clipped = exact(
                open.restrict(
                    [limit.start().clone(), limit.end().clone()],
                    [
                        overlap.second_range().start().clone(),
                        overlap.second_range().end().clone(),
                    ],
                    policy,
                )
                .unwrap(),
            )
            .unwrap();
            assert!(!clipped.includes_start());
            assert!(clipped.includes_end());
            let a = Curve2::from(
                first_spans[overlap.first_span_index()]
                    .native_curve()
                    .clone(),
            );
            let b = Curve2::from(
                second_spans[overlap.second_span_index()]
                    .native_curve()
                    .clone(),
            );
            for (a_parameter, b_parameter) in [
                (
                    clipped.first_range().start(),
                    clipped.second_range().start(),
                ),
                (clipped.first_range().end(), clipped.second_range().end()),
            ] {
                let a = a
                    .point_at_with_policy(a_parameter, policy)
                    .unwrap()
                    .into_value();
                let b = b
                    .point_at_with_policy(b_parameter, policy)
                    .unwrap()
                    .into_value();
                assert!(exact(a.coincides_with(&b, policy)));
            }
            for _ in 0..8 {
                assert_eq!(
                    exact(
                        clipped
                            .restrict(
                                [
                                    overlap.first_range().start().clone(),
                                    overlap.first_range().end().clone()
                                ],
                                [
                                    overlap.second_range().start().clone(),
                                    overlap.second_range().end().clone()
                                ],
                                policy
                            )
                            .unwrap()
                    )
                    .unwrap(),
                    clipped
                );
            }
            let singleton = std::array::from_fn(|_| clipped.first_range().end().clone());
            assert!(
                exact(
                    open.restrict(
                        singleton,
                        [
                            overlap.second_range().start().clone(),
                            overlap.second_range().end().clone()
                        ],
                        policy
                    )
                    .unwrap()
                )
                .is_none()
            );
            let rest = CurveParameterRange2::new_validated(
                clipped.first_range().end().clone(),
                overlap.first_range().end().clone(),
            );
            assert!(
                exact(
                    clipped
                        .restrict(
                            [rest.start().clone(), rest.end().clone()],
                            [
                                overlap.second_range().start().clone(),
                                overlap.second_range().end().clone()
                            ],
                            policy
                        )
                        .unwrap()
                )
                .is_none()
            );
            let reversed_limit =
                CurveParameterRange2::new_validated(limit.end().clone(), limit.start().clone());
            assert_eq!(
                exact(
                    open.restrict(
                        [reversed_limit.start().clone(), reversed_limit.end().clone()],
                        [
                            overlap.second_range().start().clone(),
                            overlap.second_range().end().clone()
                        ],
                        policy
                    )
                    .unwrap()
                )
                .unwrap(),
                clipped
            );
        }
    }

    #[test]
    fn native_line_overlap_restriction_retains_affine_transport_and_open_ends() {
        let first = Curve2::from(
            LineSeg2::try_new(Point2::from_values(0, 0), Point2::from_values(4, 0)).unwrap(),
        );
        let second = Curve2::from(
            LineSeg2::try_new(Point2::from_values(6, 0), Point2::from_values(2, 0)).unwrap(),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for second in [
                second.clone(),
                second.reversed_with_policy(&policy).unwrap().into_value(),
            ] {
                for (a, b) in [(&first, &second), (&second, &first)] {
                    check_native_restriction(a, b, &policy);
                }
            }
        }
    }

    #[test]
    fn native_arc_overlap_restriction_retains_distinct_rational_charts() {
        let first = Curve2::from(
            CircularArc2::try_from_center(
                Point2::from_values(1, 0),
                Point2::from_values(0, 1),
                Point2::from_values(0, 0),
                false,
            )
            .unwrap(),
        );
        let second = Curve2::from(
            CircularArc2::try_from_center(
                Point2::new(
                    (Real::from(3) / Real::from(5)).unwrap(),
                    (Real::from(4) / Real::from(5)).unwrap(),
                ),
                Point2::from_values(-1, 0),
                Point2::from_values(0, 0),
                false,
            )
            .unwrap(),
        );
        for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
            for second in [
                second.clone(),
                second.reversed_with_policy(&policy).unwrap().into_value(),
            ] {
                for (a, b) in [(&first, &second), (&second, &first)] {
                    check_native_restriction(a, b, &policy);
                }
            }
        }
    }
}
