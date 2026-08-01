//! Top-level exact curve-pair intersection with retained parameter intervals.

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
    ArcArcIntersection, BezierArrangementGraph2, BezierParameter2, BezierParameterRange2,
    BezierSplitMaterialization2, CircleCircleRelation, CircularArc2, Classification, Curve2,
    CurveContext, CurveError, CurveGeometry2, CurveOperation2, CurveOutcome, CurveResult,
    CurveSpanRange2, ExactCurveError, ExactCurveResult, LineArcIntersection,
    LineArcIntersectionPoint, LineArcOrder, LineLineIntersection, ParamRange, Point2,
    RationalBezier2, RationalBezierIntersectionCandidates2, RationalBezierIntersectionContact2,
    RationalBezierIntersectionContacts2, RationalBezierIntersectionPointEvidence2,
    RationalBezierOverlapOrientation2, UncertaintyReason,
};

/// Exact source parameter retained for one top-level curve contact.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionParameter2 {
    promoted_span_index: usize,
    span_range: CurveSpanRange2,
    local_parameter: BezierParameter2,
}

/// One exact top-level curve contact with parameters on both operands.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionContact2 {
    first: CurveIntersectionParameter2,
    second: CurveIntersectionParameter2,
    point: RationalBezierIntersectionPointEvidence2,
    certified_transverse: bool,
}

/// Certified positive-length overlap between two promoted top-level spans.
#[derive(Clone, Debug)]
pub struct CurveIntersectionOverlap2 {
    first_span_index: usize,
    second_span_index: usize,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
    parameter_correspondence: Option<RationalBezierOverlapParameterCorrespondence2>,
}

impl PartialEq for CurveIntersectionOverlap2 {
    fn eq(&self, other: &Self) -> bool {
        self.first_span_index == other.first_span_index
            && self.second_span_index == other.second_span_index
            && self.first_range == other.first_range
            && self.second_range == other.second_range
            && self.orientation == other.orientation
    }
}

/// Reason one promoted span pair did not produce complete contact topology.
#[derive(Clone, Debug, PartialEq)]
pub enum CurveIntersectionPairBlockerKind2 {
    /// A required predicate remained undecided under the active policy.
    Uncertain(UncertaintyReason),
    /// Candidate replay retained some contacts but not a complete pairing.
    IncompleteReplay {
        /// Complete unpaired resultant projections available for later replay.
        candidates: RationalBezierIntersectionCandidates2,
    },
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

/// Clone-shared split and arrangement topology for one complete curve-pair result.
#[derive(Clone, Debug)]
pub struct CurveIntersectionTopology2 {
    data: Arc<CurveIntersectionTopologyData>,
}

#[derive(Debug)]
struct CurveIntersectionTopologyData {
    result: CurveIntersectionResult2,
    first: Arc<[BezierSplitMaterialization2]>,
    second: Arc<[BezierSplitMaterialization2]>,
    arrangement: OnceLock<CurveResult<BezierArrangementGraph2>>,
}

#[derive(Debug)]
struct CurveIntersectionResultData {
    span_pair_count: usize,
    contacts: Arc<[CurveIntersectionContact2]>,
    overlaps: Arc<[CurveIntersectionOverlap2]>,
    blockers: Arc<[CurveIntersectionPairBlocker2]>,
}

#[derive(Debug)]
pub(crate) struct CurveIntersectionContext {
    data: CurveIntersectionContextData,
}

#[derive(Debug, Default)]
pub(crate) struct CurveIntersectionBatchCache {
    circular_support_relations: Vec<CircularSupportRelationCacheEntry>,
    circular_point_parameters: Vec<CircularPointParameterCacheEntry>,
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
                },
                second: RationalQuadraticCircle2 {
                    center: second.center.clone(),
                    radius_squared: second.radius_squared.clone(),
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
        CurveGeometry2::CircularArc(curve) if curve.endpoints_on_stored_circle_are_certified() => {
            Some(CircularSupportRef {
                center: curve.center(),
                radius_squared: curve.radius_squared_ref(),
            })
        }
        CurveGeometry2::RationalQuadraticBezier(curve) => {
            curve
                .retained_circular_conic()
                .map(|support| CircularSupportRef {
                    center: &support.center,
                    radius_squared: &support.radius_squared,
                })
        }
        CurveGeometry2::RationalBezier(curve) => {
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
    SpanPairs(Vec<CurveSpanPair>),
    CertifiedEndpointContact(CurveIntersectionContact2),
    NativeLine(LineLineIntersection),
    NativeLineArc {
        order: LineArcOrder,
        relation: LineArcIntersection,
    },
    NativeArcPoints(Vec<Point2>),
    NativeCoincidentArcs,
}

enum NativeArcIntersectionDispatch {
    Points(Vec<Point2>),
    Coincident,
}

#[derive(Debug)]
struct CurveSpanPair {
    first_span_index: usize,
    second_span_index: usize,
    state: CurveSpanPairState,
}

#[derive(Debug)]
enum CurveSpanPairState {
    Rational(RationalBezierIntersectionContext),
    RetainedLineageOverlap {
        first_range: ParamRange,
        second_range: ParamRange,
        orientation: RationalBezierOverlapOrientation2,
    },
    Blocked(UncertaintyReason),
}

fn build_span_pairs(
    first_curve: &Curve2,
    second_curve: &Curve2,
    first_evaluators: &[RationalBezier2],
    second_evaluators: &[RationalBezier2],
    policy: &CurveContext,
    circle_relation: Option<&CircleCircleRelation>,
    mut batch_cache: Option<&mut CurveIntersectionBatchCache>,
) -> ExactCurveResult<Vec<CurveSpanPair>> {
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
                    pairs.push(CurveSpanPair {
                        first_span_index,
                        second_span_index,
                        state: CurveSpanPairState::Blocked(reason),
                    });
                    continue;
                }
            };
            if let Some(overlap) = retained_overlap {
                pairs.push(CurveSpanPair {
                    first_span_index,
                    second_span_index,
                    state: CurveSpanPairState::RetainedLineageOverlap {
                        first_range: overlap.first,
                        second_range: overlap.second,
                        orientation: if overlap.same_orientation {
                            RationalBezierOverlapOrientation2::Same
                        } else {
                            RationalBezierOverlapOrientation2::Reversed
                        },
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
                Ok(intersection) => CurveSpanPairState::Rational(intersection),
                Err(ExactCurveError::Blocked(blocker)) => {
                    CurveSpanPairState::Blocked(blocker.reason())
                }
                Err(ExactCurveError::Invalid { cause, .. }) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        first_curve.family(),
                        cause,
                    ));
                }
            };
            pairs.push(CurveSpanPair {
                first_span_index,
                second_span_index,
                state,
            });
        }
    }
    Ok(pairs)
}

fn certified_singleton_aabb_endpoint_contact(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionContact2>> {
    let (Ok(first_bounds), Ok(second_bounds)) = (first.bounds(), second.bounds()) else {
        return Ok(None);
    };
    let point = match first_bounds.singleton_intersection(second_bounds, policy) {
        Classification::Decided(Some(point)) => point,
        Classification::Decided(None) | Classification::Uncertain(_) => return Ok(None),
    };
    let Some(first_parameter) = endpoint_parameter(first, &point, policy)? else {
        return Ok(None);
    };
    let Some(second_parameter) = endpoint_parameter(second, &point, policy)? else {
        return Ok(None);
    };
    Ok(Some(CurveIntersectionContact2 {
        first: first_parameter,
        second: second_parameter,
        point: RationalBezierIntersectionPointEvidence2::Exact(point),
        certified_transverse: false,
    }))
}

fn endpoint_parameter(
    curve: &Curve2,
    point: &Point2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<CurveIntersectionParameter2>> {
    let fragments =
        curve.native_bezier_fragments_for_operation(policy, CurveOperation2::Intersection)?;
    let (promoted_span_index, local_parameter) =
        if crate::classify::is_zero(&curve.start().distance_squared(point), policy) == Some(true) {
            (0, Real::zero())
        } else if crate::classify::is_zero(&curve.end().distance_squared(point), policy)
            == Some(true)
        {
            (fragments.len() - 1, Real::one())
        } else {
            return Ok(None);
        };
    Ok(Some(CurveIntersectionParameter2 {
        promoted_span_index,
        span_range: fragments[promoted_span_index].span_range().clone(),
        local_parameter: BezierParameter2::Exact(local_parameter),
    }))
}

fn native_line_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
) -> ExactCurveResult<Option<LineLineIntersection>> {
    let (Some(first_line), Some(second_line)) = (
        affine_line_image(first.geometry()),
        affine_line_image(second.geometry()),
    ) else {
        return Ok(None);
    };
    first_line
        .intersect_line(second_line, policy)
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
) -> ExactCurveResult<Option<(LineArcOrder, LineArcIntersection)>> {
    let (order, relation) = match (first.geometry(), second.geometry()) {
        (CurveGeometry2::Line(line), CurveGeometry2::CircularArc(arc)) => {
            (LineArcOrder::LineThenArc, line.intersect_arc(arc, policy))
        }
        (CurveGeometry2::CircularArc(arc), CurveGeometry2::Line(line)) => {
            (LineArcOrder::ArcThenLine, line.intersect_arc(arc, policy))
        }
        _ => return Ok(None),
    };
    relation
        .map(|relation| Some((order, relation)))
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Intersection, first.family(), cause)
        })
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
            first: CurveIntersectionParameter2 {
                promoted_span_index: 0,
                span_range: first_fragment.span_range().clone(),
                local_parameter: BezierParameter2::Exact(first_parameter),
            },
            second: CurveIntersectionParameter2 {
                promoted_span_index: 0,
                span_range: second_fragment.span_range().clone(),
                local_parameter: BezierParameter2::Exact(second_parameter),
            },
            point: RationalBezierIntersectionPointEvidence2::Exact(point),
            certified_transverse: false,
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
                Some(std::cmp::Ordering::Less) => RationalBezierOverlapOrientation2::Same,
                Some(std::cmp::Ordering::Greater) => RationalBezierOverlapOrientation2::Reversed,
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
                    first_range: BezierParameterRange2::from_exact(
                        a_range.start().clone(),
                        a_range.end().clone(),
                    ),
                    second_range: BezierParameterRange2::from_exact(
                        b_range.start().clone(),
                        b_range.end().clone(),
                    ),
                    orientation,
                    parameter_correspondence: None,
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
        }),
    })
}

fn build_native_line_arc_evidence(
    first: &Curve2,
    second: &Curve2,
    order: LineArcOrder,
    relation: &LineArcIntersection,
    policy: &CurveContext,
    span_pair_count: usize,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let mut contacts = Vec::new();
    match relation {
        LineArcIntersection::None => {}
        LineArcIntersection::Point(hit) => {
            append_native_line_arc_contact(&mut contacts, first, second, order, hit, policy)?;
        }
        LineArcIntersection::TwoPoints {
            first: first_hit,
            second: second_hit,
        } => {
            append_native_line_arc_contact(&mut contacts, first, second, order, first_hit, policy)?;
            append_native_line_arc_contact(
                &mut contacts,
                first,
                second,
                order,
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
        }),
    })
}

fn append_native_line_arc_contact(
    contacts: &mut Vec<CurveIntersectionContact2>,
    first: &Curve2,
    second: &Curve2,
    order: LineArcOrder,
    hit: &LineArcIntersectionPoint,
    policy: &CurveContext,
) -> ExactCurveResult<()> {
    let (line, arc_curve, arc) = match order {
        LineArcOrder::LineThenArc => {
            let CurveGeometry2::CircularArc(arc) = second.geometry() else {
                unreachable!("line-then-arc dispatch requires an arc second operand")
            };
            (first, second, arc)
        }
        LineArcOrder::ArcThenLine => {
            let CurveGeometry2::CircularArc(arc) = first.geometry() else {
                unreachable!("arc-then-line dispatch requires an arc first operand")
            };
            (second, first, arc)
        }
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
        let line_parameter = CurveIntersectionParameter2 {
            promoted_span_index: 0,
            span_range: line_fragment.span_range().clone(),
            local_parameter: BezierParameter2::Exact(hit.line_param.clone()),
        };
        let arc_parameter = CurveIntersectionParameter2 {
            promoted_span_index: arc_span_index,
            span_range: arc_fragments[arc_span_index].span_range().clone(),
            local_parameter: native_arc_span_parameter(
                arc_curve,
                &arc_evaluators[arc_span_index],
                &hit.point,
                policy,
            )?,
        };
        let (first_parameter, second_parameter) = match order {
            LineArcOrder::LineThenArc => (line_parameter, arc_parameter),
            LineArcOrder::ArcThenLine => (arc_parameter, line_parameter),
        };
        let candidate = CurveIntersectionContact2 {
            first: first_parameter,
            second: second_parameter,
            point: RationalBezierIntersectionPointEvidence2::Exact(hit.point.clone()),
            certified_transverse: false,
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
) -> ExactCurveResult<Option<NativeArcIntersectionDispatch>> {
    let (CurveGeometry2::CircularArc(first_arc), CurveGeometry2::CircularArc(second_arc)) =
        (first.geometry(), second.geometry())
    else {
        return Ok(None);
    };
    let relation = first_arc
        .circle_relation(second_arc, policy)
        .map_err(|cause| {
            ExactCurveError::invalid(CurveOperation2::Intersection, first.family(), cause)
        })?;
    let candidates = match relation {
        CircleCircleRelation::Coincident => {
            return Ok(Some(NativeArcIntersectionDispatch::Coincident));
        }
        CircleCircleRelation::Disjoint => {
            return Ok(Some(NativeArcIntersectionDispatch::Points(Vec::new())));
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
            first_arc.contains_sweep_point(&point, policy),
            second_arc.contains_sweep_point(&point, policy),
        ) {
            (Classification::Decided(true), Classification::Decided(true)) => points.push(point),
            (Classification::Decided(false), _) | (_, Classification::Decided(false)) => {}
            (Classification::Uncertain(_), _) | (_, Classification::Uncertain(_)) => {
                return Ok(None);
            }
        }
    }
    Ok(Some(NativeArcIntersectionDispatch::Points(points)))
}

fn build_native_arc_evidence(
    first: &Curve2,
    second: &Curve2,
    points: &[Point2],
    policy: &CurveContext,
    span_pair_count: usize,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let (CurveGeometry2::CircularArc(first_arc), CurveGeometry2::CircularArc(second_arc)) =
        (first.geometry(), second.geometry())
    else {
        unreachable!("native arc result requires two circular arcs")
    };
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
                    first: CurveIntersectionParameter2 {
                        promoted_span_index: first_span_index,
                        span_range: first_fragments[first_span_index].span_range().clone(),
                        local_parameter: native_arc_span_parameter(
                            first,
                            &first_evaluators[first_span_index],
                            point,
                            policy,
                        )?,
                    },
                    second: CurveIntersectionParameter2 {
                        promoted_span_index: second_span_index,
                        span_range: second_fragments[second_span_index].span_range().clone(),
                        local_parameter: native_arc_span_parameter(
                            second,
                            &second_evaluators[second_span_index],
                            point,
                            policy,
                        )?,
                    },
                    point: RationalBezierIntersectionPointEvidence2::Exact(point.clone()),
                    certified_transverse: false,
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
        }),
    })
}

fn build_native_coincident_arc_evidence(
    first: &Curve2,
    second: &Curve2,
    policy: &CurveContext,
    span_pair_count: usize,
) -> ExactCurveResult<CurveIntersectionResult2> {
    let (CurveGeometry2::CircularArc(first_arc), CurveGeometry2::CircularArc(second_arc)) =
        (first.geometry(), second.geometry())
    else {
        unreachable!("coincident-arc result requires two circular arcs")
    };
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
        let (first_start, first_end) = first_fragment.curve().endpoints();
        let first_span = CircularArc2::new_with_certified_radius(
            first_start.clone(),
            first_end.clone(),
            first_arc.center().clone(),
            first_arc.radius_squared(),
            first_arc.is_clockwise(),
            None,
        );
        for (second_span_index, second_fragment) in second_fragments.iter().enumerate() {
            let (second_start, second_end) = second_fragment.curve().endpoints();
            let second_span = CircularArc2::new_with_certified_radius(
                second_start.clone(),
                second_end.clone(),
                first_arc.center().clone(),
                first_arc.radius_squared(),
                second_arc.is_clockwise(),
                None,
            );
            let relation = first_span
                .intersect_arc(&second_span, policy)
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
                            RationalBezierOverlapOrientation2::Same
                        }
                        Classification::Decided(std::cmp::Ordering::Greater) => {
                            RationalBezierOverlapOrientation2::Reversed
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
                    overlaps.push(CurveIntersectionOverlap2 {
                        first_span_index,
                        second_span_index,
                        first_range,
                        second_range,
                        orientation,
                        parameter_correspondence: None,
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
        first: CurveIntersectionParameter2 {
            promoted_span_index: first_span_index,
            span_range: first_fragments[first_span_index].span_range().clone(),
            local_parameter: native_arc_span_parameter(
                first,
                &first_evaluators[first_span_index],
                point,
                policy,
            )?,
        },
        second: CurveIntersectionParameter2 {
            promoted_span_index: second_span_index,
            span_range: second_fragments[second_span_index].span_range().clone(),
            local_parameter: native_arc_span_parameter(
                second,
                &second_evaluators[second_span_index],
                point,
                policy,
            )?,
        },
        point: RationalBezierIntersectionPointEvidence2::Exact(point.clone()),
        certified_transverse: false,
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
    let mut indices = Vec::new();
    for (span_index, fragment) in fragments.iter().enumerate() {
        let (start, end) = fragment.curve().endpoints();
        let span = CircularArc2::new_with_certified_radius(
            start,
            end,
            arc.center().clone(),
            arc.radius_squared(),
            arc.is_clockwise(),
            None,
        );
        match span.contains_sweep_point(point, policy) {
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
    if span.control_points().len() != 3 || span.weights().len() != 3 {
        return Err(ExactCurveError::invalid(
            CurveOperation2::Intersection,
            curve.family(),
            CurveError::InvalidRationalBezier,
        ));
    }
    if crate::classify::is_zero(&span.start().distance_squared(point), policy) == Some(true) {
        return Ok(BezierParameter2::Exact(Real::zero()));
    }
    if crate::classify::is_zero(&span.end().distance_squared(point), policy) == Some(true) {
        return Ok(BezierParameter2::Exact(Real::one()));
    }

    let controls = span.control_points();
    let weights = span.weights();
    let p0 = controls[0].delta_from(point);
    let p1 = controls[1].delta_from(point);
    let p2 = controls[2].delta_from(point);
    let beta2_scaled = ((&p0.0 * &p1.1) - (&p0.1 * &p1.0)) * &weights[0];
    let beta0_scaled = ((&p1.0 * &p2.1) - (&p1.1 * &p2.0)) * &weights[2];
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
        policy: &CurveContext,
    ) -> ExactCurveResult<CurveOutcome<CurveIntersectionResult2>> {
        resolve_certified_operation(policy, |attempt| self.intersect_curve_raw(other, attempt))
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

impl CurveIntersectionContext {
    pub(crate) fn try_new(
        first: &Curve2,
        second: &Curve2,
        policy: &CurveContext,
    ) -> ExactCurveResult<Self> {
        Self::try_new_with_optional_batch_cache(first, second, policy, None)
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
        let (span_pair_count, dispatch) = match native_line_intersection(first, second, policy)? {
            Some(relation) => (1, CurveIntersectionDispatch::NativeLine(relation)),
            None => {
                if let Some((order, relation)) =
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
                        CurveIntersectionDispatch::NativeLineArc { order, relation },
                    )
                } else {
                    match native_arc_intersection(first, second, policy)? {
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
                                NativeArcIntersectionDispatch::Points(points) => {
                                    CurveIntersectionDispatch::NativeArcPoints(points)
                                }
                                NativeArcIntersectionDispatch::Coincident => {
                                    CurveIntersectionDispatch::NativeCoincidentArcs
                                }
                            };
                            (span_pair_count, dispatch)
                        }
                        None => {
                            if let Some(contact) =
                                certified_singleton_aabb_endpoint_contact(first, second, policy)?
                            {
                                (
                                    1,
                                    CurveIntersectionDispatch::CertifiedEndpointContact(contact),
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
                                let dispatch =
                                    CurveIntersectionDispatch::SpanPairs(build_span_pairs(
                                        first,
                                        second,
                                        first_evaluators,
                                        second_evaluators,
                                        policy,
                                        circle_relation.as_ref(),
                                        batch_cache,
                                    )?);
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
        if let CurveIntersectionDispatch::CertifiedEndpointContact(contact) = &self.data.dispatch {
            return Ok(CurveIntersectionResult2 {
                data: Arc::new(CurveIntersectionResultData {
                    span_pair_count: self.data.span_pair_count,
                    contacts: Arc::from([contact.clone()]),
                    overlaps: Arc::from([]),
                    blockers: Arc::from([]),
                }),
            });
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
        if let CurveIntersectionDispatch::NativeLineArc { order, relation } = &self.data.dispatch {
            return build_native_line_arc_evidence(
                &self.data.first,
                &self.data.second,
                *order,
                relation,
                &self.data.policy,
                self.data.span_pair_count,
            );
        }
        if let CurveIntersectionDispatch::NativeArcPoints(points) = &self.data.dispatch {
            return build_native_arc_evidence(
                &self.data.first,
                &self.data.second,
                points,
                &self.data.policy,
                self.data.span_pair_count,
            );
        }
        if matches!(
            self.data.dispatch,
            CurveIntersectionDispatch::NativeCoincidentArcs
        ) {
            return build_native_coincident_arc_evidence(
                &self.data.first,
                &self.data.second,
                &self.data.policy,
                self.data.span_pair_count,
            );
        }
        let first_fragments = self.data.first.native_bezier_fragments_for_operation(
            &self.data.policy,
            CurveOperation2::Intersection,
        )?;
        let second_fragments = self.data.second.native_bezier_fragments_for_operation(
            &self.data.policy,
            CurveOperation2::Intersection,
        )?;
        let mut contacts = Vec::new();
        let mut overlaps = Vec::new();
        let mut blockers = Vec::new();
        let CurveIntersectionDispatch::SpanPairs(pairs) = &self.data.dispatch else {
            unreachable!("native dispatch returned before generic span replay")
        };
        for pair in pairs {
            let first_span_range = first_fragments[pair.first_span_index].span_range().clone();
            let second_span_range = second_fragments[pair.second_span_index]
                .span_range()
                .clone();
            if let CurveSpanPairState::RetainedLineageOverlap {
                first_range,
                second_range,
                orientation,
            } = &pair.state
            {
                overlaps.push(CurveIntersectionOverlap2 {
                    first_span_index: pair.first_span_index,
                    second_span_index: pair.second_span_index,
                    first_range: BezierParameterRange2::from_exact(
                        first_range.start().clone(),
                        first_range.end().clone(),
                    ),
                    second_range: BezierParameterRange2::from_exact(
                        second_range.start().clone(),
                        second_range.end().clone(),
                    ),
                    orientation: *orientation,
                    parameter_correspondence: None,
                });
                continue;
            }
            let span_contacts = match &pair.state {
                CurveSpanPairState::Blocked(reason) => {
                    blockers.push(CurveIntersectionPairBlocker2 {
                        first_span_index: pair.first_span_index,
                        second_span_index: pair.second_span_index,
                        kind: CurveIntersectionPairBlockerKind2::Uncertain(*reason),
                    });
                    continue;
                }
                CurveSpanPairState::Rational(intersection) => match intersection.try_contacts() {
                    Ok(contacts) => contacts,
                    Err(ExactCurveError::Blocked(blocker)) => {
                        blockers.push(CurveIntersectionPairBlocker2 {
                            first_span_index: pair.first_span_index,
                            second_span_index: pair.second_span_index,
                            kind: CurveIntersectionPairBlockerKind2::Uncertain(blocker.reason()),
                        });
                        continue;
                    }
                    Err(ExactCurveError::Invalid { cause, .. }) => {
                        return Err(ExactCurveError::invalid(
                            CurveOperation2::Intersection,
                            self.data.first.family(),
                            cause,
                        ));
                    }
                },
                CurveSpanPairState::RetainedLineageOverlap { .. } => {
                    unreachable!("retained lineage overlap returned before contact replay")
                }
            };
            match span_contacts {
                RationalBezierIntersectionContacts2::NoIntersection => {}
                RationalBezierIntersectionContacts2::Contacts(span_contacts) => {
                    if let Classification::Uncertain(reason) = append_unique_contacts(
                        &mut contacts,
                        &span_contacts,
                        &first_span_range,
                        &second_span_range,
                        pair.first_span_index,
                        pair.second_span_index,
                        &self.data.policy,
                    ) {
                        blockers.push(CurveIntersectionPairBlocker2 {
                            first_span_index: pair.first_span_index,
                            second_span_index: pair.second_span_index,
                            kind: CurveIntersectionPairBlockerKind2::Uncertain(reason),
                        });
                        continue;
                    }
                }
                RationalBezierIntersectionContacts2::Overlap(overlap) => {
                    overlaps.push(CurveIntersectionOverlap2 {
                        first_span_index: pair.first_span_index,
                        second_span_index: pair.second_span_index,
                        first_range: overlap.first_range().clone(),
                        second_range: overlap.second_range().clone(),
                        orientation: overlap.orientation(),
                        parameter_correspondence: match &pair.state {
                            CurveSpanPairState::Rational(intersection) => {
                                Some(intersection.overlap_parameter_correspondence(&overlap))
                            }
                            _ => None,
                        },
                    });
                }
                RationalBezierIntersectionContacts2::Incomplete {
                    contacts: span_contacts,
                    candidates,
                } => {
                    if let Classification::Uncertain(reason) = append_unique_contacts(
                        &mut contacts,
                        &span_contacts,
                        &first_span_range,
                        &second_span_range,
                        pair.first_span_index,
                        pair.second_span_index,
                        &self.data.policy,
                    ) {
                        blockers.push(CurveIntersectionPairBlocker2 {
                            first_span_index: pair.first_span_index,
                            second_span_index: pair.second_span_index,
                            kind: CurveIntersectionPairBlockerKind2::Uncertain(reason),
                        });
                        continue;
                    }
                    blockers.push(CurveIntersectionPairBlocker2 {
                        first_span_index: pair.first_span_index,
                        second_span_index: pair.second_span_index,
                        kind: CurveIntersectionPairBlockerKind2::IncompleteReplay { candidates },
                    });
                }
                RationalBezierIntersectionContacts2::DegenerateResultant => {
                    blockers.push(CurveIntersectionPairBlocker2 {
                        first_span_index: pair.first_span_index,
                        second_span_index: pair.second_span_index,
                        kind: CurveIntersectionPairBlockerKind2::SharedComponent,
                    });
                }
            }
        }
        Ok(CurveIntersectionResult2 {
            data: Arc::new(CurveIntersectionResultData {
                span_pair_count: self.data.span_pair_count,
                contacts: contacts.into(),
                overlaps: overlaps.into(),
                blockers: blockers.into(),
            }),
        })
    }

    fn build_topology(&self) -> ExactCurveResult<CurveIntersectionTopology2> {
        let result = self.result_view()?.clone();
        if let Some(blocker) = result.blockers().first() {
            let reason = match blocker.kind() {
                CurveIntersectionPairBlockerKind2::Uncertain(reason) => *reason,
                CurveIntersectionPairBlockerKind2::IncompleteReplay { .. } => {
                    UncertaintyReason::Predicate
                }
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
                    contact.first().promoted_span_index(),
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
        let first = split_curve_spans(&self.data.first, first_parameters, &self.data.policy)?;
        let second_parameters = result
            .contacts()
            .iter()
            .map(|contact| {
                (
                    contact.second().promoted_span_index(),
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
        let second = split_curve_spans(&self.data.second, second_parameters, &self.data.policy)?;
        Ok(CurveIntersectionTopology2 {
            data: Arc::new(CurveIntersectionTopologyData {
                result,
                first: first.into(),
                second: second.into(),
                arrangement: OnceLock::new(),
            }),
        })
    }
}

impl CurveIntersectionParameter2 {
    /// Returns the promoted span index used by top-level dispatch.
    pub const fn promoted_span_index(&self) -> usize {
        self.promoted_span_index
    }

    /// Returns the promoted span's public parameter interval.
    pub const fn span_range(&self) -> &CurveSpanRange2 {
        &self.span_range
    }

    /// Returns the exact parameter in the promoted span's local `[0, 1]` domain.
    pub const fn local_parameter(&self) -> &BezierParameter2 {
        &self.local_parameter
    }

    /// Returns the exact authored curve parameter when directly represented.
    pub fn exact_curve_parameter(&self) -> Option<Real> {
        let local = self.local_parameter.as_exact()?;
        let (start, end) = self.span_range.endpoints();
        Some(start + (end - start) * local)
    }
}

impl CurveIntersectionContact2 {
    /// Returns parameter evidence on the first top-level curve.
    pub const fn first(&self) -> &CurveIntersectionParameter2 {
        &self.first
    }

    /// Returns parameter evidence on the second top-level curve.
    pub const fn second(&self) -> &CurveIntersectionParameter2 {
        &self.second
    }

    /// Returns exact affine point evidence retained by candidate replay.
    pub const fn point(&self) -> &RationalBezierIntersectionPointEvidence2 {
        &self.point
    }

    /// Returns whether retained exact evidence certifies a transverse contact.
    pub const fn is_certified_transverse(&self) -> bool {
        self.certified_transverse
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
    /// Returns the promoted span index on the first curve.
    pub const fn first_span_index(&self) -> usize {
        self.first_span_index
    }

    /// Returns the promoted span index on the second curve.
    pub const fn second_span_index(&self) -> usize {
        self.second_span_index
    }

    /// Returns the exact local overlap range on the first promoted span.
    pub const fn first_range(&self) -> &BezierParameterRange2 {
        &self.first_range
    }

    /// Returns the exact local overlap range on the second promoted span.
    ///
    /// A descending range records reversed image orientation.
    pub const fn second_range(&self) -> &BezierParameterRange2 {
        &self.second_range
    }

    /// Returns relative traversal orientation on the shared image.
    pub const fn orientation(&self) -> RationalBezierOverlapOrientation2 {
        self.orientation
    }

    pub(crate) const fn parameter_correspondence(
        &self,
    ) -> Option<&RationalBezierOverlapParameterCorrespondence2> {
        self.parameter_correspondence.as_ref()
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

    /// Returns true when every promoted span pair was completely replayed.
    pub fn is_complete(&self) -> bool {
        self.data.blockers.is_empty()
    }

    /// Returns true when complete replay certified no intersection.
    pub fn is_disjoint(&self) -> bool {
        self.is_complete() && self.data.contacts.is_empty() && self.data.overlaps.is_empty()
    }
}

impl CurveIntersectionTopology2 {
    /// Returns the complete contact result that generated this topology.
    pub fn result(&self) -> &CurveIntersectionResult2 {
        &self.data.result
    }

    /// Returns first-curve materializations in promoted span order.
    pub fn first(&self) -> &[BezierSplitMaterialization2] {
        &self.data.first
    }

    /// Returns second-curve materializations in promoted span order.
    pub fn second(&self) -> &[BezierSplitMaterialization2] {
        &self.data.second
    }

    /// Borrows the lazily assembled arrangement graph.
    pub fn arrangement_graph_view(&self) -> CurveResult<&BezierArrangementGraph2> {
        match self.data.arrangement.get_or_init(|| {
            let materializations = self
                .data
                .first
                .iter()
                .chain(self.data.second.iter())
                .cloned()
                .collect::<Vec<_>>();
            BezierArrangementGraph2::from_split_materializations(&materializations)
        }) {
            Ok(graph) => Ok(graph),
            Err(cause) => Err(cause.clone()),
        }
    }

    /// Returns an owned arrangement graph from the retained assembly.
    pub fn arrangement_graph(&self) -> CurveResult<BezierArrangementGraph2> {
        self.arrangement_graph_view().cloned()
    }
}

pub(crate) fn split_curve_spans(
    curve: &Curve2,
    parameters: impl Iterator<Item = (usize, BezierParameter2)>,
    policy: &CurveContext,
) -> ExactCurveResult<Vec<BezierSplitMaterialization2>> {
    let native_fragments =
        curve.native_bezier_fragments_for_operation(policy, CurveOperation2::Arrangement)?;
    let mut by_span = vec![Vec::new(); native_fragments.len()];
    for (span_index, parameter) in parameters {
        by_span[span_index].push(parameter);
    }
    native_fragments
        .iter()
        .zip(by_span)
        .map(|(fragment, parameters)| {
            match fragment.curve().split_at_parameters(&parameters, policy) {
                Ok(Classification::Decided(materialization)) => Ok(materialization),
                Ok(Classification::Uncertain(reason)) => Err(ExactCurveError::blocked(
                    CurveOperation2::Arrangement,
                    curve.family(),
                    reason,
                )),
                Err(cause) => Err(ExactCurveError::invalid(
                    CurveOperation2::Arrangement,
                    curve.family(),
                    cause,
                )),
            }
        })
        .collect()
}

fn append_unique_contacts(
    output: &mut Vec<CurveIntersectionContact2>,
    contacts: &[RationalBezierIntersectionContact2],
    first_span_range: &CurveSpanRange2,
    second_span_range: &CurveSpanRange2,
    first_span_index: usize,
    second_span_index: usize,
    policy: &CurveContext,
) -> Classification<()> {
    let original_len = output.len();
    for contact in contacts {
        let candidate = CurveIntersectionContact2 {
            first: CurveIntersectionParameter2 {
                promoted_span_index: first_span_index,
                span_range: first_span_range.clone(),
                local_parameter: contact.first_parameter().clone(),
            },
            second: CurveIntersectionParameter2 {
                promoted_span_index: second_span_index,
                span_range: second_span_range.clone(),
                local_parameter: contact.second_parameter().clone(),
            },
            point: contact.point().clone(),
            certified_transverse: contact.is_certified_transverse(),
        };
        match matching_contact_index(output, &candidate, policy) {
            Classification::Decided(Some(index)) => {
                // A duplicate span-pair replay can contribute stronger evidence.
                // Retain it even when the parameter pair is already present.
                output[index].certified_transverse |= candidate.certified_transverse;
            }
            Classification::Decided(None) => output.push(candidate),
            Classification::Uncertain(reason) => {
                output.truncate(original_len);
                return Classification::Uncertain(reason);
            }
        }
    }
    Classification::Decided(())
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
    first: &CurveIntersectionParameter2,
    second: &CurveIntersectionParameter2,
    policy: &CurveContext,
) -> Classification<bool> {
    if first == second {
        return Classification::Decided(true);
    }
    if first.promoted_span_index == second.promoted_span_index
        && first.span_range == second.span_range
    {
        return first
            .local_parameter
            .same_value(&second.local_parameter, policy)
            .unwrap_or(Classification::Uncertain(UncertaintyReason::Unsupported));
    }
    let (first_start, first_end) = first.span_range.endpoints();
    let (second_start, second_end) = second.span_range.endpoints();
    if matches!(
        compare_reals(first_end, second_start, policy),
        Some(std::cmp::Ordering::Less)
    ) || matches!(
        compare_reals(second_end, first_start, policy),
        Some(std::cmp::Ordering::Less)
    ) {
        return Classification::Decided(false);
    }
    let (Some(first), Some(second)) = (
        first.exact_curve_parameter(),
        second.exact_curve_parameter(),
    ) else {
        return Classification::Uncertain(UncertaintyReason::Ordering);
    };
    match compare_reals(&first, &second, policy) {
        Some(ordering) => Classification::Decided(ordering == std::cmp::Ordering::Equal),
        None => Classification::Uncertain(UncertaintyReason::Ordering),
    }
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
            result.contacts()[0].first().exact_curve_parameter(),
            Some((Real::one() / Real::from(2_i8)).unwrap())
        );
        assert_eq!(
            result.contacts()[0].second().exact_curve_parameter(),
            Some((Real::one() / Real::from(2_i8)).unwrap())
        );
    }

    #[test]
    fn batch_cache_reuses_one_circle_support_relation_across_spans() {
        let first =
            CircularArc2::try_from_center(point(2, 0), point(2, 0), point(0, 0), false).unwrap();
        let second =
            CircularArc2::try_from_center(point(3, 0), point(3, 0), point(1, 0), false).unwrap();
        let first = first
            .rational_bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value();
        let second = second
            .rational_bezier_decomposition(&CurveContext::STRICT)
            .unwrap()
            .into_value();
        let mut cache = CurveIntersectionBatchCache::default();
        for first_span in first.spans() {
            for second_span in second.spans() {
                CurveIntersectionContext::try_new_with_batch_cache(
                    &Curve2::from(first_span.curve().clone()),
                    &Curve2::from(second_span.curve().clone()),
                    &CurveContext::STRICT,
                    &mut cache,
                )
                .unwrap();
            }
        }
        assert_eq!(cache.circular_support_relations.len(), 1);
    }
}
