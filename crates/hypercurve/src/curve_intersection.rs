//! Top-level exact curve-pair intersection with retained span provenance.

use std::cell::OnceCell;
use std::rc::Rc;

use hyperreal::Real;

use crate::classify::compare_reals;
use crate::intersect::oriented_param_range_overlap;
use crate::{
    ArcArcIntersection, BezierArrangementGraph2, BezierParameter2, BezierParameterRange2,
    BezierSplitMaterialization2, CircleCircleRelation, CircularArc2, Classification, Curve2,
    CurveError, CurveGeometry2, CurveOperation2, CurvePolicy, CurveResult, CurveSpanProvenance2,
    ExactCurveError, ExactCurveResult, LineArcIntersection, LineArcIntersectionPoint, LineArcOrder,
    LineLineIntersection, ParamRange, Point2, PreparedRationalBezierIntersection2, RationalBezier2,
    RationalBezierIntersectionCandidates2, RationalBezierIntersectionContact2,
    RationalBezierIntersectionContacts2, RationalBezierIntersectionPointEvidence2,
    RationalBezierOverlapOrientation2, UncertaintyReason,
};

/// Exact source parameter retained for one top-level curve contact.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionParameter2 {
    promoted_span_index: usize,
    provenance: CurveSpanProvenance2,
    local_parameter: BezierParameter2,
}

/// One exact top-level curve contact with source provenance on both operands.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionContact2 {
    first: CurveIntersectionParameter2,
    second: CurveIntersectionParameter2,
    point: RationalBezierIntersectionPointEvidence2,
}

/// Certified positive-length overlap between two promoted top-level spans.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionOverlap2 {
    first_span_index: usize,
    second_span_index: usize,
    first: CurveSpanProvenance2,
    second: CurveSpanProvenance2,
    first_range: BezierParameterRange2,
    second_range: BezierParameterRange2,
    orientation: RationalBezierOverlapOrientation2,
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

/// Provenance-bearing blocker for one pair of promoted source spans.
#[derive(Clone, Debug, PartialEq)]
pub struct CurveIntersectionPairBlocker2 {
    first: CurveSpanProvenance2,
    second: CurveSpanProvenance2,
    kind: CurveIntersectionPairBlockerKind2,
}

/// Retained top-level curve intersection report.
#[derive(Clone, Debug)]
pub struct CurveIntersectionReport2 {
    data: Rc<CurveIntersectionReportData>,
}

/// Clone-shared split and arrangement topology for one complete curve-pair report.
#[derive(Clone, Debug)]
pub struct CurveIntersectionTopology2 {
    data: Rc<CurveIntersectionTopologyData>,
}

#[derive(Debug)]
struct CurveIntersectionTopologyData {
    report: CurveIntersectionReport2,
    first: Rc<[BezierSplitMaterialization2]>,
    second: Rc<[BezierSplitMaterialization2]>,
    arrangement: OnceCell<CurveResult<BezierArrangementGraph2>>,
}

#[derive(Debug)]
struct CurveIntersectionReportData {
    contacts: Rc<[CurveIntersectionContact2]>,
    overlaps: Rc<[CurveIntersectionOverlap2]>,
    blockers: Rc<[CurveIntersectionPairBlocker2]>,
}

/// Clone-shared prepared intersection facts for two top-level curves.
#[derive(Clone, Debug)]
pub struct PreparedCurveIntersection2 {
    data: Rc<PreparedCurveIntersectionData>,
}

#[derive(Debug)]
struct PreparedCurveIntersectionData {
    first: Curve2,
    second: Curve2,
    policy: CurvePolicy,
    span_pair_count: usize,
    dispatch: PreparedCurveIntersectionDispatch,
    report: OnceCell<ExactCurveResult<CurveIntersectionReport2>>,
    topology: OnceCell<ExactCurveResult<CurveIntersectionTopology2>>,
}

#[derive(Debug)]
enum PreparedCurveIntersectionDispatch {
    SpanPairs(Vec<PreparedCurveSpanPair>),
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
struct PreparedCurveSpanPair {
    first_span_index: usize,
    second_span_index: usize,
    state: PreparedCurveSpanPairState,
}

#[derive(Debug)]
enum PreparedCurveSpanPairState {
    Prepared(PreparedRationalBezierIntersection2),
    RetainedLineageOverlap {
        first_range: ParamRange,
        second_range: ParamRange,
        orientation: RationalBezierOverlapOrientation2,
    },
    Blocked(UncertaintyReason),
}

fn prepare_span_pairs(
    first_curve: &Curve2,
    second_curve: &Curve2,
    first_evaluators: &[RationalBezier2],
    second_evaluators: &[RationalBezier2],
    policy: &CurvePolicy,
) -> ExactCurveResult<Vec<PreparedCurveSpanPair>> {
    let first_fragments = first_curve.native_bezier_fragments()?;
    let second_fragments = second_curve.native_bezier_fragments()?;
    let shares_injective_lineage = first_curve.shares_certified_parameter_lineage(second_curve);
    let mut pairs = Vec::with_capacity(first_evaluators.len() * second_evaluators.len());
    for (first_span_index, first) in first_evaluators.iter().enumerate() {
        for (second_span_index, second) in second_evaluators.iter().enumerate() {
            let retained_overlap = if shares_injective_lineage {
                let (first_start, first_end) = first_fragments[first_span_index]
                    .provenance()
                    .source_parameter_range();
                let (second_start, second_end) = second_fragments[second_span_index]
                    .provenance()
                    .source_parameter_range();
                oriented_param_range_overlap(
                    &ParamRange::new(first_start.clone(), first_end.clone()),
                    &ParamRange::new(second_start.clone(), second_end.clone()),
                    policy,
                )
            } else {
                Classification::Decided(None)
            };
            let retained_overlap = match retained_overlap {
                Classification::Decided(overlap) => overlap,
                Classification::Uncertain(reason) => {
                    pairs.push(PreparedCurveSpanPair {
                        first_span_index,
                        second_span_index,
                        state: PreparedCurveSpanPairState::Blocked(reason),
                    });
                    continue;
                }
            };
            if let Some(overlap) = retained_overlap {
                pairs.push(PreparedCurveSpanPair {
                    first_span_index,
                    second_span_index,
                    state: PreparedCurveSpanPairState::RetainedLineageOverlap {
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
            let state = match first.try_prepare_intersection(second, policy) {
                Ok(prepared) => PreparedCurveSpanPairState::Prepared(prepared),
                Err(ExactCurveError::Blocked(blocker)) => {
                    PreparedCurveSpanPairState::Blocked(blocker.reason())
                }
                Err(ExactCurveError::Invalid { cause, .. }) => {
                    return Err(ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        first_curve.family(),
                        first_curve.source(),
                        cause,
                    ));
                }
            };
            pairs.push(PreparedCurveSpanPair {
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
    policy: &CurvePolicy,
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
    }))
}

fn endpoint_parameter(
    curve: &Curve2,
    point: &Point2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Option<CurveIntersectionParameter2>> {
    let fragments = curve.native_bezier_fragments()?;
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
        provenance: fragments[promoted_span_index].provenance().clone(),
        local_parameter: BezierParameter2::Exact(local_parameter),
    }))
}

fn native_line_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Option<LineLineIntersection>> {
    let (CurveGeometry2::Line(first_line), CurveGeometry2::Line(second_line)) =
        (first.geometry(), second.geometry())
    else {
        return Ok(None);
    };
    first_line
        .intersect_line(second_line, policy)
        .map(Some)
        .map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Intersection,
                first.family(),
                first.source(),
                cause,
            )
        })
}

fn native_line_arc_intersection(
    first: &Curve2,
    second: &Curve2,
    policy: &CurvePolicy,
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
            ExactCurveError::invalid(
                CurveOperation2::Intersection,
                first.family(),
                first.source(),
                cause,
            )
        })
}

fn build_native_line_report(
    first: &Curve2,
    second: &Curve2,
    relation: &LineLineIntersection,
    policy: &CurvePolicy,
) -> ExactCurveResult<CurveIntersectionReport2> {
    let first_fragment = &first.native_bezier_fragments()?[0];
    let second_fragment = &second.native_bezier_fragments()?[0];
    let contact =
        |first_parameter: Real, second_parameter: Real, point: Point2| CurveIntersectionContact2 {
            first: CurveIntersectionParameter2 {
                promoted_span_index: 0,
                provenance: first_fragment.provenance().clone(),
                local_parameter: BezierParameter2::Exact(first_parameter),
            },
            second: CurveIntersectionParameter2 {
                promoted_span_index: 0,
                provenance: second_fragment.provenance().clone(),
                local_parameter: BezierParameter2::Exact(second_parameter),
            },
            point: RationalBezierIntersectionPointEvidence2::Exact(point),
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
                        first.source(),
                        CurveError::DegenerateOverlapRange,
                    ));
                }
                None => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Intersection,
                        first.family(),
                        first.source(),
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
                    first: first_fragment.provenance().clone(),
                    second: second_fragment.provenance().clone(),
                    first_range: BezierParameterRange2::from_exact(
                        a_range.start().clone(),
                        a_range.end().clone(),
                    ),
                    second_range: BezierParameterRange2::from_exact(
                        b_range.start().clone(),
                        b_range.end().clone(),
                    ),
                    orientation,
                }],
            )
        }
        LineLineIntersection::Uncertain { reason } => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                first.family(),
                first.source(),
                *reason,
            ));
        }
    };
    Ok(CurveIntersectionReport2 {
        data: Rc::new(CurveIntersectionReportData {
            contacts: contacts.into(),
            overlaps: overlaps.into(),
            blockers: Rc::from([]),
        }),
    })
}

fn build_native_line_arc_report(
    first: &Curve2,
    second: &Curve2,
    order: LineArcOrder,
    relation: &LineArcIntersection,
    policy: &CurvePolicy,
) -> ExactCurveResult<CurveIntersectionReport2> {
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
                first.source(),
                *reason,
            ));
        }
    }
    Ok(CurveIntersectionReport2 {
        data: Rc::new(CurveIntersectionReportData {
            contacts: contacts.into(),
            overlaps: Rc::from([]),
            blockers: Rc::from([]),
        }),
    })
}

fn append_native_line_arc_contact(
    contacts: &mut Vec<CurveIntersectionContact2>,
    first: &Curve2,
    second: &Curve2,
    order: LineArcOrder,
    hit: &LineArcIntersectionPoint,
    policy: &CurvePolicy,
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
    let line_fragment = &line.native_bezier_fragments()?[0];
    let arc_fragments = arc_curve.native_bezier_fragments()?;
    let arc_evaluators = arc_curve.rational_evaluators()?;
    let arc_span_indices = arc_span_indices_for_point(arc_curve, arc, &hit.point, policy)?;
    let contact_count = contacts.len();
    for arc_span_index in arc_span_indices {
        let line_parameter = CurveIntersectionParameter2 {
            promoted_span_index: 0,
            provenance: line_fragment.provenance().clone(),
            local_parameter: BezierParameter2::Exact(hit.line_param.clone()),
        };
        let arc_parameter = CurveIntersectionParameter2 {
            promoted_span_index: arc_span_index,
            provenance: arc_fragments[arc_span_index].provenance().clone(),
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
        };
        if !contacts
            .iter()
            .any(|existing| same_contact(existing, &candidate, policy))
        {
            contacts.push(candidate);
        }
    }
    if contacts.len() == contact_count {
        return Err(ExactCurveError::blocked(
            CurveOperation2::Intersection,
            arc_curve.family(),
            arc_curve.source(),
            UncertaintyReason::Predicate,
        ));
    }
    Ok(())
}

fn parameter_range_covers_unit(
    range: &ParamRange,
    curve: &Curve2,
    policy: &CurvePolicy,
) -> ExactCurveResult<bool> {
    let (lower, upper) = match compare_reals(range.start(), range.end(), policy) {
        Some(std::cmp::Ordering::Less) => (range.start(), range.end()),
        Some(std::cmp::Ordering::Greater) => (range.end(), range.start()),
        Some(std::cmp::Ordering::Equal) => return Ok(false),
        None => {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                curve.family(),
                curve.source(),
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
    policy: &CurvePolicy,
) -> ExactCurveResult<Option<NativeArcIntersectionDispatch>> {
    let (CurveGeometry2::CircularArc(first_arc), CurveGeometry2::CircularArc(second_arc)) =
        (first.geometry(), second.geometry())
    else {
        return Ok(None);
    };
    let relation = first_arc
        .circle_relation(second_arc, policy)
        .map_err(|cause| {
            ExactCurveError::invalid(
                CurveOperation2::Intersection,
                first.family(),
                first.source(),
                cause,
            )
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

fn build_native_arc_report(
    first: &Curve2,
    second: &Curve2,
    points: &[Point2],
    policy: &CurvePolicy,
) -> ExactCurveResult<CurveIntersectionReport2> {
    let (CurveGeometry2::CircularArc(first_arc), CurveGeometry2::CircularArc(second_arc)) =
        (first.geometry(), second.geometry())
    else {
        unreachable!("native arc report requires two circular arcs")
    };
    let first_fragments = first.native_bezier_fragments()?;
    let second_fragments = second.native_bezier_fragments()?;
    let first_evaluators = first.rational_evaluators()?;
    let second_evaluators = second.rational_evaluators()?;
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
                        provenance: first_fragments[first_span_index].provenance().clone(),
                        local_parameter: native_arc_span_parameter(
                            first,
                            &first_evaluators[first_span_index],
                            point,
                            policy,
                        )?,
                    },
                    second: CurveIntersectionParameter2 {
                        promoted_span_index: second_span_index,
                        provenance: second_fragments[second_span_index].provenance().clone(),
                        local_parameter: native_arc_span_parameter(
                            second,
                            &second_evaluators[second_span_index],
                            point,
                            policy,
                        )?,
                    },
                    point: RationalBezierIntersectionPointEvidence2::Exact(point.clone()),
                };
                if !contacts
                    .iter()
                    .any(|existing| same_contact(existing, &candidate, policy))
                {
                    contacts.push(candidate);
                }
            }
        }
        if contacts.len() == contact_count {
            return Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                first.family(),
                first.source(),
                UncertaintyReason::Predicate,
            ));
        }
    }
    Ok(CurveIntersectionReport2 {
        data: Rc::new(CurveIntersectionReportData {
            contacts: contacts.into(),
            overlaps: Rc::from([]),
            blockers: Rc::from([]),
        }),
    })
}

fn build_native_coincident_arc_report(
    first: &Curve2,
    second: &Curve2,
    policy: &CurvePolicy,
) -> ExactCurveResult<CurveIntersectionReport2> {
    let (CurveGeometry2::CircularArc(first_arc), CurveGeometry2::CircularArc(second_arc)) =
        (first.geometry(), second.geometry())
    else {
        unreachable!("coincident-arc report requires two circular arcs")
    };
    let first_fragments = first.native_bezier_fragments()?;
    let second_fragments = second.native_bezier_fragments()?;
    let first_evaluators = first.rational_evaluators()?;
    let second_evaluators = second.rational_evaluators()?;
    let mut contacts = Vec::new();
    let mut overlaps = Vec::new();

    for (first_span_index, first_fragment) in first_fragments.iter().enumerate() {
        let (first_start, first_end) = first_fragment.curve().endpoints();
        let first_span = CircularArc2::try_from_center(
            first_start.clone(),
            first_end.clone(),
            first_arc.center().clone(),
            first_arc.is_clockwise(),
        )
        .map_err(|cause| native_arc_parameter_error(first, cause))?;
        for (second_span_index, second_fragment) in second_fragments.iter().enumerate() {
            let (second_start, second_end) = second_fragment.curve().endpoints();
            let second_span = CircularArc2::try_from_center(
                second_start.clone(),
                second_end.clone(),
                second_arc.center().clone(),
                second_arc.is_clockwise(),
            )
            .map_err(|cause| native_arc_parameter_error(second, cause))?;
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
                                second.source(),
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
                        first: first_fragment.provenance().clone(),
                        second: second_fragment.provenance().clone(),
                        first_range,
                        second_range,
                        orientation,
                    });
                }
                ArcArcIntersection::Uncertain { reason } => {
                    return Err(ExactCurveError::blocked(
                        CurveOperation2::Intersection,
                        first.family(),
                        first.source(),
                        reason,
                    ));
                }
            }
        }
    }

    Ok(CurveIntersectionReport2 {
        data: Rc::new(CurveIntersectionReportData {
            contacts: contacts.into(),
            overlaps: overlaps.into(),
            blockers: Rc::from([]),
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
    policy: &CurvePolicy,
) -> ExactCurveResult<()> {
    let first_fragments = first.native_bezier_fragments()?;
    let second_fragments = second.native_bezier_fragments()?;
    let first_evaluators = first.rational_evaluators()?;
    let second_evaluators = second.rational_evaluators()?;
    let candidate = CurveIntersectionContact2 {
        first: CurveIntersectionParameter2 {
            promoted_span_index: first_span_index,
            provenance: first_fragments[first_span_index].provenance().clone(),
            local_parameter: native_arc_span_parameter(
                first,
                &first_evaluators[first_span_index],
                point,
                policy,
            )?,
        },
        second: CurveIntersectionParameter2 {
            promoted_span_index: second_span_index,
            provenance: second_fragments[second_span_index].provenance().clone(),
            local_parameter: native_arc_span_parameter(
                second,
                &second_evaluators[second_span_index],
                point,
                policy,
            )?,
        },
        point: RationalBezierIntersectionPointEvidence2::Exact(point.clone()),
    };
    if !contacts
        .iter()
        .any(|existing| same_contact(existing, &candidate, policy))
    {
        contacts.push(candidate);
    }
    Ok(())
}

fn native_arc_overlap_range(
    curve: &Curve2,
    evaluator: &RationalBezier2,
    overlap: &CircularArc2,
    policy: &CurvePolicy,
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
            curve.source(),
            reason,
        )),
    }
}

fn bezier_parameter_range_covers_unit(
    range: &BezierParameterRange2,
    curve: &Curve2,
    policy: &CurvePolicy,
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
                curve.source(),
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
        (Classification::Uncertain(reason), _) | (_, Classification::Uncertain(reason)) => {
            Err(ExactCurveError::blocked(
                CurveOperation2::Intersection,
                curve.family(),
                curve.source(),
                reason,
            ))
        }
    }
}

fn arc_span_indices_for_point(
    curve: &Curve2,
    arc: &CircularArc2,
    point: &Point2,
    policy: &CurvePolicy,
) -> ExactCurveResult<Vec<usize>> {
    let fragments = curve.native_bezier_fragments()?;
    let mut indices = Vec::new();
    for (span_index, fragment) in fragments.iter().enumerate() {
        let (start, end) = fragment.curve().endpoints();
        let span =
            CircularArc2::try_from_center(start, end, arc.center().clone(), arc.is_clockwise())
                .map_err(|cause| {
                    ExactCurveError::invalid(
                        CurveOperation2::Intersection,
                        curve.family(),
                        curve.source(),
                        cause,
                    )
                })?;
        match span.contains_sweep_point(point, policy) {
            Classification::Decided(true) => indices.push(span_index),
            Classification::Decided(false) => {}
            Classification::Uncertain(reason) => {
                return Err(ExactCurveError::blocked(
                    CurveOperation2::Intersection,
                    curve.family(),
                    curve.source(),
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
    policy: &CurvePolicy,
) -> ExactCurveResult<BezierParameter2> {
    if span.control_points().len() != 3 || span.weights().len() != 3 {
        return Err(ExactCurveError::invalid(
            CurveOperation2::Intersection,
            curve.family(),
            curve.source(),
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
    ExactCurveError::invalid(
        CurveOperation2::Intersection,
        curve.family(),
        curve.source(),
        cause,
    )
}

impl Curve2 {
    /// Prepares all promoted span pairs and retains their exact resultant facts.
    pub fn try_prepare_intersection(
        &self,
        other: &Self,
        policy: &CurvePolicy,
    ) -> ExactCurveResult<PreparedCurveIntersection2> {
        let (span_pair_count, dispatch) = match native_line_intersection(self, other, policy)? {
            Some(relation) => (1, PreparedCurveIntersectionDispatch::NativeLine(relation)),
            None => {
                if let Some((order, relation)) = native_line_arc_intersection(self, other, policy)?
                {
                    let span_pair_count = self.native_bezier_fragments()?.len()
                        * other.native_bezier_fragments()?.len();
                    (
                        span_pair_count,
                        PreparedCurveIntersectionDispatch::NativeLineArc { order, relation },
                    )
                } else {
                    match native_arc_intersection(self, other, policy)? {
                        Some(native) => {
                            let span_pair_count = self.native_bezier_fragments()?.len()
                                * other.native_bezier_fragments()?.len();
                            let dispatch = match native {
                                NativeArcIntersectionDispatch::Points(points) => {
                                    PreparedCurveIntersectionDispatch::NativeArcPoints(points)
                                }
                                NativeArcIntersectionDispatch::Coincident => {
                                    PreparedCurveIntersectionDispatch::NativeCoincidentArcs
                                }
                            };
                            (span_pair_count, dispatch)
                        }
                        None => {
                            if let Some(contact) =
                                certified_singleton_aabb_endpoint_contact(self, other, policy)?
                            {
                                (
                                    1,
                                    PreparedCurveIntersectionDispatch::CertifiedEndpointContact(
                                        contact,
                                    ),
                                )
                            } else {
                                let first_evaluators = self.rational_evaluators()?;
                                let second_evaluators = other.rational_evaluators()?;
                                let span_pair_count =
                                    first_evaluators.len() * second_evaluators.len();
                                let dispatch = PreparedCurveIntersectionDispatch::SpanPairs(
                                    prepare_span_pairs(
                                        self,
                                        other,
                                        first_evaluators,
                                        second_evaluators,
                                        policy,
                                    )?,
                                );
                                (span_pair_count, dispatch)
                            }
                        }
                    }
                }
            }
        };
        Ok(PreparedCurveIntersection2 {
            data: Rc::new(PreparedCurveIntersectionData {
                first: self.clone(),
                second: other.clone(),
                policy: policy.clone(),
                span_pair_count,
                dispatch,
                report: OnceCell::new(),
                topology: OnceCell::new(),
            }),
        })
    }
}

impl PreparedCurveIntersection2 {
    /// Returns the retained first operand.
    pub fn first(&self) -> &Curve2 {
        &self.data.first
    }

    /// Returns the retained second operand.
    pub fn second(&self) -> &Curve2 {
        &self.data.second
    }

    /// Returns the exact policy captured when this pair was prepared.
    pub fn policy(&self) -> &CurvePolicy {
        &self.data.policy
    }

    /// Returns the number of promoted span pairs considered by dispatch.
    pub fn span_pair_count(&self) -> usize {
        self.data.span_pair_count
    }

    /// Returns whether complete contact replay has already been retained.
    pub fn is_report_cached(&self) -> bool {
        self.data.report.get().is_some()
    }

    /// Returns a clone-shared retained report.
    pub fn report(&self) -> ExactCurveResult<CurveIntersectionReport2> {
        self.report_view().cloned()
    }

    /// Borrows the retained report without copying contacts or blockers.
    pub fn report_view(&self) -> ExactCurveResult<&CurveIntersectionReport2> {
        match self.data.report.get_or_init(|| self.build_report()) {
            Ok(report) => Ok(report),
            Err(error) => Err(error.clone()),
        }
    }

    /// Returns whether contact-derived split topology has already been retained.
    pub fn is_topology_cached(&self) -> bool {
        self.data.topology.get().is_some()
    }

    /// Returns clone-shared split topology for a complete contact report.
    pub fn topology(&self) -> ExactCurveResult<CurveIntersectionTopology2> {
        self.topology_view().cloned()
    }

    /// Borrows contact-derived split topology without copying fragments.
    pub fn topology_view(&self) -> ExactCurveResult<&CurveIntersectionTopology2> {
        match self.data.topology.get_or_init(|| self.build_topology()) {
            Ok(topology) => Ok(topology),
            Err(error) => Err(error.clone()),
        }
    }

    fn build_report(&self) -> ExactCurveResult<CurveIntersectionReport2> {
        if let PreparedCurveIntersectionDispatch::CertifiedEndpointContact(contact) =
            &self.data.dispatch
        {
            return Ok(CurveIntersectionReport2 {
                data: Rc::new(CurveIntersectionReportData {
                    contacts: Rc::from([contact.clone()]),
                    overlaps: Rc::from([]),
                    blockers: Rc::from([]),
                }),
            });
        }
        if let PreparedCurveIntersectionDispatch::NativeLine(relation) = &self.data.dispatch {
            return build_native_line_report(
                &self.data.first,
                &self.data.second,
                relation,
                &self.data.policy,
            );
        }
        if let PreparedCurveIntersectionDispatch::NativeLineArc { order, relation } =
            &self.data.dispatch
        {
            return build_native_line_arc_report(
                &self.data.first,
                &self.data.second,
                *order,
                relation,
                &self.data.policy,
            );
        }
        if let PreparedCurveIntersectionDispatch::NativeArcPoints(points) = &self.data.dispatch {
            return build_native_arc_report(
                &self.data.first,
                &self.data.second,
                points,
                &self.data.policy,
            );
        }
        if matches!(
            self.data.dispatch,
            PreparedCurveIntersectionDispatch::NativeCoincidentArcs
        ) {
            return build_native_coincident_arc_report(
                &self.data.first,
                &self.data.second,
                &self.data.policy,
            );
        }
        let first_fragments = self.data.first.native_bezier_fragments()?;
        let second_fragments = self.data.second.native_bezier_fragments()?;
        let mut contacts = Vec::new();
        let mut overlaps = Vec::new();
        let mut blockers = Vec::new();
        let PreparedCurveIntersectionDispatch::SpanPairs(pairs) = &self.data.dispatch else {
            unreachable!("native dispatch returned before generic span replay")
        };
        for pair in pairs {
            let first_provenance = first_fragments[pair.first_span_index].provenance().clone();
            let second_provenance = second_fragments[pair.second_span_index]
                .provenance()
                .clone();
            if let PreparedCurveSpanPairState::RetainedLineageOverlap {
                first_range,
                second_range,
                orientation,
            } = &pair.state
            {
                overlaps.push(CurveIntersectionOverlap2 {
                    first_span_index: pair.first_span_index,
                    second_span_index: pair.second_span_index,
                    first: first_provenance,
                    second: second_provenance,
                    first_range: BezierParameterRange2::from_exact(
                        first_range.start().clone(),
                        first_range.end().clone(),
                    ),
                    second_range: BezierParameterRange2::from_exact(
                        second_range.start().clone(),
                        second_range.end().clone(),
                    ),
                    orientation: *orientation,
                });
                continue;
            }
            let span_contacts = match &pair.state {
                PreparedCurveSpanPairState::Blocked(reason) => {
                    blockers.push(CurveIntersectionPairBlocker2 {
                        first: first_provenance,
                        second: second_provenance,
                        kind: CurveIntersectionPairBlockerKind2::Uncertain(*reason),
                    });
                    continue;
                }
                PreparedCurveSpanPairState::Prepared(prepared) => match prepared.try_contacts() {
                    Ok(contacts) => contacts,
                    Err(ExactCurveError::Blocked(blocker)) => {
                        blockers.push(CurveIntersectionPairBlocker2 {
                            first: first_provenance,
                            second: second_provenance,
                            kind: CurveIntersectionPairBlockerKind2::Uncertain(blocker.reason()),
                        });
                        continue;
                    }
                    Err(ExactCurveError::Invalid { cause, .. }) => {
                        return Err(ExactCurveError::invalid(
                            CurveOperation2::Intersection,
                            self.data.first.family(),
                            self.data.first.source(),
                            cause,
                        ));
                    }
                },
                PreparedCurveSpanPairState::RetainedLineageOverlap { .. } => {
                    unreachable!("retained lineage overlap returned before contact replay")
                }
            };
            match span_contacts {
                RationalBezierIntersectionContacts2::NoIntersection => {}
                RationalBezierIntersectionContacts2::Contacts(span_contacts) => {
                    append_unique_contacts(
                        &mut contacts,
                        &span_contacts,
                        &first_provenance,
                        &second_provenance,
                        pair.first_span_index,
                        pair.second_span_index,
                        &self.data.policy,
                    );
                }
                RationalBezierIntersectionContacts2::Overlap(overlap) => {
                    overlaps.push(CurveIntersectionOverlap2 {
                        first_span_index: pair.first_span_index,
                        second_span_index: pair.second_span_index,
                        first: first_provenance,
                        second: second_provenance,
                        first_range: overlap.first_range().clone(),
                        second_range: overlap.second_range().clone(),
                        orientation: overlap.orientation(),
                    });
                }
                RationalBezierIntersectionContacts2::Incomplete {
                    contacts: span_contacts,
                    candidates,
                } => {
                    append_unique_contacts(
                        &mut contacts,
                        &span_contacts,
                        &first_provenance,
                        &second_provenance,
                        pair.first_span_index,
                        pair.second_span_index,
                        &self.data.policy,
                    );
                    blockers.push(CurveIntersectionPairBlocker2 {
                        first: first_provenance,
                        second: second_provenance,
                        kind: CurveIntersectionPairBlockerKind2::IncompleteReplay { candidates },
                    });
                }
                RationalBezierIntersectionContacts2::DegenerateResultant => {
                    blockers.push(CurveIntersectionPairBlocker2 {
                        first: first_provenance,
                        second: second_provenance,
                        kind: CurveIntersectionPairBlockerKind2::SharedComponent,
                    });
                }
            }
        }
        Ok(CurveIntersectionReport2 {
            data: Rc::new(CurveIntersectionReportData {
                contacts: contacts.into(),
                overlaps: overlaps.into(),
                blockers: blockers.into(),
            }),
        })
    }

    fn build_topology(&self) -> ExactCurveResult<CurveIntersectionTopology2> {
        let report = self.report_view()?.clone();
        if let Some(blocker) = report.blockers().first() {
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
                self.data.first.source(),
                reason,
            ));
        }
        let first_parameters = report
            .contacts()
            .iter()
            .map(|contact| {
                (
                    contact.first().promoted_span_index(),
                    contact.first().local_parameter().clone(),
                )
            })
            .chain(report.overlaps().iter().flat_map(|overlap| {
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
        let second_parameters = report
            .contacts()
            .iter()
            .map(|contact| {
                (
                    contact.second().promoted_span_index(),
                    contact.second().local_parameter().clone(),
                )
            })
            .chain(report.overlaps().iter().flat_map(|overlap| {
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
            data: Rc::new(CurveIntersectionTopologyData {
                report,
                first: first.into(),
                second: second.into(),
                arrangement: OnceCell::new(),
            }),
        })
    }
}

impl CurveIntersectionParameter2 {
    /// Returns the promoted span index used by top-level dispatch.
    pub const fn promoted_span_index(&self) -> usize {
        self.promoted_span_index
    }

    /// Returns source identity and exact source-span range.
    pub const fn provenance(&self) -> &CurveSpanProvenance2 {
        &self.provenance
    }

    /// Returns the exact parameter in the promoted span's local `[0, 1]` domain.
    pub const fn local_parameter(&self) -> &BezierParameter2 {
        &self.local_parameter
    }

    /// Returns the exact authored curve parameter when directly represented.
    pub fn exact_curve_parameter(&self) -> Option<Real> {
        let local = self.local_parameter.as_exact()?;
        let (start, end) = self.provenance.parameter_range();
        Some(start + (end - start) * local)
    }

    /// Returns the exact parameter in the root source curve when directly represented.
    pub fn exact_source_parameter(&self) -> Option<Real> {
        let local = self.local_parameter.as_exact()?;
        let (start, end) = self.provenance.source_parameter_range();
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
}

impl CurveIntersectionPairBlocker2 {
    /// Returns provenance for the first blocked span.
    pub const fn first(&self) -> &CurveSpanProvenance2 {
        &self.first
    }

    /// Returns provenance for the second blocked span.
    pub const fn second(&self) -> &CurveSpanProvenance2 {
        &self.second
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

    /// Returns provenance for the first shared span.
    pub const fn first(&self) -> &CurveSpanProvenance2 {
        &self.first
    }

    /// Returns provenance for the second shared span.
    pub const fn second(&self) -> &CurveSpanProvenance2 {
        &self.second
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
}

impl CurveIntersectionReport2 {
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
    /// Returns the complete contact report that generated this topology.
    pub fn report(&self) -> &CurveIntersectionReport2 {
        &self.data.report
    }

    /// Returns first-curve materializations in promoted span order.
    pub fn first(&self) -> &[BezierSplitMaterialization2] {
        &self.data.first
    }

    /// Returns second-curve materializations in promoted span order.
    pub fn second(&self) -> &[BezierSplitMaterialization2] {
        &self.data.second
    }

    /// Returns whether arrangement assembly has already been retained.
    pub fn is_arrangement_cached(&self) -> bool {
        self.data.arrangement.get().is_some()
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
    policy: &CurvePolicy,
) -> ExactCurveResult<Vec<BezierSplitMaterialization2>> {
    let native_fragments = curve.native_bezier_fragments()?;
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
                    curve.source(),
                    reason,
                )),
                Err(cause) => Err(ExactCurveError::invalid(
                    CurveOperation2::Arrangement,
                    curve.family(),
                    curve.source(),
                    cause,
                )),
            }
        })
        .collect()
}

fn append_unique_contacts(
    output: &mut Vec<CurveIntersectionContact2>,
    contacts: &[RationalBezierIntersectionContact2],
    first_provenance: &CurveSpanProvenance2,
    second_provenance: &CurveSpanProvenance2,
    first_span_index: usize,
    second_span_index: usize,
    policy: &CurvePolicy,
) {
    for contact in contacts {
        let candidate = CurveIntersectionContact2 {
            first: CurveIntersectionParameter2 {
                promoted_span_index: first_span_index,
                provenance: first_provenance.clone(),
                local_parameter: contact.first_parameter().clone(),
            },
            second: CurveIntersectionParameter2 {
                promoted_span_index: second_span_index,
                provenance: second_provenance.clone(),
                local_parameter: contact.second_parameter().clone(),
            },
            point: contact.point().clone(),
        };
        if !output
            .iter()
            .any(|existing| same_contact(existing, &candidate, policy))
        {
            output.push(candidate);
        }
    }
}

fn same_contact(
    first: &CurveIntersectionContact2,
    second: &CurveIntersectionContact2,
    policy: &CurvePolicy,
) -> bool {
    same_curve_parameter(&first.first, &second.first, policy)
        && same_curve_parameter(&first.second, &second.second, policy)
}

fn same_curve_parameter(
    first: &CurveIntersectionParameter2,
    second: &CurveIntersectionParameter2,
    policy: &CurvePolicy,
) -> bool {
    if first == second {
        return true;
    }
    let (Some(first), Some(second)) = (
        first.exact_curve_parameter(),
        second.exact_curve_parameter(),
    ) else {
        return false;
    };
    compare_reals(&first, &second, policy) == Some(std::cmp::Ordering::Equal)
}
