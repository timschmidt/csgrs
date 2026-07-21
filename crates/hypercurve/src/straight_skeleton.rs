//! Exact straight skeletons for line contours.
//!
//! The implementation is a genuine inward wavefront construction. Every
//! source edge advances along its exact unit inward normal, adjacent moving
//! support lines define wavefront-vertex trajectories, and exact edge-collapse
//! times drive topology changes. Simultaneous collapses are handled as one
//! event. Reflex vertices are scheduled against nonincident moving edges and
//! split the active wavefront into independently propagating cycles.
//!
//! The construction follows the wavefront definition introduced by Aichholzer,
//! Aurenhammer, Alberts, and Gärtner and the edge/split event model documented
//! by CGAL; Hypercurve keeps every scheduled coordinate and time as `Real`.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperreal::{Real, RealSign, ZeroKnowledge as ZeroStatus};

use crate::classify::{compare_reals, real_sign};
use crate::{Classification, Contour2, CurvePolicy, CurveResult, Point2, Segment2};

/// Furthest stage reached by straight-skeleton construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StraightSkeletonStage2 {
    /// Source topology and orientation were being checked.
    InputValidation,
    /// Exact moving support lines and vertex trajectories were being prepared.
    WavefrontPreparation,
    /// Exact wavefront collapse events were being scheduled.
    EventScheduling,
    /// The complete supported straight-skeleton graph was materialized.
    Complete,
}

/// Explicit reason a straight skeleton was not materialized.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum StraightSkeletonBlocker2 {
    /// The contour contains a non-line segment.
    UnsupportedSegment { segment_index: usize },
    /// The source contour has a self-contact or self-intersection.
    SelfContact,
    /// Source self-contact classification was indeterminate.
    UncertainSelfContact,
    /// Exact signed area was unavailable.
    UnsupportedSignedArea,
    /// The contour has zero signed area.
    DegenerateSignedArea,
    /// Signed-area orientation could not be decided.
    UncertainOrientation,
    /// Legacy compatibility blocker from the convex-only implementation.
    ///
    /// General-position line contours now schedule split events directly, so
    /// current construction does not emit this variant.
    SplitEventsRequired { vertex_index: usize },
    /// Multiple topological event classes share one exact time.
    DegenerateSimultaneousEvents,
    /// A certified split did not yield two valid wavefront cycles.
    InvalidSplitTopology,
    /// A source vertex is collinear or otherwise locally degenerate.
    DegenerateVertex { vertex_index: usize },
    /// A local turn could not be classified exactly.
    UncertainVertexTurn { vertex_index: usize },
    /// Two active wavefront supports became parallel before terminal collapse.
    ParallelWavefrontSupports {
        first_source_edge: usize,
        second_source_edge: usize,
    },
    /// A wavefront relation could not be decided exactly.
    UncertainWavefrontRelation,
    /// Candidate collapse times could not be totally ordered.
    UncertainEventOrdering,
    /// The active wavefront had no future edge event.
    MissingFutureEvent,
    /// An event failed to advance beyond the current wavefront time.
    NonAdvancingEvent,
}

/// Kind and source evidence retained by one skeleton node.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum StraightSkeletonNodeKind2 {
    /// Source polygon vertex at wavefront time zero.
    SourceVertex { source_vertex: usize },
    /// One or more source-edge collapse events at the same point and time.
    EdgeEvent { collapsed_source_edges: Vec<usize> },
    /// A reflex vertex contacted the live interior of a nonincident edge.
    SplitEvent {
        left_source_edge: usize,
        right_source_edge: usize,
        hit_source_edge: usize,
    },
    /// Several wavefront vertices met at one non-general-position event.
    VertexEvent {
        incident_source_edges: Vec<usize>,
        collapsed_source_edges: Vec<usize>,
    },
}

/// Exact straight-skeleton graph node.
#[derive(Clone, Debug, PartialEq)]
pub struct StraightSkeletonNode2 {
    point: Point2,
    time: Real,
    kind: StraightSkeletonNodeKind2,
}

impl StraightSkeletonNode2 {
    /// Return the exact node point.
    pub const fn point(&self) -> &Point2 {
        &self.point
    }

    /// Return the exact inward wavefront time.
    pub const fn time(&self) -> &Real {
        &self.time
    }

    /// Return source/event evidence for this node.
    pub const fn kind(&self) -> &StraightSkeletonNodeKind2 {
        &self.kind
    }
}

/// Construction family of one straight-skeleton arc.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum StraightSkeletonArcKind2 {
    /// Trace of the vertex between two moving source-edge supports.
    VertexBisector {
        left_source_edge: usize,
        right_source_edge: usize,
    },
    /// Terminal ridge left when a convex wavefront collapses to a segment.
    TerminalRidge,
}

/// Indexed exact straight-skeleton graph arc.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StraightSkeletonArc2 {
    start_node: usize,
    end_node: usize,
    kind: StraightSkeletonArcKind2,
}

impl StraightSkeletonArc2 {
    /// Return the first node index.
    pub const fn start_node(&self) -> usize {
        self.start_node
    }

    /// Return the second node index.
    pub const fn end_node(&self) -> usize {
        self.end_node
    }

    /// Return the wavefront construction family.
    pub const fn kind(&self) -> &StraightSkeletonArcKind2 {
        &self.kind
    }
}

/// Exact straight-skeleton graph for one supported contour.
#[derive(Clone, Debug, PartialEq)]
pub struct StraightSkeleton2 {
    nodes: Vec<StraightSkeletonNode2>,
    arcs: Vec<StraightSkeletonArc2>,
    source_edge_count: usize,
    maximum_time: Real,
}

impl StraightSkeleton2 {
    /// Return graph nodes in deterministic construction order.
    pub fn nodes(&self) -> &[StraightSkeletonNode2] {
        &self.nodes
    }

    /// Return indexed skeleton arcs.
    pub fn arcs(&self) -> &[StraightSkeletonArc2] {
        &self.arcs
    }

    /// Return the source polygon edge count.
    pub const fn source_edge_count(&self) -> usize {
        self.source_edge_count
    }

    /// Return the last exact wavefront event time.
    pub const fn maximum_time(&self) -> &Real {
        &self.maximum_time
    }
}

/// Report for exact straight-skeleton construction.
#[derive(Clone, Debug, PartialEq)]
pub struct StraightSkeletonReport2 {
    stage: StraightSkeletonStage2,
    source_edge_count: usize,
    event_count: usize,
    simultaneous_event_count: usize,
    split_event_count: usize,
    vertex_event_count: usize,
    skeleton: Option<StraightSkeleton2>,
    blocker: Option<StraightSkeletonBlocker2>,
}

impl StraightSkeletonReport2 {
    /// Return the furthest completed construction stage.
    pub const fn stage(&self) -> StraightSkeletonStage2 {
        self.stage
    }

    /// Return the number of source edges inspected.
    pub const fn source_edge_count(&self) -> usize {
        self.source_edge_count
    }

    /// Return the number of distinct wavefront times processed.
    pub const fn event_count(&self) -> usize {
        self.event_count
    }

    /// Return the number of processed times containing multiple edge collapses.
    pub const fn simultaneous_event_count(&self) -> usize {
        self.simultaneous_event_count
    }

    /// Return the number of processed reflex split events.
    pub const fn split_event_count(&self) -> usize {
        self.split_event_count
    }

    /// Return the number of processed non-general-position vertex events.
    pub const fn vertex_event_count(&self) -> usize {
        self.vertex_event_count
    }

    /// Return the completed straight skeleton, when supported and decided.
    pub const fn skeleton(&self) -> Option<&StraightSkeleton2> {
        self.skeleton.as_ref()
    }

    /// Return the explicit construction blocker.
    pub const fn blocker(&self) -> Option<&StraightSkeletonBlocker2> {
        self.blocker.as_ref()
    }

    /// Consume the report and return its skeleton.
    pub fn into_skeleton(self) -> Option<StraightSkeleton2> {
        self.skeleton
    }
}

#[derive(Clone, Debug)]
struct MovingSupport2 {
    source_edge: usize,
    normal_x: Real,
    normal_y: Real,
    constant: Real,
}

#[derive(Clone, Debug)]
struct VertexTrajectory2 {
    origin_x: Real,
    origin_y: Real,
    velocity_x: Real,
    velocity_y: Real,
}

impl VertexTrajectory2 {
    fn point_at(&self, time: &Real) -> Point2 {
        Point2::new(
            &self.origin_x + &self.velocity_x * time,
            &self.origin_y + &self.velocity_y * time,
        )
    }
}

#[derive(Clone, Debug)]
struct EdgeEventCandidate2 {
    active_index: usize,
    time: Real,
    point: Point2,
}

impl Contour2 {
    /// Construct the exact interior straight skeleton of a simple line contour.
    ///
    /// This is an actual inward wavefront algorithm, not a centroid-ray
    /// approximation. Generic concave input uses exact reflex split events;
    /// unresolved algebraic orderings and non-general-position event clusters
    /// remain explicit blockers.
    pub fn straight_skeleton(&self, policy: &CurvePolicy) -> CurveResult<StraightSkeletonReport2> {
        let source_edge_count = self.segments().len();
        let blocked = |stage, blocker| StraightSkeletonReport2 {
            stage,
            source_edge_count,
            event_count: 0,
            simultaneous_event_count: 0,
            split_event_count: 0,
            vertex_event_count: 0,
            skeleton: None,
            blocker: Some(blocker),
        };

        let mut lines = Vec::with_capacity(source_edge_count);
        for (segment_index, segment) in self.segments().iter().enumerate() {
            match segment {
                Segment2::Line(line) => lines.push(line),
                _ => {
                    return Ok(blocked(
                        StraightSkeletonStage2::InputValidation,
                        StraightSkeletonBlocker2::UnsupportedSegment { segment_index },
                    ));
                }
            }
        }

        match self.has_self_contacts(policy)? {
            Classification::Decided(false) => {}
            Classification::Decided(true) => {
                return Ok(blocked(
                    StraightSkeletonStage2::InputValidation,
                    StraightSkeletonBlocker2::SelfContact,
                ));
            }
            Classification::Uncertain(_) => {
                return Ok(blocked(
                    StraightSkeletonStage2::InputValidation,
                    StraightSkeletonBlocker2::UncertainSelfContact,
                ));
            }
        }

        let Some(area) = self.signed_area()? else {
            return Ok(blocked(
                StraightSkeletonStage2::InputValidation,
                StraightSkeletonBlocker2::UnsupportedSignedArea,
            ));
        };
        let orientation = match real_sign(&area, policy) {
            Some(RealSign::Positive) => RealSign::Positive,
            Some(RealSign::Negative) => RealSign::Negative,
            Some(RealSign::Zero) => {
                return Ok(blocked(
                    StraightSkeletonStage2::InputValidation,
                    StraightSkeletonBlocker2::DegenerateSignedArea,
                ));
            }
            None => {
                return Ok(blocked(
                    StraightSkeletonStage2::InputValidation,
                    StraightSkeletonBlocker2::UncertainOrientation,
                ));
            }
        };

        let mut has_reflex_vertex = false;
        for vertex_index in 0..source_edge_count {
            let incoming = lines[(vertex_index + source_edge_count - 1) % source_edge_count];
            let outgoing = lines[vertex_index];
            let (incoming_x, incoming_y) = incoming.delta();
            let (outgoing_x, outgoing_y) = outgoing.delta();
            let turn = &incoming_x * &outgoing_y - &incoming_y * &outgoing_x;
            match real_sign(&turn, policy) {
                Some(RealSign::Zero) => {
                    return Ok(blocked(
                        StraightSkeletonStage2::InputValidation,
                        StraightSkeletonBlocker2::DegenerateVertex { vertex_index },
                    ));
                }
                Some(sign) if sign == orientation => {}
                Some(_) => {
                    has_reflex_vertex = true;
                }
                None => {
                    return Ok(blocked(
                        StraightSkeletonStage2::InputValidation,
                        StraightSkeletonBlocker2::UncertainVertexTurn { vertex_index },
                    ));
                }
            }
        }

        let mut supports = Vec::with_capacity(source_edge_count);
        for (source_edge, line) in lines.iter().enumerate() {
            let (dx, dy) = line.delta();
            let length = (&dx * &dx + &dy * &dy).sqrt()?;
            let (raw_normal_x, raw_normal_y) = match orientation {
                RealSign::Positive => (-dy, dx),
                RealSign::Negative => (dy, -dx),
                RealSign::Zero => unreachable!(),
            };
            let normal_x = (raw_normal_x / &length)?;
            let normal_y = (raw_normal_y / length)?;
            let constant = &normal_x * line.start().x() + &normal_y * line.start().y();
            supports.push(MovingSupport2 {
                source_edge,
                normal_x,
                normal_y,
                constant,
            });
        }

        let result = if has_reflex_vertex {
            build_general_line_straight_skeleton(&supports, &lines, orientation, policy)
        } else {
            build_convex_straight_skeleton(&supports, &lines, policy)
        }?;
        Ok(match result {
            Ok((skeleton, event_count, simultaneous_event_count)) => StraightSkeletonReport2 {
                stage: StraightSkeletonStage2::Complete,
                source_edge_count,
                event_count,
                simultaneous_event_count,
                split_event_count: skeleton_split_event_count(&skeleton),
                vertex_event_count: skeleton_vertex_event_count(&skeleton),
                skeleton: Some(skeleton),
                blocker: None,
            },
            Err((stage, blocker, event_count, simultaneous_event_count)) => {
                StraightSkeletonReport2 {
                    stage,
                    source_edge_count,
                    event_count,
                    simultaneous_event_count,
                    split_event_count: 0,
                    vertex_event_count: 0,
                    skeleton: None,
                    blocker: Some(blocker),
                }
            }
        })
    }
}

type SkeletonBuildBlock = (
    StraightSkeletonStage2,
    StraightSkeletonBlocker2,
    usize,
    usize,
);

#[derive(Clone, Debug)]
struct ActiveWavefrontCycle2 {
    source_edges: Vec<usize>,
    vertex_start_nodes: Vec<usize>,
}

#[derive(Clone, Debug)]
struct SplitCandidate2 {
    cycle: usize,
    vertex: usize,
    target_edge: usize,
    time: Real,
    point: Point2,
}

#[derive(Clone, Debug)]
enum GeneralLineEvent2 {
    Edge {
        cycle: usize,
        candidate: EdgeEventCandidate2,
    },
    Split(SplitCandidate2),
}

impl GeneralLineEvent2 {
    fn time(&self) -> &Real {
        match self {
            Self::Edge { candidate, .. } => &candidate.time,
            Self::Split(candidate) => &candidate.time,
        }
    }
}

fn skeleton_split_event_count(skeleton: &StraightSkeleton2) -> usize {
    skeleton
        .nodes
        .iter()
        .filter(|node| matches!(node.kind, StraightSkeletonNodeKind2::SplitEvent { .. }))
        .count()
}

fn skeleton_vertex_event_count(skeleton: &StraightSkeleton2) -> usize {
    skeleton
        .nodes
        .iter()
        .filter(|node| matches!(node.kind, StraightSkeletonNodeKind2::VertexEvent { .. }))
        .count()
}

fn build_general_line_straight_skeleton(
    supports: &[MovingSupport2],
    source_lines: &[&crate::LineSeg2],
    orientation: RealSign,
    policy: &CurvePolicy,
) -> CurveResult<Result<(StraightSkeleton2, usize, usize), SkeletonBuildBlock>> {
    let source_edge_count = supports.len();
    let mut nodes = source_lines
        .iter()
        .enumerate()
        .map(|(source_vertex, line)| StraightSkeletonNode2 {
            point: line.start().clone(),
            time: Real::zero(),
            kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
        })
        .collect::<Vec<_>>();
    let mut arcs = Vec::new();
    let mut cycles = vec![ActiveWavefrontCycle2 {
        source_edges: (0..source_edge_count).collect(),
        vertex_start_nodes: (0..source_edge_count).collect(),
    }];
    let mut current_time = Real::zero();
    let mut event_count = 0usize;
    let mut simultaneous_event_count = 0usize;

    while cycles.iter().any(|cycle| cycle.source_edges.len() >= 3) {
        cycles.retain(|cycle| cycle.source_edges.len() >= 3);
        let mut candidates = Vec::new();
        for (cycle_index, cycle) in cycles.iter().enumerate() {
            for active_index in 0..cycle.source_edges.len() {
                match edge_event_candidate(
                    supports,
                    &cycle.source_edges,
                    active_index,
                    &current_time,
                    policy,
                )? {
                    Ok(Some(candidate)) => candidates.push(GeneralLineEvent2::Edge {
                        cycle: cycle_index,
                        candidate,
                    }),
                    Ok(None) => {}
                    Err(blocker) => {
                        return Ok(Err((
                            StraightSkeletonStage2::EventScheduling,
                            blocker,
                            event_count,
                            simultaneous_event_count,
                        )));
                    }
                }
            }
            for vertex in 0..cycle.source_edges.len() {
                match active_vertex_is_reflex(cycle, source_lines, vertex, orientation, policy) {
                    Some(false) => continue,
                    Some(true) => {}
                    None => {
                        return Ok(Err((
                            StraightSkeletonStage2::EventScheduling,
                            StraightSkeletonBlocker2::UncertainWavefrontRelation,
                            event_count,
                            simultaneous_event_count,
                        )));
                    }
                }
                for target_edge in 0..cycle.source_edges.len() {
                    let previous =
                        (vertex + cycle.source_edges.len() - 1) % cycle.source_edges.len();
                    if target_edge == vertex || target_edge == previous {
                        continue;
                    }
                    match general_split_candidate(
                        supports,
                        source_lines,
                        cycle,
                        cycle_index,
                        vertex,
                        target_edge,
                        &current_time,
                        policy,
                    )? {
                        Ok(Some(candidate)) => candidates.push(GeneralLineEvent2::Split(candidate)),
                        Ok(None) => {}
                        Err(blocker) => {
                            return Ok(Err((
                                StraightSkeletonStage2::EventScheduling,
                                blocker,
                                event_count,
                                simultaneous_event_count,
                            )));
                        }
                    }
                }
            }
        }

        let Some(mut minimum_time) = candidates.first().map(|event| event.time().clone()) else {
            return Ok(Err((
                StraightSkeletonStage2::EventScheduling,
                StraightSkeletonBlocker2::MissingFutureEvent,
                event_count,
                simultaneous_event_count,
            )));
        };
        for candidate in candidates.iter().skip(1) {
            match compare_reals(candidate.time(), &minimum_time, policy) {
                Some(Ordering::Less) => minimum_time = candidate.time().clone(),
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => {
                    return Ok(Err((
                        StraightSkeletonStage2::EventScheduling,
                        StraightSkeletonBlocker2::UncertainEventOrdering,
                        event_count,
                        simultaneous_event_count,
                    )));
                }
            }
        }
        let mut simultaneous = Vec::new();
        for candidate in candidates {
            match compare_reals(candidate.time(), &minimum_time, policy) {
                Some(Ordering::Equal) => simultaneous.push(candidate),
                Some(Ordering::Less | Ordering::Greater) => {}
                None => {
                    return Ok(Err((
                        StraightSkeletonStage2::EventScheduling,
                        StraightSkeletonBlocker2::UncertainEventOrdering,
                        event_count,
                        simultaneous_event_count,
                    )));
                }
            }
        }
        event_count += 1;
        if simultaneous.len() > 1 {
            simultaneous_event_count += 1;
        }

        let split_count = simultaneous
            .iter()
            .filter(|event| matches!(event, GeneralLineEvent2::Split(_)))
            .count();
        if split_count != 0 {
            if split_count != 1 || simultaneous.len() != 1 {
                match split_cycles_are_terminal_after_edge_collapses(
                    &cycles,
                    &simultaneous,
                    supports,
                    &minimum_time,
                    policy,
                ) {
                    Some(true) => {
                        let edge_events = simultaneous
                            .iter()
                            .filter(|event| matches!(event, GeneralLineEvent2::Edge { .. }))
                            .cloned()
                            .collect::<Vec<_>>();
                        if let Err(blocker) = apply_general_edge_events(
                            &mut cycles,
                            &mut nodes,
                            &mut arcs,
                            &edge_events,
                            &minimum_time,
                            supports,
                            policy,
                        )? {
                            return Ok(Err((
                                StraightSkeletonStage2::EventScheduling,
                                blocker,
                                event_count,
                                simultaneous_event_count,
                            )));
                        }
                        if let Err(blocker) = finish_terminal_cycles(
                            &mut cycles,
                            &mut nodes,
                            &mut arcs,
                            &minimum_time,
                            supports,
                            policy,
                        )? {
                            return Ok(Err((
                                StraightSkeletonStage2::EventScheduling,
                                blocker,
                                event_count,
                                simultaneous_event_count,
                            )));
                        }
                    }
                    Some(false) => {
                        if let Err(blocker) = apply_independent_simultaneous_events(
                            &mut cycles,
                            &mut nodes,
                            &mut arcs,
                            &simultaneous,
                            supports,
                            policy,
                        )? {
                            return Ok(Err((
                                StraightSkeletonStage2::EventScheduling,
                                blocker,
                                event_count,
                                simultaneous_event_count,
                            )));
                        }
                    }
                    None => {
                        return Ok(Err((
                            StraightSkeletonStage2::EventScheduling,
                            StraightSkeletonBlocker2::UncertainWavefrontRelation,
                            event_count,
                            simultaneous_event_count,
                        )));
                    }
                }
            } else {
                let GeneralLineEvent2::Split(split) = &simultaneous[0] else {
                    unreachable!()
                };
                if let Err(blocker) =
                    apply_general_split_event(&mut cycles, &mut nodes, &mut arcs, split)
                {
                    return Ok(Err((
                        StraightSkeletonStage2::EventScheduling,
                        blocker,
                        event_count,
                        simultaneous_event_count,
                    )));
                }
            }
        } else if let Err(blocker) = apply_general_edge_events(
            &mut cycles,
            &mut nodes,
            &mut arcs,
            &simultaneous,
            &minimum_time,
            supports,
            policy,
        )? {
            return Ok(Err((
                StraightSkeletonStage2::EventScheduling,
                blocker,
                event_count,
                simultaneous_event_count,
            )));
        }
        current_time = minimum_time;
    }

    Ok(Ok((
        StraightSkeleton2 {
            nodes,
            arcs,
            source_edge_count,
            maximum_time: current_time,
        },
        event_count,
        simultaneous_event_count,
    )))
}

fn split_cycles_are_terminal_after_edge_collapses(
    cycles: &[ActiveWavefrontCycle2],
    events: &[GeneralLineEvent2],
    supports: &[MovingSupport2],
    time: &Real,
    policy: &CurvePolicy,
) -> Option<bool> {
    let split_cycles = events
        .iter()
        .filter_map(|event| match event {
            GeneralLineEvent2::Split(candidate) => Some(candidate.cycle),
            GeneralLineEvent2::Edge { .. } => None,
        })
        .collect::<BTreeSet<_>>();
    for cycle_index in split_cycles {
        let cycle = cycles.get(cycle_index)?;
        let removed = events
            .iter()
            .filter_map(|event| match event {
                GeneralLineEvent2::Edge { cycle, candidate } if *cycle == cycle_index => {
                    Some(candidate.active_index)
                }
                _ => None,
            })
            .collect::<BTreeSet<_>>();
        let survivors = cycle
            .source_edges
            .iter()
            .enumerate()
            .filter(|(index, _)| !removed.contains(index))
            .map(|(_, source_edge)| *source_edge)
            .collect::<Vec<_>>();
        if !terminal_support_set(&survivors, supports, time, policy)? {
            return Some(false);
        }
    }
    Some(true)
}

fn terminal_support_set(
    source_edges: &[usize],
    supports: &[MovingSupport2],
    time: &Real,
    policy: &CurvePolicy,
) -> Option<bool> {
    if source_edges.len() <= 1 {
        return Some(true);
    }
    for (index, source_edge) in source_edges.iter().copied().enumerate() {
        let paired =
            source_edges
                .iter()
                .copied()
                .enumerate()
                .any(|(other_index, other_source_edge)| {
                    index != other_index
                        && supports_are_opposed_and_coincident(
                            &supports[source_edge],
                            &supports[other_source_edge],
                            time,
                            policy,
                        ) == Some(true)
                });
        if !paired {
            return Some(false);
        }
    }
    Some(true)
}

fn supports_are_opposed_and_coincident(
    first: &MovingSupport2,
    second: &MovingSupport2,
    time: &Real,
    policy: &CurvePolicy,
) -> Option<bool> {
    let normal_x_sum = &first.normal_x + &second.normal_x;
    let normal_y_sum = &first.normal_y + &second.normal_y;
    let moved_constant_sum = &first.constant + &second.constant + &(time.clone() + time);
    match (
        real_sign(&normal_x_sum, policy),
        real_sign(&normal_y_sum, policy),
        real_sign(&moved_constant_sum, policy),
    ) {
        (Some(RealSign::Zero), Some(RealSign::Zero), Some(RealSign::Zero)) => Some(true),
        (Some(_), Some(_), Some(_)) => Some(false),
        _ => None,
    }
}

#[derive(Clone, Debug)]
struct StableSplitCandidate2 {
    left_source_edge: usize,
    right_source_edge: usize,
    hit_source_edge: usize,
    time: Real,
    point: Point2,
}

#[derive(Clone, Debug)]
struct StableEdgeEvent2 {
    source_edge: usize,
    time: Real,
    point: Point2,
}

/// Apply simultaneous events at distinct exact points through stable source
/// evidence. Earlier transitions can renumber or split active cycles, so every
/// later transition is relocated by its incident source supports before use.
fn apply_independent_simultaneous_events(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    events: &[GeneralLineEvent2],
    supports: &[MovingSupport2],
    policy: &CurvePolicy,
) -> CurveResult<Result<(), StraightSkeletonBlocker2>> {
    let mut splits = Vec::new();
    let mut edges = Vec::new();
    for event in events {
        match event {
            GeneralLineEvent2::Split(candidate) => {
                let Some(cycle) = cycles.get(candidate.cycle) else {
                    return Ok(Err(StraightSkeletonBlocker2::InvalidSplitTopology));
                };
                let count = cycle.source_edges.len();
                splits.push(StableSplitCandidate2 {
                    left_source_edge: cycle.source_edges[(candidate.vertex + count - 1) % count],
                    right_source_edge: cycle.source_edges[candidate.vertex],
                    hit_source_edge: cycle.source_edges[candidate.target_edge],
                    time: candidate.time.clone(),
                    point: candidate.point.clone(),
                });
            }
            GeneralLineEvent2::Edge { cycle, candidate } => {
                let Some(cycle) = cycles.get(*cycle) else {
                    return Ok(Err(StraightSkeletonBlocker2::InvalidSplitTopology));
                };
                edges.push(StableEdgeEvent2 {
                    source_edge: cycle.source_edges[candidate.active_index],
                    time: candidate.time.clone(),
                    point: candidate.point.clone(),
                });
            }
        }
    }

    for (index, split) in splits.iter().enumerate() {
        if splits
            .iter()
            .skip(index + 1)
            .any(|other| other.point == split.point)
            || edges.iter().any(|edge| edge.point == split.point)
        {
            return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
        }
    }

    splits.sort_by_key(|split| {
        (
            split.left_source_edge,
            split.right_source_edge,
            split.hit_source_edge,
        )
    });
    for stable in splits {
        let split = match relocate_split_candidate(cycles, &stable) {
            Some(candidate) => candidate,
            None => return Ok(Err(StraightSkeletonBlocker2::InvalidSplitTopology)),
        };
        if let Err(blocker) = apply_general_split_event(cycles, nodes, arcs, &split) {
            return Ok(Err(blocker));
        }
    }

    let mut relocated_edges = Vec::new();
    for stable in edges {
        let mut matches = Vec::new();
        for (cycle_index, cycle) in cycles.iter().enumerate() {
            for (active_index, source_edge) in cycle.source_edges.iter().copied().enumerate() {
                if source_edge != stable.source_edge {
                    continue;
                }
                let left =
                    match active_vertex_point(supports, cycle, active_index, &stable.time, policy)?
                    {
                        Ok(point) => point,
                        Err(_) => continue,
                    };
                let right = match active_vertex_point(
                    supports,
                    cycle,
                    (active_index + 1) % cycle.source_edges.len(),
                    &stable.time,
                    policy,
                )? {
                    Ok(point) => point,
                    Err(_) => continue,
                };
                if left == stable.point && right == stable.point {
                    matches.push(GeneralLineEvent2::Edge {
                        cycle: cycle_index,
                        candidate: EdgeEventCandidate2 {
                            active_index,
                            time: stable.time.clone(),
                            point: stable.point.clone(),
                        },
                    });
                }
            }
        }
        if matches.len() != 1 {
            return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
        }
        relocated_edges.push(matches.pop().unwrap());
    }
    apply_general_edge_events(
        cycles,
        nodes,
        arcs,
        &relocated_edges,
        events[0].time(),
        supports,
        policy,
    )
}

fn relocate_split_candidate(
    cycles: &[ActiveWavefrontCycle2],
    stable: &StableSplitCandidate2,
) -> Option<SplitCandidate2> {
    let mut found = None;
    for (cycle_index, cycle) in cycles.iter().enumerate() {
        let count = cycle.source_edges.len();
        for vertex in 0..count {
            let previous = cycle.source_edges[(vertex + count - 1) % count];
            let current = cycle.source_edges[vertex];
            if previous != stable.left_source_edge || current != stable.right_source_edge {
                continue;
            }
            for target_edge in 0..count {
                if cycle.source_edges[target_edge] != stable.hit_source_edge
                    || target_edge == vertex
                    || target_edge == (vertex + count - 1) % count
                {
                    continue;
                }
                if found.is_some() {
                    return None;
                }
                found = Some(SplitCandidate2 {
                    cycle: cycle_index,
                    vertex,
                    target_edge,
                    time: stable.time.clone(),
                    point: stable.point.clone(),
                });
            }
        }
    }
    found
}

fn active_vertex_is_reflex(
    cycle: &ActiveWavefrontCycle2,
    source_lines: &[&crate::LineSeg2],
    vertex: usize,
    orientation: RealSign,
    policy: &CurvePolicy,
) -> Option<bool> {
    let previous =
        cycle.source_edges[(vertex + cycle.source_edges.len() - 1) % cycle.source_edges.len()];
    let current = cycle.source_edges[vertex];
    let (incoming_x, incoming_y) = source_lines[previous].delta();
    let (outgoing_x, outgoing_y) = source_lines[current].delta();
    let turn = &incoming_x * &outgoing_y - &incoming_y * &outgoing_x;
    real_sign(&turn, policy).map(|sign| sign != RealSign::Zero && sign != orientation)
}

#[allow(clippy::too_many_arguments)]
fn general_split_candidate(
    supports: &[MovingSupport2],
    source_lines: &[&crate::LineSeg2],
    cycle: &ActiveWavefrontCycle2,
    cycle_index: usize,
    vertex: usize,
    target_edge: usize,
    current_time: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Result<Option<SplitCandidate2>, StraightSkeletonBlocker2>> {
    let count = cycle.source_edges.len();
    let previous_source = cycle.source_edges[(vertex + count - 1) % count];
    let current_source = cycle.source_edges[vertex];
    let target_source = cycle.source_edges[target_edge];
    let trajectory = match vertex_trajectory(
        &supports[previous_source],
        &supports[current_source],
        policy,
    )? {
        Ok(trajectory) => trajectory,
        Err(blocker) => return Ok(Err(blocker)),
    };
    let target = &supports[target_source];
    let origin = &target.normal_x * &trajectory.origin_x + &target.normal_y * &trajectory.origin_y
        - &target.constant;
    let velocity = &target.normal_x * &trajectory.velocity_x
        + &target.normal_y * &trajectory.velocity_y
        - Real::one();
    let time = match solve_collision_coordinate(&origin, &velocity, policy)? {
        CollisionCoordinate::Time(time) => time,
        CollisionCoordinate::Coincident | CollisionCoordinate::Never => return Ok(Ok(None)),
    };
    match compare_reals(&time, current_time, policy) {
        Some(Ordering::Greater) => {}
        Some(Ordering::Less | Ordering::Equal) => return Ok(Ok(None)),
        None => return Ok(Err(StraightSkeletonBlocker2::UncertainEventOrdering)),
    }
    let point = trajectory.point_at(&time);
    let target_start = match active_vertex_point(supports, cycle, target_edge, &time, policy)? {
        Ok(point) => point,
        Err(blocker) => return Ok(Err(blocker)),
    };
    let target_end =
        match active_vertex_point(supports, cycle, (target_edge + 1) % count, &time, policy)? {
            Ok(point) => point,
            Err(blocker) => return Ok(Err(blocker)),
        };
    let (direction_x, direction_y) = source_lines[target_source].delta();
    let query = &direction_x * point.x() + &direction_y * point.y();
    let start = &direction_x * target_start.x() + &direction_y * target_start.y();
    let end = &direction_x * target_end.x() + &direction_y * target_end.y();
    match (
        compare_reals(&start, &query, policy),
        compare_reals(&query, &end, policy),
    ) {
        (Some(Ordering::Less), Some(Ordering::Less)) => Ok(Ok(Some(SplitCandidate2 {
            cycle: cycle_index,
            vertex,
            target_edge,
            time,
            point,
        }))),
        (Some(_), Some(_)) => Ok(Ok(None)),
        _ => Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation)),
    }
}

fn active_vertex_point(
    supports: &[MovingSupport2],
    cycle: &ActiveWavefrontCycle2,
    vertex: usize,
    time: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Result<Point2, StraightSkeletonBlocker2>> {
    let count = cycle.source_edges.len();
    let previous = cycle.source_edges[(vertex + count - 1) % count];
    let current = cycle.source_edges[vertex];
    Ok(
        vertex_trajectory(&supports[previous], &supports[current], policy)?
            .map(|trajectory| trajectory.point_at(time)),
    )
}

fn apply_general_split_event(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    split: &SplitCandidate2,
) -> Result<(), StraightSkeletonBlocker2> {
    let cycle = cycles.remove(split.cycle);
    let count = cycle.source_edges.len();
    let previous_vertex = (split.vertex + count - 1) % count;
    let left_source_edge = cycle.source_edges[previous_vertex];
    let right_source_edge = cycle.source_edges[split.vertex];
    let hit_source_edge = cycle.source_edges[split.target_edge];
    let node = nodes.len();
    nodes.push(StraightSkeletonNode2 {
        point: split.point.clone(),
        time: split.time.clone(),
        kind: StraightSkeletonNodeKind2::SplitEvent {
            left_source_edge,
            right_source_edge,
            hit_source_edge,
        },
    });
    add_arc(
        arcs,
        cycle.vertex_start_nodes[split.vertex],
        node,
        StraightSkeletonArcKind2::VertexBisector {
            left_source_edge,
            right_source_edge,
        },
    );

    let first_indices = cyclic_index_range(split.vertex, split.target_edge, count);
    let second_indices = cyclic_index_range(split.target_edge, previous_vertex, count);
    let first = split_active_cycle(&cycle, &first_indices, node);
    let second = split_active_cycle(&cycle, &second_indices, node);
    if first.source_edges.len() < 3 || second.source_edges.len() < 3 {
        return Err(StraightSkeletonBlocker2::InvalidSplitTopology);
    }
    cycles.insert(split.cycle, second);
    cycles.insert(split.cycle, first);
    Ok(())
}

fn cyclic_index_range(start: usize, end: usize, count: usize) -> Vec<usize> {
    let mut result = vec![start];
    let mut index = start;
    while index != end {
        index = (index + 1) % count;
        result.push(index);
    }
    result
}

fn split_active_cycle(
    source: &ActiveWavefrontCycle2,
    indices: &[usize],
    split_node: usize,
) -> ActiveWavefrontCycle2 {
    let source_edges = indices
        .iter()
        .map(|index| source.source_edges[*index])
        .collect::<Vec<_>>();
    let mut vertex_start_nodes = Vec::with_capacity(indices.len());
    vertex_start_nodes.push(split_node);
    vertex_start_nodes.extend(
        indices
            .iter()
            .skip(1)
            .map(|index| source.vertex_start_nodes[*index]),
    );
    ActiveWavefrontCycle2 {
        source_edges,
        vertex_start_nodes,
    }
}

fn apply_general_edge_events(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    events: &[GeneralLineEvent2],
    time: &Real,
    supports: &[MovingSupport2],
    policy: &CurvePolicy,
) -> CurveResult<Result<(), StraightSkeletonBlocker2>> {
    let mut by_cycle = BTreeMap::<usize, Vec<EdgeEventCandidate2>>::new();
    for event in events {
        let GeneralLineEvent2::Edge { cycle, candidate } = event else {
            unreachable!()
        };
        by_cycle.entry(*cycle).or_default().push(candidate.clone());
    }
    for (cycle_index, mut collapsing) in by_cycle.into_iter().rev() {
        collapsing.sort_by_key(|candidate| candidate.active_index);
        let cycle = cycles.remove(cycle_index);
        let count = cycle.source_edges.len();
        let mut removed = BTreeSet::new();
        let mut event_node_by_edge = BTreeMap::new();
        let mut event_nodes = Vec::new();
        for candidate in &collapsing {
            removed.insert(candidate.active_index);
            let collapsed = cycle.source_edges[candidate.active_index];
            let node = event_node(
                nodes,
                &mut event_nodes,
                candidate.point.clone(),
                time.clone(),
                collapsed,
            );
            event_node_by_edge.insert(candidate.active_index, node);
            for vertex in [candidate.active_index, (candidate.active_index + 1) % count] {
                add_arc(
                    arcs,
                    cycle.vertex_start_nodes[vertex],
                    node,
                    StraightSkeletonArcKind2::VertexBisector {
                        left_source_edge: cycle.source_edges[(vertex + count - 1) % count],
                        right_source_edge: cycle.source_edges[vertex],
                    },
                );
            }
        }
        let survivors = (0..count)
            .filter(|index| !removed.contains(index))
            .collect::<Vec<_>>();
        if survivors.len() <= 1 {
            continue;
        }
        if survivors.len() == 2 {
            let unique_nodes = event_nodes.into_iter().collect::<BTreeSet<_>>();
            if unique_nodes.len() != 2 {
                return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
            }
            let mut unique_nodes = unique_nodes.into_iter();
            add_arc(
                arcs,
                unique_nodes.next().unwrap(),
                unique_nodes.next().unwrap(),
                StraightSkeletonArcKind2::TerminalRidge,
            );
            continue;
        }

        let source_edges = survivors
            .iter()
            .map(|index| cycle.source_edges[*index])
            .collect::<Vec<_>>();
        let mut vertex_start_nodes = Vec::with_capacity(survivors.len());
        for (new_index, old_index) in survivors.iter().copied().enumerate() {
            let previous_old = survivors[(new_index + survivors.len() - 1) % survivors.len()];
            if (previous_old + 1) % count == old_index {
                vertex_start_nodes.push(cycle.vertex_start_nodes[old_index]);
                continue;
            }
            let mut cursor = (previous_old + 1) % count;
            let mut bridge_node = None;
            while cursor != old_index {
                if let Some(node) = event_node_by_edge.get(&cursor) {
                    bridge_node = Some(*node);
                }
                cursor = (cursor + 1) % count;
            }
            let Some(bridge_node) = bridge_node else {
                return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation));
            };
            vertex_start_nodes.push(bridge_node);
        }
        let next_cycle = ActiveWavefrontCycle2 {
            source_edges,
            vertex_start_nodes,
        };
        match finish_terminal_parallel_cycle(supports, &next_cycle, nodes, arcs, time, policy)? {
            Ok(true) => {}
            Ok(false) => cycles.insert(cycle_index, next_cycle),
            Err(blocker) => return Ok(Err(blocker)),
        }
    }
    Ok(Ok(()))
}

fn finish_terminal_cycles(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    time: &Real,
    supports: &[MovingSupport2],
    policy: &CurvePolicy,
) -> CurveResult<Result<(), StraightSkeletonBlocker2>> {
    for cycle_index in (0..cycles.len()).rev() {
        match finish_terminal_parallel_cycle(
            supports,
            &cycles[cycle_index],
            nodes,
            arcs,
            time,
            policy,
        )? {
            Ok(true) => {
                cycles.remove(cycle_index);
            }
            Ok(false) => {}
            Err(blocker) => return Ok(Err(blocker)),
        }
    }
    Ok(Ok(()))
}

/// Finish a wavefront component whose entire support set has collapsed onto
/// coincident opposing supports at the current event time.
///
/// The one-dimensional terminal wavefront is materialized edge by edge. This
/// covers a single vertex-event point (an L or cross) as well as several exact
/// contact points joined by collapsed wavefront edges (a U or T).
fn finish_terminal_parallel_cycle(
    supports: &[MovingSupport2],
    cycle: &ActiveWavefrontCycle2,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    time: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Result<bool, StraightSkeletonBlocker2>> {
    let count = cycle.source_edges.len();
    match terminal_support_set(&cycle.source_edges, supports, time, policy) {
        Some(true) => {}
        Some(false) => return Ok(Ok(false)),
        None => {
            return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation));
        }
    }

    let mut finite_vertices = Vec::<(usize, Point2)>::new();
    let mut boundary_nodes = vec![None; count];

    for (vertex, boundary_node) in boundary_nodes.iter_mut().enumerate() {
        let previous = cycle.source_edges[(vertex + count - 1) % count];
        let current = cycle.source_edges[vertex];
        let first = &supports[previous];
        let second = &supports[current];
        let determinant = &first.normal_x * &second.normal_y - &first.normal_y * &second.normal_x;
        match real_sign(&determinant, policy) {
            Some(RealSign::Positive | RealSign::Negative) => {
                let trajectory = match vertex_trajectory(first, second, policy)? {
                    Ok(trajectory) => trajectory,
                    Err(blocker) => return Ok(Err(blocker)),
                };
                finite_vertices.push((vertex, trajectory.point_at(time)));
            }
            Some(RealSign::Zero) => {
                match supports_are_opposed_and_coincident(first, second, time, policy) {
                    Some(true) => {
                        let start_node = cycle.vertex_start_nodes[vertex];
                        if nodes[start_node].time != *time {
                            return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
                        }
                        *boundary_node = Some(start_node);
                    }
                    Some(false) => {
                        return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
                    }
                    None => {
                        return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation));
                    }
                }
            }
            None => {
                return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation));
            }
        }
    }

    if finite_vertices.is_empty() {
        return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
    }

    let mut point_nodes = Vec::<(Point2, usize)>::new();
    for (vertex, point) in finite_vertices {
        let event_node = if let Some((_, node)) = point_nodes
            .iter()
            .find(|(candidate, _)| candidate == &point)
        {
            *node
        } else {
            let mut incident_source_edges = Vec::new();
            for source_edge in cycle.source_edges.iter().copied() {
                let support = &supports[source_edge];
                let residual = &support.normal_x * point.x() + &support.normal_y * point.y()
                    - &support.constant
                    - time;
                match real_sign(&residual, policy) {
                    Some(RealSign::Zero) => incident_source_edges.push(source_edge),
                    Some(RealSign::Positive | RealSign::Negative) => {}
                    None => {
                        return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation));
                    }
                }
            }
            incident_source_edges.sort_unstable();
            incident_source_edges.dedup();
            let existing = nodes
                .iter()
                .position(|node| node.point == point && node.time == *time);
            let node = if let Some(existing) = existing {
                let collapsed_source_edges = match &nodes[existing].kind {
                    StraightSkeletonNodeKind2::EdgeEvent {
                        collapsed_source_edges,
                    } => collapsed_source_edges.clone(),
                    StraightSkeletonNodeKind2::VertexEvent {
                        collapsed_source_edges,
                        ..
                    } => collapsed_source_edges.clone(),
                    _ => Vec::new(),
                };
                nodes[existing].kind = StraightSkeletonNodeKind2::VertexEvent {
                    incident_source_edges,
                    collapsed_source_edges,
                };
                existing
            } else {
                let node = nodes.len();
                nodes.push(StraightSkeletonNode2 {
                    point: point.clone(),
                    time: time.clone(),
                    kind: StraightSkeletonNodeKind2::VertexEvent {
                        incident_source_edges,
                        collapsed_source_edges: Vec::new(),
                    },
                });
                node
            };
            point_nodes.push((point, node));
            node
        };
        boundary_nodes[vertex] = Some(event_node);
        let previous = cycle.source_edges[(vertex + count - 1) % count];
        let current = cycle.source_edges[vertex];
        add_arc(
            arcs,
            cycle.vertex_start_nodes[vertex],
            event_node,
            StraightSkeletonArcKind2::VertexBisector {
                left_source_edge: previous,
                right_source_edge: current,
            },
        );
    }

    for source_edge_index in 0..count {
        let Some(start_node) = boundary_nodes[source_edge_index] else {
            return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
        };
        let Some(end_node) = boundary_nodes[(source_edge_index + 1) % count] else {
            return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
        };
        add_arc(
            arcs,
            start_node,
            end_node,
            StraightSkeletonArcKind2::TerminalRidge,
        );
    }
    Ok(Ok(true))
}

fn build_convex_straight_skeleton(
    supports: &[MovingSupport2],
    source_lines: &[&crate::LineSeg2],
    policy: &CurvePolicy,
) -> CurveResult<Result<(StraightSkeleton2, usize, usize), SkeletonBuildBlock>> {
    let source_edge_count = supports.len();
    let mut nodes = source_lines
        .iter()
        .enumerate()
        .map(|(source_vertex, line)| StraightSkeletonNode2 {
            point: line.start().clone(),
            time: Real::zero(),
            kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
        })
        .collect::<Vec<_>>();
    let mut arcs = Vec::new();
    let mut active = (0..source_edge_count).collect::<Vec<_>>();
    let mut pair_start = BTreeMap::new();
    for index in 0..source_edge_count {
        pair_start.insert(
            (
                active[(index + source_edge_count - 1) % source_edge_count],
                active[index],
            ),
            index,
        );
    }

    let mut current_time = Real::zero();
    let mut event_count = 0usize;
    let mut simultaneous_event_count = 0usize;

    while active.len() >= 3 {
        let mut candidates = Vec::with_capacity(active.len());
        for active_index in 0..active.len() {
            match edge_event_candidate(supports, &active, active_index, &current_time, policy)? {
                Ok(Some(candidate)) => candidates.push(candidate),
                Ok(None) => {}
                Err(blocker) => {
                    return Ok(Err((
                        StraightSkeletonStage2::EventScheduling,
                        blocker,
                        event_count,
                        simultaneous_event_count,
                    )));
                }
            }
        }
        let Some(mut minimum_time) = candidates.first().map(|candidate| candidate.time.clone())
        else {
            return Ok(Err((
                StraightSkeletonStage2::EventScheduling,
                StraightSkeletonBlocker2::MissingFutureEvent,
                event_count,
                simultaneous_event_count,
            )));
        };
        for candidate in candidates.iter().skip(1) {
            match compare_reals(&candidate.time, &minimum_time, policy) {
                Some(Ordering::Less) => minimum_time = candidate.time.clone(),
                Some(_) => {}
                None => {
                    return Ok(Err((
                        StraightSkeletonStage2::EventScheduling,
                        StraightSkeletonBlocker2::UncertainEventOrdering,
                        event_count,
                        simultaneous_event_count,
                    )));
                }
            }
        }

        let mut collapsing = Vec::new();
        for candidate in candidates {
            match compare_reals(&candidate.time, &minimum_time, policy) {
                Some(Ordering::Equal) => collapsing.push(candidate),
                Some(_) => {}
                None => {
                    return Ok(Err((
                        StraightSkeletonStage2::EventScheduling,
                        StraightSkeletonBlocker2::UncertainEventOrdering,
                        event_count,
                        simultaneous_event_count,
                    )));
                }
            }
        }
        if collapsing.len() > 1 {
            simultaneous_event_count += 1;
        }
        event_count += 1;

        let old_pair_start = pair_start.clone();
        let mut removed = BTreeSet::new();
        let mut event_nodes = Vec::new();
        for candidate in &collapsing {
            let index = candidate.active_index;
            let previous = active[(index + active.len() - 1) % active.len()];
            let collapsed = active[index];
            let next = active[(index + 1) % active.len()];
            removed.insert(collapsed);

            let node = event_node(
                &mut nodes,
                &mut event_nodes,
                candidate.point.clone(),
                minimum_time.clone(),
                collapsed,
            );
            for pair in [(previous, collapsed), (collapsed, next)] {
                let Some(start_node) = old_pair_start.get(&pair).copied() else {
                    continue;
                };
                add_arc(
                    &mut arcs,
                    start_node,
                    node,
                    StraightSkeletonArcKind2::VertexBisector {
                        left_source_edge: pair.0,
                        right_source_edge: pair.1,
                    },
                );
            }
        }

        let next_active = active
            .iter()
            .copied()
            .filter(|support| !removed.contains(support))
            .collect::<Vec<_>>();
        current_time = minimum_time;

        if next_active.len() <= 1 {
            active = next_active;
            break;
        }
        if next_active.len() == 2 {
            let unique = event_nodes
                .iter()
                .copied()
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect::<Vec<_>>();
            if unique.len() == 2 {
                add_arc(
                    &mut arcs,
                    unique[0],
                    unique[1],
                    StraightSkeletonArcKind2::TerminalRidge,
                );
            }
            active = next_active;
            break;
        }

        let mut next_pair_start = BTreeMap::new();
        for index in 0..next_active.len() {
            let pair = (
                next_active[(index + next_active.len() - 1) % next_active.len()],
                next_active[index],
            );
            if let Some(node) = old_pair_start.get(&pair).copied() {
                next_pair_start.insert(pair, node);
                continue;
            }
            let trajectory = match vertex_trajectory(&supports[pair.0], &supports[pair.1], policy)?
            {
                Ok(trajectory) => trajectory,
                Err(blocker) => {
                    return Ok(Err((
                        StraightSkeletonStage2::WavefrontPreparation,
                        blocker,
                        event_count,
                        simultaneous_event_count,
                    )));
                }
            };
            let point = trajectory.point_at(&current_time);
            let Some(node) = event_nodes
                .iter()
                .copied()
                .find(|node| nodes[*node].point == point)
            else {
                return Ok(Err((
                    StraightSkeletonStage2::EventScheduling,
                    StraightSkeletonBlocker2::UncertainWavefrontRelation,
                    event_count,
                    simultaneous_event_count,
                )));
            };
            next_pair_start.insert(pair, node);
        }
        active = next_active;
        pair_start = next_pair_start;
    }

    let _ = active;
    Ok(Ok((
        StraightSkeleton2 {
            nodes,
            arcs,
            source_edge_count,
            maximum_time: current_time,
        },
        event_count,
        simultaneous_event_count,
    )))
}

fn vertex_trajectory(
    first: &MovingSupport2,
    second: &MovingSupport2,
    policy: &CurvePolicy,
) -> CurveResult<Result<VertexTrajectory2, StraightSkeletonBlocker2>> {
    let determinant = &first.normal_x * &second.normal_y - &first.normal_y * &second.normal_x;
    match real_sign(&determinant, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {}
        Some(RealSign::Zero) => {
            return Ok(Err(StraightSkeletonBlocker2::ParallelWavefrontSupports {
                first_source_edge: first.source_edge,
                second_source_edge: second.source_edge,
            }));
        }
        None => return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation)),
    }

    let origin_x = ((&first.constant * &second.normal_y) - (&first.normal_y * &second.constant))
        / &determinant;
    let origin_y = ((&first.normal_x * &second.constant) - (&first.constant * &second.normal_x))
        / &determinant;
    let velocity_x = (&second.normal_y - &first.normal_y) / &determinant;
    let velocity_y = (&first.normal_x - &second.normal_x) / determinant;
    Ok(Ok(VertexTrajectory2 {
        origin_x: origin_x?,
        origin_y: origin_y?,
        velocity_x: velocity_x?,
        velocity_y: velocity_y?,
    }))
}

fn edge_event_candidate(
    supports: &[MovingSupport2],
    active: &[usize],
    active_index: usize,
    current_time: &Real,
    policy: &CurvePolicy,
) -> CurveResult<Result<Option<EdgeEventCandidate2>, StraightSkeletonBlocker2>> {
    let previous = active[(active_index + active.len() - 1) % active.len()];
    let edge = active[active_index];
    let next = active[(active_index + 1) % active.len()];
    let left = match vertex_trajectory(&supports[previous], &supports[edge], policy)? {
        Ok(trajectory) => trajectory,
        Err(blocker) => return Ok(Err(blocker)),
    };
    let right = match vertex_trajectory(&supports[edge], &supports[next], policy)? {
        Ok(trajectory) => trajectory,
        Err(blocker) => return Ok(Err(blocker)),
    };

    let delta_origin_x = &left.origin_x - &right.origin_x;
    let delta_origin_y = &left.origin_y - &right.origin_y;
    let delta_velocity_x = &left.velocity_x - &right.velocity_x;
    let delta_velocity_y = &left.velocity_y - &right.velocity_y;
    let time = match solve_collision_coordinate(&delta_origin_x, &delta_velocity_x, policy)? {
        CollisionCoordinate::Time(time) => time,
        CollisionCoordinate::Coincident => {
            match solve_collision_coordinate(&delta_origin_y, &delta_velocity_y, policy)? {
                CollisionCoordinate::Time(time) => time,
                CollisionCoordinate::Coincident => {
                    return Ok(Err(StraightSkeletonBlocker2::NonAdvancingEvent));
                }
                CollisionCoordinate::Never => return Ok(Ok(None)),
            }
        }
        CollisionCoordinate::Never => return Ok(Ok(None)),
    };

    let residual_x = &delta_origin_x + &delta_velocity_x * &time;
    let residual_y = &delta_origin_y + &delta_velocity_y * &time;
    for residual in [&residual_x, &residual_y] {
        match residual.zero_status() {
            ZeroStatus::Zero => {}
            ZeroStatus::NonZero => return Ok(Ok(None)),
            ZeroStatus::Unknown => {
                return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation));
            }
        }
    }
    match compare_reals(&time, current_time, policy) {
        Some(Ordering::Greater) => Ok(Ok(Some(EdgeEventCandidate2 {
            active_index,
            point: left.point_at(&time),
            time,
        }))),
        Some(Ordering::Less) => Ok(Ok(None)),
        Some(Ordering::Equal) => Ok(Err(StraightSkeletonBlocker2::NonAdvancingEvent)),
        None => Ok(Err(StraightSkeletonBlocker2::UncertainEventOrdering)),
    }
}

enum CollisionCoordinate {
    Time(Real),
    Coincident,
    Never,
}

fn solve_collision_coordinate(
    origin: &Real,
    velocity: &Real,
    policy: &CurvePolicy,
) -> CurveResult<CollisionCoordinate> {
    match real_sign(velocity, policy) {
        Some(RealSign::Positive | RealSign::Negative) => {
            Ok(CollisionCoordinate::Time((-origin / velocity)?))
        }
        Some(RealSign::Zero) => match real_sign(origin, policy) {
            Some(RealSign::Zero) => Ok(CollisionCoordinate::Coincident),
            Some(_) => Ok(CollisionCoordinate::Never),
            None => Ok(CollisionCoordinate::Never),
        },
        None => Ok(CollisionCoordinate::Never),
    }
}

fn event_node(
    nodes: &mut Vec<StraightSkeletonNode2>,
    event_nodes: &mut Vec<usize>,
    point: Point2,
    time: Real,
    collapsed_source_edge: usize,
) -> usize {
    if let Some(index) = event_nodes
        .iter()
        .copied()
        .find(|index| nodes[*index].point == point && nodes[*index].time == time)
    {
        if let StraightSkeletonNodeKind2::EdgeEvent {
            collapsed_source_edges,
        } = &mut nodes[index].kind
            && !collapsed_source_edges.contains(&collapsed_source_edge)
        {
            collapsed_source_edges.push(collapsed_source_edge);
            collapsed_source_edges.sort_unstable();
        }
        return index;
    }
    let index = nodes.len();
    nodes.push(StraightSkeletonNode2 {
        point,
        time,
        kind: StraightSkeletonNodeKind2::EdgeEvent {
            collapsed_source_edges: vec![collapsed_source_edge],
        },
    });
    event_nodes.push(index);
    index
}

fn add_arc(
    arcs: &mut Vec<StraightSkeletonArc2>,
    start_node: usize,
    end_node: usize,
    kind: StraightSkeletonArcKind2,
) {
    if start_node == end_node {
        return;
    }
    let duplicate = arcs.iter().any(|arc| {
        ((arc.start_node == start_node && arc.end_node == end_node)
            || (arc.start_node == end_node && arc.end_node == start_node))
            && arc.kind == kind
    });
    if !duplicate {
        arcs.push(StraightSkeletonArc2 {
            start_node,
            end_node,
            kind,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{LineSeg2, Segment2};

    fn r(value: i32) -> Real {
        Real::from(value)
    }

    fn contour(points: &[(i32, i32)]) -> Contour2 {
        let points = points
            .iter()
            .map(|(x, y)| Point2::new(r(*x), r(*y)))
            .collect::<Vec<_>>();
        let segments = (0..points.len())
            .map(|index| {
                Segment2::Line(
                    LineSeg2::try_new(
                        points[index].clone(),
                        points[(index + 1) % points.len()].clone(),
                    )
                    .unwrap(),
                )
            })
            .collect();
        Contour2::try_new(segments).unwrap()
    }

    #[test]
    fn square_collapses_to_one_exact_center_event() {
        let report = contour(&[(0, 0), (2, 0), (2, 2), (0, 2)])
            .straight_skeleton(&CurvePolicy::certified())
            .unwrap();
        assert_eq!(report.stage(), StraightSkeletonStage2::Complete);
        assert_eq!(report.event_count(), 1);
        assert_eq!(report.simultaneous_event_count(), 1);
        let skeleton = report.skeleton().unwrap();
        assert_eq!(skeleton.nodes().len(), 5);
        assert_eq!(skeleton.arcs().len(), 4);
        let center = skeleton.nodes().last().unwrap();
        assert_eq!(center.point(), &Point2::new(r(1), r(1)));
        assert_eq!(center.time(), &r(1));
    }

    #[test]
    fn rectangle_retains_the_terminal_ridge() {
        let report = contour(&[(0, 0), (4, 0), (4, 2), (0, 2)])
            .straight_skeleton(&CurvePolicy::certified())
            .unwrap();
        let skeleton = report.skeleton().unwrap();
        assert_eq!(skeleton.nodes().len(), 6);
        assert_eq!(skeleton.arcs().len(), 5);
        assert!(
            skeleton
                .arcs()
                .iter()
                .any(|arc| arc.kind() == &StraightSkeletonArcKind2::TerminalRidge)
        );
        assert_eq!(skeleton.maximum_time(), &r(1));
    }

    #[test]
    fn clockwise_square_has_the_same_exact_collapse() {
        let report = contour(&[(0, 0), (0, 2), (2, 2), (2, 0)])
            .straight_skeleton(&CurvePolicy::certified())
            .unwrap();
        let skeleton = report.skeleton().unwrap();
        assert_eq!(
            skeleton.nodes().last().unwrap().point(),
            &Point2::new(r(1), r(1))
        );
    }

    #[test]
    fn non_general_position_l_shape_materializes_terminal_vertex_event() {
        let report = contour(&[(0, 0), (3, 0), (3, 1), (1, 1), (1, 3), (0, 3)])
            .straight_skeleton(&CurvePolicy::certified())
            .unwrap();
        assert_eq!(report.stage(), StraightSkeletonStage2::Complete);
        assert_eq!(report.vertex_event_count(), 1);
        let skeleton = report.skeleton().unwrap();
        let two = r(2);
        let half = (r(1) / two).unwrap();
        let event = skeleton
            .nodes()
            .iter()
            .find(|node| matches!(node.kind(), StraightSkeletonNodeKind2::VertexEvent { .. }))
            .unwrap();
        assert_eq!(event.point(), &Point2::new(half.clone(), half.clone()));
        assert_eq!(event.time(), &half);
        assert_eq!(
            skeleton
                .arcs()
                .iter()
                .filter(|arc| arc.kind() == &StraightSkeletonArcKind2::TerminalRidge)
                .count(),
            2
        );
    }

    #[test]
    fn general_position_concave_polygon_materializes_exact_split_topology() {
        let report = contour(&[
            (0, 0),
            (30, 0),
            (30, 24),
            (20, 24),
            (20, 7),
            (17, 11),
            (17, 24),
            (0, 24),
        ])
        .straight_skeleton(&CurvePolicy::certified())
        .unwrap();
        assert_eq!(report.stage(), StraightSkeletonStage2::Complete);
        assert_eq!(report.split_event_count(), 1);
        let skeleton = report.skeleton().unwrap();
        let four = r(4);
        let split_time = (r(7) / &four).unwrap();
        let split_x = (r(87) / four).unwrap();
        assert!(skeleton.nodes().iter().any(|node| {
            node.point() == &Point2::new(split_x.clone(), split_time.clone())
                && node.time() == &split_time
                && matches!(
                    node.kind(),
                    StraightSkeletonNodeKind2::SplitEvent {
                        left_source_edge: 3,
                        right_source_edge: 4,
                        hit_source_edge: 0,
                    }
                )
        }));
        assert!(skeleton.arcs().iter().all(|arc| {
            arc.start_node() < skeleton.nodes().len()
                && arc.end_node() < skeleton.nodes().len()
                && arc.start_node() != arc.end_node()
        }));
    }

    #[test]
    fn clockwise_general_position_concave_polygon_completes() {
        let report = contour(&[
            (0, 24),
            (17, 24),
            (17, 11),
            (20, 7),
            (20, 24),
            (30, 24),
            (30, 0),
            (0, 0),
        ])
        .straight_skeleton(&CurvePolicy::certified())
        .unwrap();
        assert_eq!(report.stage(), StraightSkeletonStage2::Complete);
        assert_eq!(report.split_event_count(), 1);
        assert!(report.skeleton().is_some());
    }

    #[test]
    fn non_general_position_line_fixtures_complete_exactly() {
        let fixtures: &[(&str, &[(i32, i32)])] = &[
            (
                "u",
                &[
                    (0, 0),
                    (6, 0),
                    (6, 6),
                    (4, 6),
                    (4, 2),
                    (2, 2),
                    (2, 6),
                    (0, 6),
                ],
            ),
            (
                "t",
                &[
                    (0, 0),
                    (6, 0),
                    (6, 2),
                    (4, 2),
                    (4, 6),
                    (2, 6),
                    (2, 2),
                    (0, 2),
                ],
            ),
            (
                "cross",
                &[
                    (2, 0),
                    (4, 0),
                    (4, 2),
                    (6, 2),
                    (6, 4),
                    (4, 4),
                    (4, 6),
                    (2, 6),
                    (2, 4),
                    (0, 4),
                    (0, 2),
                    (2, 2),
                ],
            ),
            (
                "asymmetric_u",
                &[
                    (0, 0),
                    (9, 0),
                    (9, 7),
                    (6, 7),
                    (6, 2),
                    (2, 2),
                    (2, 5),
                    (0, 5),
                ],
            ),
        ];
        for (name, points) in fixtures {
            let report = contour(points)
                .straight_skeleton(&CurvePolicy::certified())
                .unwrap();
            assert_eq!(
                report.stage(),
                StraightSkeletonStage2::Complete,
                "{name}: {:?}",
                report.blocker()
            );
            let skeleton = report.skeleton().unwrap();
            assert!(skeleton.arcs().iter().all(|arc| {
                arc.start_node() < skeleton.nodes().len()
                    && arc.end_node() < skeleton.nodes().len()
                    && arc.start_node() != arc.end_node()
            }));
        }
    }
}
