//! General and convex line straight-skeleton builders.
//!
//! These builders advance every line support at unit speed and process
//! certified edge, split and vertex-cluster events in exact time order; the
//! shape-preserving curve machinery remains in the parent module.

use super::*;

pub(super) type SkeletonBuildBlock = (
    StraightSkeletonStage2,
    StraightSkeletonBlocker2,
    usize,
    usize,
);

#[derive(Clone, Debug)]
pub(super) struct ActiveWavefrontCycle2 {
    source_edges: Vec<usize>,
    vertex_start_nodes: Vec<usize>,
}

#[derive(Clone, Debug)]
pub(super) struct SplitCandidate2 {
    cycle: usize,
    vertex: usize,
    target_edge: usize,
    time: Real,
    point: Point2,
}

#[derive(Clone, Debug)]
pub(super) struct VertexCandidate2 {
    cycle: usize,
    first_vertex: usize,
    second_vertex: usize,
    time: Real,
    point: Point2,
}

#[derive(Clone, Debug)]
pub(super) enum GeneralLineEvent2 {
    Edge {
        cycle: usize,
        candidate: EdgeEventCandidate2,
    },
    Split(SplitCandidate2),
    Vertex(VertexCandidate2),
}

impl GeneralLineEvent2 {
    fn time(&self) -> &Real {
        match self {
            Self::Edge { candidate, .. } => &candidate.time,
            Self::Split(candidate) => &candidate.time,
            Self::Vertex(candidate) => &candidate.time,
        }
    }
}

pub(super) fn retain_earliest_general_line_event(
    minimum_time: &mut Option<Real>,
    events: &mut Vec<GeneralLineEvent2>,
    candidate: GeneralLineEvent2,
    policy: &CurveContext,
) -> Result<(), StraightSkeletonBlocker2> {
    match minimum_time
        .as_ref()
        .map(|minimum| compare_reals(candidate.time(), minimum, policy))
    {
        None => {
            *minimum_time = Some(candidate.time().clone());
            events.push(candidate);
        }
        Some(Some(Ordering::Less)) => {
            *minimum_time = Some(candidate.time().clone());
            events.clear();
            events.push(candidate);
        }
        Some(Some(Ordering::Equal)) => events.push(candidate),
        Some(Some(Ordering::Greater)) => {}
        Some(None) => return Err(StraightSkeletonBlocker2::UncertainEventOrdering),
    }
    Ok(())
}

pub(super) fn skeleton_split_event_count(skeleton: &StraightSkeleton2) -> usize {
    fn count(kind: &StraightSkeletonNodeKind2) -> usize {
        match kind {
            StraightSkeletonNodeKind2::SplitEvent { .. }
            | StraightSkeletonNodeKind2::SupportSplitEvent { .. } => 1,
            StraightSkeletonNodeKind2::EventCluster { events } => events.iter().map(count).sum(),
            _ => 0,
        }
    }
    skeleton.nodes.iter().map(|node| count(&node.kind)).sum()
}

pub(super) fn skeleton_vertex_event_count(skeleton: &StraightSkeleton2) -> usize {
    fn count(kind: &StraightSkeletonNodeKind2) -> usize {
        match kind {
            StraightSkeletonNodeKind2::VertexEvent { .. } => 1,
            StraightSkeletonNodeKind2::EventCluster { events } => events.iter().map(count).sum(),
            _ => 0,
        }
    }
    skeleton.nodes.iter().map(|node| count(&node.kind)).sum()
}

pub(super) fn build_general_line_straight_skeleton(
    supports: &[MovingSupport2],
    source_lines: &[&crate::LineSeg2],
    orientation: RealSign,
    policy: &CurveContext,
) -> CurveResult<Result<(StraightSkeleton2, usize, usize), SkeletonBuildBlock>> {
    let source_edge_count = supports.len();
    let mut initial_cycle = ActiveWavefrontCycle2 {
        source_edges: (0..source_edge_count).collect(),
        vertex_start_nodes: (0..source_edge_count).collect(),
    };
    if merge_codirected_coincident_edges(&mut initial_cycle, supports, policy).is_none() {
        return Ok(Err((
            StraightSkeletonStage2::WavefrontPreparation,
            StraightSkeletonBlocker2::UncertainWavefrontRelation,
            0,
            0,
        )));
    }
    let mut nodes = Vec::with_capacity(source_edge_count.saturating_mul(2));
    nodes.extend(
        initial_cycle
            .vertex_start_nodes
            .iter()
            .copied()
            .map(|source_vertex| StraightSkeletonNode2 {
                point: source_lines[source_vertex].start().clone(),
                time: Real::zero(),
                kind: StraightSkeletonNodeKind2::SourceVertex { source_vertex },
            }),
    );
    initial_cycle.vertex_start_nodes = (0..initial_cycle.source_edges.len()).collect();
    let mut arcs = Vec::with_capacity(source_edge_count.saturating_mul(2));
    let mut cycles = vec![initial_cycle];
    let mut current_time = Real::zero();
    let mut event_count = 0usize;
    let mut simultaneous_event_count = 0usize;

    while cycles.iter().any(|cycle| cycle.source_edges.len() >= 3) {
        cycles.retain(|cycle| cycle.source_edges.len() >= 3);
        let mut minimum_time = None;
        let mut simultaneous = Vec::with_capacity(1);
        for (cycle_index, cycle) in cycles.iter().enumerate() {
            for active_index in 0..cycle.source_edges.len() {
                match edge_event_candidate(
                    supports,
                    &cycle.source_edges,
                    active_index,
                    &current_time,
                    policy,
                )? {
                    Ok(Some(candidate)) => {
                        if let Err(blocker) = retain_earliest_general_line_event(
                            &mut minimum_time,
                            &mut simultaneous,
                            GeneralLineEvent2::Edge {
                                cycle: cycle_index,
                                candidate,
                            },
                            policy,
                        ) {
                            return Ok(Err((
                                StraightSkeletonStage2::EventScheduling,
                                blocker,
                                event_count,
                                simultaneous_event_count,
                            )));
                        }
                    }
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
                        Ok(Some(candidate)) => {
                            if let Err(blocker) = retain_earliest_general_line_event(
                                &mut minimum_time,
                                &mut simultaneous,
                                GeneralLineEvent2::Split(candidate),
                                policy,
                            ) {
                                return Ok(Err((
                                    StraightSkeletonStage2::EventScheduling,
                                    blocker,
                                    event_count,
                                    simultaneous_event_count,
                                )));
                            }
                        }
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
            for first_vertex in 0..cycle.source_edges.len() {
                if active_vertex_is_reflex(cycle, source_lines, first_vertex, orientation, policy)
                    != Some(true)
                {
                    continue;
                }
                for second_vertex in (first_vertex + 1)..cycle.source_edges.len() {
                    if second_vertex == first_vertex + 1
                        || (first_vertex == 0 && second_vertex + 1 == cycle.source_edges.len())
                        || active_vertex_is_reflex(
                            cycle,
                            source_lines,
                            second_vertex,
                            orientation,
                            policy,
                        ) != Some(true)
                    {
                        continue;
                    }
                    match vertex_collision_candidate(
                        supports,
                        cycle,
                        cycle_index,
                        first_vertex,
                        second_vertex,
                        &current_time,
                        policy,
                    )? {
                        Ok(Some(candidate)) => {
                            if let Err(blocker) = retain_earliest_general_line_event(
                                &mut minimum_time,
                                &mut simultaneous,
                                GeneralLineEvent2::Vertex(candidate),
                                policy,
                            ) {
                                return Ok(Err((
                                    StraightSkeletonStage2::EventScheduling,
                                    blocker,
                                    event_count,
                                    simultaneous_event_count,
                                )));
                            }
                        }
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

        let Some(minimum_time) = minimum_time else {
            return Ok(Err((
                StraightSkeletonStage2::EventScheduling,
                StraightSkeletonBlocker2::MissingFutureEvent,
                event_count,
                simultaneous_event_count,
            )));
        };
        event_count += 1;
        if simultaneous.len() > 1 {
            simultaneous_event_count += 1;
        }

        let split_count = simultaneous
            .iter()
            .filter(|event| matches!(event, GeneralLineEvent2::Split(_)))
            .count();
        let vertex_count = simultaneous
            .iter()
            .filter(|event| matches!(event, GeneralLineEvent2::Vertex(_)))
            .count();
        if split_count + vertex_count != 0 {
            if split_count != 1 || vertex_count != 0 || simultaneous.len() != 1 {
                match topological_event_cycles_are_terminal_after_edge_collapses(
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

pub(super) fn topological_event_cycles_are_terminal_after_edge_collapses(
    cycles: &[ActiveWavefrontCycle2],
    events: &[GeneralLineEvent2],
    supports: &[MovingSupport2],
    time: &Real,
    policy: &CurveContext,
) -> Option<bool> {
    let event_cycles = events
        .iter()
        .filter_map(|event| match event {
            GeneralLineEvent2::Split(candidate) => Some(candidate.cycle),
            GeneralLineEvent2::Vertex(candidate) => Some(candidate.cycle),
            GeneralLineEvent2::Edge { .. } => None,
        })
        .collect::<BTreeSet<_>>();
    for cycle_index in event_cycles {
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

pub(super) fn terminal_support_set(
    source_edges: &[usize],
    supports: &[MovingSupport2],
    time: &Real,
    policy: &CurveContext,
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

pub(super) fn supports_are_opposed_and_coincident(
    first: &MovingSupport2,
    second: &MovingSupport2,
    time: &Real,
    policy: &CurveContext,
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
pub(super) struct StableSplitCandidate2 {
    left_source_edge: usize,
    right_source_edge: usize,
    hit_source_edge: usize,
    time: Real,
    point: Point2,
}

#[derive(Clone, Debug)]
pub(super) struct StableEdgeEvent2 {
    source_edge: usize,
    time: Real,
    point: Point2,
}

#[derive(Clone, Debug)]
pub(super) struct StableVertexCluster2 {
    source_pairs: Vec<(usize, usize)>,
    time: Real,
    point: Point2,
}

/// Apply simultaneous events at distinct exact points through stable source
/// evidence. Earlier transitions can renumber or split active cycles, so every
/// later transition is relocated by its incident source supports before use.
pub(super) fn apply_independent_simultaneous_events(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    events: &[GeneralLineEvent2],
    supports: &[MovingSupport2],
    policy: &CurveContext,
) -> CurveResult<Result<(), StraightSkeletonBlocker2>> {
    let mut splits = Vec::new();
    let mut vertices = Vec::new();
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
            GeneralLineEvent2::Vertex(candidate) => {
                let Some(cycle) = cycles.get(candidate.cycle) else {
                    return Ok(Err(StraightSkeletonBlocker2::InvalidSplitTopology));
                };
                let count = cycle.source_edges.len();
                let pairs = [
                    (
                        cycle.source_edges[(candidate.first_vertex + count - 1) % count],
                        cycle.source_edges[candidate.first_vertex],
                    ),
                    (
                        cycle.source_edges[(candidate.second_vertex + count - 1) % count],
                        cycle.source_edges[candidate.second_vertex],
                    ),
                ];
                if let Some(cluster) =
                    vertices
                        .iter_mut()
                        .find(|cluster: &&mut StableVertexCluster2| {
                            cluster.point == candidate.point && cluster.time == candidate.time
                        })
                {
                    cluster.source_pairs.extend(pairs);
                    cluster.source_pairs.sort_unstable();
                    cluster.source_pairs.dedup();
                } else {
                    vertices.push(StableVertexCluster2 {
                        source_pairs: pairs.into_iter().collect(),
                        time: candidate.time.clone(),
                        point: candidate.point.clone(),
                    });
                }
            }
        }
    }

    let topological_points = splits
        .iter()
        .map(|split| &split.point)
        .chain(vertices.iter().map(|vertex| &vertex.point))
        .collect::<Vec<_>>();
    for (index, point) in topological_points.iter().enumerate() {
        if topological_points
            .iter()
            .skip(index + 1)
            .any(|other| *other == *point)
            || edges.iter().any(|edge| &edge.point == *point)
        {
            return Ok(Err(StraightSkeletonBlocker2::DegenerateSimultaneousEvents));
        }
    }

    vertices.sort_by(|first, second| first.source_pairs.cmp(&second.source_pairs));
    for stable in vertices {
        let vertex = match relocate_vertex_cluster(cycles, &stable) {
            Some(candidate) => candidate,
            None => return Ok(Err(StraightSkeletonBlocker2::InvalidSplitTopology)),
        };
        if let Err(blocker) =
            apply_general_vertex_cluster(cycles, nodes, arcs, &vertex, supports, policy)?
        {
            return Ok(Err(blocker));
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

pub(super) fn relocate_split_candidate(
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

#[derive(Clone, Debug)]
pub(super) struct VertexClusterCandidate2 {
    cycle: usize,
    vertices: Vec<usize>,
    time: Real,
    point: Point2,
}

pub(super) fn relocate_vertex_cluster(
    cycles: &[ActiveWavefrontCycle2],
    stable: &StableVertexCluster2,
) -> Option<VertexClusterCandidate2> {
    let mut found = None;
    for (cycle_index, cycle) in cycles.iter().enumerate() {
        let count = cycle.source_edges.len();
        let mut vertices = Vec::new();
        for vertex in 0..count {
            let pair = (
                cycle.source_edges[(vertex + count - 1) % count],
                cycle.source_edges[vertex],
            );
            if stable.source_pairs.contains(&pair) {
                vertices.push(vertex);
            }
        }
        if vertices.len() != stable.source_pairs.len() {
            continue;
        }
        if found.is_some() {
            return None;
        }
        found = Some(VertexClusterCandidate2 {
            cycle: cycle_index,
            vertices,
            time: stable.time.clone(),
            point: stable.point.clone(),
        });
    }
    found
}

pub(super) fn active_vertex_is_reflex(
    cycle: &ActiveWavefrontCycle2,
    source_lines: &[&crate::LineSeg2],
    vertex: usize,
    orientation: RealSign,
    policy: &CurveContext,
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
pub(super) fn general_split_candidate(
    supports: &[MovingSupport2],
    source_lines: &[&crate::LineSeg2],
    cycle: &ActiveWavefrontCycle2,
    cycle_index: usize,
    vertex: usize,
    target_edge: usize,
    current_time: &Real,
    policy: &CurveContext,
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

pub(super) fn vertex_collision_candidate(
    supports: &[MovingSupport2],
    cycle: &ActiveWavefrontCycle2,
    cycle_index: usize,
    first_vertex: usize,
    second_vertex: usize,
    current_time: &Real,
    policy: &CurveContext,
) -> CurveResult<Result<Option<VertexCandidate2>, StraightSkeletonBlocker2>> {
    let count = cycle.source_edges.len();
    let trajectory = |vertex: usize| {
        vertex_trajectory(
            &supports[cycle.source_edges[(vertex + count - 1) % count]],
            &supports[cycle.source_edges[vertex]],
            policy,
        )
    };
    let first = match trajectory(first_vertex)? {
        Ok(trajectory) => trajectory,
        Err(blocker) => return Ok(Err(blocker)),
    };
    let second = match trajectory(second_vertex)? {
        Ok(trajectory) => trajectory,
        Err(blocker) => return Ok(Err(blocker)),
    };
    let delta_origin_x = &first.origin_x - &second.origin_x;
    let delta_origin_y = &first.origin_y - &second.origin_y;
    let delta_velocity_x = &first.velocity_x - &second.velocity_x;
    let delta_velocity_y = &first.velocity_y - &second.velocity_y;
    let time = match solve_collision_coordinate(&delta_origin_x, &delta_velocity_x, policy)? {
        CollisionCoordinate::Time(time) => time,
        CollisionCoordinate::Coincident => {
            match solve_collision_coordinate(&delta_origin_y, &delta_velocity_y, policy)? {
                CollisionCoordinate::Time(time) => time,
                CollisionCoordinate::Coincident | CollisionCoordinate::Never => {
                    return Ok(Ok(None));
                }
            }
        }
        CollisionCoordinate::Never => return Ok(Ok(None)),
    };
    let residual_x = &delta_origin_x + &delta_velocity_x * &time;
    let residual_y = &delta_origin_y + &delta_velocity_y * &time;
    for residual in [&residual_x, &residual_y] {
        match real_sign(residual, policy) {
            Some(RealSign::Zero) => {}
            Some(RealSign::Positive | RealSign::Negative) => return Ok(Ok(None)),
            None => return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation)),
        }
    }
    match compare_reals(&time, current_time, policy) {
        Some(Ordering::Greater) => Ok(Ok(Some(VertexCandidate2 {
            cycle: cycle_index,
            first_vertex,
            second_vertex,
            point: first.point_at(&time),
            time,
        }))),
        Some(Ordering::Less | Ordering::Equal) => Ok(Ok(None)),
        None => Ok(Err(StraightSkeletonBlocker2::UncertainEventOrdering)),
    }
}

pub(super) fn active_vertex_point(
    supports: &[MovingSupport2],
    cycle: &ActiveWavefrontCycle2,
    vertex: usize,
    time: &Real,
    policy: &CurveContext,
) -> CurveResult<Result<Point2, StraightSkeletonBlocker2>> {
    let count = cycle.source_edges.len();
    let previous = cycle.source_edges[(vertex + count - 1) % count];
    let current = cycle.source_edges[vertex];
    Ok(
        vertex_trajectory(&supports[previous], &supports[current], policy)?
            .map(|trajectory| trajectory.point_at(time)),
    )
}

pub(super) fn apply_general_vertex_cluster(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    candidate: &VertexClusterCandidate2,
    supports: &[MovingSupport2],
    policy: &CurveContext,
) -> CurveResult<Result<(), StraightSkeletonBlocker2>> {
    let cycle = cycles.remove(candidate.cycle);
    let count = cycle.source_edges.len();
    if candidate.vertices.len() < 2 {
        return Ok(Err(StraightSkeletonBlocker2::InvalidSplitTopology));
    }
    let mut vertices = candidate.vertices.clone();
    vertices.sort_unstable();
    vertices.dedup();
    let mut incident_source_edges = Vec::with_capacity(vertices.len() * 2);
    for vertex in vertices.iter().copied() {
        incident_source_edges.push(cycle.source_edges[(vertex + count - 1) % count]);
        incident_source_edges.push(cycle.source_edges[vertex]);
    }
    incident_source_edges.sort_unstable();
    incident_source_edges.dedup();
    let node = nodes.len();
    nodes.push(StraightSkeletonNode2 {
        point: candidate.point.clone(),
        time: candidate.time.clone(),
        kind: StraightSkeletonNodeKind2::VertexEvent {
            incident_source_edges,
            collapsed_source_edges: Vec::new(),
        },
    });
    for vertex in vertices.iter().copied() {
        let pair = (
            cycle.source_edges[(vertex + count - 1) % count],
            cycle.source_edges[vertex],
        );
        add_arc(
            arcs,
            cycle.vertex_start_nodes[vertex],
            node,
            StraightSkeletonArcKind2::VertexBisector {
                left_source_edge: pair.0,
                right_source_edge: pair.1,
            },
        );
    }

    let mut outputs = Vec::with_capacity(vertices.len());
    for (index, vertex) in vertices.iter().copied().enumerate() {
        let next_vertex = vertices[(index + 1) % vertices.len()];
        let end = (next_vertex + count - 1) % count;
        outputs.push(split_active_cycle(
            &cycle,
            &cyclic_index_range(vertex, end, count),
            node,
        ));
    }
    for output in &mut outputs {
        if merge_codirected_coincident_edges(output, supports, policy).is_none() {
            return Ok(Err(StraightSkeletonBlocker2::UncertainWavefrontRelation));
        }
    }
    for output in outputs.into_iter().rev() {
        match output.source_edges.len() {
            0 | 1 => {}
            2 => {
                if supports_are_opposed_and_coincident(
                    &supports[output.source_edges[0]],
                    &supports[output.source_edges[1]],
                    &candidate.time,
                    policy,
                ) != Some(true)
                {
                    return Ok(Err(StraightSkeletonBlocker2::InvalidSplitTopology));
                }
                add_arc(
                    arcs,
                    output.vertex_start_nodes[0],
                    output.vertex_start_nodes[1],
                    StraightSkeletonArcKind2::TerminalRidge,
                );
            }
            _ => cycles.insert(candidate.cycle, output),
        }
    }
    Ok(Ok(()))
}

pub(super) fn merge_codirected_coincident_edges(
    cycle: &mut ActiveWavefrontCycle2,
    supports: &[MovingSupport2],
    policy: &CurveContext,
) -> Option<bool> {
    let mut changed = false;
    loop {
        let count = cycle.source_edges.len();
        if count <= 1 {
            return Some(changed);
        }
        let mut merged = None;
        for vertex in 0..count {
            let previous = cycle.source_edges[(vertex + count - 1) % count];
            let current = cycle.source_edges[vertex];
            match supports_are_codirected_and_coincident(
                &supports[previous],
                &supports[current],
                policy,
            ) {
                Some(true) => {
                    merged = Some(vertex);
                    break;
                }
                Some(false) => {}
                None => return None,
            }
        }
        let Some(vertex) = merged else {
            return Some(changed);
        };
        cycle.source_edges.remove(vertex);
        cycle.vertex_start_nodes.remove(vertex);
        changed = true;
    }
}

pub(super) fn supports_are_codirected_and_coincident(
    first: &MovingSupport2,
    second: &MovingSupport2,
    policy: &CurveContext,
) -> Option<bool> {
    let normal_x_difference = &first.normal_x - &second.normal_x;
    let normal_y_difference = &first.normal_y - &second.normal_y;
    let constant_difference = &first.constant - &second.constant;
    match (
        real_sign(&normal_x_difference, policy),
        real_sign(&normal_y_difference, policy),
        real_sign(&constant_difference, policy),
    ) {
        (Some(RealSign::Zero), Some(RealSign::Zero), Some(RealSign::Zero)) => Some(true),
        (Some(_), Some(_), Some(_)) => Some(false),
        _ => None,
    }
}

pub(super) fn apply_general_split_event(
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

pub(super) fn cyclic_index_range(start: usize, end: usize, count: usize) -> Vec<usize> {
    let mut result = vec![start];
    let mut index = start;
    while index != end {
        index = (index + 1) % count;
        result.push(index);
    }
    result
}

pub(super) fn split_active_cycle(
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

pub(super) fn apply_general_edge_events(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    events: &[GeneralLineEvent2],
    time: &Real,
    supports: &[MovingSupport2],
    policy: &CurveContext,
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

pub(super) fn finish_terminal_cycles(
    cycles: &mut Vec<ActiveWavefrontCycle2>,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    time: &Real,
    supports: &[MovingSupport2],
    policy: &CurveContext,
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
pub(super) fn finish_terminal_parallel_cycle(
    supports: &[MovingSupport2],
    cycle: &ActiveWavefrontCycle2,
    nodes: &mut Vec<StraightSkeletonNode2>,
    arcs: &mut Vec<StraightSkeletonArc2>,
    time: &Real,
    policy: &CurveContext,
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

pub(super) fn build_convex_straight_skeleton(
    supports: &[MovingSupport2],
    source_lines: &[&crate::LineSeg2],
    policy: &CurveContext,
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

pub(super) fn vertex_trajectory(
    first: &MovingSupport2,
    second: &MovingSupport2,
    policy: &CurveContext,
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

pub(super) fn edge_event_candidate(
    supports: &[MovingSupport2],
    active: &[usize],
    active_index: usize,
    current_time: &Real,
    policy: &CurveContext,
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
            ZeroKnowledge::Zero => {}
            ZeroKnowledge::NonZero => return Ok(Ok(None)),
            ZeroKnowledge::Unknown => {
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
