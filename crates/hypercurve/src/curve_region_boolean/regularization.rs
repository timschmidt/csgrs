//! Regularized fragment actions and side decisions.

use super::*;

impl<'a> CurveRegionBooleanContext<'a> {
    /// Resolves result-side actions after complete pair replay.
    ///
    /// A representative ray remains the cheapest seed for most fragments.
    /// Some exact carriers deliberately live in a larger selected field than
    /// any materialized `Real` point, however. At a degree-two authored
    /// continuation no boundary is crossed and the same two faces continue on
    /// either side of the vertex. Propagating a decided neighboring action is
    /// therefore an exact topological certificate and avoids manufacturing a
    /// second algebraic coordinate field solely for point classification.
    pub(super) fn regularized_fragment_actions(
        &self,
        topology: &CurveRegionSplitTopology,
        simple_loop_filled_side: Option<bool>,
    ) -> ExactCurveResult<RegularizedFragmentSelection> {
        if let Some(filled_side_is_left) = simple_loop_filled_side {
            let action = if filled_side_is_left {
                RegionFragmentAction::Keep
            } else {
                RegionFragmentAction::KeepReversed
            };
            let actions = topology
                .split_fragments
                .iter()
                .map(|splits| vec![action; splits.len()])
                .collect::<Vec<_>>();
            let successor_edge_ids = topology.split_fragments.iter().map(Vec::len).sum::<usize>();
            return Ok(RegularizedFragmentSelection {
                actions,
                successor_edge_ids: vec![NO_REGULARIZED_EDGE; successor_edge_ids],
            });
        }

        let mut actions = topology
            .split_fragments
            .iter()
            .map(|splits| vec![None; splits.len()])
            .collect::<Vec<_>>();
        let mut blockers = topology
            .split_fragments
            .iter()
            .map(|splits| vec![None; splits.len()])
            .collect::<Vec<_>>();
        let mut incidents = Vec::<Vec<(usize, usize, bool)>>::new();
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_index, split) in splits.iter().enumerate() {
                for (vertex, is_start) in [
                    (split.start_topology_vertex, true),
                    (split.end_topology_vertex, false),
                ] {
                    let Some(vertex) = vertex else {
                        continue;
                    };
                    if incidents.len() <= vertex {
                        incidents.resize_with(vertex + 1, Vec::new);
                    }
                    incidents[vertex].push((carrier_index, split_index, is_start));
                }
            }
        }
        let authored_successor = |incoming: (usize, usize), outgoing: (usize, usize)| {
            let (incoming_carrier, incoming_split) = incoming;
            let (outgoing_carrier, outgoing_split) = outgoing;
            if incoming_carrier == outgoing_carrier {
                return incoming_split.checked_add(1) == Some(outgoing_split);
            }
            if incoming_split.checked_add(1)
                != Some(topology.split_fragments[incoming_carrier].len())
                || outgoing_split != 0
            {
                return false;
            }
            let incoming = &self.data.carriers[incoming_carrier];
            let outgoing = &self.data.carriers[outgoing_carrier];
            if incoming.operand != outgoing.operand || incoming.loop_index != outgoing.loop_index {
                return false;
            }
            outgoing.fragment_index == incoming.fragment_index.saturating_add(1)
                || outgoing.fragment_index == 0
                    && !self.data.carriers.iter().any(|candidate| {
                        candidate.operand == incoming.operand
                            && candidate.loop_index == incoming.loop_index
                            && candidate.fragment_index > incoming.fragment_index
                    })
        };

        let mut edge_offsets = Vec::with_capacity(topology.split_fragments.len());
        let mut edge_count = 0_usize;
        for splits in &topology.split_fragments {
            edge_offsets.push(edge_count);
            edge_count = edge_count.saturating_add(splits.len());
        }
        let mut edge_sources = vec![(0_usize, 0_usize); edge_count];
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for split_index in 0..splits.len() {
                edge_sources[edge_offsets[carrier_index] + split_index] =
                    (carrier_index, split_index);
            }
        }
        let edge_index = |carrier_index: usize, split_index: usize| {
            edge_offsets[carrier_index].saturating_add(split_index)
        };
        let left_face = |edge: usize| edge.saturating_mul(2);
        let right_face = |edge: usize| edge.saturating_mul(2).saturating_add(1);
        let mut vertex_sector_links = Vec::new();
        let mut winding_sector_links = Vec::new();
        let mut transverse_winding_sector_links = Vec::new();

        // An authored continuation with no transverse event preserves both
        // local face sectors exactly.
        for (vertex, incident) in incidents.iter().enumerate() {
            // Several pair contacts can share one vertex. No single pair's
            // continuation theorem then owns the complete cyclic order.
            if incident.len() > 2
                && !topology.contact_candidates.contains_key(&vertex)
                && let Some(vertex_sectors) = regularized_incident_ray_sectors(
                    incident,
                    topology,
                    &edge_offsets,
                    &self.data.policy,
                )
            {
                if let Some(sectors) = &vertex_sectors.sectors {
                    record_regularized_vertex_sectors(&mut vertex_sector_links, vertex, sectors);
                }
                winding_sector_links.extend_from_slice(&vertex_sectors.winding_links);
                transverse_winding_sector_links.extend(vertex_sectors.winding_links);
            }
            if incident.len() != 2
                || topology
                    .transverse_vertices
                    .get(vertex)
                    .copied()
                    .unwrap_or(false)
            {
                continue;
            }
            let (incoming, outgoing) = match (incident[0], incident[1]) {
                ((carrier, split, false), (next_carrier, next_split, true)) => {
                    ((carrier, split), (next_carrier, next_split))
                }
                ((next_carrier, next_split, true), (carrier, split, false)) => {
                    ((carrier, split), (next_carrier, next_split))
                }
                _ => continue,
            };
            if !authored_successor(incoming, outgoing) {
                continue;
            }
            let incoming = edge_index(incoming.0, incoming.1);
            let outgoing = edge_index(outgoing.0, outgoing.1);
            winding_sector_links.push((left_face(incoming), left_face(outgoing)));
            winding_sector_links.push((right_face(incoming), right_face(outgoing)));
        }

        let overlap_orientation_between_edges =
            |first_edge: usize, second_edge: usize| -> ExactCurveResult<Option<bool>> {
                let (first_carrier_index, first_split_index) = edge_sources[first_edge];
                let (second_carrier_index, second_split_index) = edge_sources[second_edge];
                let first_fragment =
                    &topology.split_fragments[first_carrier_index][first_split_index].fragment;
                let second_fragment =
                    &topology.split_fragments[second_carrier_index][second_split_index].fragment;
                let first_range = first_fragment.curve_region_parameter_range();
                let second_range = second_fragment.curve_region_parameter_range();
                let mut relation = None;
                for overlap in &topology.overlaps {
                    let ranges = if overlap.first_carrier_index == first_carrier_index
                        && overlap.second_carrier_index == second_carrier_index
                    {
                        Some((&overlap.first_range, &overlap.second_range))
                    } else if overlap.second_carrier_index == first_carrier_index
                        && overlap.first_carrier_index == second_carrier_index
                    {
                        Some((&overlap.second_range, &overlap.first_range))
                    } else {
                        None
                    };
                    let Some((first_overlap, second_overlap)) = ranges else {
                        continue;
                    };
                    if !range_contains_fragment(
                        first_overlap,
                        first_range.start(),
                        first_range.end(),
                        &self.data.policy,
                    )? || !range_contains_fragment(
                        second_overlap,
                        second_range.start(),
                        second_range.end(),
                        &self.data.policy,
                    )? {
                        continue;
                    }
                    let reversed = (overlap.orientation == CurveOverlapOrientation2::Reversed)
                        ^ self.data.carriers[first_carrier_index].reversed
                        ^ self.data.carriers[second_carrier_index].reversed;
                    match relation {
                        Some(existing) if existing != reversed => {
                            return Err(self.invalid(
                                first_carrier_index,
                                CurveError::Topology(
                                    "overlap orientations disagree at a contact sector".into(),
                                ),
                            ));
                        }
                        Some(_) => {}
                        None => relation = Some(reversed),
                    }
                }
                Ok(relation)
            };

        // Interior crossing and tangent certificates fix the local face
        // sectors without materializing the contact coordinate. At authored
        // corners, order the actual branches before linking their faces.
        for (&vertex, contact) in &topology.contact_candidates {
            let Some(incident) = incidents.get(vertex) else {
                continue;
            };
            let mut branches = [None; 4];
            for &(carrier_index, split_index, is_start) in incident {
                let branch = if contact.first_carrier != contact.second_carrier {
                    if carrier_index == contact.first_carrier {
                        Some(TransitionContactBranch::First)
                    } else if carrier_index == contact.second_carrier {
                        Some(TransitionContactBranch::Second)
                    } else {
                        None
                    }
                } else {
                    let split = &topology.split_fragments[carrier_index][split_index];
                    let range = split.fragment.curve_region_parameter_range();
                    self.transition_contact_branch(
                        topology,
                        carrier_index,
                        Some(vertex),
                        if is_start ^ self.data.carriers[carrier_index].reversed {
                            range.start()
                        } else {
                            range.end()
                        },
                    )?
                };
                let Some(branch) = branch else {
                    continue;
                };
                let slot = match (branch, is_start) {
                    (TransitionContactBranch::First, false) => 0,
                    (TransitionContactBranch::First, true) => 1,
                    (TransitionContactBranch::Second, false) => 2,
                    (TransitionContactBranch::Second, true) => 3,
                };
                let edge = edge_index(carrier_index, split_index);
                if branches[slot].replace(edge).is_some() {
                    branches[slot] = None;
                }
            }
            if incident.len() != 4 || branches.iter().any(Option::is_none) {
                if let Some(vertex_sectors) = regularized_incident_ray_sectors(
                    incident,
                    topology,
                    &edge_offsets,
                    &self.data.policy,
                ) {
                    if let Some(sectors) = &vertex_sectors.sectors {
                        record_regularized_vertex_sectors(
                            &mut vertex_sector_links,
                            vertex,
                            sectors,
                        );
                    }
                    winding_sector_links.extend_from_slice(&vertex_sectors.winding_links);
                    transverse_winding_sector_links.extend(vertex_sectors.winding_links);
                }
                continue;
            }
            let [
                Some(first_in),
                Some(first_out),
                Some(second_in),
                Some(second_out),
            ] = branches
            else {
                continue;
            };
            if first_in == second_in && first_out == second_out {
                // The pair replay may report the common authored endpoint of
                // consecutive carriers. Both pair identities then resolve to
                // the same boundary walk; it is a join, not a second branch.
                continue;
            }
            let first = &self.data.carriers[contact.first_carrier];
            let second = &self.data.carriers[contact.second_carrier];
            if let Some(source_cross_is_positive) = contact.cross_is_positive {
                let cross_is_positive = source_cross_is_positive ^ first.reversed ^ second.reversed;
                if overlap_orientation_between_edges(first_out, second_in)? == Some(true) {
                    let sector_pairs = if cross_is_positive {
                        [
                            (left_face(first_in), left_face(second_out)),
                            (right_face(second_out), left_face(first_out)),
                            (right_face(first_out), right_face(first_in)),
                        ]
                    } else {
                        [
                            (left_face(first_in), left_face(first_out)),
                            (right_face(first_out), left_face(second_out)),
                            (right_face(second_out), right_face(first_in)),
                        ]
                    };
                    // Coincident endpoint sectors are exact local traversal
                    // links, but the same face can occupy several pinched
                    // sectors at the vertex. The overlap group's aggregate
                    // edge below is the sole global winding constraint.
                    record_regularized_vertex_sectors(
                        &mut vertex_sector_links,
                        vertex,
                        &sector_pairs,
                    );
                    continue;
                }
                if overlap_orientation_between_edges(first_in, second_out)? == Some(true) {
                    let sector_pairs = if cross_is_positive {
                        [
                            (right_face(first_out), right_face(second_in)),
                            (left_face(second_in), right_face(first_in)),
                            (left_face(first_in), left_face(first_out)),
                        ]
                    } else {
                        [
                            (right_face(first_out), right_face(first_in)),
                            (left_face(first_in), right_face(second_in)),
                            (left_face(second_in), left_face(first_out)),
                        ]
                    };
                    record_regularized_vertex_sectors(
                        &mut vertex_sector_links,
                        vertex,
                        &sector_pairs,
                    );
                    continue;
                }
                let sector_pairs = if cross_is_positive {
                    [
                        (left_face(first_out), right_face(second_out)),
                        (right_face(first_out), right_face(second_in)),
                        (left_face(first_in), left_face(second_out)),
                        (right_face(first_in), left_face(second_in)),
                    ]
                } else {
                    [
                        (left_face(first_out), left_face(second_in)),
                        (right_face(first_out), left_face(second_out)),
                        (left_face(first_in), right_face(second_in)),
                        (right_face(first_in), right_face(second_out)),
                    ]
                };
                record_regularized_vertex_sectors(&mut vertex_sector_links, vertex, &sector_pairs);
                winding_sector_links.extend(sector_pairs);
                transverse_winding_sector_links.extend(sector_pairs);
                continue;
            }
            let (Some(mut same_direction), Some(mut side)) = (
                contact.tangent_dot_is_positive,
                contact.second_side_of_first,
            ) else {
                continue;
            };
            same_direction ^= first.reversed ^ second.reversed;
            if first.reversed {
                side = match side {
                    LineSide::Left => LineSide::Right,
                    LineSide::Right => LineSide::Left,
                    LineSide::On => continue,
                };
            }
            let sector_pairs = match (same_direction, side) {
                (true, LineSide::Left) => [
                    (left_face(first_in), right_face(second_in)),
                    (right_face(first_out), right_face(first_in)),
                    (left_face(second_in), left_face(second_out)),
                    (right_face(second_out), left_face(first_out)),
                ],
                (true, LineSide::Right) => [
                    (left_face(first_in), left_face(first_out)),
                    (right_face(second_out), right_face(second_in)),
                    (left_face(second_in), right_face(first_in)),
                    (right_face(first_out), left_face(second_out)),
                ],
                (false, LineSide::Left) => [
                    (left_face(first_in), left_face(second_out)),
                    (right_face(first_out), right_face(first_in)),
                    (left_face(second_in), left_face(first_out)),
                    (right_face(second_out), right_face(second_in)),
                ],
                (false, LineSide::Right) => [
                    (left_face(first_in), left_face(first_out)),
                    (left_face(second_in), left_face(second_out)),
                    (right_face(second_out), right_face(first_in)),
                    (right_face(first_out), right_face(second_in)),
                ],
                (_, LineSide::On) => continue,
            };
            record_regularized_vertex_sectors(&mut vertex_sector_links, vertex, &sector_pairs);
            winding_sector_links.extend(sector_pairs);
        }

        let mut edge_overlapped = topology
            .split_fragments
            .iter()
            .map(|splits| vec![false; splits.len()])
            .collect::<Vec<_>>();
        let mut edge_owns_overlap = topology
            .split_fragments
            .iter()
            .map(|splits| vec![true; splits.len()])
            .collect::<Vec<_>>();
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_index, split) in splits.iter().enumerate() {
                let range = split.fragment.curve_region_parameter_range();
                for overlap in &topology.overlaps {
                    for (second, side_carrier_index, own_range, other_carrier_index) in [
                        (
                            false,
                            overlap.first_carrier_index,
                            &overlap.first_range,
                            overlap.second_carrier_index,
                        ),
                        (
                            true,
                            overlap.second_carrier_index,
                            &overlap.second_range,
                            overlap.first_carrier_index,
                        ),
                    ] {
                        if side_carrier_index != carrier_index {
                            continue;
                        }
                        let contains = match carrier_overlap_split_interval(overlap, second, splits)
                        {
                            Some(interval) => interval.contains(&split_index),
                            None => range_contains_fragment(
                                own_range,
                                range.start(),
                                range.end(),
                                &self.data.policy,
                            )?,
                        };
                        // The lower carrier owns a shared span; within one
                        // retraced carrier, its first overlap side owns it.
                        if contains {
                            edge_overlapped[carrier_index][split_index] = true;
                            if other_carrier_index < carrier_index
                                || (second && other_carrier_index == carrier_index)
                            {
                                edge_owns_overlap[carrier_index][split_index] = false;
                            }
                        }
                    }
                }
                if !edge_owns_overlap[carrier_index][split_index] {
                    actions[carrier_index][split_index] = Some(RegionFragmentAction::Discard);
                }
            }
        }

        let mut overlap_links = vec![Vec::<(usize, bool)>::new(); edge_count];
        for overlap in &topology.overlaps {
            let collect_edges = |carrier_index: usize,
                                 second: bool,
                                 overlap_range: &CurveParameterRange2|
             -> ExactCurveResult<Vec<usize>> {
                let mut edges = Vec::new();
                for (split_index, split) in
                    topology.split_fragments[carrier_index].iter().enumerate()
                {
                    let range = split.fragment.curve_region_parameter_range();
                    let contains = match carrier_overlap_split_interval(
                        overlap,
                        second,
                        &topology.split_fragments[carrier_index],
                    ) {
                        Some(interval) => interval.contains(&split_index),
                        None => range_contains_fragment(
                            overlap_range,
                            range.start(),
                            range.end(),
                            &self.data.policy,
                        )?,
                    };
                    if contains {
                        edges.push(edge_index(carrier_index, split_index));
                    }
                }
                Ok(edges)
            };
            let first_edges =
                collect_edges(overlap.first_carrier_index, false, &overlap.first_range)?;
            let second_edges =
                collect_edges(overlap.second_carrier_index, true, &overlap.second_range)?;
            let first_carrier = &self.data.carriers[overlap.first_carrier_index];
            let second_carrier = &self.data.carriers[overlap.second_carrier_index];
            let reversed = (overlap.orientation == CurveOverlapOrientation2::Reversed)
                ^ first_carrier.reversed
                ^ second_carrier.reversed;

            let mut pairs = Vec::new();
            if first_edges.len() == second_edges.len() {
                for (index, &first_edge) in first_edges.iter().enumerate() {
                    let second_index = if reversed {
                        second_edges.len() - 1 - index
                    } else {
                        index
                    };
                    pairs.push((first_edge, second_edges[second_index], reversed));
                }
            } else {
                let mut used = vec![false; second_edges.len()];
                for &first_edge in &first_edges {
                    let (first_carrier_index, first_split_index) = edge_sources[first_edge];
                    let first_split =
                        &topology.split_fragments[first_carrier_index][first_split_index];
                    let mut matched = None;
                    for (second_index, &second_edge) in second_edges.iter().enumerate() {
                        if used[second_index] {
                            continue;
                        }
                        let (second_carrier_index, second_split_index) = edge_sources[second_edge];
                        let second_split =
                            &topology.split_fragments[second_carrier_index][second_split_index];
                        let endpoint_match = if reversed {
                            first_split.start_topology_vertex == second_split.end_topology_vertex
                                && first_split.end_topology_vertex
                                    == second_split.start_topology_vertex
                        } else {
                            first_split.start_topology_vertex == second_split.start_topology_vertex
                                && first_split.end_topology_vertex
                                    == second_split.end_topology_vertex
                        };
                        if endpoint_match
                            && first_split.start_topology_vertex.is_some()
                            && first_split.end_topology_vertex.is_some()
                        {
                            if matched.is_some() {
                                matched = None;
                                break;
                            }
                            matched = Some((second_index, second_edge));
                        }
                    }
                    if let Some((second_index, second_edge)) = matched {
                        used[second_index] = true;
                        pairs.push((first_edge, second_edge, reversed));
                    }
                }
            }

            for (first_edge, second_edge, reversed) in pairs {
                overlap_links[first_edge].push((second_edge, reversed));
                overlap_links[second_edge].push((first_edge, reversed));
            }
        }

        // Source winding is local to an open face sector. A global component
        // can touch itself at a point while carrying different winding values
        // in those sectors, so retain one node per oriented edge side and use
        // explicit zero-jump links for the certified local adjacencies.
        let face_roots = topology
            .split_fragments
            .iter()
            .enumerate()
            .map(|(carrier_index, splits)| {
                splits
                    .iter()
                    .enumerate()
                    .map(|(split_index, _)| {
                        let edge = edge_index(carrier_index, split_index);
                        [left_face(edge), right_face(edge)]
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();

        let mut face_adjacency = vec![Vec::new(); edge_count.saturating_mul(2)];
        let mut transverse_face_adjacency = vec![Vec::new(); edge_count.saturating_mul(2)];
        let mut winding_jumps = Vec::new();
        let mut edge_overlap_grouped = vec![false; edge_count];
        let mut overlap_orientation = vec![None; edge_count];
        let mut invalid_face_roots = vec![false; edge_count.saturating_mul(2)];
        for edge in 0..edge_count {
            if overlap_links[edge].is_empty() || overlap_orientation[edge].is_some() {
                continue;
            }
            overlap_orientation[edge] = Some(false);
            let mut queue = std::collections::VecDeque::from([edge]);
            let mut members = Vec::new();
            while let Some(member) = queue.pop_front() {
                let orientation =
                    overlap_orientation[member].expect("queued overlap member has an orientation");
                members.push((member, orientation));
                edge_overlap_grouped[member] = true;
                for &(neighbor, reversed) in &overlap_links[member] {
                    let neighbor_orientation = orientation ^ reversed;
                    match overlap_orientation[neighbor] {
                        Some(existing) if existing != neighbor_orientation => {
                            return Err(self.invalid(
                                edge_sources[member].0,
                                CurveError::Topology(
                                    "coincident carrier orientations are inconsistent".into(),
                                ),
                            ));
                        }
                        Some(_) => {}
                        None => {
                            overlap_orientation[neighbor] = Some(neighbor_orientation);
                            queue.push_back(neighbor);
                        }
                    }
                }
            }
            let mut components = vec![0_i32; self.data.first.boundary_loops().len()];
            for &(member, reversed) in &members {
                let carrier_index = edge_sources[member].0;
                let loop_index = self.data.carriers[carrier_index].loop_index;
                let Some(component) = components.get_mut(loop_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "coincident carrier references a missing source loop".into(),
                        ),
                    ));
                };
                *component += if reversed { -1 } else { 1 };
            }
            let components = components
                .into_iter()
                .enumerate()
                .filter_map(|(loop_index, delta)| (delta != 0).then_some((loop_index, delta)))
                .collect::<Vec<_>>()
                .into_boxed_slice();
            if components.is_empty() {
                for &(member, _) in &members {
                    let (carrier_index, split_index) = edge_sources[member];
                    actions[carrier_index][split_index] = Some(RegionFragmentAction::Discard);
                }
                // Crossing a cancelling coincident group changes no loop's
                // winding, so its two sides carry equal winding vectors. Link
                // them exactly; otherwise every shared internal edge splits the
                // face equations and asks for another geometric seed.
                let [left, right] =
                    face_roots[edge_sources[members[0].0].0][edge_sources[members[0].0].1];
                if left != right {
                    let jump = winding_jumps.len();
                    winding_jumps.push(RegularizedWindingJump::Zero);
                    face_adjacency[right].push(RegularizedFaceAdjacency {
                        face: left,
                        jump,
                        direction: 0,
                    });
                    face_adjacency[left].push(RegularizedFaceAdjacency {
                        face: right,
                        jump,
                        direction: 0,
                    });
                }
                continue;
            }
            let reference = members[0].0;
            let [left, right] = face_roots[edge_sources[reference].0][edge_sources[reference].1];
            if left == right {
                // A contact-sector union may conservatively collapse both
                // sides of a same-image span at a multi-branch endpoint. A
                // nonzero aggregate winding jump cannot be a self-loop. Mark
                // only this connected face component unusable; disjoint exact
                // sector equations remain authoritative.
                invalid_face_roots[left] = true;
                continue;
            }
            let jump = winding_jumps.len();
            winding_jumps.push(RegularizedWindingJump::Aggregate(components));
            face_adjacency[right].push(RegularizedFaceAdjacency {
                face: left,
                jump,
                direction: 1,
            });
            face_adjacency[left].push(RegularizedFaceAdjacency {
                face: right,
                jump,
                direction: -1,
            });
        }
        for (carrier_index, roots) in face_roots.iter().enumerate() {
            let loop_index = self.data.carriers[carrier_index].loop_index;
            for (split_index, &[left, right]) in roots.iter().enumerate() {
                // Coincident carriers require the aggregate jump of their
                // overlap group. Until that group is formed below, leaving
                // this edge disconnected is exact and merely asks for a seed.
                let edge = edge_index(carrier_index, split_index);
                if edge_overlapped[carrier_index][split_index] {
                    continue;
                }
                if left == right {
                    // Contact-sector unions can conservatively collapse the
                    // two local sides of a neighboring non-overlap edge as
                    // well as an overlap edge. Quarantine that component and
                    // leave its per-fragment geometric classifier authoritative.
                    invalid_face_roots[left] = true;
                    continue;
                }
                debug_assert!(!edge_overlap_grouped[edge]);
                let jump = winding_jumps.len();
                winding_jumps.push(RegularizedWindingJump::Single(loop_index));
                face_adjacency[right].push(RegularizedFaceAdjacency {
                    face: left,
                    jump,
                    direction: 1,
                });
                face_adjacency[left].push(RegularizedFaceAdjacency {
                    face: right,
                    jump,
                    direction: -1,
                });
                transverse_face_adjacency[right].push(RegularizedFaceAdjacency {
                    face: left,
                    jump,
                    direction: 1,
                });
                transverse_face_adjacency[left].push(RegularizedFaceAdjacency {
                    face: right,
                    jump,
                    direction: -1,
                });
            }
        }
        if !winding_sector_links.is_empty() || !transverse_winding_sector_links.is_empty() {
            let zero = winding_jumps.len();
            winding_jumps.push(RegularizedWindingJump::Zero);
            for (first, second) in winding_sector_links {
                face_adjacency[first].push(RegularizedFaceAdjacency {
                    face: second,
                    jump: zero,
                    direction: 0,
                });
                face_adjacency[second].push(RegularizedFaceAdjacency {
                    face: first,
                    jump: zero,
                    direction: 0,
                });
            }
            for (first, second) in transverse_winding_sector_links {
                transverse_face_adjacency[first].push(RegularizedFaceAdjacency {
                    face: second,
                    jump: zero,
                    direction: 0,
                });
                transverse_face_adjacency[second].push(RegularizedFaceAdjacency {
                    face: first,
                    jump: zero,
                    direction: 0,
                });
            }
        }
        // A root can be found invalid after another edge has already added an
        // equation to it. Remove both directions of every such equation while
        // preserving independent exact face components.
        for face in 0..face_adjacency.len() {
            if invalid_face_roots[face] {
                face_adjacency[face].clear();
            } else {
                face_adjacency[face]
                    .retain(|edge| !invalid_face_roots.get(edge.face).copied().unwrap_or(true));
            }
        }
        // Keep direct overlap and geometric classifications separate from
        // actions inferred through sparse face equations. If later exact
        // evidence disproves an equation component, inference is rebuilt from
        // these authoritative actions instead of retaining stale decisions.
        let mut authoritative_actions = actions.clone();
        let mut face_windings = vec![None; edge_count.saturating_mul(2)];
        let mut transverse_face_windings = vec![None; edge_count.saturating_mul(2)];
        let mut face_equation_faces_valid = invalid_face_roots
            .iter()
            .map(|invalid| !invalid)
            .collect::<Vec<_>>();
        let mut transverse_face_equation_faces_valid = vec![true; edge_count.saturating_mul(2)];
        let action_from_windings = |carrier_index: usize,
                                    split_index: usize,
                                    windings: &[Option<LoopWindings>],
                                    reject_invalid_roots: bool|
         -> ExactCurveResult<Option<RegionFragmentAction>> {
            let [left_face, right_face] = face_roots[carrier_index][split_index];
            if reject_invalid_roots
                && (invalid_face_roots[left_face] || invalid_face_roots[right_face])
            {
                return Ok(None);
            }
            let (Some(left), Some(right)) =
                (windings[left_face].as_ref(), windings[right_face].as_ref())
            else {
                return Ok(None);
            };
            let left = self
                .location_from_loop_windings(self.data.first, left)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            let right = self
                .location_from_loop_windings(self.data.first, right)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            Ok(Some(action_from_result_sides(
                left == RegionPointLocation::Inside,
                right == RegionPointLocation::Inside,
            )))
        };

        // Exact affine fragments are the cheapest authoritative local face
        // seeds. Evaluate them first and propagate their winding equations.
        // Skip the global exterior ray only when those exact seeds already
        // cover every edge; otherwise the historical probe remains the
        // complete fallback rather than merely the no-line fallback.
        for (carrier_index, splits) in topology.split_fragments.iter().enumerate() {
            for (split_index, split) in splits.iter().enumerate() {
                if actions[carrier_index][split_index].is_some()
                    || edge_overlapped[carrier_index][split_index]
                    || !match &split.fragment {
                        BezierSplitFragment2::AlgebraicChord(chord) => chord.exact_line().is_some(),
                        BezierSplitFragment2::Materialized { .. } => {
                            split_fragment_is_affine_line(&split.fragment)
                        }
                        BezierSplitFragment2::RetainedBezier { .. }
                        | BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::AlgebraicCuspSemicircle(_)
                        | BezierSplitFragment2::SelectedFiber(_) => false,
                    }
                {
                    continue;
                }
                // Earlier seeds may already have propagated both side windings
                // to this fragment. Leave its action to the derivation passes
                // below, which rebuild derived actions if a later seed
                // disables a contradictory component; a geometric ray probe
                // here would only rediscover the same face action.
                if action_from_windings(
                    carrier_index,
                    split_index,
                    &transverse_face_windings,
                    false,
                )?
                .is_some()
                    || action_from_windings(carrier_index, split_index, &face_windings, true)?
                        .is_some()
                {
                    continue;
                }
                let decision = self.regularized_fragment_geometric_decision(
                    carrier_index,
                    &split.fragment,
                    true,
                );
                match decision {
                    Ok(decision) => {
                        actions[carrier_index][split_index] = Some(decision.action);
                        authoritative_actions[carrier_index][split_index] = Some(decision.action);
                        let [left, right] = decision.side_windings;
                        let roots = face_roots[carrier_index][split_index];
                        seed_regularized_face_windings(
                            &mut transverse_face_windings,
                            &mut transverse_face_equation_faces_valid,
                            &transverse_face_adjacency,
                            &winding_jumps,
                            [(roots[0], left.clone()), (roots[1], right.clone())],
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                        seed_regularized_face_windings(
                            &mut face_windings,
                            &mut face_equation_faces_valid,
                            &face_adjacency,
                            &winding_jumps,
                            [(roots[0], left), (roots[1], right)]
                                .into_iter()
                                .filter(|(face, _)| !invalid_face_roots[*face]),
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    }
                    Err(error @ ExactCurveError::Blocked(_)) => {
                        blockers[carrier_index][split_index] = Some(error);
                    }
                    Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
                }
            }
        }
        let exact_local_seeds_cover_arrangement =
            topology
                .split_fragments
                .iter()
                .enumerate()
                .all(|(carrier_index, splits)| {
                    splits.iter().enumerate().all(|(split_index, _)| {
                        actions[carrier_index][split_index].is_some() || {
                            let [left, right] = face_roots[carrier_index][split_index];
                            !invalid_face_roots[left]
                                && !invalid_face_roots[right]
                                && face_windings[left].is_some()
                                && face_windings[right].is_some()
                        }
                    })
                });
        if !exact_local_seeds_cover_arrangement {
            let mut retained_bounds = Vec::with_capacity(self.data.carriers.len());
            for carrier in &self.data.carriers {
                let cached = carrier.bounds.get_or_init(|| {
                    carrier
                        .geometry
                        .certified_outer_bounds(&carrier.range(), 0, &self.data.policy)
                });
                let bounds = match cached {
                    Classification::Decided(bounds) => Some(bounds.clone()),
                    Classification::Uncertain(_) => None,
                };
                retained_bounds.push(bounds);
            }
            if retained_bounds.len() == self.data.carriers.len() {
                let approximate_magnitude = retained_bounds
                    .iter()
                    .flatten()
                    .flat_map(|bounds| {
                        [
                            bounds.min().x(),
                            bounds.min().y(),
                            bounds.max().x(),
                            bounds.max().y(),
                        ]
                    })
                    .filter_map(|coordinate| coordinate.to_f64_lossy())
                    .map(f64::abs)
                    .fold(1.0_f64, f64::max);
                let initial_magnitude = if approximate_magnitude.is_finite()
                    && approximate_magnitude < (i64::MAX / 4) as f64
                {
                    approximate_magnitude.ceil() as i64 + 2
                } else {
                    1
                };
                let mut magnitude = Real::from(initial_magnitude);
                let mut exterior_coordinates = None;
                for _ in 0..64 {
                    let negative = -magnitude.clone();
                    let outside_axis = |axis: Axis2,
                                        value: &Real,
                                        negative_side: bool|
                     -> ExactCurveResult<bool> {
                        for (carrier_index, (carrier, bounds)) in
                            self.data.carriers.iter().zip(&retained_bounds).enumerate()
                        {
                            let outside = if let Some(bounds) = bounds {
                                let boundary = match (axis, negative_side) {
                                    (Axis2::X, true) => bounds.min().x(),
                                    (Axis2::X, false) => bounds.max().x(),
                                    (Axis2::Y, true) => bounds.min().y(),
                                    (Axis2::Y, false) => bounds.max().y(),
                                };
                                compare_reals(value, boundary, &self.data.policy)
                                    == Some(if negative_side {
                                        Ordering::Less
                                    } else {
                                        Ordering::Greater
                                    })
                            } else if let CurveSupport2::Line(chord) = &carrier.geometry {
                                let expected = if negative_side {
                                    Ordering::Greater
                                } else {
                                    Ordering::Less
                                };
                                match chord
                                    .endpoints_strict_axis_order_to_real(
                                        axis,
                                        value,
                                        expected,
                                        &self.data.policy,
                                    )
                                    .map_err(|cause| self.invalid(carrier_index, cause))?
                                {
                                    Classification::Decided(outside) => outside,
                                    Classification::Uncertain(_) => false,
                                }
                            } else {
                                false
                            };
                            if !outside {
                                return Ok(false);
                            }
                        }
                        Ok(true)
                    };
                    let left = outside_axis(Axis2::X, &negative, true)?;
                    let right = outside_axis(Axis2::X, &magnitude, false)?;
                    let below = outside_axis(Axis2::Y, &negative, true)?;
                    let above = outside_axis(Axis2::Y, &magnitude, false)?;
                    if left || right || below || above {
                        exterior_coordinates =
                            Some((negative, magnitude.clone(), [left, right, below, above]));
                        if left && right && below && above {
                            break;
                        }
                    }
                    magnitude *= Real::from(2_u8);
                }
                let half = (Real::one() / Real::from(2_u8))
                    .map_err(|cause| self.invalid(0, cause.into()))?;
                let targets = topology
                    .split_fragments
                    .iter()
                    .flat_map(|splits| splits.iter())
                    .filter_map(|split| match &split.fragment {
                        BezierSplitFragment2::AlgebraicChord(chord) => {
                            chord.exact_line().map(|line| line.point_at(half.clone()))
                        }
                        BezierSplitFragment2::Materialized { .. }
                        | BezierSplitFragment2::RetainedBezier { .. }
                        | BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::SelectedFiber(_)
                        | BezierSplitFragment2::AlgebraicCuspSemicircle(_) => None,
                    })
                    .collect::<Vec<_>>();
                'probe: for target in targets {
                    let Some((negative, positive, certified)) = &exterior_coordinates else {
                        break;
                    };
                    let outside_points = [
                        (
                            certified[0]
                                .then(|| crate::Point2::new(negative.clone(), target.y().clone())),
                            Axis2::X,
                            true,
                        ),
                        (
                            certified[1]
                                .then(|| crate::Point2::new(positive.clone(), target.y().clone())),
                            Axis2::X,
                            false,
                        ),
                        (
                            certified[2]
                                .then(|| crate::Point2::new(target.x().clone(), negative.clone())),
                            Axis2::Y,
                            true,
                        ),
                        (
                            certified[3]
                                .then(|| crate::Point2::new(target.x().clone(), positive.clone())),
                            Axis2::Y,
                            false,
                        ),
                    ];
                    for (outside, probe_axis, coordinate_increases) in outside_points {
                        let Some(outside) = outside else {
                            continue;
                        };
                        let probe = match crate::BezierAlgebraicChord2::try_new(
                            CurvePoint2::from(outside),
                            CurvePoint2::from(target.clone()),
                            &self.data.policy,
                        )
                        .map_err(|cause| self.invalid(0, cause))?
                        {
                            Classification::Decided(probe) => probe,
                            Classification::Uncertain(_) => continue,
                        };
                        let evidence = match self.intersect_algebraic_probe_boundary(probe, None) {
                            Ok(evidence) if evidence.overlaps().is_empty() => evidence,
                            Ok(_) | Err(_) => continue,
                        };
                        let mut contacts = evidence
                            .contacts()
                            .iter()
                            .filter(|contact| contact.is_certified_transverse())
                            .collect::<Vec<_>>();
                        let mut ordered = true;
                        for index in 1..contacts.len() {
                            let mut cursor = index;
                            while cursor > 0 {
                                let order = match contacts[cursor]
                                    .first_parameter()
                                    .cmp_by_refinement(
                                        contacts[cursor - 1].first_parameter(),
                                        &self.data.policy,
                                    )
                                    .map_err(|cause| self.invalid(0, cause))?
                                {
                                    Classification::Decided(order) => order,
                                    Classification::Uncertain(_) => {
                                        ordered = false;
                                        break;
                                    }
                                };
                                if order != Ordering::Less {
                                    break;
                                }
                                contacts.swap(cursor, cursor - 1);
                                cursor -= 1;
                            }
                            if !ordered {
                                break;
                            }
                        }
                        if !ordered {
                            continue;
                        }
                        for contact in contacts {
                            let Some(probe_parameter) =
                                contact.first_parameter().as_algebraic_chord()
                            else {
                                continue;
                            };
                            let contact_point = probe_parameter.point();
                            let mut blockers_are_after = true;
                            for blocker in evidence.blockers() {
                                let Some(carrier_index) =
                                    blocker.second().carrier_index().checked_sub(1)
                                else {
                                    blockers_are_after = false;
                                    break;
                                };
                                let Some(carrier) = self.data.carriers.get(carrier_index) else {
                                    blockers_are_after = false;
                                    break;
                                };
                                let expected_endpoint_order = if coordinate_increases {
                                    Ordering::Greater
                                } else {
                                    Ordering::Less
                                };
                                let after = if let CurveSupport2::Line(chord) = &carrier.geometry {
                                    let mut after = true;
                                    for endpoint in [chord.start(), chord.end()] {
                                        match crate::BezierAlgebraicChord2::point_axis_order(
                                            endpoint,
                                            contact_point,
                                            probe_axis,
                                            &self.data.policy,
                                        )
                                        .map_err(|cause| self.invalid(carrier_index, cause))?
                                        {
                                            Classification::Decided(order)
                                                if order == expected_endpoint_order => {}
                                            Classification::Decided(_)
                                            | Classification::Uncertain(_) => {
                                                after = false;
                                                break;
                                            }
                                        }
                                    }
                                    after
                                } else if let Some(Some(bounds)) =
                                    retained_bounds.get(carrier_index)
                                {
                                    let boundary = match (probe_axis, coordinate_increases) {
                                        (Axis2::X, true) => bounds.min().x(),
                                        (Axis2::X, false) => bounds.max().x(),
                                        (Axis2::Y, true) => bounds.min().y(),
                                        (Axis2::Y, false) => bounds.max().y(),
                                    };
                                    let expected_contact_order = if coordinate_increases {
                                        Ordering::Less
                                    } else {
                                        Ordering::Greater
                                    };
                                    crate::BezierAlgebraicChord2::point_axis_order_to_real(
                                        contact_point,
                                        probe_axis,
                                        boundary,
                                        &self.data.policy,
                                    )
                                    .map_err(|cause| self.invalid(carrier_index, cause))?
                                        == Classification::Decided(expected_contact_order)
                                } else {
                                    false
                                };
                                if !after {
                                    blockers_are_after = false;
                                    break;
                                }
                            }
                            if !blockers_are_after {
                                continue;
                            }
                            let Some(carrier_index) =
                                self.data.carriers.iter().position(|carrier| {
                                    carrier.loop_index == contact.second().loop_index()
                                        && carrier.fragment_index
                                            == contact.second().fragment_index()
                                })
                            else {
                                continue;
                            };
                            let parameter = contact.second_parameter();
                            let mut containing_split = None;
                            for (split_index, split) in
                                topology.split_fragments[carrier_index].iter().enumerate()
                            {
                                let range = split.fragment.curve_region_parameter_range();
                                let start = parameter
                                    .cmp_by_refinement(range.start(), &self.data.policy)
                                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                                let end = parameter
                                    .cmp_by_refinement(range.end(), &self.data.policy)
                                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                                if start == Classification::Decided(Ordering::Greater)
                                    && end == Classification::Decided(Ordering::Less)
                                {
                                    containing_split = Some(split_index);
                                    break;
                                }
                            }
                            let Some(split_index) = containing_split else {
                                continue;
                            };
                            let Some(mut cross_is_positive) =
                                contact.evidence.tangent_cross_is_positive()
                            else {
                                continue;
                            };
                            cross_is_positive ^= self.data.carriers[carrier_index].reversed;
                            let roots = face_roots[carrier_index][split_index];
                            let exterior_face = if cross_is_positive {
                                roots[0]
                            } else {
                                roots[1]
                            };
                            if invalid_face_roots[exterior_face] {
                                continue;
                            }
                            seed_regularized_face_windings(
                                &mut face_windings,
                                &mut face_equation_faces_valid,
                                &face_adjacency,
                                &winding_jumps,
                                [(
                                    exterior_face,
                                    vec![0; self.data.first.boundary_loops().len()],
                                )],
                            )
                            .map_err(|cause| self.invalid(carrier_index, cause))?;
                            break 'probe;
                        }
                    }
                }
            }
        }

        let update_actions_from_faces = |actions: &mut Vec<Vec<Option<RegionFragmentAction>>>,
                                         face_windings: &[Option<LoopWindings>],
                                         transverse_face_windings: &[Option<LoopWindings>],
                                         _blockers: &[Vec<Option<ExactCurveError>>]|
         -> ExactCurveResult<()> {
            for (derived_carrier_index, roots) in face_roots.iter().enumerate() {
                for derived_split_index in 0..roots.len() {
                    if actions[derived_carrier_index][derived_split_index].is_some() {
                        continue;
                    }
                    let derived_edge = edge_index(derived_carrier_index, derived_split_index);
                    if edge_overlapped[derived_carrier_index][derived_split_index]
                        && (!edge_overlap_grouped[derived_edge]
                            || !edge_owns_overlap[derived_carrier_index][derived_split_index])
                    {
                        continue;
                    }
                    let derived = action_from_windings(
                        derived_carrier_index,
                        derived_split_index,
                        transverse_face_windings,
                        false,
                    )?;
                    let Some(derived) = derived.or(action_from_windings(
                        derived_carrier_index,
                        derived_split_index,
                        face_windings,
                        true,
                    )?) else {
                        continue;
                    };
                    match actions[derived_carrier_index][derived_split_index] {
                        Some(existing) if existing != derived => {
                            return Err(self.invalid(
                                derived_carrier_index,
                                CurveError::Topology(
                                    "regularized face winding disagrees with a side classification"
                                        .into(),
                                ),
                            ));
                        }
                        Some(_) => {}
                        None => {
                            actions[derived_carrier_index][derived_split_index] = Some(derived);
                        }
                    }
                }
            }
            Ok(())
        };
        let propagate_authored_actions = |actions: &mut Vec<Vec<Option<RegionFragmentAction>>>,
                                          blockers: &[Vec<Option<ExactCurveError>>]|
         -> ExactCurveResult<()> {
            let mut changed = true;
            while changed {
                changed = false;
                for (vertex, incident) in incidents.iter().enumerate() {
                    if incident.len() != 2
                        || topology
                            .transverse_vertices
                            .get(vertex)
                            .copied()
                            .unwrap_or(false)
                    {
                        continue;
                    }
                    let (incoming, outgoing) = match (incident[0], incident[1]) {
                        ((carrier, split, false), (next_carrier, next_split, true)) => {
                            ((carrier, split), (next_carrier, next_split))
                        }
                        ((next_carrier, next_split, true), (carrier, split, false)) => {
                            ((carrier, split), (next_carrier, next_split))
                        }
                        _ => continue,
                    };
                    if !authored_successor(incoming, outgoing) {
                        continue;
                    }
                    // A coincident edge that lost deterministic overlap
                    // ownership is discarded as a duplicate image.  That
                    // ownership action belongs only to the overlap cell;
                    // it cannot propagate through the authored endpoint
                    // into an adjacent unique edge.
                    if edge_overlapped[incoming.0][incoming.1]
                        && !edge_owns_overlap[incoming.0][incoming.1]
                        || edge_overlapped[outgoing.0][outgoing.1]
                            && !edge_owns_overlap[outgoing.0][outgoing.1]
                    {
                        continue;
                    }
                    let first = actions[incoming.0][incoming.1];
                    let second = actions[outgoing.0][outgoing.1];
                    match (first, second) {
                        (Some(first), Some(second)) if first != second => {
                            return Err(self.invalid(
                                incoming.0,
                                CurveError::Topology(format!(
                                    "a degree-two authored continuation changed regularized faces at vertex {vertex}: incoming {incoming:?} is {first:?}, outgoing {outgoing:?} is {second:?}",
                                )),
                            ));
                        }
                        (Some(action), None) if blockers[outgoing.0][outgoing.1].is_some() => {
                            actions[outgoing.0][outgoing.1] = Some(action);
                            changed = true;
                        }
                        (None, Some(action)) if blockers[incoming.0][incoming.1].is_some() => {
                            actions[incoming.0][incoming.1] = Some(action);
                            changed = true;
                        }
                        (Some(_), Some(_)) | (Some(_), None) | (None, Some(_)) | (None, None) => {}
                    }
                }
            }
            Ok(())
        };

        update_actions_from_faces(
            &mut actions,
            &face_windings,
            &transverse_face_windings,
            &blockers,
        )?;
        propagate_authored_actions(&mut actions, &blockers)?;

        let mut work = topology
            .split_fragments
            .iter()
            .enumerate()
            .flat_map(|(carrier_index, splits)| {
                splits.iter().enumerate().map(move |(split_index, split)| {
                    let rank = match &split.fragment {
                        BezierSplitFragment2::AlgebraicChord(chord)
                            if chord.exact_line().is_some() =>
                        {
                            0_u8
                        }
                        BezierSplitFragment2::Materialized { .. }
                            if split_fragment_is_affine_line(&split.fragment) =>
                        {
                            0
                        }
                        BezierSplitFragment2::AlgebraicChord(_) => 1,
                        BezierSplitFragment2::AlgebraicCuspSemicircle(_) => 2,
                        BezierSplitFragment2::Materialized { .. }
                        | BezierSplitFragment2::RetainedBezier { .. } => 3,
                        BezierSplitFragment2::AnalyticParallel(_)
                        | BezierSplitFragment2::SelectedFiber(_) => 4,
                    };
                    (rank, carrier_index, split_index)
                })
            })
            .collect::<Vec<_>>();
        work.sort_by_key(|&(rank, _, _)| rank);
        loop {
            let mut equation_component_changed = false;
            for &(_, carrier_index, split_index) in &work {
                if actions[carrier_index][split_index].is_some() {
                    continue;
                }
                // Each successful seed propagates exact winding values before
                // the next work item. Consume those values immediately: a
                // second geometric probe can introduce an unrelated algebraic
                // field merely to rediscover an already certified face action.
                // Coincident cells still obey their chosen overlap ownership.
                let edge = edge_index(carrier_index, split_index);
                if !edge_overlapped[carrier_index][split_index]
                    || edge_overlap_grouped[edge] && edge_owns_overlap[carrier_index][split_index]
                {
                    let derived = match action_from_windings(
                        carrier_index,
                        split_index,
                        &transverse_face_windings,
                        false,
                    )? {
                        Some(action) => Some(action),
                        None => {
                            action_from_windings(carrier_index, split_index, &face_windings, true)?
                        }
                    };
                    if let Some(action) = derived {
                        actions[carrier_index][split_index] = Some(action);
                        continue;
                    }
                }
                let split = &topology.split_fragments[carrier_index][split_index];
                let decision = self.regularized_fragment_geometric_decision(
                    carrier_index,
                    &split.fragment,
                    !edge_overlapped[carrier_index][split_index],
                );
                match decision {
                    Ok(decision) => {
                        actions[carrier_index][split_index] = Some(decision.action);
                        authoritative_actions[carrier_index][split_index] = Some(decision.action);
                        blockers[carrier_index][split_index] = None;
                        let [left, right] = decision.side_windings;
                        let roots = face_roots[carrier_index][split_index];
                        let transverse_changed = seed_regularized_face_windings(
                            &mut transverse_face_windings,
                            &mut transverse_face_equation_faces_valid,
                            &transverse_face_adjacency,
                            &winding_jumps,
                            [(roots[0], left.clone()), (roots[1], right.clone())],
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                        let face_changed = seed_regularized_face_windings(
                            &mut face_windings,
                            &mut face_equation_faces_valid,
                            &face_adjacency,
                            &winding_jumps,
                            [(roots[0], left), (roots[1], right)]
                                .into_iter()
                                .filter(|(face, _)| !invalid_face_roots[*face]),
                        )
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                        if transverse_changed || face_changed {
                            // A declined equation component may have supplied
                            // earlier actions. Rebuild every derived decision
                            // from the surviving equations and direct exact
                            // classifications before continuing the worklist.
                            actions.clone_from(&authoritative_actions);
                            update_actions_from_faces(
                                &mut actions,
                                &face_windings,
                                &transverse_face_windings,
                                &blockers,
                            )?;
                            propagate_authored_actions(&mut actions, &blockers)?;
                            equation_component_changed = true;
                            break;
                        }
                    }
                    Err(error @ ExactCurveError::Blocked(_)) => {
                        blockers[carrier_index][split_index] = Some(error);
                    }
                    Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
                }
            }
            if !equation_component_changed {
                break;
            }
        }

        propagate_authored_actions(&mut actions, &blockers)?;
        update_actions_from_faces(
            &mut actions,
            &face_windings,
            &transverse_face_windings,
            &blockers,
        )?;
        propagate_authored_actions(&mut actions, &blockers)?;

        let mut decided = Vec::with_capacity(actions.len());
        for (carrier_index, carrier_actions) in actions.into_iter().enumerate() {
            let mut carrier_decided = Vec::with_capacity(carrier_actions.len());
            for (split_index, action) in carrier_actions.into_iter().enumerate() {
                let Some(action) = action else {
                    let error = blockers[carrier_index][split_index]
                        .clone()
                        .unwrap_or_else(|| {
                            self.blocked(carrier_index, UncertaintyReason::Predicate)
                        });
                    return Err(error);
                };
                carrier_decided.push(action);
            }
            decided.push(carrier_decided);
        }
        let mut successor_edge_ids = vec![NO_REGULARIZED_EDGE; edge_count];
        let oriented_sector_edge = |vertex: usize, sector: usize| {
            let edge = sector / 2;
            let &(carrier_index, split_index) = edge_sources.get(edge)?;
            let action = decided[carrier_index][split_index];
            let filled_sector = match action {
                RegionFragmentAction::Keep => left_face(edge),
                RegionFragmentAction::KeepReversed => right_face(edge),
                RegionFragmentAction::Discard => return None,
            };
            if sector != filled_sector {
                return None;
            }
            let split = &topology.split_fragments[carrier_index][split_index];
            let (start, end) = match action {
                RegionFragmentAction::Keep => {
                    (split.start_topology_vertex, split.end_topology_vertex)
                }
                RegionFragmentAction::KeepReversed => {
                    (split.end_topology_vertex, split.start_topology_vertex)
                }
                RegionFragmentAction::Discard => unreachable!(),
            };
            if start == Some(vertex) {
                Some((edge, true))
            } else if end == Some(vertex) {
                Some((edge, false))
            } else {
                None
            }
        };
        for &(vertex, first_sector, second_sector) in &vertex_sector_links {
            let (Some(first), Some(second)) = (
                oriented_sector_edge(vertex, first_sector),
                oriented_sector_edge(vertex, second_sector),
            ) else {
                continue;
            };
            let (incoming, outgoing) = match (first, second) {
                ((incoming, false), (outgoing, true)) | ((outgoing, true), (incoming, false)) => {
                    (incoming, outgoing)
                }
                ((_, true), (_, true)) | ((_, false), (_, false)) => continue,
            };
            match successor_edge_ids[incoming] {
                NO_REGULARIZED_EDGE => {
                    successor_edge_ids[incoming] = outgoing;
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "regularization-successor",
                        "face-sector",
                    );
                }
                AMBIGUOUS_REGULARIZED_EDGE => {}
                existing if existing == outgoing => {}
                _ => successor_edge_ids[incoming] = AMBIGUOUS_REGULARIZED_EDGE,
            }
        }
        Ok(RegularizedFragmentSelection {
            actions: decided,
            successor_edge_ids,
        })
    }

    pub(super) fn certified_simple_single_loop_filled_side(
        &self,
        topology: &CurveRegionSplitTopology,
    ) -> Option<bool> {
        if self.data.first.boundary_loops().len() != 1
            || self.data.carriers.is_empty()
            || topology.split_fragments.len() != self.data.carriers.len()
            || !topology.overlaps.is_empty()
            || self.data.carriers.iter().any(|carrier| {
                carrier.operand != CurveRegionBooleanOperand2::First
                    || carrier.loop_index != 0
                    || !carrier_has_certified_injective_image(carrier, &self.data.policy)
            })
        {
            return None;
        }
        let Ok(Classification::Decided(roles)) = self.data.first.loop_roles_raw(&self.data.policy)
        else {
            return None;
        };
        if roles.as_slice() != [CurveRegionLoopRole::Material] {
            return None;
        }
        let Ok(Classification::Decided(filled_sides)) =
            self.data.first.filled_side_is_left_raw(&self.data.policy)
        else {
            return None;
        };
        let [filled_side_is_left] = filled_sides else {
            return None;
        };

        // Complete pair replay and splitting have already run. A simple
        // authored loop therefore has one unsplit fragment per injective
        // carrier, each end joined only to the next authored start. Requiring
        // every authored start vertex to be distinct excludes nonadjacent
        // endpoint aliases and pinched walks without allocating a side sample
        // or materializing an algebraic carrier coordinate.
        for (index, splits) in topology.split_fragments.iter().enumerate() {
            let [split] = splits.as_slice() else {
                return None;
            };
            let start = split.start_topology_vertex?;
            let end = split.end_topology_vertex?;
            if start == end {
                return None;
            }
            let next = topology
                .split_fragments
                .get((index + 1) % topology.split_fragments.len())?;
            let [next] = next.as_slice() else {
                return None;
            };
            if next.start_topology_vertex != Some(end) {
                return None;
            }
            for previous in &topology.split_fragments[..index] {
                let [previous] = previous.as_slice() else {
                    return None;
                };
                if previous.start_topology_vertex == Some(start) {
                    return None;
                }
            }
        }
        Some(*filled_side_is_left)
    }

    pub(super) fn regularized_fragment_geometric_decision(
        &self,
        carrier_index: usize,
        fragment: &BezierSplitFragment2,
        has_single_boundary_jump: bool,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        // A native side ray may fail an equality that the retained boundary
        // probe can replay from construction evidence. Exhaust both exact
        // routes before a local comparison may consume approximation.
        if self.data.policy.permits_approximate_512() {
            match self.data.policy.strict_predicate_pass(|| {
                self.regularized_fragment_geometric_decision(
                    carrier_index,
                    fragment,
                    has_single_boundary_jump,
                )
            }) {
                Err(ExactCurveError::Blocked(_)) => {}
                result => return result,
            }
        }
        if let BezierSplitFragment2::AlgebraicCuspSemicircle(fragment) = fragment {
            return self.regularized_algebraic_cusp_fragment_decision(carrier_index, fragment);
        }
        if let BezierSplitFragment2::AlgebraicChord(chord) = fragment {
            return self.regularized_algebraic_chord_fragment_decision(
                carrier_index,
                chord,
                has_single_boundary_jump,
            );
        }
        let carrier = &self.data.carriers[carrier_index];
        let max_representatives = match &carrier.geometry {
            CurveSupport2::Bezier(
                BezierSubcurve2::Quadratic(_) | BezierSubcurve2::RationalQuadratic(_),
            ) => 4,
            CurveSupport2::Bezier(BezierSubcurve2::Cubic(_)) => 6,
            CurveSupport2::Bezier(BezierSubcurve2::Rational(curve)) => {
                curve.degree().saturating_mul(2).max(2)
            }
            CurveSupport2::Parallel(_) => 4,
            CurveSupport2::Line(_) => 2,
            CurveSupport2::Circle(_) => 4,
        };
        let fragment_parameter_range = fragment.curve_region_parameter_range();
        let (start, end) = (
            fragment_parameter_range.start(),
            fragment_parameter_range.end(),
        );
        let mut upper = end.clone();
        let mut last_reason = UncertaintyReason::Boundary;
        // Retained circular-conic provenance is a construction certificate for
        // a proper rational parametrization of a nondegenerate minor arc.
        let retained_regular_circle = matches!(
            &carrier.geometry,
            CurveSupport2::Bezier(curve) if retained_circular_support(curve).is_some()
        );
        // General rational conversion is only a fallback certificate source.
        // Avoid rebuilding a circular quadratic whose native provenance has
        // already supplied everything this classifier needs.
        let rational_geometry = if retained_regular_circle {
            None
        } else {
            match &carrier.geometry {
                CurveSupport2::Bezier(curve) => RationalBezier2::try_from_subcurve(curve).ok(),
                CurveSupport2::Parallel(_) | CurveSupport2::Line(_) | CurveSupport2::Circle(_) => {
                    None
                }
            }
        };
        let isolator_touches =
            |parameter: &BezierParameter2, boundary: &Real, use_interval_start: bool| {
                match parameter.known_interval_with_policy(&self.data.policy) {
                    Ok(Classification::Decided(interval)) => {
                        compare_reals(
                            if use_interval_start {
                                interval.start()
                            } else {
                                interval.end()
                            },
                            boundary,
                            &self.data.policy,
                        ) == Some(Ordering::Equal)
                    }
                    Ok(Classification::Uncertain(_)) | Err(_) => false,
                }
            };
        // An endpoint witness is only needed when the adjacent algebraic
        // isolator still touches that endpoint. Otherwise the represented
        // interior gap is cheaper and avoids an unnecessary boundary ray.
        let source_endpoint_witness = if retained_regular_circle
            && carrier.start == *start
            && start.scalar().is_some_and(|boundary| {
                end.as_bezier_parameter()
                    .is_some_and(|end| isolator_touches(end, boundary, true))
            }) {
            start.scalar().cloned()
        } else if retained_regular_circle
            && carrier.end == *end
            && end.scalar().is_some_and(|boundary| {
                start
                    .as_bezier_parameter()
                    .is_some_and(|start| isolator_touches(start, boundary, false))
            })
        {
            end.scalar().cloned()
        } else {
            None
        };
        let mut source_endpoint_witness_attempted = false;
        let mut boundary_probe_representative = None;
        for _ in 0..max_representatives {
            // Keep the original chart and its parameter as the winding
            // authority. Sampling a rematerialized circular fragment both
            // expands its coordinate expressions and discards that identity.
            let (parameter, representative, derivative) = {
                let parameter = if !source_endpoint_witness_attempted {
                    source_endpoint_witness_attempted = true;
                    source_endpoint_witness.clone()
                } else {
                    None
                };
                let parameter = match parameter {
                    Some(parameter) => parameter,
                    None => {
                        let parameter = match start
                            .strict_scalar_between_ordered(&upper, &self.data.policy)
                            .map_err(|cause| self.invalid(carrier_index, cause))?
                        {
                            Classification::Decided(parameter) => parameter,
                            Classification::Uncertain(reason) => {
                                last_reason = reason;
                                break;
                            }
                        };
                        upper = CurveParameter2::from(BezierParameter2::Exact(parameter.clone()));
                        parameter
                    }
                };
                let retained_endpoint = source_endpoint_witness
                    .as_ref()
                    .filter(|endpoint| *endpoint == &parameter)
                    .and_then(|endpoint| match &carrier.geometry {
                        CurveSupport2::Bezier(curve) if endpoint == &crate::Real::zero() => {
                            Some(curve.endpoint_refs().0.clone())
                        }
                        CurveSupport2::Bezier(curve) if endpoint == &crate::Real::one() => {
                            Some(curve.endpoint_refs().1.clone())
                        }
                        CurveSupport2::Bezier(_)
                        | CurveSupport2::Parallel(_)
                        | CurveSupport2::Line(_)
                        | CurveSupport2::Circle(_) => None,
                    });
                let representative = match retained_endpoint {
                    Some(point) => point,
                    None => match carrier
                        .geometry
                        .point_at(&parameter, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(point) => point,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue;
                        }
                    },
                };
                let derivative = match carrier
                    .geometry
                    .derivative_at(&parameter, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(derivative) => derivative,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue;
                    }
                };
                (parameter, representative, derivative)
            };
            let tangent_squared =
                derivative.dx() * derivative.dx() + derivative.dy() * derivative.dy();
            let regular = match crate::classify::is_zero(&tangent_squared, &self.data.policy) {
                Some(false) => true,
                Some(true) => {
                    last_reason = UncertaintyReason::Boundary;
                    false
                }
                None if retained_regular_circle => true,
                None => {
                    match rational_geometry.as_ref().map(|curve| {
                        curve.derivative_is_certified_nonzero_at(&parameter, &self.data.policy)
                    }) {
                        Some(Ok(Classification::Decided(true))) => true,
                        Some(Ok(Classification::Uncertain(reason))) => {
                            last_reason = reason;
                            false
                        }
                        Some(Ok(Classification::Decided(false))) | None => {
                            last_reason = UncertaintyReason::RealSign;
                            false
                        }
                        Some(Err(cause)) => return Err(self.invalid(carrier_index, cause)),
                    }
                }
            };
            if !regular {
                continue;
            }
            let (mut tangent_x, mut tangent_y) = (derivative.dx().clone(), derivative.dy().clone());
            if carrier.reversed {
                tangent_x = -tangent_x;
                tangent_y = -tangent_y;
            }
            let source_parameter = CurveParameter2::from(BezierParameter2::Exact(parameter));
            boundary_probe_representative =
                Some((representative.clone(), source_parameter.clone()));
            let left = match self.fragment_side_classification(
                carrier_index,
                &representative,
                Some(&source_parameter),
                &tangent_x,
                &tangent_y,
                true,
            ) {
                Ok(location) => location,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error) => return Err(error),
            };
            let right = match self.fragment_side_classification(
                carrier_index,
                &representative,
                Some(&source_parameter),
                &tangent_x,
                &tangent_y,
                false,
            ) {
                Ok(location) => location,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error) => return Err(error),
            };
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                left, right,
            ));
        }
        if last_reason == UncertaintyReason::Boundary
            && let Some((representative, parameter)) = boundary_probe_representative
        {
            return self.regularized_fragment_decision_by_boundary_probe(
                carrier_index,
                CurvePoint2::from(representative),
                Some(&parameter),
            );
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    pub(super) fn regularized_algebraic_cusp_fragment_decision(
        &self,
        carrier_index: usize,
        fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let start = CurveParameter2::from_algebraic_cusp(fragment.start_parameter().clone());
        let end = CurveParameter2::from_algebraic_cusp(fragment.end_parameter().clone());
        let parameter = match start
            .strict_scalar_between_ordered(&end, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(parameter) => parameter,
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
        let representative = match fragment
            .semicircle()
            .point_at(&parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(UncertaintyReason::Unsupported) => {
                return self.regularized_algebraic_cusp_fragment_decision_by_probe(
                    carrier_index,
                    fragment,
                    &parameter,
                );
            }
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
        let tangent = match fragment
            .semicircle()
            .tangent_at(&parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(tangent) => tangent,
            Classification::Uncertain(reason) => return Err(self.blocked(carrier_index, reason)),
        };
        let (Some(representative_point), Some((mut tangent_x, mut tangent_y))) = (
            representative.exact_point(&self.data.policy),
            tangent.exact_vector(&self.data.policy),
        ) else {
            match self.regularized_algebraic_cusp_fragment_decision_in_selected_field(
                carrier_index,
                fragment,
                &representative,
                &tangent,
            ) {
                Ok(decision) => return Ok(decision),
                Err(ExactCurveError::Blocked(_)) => {
                    return self.regularized_algebraic_cusp_fragment_decision_by_probe(
                        carrier_index,
                        fragment,
                        &parameter,
                    );
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            }
        };
        if fragment.is_reversed() {
            tangent_x = -tangent_x;
            tangent_y = -tangent_y;
        }
        let source_parameter = CurveParameter2::from_algebraic_cusp(
            crate::bezier_offset::BezierAlgebraicCuspSemicircleParameter2::Exact(parameter),
        );
        let left = self.fragment_side_classification(
            carrier_index,
            &representative_point,
            Some(&source_parameter),
            &tangent_x,
            &tangent_y,
            true,
        )?;
        let right = self.fragment_side_classification(
            carrier_index,
            &representative_point,
            Some(&source_parameter),
            &tangent_x,
            &tangent_y,
            false,
        )?;
        Ok(RegularizedFragmentDecision::from_classified_sides(
            left, right,
        ))
    }

    /// Same-selected-field accelerator for unary cusp side classification.
    /// Any incomplete sign or ray predicate falls through to the general
    /// retained boundary probe instead of becoming an operation blocker.
    pub(super) fn regularized_algebraic_cusp_fragment_decision_in_selected_field(
        &self,
        carrier_index: usize,
        fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
        representative: &crate::RationalBezierAlgebraicPointImage2,
        tangent: &crate::RationalBezierAlgebraicTangentImage2,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let tangent_x = tangent
            .coordinate_sign(true, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?;
        let tangent_y = tangent
            .coordinate_sign(false, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?;
        let reverse_sign = |sign| match sign {
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => RealSign::Zero,
            RealSign::Positive => RealSign::Negative,
        };
        let tangent_x = tangent_x.map(|sign| {
            if fragment.is_reversed() {
                reverse_sign(sign)
            } else {
                sign
            }
        });
        let tangent_y = tangent_y.map(|sign| {
            if fragment.is_reversed() {
                reverse_sign(sign)
            } else {
                sign
            }
        });
        let left = self.algebraic_fragment_side_classification(
            carrier_index,
            representative,
            tangent_x,
            tangent_y,
            true,
        )?;
        let right = self.algebraic_fragment_side_classification(
            carrier_index,
            representative,
            tangent_x,
            tangent_y,
            false,
        )?;
        Ok(RegularizedFragmentDecision::from_classified_sides(
            left, right,
        ))
    }

    pub(super) fn regularized_algebraic_cusp_fragment_decision_by_probe(
        &self,
        carrier_index: usize,
        fragment: &crate::BezierAlgebraicCuspSemicircleFragment2,
        parameter: &Real,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let target = match fragment
            .semicircle()
            .point_evidence_at(parameter, &self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };

        let mut last_reason = UncertaintyReason::Unsupported;
        let mut outer_bounds = None;
        for refinement_steps in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512] {
            let mut accumulated = None::<Aabb2>;
            let mut complete = true;
            for carrier in &self.data.carriers {
                let bounds = match carrier.geometry.certified_outer_bounds(
                    &carrier.range(),
                    refinement_steps,
                    &self.data.policy,
                ) {
                    Classification::Decided(bounds) => bounds,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        complete = false;
                        break;
                    }
                };
                let bounds = bounds
                    .certified_rational_outer_envelope(refinement_steps)
                    .unwrap_or(bounds);
                accumulated = Some(match accumulated {
                    None => bounds,
                    Some(ref accumulated) => match accumulated.union(&bounds) {
                        Classification::Decided(bounds) => bounds,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            complete = false;
                            break;
                        }
                    },
                });
            }
            if complete {
                outer_bounds = accumulated;
                break;
            }
        }
        let Some(outer_bounds) = outer_bounds else {
            return Err(self.blocked(carrier_index, last_reason));
        };
        let one = Real::one();
        let outside_points = [
            crate::Point2::new(outer_bounds.min().x() - &one, outer_bounds.min().y() - &one),
            crate::Point2::new(outer_bounds.max().x() + &one, outer_bounds.min().y() - &one),
            crate::Point2::new(outer_bounds.max().x() + &one, outer_bounds.max().y() + &one),
            crate::Point2::new(outer_bounds.min().x() - &one, outer_bounds.max().y() + &one),
        ];

        for outside in outside_points {
            let probe = match crate::BezierAlgebraicChord2::try_new(
                CurvePoint2::from(outside),
                target.clone(),
                &self.data.policy,
            )
            .map_err(|cause| self.invalid(carrier_index, cause))?
            {
                Classification::Decided(probe) => probe,
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                    continue;
                }
            };
            let probe_end = CurveParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_boundary(probe, None) {
                Ok(evidence) => evidence,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            };
            if !evidence.overlaps().is_empty() {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            if let Some(blocker) = evidence.blockers().first() {
                last_reason = blocker
                    .uncertainty_reason()
                    .unwrap_or(UncertaintyReason::Unsupported);
                continue;
            }

            let mut crossings = Vec::<(CurveParameter2, usize, bool)>::new();
            let mut target_cross = None;
            let mut ambiguous = false;
            for contact in evidence.contacts() {
                let order = match contact
                    .first_parameter()
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(order) => order,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        ambiguous = true;
                        break;
                    }
                };
                if order == Ordering::Greater {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization boundary probe retained a contact past its endpoint"
                                .into(),
                        ),
                    ));
                }
                let Some(boundary_index) = contact.second().carrier_index().checked_sub(1) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization boundary probe lost its source carrier".into(),
                        ),
                    ));
                };
                let Some(boundary) = self.data.carriers.get(boundary_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization boundary probe referenced an unknown carrier".into(),
                        ),
                    ));
                };
                let Some(mut cross_is_positive) = contact.evidence.tangent_cross_is_positive()
                else {
                    if order == Ordering::Equal {
                        last_reason = UncertaintyReason::Boundary;
                        ambiguous = true;
                        break;
                    }
                    continue;
                };
                cross_is_positive ^= boundary.reversed;
                if order == Ordering::Equal {
                    if boundary_index != carrier_index
                        || target_cross.replace(cross_is_positive).is_some()
                    {
                        last_reason = UncertaintyReason::Boundary;
                        ambiguous = true;
                        break;
                    }
                    continue;
                }
                if !contact.is_certified_transverse() {
                    continue;
                }
                for (parameter, _, _) in &crossings {
                    match contact
                        .first_parameter()
                        .cmp_by_refinement(parameter, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(Ordering::Equal) => {
                            last_reason = UncertaintyReason::Boundary;
                            ambiguous = true;
                            break;
                        }
                        Classification::Decided(Ordering::Less | Ordering::Greater) => {}
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            ambiguous = true;
                            break;
                        }
                    }
                }
                if ambiguous {
                    break;
                }
                crossings.push((
                    contact.first_parameter().clone(),
                    boundary_index,
                    cross_is_positive,
                ));
            }
            if ambiguous {
                continue;
            }
            let Some(target_cross) = target_cross else {
                last_reason = UncertaintyReason::Boundary;
                continue;
            };

            let mut incoming = vec![0_i32; self.data.first.boundary_loops().len()];
            for (_, boundary_index, cross_is_positive) in crossings {
                let loop_index = self.data.carriers[boundary_index].loop_index;
                let Some(winding) = incoming.get_mut(loop_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a regularization probe crossing lost its boundary loop".into(),
                        ),
                    ));
                };
                *winding += if cross_is_positive { -1 } else { 1 };
            }
            let mut outgoing = incoming.clone();
            outgoing[self.data.carriers[carrier_index].loop_index] +=
                if target_cross { -1 } else { 1 };
            let (left_windings, right_windings) = if target_cross {
                (incoming, outgoing)
            } else {
                (outgoing, incoming)
            };
            let left = self
                .location_from_windings(self.data.first, &left_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            let right = self
                .location_from_windings(self.data.first, &right_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-regularization-cusp-side",
                "exact-boundary-probe",
            );
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                (left_windings, left),
                (right_windings, right),
            ));
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    pub(super) fn regularized_algebraic_chord_fragment_decision(
        &self,
        carrier_index: usize,
        chord: &crate::BezierAlgebraicChord2,
        has_single_boundary_jump: bool,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let representative = match chord
            .representative_point(&self.data.policy)
            .map_err(|cause| self.invalid(carrier_index, cause))?
        {
            Classification::Decided(point) => point,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        // Complete pair replay has split every transverse contact away from
        // this open fragment, and the caller excludes coincident spans.  Its
        // two local sides therefore differ by exactly this oriented source
        // edge's unit winding.  Classify one side and derive the other instead
        // of intersecting a second, antiparallel boundary ray with the entire
        // region.
        let opposite_from_left = |left: &(Vec<i32>, RegionPointLocation)| {
            if !has_single_boundary_jump {
                return Ok(None);
            }
            let loop_index = self.data.carriers[carrier_index].loop_index;
            let mut windings = left.0.clone();
            let Some(winding) = windings.get_mut(loop_index) else {
                return Err(self.invalid(
                    carrier_index,
                    CurveError::Topology(
                        "a regularized fragment references a missing source loop".into(),
                    ),
                ));
            };
            *winding = winding.checked_sub(1).ok_or_else(|| {
                self.invalid(
                    carrier_index,
                    CurveError::Topology("a regularized winding underflowed i32".into()),
                )
            })?;
            let location = self
                .location_from_windings(self.region_for_carrier(carrier_index), &windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "regularization-fragment-side",
                "single-boundary-winding-jump",
            );
            Ok(Some((windings, location)))
        };
        if let CurvePoint2(CurvePointData2::Exact(representative)) = representative {
            let tangent = chord
                .exact_line()
                .map(|line| line.delta())
                .or_else(|| {
                    chord
                        .strict_provenance_support_line(&self.data.policy)
                        .map(|line| line.delta())
                })
                .or_else(|| chord.certified_unit_tangent());
            let Some((tangent_x, tangent_y)) = tangent else {
                return self.regularized_fragment_decision_by_boundary_probe(
                    carrier_index,
                    CurvePoint2::from(representative),
                    None,
                );
            };
            let source_chord = match &self.data.carriers[carrier_index].geometry {
                CurveSupport2::Line(source) => source,
                CurveSupport2::Bezier(_)
                | CurveSupport2::Parallel(_)
                | CurveSupport2::Circle(_) => chord,
            };
            // `chord` is an ordered nonempty split of `source_chord`, and its
            // representative is strictly interior to that split.  This
            // construction is a stronger finite-domain certificate than
            // independently reordering the transformed endpoint fields.
            let parameter = CurveParameter2::from_algebraic_chord(
                source_chord.parameter_at_certified_interior_point(CurvePoint2::from(
                    representative.clone(),
                )),
            );
            let classify = |left| {
                self.fragment_side_classification(
                    carrier_index,
                    &representative,
                    Some(&parameter),
                    &tangent_x,
                    &tangent_y,
                    left,
                )
            };
            let left = match classify(true) {
                Ok(classification) => classification,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() == UncertaintyReason::Boundary =>
                {
                    return self.regularized_fragment_decision_by_boundary_probe(
                        carrier_index,
                        CurvePoint2::from(representative),
                        None,
                    );
                }
                Err(error) => return Err(error),
            };
            let right = match opposite_from_left(&left)? {
                Some(classification) => classification,
                None => match classify(false) {
                    Ok(classification) => classification,
                    Err(ExactCurveError::Blocked(blocker))
                        if blocker.reason() == UncertaintyReason::Boundary =>
                    {
                        return self.regularized_fragment_decision_by_boundary_probe(
                            carrier_index,
                            CurvePoint2::from(representative),
                            None,
                        );
                    }
                    Err(error) => return Err(error),
                },
            };
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                left, right,
            ));
        }
        let representative = match representative {
            CurvePoint2(CurvePointData2::Algebraic(representative)) => representative,
            representative => {
                return self.regularized_fragment_decision_by_boundary_probe(
                    carrier_index,
                    representative,
                    None,
                );
            }
        };
        let tangent = |axis| {
            chord
                .tangent_axis_sign(axis, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))
        };
        let [tangent_x, tangent_y] = [tangent(Axis2::X)?, tangent(Axis2::Y)?];
        let classify = |left| {
            self.algebraic_fragment_side_classification(
                carrier_index,
                &representative,
                tangent_x,
                tangent_y,
                left,
            )
        };
        let left = match classify(true) {
            Ok(classification) => classification,
            Err(ExactCurveError::Blocked(blocker))
                if blocker.reason() == UncertaintyReason::Boundary =>
            {
                return self.regularized_fragment_decision_by_boundary_probe(
                    carrier_index,
                    CurvePoint2::from(representative),
                    None,
                );
            }
            Err(error) => return Err(error),
        };
        let right = match opposite_from_left(&left)? {
            Some(classification) => classification,
            None => match classify(false) {
                Ok(classification) => classification,
                Err(ExactCurveError::Blocked(blocker))
                    if blocker.reason() == UncertaintyReason::Boundary =>
                {
                    return self.regularized_fragment_decision_by_boundary_probe(
                        carrier_index,
                        CurvePoint2::from(representative),
                        None,
                    );
                }
                Err(error) => return Err(error),
            },
        };
        Ok(RegularizedFragmentDecision::from_classified_sides(
            left, right,
        ))
    }

    /// Seeds both local faces when a representative lies on a coincident
    /// boundary stack or lives in more than one selected field.
    ///
    /// A certified exterior rational point has zero winding in every source
    /// loop. Complete carrier-pair replay orders the crossings of the retained
    /// probe ending at the fragment representative. The transverse endpoint
    /// contacts identify the incoming face, and their aggregate loop-winding
    /// jump identifies the opposite face. No Cartesian coordinate or finite
    /// side displacement is manufactured.
    #[cold]
    pub(super) fn regularized_fragment_decision_by_boundary_probe(
        &self,
        carrier_index: usize,
        representative: CurvePoint2,
        source_parameter: Option<&CurveParameter2>,
    ) -> ExactCurveResult<RegularizedFragmentDecision> {
        let outer_bounds = match retained_probe_outer_bounds(&self.data.carriers, &self.data.policy)
        {
            Classification::Decided(bounds) => bounds,
            Classification::Uncertain(reason) => {
                return Err(self.blocked(carrier_index, reason));
            }
        };
        let loop_count = self.data.first.boundary_loops().len();
        let source_loop_index = self.data.carriers[carrier_index].loop_index;
        if source_loop_index >= loop_count {
            return Err(self.invalid(
                carrier_index,
                CurveError::Topology(
                    "a regularization boundary probe references a missing source loop".into(),
                ),
            ));
        }
        let retained_probe = match (&representative, &self.data.carriers[carrier_index].geometry) {
            (
                CurvePoint2(CurvePointData2::AlgebraicChordPair(point)),
                CurveSupport2::Line(boundary),
            ) => point
                .exterior_axis_probe_avoiding(boundary, &outer_bounds, &self.data.policy)
                .map_err(|cause| self.invalid(carrier_index, cause))?,
            _ => None,
        };
        let retained_probe_count = usize::from(retained_probe.is_some());
        let candidate_count = self
            .data
            .carriers
            .len()
            .saturating_mul(2)
            .saturating_add(5)
            .saturating_add(retained_probe_count);
        let mut last_reason = UncertaintyReason::Unsupported;
        let authored_successor = |first_index: usize, second_index: usize| {
            let first = &self.data.carriers[first_index];
            let second = &self.data.carriers[second_index];
            if first.loop_index != second.loop_index {
                return false;
            }
            let fragment_count = self.data.first.boundary_loops()[first.loop_index].len();
            first.fragment_index.checked_add(1) == Some(second.fragment_index)
                || second.fragment_index == 0
                    && first.fragment_index.checked_add(1) == Some(fragment_count)
        };

        'candidate: for candidate_index in 0..candidate_count {
            let probe = if candidate_index < retained_probe_count {
                #[cfg(feature = "dispatch-trace")]
                hyperreal::dispatch_trace::record(
                    "hypercurve",
                    "curve-region-regularization-chord-side",
                    "retained-axis-probe",
                );
                retained_probe
                    .as_ref()
                    .expect("the retained probe count reflects its value")
                    .0
                    .clone()
            } else {
                let Some(outside) = retained_probe_exterior_candidate(
                    &outer_bounds,
                    candidate_index - retained_probe_count,
                ) else {
                    continue;
                };
                match crate::BezierAlgebraicChord2::try_new(
                    CurvePoint2::from(outside),
                    representative.clone(),
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(probe) => probe,
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue;
                    }
                }
            };
            let probe_end = CurveParameter2::from_algebraic_chord(probe.end_parameter());
            let evidence = match self.intersect_algebraic_probe_boundary(
                probe,
                source_parameter.map(|parameter| (carrier_index, parameter)),
            ) {
                Ok(evidence) => evidence,
                Err(ExactCurveError::Blocked(blocker)) => {
                    last_reason = blocker.reason();
                    continue;
                }
                Err(error @ ExactCurveError::Invalid { .. }) => return Err(error),
            };
            if !evidence.overlaps().is_empty() {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            if let Some(blocker) = evidence.blockers().first() {
                last_reason = blocker
                    .uncertainty_reason()
                    .unwrap_or(UncertaintyReason::Unsupported);
                continue;
            }

            let mut has_nontransverse_endpoint_contact = false;
            for contact in evidence.contacts() {
                let order = contact
                    .first_parameter()
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?;
                match order {
                    Classification::Decided(Ordering::Less) => {}
                    Classification::Decided(Ordering::Equal) => {
                        has_nontransverse_endpoint_contact |= !contact.is_certified_transverse();
                    }
                    Classification::Decided(Ordering::Greater) => {
                        return Err(self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "a retained boundary-side probe kept a contact past its endpoint"
                                    .into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue 'candidate;
                    }
                }
            }

            let mut crossings =
                Vec::<(&CurveParameter2, &CurveParameter2, usize, bool)>::with_capacity(
                    evidence.contacts().len(),
                );
            for contact in evidence
                .contacts()
                .iter()
                .filter(|contact| contact.is_certified_transverse())
            {
                let Some(mut cross_is_positive) = contact.evidence.tangent_cross_is_positive()
                else {
                    last_reason = UncertaintyReason::Predicate;
                    continue 'candidate;
                };
                let Some(boundary_index) = contact.second().carrier_index().checked_sub(1) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a retained boundary-side contact resolved to its probe carrier".into(),
                        ),
                    ));
                };
                let Some(boundary) = self.data.carriers.get(boundary_index) else {
                    return Err(self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "a retained boundary-side contact lost its boundary carrier".into(),
                        ),
                    ));
                };
                cross_is_positive ^= boundary.reversed;
                crossings.push((
                    contact.first_parameter(),
                    contact.second_parameter(),
                    boundary_index,
                    cross_is_positive,
                ));
            }
            for index in 1..crossings.len() {
                let mut cursor = index;
                while cursor > 0 {
                    match crossings[cursor]
                        .0
                        .cmp_by_refinement(crossings[cursor - 1].0, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(Ordering::Less) => {
                            crossings.swap(cursor, cursor - 1);
                            cursor -= 1;
                        }
                        Classification::Decided(Ordering::Equal | Ordering::Greater) => break,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    }
                }
            }

            let mut windings = vec![0_i32; loop_count];
            let mut group_start = 0_usize;
            while group_start < crossings.len() {
                let mut group_end = group_start + 1;
                while group_end < crossings.len() {
                    match crossings[group_end]
                        .0
                        .cmp_by_refinement(crossings[group_start].0, &self.data.policy)
                        .map_err(|cause| self.invalid(carrier_index, cause))?
                    {
                        Classification::Decided(Ordering::Equal) => group_end += 1,
                        Classification::Decided(Ordering::Greater) => break,
                        Classification::Decided(Ordering::Less) => {
                            return Err(self.invalid(
                                carrier_index,
                                CurveError::Topology(
                                    "retained boundary-side probe crossings lost exact order"
                                        .into(),
                                ),
                            ));
                        }
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    }
                }
                let group = &crossings[group_start..group_end];
                let endpoint_order = match group[0]
                    .0
                    .cmp_by_refinement(&probe_end, &self.data.policy)
                    .map_err(|cause| self.invalid(carrier_index, cause))?
                {
                    Classification::Decided(order @ (Ordering::Equal | Ordering::Less)) => order,
                    Classification::Decided(Ordering::Greater) => {
                        return Err(self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "retained boundary-side crossing followed its probe endpoint"
                                    .into(),
                            ),
                        ));
                    }
                    Classification::Uncertain(reason) => {
                        last_reason = reason;
                        continue 'candidate;
                    }
                };

                let source_cross_is_positive = if endpoint_order == Ordering::Equal {
                    let mut source_cross = None;
                    for &(_, _, boundary_index, cross_is_positive) in group {
                        if boundary_index != carrier_index {
                            continue;
                        }
                        if source_cross.replace(cross_is_positive).is_some() {
                            last_reason = UncertaintyReason::Boundary;
                            continue 'candidate;
                        }
                    }
                    let Some(source_cross) = source_cross else {
                        last_reason = UncertaintyReason::Predicate;
                        continue 'candidate;
                    };
                    Some(source_cross)
                } else {
                    None
                };

                let mut endpoint_roles = Vec::with_capacity(group.len());
                for &(_, parameter, boundary_index, _) in group {
                    let boundary = &self.data.carriers[boundary_index];
                    let (start, end) = if boundary.reversed {
                        (&boundary.end, &boundary.start)
                    } else {
                        (&boundary.start, &boundary.end)
                    };
                    let endpoint_role = |endpoint| {
                        parameter
                            .same_value(endpoint, &self.data.policy)
                            .map_err(|cause| self.invalid(carrier_index, cause))
                    };
                    let at_start = match endpoint_role(start)? {
                        Classification::Decided(at_start) => at_start,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    };
                    let at_end = match endpoint_role(end)? {
                        Classification::Decided(at_end) => at_end,
                        Classification::Uncertain(reason) => {
                            last_reason = reason;
                            continue 'candidate;
                        }
                    };
                    endpoint_roles.push([at_start, at_end]);
                }
                let mut group_deltas = vec![0_i32; loop_count];
                let mut consumed = vec![false; group.len()];
                for index in 0..group.len() {
                    if consumed[index] {
                        continue;
                    }
                    let (_, _, boundary_index, cross_is_positive) = group[index];
                    let mut partner = None;
                    for candidate in index + 1..group.len() {
                        if consumed[candidate] {
                            continue;
                        }
                        let candidate_index = group[candidate].2;
                        let adjacent = authored_successor(boundary_index, candidate_index)
                            && endpoint_roles[index][1]
                            && endpoint_roles[candidate][0]
                            || authored_successor(candidate_index, boundary_index)
                                && endpoint_roles[candidate][1]
                                && endpoint_roles[index][0];
                        if !adjacent {
                            continue;
                        }
                        if partner.replace(candidate).is_some() {
                            last_reason = UncertaintyReason::Boundary;
                            continue 'candidate;
                        }
                    }
                    consumed[index] = true;
                    let delta = if let Some(partner) = partner {
                        consumed[partner] = true;
                        if cross_is_positive == group[partner].3 {
                            if cross_is_positive { -1 } else { 1 }
                        } else {
                            0
                        }
                    } else if cross_is_positive {
                        -1
                    } else {
                        1
                    };
                    let loop_index = self.data.carriers[boundary_index].loop_index;
                    let Some(winding) = group_deltas.get_mut(loop_index) else {
                        return Err(self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "a retained boundary-side contact references a missing loop".into(),
                            ),
                        ));
                    };
                    *winding = winding.checked_add(delta).ok_or_else(|| {
                        self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "retained boundary-side winding overflowed i32".into(),
                            ),
                        )
                    })?;
                }
                if let Some(source_cross_is_positive) = source_cross_is_positive {
                    let mut opposite = windings.clone();
                    for (winding, delta) in opposite.iter_mut().zip(group_deltas) {
                        *winding = winding.checked_add(delta).ok_or_else(|| {
                            self.invalid(
                                carrier_index,
                                CurveError::Topology(
                                    "retained boundary-side winding overflowed i32".into(),
                                ),
                            )
                        })?;
                    }
                    let (left_windings, right_windings) = if source_cross_is_positive {
                        (windings, opposite)
                    } else {
                        (opposite, windings)
                    };
                    let left = self
                        .location_from_windings(self.data.first, &left_windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    let right = self
                        .location_from_windings(self.data.first, &right_windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    #[cfg(feature = "dispatch-trace")]
                    hyperreal::dispatch_trace::record(
                        "hypercurve",
                        "curve-region-regularization-chord-side",
                        "retained-endpoint-winding-probe",
                    );
                    return Ok(RegularizedFragmentDecision::from_classified_sides(
                        (left_windings, left),
                        (right_windings, right),
                    ));
                }
                for (winding, delta) in windings.iter_mut().zip(group_deltas) {
                    *winding = winding.checked_add(delta).ok_or_else(|| {
                        self.invalid(
                            carrier_index,
                            CurveError::Topology(
                                "retained boundary-side winding overflowed i32".into(),
                            ),
                        )
                    })?;
                }
                group_start = group_end;
            }
            if has_nontransverse_endpoint_contact {
                last_reason = UncertaintyReason::Boundary;
                continue;
            }
            let Some((_, source_cross_is_positive)) = retained_probe
                .as_ref()
                .filter(|_| candidate_index < retained_probe_count)
            else {
                continue;
            };
            // This retained probe ends at a point constructed strictly inside
            // the split source support. If generic finite-incidence replay did
            // not rediscover that endpoint, its defining-support cross still
            // owns the exact missing source jump. The representative cannot
            // be another event because complete pair replay selected it in an
            // open split fragment.
            let source_cross_is_positive =
                *source_cross_is_positive ^ self.data.carriers[carrier_index].reversed;
            let mut opposite = windings.clone();
            opposite[source_loop_index] = opposite[source_loop_index]
                .checked_add(if source_cross_is_positive { -1 } else { 1 })
                .ok_or_else(|| {
                    self.invalid(
                        carrier_index,
                        CurveError::Topology(
                            "retained boundary-side winding overflowed i32".into(),
                        ),
                    )
                })?;
            let (left_windings, right_windings) = if source_cross_is_positive {
                (windings, opposite)
            } else {
                (opposite, windings)
            };
            let left = self
                .location_from_windings(self.data.first, &left_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            let right = self
                .location_from_windings(self.data.first, &right_windings)
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            #[cfg(feature = "dispatch-trace")]
            hyperreal::dispatch_trace::record(
                "hypercurve",
                "curve-region-regularization-chord-side",
                "retained-constructed-endpoint-winding-probe",
            );
            return Ok(RegularizedFragmentDecision::from_classified_sides(
                (left_windings, left),
                (right_windings, right),
            ));
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    pub(super) fn algebraic_fragment_side_classification(
        &self,
        carrier_index: usize,
        representative: &crate::RationalBezierAlgebraicPointImage2,
        tangent_x: Classification<RealSign>,
        tangent_y: Classification<RealSign>,
        left: bool,
    ) -> ExactCurveResult<(Vec<i32>, RegionPointLocation)> {
        let carrier = &self.data.carriers[carrier_index];
        let reverse_sign = |sign| match sign {
            RealSign::Negative => RealSign::Positive,
            RealSign::Zero => RealSign::Zero,
            RealSign::Positive => RealSign::Negative,
        };
        let mut last_reason = UncertaintyReason::RealSign;
        let tangent_x = match tangent_x {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(reason) => {
                last_reason = reason;
                None
            }
        };
        let tangent_y = match tangent_y {
            Classification::Decided(sign) => Some(sign),
            Classification::Uncertain(reason) => {
                last_reason = reason;
                None
            }
        };
        let normal_x = tangent_y.map(reverse_sign);
        let normal_y = tangent_x;
        let normal_x = if left {
            normal_x
        } else {
            normal_x.map(reverse_sign)
        };
        let normal_y = if left {
            normal_y
        } else {
            normal_y.map(reverse_sign)
        };
        let unit = |sign: Option<RealSign>| match sign {
            Some(RealSign::Negative) => -1_i8,
            Some(RealSign::Positive) => 1_i8,
            Some(RealSign::Zero) | None => 0_i8,
        };
        let x = unit(normal_x);
        let y = unit(normal_y);
        if x == 0 && y == 0 {
            return Err(self.blocked(carrier_index, last_reason));
        }
        let directions = [
            (x, 0_i8),
            (0_i8, y),
            (x, y),
            (x.saturating_mul(2), y),
            (x, y.saturating_mul(2)),
            (x.saturating_mul(3), y),
            (x, y.saturating_mul(3)),
        ];
        for (direction_x, direction_y) in directions {
            if direction_x == 0 && direction_y == 0 {
                continue;
            }
            let classification = self
                .region_for_carrier(carrier_index)
                .algebraic_loop_windings_from_boundary_side_ray(
                    representative,
                    Real::from(direction_x),
                    Real::from(direction_y),
                    carrier.loop_index,
                    carrier.fragment_index,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            match classification {
                Classification::Decided(windings) => {
                    let location = self
                        .location_from_windings(self.region_for_carrier(carrier_index), &windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    return Ok((windings, location));
                }
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                }
            }
        }
        Err(self.blocked(carrier_index, last_reason))
    }

    pub(super) fn fragment_side_classification(
        &self,
        carrier_index: usize,
        representative: &crate::Point2,
        source_parameter: Option<&CurveParameter2>,
        tangent_x: &crate::Real,
        tangent_y: &crate::Real,
        left: bool,
    ) -> ExactCurveResult<(Vec<i32>, RegionPointLocation)> {
        self.fragment_side_classification_with_reference_tangent(
            carrier_index,
            representative,
            source_parameter,
            tangent_x,
            tangent_y,
            left,
            true,
        )
    }

    pub(super) fn fragment_side_classification_with_reference_tangent(
        &self,
        carrier_index: usize,
        representative: &crate::Point2,
        source_parameter: Option<&CurveParameter2>,
        tangent_x: &crate::Real,
        tangent_y: &crate::Real,
        left: bool,
        source_follows_reference_tangent: bool,
    ) -> ExactCurveResult<(Vec<i32>, RegionPointLocation)> {
        let carrier = &self.data.carriers[carrier_index];
        let normal_x = if left {
            -tangent_y.clone()
        } else {
            tangent_y.clone()
        };
        let normal_y = if left {
            tangent_x.clone()
        } else {
            -tangent_x.clone()
        };
        let x_axis = match crate::classify::compare_reals(
            &normal_x,
            &crate::Real::zero(),
            &self.data.policy,
        ) {
            Some(Ordering::Greater) => Some((crate::Real::one(), crate::Real::zero())),
            Some(Ordering::Less) => Some((-crate::Real::one(), crate::Real::zero())),
            Some(Ordering::Equal) | None => None,
        };
        let y_axis = match crate::classify::compare_reals(
            &normal_y,
            &crate::Real::zero(),
            &self.data.policy,
        ) {
            Some(Ordering::Greater) => Some((crate::Real::zero(), crate::Real::one())),
            Some(Ordering::Less) => Some((crate::Real::zero(), -crate::Real::one())),
            Some(Ordering::Equal) | None => None,
        };
        // Axis rays keep the analytic line-incidence coefficients smallest;
        // the tangent-derived directions remain exact fallbacks when an axis
        // contact lands on a harder algebraic ordering boundary.
        let directions = [
            x_axis,
            y_axis,
            Some((normal_x.clone(), normal_y.clone())),
            Some((&normal_x + tangent_x, &normal_y + tangent_y)),
            Some((&normal_x - tangent_x, &normal_y - tangent_y)),
            Some((
                &normal_x * crate::Real::from(2_u8) + tangent_x,
                &normal_y * crate::Real::from(2_u8) + tangent_y,
            )),
            Some((
                &normal_x * crate::Real::from(2_u8) - tangent_x,
                &normal_y * crate::Real::from(2_u8) - tangent_y,
            )),
        ];
        let mut last_reason = UncertaintyReason::Boundary;
        for (direction_x, direction_y) in directions.into_iter().flatten() {
            let result = self
                .region_for_carrier(carrier_index)
                .loop_windings_from_boundary_side_ray(
                    representative,
                    direction_x,
                    direction_y,
                    true,
                    if left == source_follows_reference_tangent {
                        BezierLineCrossingDirection::PositiveToNegative
                    } else {
                        BezierLineCrossingDirection::NegativeToPositive
                    },
                    carrier.loop_index,
                    carrier.fragment_index,
                    source_parameter,
                    &self.data.policy,
                )
                .map_err(|cause| self.invalid(carrier_index, cause))?;
            match result {
                Classification::Decided(windings) => {
                    let location = self
                        .location_from_windings(self.region_for_carrier(carrier_index), &windings)
                        .map_err(|cause| self.invalid(carrier_index, cause))?;
                    return Ok((windings, location));
                }
                Classification::Uncertain(reason) => {
                    last_reason = reason;
                }
            }
        }
        Err(self.blocked(carrier_index, last_reason))
    }
}
