//! Exact output face-loop assembly for boolmesh `boolean45`.
//!
//! Legacy triangulation first groups each output face's halfedges by tail
//! vertex, then walks from the current head to the next halfedge with that
//! tail.  This is the topological face-boundary step before triangulation.
//! Closed walks with fewer than three halfedges are preserved here because
//! legacy `assemble_halfs` returns raw loops; the later triangulation handoff
//! owns the short-loop rejection.
//! Following Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997), this exact port records incomplete and malformed
//! traversal states instead of using rounded geometry or panic-driven repair.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use super::super::{
    ExactBoolMeshDroppedOpenChain, ExactBoolMeshDroppedOpenChainOwner,
    ExactBoolMeshDroppedOpenChainSourceKind, ExactBoolMeshFaceLoopAssemblyStage,
    ExactBoolMeshHalfedgeAssemblyStage, ExactBoolMeshOutputFaceLoop,
    ExactBoolMeshOutputHalfedgeSource,
};

/// Assemble per-face closed boundary loops from emitted output halfedges.
pub(super) fn assemble_output_face_loops(
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_halfedge_offsets: &[usize],
    exact_degenerate_halfedges: &[bool],
    canonical_output_vertices: &[usize],
) -> ExactBoolMeshFaceLoopAssemblyStage {
    let mut stage = ExactBoolMeshFaceLoopAssemblyStage {
        canonical_output_vertices: canonical_output_vertices.to_vec(),
        ..ExactBoolMeshFaceLoopAssemblyStage::default()
    };
    for output_face in 0..face_halfedge_offsets.len().saturating_sub(1) {
        assemble_output_face_loop(
            output_face,
            halfedges,
            face_halfedge_offsets,
            exact_degenerate_halfedges,
            canonical_output_vertices,
            &mut stage,
        );
    }
    stage
}

fn assemble_output_face_loop(
    output_face: usize,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_halfedge_offsets: &[usize],
    exact_degenerate_halfedges: &[bool],
    canonical_output_vertices: &[usize],
    stage: &mut ExactBoolMeshFaceLoopAssemblyStage,
) {
    let begin = face_halfedge_offsets[output_face];
    let end = face_halfedge_offsets[output_face + 1];
    if begin == end {
        return;
    }
    if halfedges.output_halfedges[begin..end]
        .iter()
        .any(Option::is_none)
    {
        stage.incomplete_faces += 1;
        return;
    }
    let loops_before = stage.loops.len();
    let non_loop_before = stage.non_loop_halfedges;
    let repeated_before = stage.repeated_halfedges;
    let mut non_loop_open_chains = Vec::<ExactBoolMeshDroppedOpenChain>::new();

    let mut tail_to_halfedges = BTreeMap::<usize, VecDeque<usize>>::new();
    for slot in begin..end {
        let halfedge = halfedges.output_halfedges[slot]
            .as_ref()
            .expect("face range was checked as complete");
        if halfedge.face != output_face {
            continue;
        }
        tail_to_halfedges
            .entry(halfedge.tail)
            .or_default()
            .push_front(slot);
    }

    let mut consumed = BTreeSet::<usize>::new();
    while let Some(start) = next_loop_start(&tail_to_halfedges) {
        let mut current = start;
        let mut loop_halfedges = Vec::new();
        let mut loop_vertices = Vec::new();
        let mut local_seen = BTreeSet::<usize>::new();

        loop {
            if !local_seen.insert(current) {
                stage.repeated_halfedges += 1;
                break;
            }
            consumed.insert(current);
            let halfedge = halfedges.output_halfedges[current]
                .as_ref()
                .expect("face range was checked as complete");
            loop_halfedges.push(current);
            loop_vertices.push(halfedge.tail);
            pop_consumed(&mut tail_to_halfedges, halfedge.tail, current);

            if halfedge.head == loop_vertices[0]
                || (halfedge_chain_is_new_face_pair(halfedges, &loop_halfedges)
                    && canonical_vertex(canonical_output_vertices, halfedge.head)
                        == canonical_vertex(canonical_output_vertices, loop_vertices[0]))
            {
                stage.loops.push(ExactBoolMeshOutputFaceLoop {
                    output_face,
                    halfedges: loop_halfedges,
                    vertices: loop_vertices,
                });
                break;
            }

            let next = pop_next_for_tail(&mut tail_to_halfedges, halfedge.head).or_else(|| {
                if halfedge_chain_is_new_face_pair(halfedges, &loop_halfedges) {
                    pop_next_new_face_pair_for_canonical_tail(
                        &mut tail_to_halfedges,
                        canonical_vertex(canonical_output_vertices, halfedge.head),
                        halfedges,
                        canonical_output_vertices,
                    )
                } else {
                    None
                }
            });
            let Some(next) = next else {
                if halfedge_chain_is_source_edge(halfedges, &loop_halfedges)
                    || halfedge_chain_is_exact_degenerate(
                        exact_degenerate_halfedges,
                        &loop_halfedges,
                    )
                {
                    push_dropped_open_chain(
                        stage,
                        halfedges,
                        output_face,
                        loop_halfedges.clone(),
                        loop_vertices.clone(),
                    );
                } else {
                    stage.non_loop_halfedges += loop_halfedges.len();
                    non_loop_open_chains.push(ExactBoolMeshDroppedOpenChain {
                        output_face,
                        owner: dropped_open_chain_owner(halfedges, &loop_halfedges),
                        source_kind: dropped_open_chain_source_kind(halfedges, &loop_halfedges),
                        halfedges: loop_halfedges.clone(),
                        vertices: loop_vertices.clone(),
                    });
                }
                break;
            };
            current = next;
        }
    }

    let expected = end - begin;
    if consumed.len() < expected {
        let unconsumed = (begin..end)
            .filter(|slot| !consumed.contains(slot))
            .collect::<Vec<_>>();
        stage.non_loop_halfedges += unconsumed.len();
        if !unconsumed.is_empty() {
            non_loop_open_chains.push(ExactBoolMeshDroppedOpenChain {
                output_face,
                owner: dropped_open_chain_owner(halfedges, &unconsumed),
                source_kind: dropped_open_chain_source_kind(halfedges, &unconsumed),
                vertices: unconsumed
                    .iter()
                    .filter_map(|slot| {
                        halfedges.output_halfedges[*slot]
                            .as_ref()
                            .map(|halfedge| halfedge.tail)
                    })
                    .collect(),
                halfedges: unconsumed,
            });
        }
    }
    if stage.loops.len() == loops_before
        && stage.repeated_halfedges == repeated_before
        && stage.non_loop_halfedges > non_loop_before
        && (face_has_boundary_halfedge(output_face, halfedges, begin, end)
            || face_has_only_partial_source_edge_halfedges(output_face, halfedges, begin, end)
            || face_has_source_edge_halfedge(output_face, halfedges, begin, end))
    {
        let dropped = stage.non_loop_halfedges - non_loop_before;
        stage.non_loop_halfedges = non_loop_before;
        stage.dropped_open_chain_halfedges += dropped;
        stage.dropped_open_chains.append(&mut non_loop_open_chains);
    }
}

fn push_dropped_open_chain(
    stage: &mut ExactBoolMeshFaceLoopAssemblyStage,
    stage_halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    output_face: usize,
    halfedges: Vec<usize>,
    vertices: Vec<usize>,
) {
    stage.dropped_open_chain_halfedges += halfedges.len();
    stage
        .dropped_open_chains
        .push(ExactBoolMeshDroppedOpenChain {
            output_face,
            owner: dropped_open_chain_owner(stage_halfedges, &halfedges),
            source_kind: dropped_open_chain_source_kind(stage_halfedges, &halfedges),
            halfedges,
            vertices,
        });
}

fn dropped_open_chain_owner(
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    slots: &[usize],
) -> Option<ExactBoolMeshDroppedOpenChainOwner> {
    let mut owner = None;
    for slot in slots {
        let halfedge = halfedges.output_halfedges.get(*slot)?.as_ref()?;
        let current = output_halfedge_source_owner(&halfedge.source);
        match owner {
            Some(existing) if existing != current => return None,
            Some(_) => {}
            None => owner = Some(current),
        }
    }
    owner
}

fn output_halfedge_source_owner(
    source: &ExactBoolMeshOutputHalfedgeSource,
) -> ExactBoolMeshDroppedOpenChainOwner {
    match source {
        ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge {
            side, source_face, ..
        }
        | ExactBoolMeshOutputHalfedgeSource::NewFacePair {
            side, source_face, ..
        }
        | ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
            side, source_face, ..
        } => ExactBoolMeshDroppedOpenChainOwner {
            side: *side,
            source_face: *source_face,
        },
    }
}

fn dropped_open_chain_source_kind(
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    slots: &[usize],
) -> ExactBoolMeshDroppedOpenChainSourceKind {
    let mut has_source_edge = false;
    let mut has_face_pair = false;
    for slot in slots {
        let Some(halfedge) = halfedges
            .output_halfedges
            .get(*slot)
            .and_then(Option::as_ref)
        else {
            return ExactBoolMeshDroppedOpenChainSourceKind::Mixed;
        };
        match halfedge.source {
            ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge { .. }
            | ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge { .. } => {
                has_source_edge = true;
            }
            ExactBoolMeshOutputHalfedgeSource::NewFacePair { .. } => {
                has_face_pair = true;
            }
        }
    }
    match (has_source_edge, has_face_pair) {
        (true, false) => ExactBoolMeshDroppedOpenChainSourceKind::SourceEdge,
        (false, true) => ExactBoolMeshDroppedOpenChainSourceKind::FacePair,
        _ => ExactBoolMeshDroppedOpenChainSourceKind::Mixed,
    }
}

fn halfedge_chain_is_source_edge(
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    slots: &[usize],
) -> bool {
    !slots.is_empty()
        && slots.iter().all(|slot| {
            halfedges.output_halfedges[*slot]
                .as_ref()
                .is_some_and(|halfedge| {
                    matches!(
                        halfedge.source,
                        ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge { .. }
                            | ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge { .. }
                    )
                })
        })
}

fn halfedge_chain_is_new_face_pair(
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    slots: &[usize],
) -> bool {
    !slots.is_empty()
        && slots.iter().all(|slot| {
            halfedges.output_halfedges[*slot]
                .as_ref()
                .is_some_and(|halfedge| {
                    matches!(
                        halfedge.source,
                        ExactBoolMeshOutputHalfedgeSource::NewFacePair { .. }
                    )
                })
        })
}

fn halfedge_chain_is_exact_degenerate(
    exact_degenerate_halfedges: &[bool],
    slots: &[usize],
) -> bool {
    !slots.is_empty()
        && slots.iter().all(|slot| {
            exact_degenerate_halfedges
                .get(*slot)
                .copied()
                .unwrap_or(false)
        })
}

fn pop_next_new_face_pair_for_canonical_tail(
    tail_to_halfedges: &mut BTreeMap<usize, VecDeque<usize>>,
    canonical_tail: usize,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    canonical_output_vertices: &[usize],
) -> Option<usize> {
    let tail = tail_to_halfedges
        .iter()
        .filter_map(|(tail, queue)| {
            let slot = queue.back().copied()?;
            let halfedge = halfedges.output_halfedges[slot].as_ref()?;
            let is_new_face_pair = matches!(
                halfedge.source,
                ExactBoolMeshOutputHalfedgeSource::NewFacePair { .. }
            );
            (is_new_face_pair
                && canonical_vertex(canonical_output_vertices, *tail) == canonical_tail)
                .then_some(*tail)
        })
        .next()?;
    pop_next_for_tail(tail_to_halfedges, tail)
}

fn canonical_vertex(canonical_output_vertices: &[usize], vertex: usize) -> usize {
    canonical_output_vertices
        .get(vertex)
        .copied()
        .unwrap_or(vertex)
}

fn face_has_only_partial_source_edge_halfedges(
    output_face: usize,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    begin: usize,
    end: usize,
) -> bool {
    halfedges.output_halfedges[begin..end]
        .iter()
        .all(|halfedge| {
            halfedge.as_ref().is_some_and(|halfedge| {
                halfedge.face == output_face
                    && matches!(
                        halfedge.source,
                        ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge { .. }
                    )
            })
        })
}

fn face_has_boundary_halfedge(
    output_face: usize,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    begin: usize,
    end: usize,
) -> bool {
    halfedges.output_halfedges[begin..end]
        .iter()
        .enumerate()
        .any(|(local, halfedge)| {
            let slot = begin + local;
            halfedge
                .as_ref()
                .is_some_and(|halfedge| halfedge.face == output_face && halfedge.pair == slot)
        })
}

fn face_has_source_edge_halfedge(
    output_face: usize,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    begin: usize,
    end: usize,
) -> bool {
    halfedges.output_halfedges[begin..end]
        .iter()
        .any(|halfedge| {
            halfedge.as_ref().is_some_and(|halfedge| {
                halfedge.face == output_face
                    && matches!(
                        halfedge.source,
                        ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge { .. }
                            | ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge { .. }
                    )
            })
        })
}

fn next_loop_start(tail_to_halfedges: &BTreeMap<usize, VecDeque<usize>>) -> Option<usize> {
    tail_to_halfedges
        .values()
        .find_map(|queue| queue.back().copied())
}

fn pop_next_for_tail(
    tail_to_halfedges: &mut BTreeMap<usize, VecDeque<usize>>,
    tail: usize,
) -> Option<usize> {
    let next = tail_to_halfedges
        .get_mut(&tail)
        .and_then(VecDeque::pop_back);
    tail_to_halfedges.retain(|_, queue| !queue.is_empty());
    next
}

fn pop_consumed(
    tail_to_halfedges: &mut BTreeMap<usize, VecDeque<usize>>,
    tail: usize,
    consumed: usize,
) {
    if let Some(queue) = tail_to_halfedges.get_mut(&tail) {
        if queue.back().copied() == Some(consumed) {
            queue.pop_back();
        } else if let Some(position) = queue.iter().position(|slot| *slot == consumed) {
            queue.remove(position);
        }
    }
    tail_to_halfedges.retain(|_, queue| !queue.is_empty());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::{
        ExactBoolMeshDroppedOpenChainOwner, ExactBoolMeshDroppedOpenChainSourceKind,
        ExactBoolMeshHalfedgeAssemblyStage, ExactBoolMeshOutputHalfedge,
        ExactBoolMeshOutputHalfedgeSource, ExactBoolMeshSide,
    };

    #[test]
    fn two_edge_closed_walk_is_preserved_for_triangulation_stage() {
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 0,
                    head: 1,
                    pair: 0,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 1,
                    head: 0,
                    pair: 1,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 1,
                        forward: true,
                    },
                }),
            ],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };

        let stage = assemble_output_face_loops(&halfedges, &[0, 2], &[false, false], &[0, 1]);

        assert_eq!(stage.incomplete_faces, 0);
        assert_eq!(stage.repeated_halfedges, 0);
        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.loops.len(), 1);
        assert_eq!(stage.loops[0].halfedges, vec![0, 1]);
        assert_eq!(stage.loops[0].vertices, vec![0, 1]);
    }

    #[test]
    fn open_boundary_chain_is_replayed_as_lower_dimensional_drop() {
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 1,
                    head: 0,
                    pair: 0,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge {
                        side: ExactBoolMeshSide::Left,
                        source_halfedge: 0,
                        source_face: 0,
                        edge: [0, 1],
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 0,
                    head: 2,
                    pair: 4,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 3,
                    head: 1,
                    pair: 5,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 1,
                        forward: true,
                    },
                }),
            ],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };

        let stage =
            assemble_output_face_loops(&halfedges, &[0, 3], &[false, false, false], &[0, 1, 2, 3]);

        assert!(stage.loops.is_empty());
        assert_eq!(stage.incomplete_faces, 0);
        assert_eq!(stage.repeated_halfedges, 0);
        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.dropped_open_chain_halfedges, 3);
        assert_eq!(
            stage.dropped_open_chains[0].owner,
            Some(ExactBoolMeshDroppedOpenChainOwner {
                side: ExactBoolMeshSide::Left,
                source_face: 0,
            })
        );
        assert_eq!(
            stage.dropped_open_chains[0].source_kind,
            ExactBoolMeshDroppedOpenChainSourceKind::SourceEdge
        );
    }

    #[test]
    fn exact_degenerate_open_chain_is_replayed_as_lower_dimensional_drop() {
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![Some(ExactBoolMeshOutputHalfedge {
                tail: 0,
                head: 1,
                pair: 0,
                face: 0,
                source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                    side: ExactBoolMeshSide::Left,
                    source_face: 0,
                    opposite_face: 0,
                    fragment: 0,
                    forward: true,
                },
            })],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };

        let stage = assemble_output_face_loops(&halfedges, &[0, 1], &[true], &[0, 1]);

        assert!(stage.loops.is_empty());
        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.dropped_open_chain_halfedges, 1);
        assert_eq!(
            stage.dropped_open_chains[0].owner,
            Some(ExactBoolMeshDroppedOpenChainOwner {
                side: ExactBoolMeshSide::Left,
                source_face: 0,
            })
        );
        assert_eq!(
            stage.dropped_open_chains[0].source_kind,
            ExactBoolMeshDroppedOpenChainSourceKind::FacePair
        );
    }

    #[test]
    fn source_edge_seam_face_without_loops_is_replayed_as_lower_dimensional_drop() {
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 0,
                    head: 1,
                    pair: 10,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
                        side: ExactBoolMeshSide::Left,
                        source_halfedge: 0,
                        source_face: 0,
                        edge: [0, 1],
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 2,
                    head: 3,
                    pair: 11,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 0,
                        forward: true,
                    },
                }),
            ],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };

        let stage = assemble_output_face_loops(&halfedges, &[0, 2], &[false, false], &[0, 1, 2, 3]);

        assert!(stage.loops.is_empty());
        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.dropped_open_chain_halfedges, 2);
        assert!(stage.dropped_open_chains.iter().all(|chain| {
            chain.owner
                == Some(ExactBoolMeshDroppedOpenChainOwner {
                    side: ExactBoolMeshSide::Left,
                    source_face: 0,
                })
        }));
        assert_eq!(
            stage.dropped_open_chains[0].source_kind,
            ExactBoolMeshDroppedOpenChainSourceKind::SourceEdge
        );
        assert_eq!(
            stage.dropped_open_chains[1].source_kind,
            ExactBoolMeshDroppedOpenChainSourceKind::FacePair
        );
    }

    #[test]
    fn mixed_source_edge_open_chain_beside_loop_is_replayed_as_lower_dimensional_drop() {
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 0,
                    head: 1,
                    pair: 10,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge {
                        side: ExactBoolMeshSide::Left,
                        source_halfedge: 0,
                        source_face: 0,
                        edge: [0, 1],
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 1,
                    head: 2,
                    pair: 11,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
                        side: ExactBoolMeshSide::Left,
                        source_halfedge: 1,
                        source_face: 0,
                        edge: [1, 2],
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 3,
                    head: 4,
                    pair: 2,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 4,
                    head: 3,
                    pair: 3,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 1,
                        forward: true,
                    },
                }),
            ],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };

        let stage = assemble_output_face_loops(
            &halfedges,
            &[0, 4],
            &[false, false, false, false],
            &[0, 1, 2, 3, 4],
        );

        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.dropped_open_chain_halfedges, 2);
        assert_eq!(
            stage.dropped_open_chains[0].owner,
            Some(ExactBoolMeshDroppedOpenChainOwner {
                side: ExactBoolMeshSide::Left,
                source_face: 0,
            })
        );
        assert_eq!(
            stage.dropped_open_chains[0].source_kind,
            ExactBoolMeshDroppedOpenChainSourceKind::SourceEdge
        );
        assert_eq!(stage.loops.len(), 1);
        assert_eq!(stage.loops[0].halfedges, vec![2, 3]);
    }

    #[test]
    fn face_pair_only_open_face_remains_a_non_loop_blocker() {
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 0,
                    head: 1,
                    pair: 10,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 2,
                    head: 3,
                    pair: 11,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 1,
                        forward: true,
                    },
                }),
            ],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };

        let stage = assemble_output_face_loops(&halfedges, &[0, 2], &[false, false], &[0, 1, 2, 3]);

        assert!(stage.loops.is_empty());
        assert_eq!(stage.non_loop_halfedges, 2);
        assert_eq!(stage.dropped_open_chain_halfedges, 0);
    }

    #[test]
    fn new_face_pair_chain_can_close_by_exact_representative() {
        let halfedges = ExactBoolMeshHalfedgeAssemblyStage {
            output_halfedges: vec![
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 0,
                    head: 1,
                    pair: 0,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 0,
                        forward: true,
                    },
                }),
                Some(ExactBoolMeshOutputHalfedge {
                    tail: 2,
                    head: 3,
                    pair: 1,
                    face: 0,
                    source: ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                        side: ExactBoolMeshSide::Left,
                        source_face: 0,
                        opposite_face: 0,
                        fragment: 1,
                        forward: true,
                    },
                }),
            ],
            ..ExactBoolMeshHalfedgeAssemblyStage::default()
        };

        let stage = assemble_output_face_loops(&halfedges, &[0, 2], &[false, false], &[0, 1, 1, 0]);

        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.loops.len(), 1);
        assert_eq!(stage.loops[0].halfedges, vec![0, 1]);
        assert_eq!(stage.loops[0].vertices, vec![0, 2]);
    }
}
