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
    ExactBoolMeshFaceLoopAssemblyStage, ExactBoolMeshHalfedgeAssemblyStage,
    ExactBoolMeshOutputFaceLoop, ExactBoolMeshOutputHalfedgeSource,
};

/// Assemble per-face closed boundary loops from emitted output halfedges.
pub(super) fn assemble_output_face_loops(
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_halfedge_offsets: &[usize],
    exact_degenerate_halfedges: &[bool],
) -> ExactBoolMeshFaceLoopAssemblyStage {
    let mut stage = ExactBoolMeshFaceLoopAssemblyStage::default();
    for output_face in 0..face_halfedge_offsets.len().saturating_sub(1) {
        assemble_output_face_loop(
            output_face,
            halfedges,
            face_halfedge_offsets,
            exact_degenerate_halfedges,
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

            if halfedge.head == loop_vertices[0] {
                stage.loops.push(ExactBoolMeshOutputFaceLoop {
                    output_face,
                    halfedges: loop_halfedges,
                    vertices: loop_vertices,
                });
                break;
            }

            let Some(next) = pop_next_for_tail(&mut tail_to_halfedges, halfedge.head) else {
                if halfedge_chain_is_partial_source_edge(halfedges, &loop_halfedges)
                    || halfedge_chain_is_exact_degenerate(
                        exact_degenerate_halfedges,
                        &loop_halfedges,
                    )
                {
                    stage.dropped_open_chain_halfedges += loop_halfedges.len();
                } else {
                    stage.non_loop_halfedges += loop_halfedges.len();
                }
                break;
            };
            current = next;
        }
    }

    let expected = end - begin;
    if consumed.len() < expected {
        stage.non_loop_halfedges += expected - consumed.len();
    }
    if stage.loops.len() == loops_before
        && stage.repeated_halfedges == repeated_before
        && stage.non_loop_halfedges > non_loop_before
        && (face_has_boundary_halfedge(output_face, halfedges, begin, end)
            || face_has_only_partial_source_edge_halfedges(output_face, halfedges, begin, end))
    {
        let dropped = stage.non_loop_halfedges - non_loop_before;
        stage.non_loop_halfedges = non_loop_before;
        stage.dropped_open_chain_halfedges += dropped;
    }
}

fn halfedge_chain_is_partial_source_edge(
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

        let stage = assemble_output_face_loops(&halfedges, &[0, 2], &[false, false]);

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

        let stage = assemble_output_face_loops(&halfedges, &[0, 3], &[false, false, false]);

        assert!(stage.loops.is_empty());
        assert_eq!(stage.incomplete_faces, 0);
        assert_eq!(stage.repeated_halfedges, 0);
        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.dropped_open_chain_halfedges, 3);
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

        let stage = assemble_output_face_loops(&halfedges, &[0, 1], &[true]);

        assert!(stage.loops.is_empty());
        assert_eq!(stage.non_loop_halfedges, 0);
        assert_eq!(stage.dropped_open_chain_halfedges, 1);
    }
}
