//! Exact output face-loop assembly for boolmesh `boolean45`.
//!
//! Legacy triangulation first groups each output face's halfedges by tail
//! vertex, then walks from the current head to the next halfedge with that
//! tail.  This is the topological face-boundary step before triangulation.
//! Following Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997), this exact port records incomplete and malformed
//! traversal states instead of using rounded geometry or panic-driven repair.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use super::super::{
    ExactBoolMeshFaceLoopAssemblyStage, ExactBoolMeshHalfedgeAssemblyStage,
    ExactBoolMeshOutputFaceLoop,
};

/// Assemble per-face closed boundary loops from emitted output halfedges.
pub(super) fn assemble_output_face_loops(
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_halfedge_offsets: &[usize],
) -> ExactBoolMeshFaceLoopAssemblyStage {
    let mut stage = ExactBoolMeshFaceLoopAssemblyStage::default();
    for output_face in 0..face_halfedge_offsets.len().saturating_sub(1) {
        assemble_output_face_loop(output_face, halfedges, face_halfedge_offsets, &mut stage);
    }
    stage
}

fn assemble_output_face_loop(
    output_face: usize,
    halfedges: &ExactBoolMeshHalfedgeAssemblyStage,
    face_halfedge_offsets: &[usize],
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
                stage.non_loop_halfedges += loop_halfedges.len();
                break;
            };
            current = next;
        }
    }

    let expected = end - begin;
    if consumed.len() < expected {
        stage.non_loop_halfedges += expected - consumed.len();
    }
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
