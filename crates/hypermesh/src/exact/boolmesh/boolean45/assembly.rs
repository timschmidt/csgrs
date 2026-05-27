//! Exact halfedge emission for boolmesh `boolean45`.
//!
//! Legacy boolmesh writes partial source-edge fragments, new face-pair
//! fragments, and whole source edges into one `hs_r` array using per-face
//! cursors.  This module keeps that mutation order and makes every written
//! halfedge replayable.  Following Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997), incomplete upstream decisions remain
//! explicit unfilled slots rather than being patched by numerical guesses.

use super::super::{
    ExactBoolMeshHalfedgeAssemblyStage, ExactBoolMeshNewFacePairStage, ExactBoolMeshOutputHalfedge,
    ExactBoolMeshOutputHalfedgeSource, ExactBoolMeshPartialSourceEdgeStage, ExactBoolMeshSide,
    ExactBoolMeshWholeSourceEdgeStage,
};

/// Emit exact boolmesh output halfedge slots from staged fragments.
pub(super) fn assemble_output_halfedges(
    partial_source_edges: &ExactBoolMeshPartialSourceEdgeStage,
    new_face_pair_edges: &ExactBoolMeshNewFacePairStage,
    whole_source_edges: &ExactBoolMeshWholeSourceEdgeStage,
    source_face_to_output_face: &[Option<usize>],
    face_halfedge_offsets: &[usize],
    left_faces: usize,
) -> ExactBoolMeshHalfedgeAssemblyStage {
    let total_halfedges = face_halfedge_offsets.last().copied().unwrap_or(0);
    let output_face_count = face_halfedge_offsets.len().saturating_sub(1);
    let mut stage = ExactBoolMeshHalfedgeAssemblyStage {
        output_halfedges: vec![None; total_halfedges],
        face_write_offsets: face_halfedge_offsets
            .iter()
            .copied()
            .take(output_face_count)
            .collect(),
        ..ExactBoolMeshHalfedgeAssemblyStage::default()
    };

    append_partial_source_halfedges(
        partial_source_edges,
        source_face_to_output_face,
        face_halfedge_offsets,
        left_faces,
        &mut stage,
    );
    append_new_face_pair_halfedges(
        new_face_pair_edges,
        source_face_to_output_face,
        face_halfedge_offsets,
        left_faces,
        &mut stage,
    );
    append_whole_source_halfedges(
        whole_source_edges,
        source_face_to_output_face,
        face_halfedge_offsets,
        left_faces,
        &mut stage,
    );

    stage.unfilled_halfedges = stage
        .output_halfedges
        .iter()
        .filter(|halfedge| halfedge.is_none())
        .count();
    stage
}

fn append_partial_source_halfedges(
    partial_source_edges: &ExactBoolMeshPartialSourceEdgeStage,
    source_face_to_output_face: &[Option<usize>],
    face_halfedge_offsets: &[usize],
    left_faces: usize,
    stage: &mut ExactBoolMeshHalfedgeAssemblyStage,
) {
    for run in &partial_source_edges.source_edge_runs {
        let Some((&first_face, &first_edge)) =
            run.incident_faces.first().zip(run.incident_edges.first())
        else {
            stage.source_edge_incident_gaps += run.fragments.len();
            continue;
        };
        let second_face = run.incident_faces.get(1).copied();
        let edge = [run.tail, run.head];
        for (fragment_index, fragment) in run.fragments.iter().enumerate() {
            let Some((tail, head)) = oriented_fragment_endpoints(
                edge,
                first_edge,
                fragment.tail_point.output_vertex,
                fragment.head_point.output_vertex,
            ) else {
                stage.source_edge_incident_gaps += 1;
                continue;
            };
            if let Some(second_face) = second_face {
                emit_source_edge_pair(
                    run.side,
                    first_face,
                    second_face,
                    tail,
                    head,
                    edge,
                    fragment_index,
                    SourceEdgeEmissionKind::Partial,
                    source_face_to_output_face,
                    face_halfedge_offsets,
                    left_faces,
                    stage,
                );
            } else {
                emit_source_boundary_halfedge(
                    run.side,
                    first_face,
                    tail,
                    head,
                    edge,
                    fragment_index,
                    SourceEdgeEmissionKind::Partial,
                    source_face_to_output_face,
                    face_halfedge_offsets,
                    left_faces,
                    stage,
                );
            }
        }
    }
}

fn append_new_face_pair_halfedges(
    new_face_pair_edges: &ExactBoolMeshNewFacePairStage,
    source_face_to_output_face: &[Option<usize>],
    face_halfedge_offsets: &[usize],
    left_faces: usize,
    stage: &mut ExactBoolMeshHalfedgeAssemblyStage,
) {
    for run in &new_face_pair_edges.face_pair_runs {
        let left_source_face = run.face_pair.left_face;
        let right_source_face = left_faces + run.face_pair.right_face;
        for (fragment_index, fragment) in run.fragments.iter().enumerate() {
            let forward = ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                side: ExactBoolMeshSide::Left,
                source_face: run.face_pair.left_face,
                opposite_face: run.face_pair.right_face,
                fragment: fragment_index,
                forward: true,
            };
            let backward = ExactBoolMeshOutputHalfedgeSource::NewFacePair {
                side: ExactBoolMeshSide::Right,
                source_face: run.face_pair.right_face,
                opposite_face: run.face_pair.left_face,
                fragment: fragment_index,
                forward: false,
            };
            emit_halfedge_pair(
                left_source_face,
                right_source_face,
                fragment.tail_point.output_vertex,
                fragment.head_point.output_vertex,
                forward,
                backward,
                source_face_to_output_face,
                face_halfedge_offsets,
                stage,
            );
        }
    }
}

fn append_whole_source_halfedges(
    whole_source_edges: &ExactBoolMeshWholeSourceEdgeStage,
    source_face_to_output_face: &[Option<usize>],
    face_halfedge_offsets: &[usize],
    left_faces: usize,
    stage: &mut ExactBoolMeshHalfedgeAssemblyStage,
) {
    for run in &whole_source_edges.source_edge_runs {
        let Some((&first_face, &first_edge)) =
            run.incident_faces.first().zip(run.incident_edges.first())
        else {
            stage.source_edge_incident_gaps += run.fragments.len();
            continue;
        };
        let second_face = run.incident_faces.get(1).copied();
        let edge = if run.signed_count < 0 {
            [run.edge[1], run.edge[0]]
        } else {
            run.edge
        };
        for (fragment_index, fragment) in run.fragments.iter().enumerate() {
            let desired_edge = if run.signed_count < 0 {
                [first_edge[1], first_edge[0]]
            } else {
                first_edge
            };
            let Some((tail, head)) = oriented_fragment_endpoints(
                edge,
                desired_edge,
                fragment.output_tail,
                fragment.output_head,
            ) else {
                stage.source_edge_incident_gaps += 1;
                continue;
            };
            if let Some(second_face) = second_face {
                emit_source_edge_pair(
                    run.side,
                    first_face,
                    second_face,
                    tail,
                    head,
                    edge,
                    fragment_index,
                    SourceEdgeEmissionKind::Whole,
                    source_face_to_output_face,
                    face_halfedge_offsets,
                    left_faces,
                    stage,
                );
            } else {
                emit_source_boundary_halfedge(
                    run.side,
                    first_face,
                    tail,
                    head,
                    edge,
                    fragment_index,
                    SourceEdgeEmissionKind::Whole,
                    source_face_to_output_face,
                    face_halfedge_offsets,
                    left_faces,
                    stage,
                );
            }
        }
    }
}

fn oriented_fragment_endpoints(
    stored_edge: [usize; 2],
    desired_edge: [usize; 2],
    tail: usize,
    head: usize,
) -> Option<(usize, usize)> {
    // Boolmesh receives this orientation from its halfedge structure.  The
    // exact port replays the same combinatorial choice from retained directed
    // source-edge uses, which is the kind of topology/numeric separation Yap
    // requires in "Towards Exact Geometric Computation" (1997).
    if desired_edge == stored_edge {
        Some((tail, head))
    } else if desired_edge == [stored_edge[1], stored_edge[0]] {
        Some((head, tail))
    } else {
        None
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum SourceEdgeEmissionKind {
    Partial,
    Whole,
}

fn emit_source_edge_pair(
    side: ExactBoolMeshSide,
    first_face: usize,
    second_face: usize,
    tail: usize,
    head: usize,
    edge: [usize; 2],
    fragment: usize,
    kind: SourceEdgeEmissionKind,
    source_face_to_output_face: &[Option<usize>],
    face_halfedge_offsets: &[usize],
    left_faces: usize,
    stage: &mut ExactBoolMeshHalfedgeAssemblyStage,
) {
    let first_source_face = source_face_index(side, first_face, left_faces);
    let second_source_face = source_face_index(side, second_face, left_faces);
    let forward = source_edge_halfedge_source(side, first_face, edge, fragment, true, kind);
    let backward = source_edge_halfedge_source(side, second_face, edge, fragment, false, kind);
    emit_halfedge_pair(
        first_source_face,
        second_source_face,
        tail,
        head,
        forward,
        backward,
        source_face_to_output_face,
        face_halfedge_offsets,
        stage,
    );
}

fn emit_source_boundary_halfedge(
    side: ExactBoolMeshSide,
    face: usize,
    tail: usize,
    head: usize,
    edge: [usize; 2],
    fragment: usize,
    kind: SourceEdgeEmissionKind,
    source_face_to_output_face: &[Option<usize>],
    face_halfedge_offsets: &[usize],
    left_faces: usize,
    stage: &mut ExactBoolMeshHalfedgeAssemblyStage,
) {
    // Legacy boolmesh obtains boundary behavior from source halfedge topology:
    // an open mesh edge has one incident face instead of a reciprocal pair.
    // The exact port records that one-sided combinatorial fact directly for
    // both split (`append_partial_edges`) and untouched (`append_whole_edges`)
    // source edges.  This follows Yap, "Towards Exact Geometric Computation,"
    // Comput. Geom. 7.1-2 (1997): topology evidence is retained as object
    // state, not recovered later from rounded coordinates or epsilon pairing.
    let source_face = source_face_index(side, face, left_faces);
    let Some(Some(output_face)) = source_face_to_output_face.get(source_face) else {
        stage.missing_source_face_maps += 1;
        return;
    };
    let Some(slot) = allocate_face_slot(*output_face, face_halfedge_offsets, stage) else {
        return;
    };
    stage.output_halfedges[slot] = Some(ExactBoolMeshOutputHalfedge {
        tail,
        head,
        pair: slot,
        face: *output_face,
        source: source_edge_halfedge_source(side, face, edge, fragment, true, kind),
    });
    stage.emitted_boundary_halfedges += 1;
}

fn source_edge_halfedge_source(
    side: ExactBoolMeshSide,
    source_face: usize,
    edge: [usize; 2],
    fragment: usize,
    forward: bool,
    kind: SourceEdgeEmissionKind,
) -> ExactBoolMeshOutputHalfedgeSource {
    match kind {
        SourceEdgeEmissionKind::Partial => ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge {
            side,
            source_face,
            edge,
            fragment,
            forward,
        },
        SourceEdgeEmissionKind::Whole => ExactBoolMeshOutputHalfedgeSource::WholeSourceEdge {
            side,
            source_face,
            edge,
            fragment,
            forward,
        },
    }
}

fn emit_halfedge_pair(
    first_source_face: usize,
    second_source_face: usize,
    tail: usize,
    head: usize,
    forward_source: ExactBoolMeshOutputHalfedgeSource,
    backward_source: ExactBoolMeshOutputHalfedgeSource,
    source_face_to_output_face: &[Option<usize>],
    face_halfedge_offsets: &[usize],
    stage: &mut ExactBoolMeshHalfedgeAssemblyStage,
) {
    let Some(Some(first_output_face)) = source_face_to_output_face.get(first_source_face) else {
        stage.missing_source_face_maps += 1;
        return;
    };
    let Some(Some(second_output_face)) = source_face_to_output_face.get(second_source_face) else {
        stage.missing_source_face_maps += 1;
        return;
    };
    let Some(first_slot) = allocate_face_slot(*first_output_face, face_halfedge_offsets, stage)
    else {
        return;
    };
    let Some(second_slot) = allocate_face_slot(*second_output_face, face_halfedge_offsets, stage)
    else {
        stage.output_halfedges[first_slot] = None;
        return;
    };

    stage.output_halfedges[first_slot] = Some(ExactBoolMeshOutputHalfedge {
        tail,
        head,
        pair: second_slot,
        face: *first_output_face,
        source: forward_source,
    });
    stage.output_halfedges[second_slot] = Some(ExactBoolMeshOutputHalfedge {
        tail: head,
        head: tail,
        pair: first_slot,
        face: *second_output_face,
        source: backward_source,
    });
    stage.emitted_pairs += 1;
}

fn allocate_face_slot(
    output_face: usize,
    face_halfedge_offsets: &[usize],
    stage: &mut ExactBoolMeshHalfedgeAssemblyStage,
) -> Option<usize> {
    let Some(cursor) = stage.face_write_offsets.get_mut(output_face) else {
        stage.face_overflows += 1;
        return None;
    };
    let Some(limit) = face_halfedge_offsets.get(output_face + 1).copied() else {
        stage.face_overflows += 1;
        return None;
    };
    if *cursor >= limit {
        stage.face_overflows += 1;
        return None;
    }
    let slot = *cursor;
    *cursor += 1;
    Some(slot)
}

fn source_face_index(side: ExactBoolMeshSide, face: usize, left_faces: usize) -> usize {
    match side {
        ExactBoolMeshSide::Left => face,
        ExactBoolMeshSide::Right => left_faces + face,
    }
}

#[cfg(test)]
mod tests {
    use super::super::super::{
        ExactBoolMeshNewFacePairStage, ExactBoolMeshOutputHalfedgeSource,
        ExactBoolMeshPartialEdgePoint, ExactBoolMeshPartialEdgePointOrigin,
        ExactBoolMeshPartialSourceEdgeFragment, ExactBoolMeshPartialSourceEdgeRun,
        ExactBoolMeshPartialSourceEdgeStage, ExactBoolMeshSide, ExactBoolMeshSourceVertex,
        ExactBoolMeshWholeSourceEdgeStage,
    };
    use super::*;

    #[test]
    fn partial_one_incident_run_emits_boundary_halfedge() {
        let source = ExactBoolMeshSourceVertex {
            side: ExactBoolMeshSide::Left,
            vertex: 0,
        };
        let tail_point = ExactBoolMeshPartialEdgePoint {
            output_vertex: 0,
            is_tail: true,
            order_index: 0,
            collision: usize::MAX,
            origin: ExactBoolMeshPartialEdgePointOrigin::RetainedEndpoint { source, copy: 0 },
        };
        let head_point = ExactBoolMeshPartialEdgePoint {
            output_vertex: 1,
            is_tail: false,
            order_index: usize::MAX,
            collision: usize::MAX,
            origin: ExactBoolMeshPartialEdgePointOrigin::RetainedEndpoint {
                source: ExactBoolMeshSourceVertex {
                    side: ExactBoolMeshSide::Left,
                    vertex: 1,
                },
                copy: 0,
            },
        };
        let partial_source_edges = ExactBoolMeshPartialSourceEdgeStage {
            source_edge_runs: vec![ExactBoolMeshPartialSourceEdgeRun {
                side: ExactBoolMeshSide::Left,
                tail: 0,
                head: 1,
                incident_faces: vec![0],
                incident_edges: vec![[0, 1]],
                points: vec![tail_point, head_point],
                fragments: vec![ExactBoolMeshPartialSourceEdgeFragment {
                    tail_point,
                    head_point,
                }],
                unpaired_points: 0,
            }],
            unpaired_runs: 0,
            missing_parameter_orders: 0,
        };

        let stage = assemble_output_halfedges(
            &partial_source_edges,
            &ExactBoolMeshNewFacePairStage::default(),
            &ExactBoolMeshWholeSourceEdgeStage::default(),
            &[Some(0)],
            &[0, 1],
            1,
        );

        assert_eq!(stage.emitted_pairs, 0);
        assert_eq!(stage.emitted_boundary_halfedges, 1);
        assert_eq!(stage.source_edge_incident_gaps, 0);
        assert_eq!(stage.unfilled_halfedges, 0);
        let halfedge = stage.output_halfedges[0].as_ref().unwrap();
        assert_eq!(halfedge.tail, 0);
        assert_eq!(halfedge.head, 1);
        assert_eq!(halfedge.pair, 0);
        assert_eq!(halfedge.face, 0);
        assert_eq!(
            halfedge.source,
            ExactBoolMeshOutputHalfedgeSource::PartialSourceEdge {
                side: ExactBoolMeshSide::Left,
                source_face: 0,
                edge: [0, 1],
                fragment: 0,
                forward: true,
            }
        );

        assert_eq!(source.vertex, 0);
    }
}
