//! Exact staging for boolmesh `boolean45` edge-event pairing.
//!
//! Legacy `boolean45::pair_up` partitions edge events into tails and heads,
//! sorts each side by `EdgePt.val`, and zips the sorted halves into partial
//! halfedges.  This exact port keeps that algorithmic shape but orders by the
//! retained edge parameter from `kernel12` rather than a rounded dot product.
//! Yap, "Towards Exact Geometric Computation," *Computational Geometry*
//! 7.1-2 (1997), is the rule here: the pairing decision consumes certified
//! construction parameters and remains a replayable staging artifact before
//! final topology mutation.  The boundary-fragment pairing model follows
//! Weiler and Atherton, "Hidden Surface Removal Using Polygon Area Sorting,"
//! *SIGGRAPH* (1977).

use std::cmp::Ordering;
use std::collections::BTreeMap;

use hyperlimit::compare_reals;

use crate::exact::boolean::ExactBooleanOperation;
use crate::exact::mesh::{ExactMesh, Triangle};

use super::{
    ExactBoolMeshBoolean03, ExactBoolMeshBoolean45Stage, ExactBoolMeshEdgeEvent,
    ExactBoolMeshEdgeFacePair, ExactBoolMeshOutputVertexAllocation,
    ExactBoolMeshOutputVertexOrigin, ExactBoolMeshPairUpStage, ExactBoolMeshPairedEdgeFragment,
    ExactBoolMeshSide, ExactBoolMeshSourceEdgeRun, ExactBoolMeshSourceVertex,
};

/// Build the exact `boolean45::size_output` staging record.
///
/// This is a direct structural port of legacy boolmesh `size_output`: first
/// compute operation-signed `i03`/`i30` retained-vertex counters from `w03` and
/// `w30`, then count `i12`/`i21` crossing vertices onto both incident source
/// faces and the opposite triangle face.  The only intentional difference is
/// that source-edge adjacency is recovered from exact source triangles instead
/// of indexing the primitive halfedge array.
///
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997), motivates keeping this as replayable integer topology
/// staging rather than constructing coordinates during sizing.  The counting
/// semantics themselves follow the boolmesh `boolean45::size_output` kernel.
/// The vertex allocation below is the exact counterpart of boolmesh's
/// `exclusive_scan` ranges and `duplicate_verts` calls; exact coordinate
/// construction is deferred, which is the separation between topology and
/// numeric objects required by Yap's exact-computation paradigm.
pub(super) fn size_output_stage(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    operation: ExactBooleanOperation,
) -> ExactBoolMeshBoolean45Stage {
    let (left_base, right_base, crossing_sign) = operation_coefficients(operation);
    let i03 = boolean03
        .w03
        .iter()
        .map(|winding| left_base + crossing_sign * winding)
        .collect::<Vec<_>>();
    let i30 = boolean03
        .w30
        .iter()
        .map(|winding| right_base + crossing_sign * winding)
        .collect::<Vec<_>>();
    let i12 = boolean03
        .x12
        .iter()
        .map(|crossing| crossing_sign * crossing)
        .collect::<Vec<_>>();
    let i21 = boolean03
        .x21
        .iter()
        .map(|crossing| crossing_sign * crossing)
        .collect::<Vec<_>>();
    let vertex_allocation = allocate_output_vertices(&i03, &i30, &i12, &i21);

    let mut left_face_halfedge_counts = retained_vertex_counts(left.triangles(), &i03);
    let mut right_face_halfedge_counts = retained_vertex_counts(right.triangles(), &i30);
    let mut source_edge_incident_gaps = 0;

    for (pair, signed_count) in boolean03.p1q2.iter().zip(i12.iter()) {
        source_edge_incident_gaps += count_crossing_vertex(
            pair,
            signed_abs(*signed_count),
            left,
            &mut left_face_halfedge_counts,
            &mut right_face_halfedge_counts,
        );
    }
    for (pair, signed_count) in boolean03.p2q1.iter().zip(i21.iter()) {
        source_edge_incident_gaps += count_crossing_vertex(
            pair,
            signed_abs(*signed_count),
            right,
            &mut right_face_halfedge_counts,
            &mut left_face_halfedge_counts,
        );
    }

    let source_face_counts = left_face_halfedge_counts
        .iter()
        .chain(right_face_halfedge_counts.iter())
        .copied()
        .collect::<Vec<_>>();
    let mut source_face_to_output_face = vec![None; source_face_counts.len()];
    let mut face_halfedge_offsets = vec![0];
    let mut output_face = 0;
    let mut halfedge_sum = 0;
    for (source_face, count) in source_face_counts.iter().enumerate() {
        if *count > 0 {
            source_face_to_output_face[source_face] = Some(output_face);
            output_face += 1;
            halfedge_sum += *count;
            face_halfedge_offsets.push(halfedge_sum);
        }
    }

    ExactBoolMeshBoolean45Stage {
        left_face_halfedge_counts,
        right_face_halfedge_counts,
        face_halfedge_offsets,
        source_face_to_output_face,
        vertex_allocation,
        vertices_from_left: i03.iter().map(|value| signed_abs(*value)).sum(),
        vertices_from_right: i30.iter().map(|value| signed_abs(*value)).sum(),
        inserted_intersection_vertices: i12
            .iter()
            .chain(i21.iter())
            .map(|value| signed_abs(*value))
            .sum(),
        source_edge_incident_gaps,
    }
}

/// Pair lowered source-edge events with exact parameter ordering.
///
/// This is the exact counterpart to the source-edge `pt_old` path in
/// `boolean45::append_partial_edges`.  It deliberately does not synthesize
/// retained source endpoints from `kernel03` yet; runs that need those
/// endpoints keep explicit unpaired counts so the later winding slice has a
/// checked place to attach them.
pub(super) fn pair_source_edge_events(
    events: Vec<ExactBoolMeshEdgeEvent>,
) -> ExactBoolMeshPairUpStage {
    let mut grouped = BTreeMap::<(u8, usize, usize), Vec<ExactBoolMeshEdgeEvent>>::new();
    for event in events {
        grouped
            .entry((side_key(event.side), event.tail, event.head))
            .or_default()
            .push(event);
    }

    let mut unknown_orderings = 0;
    let mut unpaired_event_runs = 0;
    let mut source_edge_runs = Vec::new();
    for ((_side_key, tail, head), mut events) in grouped {
        unknown_orderings += sort_events(&mut events);
        let side = events
            .first()
            .map(|event| event.side)
            .unwrap_or(ExactBoolMeshSide::Left);
        let mut tails = events
            .iter()
            .filter(|event| event.is_tail)
            .cloned()
            .collect::<Vec<_>>();
        let heads = events
            .iter()
            .filter(|event| !event.is_tail)
            .cloned()
            .collect::<Vec<_>>();
        let pair_count = tails.len().min(heads.len());
        let unpaired_events = tails.len().abs_diff(heads.len());
        if unpaired_events > 0 {
            unpaired_event_runs += 1;
        }
        let fragments = tails
            .drain(..pair_count)
            .zip(heads.into_iter().take(pair_count))
            .map(|(tail_event, head_event)| ExactBoolMeshPairedEdgeFragment {
                side,
                tail,
                head,
                tail_event,
                head_event,
            })
            .collect();
        source_edge_runs.push(ExactBoolMeshSourceEdgeRun {
            side,
            tail,
            head,
            events,
            fragments,
            unpaired_events,
        });
    }

    ExactBoolMeshPairUpStage {
        source_edge_runs,
        unknown_orderings,
        unpaired_event_runs,
    }
}

fn sort_events(events: &mut [ExactBoolMeshEdgeEvent]) -> usize {
    let mut unknown_orderings = 0;
    events.sort_by(
        |left, right| match compare_reals(&left.parameter, &right.parameter).value() {
            Some(Ordering::Equal) => left.collision.cmp(&right.collision),
            Some(ordering) => ordering,
            None => {
                unknown_orderings += 1;
                left.collision.cmp(&right.collision)
            }
        },
    );
    unknown_orderings
}

fn side_key(side: ExactBoolMeshSide) -> u8 {
    match side {
        ExactBoolMeshSide::Left => 0,
        ExactBoolMeshSide::Right => 1,
    }
}

fn operation_coefficients(operation: ExactBooleanOperation) -> (i32, i32, i32) {
    match operation {
        ExactBooleanOperation::Union => (1, 1, -1),
        ExactBooleanOperation::Intersection => (0, 0, 1),
        ExactBooleanOperation::Difference => (1, 0, -1),
        ExactBooleanOperation::SelectedRegions(_) => (0, 0, 1),
    }
}

fn retained_vertex_counts(triangles: &[Triangle], vertex_counts: &[i32]) -> Vec<usize> {
    triangles
        .iter()
        .map(|triangle| {
            triangle
                .0
                .iter()
                .filter_map(|vertex| vertex_counts.get(*vertex))
                .map(|count| signed_abs(*count))
                .sum()
        })
        .collect()
}

fn allocate_output_vertices(
    i03: &[i32],
    i30: &[i32],
    i12: &[i32],
    i21: &[i32],
) -> ExactBoolMeshOutputVertexAllocation {
    let mut output_vertex_origins = Vec::new();
    let left_vertex_output_starts =
        allocate_source_vertices(ExactBoolMeshSide::Left, i03, &mut output_vertex_origins);
    let right_vertex_output_starts =
        allocate_source_vertices(ExactBoolMeshSide::Right, i30, &mut output_vertex_origins);
    let p1q2_output_starts = allocate_kernel12_vertices(
        i12,
        |event, copy| ExactBoolMeshOutputVertexOrigin::Kernel12LeftEdgeRightFace { event, copy },
        &mut output_vertex_origins,
    );
    let p2q1_output_starts = allocate_kernel12_vertices(
        i21,
        |event, copy| ExactBoolMeshOutputVertexOrigin::Kernel12RightEdgeLeftFace { event, copy },
        &mut output_vertex_origins,
    );

    ExactBoolMeshOutputVertexAllocation {
        left_vertex_output_starts,
        right_vertex_output_starts,
        p1q2_output_starts,
        p2q1_output_starts,
        output_vertex_origins,
    }
}

fn allocate_source_vertices(
    side: ExactBoolMeshSide,
    signed_counts: &[i32],
    output_vertex_origins: &mut Vec<ExactBoolMeshOutputVertexOrigin>,
) -> Vec<Option<usize>> {
    signed_counts
        .iter()
        .enumerate()
        .map(|(vertex, signed_count)| {
            let count = signed_abs(*signed_count);
            if count == 0 {
                return None;
            }
            let start = output_vertex_origins.len();
            for copy in 0..count {
                output_vertex_origins.push(ExactBoolMeshOutputVertexOrigin::SourceVertex {
                    source: ExactBoolMeshSourceVertex { side, vertex },
                    copy,
                });
            }
            Some(start)
        })
        .collect()
}

fn allocate_kernel12_vertices<F>(
    signed_counts: &[i32],
    origin: F,
    output_vertex_origins: &mut Vec<ExactBoolMeshOutputVertexOrigin>,
) -> Vec<Option<usize>>
where
    F: Fn(usize, usize) -> ExactBoolMeshOutputVertexOrigin,
{
    signed_counts
        .iter()
        .enumerate()
        .map(|(event, signed_count)| {
            let count = signed_abs(*signed_count);
            if count == 0 {
                return None;
            }
            let start = output_vertex_origins.len();
            for copy in 0..count {
                output_vertex_origins.push(origin(event, copy));
            }
            Some(start)
        })
        .collect()
}

fn count_crossing_vertex(
    pair: &ExactBoolMeshEdgeFacePair,
    increment: usize,
    edge_mesh: &ExactMesh,
    edge_face_counts: &mut [usize],
    opposite_face_counts: &mut [usize],
) -> usize {
    let incident_faces = incident_faces_for_edge(edge_mesh.triangles(), pair.edge);
    for face in &incident_faces {
        if let Some(count) = edge_face_counts.get_mut(*face) {
            *count += increment;
        }
    }
    if let Some(count) = opposite_face_counts.get_mut(pair.face) {
        *count += increment;
    }

    usize::from(incident_faces.len() != 2)
}

fn incident_faces_for_edge(triangles: &[Triangle], edge: [usize; 2]) -> Vec<usize> {
    let key = canonical_edge(edge);
    triangles
        .iter()
        .enumerate()
        .filter_map(|(face, triangle)| {
            triangle_edges(*triangle)
                .iter()
                .any(|candidate| canonical_edge(*candidate) == key)
                .then_some(face)
        })
        .collect()
}

fn triangle_edges(triangle: Triangle) -> [[usize; 2]; 3] {
    [
        [triangle.0[0], triangle.0[1]],
        [triangle.0[1], triangle.0[2]],
        [triangle.0[2], triangle.0[0]],
    ]
}

fn canonical_edge(edge: [usize; 2]) -> [usize; 2] {
    if edge[0] <= edge[1] {
        edge
    } else {
        [edge[1], edge[0]]
    }
}

fn signed_abs(value: i32) -> usize {
    value.unsigned_abs() as usize
}
