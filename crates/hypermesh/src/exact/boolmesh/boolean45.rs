//! Exact staging for boolmesh `boolean45` edge-event pairing.
//!
//! Legacy `boolean45::pair_up` partitions edge events into tails and heads,
//! sorts each side by `EdgePt.val`, and zips the sorted halves into partial
//! halfedges.  Boolmesh keys the old-source-edge buckets by `hid_p`; this
//! exact port keeps that row id and orders by the retained edge parameter from
//! `kernel12` rather than a rounded dot product.  Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997), is the rule
//! here: the pairing decision consumes certified construction parameters and
//! combinatorial row identity before final topology mutation.  The
//! boundary-fragment pairing model follows Weiler and Atherton, "Hidden
//! Surface Removal Using Polygon Area Sorting," *SIGGRAPH* (1977).

mod assembly;
mod export;
mod face_loops;
mod geometry;
mod output_triangles;
mod triangulation;

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::{Point3, compare_reals};

use assembly::assemble_output_halfedges;
use export::stage_mesh_export;
use face_loops::assemble_output_face_loops;
use geometry::output_vertex_point;
use output_triangles::materialize_output_triangles;
use triangulation::triangulate_output_face_loops;

use crate::exact::boolean::ExactBooleanOperation;
use crate::exact::mesh::{ExactMesh, Triangle};
use crate::exact::scalar::ExactReal;

use super::{
    ExactBoolMeshBoolean03, ExactBoolMeshBoolean45Stage, ExactBoolMeshEdgeEvent,
    ExactBoolMeshEdgeFacePair, ExactBoolMeshFacePair, ExactBoolMeshFacePairPointRun,
    ExactBoolMeshNewEdgeVertexStage, ExactBoolMeshNewFacePairFragment, ExactBoolMeshNewFacePairRun,
    ExactBoolMeshNewFacePairStage, ExactBoolMeshOutputVertexAllocation,
    ExactBoolMeshOutputVertexOrigin, ExactBoolMeshPairUpStage, ExactBoolMeshPairedEdgeFragment,
    ExactBoolMeshPartialEdgePoint, ExactBoolMeshPartialEdgePointOrigin,
    ExactBoolMeshPartialSourceEdgeFragment, ExactBoolMeshPartialSourceEdgeRun,
    ExactBoolMeshPartialSourceEdgeStage, ExactBoolMeshRoutedEdgePoint, ExactBoolMeshSide,
    ExactBoolMeshSourceEdgePointRun, ExactBoolMeshSourceEdgeRun, ExactBoolMeshSourceVertex,
    ExactBoolMeshWholeSourceEdgeFragment, ExactBoolMeshWholeSourceEdgeRun,
    ExactBoolMeshWholeSourceEdgeStage,
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
/// The new-edge routing is the exact counterpart to `add_new_edge_verts`:
/// every allocated crossing vertex is placed into one source-edge bucket and
/// two left/right face-pair buckets before later pairing/emission stages.
/// Partial source-edge staging then mirrors `append_partial_edges`: retained
/// endpoints from `i03`/`i30` are appended to each touched source-edge bucket,
/// crossings are ordered by the exact parameter order produced by
/// `pair_up`, and tail/head lists are zipped into source-edge fragments.
/// New face-pair staging mirrors `append_new_edges`: each `pt_new` bucket is
/// ordered on the longest exact coordinate span of its output points, then
/// partitioned into tail/head sides and zipped into fragments.
/// Whole source-edge staging mirrors `append_whole_edges`: untouched retained
/// source edges are copied with operation-signed orientation and exact output
/// vertex ids.
pub(super) fn size_output_stage(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    operation: ExactBooleanOperation,
    pair_up: &ExactBoolMeshPairUpStage,
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
    let new_edge_vertices = route_new_edge_vertices(
        left,
        right,
        boolean03,
        &vertex_allocation,
        &i03,
        &i30,
        &i12,
        &i21,
    );
    let partial_source_edges = stage_partial_source_edges(
        left,
        right,
        &vertex_allocation,
        &new_edge_vertices,
        &i03,
        &i30,
        pair_up,
    );
    let new_face_pair_edges = stage_new_face_pair_edges(
        left,
        right,
        boolean03,
        &vertex_allocation,
        &new_edge_vertices,
    );
    let whole_source_edges = stage_whole_source_edges(
        left,
        right,
        &vertex_allocation,
        &i03,
        &i30,
        &partial_source_edges,
    );

    let mut left_face_halfedge_counts = retained_vertex_counts(left.triangles(), &i03);
    let mut right_face_halfedge_counts = retained_vertex_counts(right.triangles(), &i30);
    let mut source_edge_incident_gaps = 0;

    for (event, (pair, signed_count)) in boolean03.p1q2.iter().zip(i12.iter()).enumerate() {
        let source_parameter = source_edge_parameter(left, pair.edge, &boolean03.v12[event]);
        let suppress_opposite_face_count = source_tail_face_pair_owned_by_source_edge(
            pair,
            *signed_count < 0,
            &i03,
            source_parameter.as_ref(),
        );
        source_edge_incident_gaps += count_crossing_vertex(
            pair,
            signed_abs(*signed_count),
            left,
            &mut left_face_halfedge_counts,
            &mut right_face_halfedge_counts,
            !suppress_opposite_face_count,
        );
    }
    for (event, (pair, signed_count)) in boolean03.p2q1.iter().zip(i21.iter()).enumerate() {
        let source_parameter = source_edge_parameter(right, pair.edge, &boolean03.v21[event]);
        let suppress_opposite_face_count = source_tail_face_pair_owned_by_source_edge(
            pair,
            *signed_count < 0,
            &i30,
            source_parameter.as_ref(),
        );
        source_edge_incident_gaps += count_crossing_vertex(
            pair,
            signed_abs(*signed_count),
            right,
            &mut right_face_halfedge_counts,
            &mut left_face_halfedge_counts,
            !suppress_opposite_face_count,
        );
    }
    apply_suppressed_retained_tail_face_counts(
        &partial_source_edges,
        &mut left_face_halfedge_counts,
        &mut right_face_halfedge_counts,
    );

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
    let halfedge_assembly = assemble_output_halfedges(
        &partial_source_edges,
        &new_face_pair_edges,
        &whole_source_edges,
        &source_face_to_output_face,
        &face_halfedge_offsets,
        left.triangles().len(),
    );
    let face_loop_assembly = assemble_output_face_loops(&halfedge_assembly, &face_halfedge_offsets);
    let loop_triangulation = triangulate_output_face_loops(
        left,
        right,
        boolean03,
        &vertex_allocation,
        &halfedge_assembly,
        &face_loop_assembly,
    );
    let output_triangles = materialize_output_triangles(&loop_triangulation);
    let mesh_export = stage_mesh_export(
        left,
        right,
        boolean03,
        &vertex_allocation,
        &output_triangles,
    );

    ExactBoolMeshBoolean45Stage {
        left_face_halfedge_counts,
        right_face_halfedge_counts,
        face_halfedge_offsets,
        source_face_to_output_face,
        vertex_allocation,
        new_edge_vertices,
        partial_source_edges,
        new_face_pair_edges,
        whole_source_edges,
        halfedge_assembly,
        face_loop_assembly,
        loop_triangulation,
        output_triangles,
        mesh_export,
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
/// `boolean45::append_partial_edges`: events are bucketed by boolmesh's
/// source halfedge row (`hid_p`), not by reconstructed endpoints.  It
/// deliberately does not synthesize retained source endpoints from `kernel03`
/// yet; runs that need those endpoints keep explicit unpaired counts so the
/// later winding slice has a checked place to attach them.
pub(super) fn pair_source_edge_events(
    events: Vec<ExactBoolMeshEdgeEvent>,
) -> ExactBoolMeshPairUpStage {
    let mut grouped = BTreeMap::<(u8, usize), Vec<ExactBoolMeshEdgeEvent>>::new();
    for event in events {
        grouped
            .entry((side_key(event.side), event.source_halfedge))
            .or_default()
            .push(event);
    }

    let mut unknown_orderings = 0;
    let mut unpaired_event_runs = 0;
    let mut source_edge_runs = Vec::new();
    for ((_side_key, source_halfedge), mut events) in grouped {
        unknown_orderings += sort_events(&mut events);
        let side = events
            .first()
            .map(|event| event.side)
            .unwrap_or(ExactBoolMeshSide::Left);
        let [tail, head] = events
            .first()
            .map(|event| [event.tail, event.head])
            .unwrap_or([0, 0]);
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
                source_halfedge,
                tail,
                head,
                tail_event,
                head_event,
            })
            .collect();
        source_edge_runs.push(ExactBoolMeshSourceEdgeRun {
            side,
            source_halfedge,
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

fn side_from_key(side: u8) -> ExactBoolMeshSide {
    match side {
        0 => ExactBoolMeshSide::Left,
        _ => ExactBoolMeshSide::Right,
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

fn route_new_edge_vertices(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    i03: &[i32],
    i30: &[i32],
    i12: &[i32],
    i21: &[i32],
) -> ExactBoolMeshNewEdgeVertexStage {
    let mut source_edge_runs = BTreeMap::<(u8, usize), SourceEdgePointBucket>::new();
    let mut face_pair_runs = BTreeMap::<(usize, usize), Vec<ExactBoolMeshRoutedEdgePoint>>::new();
    let mut missing_source_edge_adjacencies = 0;
    let mut suppressed_source_tail_face_pair_points = 0;

    for (event, (pair, signed_count)) in boolean03.p1q2.iter().zip(i12.iter()).enumerate() {
        let source_parameter = source_edge_parameter(left, pair.edge, &boolean03.v12[event]);
        let route = route_crossing_vertices(
            pair,
            *signed_count,
            allocation.p1q2_output_starts[event],
            event,
            left,
            i03,
            source_parameter.as_ref(),
            true,
            &allocation.output_vertex_origins,
            &mut source_edge_runs,
            &mut face_pair_runs,
        );
        missing_source_edge_adjacencies += route.missing_source_edge_adjacencies;
        suppressed_source_tail_face_pair_points += route.suppressed_source_tail_face_pair_points;
    }
    let collision_offset = boolean03.p1q2.len();
    for (event, (pair, signed_count)) in boolean03.p2q1.iter().zip(i21.iter()).enumerate() {
        let source_parameter = source_edge_parameter(right, pair.edge, &boolean03.v21[event]);
        let route = route_crossing_vertices(
            pair,
            *signed_count,
            allocation.p2q1_output_starts[event],
            collision_offset + event,
            right,
            i30,
            source_parameter.as_ref(),
            false,
            &allocation.output_vertex_origins,
            &mut source_edge_runs,
            &mut face_pair_runs,
        );
        missing_source_edge_adjacencies += route.missing_source_edge_adjacencies;
        suppressed_source_tail_face_pair_points += route.suppressed_source_tail_face_pair_points;
    }

    ExactBoolMeshNewEdgeVertexStage {
        source_edge_runs: source_edge_runs
            .into_iter()
            .map(
                |((side_key, source_halfedge), bucket)| ExactBoolMeshSourceEdgePointRun {
                    side: side_from_key(side_key),
                    source_halfedge,
                    tail: bucket.tail,
                    head: bucket.head,
                    points: bucket.points,
                },
            )
            .collect(),
        face_pair_runs: face_pair_runs
            .into_iter()
            .map(
                |((left_face, right_face), points)| ExactBoolMeshFacePairPointRun {
                    face_pair: ExactBoolMeshFacePair {
                        left_face,
                        right_face,
                    },
                    points,
                },
            )
            .collect(),
        missing_source_edge_adjacencies,
        suppressed_source_tail_face_pair_points,
    }
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
struct RouteCrossingVertices {
    missing_source_edge_adjacencies: usize,
    suppressed_source_tail_face_pair_points: usize,
}

#[allow(clippy::too_many_arguments)]
fn route_crossing_vertices(
    pair: &ExactBoolMeshEdgeFacePair,
    signed_count: i32,
    start: Option<usize>,
    collision: usize,
    edge_mesh: &ExactMesh,
    source_signed_counts: &[i32],
    source_parameter: Option<&ExactReal>,
    fwd: bool,
    origins: &[ExactBoolMeshOutputVertexOrigin],
    source_edge_runs: &mut BTreeMap<(u8, usize), SourceEdgePointBucket>,
    face_pair_runs: &mut BTreeMap<(usize, usize), Vec<ExactBoolMeshRoutedEdgePoint>>,
) -> RouteCrossingVertices {
    let count = signed_abs(signed_count);
    if count == 0 {
        return RouteCrossingVertices::default();
    }
    let Some(start) = start else {
        return RouteCrossingVertices {
            missing_source_edge_adjacencies: 1,
            suppressed_source_tail_face_pair_points: 0,
        };
    };

    let dir = signed_count < 0;
    let suppress_face_pair_points = source_tail_face_pair_owned_by_source_edge(
        pair,
        dir,
        source_signed_counts,
        source_parameter,
    );
    let source_key = (side_key(pair.edge_side), pair.source_halfedge);
    let primary_edge_face = match pair.edge_side {
        ExactBoolMeshSide::Left => pair.face_pair.left_face,
        ExactBoolMeshSide::Right => pair.face_pair.right_face,
    };
    let incident_faces =
        incident_faces_for_source_halfedge(edge_mesh.triangles(), pair.source_halfedge);
    let missing_source_edge_adjacencies = usize::from(!incident_faces.contains(&primary_edge_face));
    let paired_edge_face = incident_faces
        .iter()
        .copied()
        .find(|face| *face != primary_edge_face);

    for copy in 0..count {
        let output_vertex = start + copy;
        let Some(origin) = origins.get(output_vertex).copied() else {
            continue;
        };
        let source_point = ExactBoolMeshRoutedEdgePoint {
            output_vertex,
            order_index: collision,
            collision,
            is_tail: dir,
            origin,
        };
        source_edge_runs
            .entry(source_key)
            .or_insert_with(|| SourceEdgePointBucket {
                tail: pair.edge[0],
                head: pair.edge[1],
                points: Vec::new(),
            })
            .points
            .push(source_point);

        if !suppress_face_pair_points {
            let primary_point = ExactBoolMeshRoutedEdgePoint {
                is_tail: if fwd { !dir } else { dir },
                ..source_point
            };
            face_pair_runs
                .entry(face_pair_key(pair, primary_edge_face))
                .or_default()
                .push(primary_point);

            if let Some(paired_edge_face) = paired_edge_face {
                let paired_point = ExactBoolMeshRoutedEdgePoint {
                    is_tail: if fwd { dir } else { !dir },
                    ..source_point
                };
                face_pair_runs
                    .entry(face_pair_key(pair, paired_edge_face))
                    .or_default()
                    .push(paired_point);
            }
        }
    }

    RouteCrossingVertices {
        missing_source_edge_adjacencies,
        suppressed_source_tail_face_pair_points: if suppress_face_pair_points {
            count * (1 + usize::from(paired_edge_face.is_some()))
        } else {
            0
        },
    }
}

/// Return whether a source-tail `Kernel12` row should stay out of `pt_new`.
///
/// This is the face-pair side of the exact source-tail ownership rule in
/// [`retained_tail_owned_by_kernel12`].  A row at source-edge parameter `0`
/// whose operation-signed role matches the retained source-tail endpoint is
/// already consumed by the source-edge `pt_old` path.  Legacy boolmesh's
/// coplanar branch therefore does not also leave a dangling
/// `append_new_edges` point on both incident face-pair buckets.  The exact
/// port makes that rule explicit with source-halfedge ownership, exact
/// parameter comparison, and signed retained endpoint counts, following Yap,
/// "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
fn source_tail_face_pair_owned_by_source_edge(
    pair: &ExactBoolMeshEdgeFacePair,
    source_point_is_tail: bool,
    source_signed_counts: &[i32],
    source_parameter: Option<&ExactReal>,
) -> bool {
    let Some(parameter) = source_parameter else {
        return false;
    };
    if compare_reals(parameter, &ExactReal::from(0)).value() != Some(Ordering::Equal) {
        return false;
    }
    let Some(signed_count) = source_signed_counts.get(pair.edge[0]).copied() else {
        return false;
    };
    signed_abs(signed_count) > 0 && source_point_is_tail == (signed_count > 0)
}

fn face_pair_key(pair: &ExactBoolMeshEdgeFacePair, edge_face: usize) -> (usize, usize) {
    match pair.edge_side {
        ExactBoolMeshSide::Left => (edge_face, pair.face),
        ExactBoolMeshSide::Right => (pair.face, edge_face),
    }
}

fn source_edge_parameter(mesh: &ExactMesh, edge: [usize; 2], point: &Point3) -> Option<ExactReal> {
    let tail = mesh.vertices().get(edge[0])?.to_hyperlimit_point();
    let head = mesh.vertices().get(edge[1])?.to_hyperlimit_point();
    let deltas = [
        head.x.clone() - &tail.x,
        head.y.clone() - &tail.y,
        head.z.clone() - &tail.z,
    ];
    let numerators = [
        point.x.clone() - &tail.x,
        point.y.clone() - &tail.y,
        point.z.clone() - &tail.z,
    ];
    for axis in 0..3 {
        if compare_reals(&deltas[axis], &ExactReal::from(0)).value() == Some(Ordering::Equal) {
            continue;
        }
        let parameter = (&numerators[axis] / &deltas[axis]).ok()?;
        if point_matches_edge_parameter(&tail, &head, point, &parameter) {
            return Some(parameter);
        }
        return None;
    }
    None
}

fn point_matches_edge_parameter(
    tail: &Point3,
    head: &Point3,
    point: &Point3,
    parameter: &ExactReal,
) -> bool {
    axis_matches_parameter(&tail.x, &head.x, &point.x, parameter)
        && axis_matches_parameter(&tail.y, &head.y, &point.y, parameter)
        && axis_matches_parameter(&tail.z, &head.z, &point.z, parameter)
}

fn axis_matches_parameter(
    tail: &ExactReal,
    head: &ExactReal,
    point: &ExactReal,
    parameter: &ExactReal,
) -> bool {
    let delta = head.clone() - tail;
    let expected = tail.clone() + &(parameter.clone() * delta);
    compare_reals(&expected, point).value() == Some(Ordering::Equal)
}

fn stage_partial_source_edges(
    left: &ExactMesh,
    right: &ExactMesh,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    new_edge_vertices: &ExactBoolMeshNewEdgeVertexStage,
    i03: &[i32],
    i30: &[i32],
    pair_up: &ExactBoolMeshPairUpStage,
) -> ExactBoolMeshPartialSourceEdgeStage {
    let order_index = source_edge_order_index(pair_up);
    let mut source_edge_runs = Vec::new();
    let mut unpaired_runs = 0;
    let mut missing_parameter_orders = 0;

    for run in &new_edge_vertices.source_edge_runs {
        let mesh = match run.side {
            ExactBoolMeshSide::Left => left,
            ExactBoolMeshSide::Right => right,
        };
        let signed_counts = match run.side {
            ExactBoolMeshSide::Left => i03,
            ExactBoolMeshSide::Right => i30,
        };
        let starts = match run.side {
            ExactBoolMeshSide::Left => &allocation.left_vertex_output_starts,
            ExactBoolMeshSide::Right => &allocation.right_vertex_output_starts,
        };
        let mut points = Vec::new();
        let mut substituted_retained_tail_copies = BTreeSet::new();

        for routed in &run.points {
            let order_key = (side_key(run.side), run.source_halfedge, routed.collision);
            let Some(order) = order_index.get(&order_key) else {
                missing_parameter_orders += 1;
                continue;
            };
            if let Some(substitution) =
                source_tail_retained_substitution(run, routed, signed_counts, starts, order)
            {
                substituted_retained_tail_copies.insert(substitution.copy);
                points.push(ExactBoolMeshPartialEdgePoint {
                    output_vertex: substitution.output_vertex,
                    is_tail: routed.is_tail,
                    order_index: order.index + 1,
                    collision: routed.collision,
                    origin: ExactBoolMeshPartialEdgePointOrigin::RetainedEndpoint {
                        source: ExactBoolMeshSourceVertex {
                            side: run.side,
                            vertex: run.tail,
                        },
                        copy: substitution.copy,
                    },
                });
            } else {
                points.push(ExactBoolMeshPartialEdgePoint {
                    output_vertex: routed.output_vertex,
                    is_tail: routed.is_tail,
                    order_index: order.index + 1,
                    collision: routed.collision,
                    origin: ExactBoolMeshPartialEdgePointOrigin::RoutedIntersection(*routed),
                });
            }
        }

        append_retained_endpoint_excluding(
            run.side,
            run.tail,
            signed_counts,
            starts,
            0,
            &substituted_retained_tail_copies,
            &mut points,
        );
        let suppressed_retained_tail_copies = substituted_retained_tail_copies.len();
        append_retained_endpoint(
            run.side,
            run.head,
            signed_counts,
            starts,
            usize::MAX,
            &mut points,
        );

        points.sort_by(partial_point_order);
        let fragments = pair_partial_points(&points);
        let tail_count = points.iter().filter(|point| point.is_tail).count();
        let head_count = points.len() - tail_count;
        let unpaired_points = tail_count.abs_diff(head_count);
        if unpaired_points > 0 {
            unpaired_runs += 1;
        }
        let incident_uses =
            directed_edge_uses_for_source_halfedge(mesh.triangles(), run.source_halfedge);
        source_edge_runs.push(ExactBoolMeshPartialSourceEdgeRun {
            side: run.side,
            source_halfedge: run.source_halfedge,
            tail: run.tail,
            head: run.head,
            incident_faces: incident_uses.iter().map(|use_| use_.face).collect(),
            incident_edges: incident_uses.iter().map(|use_| use_.edge).collect(),
            points,
            fragments,
            suppressed_retained_tail_copies,
            unpaired_points,
        });
    }

    ExactBoolMeshPartialSourceEdgeStage {
        source_edge_runs,
        unpaired_runs,
        missing_parameter_orders,
    }
}

#[derive(Clone, Debug, PartialEq)]
struct SourceEdgeOrder {
    index: usize,
    parameter: ExactReal,
}

fn source_edge_order_index(
    pair_up: &ExactBoolMeshPairUpStage,
) -> BTreeMap<(u8, usize, usize), SourceEdgeOrder> {
    let mut order_index = BTreeMap::new();
    for run in &pair_up.source_edge_runs {
        for (index, event) in run.events.iter().enumerate() {
            order_index.insert(
                (side_key(run.side), run.source_halfedge, event.collision),
                SourceEdgeOrder {
                    index,
                    parameter: event.parameter.clone(),
                },
            );
        }
    }
    order_index
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RetainedTailSubstitution {
    output_vertex: usize,
    copy: usize,
}

/// Substitute a source-tail `Kernel12` row with the retained output vertex.
///
/// Positive-area coplanar lowering can produce a row exactly at source-edge
/// parameter `0` whose operation-signed role is the same retained source-tail
/// boundary that `append_partial_edges` would otherwise copy from `i03`/`i30`.
/// The row still owns the ordering/provenance in the `pt_old` bucket, but the
/// emitted halfedge endpoint must be the retained output vertex so boolmesh
/// face walks close by vertex id before triangulation.  This is the exact
/// object version of Yap's separation between certified construction and
/// topology mutation in "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): the parameter-zero equality is proved exactly, then
/// the combinatorial vertex identity is replayed explicitly.
fn source_tail_retained_substitution(
    run: &ExactBoolMeshSourceEdgePointRun,
    point: &ExactBoolMeshRoutedEdgePoint,
    signed_counts: &[i32],
    starts: &[Option<usize>],
    order: &SourceEdgeOrder,
) -> Option<RetainedTailSubstitution> {
    let Some(signed_count) = signed_counts.get(run.tail).copied() else {
        return None;
    };
    let count = signed_abs(signed_count);
    if count == 0 || point.is_tail != (signed_count > 0) {
        return None;
    }
    if compare_reals(&order.parameter, &ExactReal::from(0)).value() != Some(Ordering::Equal) {
        return None;
    }
    let copy = kernel12_origin_copy(point.origin)?;
    if copy >= count {
        return None;
    }
    let start = starts.get(run.tail).and_then(|start| *start)?;
    Some(RetainedTailSubstitution {
        output_vertex: start + copy,
        copy,
    })
}

fn kernel12_origin_copy(origin: ExactBoolMeshOutputVertexOrigin) -> Option<usize> {
    match origin {
        ExactBoolMeshOutputVertexOrigin::Kernel12LeftEdgeRightFace { copy, .. }
        | ExactBoolMeshOutputVertexOrigin::Kernel12RightEdgeLeftFace { copy, .. } => Some(copy),
        ExactBoolMeshOutputVertexOrigin::SourceVertex { .. } => None,
    }
}

fn append_retained_endpoint_excluding(
    side: ExactBoolMeshSide,
    vertex: usize,
    signed_counts: &[i32],
    starts: &[Option<usize>],
    order_index: usize,
    excluded_copies: &BTreeSet<usize>,
    points: &mut Vec<ExactBoolMeshPartialEdgePoint>,
) {
    append_retained_endpoint_with_filter(
        side,
        vertex,
        signed_counts,
        starts,
        order_index,
        |copy| !excluded_copies.contains(&copy),
        points,
    );
}

fn append_retained_endpoint(
    side: ExactBoolMeshSide,
    vertex: usize,
    signed_counts: &[i32],
    starts: &[Option<usize>],
    order_index: usize,
    points: &mut Vec<ExactBoolMeshPartialEdgePoint>,
) {
    append_retained_endpoint_with_filter(
        side,
        vertex,
        signed_counts,
        starts,
        order_index,
        |_| true,
        points,
    );
}

fn append_retained_endpoint_with_filter<F>(
    side: ExactBoolMeshSide,
    vertex: usize,
    signed_counts: &[i32],
    starts: &[Option<usize>],
    order_index: usize,
    include_copy: F,
    points: &mut Vec<ExactBoolMeshPartialEdgePoint>,
) where
    F: Fn(usize) -> bool,
{
    let Some(signed_count) = signed_counts.get(vertex).copied() else {
        return;
    };
    let count = signed_abs(signed_count);
    if count == 0 {
        return;
    }
    let Some(Some(start)) = starts.get(vertex) else {
        return;
    };
    for copy in 0..count {
        if !include_copy(copy) {
            continue;
        }
        points.push(ExactBoolMeshPartialEdgePoint {
            output_vertex: start + copy,
            is_tail: if order_index == 0 {
                signed_count > 0
            } else {
                signed_count < 0
            },
            order_index,
            collision: usize::MAX,
            origin: ExactBoolMeshPartialEdgePointOrigin::RetainedEndpoint {
                source: ExactBoolMeshSourceVertex { side, vertex },
                copy,
            },
        });
    }
}

fn pair_partial_points(
    points: &[ExactBoolMeshPartialEdgePoint],
) -> Vec<ExactBoolMeshPartialSourceEdgeFragment> {
    let mut tails = points
        .iter()
        .filter(|point| point.is_tail)
        .copied()
        .collect::<Vec<_>>();
    let mut heads = points
        .iter()
        .filter(|point| !point.is_tail)
        .copied()
        .collect::<Vec<_>>();
    tails.sort_by(partial_point_order);
    heads.sort_by(partial_point_order);
    tails
        .into_iter()
        .zip(heads)
        .map(
            |(tail_point, head_point)| ExactBoolMeshPartialSourceEdgeFragment {
                tail_point,
                head_point,
            },
        )
        .collect()
}

fn partial_point_order(
    left: &ExactBoolMeshPartialEdgePoint,
    right: &ExactBoolMeshPartialEdgePoint,
) -> Ordering {
    left.order_index
        .cmp(&right.order_index)
        .then_with(|| left.collision.cmp(&right.collision))
        .then_with(|| left.output_vertex.cmp(&right.output_vertex))
}

fn stage_new_face_pair_edges(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    new_edge_vertices: &ExactBoolMeshNewEdgeVertexStage,
) -> ExactBoolMeshNewFacePairStage {
    let mut unpaired_runs = 0;
    let face_pair_runs = new_edge_vertices
        .face_pair_runs
        .iter()
        .map(|run| {
            let mut points = run.points.clone();
            assign_face_pair_order_indices(left, right, boolean03, allocation, &mut points);
            points.sort_by(routed_point_order);
            let fragments = pair_routed_points(&points);
            let tail_count = points.iter().filter(|point| point.is_tail).count();
            let head_count = points.len() - tail_count;
            let unpaired_points = tail_count.abs_diff(head_count);
            if unpaired_points > 0 {
                unpaired_runs += 1;
            }
            ExactBoolMeshNewFacePairRun {
                face_pair: run.face_pair,
                points,
                fragments,
                unpaired_points,
            }
        })
        .collect();

    ExactBoolMeshNewFacePairStage {
        face_pair_runs,
        unpaired_runs,
    }
}

/// Assign the exact face-local order used by boolmesh `append_new_edges`.
///
/// Legacy boolmesh computes a bounding box over the output positions in one
/// `pt_new` face-pair bucket, chooses the longest coordinate dimension, stores
/// that coordinate in `EdgePt.val`, and then calls `pair_up`.  The exact port
/// keeps the algorithm and removes only the `f64` dependency: output vertex
/// coordinates are replayed from source/`kernel12` provenance, axis spans and
/// point order are compared with exact predicates, and the resulting ordinal is
/// stored as a replayable topology artifact.
///
/// This follows Yap, "Towards Exact Geometric Computation," *Computational
/// Geometry* 7.1-2 (1997): numeric comparisons are certified before topology
/// pairing consumes them.  The bucket algorithm itself is the boolmesh
/// `boolean45::append_new_edges` rule.
fn assign_face_pair_order_indices(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    points: &mut [ExactBoolMeshRoutedEdgePoint],
) {
    let Some(axis) = longest_exact_axis(left, right, boolean03, allocation, points) else {
        assign_symbolic_face_pair_order_indices(points);
        return;
    };
    let mut indexed = (0..points.len()).collect::<Vec<_>>();
    indexed.sort_by(|left_index, right_index| {
        compare_output_vertex_axis(
            left,
            right,
            boolean03,
            allocation,
            points[*left_index].output_vertex,
            points[*right_index].output_vertex,
            axis,
        )
        .unwrap_or(Ordering::Equal)
        .then_with(|| {
            points[*left_index]
                .collision
                .cmp(&points[*right_index].collision)
        })
        .then_with(|| {
            points[*left_index]
                .output_vertex
                .cmp(&points[*right_index].output_vertex)
        })
    });
    for (order_index, point_index) in indexed.into_iter().enumerate() {
        points[point_index].order_index = order_index;
    }
}

fn assign_symbolic_face_pair_order_indices(points: &mut [ExactBoolMeshRoutedEdgePoint]) {
    let mut indexed = (0..points.len()).collect::<Vec<_>>();
    indexed.sort_by(|left_index, right_index| {
        points[*left_index]
            .collision
            .cmp(&points[*right_index].collision)
            .then_with(|| {
                points[*left_index]
                    .output_vertex
                    .cmp(&points[*right_index].output_vertex)
            })
    });
    for (order_index, point_index) in indexed.into_iter().enumerate() {
        points[point_index].order_index = order_index;
    }
}

fn longest_exact_axis(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    points: &[ExactBoolMeshRoutedEdgePoint],
) -> Option<usize> {
    let first = output_vertex_point(
        points.first()?.output_vertex,
        allocation,
        boolean03,
        left,
        right,
    )?;
    let mut mins = [first.x.clone(), first.y.clone(), first.z.clone()];
    let mut maxes = [first.x, first.y, first.z];
    for point in points.iter().skip(1) {
        let output = output_vertex_point(point.output_vertex, allocation, boolean03, left, right)?;
        for axis in 0..3 {
            let coordinate = point_axis(&output, axis);
            if compare_reals(coordinate, &mins[axis]).value()? == Ordering::Less {
                mins[axis] = coordinate.clone();
            }
            if compare_reals(coordinate, &maxes[axis]).value()? == Ordering::Greater {
                maxes[axis] = coordinate.clone();
            }
        }
    }
    let spans = [
        maxes[0].clone() - &mins[0],
        maxes[1].clone() - &mins[1],
        maxes[2].clone() - &mins[2],
    ];
    let mut axis = 0;
    for candidate in 1..3 {
        if compare_reals(&spans[candidate], &spans[axis]).value()? == Ordering::Greater {
            axis = candidate;
        }
    }
    Some(axis)
}

fn compare_output_vertex_axis(
    left: &ExactMesh,
    right: &ExactMesh,
    boolean03: &ExactBoolMeshBoolean03,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    left_vertex: usize,
    right_vertex: usize,
    axis: usize,
) -> Option<Ordering> {
    let left_point = output_vertex_point(left_vertex, allocation, boolean03, left, right)?;
    let right_point = output_vertex_point(right_vertex, allocation, boolean03, left, right)?;
    compare_reals(
        point_axis(&left_point, axis),
        point_axis(&right_point, axis),
    )
    .value()
}

fn point_axis(point: &hyperlimit::Point3, axis: usize) -> &ExactReal {
    match axis {
        0 => &point.x,
        1 => &point.y,
        _ => &point.z,
    }
}

fn pair_routed_points(
    points: &[ExactBoolMeshRoutedEdgePoint],
) -> Vec<ExactBoolMeshNewFacePairFragment> {
    let mut tails = points
        .iter()
        .filter(|point| point.is_tail)
        .copied()
        .collect::<Vec<_>>();
    let mut heads = points
        .iter()
        .filter(|point| !point.is_tail)
        .copied()
        .collect::<Vec<_>>();
    tails.sort_by(routed_point_order);
    heads.sort_by(routed_point_order);
    tails
        .into_iter()
        .zip(heads)
        .map(
            |(tail_point, head_point)| ExactBoolMeshNewFacePairFragment {
                tail_point,
                head_point,
            },
        )
        .collect()
}

fn routed_point_order(
    left: &ExactBoolMeshRoutedEdgePoint,
    right: &ExactBoolMeshRoutedEdgePoint,
) -> Ordering {
    left.order_index
        .cmp(&right.order_index)
        .then_with(|| left.collision.cmp(&right.collision))
        .then_with(|| left.output_vertex.cmp(&right.output_vertex))
}

/// Stage legacy `boolean45::append_whole_edges` over untouched source edges.
///
/// Yap's "Towards Exact Geometric Computation" treats the combinatorial
/// decision as part of the exact object pipeline, so this pass copies only
/// source edges whose operation-signed endpoint allocations replay exactly.
/// The emitted fragments keep the Weiler-Atherton-style boundary-fragment
/// shape used by the boolmesh kernels without using rounded coordinates for
/// orientation or identity.  The source row retained on each run is the exact
/// counterpart of boolmesh's `append_whole_edges` loop over `hid_p` with
/// `Half::is_forward()` filtering.
fn stage_whole_source_edges(
    left: &ExactMesh,
    right: &ExactMesh,
    allocation: &ExactBoolMeshOutputVertexAllocation,
    i03: &[i32],
    i30: &[i32],
    partial_source_edges: &ExactBoolMeshPartialSourceEdgeStage,
) -> ExactBoolMeshWholeSourceEdgeStage {
    let touched_edges = partial_source_edges
        .source_edge_runs
        .iter()
        .map(|run| (side_key(run.side), canonical_edge([run.tail, run.head])))
        .collect::<BTreeSet<_>>();
    let mut stage = ExactBoolMeshWholeSourceEdgeStage::default();
    append_whole_source_edges_for_side(
        ExactBoolMeshSide::Left,
        left,
        i03,
        &allocation.left_vertex_output_starts,
        &touched_edges,
        &mut stage,
    );
    append_whole_source_edges_for_side(
        ExactBoolMeshSide::Right,
        right,
        i30,
        &allocation.right_vertex_output_starts,
        &touched_edges,
        &mut stage,
    );
    stage
}

fn append_whole_source_edges_for_side(
    side: ExactBoolMeshSide,
    mesh: &ExactMesh,
    signed_counts: &[i32],
    starts: &[Option<usize>],
    touched_edges: &BTreeSet<(u8, [usize; 2])>,
    stage: &mut ExactBoolMeshWholeSourceEdgeStage,
) {
    for source_edge in source_edges(mesh.triangles()) {
        if touched_edges.contains(&(side_key(side), source_edge.edge)) {
            continue;
        }
        let Some(signed_count) = signed_counts.get(source_edge.edge[0]).copied() else {
            continue;
        };
        let count = signed_abs(signed_count);
        if count == 0 {
            continue;
        }
        let Some(Some(tail_start)) = starts.get(source_edge.edge[0]) else {
            stage.missing_endpoint_allocations += 1;
            continue;
        };
        let Some(Some(head_start)) = starts.get(source_edge.edge[1]) else {
            stage.missing_endpoint_allocations += 1;
            continue;
        };
        let reversed = signed_count < 0;
        let fragments = (0..count)
            .map(|copy| {
                let tail = tail_start + copy;
                let head = head_start + copy;
                if reversed {
                    ExactBoolMeshWholeSourceEdgeFragment {
                        output_tail: head,
                        output_head: tail,
                        copy,
                        reversed,
                    }
                } else {
                    ExactBoolMeshWholeSourceEdgeFragment {
                        output_tail: tail,
                        output_head: head,
                        copy,
                        reversed,
                    }
                }
            })
            .collect();
        stage
            .source_edge_runs
            .push(ExactBoolMeshWholeSourceEdgeRun {
                side,
                source_halfedge: source_edge.source_halfedge,
                edge: source_edge.edge,
                incident_faces: source_edge.incident_faces,
                incident_edges: source_edge.incident_edges,
                signed_count,
                fragments,
            });
    }
}

#[derive(Clone, Debug)]
struct SourceEdgePointBucket {
    tail: usize,
    head: usize,
    points: Vec<ExactBoolMeshRoutedEdgePoint>,
}

#[derive(Clone, Debug)]
struct SourceEdge {
    source_halfedge: usize,
    edge: [usize; 2],
    incident_faces: Vec<usize>,
    incident_edges: Vec<[usize; 2]>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct SourceEdgeUse {
    source_halfedge: usize,
    face: usize,
    edge: [usize; 2],
}

fn source_edges(triangles: &[Triangle]) -> Vec<SourceEdge> {
    let mut edges = BTreeMap::<[usize; 2], Vec<SourceEdgeUse>>::new();
    for (face, triangle) in triangles.iter().enumerate() {
        for (local, edge) in triangle_edges(*triangle).into_iter().enumerate() {
            edges
                .entry(canonical_edge(edge))
                .or_default()
                .push(SourceEdgeUse {
                    source_halfedge: face * 3 + local,
                    face,
                    edge,
                });
        }
    }
    edges
        .into_iter()
        .map(|(edge, mut uses)| {
            let preferred = preferred_whole_source_edge(edge, &uses);
            sort_source_edge_uses(preferred.edge, &mut uses);
            SourceEdge {
                source_halfedge: preferred.source_halfedge,
                edge: preferred.edge,
                incident_faces: uses.iter().map(|use_| use_.face).collect(),
                incident_edges: uses.iter().map(|use_| use_.edge).collect(),
            }
        })
        .collect()
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PreferredSourceEdge {
    source_halfedge: usize,
    edge: [usize; 2],
}

fn preferred_whole_source_edge(_edge: [usize; 2], uses: &[SourceEdgeUse]) -> PreferredSourceEdge {
    let use_ = uses
        .iter()
        .find(|use_| use_.edge[0] < use_.edge[1])
        .or_else(|| uses.first())
        .copied()
        .expect("source edge buckets are built from at least one triangle use");
    PreferredSourceEdge {
        source_halfedge: use_.source_halfedge,
        edge: use_.edge,
    }
}

fn count_crossing_vertex(
    pair: &ExactBoolMeshEdgeFacePair,
    increment: usize,
    edge_mesh: &ExactMesh,
    edge_face_counts: &mut [usize],
    opposite_face_counts: &mut [usize],
    count_opposite_face: bool,
) -> usize {
    let incident_faces =
        incident_faces_for_source_halfedge(edge_mesh.triangles(), pair.source_halfedge);
    for face in &incident_faces {
        if let Some(count) = edge_face_counts.get_mut(*face) {
            *count += increment;
        }
    }
    if count_opposite_face {
        if let Some(count) = opposite_face_counts.get_mut(pair.face) {
            *count += increment;
        }
    }

    let primary_edge_face = match pair.edge_side {
        ExactBoolMeshSide::Left => pair.face_pair.left_face,
        ExactBoolMeshSide::Right => pair.face_pair.right_face,
    };
    usize::from(!incident_faces.contains(&primary_edge_face))
}

/// Apply exact ownership corrections to the boolmesh-style face slot counts.
///
/// Legacy `boolean45::size_output` reserves one halfedge slot per retained
/// source-vertex copy and per crossing contribution, then the mutation passes
/// consume those slots.  The exact coplanar source-tail port can prove that a
/// retained tail copy and the same-parameter `Kernel12` row represent the same
/// boundary object before mutation.  Following Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997), the certified
/// ownership decision is replayed into the combinatorial size object instead
/// of leaving later face assembly with impossible empty slots.
fn apply_suppressed_retained_tail_face_counts(
    partial_source_edges: &ExactBoolMeshPartialSourceEdgeStage,
    left_face_counts: &mut [usize],
    right_face_counts: &mut [usize],
) {
    for run in &partial_source_edges.source_edge_runs {
        if run.suppressed_retained_tail_copies == 0 {
            continue;
        }
        match run.side {
            ExactBoolMeshSide::Left => {
                for source_face in &run.incident_faces {
                    if let Some(count) = left_face_counts.get_mut(*source_face) {
                        *count = count.saturating_sub(run.suppressed_retained_tail_copies);
                    }
                }
            }
            ExactBoolMeshSide::Right => {
                for source_face in &run.incident_faces {
                    if let Some(count) = right_face_counts.get_mut(*source_face) {
                        *count = count.saturating_sub(run.suppressed_retained_tail_copies);
                    }
                }
            }
        }
    }
}

fn incident_faces_for_source_halfedge(
    triangles: &[Triangle],
    source_halfedge: usize,
) -> Vec<usize> {
    directed_edge_uses_for_source_halfedge(triangles, source_halfedge)
        .iter()
        .map(|use_| use_.face)
        .collect()
}

/// Return incident face uses in the same first-face order boolmesh gets from
/// its source halfedge cursor.
///
/// For split edges, `source_halfedge` is the `hid_p` row that owns the
/// `pt_old` bucket in boolmesh `add_new_edge_verts`/`append_partial_edges`.
/// The preferred face is `source_halfedge / 3`; the paired reverse face, when
/// present, follows by matching the undirected source edge.  The ordered uses
/// let the exact stages emit a halfedge to `face_of(hid_p)` and then to
/// `face_of(pair(hid_p))`, matching boolmesh while avoiding any rounded
/// orientation recovery.  This follows Yap, "Towards Exact Geometric
/// Computation," by making the combinatorial orientation a replayed exact
/// artifact.
fn directed_edge_uses_for_source_halfedge(
    triangles: &[Triangle],
    source_halfedge: usize,
) -> Vec<SourceEdgeUse> {
    let face = source_halfedge / 3;
    let local = source_halfedge % 3;
    let Some(triangle) = triangles.get(face).copied() else {
        return Vec::new();
    };
    directed_edge_uses_for_edge(triangles, triangle_edges(triangle)[local])
}

/// Return incident face uses for a retained whole edge.
///
/// Whole-edge emission still iterates canonical source edges, equivalent to
/// boolmesh's `append_whole_edges` pass over forward halfedges.  Split-edge
/// emission uses [`directed_edge_uses_for_source_halfedge`] instead.
fn directed_edge_uses_for_edge(
    triangles: &[Triangle],
    preferred: [usize; 2],
) -> Vec<SourceEdgeUse> {
    let key = canonical_edge(preferred);
    let mut uses = triangles
        .iter()
        .enumerate()
        .flat_map(|(face, triangle)| {
            triangle_edges(*triangle)
                .into_iter()
                .enumerate()
                .filter(move |(_, candidate)| canonical_edge(*candidate) == key)
                .map(move |(local, edge)| SourceEdgeUse {
                    source_halfedge: face * 3 + local,
                    face,
                    edge,
                })
        })
        .collect::<Vec<_>>();
    sort_source_edge_uses(preferred, &mut uses);
    uses
}

fn sort_source_edge_uses(preferred: [usize; 2], uses: &mut [SourceEdgeUse]) {
    uses.sort_by_key(|use_| {
        (
            use_.edge != preferred,
            use_.edge != [preferred[1], preferred[0]],
            use_.face,
        )
    });
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

#[cfg(test)]
mod tests {
    use super::super::{
        ExactBoolMeshEdgeEvent, ExactBoolMeshOutputVertexOrigin, ExactBoolMeshPointConstruction,
    };
    use super::*;

    fn edge_parameter_event(collision: usize, parameter: i64) -> ExactBoolMeshEdgeEvent {
        ExactBoolMeshEdgeEvent {
            side: ExactBoolMeshSide::Left,
            source_halfedge: 6,
            tail: 1,
            head: 2,
            parameter: ExactReal::from(parameter),
            collision,
            is_tail: false,
            point: ExactBoolMeshPointConstruction::EdgeParameter {
                side: ExactBoolMeshSide::Left,
                tail: 1,
                head: 2,
                parameter: ExactReal::from(parameter),
            },
        }
    }

    fn edge_face_pair() -> ExactBoolMeshEdgeFacePair {
        ExactBoolMeshEdgeFacePair {
            face_pair: ExactBoolMeshFacePair {
                left_face: 2,
                right_face: 0,
            },
            edge_side: ExactBoolMeshSide::Left,
            source_halfedge: 6,
            edge: [1, 2],
            face_side: ExactBoolMeshSide::Right,
            face: 0,
        }
    }

    #[test]
    fn source_tail_kernel12_event_substitutes_retained_tail_endpoint() {
        let pair_up = ExactBoolMeshPairUpStage {
            source_edge_runs: vec![ExactBoolMeshSourceEdgeRun {
                side: ExactBoolMeshSide::Left,
                source_halfedge: 6,
                tail: 1,
                head: 2,
                events: vec![edge_parameter_event(7, 0)],
                fragments: Vec::new(),
                unpaired_events: 1,
            }],
            unknown_orderings: 0,
            unpaired_event_runs: 1,
        };
        let order_index = source_edge_order_index(&pair_up);
        let run = ExactBoolMeshSourceEdgePointRun {
            side: ExactBoolMeshSide::Left,
            source_halfedge: 6,
            tail: 1,
            head: 2,
            points: vec![ExactBoolMeshRoutedEdgePoint {
                output_vertex: 3,
                order_index: 7,
                collision: 7,
                is_tail: true,
                origin: ExactBoolMeshOutputVertexOrigin::Kernel12LeftEdgeRightFace {
                    event: 0,
                    copy: 0,
                },
            }],
        };

        let routed = run.points[0];
        let order = order_index
            .get(&(side_key(run.side), run.source_halfedge, routed.collision))
            .unwrap();

        assert_eq!(
            source_tail_retained_substitution(
                &run,
                &routed,
                &[0, 1, 1],
                &[None, Some(3), Some(4)],
                order
            ),
            Some(RetainedTailSubstitution {
                output_vertex: 3,
                copy: 0
            })
        );
    }

    #[test]
    fn non_tail_parameter_keeps_retained_tail_endpoint() {
        let pair_up = ExactBoolMeshPairUpStage {
            source_edge_runs: vec![ExactBoolMeshSourceEdgeRun {
                side: ExactBoolMeshSide::Left,
                source_halfedge: 6,
                tail: 1,
                head: 2,
                events: vec![edge_parameter_event(7, 1)],
                fragments: Vec::new(),
                unpaired_events: 1,
            }],
            unknown_orderings: 0,
            unpaired_event_runs: 1,
        };
        let order_index = source_edge_order_index(&pair_up);
        let run = ExactBoolMeshSourceEdgePointRun {
            side: ExactBoolMeshSide::Left,
            source_halfedge: 6,
            tail: 1,
            head: 2,
            points: vec![ExactBoolMeshRoutedEdgePoint {
                output_vertex: 3,
                order_index: 7,
                collision: 7,
                is_tail: true,
                origin: ExactBoolMeshOutputVertexOrigin::Kernel12LeftEdgeRightFace {
                    event: 0,
                    copy: 0,
                },
            }],
        };

        let routed = run.points[0];
        let order = order_index
            .get(&(side_key(run.side), run.source_halfedge, routed.collision))
            .unwrap();

        assert_eq!(
            source_tail_retained_substitution(
                &run,
                &routed,
                &[0, 1, 1],
                &[None, Some(3), Some(4)],
                order
            ),
            None
        );
    }

    #[test]
    fn source_tail_kernel12_event_suppresses_duplicate_face_pair_points() {
        let parameter = ExactReal::from(0);

        assert!(source_tail_face_pair_owned_by_source_edge(
            &edge_face_pair(),
            true,
            &[0, 1, 1],
            Some(&parameter),
        ));
    }

    #[test]
    fn non_tail_parameter_keeps_face_pair_points() {
        let parameter = ExactReal::from(1);

        assert!(!source_tail_face_pair_owned_by_source_edge(
            &edge_face_pair(),
            true,
            &[0, 1, 1],
            Some(&parameter),
        ));
    }
}
