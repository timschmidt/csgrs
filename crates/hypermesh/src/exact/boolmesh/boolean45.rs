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

mod assembly;

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

use hyperlimit::compare_reals;

use assembly::assemble_output_halfedges;

use crate::exact::boolean::ExactBooleanOperation;
use crate::exact::mesh::{ExactMesh, Triangle};

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
/// partitioned into tail/head sides and zipped into fragments.  The legacy
/// kernel orders by the longest bounding-box coordinate of rounded output
/// positions; this exact stage uses collision/output ids as deterministic
/// symbolic ordering until face-local exact curve order is ported.
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
    let new_edge_vertices =
        route_new_edge_vertices(left, right, boolean03, &vertex_allocation, &i12, &i21);
    let partial_source_edges = stage_partial_source_edges(
        left,
        right,
        &vertex_allocation,
        &new_edge_vertices,
        &i03,
        &i30,
        pair_up,
    );
    let new_face_pair_edges = stage_new_face_pair_edges(&new_edge_vertices);
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
    let halfedge_assembly = assemble_output_halfedges(
        &partial_source_edges,
        &new_face_pair_edges,
        &whole_source_edges,
        &source_face_to_output_face,
        &face_halfedge_offsets,
        left.triangles().len(),
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
    i12: &[i32],
    i21: &[i32],
) -> ExactBoolMeshNewEdgeVertexStage {
    let mut source_edge_runs =
        BTreeMap::<(u8, usize, usize), Vec<ExactBoolMeshRoutedEdgePoint>>::new();
    let mut face_pair_runs = BTreeMap::<(usize, usize), Vec<ExactBoolMeshRoutedEdgePoint>>::new();
    let mut missing_source_edge_adjacencies = 0;

    for (event, (pair, signed_count)) in boolean03.p1q2.iter().zip(i12.iter()).enumerate() {
        missing_source_edge_adjacencies += route_crossing_vertices(
            pair,
            *signed_count,
            allocation.p1q2_output_starts[event],
            event,
            left,
            true,
            &allocation.output_vertex_origins,
            &mut source_edge_runs,
            &mut face_pair_runs,
        );
    }
    let collision_offset = boolean03.p1q2.len();
    for (event, (pair, signed_count)) in boolean03.p2q1.iter().zip(i21.iter()).enumerate() {
        missing_source_edge_adjacencies += route_crossing_vertices(
            pair,
            *signed_count,
            allocation.p2q1_output_starts[event],
            collision_offset + event,
            right,
            false,
            &allocation.output_vertex_origins,
            &mut source_edge_runs,
            &mut face_pair_runs,
        );
    }

    ExactBoolMeshNewEdgeVertexStage {
        source_edge_runs: source_edge_runs
            .into_iter()
            .map(
                |((side_key, tail, head), points)| ExactBoolMeshSourceEdgePointRun {
                    side: side_from_key(side_key),
                    tail,
                    head,
                    points,
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
    }
}

#[allow(clippy::too_many_arguments)]
fn route_crossing_vertices(
    pair: &ExactBoolMeshEdgeFacePair,
    signed_count: i32,
    start: Option<usize>,
    collision: usize,
    edge_mesh: &ExactMesh,
    fwd: bool,
    origins: &[ExactBoolMeshOutputVertexOrigin],
    source_edge_runs: &mut BTreeMap<(u8, usize, usize), Vec<ExactBoolMeshRoutedEdgePoint>>,
    face_pair_runs: &mut BTreeMap<(usize, usize), Vec<ExactBoolMeshRoutedEdgePoint>>,
) -> usize {
    let count = signed_abs(signed_count);
    if count == 0 {
        return 0;
    }
    let Some(start) = start else {
        return 1;
    };

    let dir = signed_count < 0;
    let source_key = (side_key(pair.edge_side), pair.edge[0], pair.edge[1]);
    let primary_edge_face = match pair.edge_side {
        ExactBoolMeshSide::Left => pair.face_pair.left_face,
        ExactBoolMeshSide::Right => pair.face_pair.right_face,
    };
    let incident_faces = incident_faces_for_edge(edge_mesh.triangles(), pair.edge);
    let mut missing_source_edge_adjacencies =
        usize::from(!incident_faces.contains(&primary_edge_face));
    let paired_edge_face = incident_faces
        .iter()
        .copied()
        .find(|face| *face != primary_edge_face);
    if paired_edge_face.is_none() {
        missing_source_edge_adjacencies += 1;
    }

    for copy in 0..count {
        let output_vertex = start + copy;
        let Some(origin) = origins.get(output_vertex).copied() else {
            continue;
        };
        let source_point = ExactBoolMeshRoutedEdgePoint {
            output_vertex,
            collision,
            is_tail: dir,
            origin,
        };
        source_edge_runs
            .entry(source_key)
            .or_default()
            .push(source_point);

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

    missing_source_edge_adjacencies
}

fn face_pair_key(pair: &ExactBoolMeshEdgeFacePair, edge_face: usize) -> (usize, usize) {
    match pair.edge_side {
        ExactBoolMeshSide::Left => (edge_face, pair.face),
        ExactBoolMeshSide::Right => (pair.face, edge_face),
    }
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

        for routed in &run.points {
            let order_key = (side_key(run.side), run.tail, run.head, routed.collision);
            let Some(order) = order_index.get(&order_key).copied() else {
                missing_parameter_orders += 1;
                continue;
            };
            points.push(ExactBoolMeshPartialEdgePoint {
                output_vertex: routed.output_vertex,
                is_tail: routed.is_tail,
                order_index: order + 1,
                collision: routed.collision,
                origin: ExactBoolMeshPartialEdgePointOrigin::RoutedIntersection(*routed),
            });
        }

        append_retained_endpoint(run.side, run.tail, signed_counts, starts, 0, &mut points);
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
        source_edge_runs.push(ExactBoolMeshPartialSourceEdgeRun {
            side: run.side,
            tail: run.tail,
            head: run.head,
            incident_faces: incident_faces_for_edge(mesh.triangles(), [run.tail, run.head]),
            points,
            fragments,
            unpaired_points,
        });
    }

    ExactBoolMeshPartialSourceEdgeStage {
        source_edge_runs,
        unpaired_runs,
        missing_parameter_orders,
    }
}

fn source_edge_order_index(
    pair_up: &ExactBoolMeshPairUpStage,
) -> BTreeMap<(u8, usize, usize, usize), usize> {
    let mut order_index = BTreeMap::new();
    for run in &pair_up.source_edge_runs {
        for (index, event) in run.events.iter().enumerate() {
            order_index.insert(
                (side_key(run.side), run.tail, run.head, event.collision),
                index,
            );
        }
    }
    order_index
}

fn append_retained_endpoint(
    side: ExactBoolMeshSide,
    vertex: usize,
    signed_counts: &[i32],
    starts: &[Option<usize>],
    order_index: usize,
    points: &mut Vec<ExactBoolMeshPartialEdgePoint>,
) {
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
    new_edge_vertices: &ExactBoolMeshNewEdgeVertexStage,
) -> ExactBoolMeshNewFacePairStage {
    let mut unpaired_runs = 0;
    let face_pair_runs = new_edge_vertices
        .face_pair_runs
        .iter()
        .map(|run| {
            let mut points = run.points.clone();
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
    left.collision
        .cmp(&right.collision)
        .then_with(|| left.output_vertex.cmp(&right.output_vertex))
}

/// Stage legacy `boolean45::append_whole_edges` over untouched source edges.
///
/// Yap's "Towards Exact Geometric Computation" treats the combinatorial
/// decision as part of the exact object pipeline, so this pass copies only
/// source edges whose operation-signed endpoint allocations replay exactly.
/// The emitted fragments keep the Weiler-Atherton-style boundary-fragment
/// shape used by the boolmesh kernels without using rounded coordinates for
/// orientation or identity.
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
                edge: source_edge.edge,
                incident_faces: source_edge.incident_faces,
                signed_count,
                fragments,
            });
    }
}

#[derive(Clone, Debug)]
struct SourceEdge {
    edge: [usize; 2],
    incident_faces: Vec<usize>,
}

fn source_edges(triangles: &[Triangle]) -> Vec<SourceEdge> {
    let mut edges = BTreeMap::<[usize; 2], Vec<usize>>::new();
    for (face, triangle) in triangles.iter().enumerate() {
        for edge in triangle_edges(*triangle) {
            edges.entry(canonical_edge(edge)).or_default().push(face);
        }
    }
    edges
        .into_iter()
        .map(|(edge, incident_faces)| SourceEdge {
            edge,
            incident_faces,
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
