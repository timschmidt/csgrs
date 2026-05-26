//! Exact lowering for the boolmesh `boolean03::kernel12` stage.
//!
//! This file is intentionally scoped to the exact counterpart of legacy
//! `boolean03::kernel12`: edge/face discovery produces `p1q2`/`p2q1`,
//! signed `x12`/`x21`, and exact `v12`/`v21` intersection points.  The first
//! lowered subcase is the certified proper segment/plane crossing, because it
//! carries all Yap-style construction evidence required before topology
//! mutation: source edge, opposite face, exact parameter, determinant ratio,
//! exact point, and endpoint side facts.  Endpoint-on-plane, edge/edge shadow,
//! and coplanar overlap lowering remain explicit `Kernel12` work until their
//! boolmesh shadow rules are ported directly.

use hyperlimit::{PlaneSide, Point3};

use super::{
    ExactBoolMeshEdgeEvent, ExactBoolMeshEdgeFacePair, ExactBoolMeshKernel12Event,
    ExactBoolMeshPointConstruction, ExactBoolMeshSide, SegmentPlaneRelation,
};

/// Lowered exact `kernel12` tables for the `Boolean03` package.
///
/// The layout mirrors boolmesh's `intersect12` output on both directions:
/// left-edge/right-face events feed `p1q2`, `x12`, and `v12`; right-edge/
/// left-face events feed `p2q1`, `x21`, and `v21`.  Sorting follows the legacy
/// `(halfedge, face)` ordering as closely as the exact mesh handle model
/// currently allows, using directed edge endpoints plus opposite face.
#[derive(Clone, Debug, Default, PartialEq)]
pub(super) struct ExactBoolMeshKernel12Lowering {
    /// Left-edge/right-face ownership pairs.
    pub(super) p1q2: Vec<ExactBoolMeshEdgeFacePair>,
    /// Right-edge/left-face ownership pairs.
    pub(super) p2q1: Vec<ExactBoolMeshEdgeFacePair>,
    /// Signed left-edge/right-face crossing multiplicities.
    pub(super) x12: Vec<i32>,
    /// Signed right-edge/left-face crossing multiplicities.
    pub(super) x21: Vec<i32>,
    /// Exact left-edge/right-face crossing points.
    pub(super) v12: Vec<Point3>,
    /// Exact right-edge/left-face crossing points.
    pub(super) v21: Vec<Point3>,
    /// Source-edge events consumed by exact `boolean45::pair_up`.
    pub(super) source_edge_events: Vec<ExactBoolMeshEdgeEvent>,
}

/// Lower certified proper segment/plane crossings into boolmesh tables.
///
/// The sign is the oriented transition of the directed source edge through the
/// opposite oriented face plane.  That is the exact analogue of the non-tie
/// branch in boolmesh's shadow accumulation: below-to-above and above-to-below
/// are the only lowered cases here.  Degenerate ties are deliberately excluded
/// until the corresponding `kernel02`/`kernel11` shadow paths are ported from
/// the paper/legacy implementation.  The segment/plane predicate split follows
/// Moller, "A Fast Triangle-Triangle Intersection Test," *Journal of Graphics
/// Tools* 2.2 (1997), and Guigue and Devillers, "Fast and Robust
/// Triangle-Triangle Overlap Test Using Orientation Predicates," *Journal of
/// Graphics Tools* 8.1 (2003); retaining the parameter, point, and endpoint
/// side facts before mutating topology follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
pub(super) fn lower_kernel12_events(
    events: &[ExactBoolMeshKernel12Event],
) -> ExactBoolMeshKernel12Lowering {
    let mut left = Vec::<LoweredKernel12Event>::new();
    let mut right = Vec::<LoweredKernel12Event>::new();

    for event in events {
        if event.relation != SegmentPlaneRelation::ProperCrossing {
            continue;
        }
        let Some(sign) = signed_plane_transition(event.endpoint_sides) else {
            continue;
        };
        let Some(point) = event.point.clone() else {
            continue;
        };
        let Some(parameter) = event.parameter.clone() else {
            continue;
        };
        if event.parameter_ratio.is_none() {
            continue;
        }

        let lowered = LoweredKernel12Event {
            edge_face: event.edge_face,
            sign,
            point,
            parameter,
        };
        match (event.edge_face.edge_side, event.edge_face.face_side) {
            (ExactBoolMeshSide::Left, ExactBoolMeshSide::Right) => {
                left.push(lowered);
            }
            (ExactBoolMeshSide::Right, ExactBoolMeshSide::Left) => {
                right.push(lowered);
            }
            _ => {}
        }
    }

    sort_lowered_events(&mut left);
    sort_lowered_events(&mut right);

    let source_edge_events = left
        .iter()
        .enumerate()
        .map(|(collision, event)| source_edge_event(event, collision))
        .chain(
            right
                .iter()
                .enumerate()
                .map(|(index, event)| source_edge_event(event, left.len() + index)),
        )
        .collect();

    ExactBoolMeshKernel12Lowering {
        p1q2: left.iter().map(|event| event.edge_face).collect(),
        p2q1: right.iter().map(|event| event.edge_face).collect(),
        x12: left.iter().map(|event| event.sign).collect(),
        x21: right.iter().map(|event| event.sign).collect(),
        v12: left.into_iter().map(|event| event.point).collect(),
        v21: right.into_iter().map(|event| event.point).collect(),
        source_edge_events,
    }
}

#[derive(Clone, Debug, PartialEq)]
struct LoweredKernel12Event {
    edge_face: ExactBoolMeshEdgeFacePair,
    sign: i32,
    point: Point3,
    parameter: super::ExactReal,
}

fn source_edge_event(event: &LoweredKernel12Event, collision: usize) -> ExactBoolMeshEdgeEvent {
    ExactBoolMeshEdgeEvent {
        side: event.edge_face.edge_side,
        tail: event.edge_face.edge[0],
        head: event.edge_face.edge[1],
        parameter: event.parameter.clone(),
        collision,
        is_tail: event.sign < 0,
        point: ExactBoolMeshPointConstruction::SegmentPlane {
            edge_side: event.edge_face.edge_side,
            tail: event.edge_face.edge[0],
            head: event.edge_face.edge[1],
            face: event.edge_face.face,
            parameter: event.parameter.clone(),
        },
    }
}

fn signed_plane_transition(endpoint_sides: [Option<PlaneSide>; 2]) -> Option<i32> {
    match endpoint_sides {
        [Some(PlaneSide::Below), Some(PlaneSide::Above)] => Some(1),
        [Some(PlaneSide::Above), Some(PlaneSide::Below)] => Some(-1),
        _ => None,
    }
}

fn sort_lowered_events(events: &mut [LoweredKernel12Event]) {
    events.sort_by(|left, right| {
        (
            left.edge_face.edge[0],
            left.edge_face.edge[1],
            left.edge_face.face,
            left.edge_face.face_pair.left_face,
            left.edge_face.face_pair.right_face,
        )
            .cmp(&(
                right.edge_face.edge[0],
                right.edge_face.edge[1],
                right.edge_face.face,
                right.edge_face.face_pair.left_face,
                right.edge_face.face_pair.right_face,
            ))
    });
}
