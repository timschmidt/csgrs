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
    ExactBoolMeshEdgeFacePair, ExactBoolMeshKernel12Event, ExactBoolMeshSide, SegmentPlaneRelation,
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
    let mut left = Vec::<(ExactBoolMeshEdgeFacePair, i32, Point3)>::new();
    let mut right = Vec::<(ExactBoolMeshEdgeFacePair, i32, Point3)>::new();

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
        if event.parameter.is_none() || event.parameter_ratio.is_none() {
            continue;
        }

        match (event.edge_face.edge_side, event.edge_face.face_side) {
            (ExactBoolMeshSide::Left, ExactBoolMeshSide::Right) => {
                left.push((event.edge_face, sign, point));
            }
            (ExactBoolMeshSide::Right, ExactBoolMeshSide::Left) => {
                right.push((event.edge_face, sign, point));
            }
            _ => {}
        }
    }

    sort_lowered_events(&mut left);
    sort_lowered_events(&mut right);

    ExactBoolMeshKernel12Lowering {
        p1q2: left.iter().map(|(pair, _, _)| *pair).collect(),
        p2q1: right.iter().map(|(pair, _, _)| *pair).collect(),
        x12: left.iter().map(|(_, sign, _)| *sign).collect(),
        x21: right.iter().map(|(_, sign, _)| *sign).collect(),
        v12: left.into_iter().map(|(_, _, point)| point).collect(),
        v21: right.into_iter().map(|(_, _, point)| point).collect(),
    }
}

fn signed_plane_transition(endpoint_sides: [Option<PlaneSide>; 2]) -> Option<i32> {
    match endpoint_sides {
        [Some(PlaneSide::Below), Some(PlaneSide::Above)] => Some(1),
        [Some(PlaneSide::Above), Some(PlaneSide::Below)] => Some(-1),
        _ => None,
    }
}

fn sort_lowered_events(events: &mut [(ExactBoolMeshEdgeFacePair, i32, Point3)]) {
    events.sort_by(|(left, _, _), (right, _, _)| {
        (
            left.edge[0],
            left.edge[1],
            left.face,
            left.face_pair.left_face,
            left.face_pair.right_face,
        )
            .cmp(&(
                right.edge[0],
                right.edge[1],
                right.face,
                right.face_pair.left_face,
                right.face_pair.right_face,
            ))
    });
}
