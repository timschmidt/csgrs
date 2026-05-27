//! Exact lowering for the boolmesh `boolean03::kernel12` stage.
//!
//! This file is intentionally scoped to the exact counterpart of legacy
//! `boolean03::kernel12`: edge/face discovery produces `p1q2`/`p2q1`,
//! signed `x12`/`x21`, and exact `v12`/`v21` intersection points.  The first
//! lowered subcase is the certified proper segment/plane crossing, because it
//! carries all Yap-style construction evidence required before topology
//! mutation: source edge, opposite face, exact parameter, determinant ratio,
//! exact point, and endpoint side facts.  Strict vertex/face endpoint shadows
//! are also lowered once the endpoint is certified inside the opposite
//! triangle.  Edge/edge boundary shadows and coplanar overlap lowering remain
//! explicit `Kernel12` work until their boolmesh shadow rules are ported
//! directly.

use std::cmp::Ordering;

use hyperlimit::{
    CoplanarProjection, PlaneSide, Point3, Sign, TriangleLocation, classify_point_triangle,
    compare_reals, project_point3, projected_polygon_area2_value,
};

use crate::exact::mesh::ExactMesh;

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

/// Lower certified segment/plane contacts into boolmesh tables.
///
/// The sign is the oriented transition of the directed source edge through the
/// opposite oriented face plane.  That is the exact analogue of the non-tie
/// branch in boolmesh's shadow accumulation for proper crossings.  For
/// endpoint contacts this ports the strict vertex/face part of
/// `boolean03::kernel12`: legacy `Kernel02` contributes when a source endpoint
/// shadows the opposite face interior, while edge/edge and boundary shadows
/// stay out of this slice.  The segment/plane predicate split follows Moller,
/// "A Fast Triangle-Triangle Intersection Test," *Journal of Graphics Tools*
/// 2.2 (1997), and Guigue and Devillers, "Fast and Robust Triangle-Triangle
/// Overlap Test Using Orientation Predicates," *Journal of Graphics Tools* 8.1
/// (2003); retaining the parameter, point, endpoint side facts, and exact
/// point-in-triangle predicate before mutating topology follows Yap, "Towards
/// Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997).
pub(super) fn lower_kernel12_events(
    events: &[ExactBoolMeshKernel12Event],
    left_mesh: &ExactMesh,
    right_mesh: &ExactMesh,
) -> ExactBoolMeshKernel12Lowering {
    let mut left = Vec::<LoweredKernel12Event>::new();
    let mut right = Vec::<LoweredKernel12Event>::new();

    for event in events {
        let lowered = match event.relation {
            SegmentPlaneRelation::ProperCrossing => lower_proper_crossing(event),
            SegmentPlaneRelation::EndpointOnPlane => {
                lower_strict_endpoint_shadow(event, left_mesh, right_mesh)
            }
            _ => None,
        };
        let Some(lowered) = lowered else {
            continue;
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

    coalesce_lowered_events(&mut left);
    coalesce_lowered_events(&mut right);

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

fn lower_proper_crossing(event: &ExactBoolMeshKernel12Event) -> Option<LoweredKernel12Event> {
    let sign = signed_plane_transition(event.endpoint_sides)?;
    let point = event.point.clone()?;
    let parameter = event.parameter.clone()?;
    event.parameter_ratio.as_ref()?;
    Some(LoweredKernel12Event {
        edge_face: event.edge_face,
        sign,
        point,
        parameter,
    })
}

fn lower_strict_endpoint_shadow(
    event: &ExactBoolMeshKernel12Event,
    left_mesh: &ExactMesh,
    right_mesh: &ExactMesh,
) -> Option<LoweredKernel12Event> {
    let sign = signed_endpoint_transition(event.endpoint_sides)?;
    let point = event.point.clone()?;
    let parameter = event.parameter.clone()?;
    if !is_endpoint_parameter(&parameter) {
        return None;
    }
    if !point_strictly_inside_opposite_face(&point, event.edge_face, left_mesh, right_mesh)? {
        return None;
    }
    Some(LoweredKernel12Event {
        edge_face: event.edge_face,
        sign,
        point,
        parameter,
    })
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

fn signed_endpoint_transition(endpoint_sides: [Option<PlaneSide>; 2]) -> Option<i32> {
    match endpoint_sides {
        [Some(PlaneSide::On), Some(PlaneSide::Above)]
        | [Some(PlaneSide::Below), Some(PlaneSide::On)] => Some(1),
        [Some(PlaneSide::On), Some(PlaneSide::Below)]
        | [Some(PlaneSide::Above), Some(PlaneSide::On)] => Some(-1),
        _ => None,
    }
}

fn is_endpoint_parameter(parameter: &super::ExactReal) -> bool {
    compare_reals(parameter, &super::ExactReal::from(0)).value() == Some(Ordering::Equal)
        || compare_reals(parameter, &super::ExactReal::from(1)).value() == Some(Ordering::Equal)
}

fn point_strictly_inside_opposite_face(
    point: &Point3,
    edge_face: ExactBoolMeshEdgeFacePair,
    left_mesh: &ExactMesh,
    right_mesh: &ExactMesh,
) -> Option<bool> {
    let face_mesh = match edge_face.face_side {
        ExactBoolMeshSide::Left => left_mesh,
        ExactBoolMeshSide::Right => right_mesh,
    };
    let triangle = face_mesh.triangles().get(edge_face.face)?.0;
    let face = [
        face_mesh.vertices().get(triangle[0])?.to_hyperlimit_point(),
        face_mesh.vertices().get(triangle[1])?.to_hyperlimit_point(),
        face_mesh.vertices().get(triangle[2])?.to_hyperlimit_point(),
    ];
    let projection = choose_triangle_projection(&face)?;
    classify_point_triangle(
        &project_point3(&face[0], projection),
        &project_point3(&face[1], projection),
        &project_point3(&face[2], projection),
        &project_point3(point, projection),
    )
    .value()
    .map(|location| location == TriangleLocation::Inside)
}

fn choose_triangle_projection(points: &[Point3; 3]) -> Option<CoplanarProjection> {
    [
        CoplanarProjection::Xy,
        CoplanarProjection::Xz,
        CoplanarProjection::Yz,
    ]
    .into_iter()
    .find(|&projection| {
        let area = projected_polygon_area2_value(points, projection);
        !matches!(real_sign(&area), Some(Sign::Zero) | None)
    })
}

fn real_sign(value: &super::ExactReal) -> Option<Sign> {
    match compare_reals(value, &super::ExactReal::from(0)).value()? {
        Ordering::Less => Some(Sign::Negative),
        Ordering::Equal => Some(Sign::Zero),
        Ordering::Greater => Some(Sign::Positive),
    }
}

/// Coalesce certified identical `Kernel12` contributions by edge/face key.
///
/// Legacy boolmesh calls `Kernel12::op` once for each `(source halfedge,
/// opposite face)` pair.  That call accumulates vertex/face and edge/edge
/// shadow terms into one signed `x12`/`x21` value and emits no row when the
/// signed sum is zero.  The exact graph may retain the same construction more
/// than once as independent segment/plane evidence, so this pass ports the
/// boolmesh accumulation shape for cases where the exact point and parameter
/// are certified identical.  Non-identical multi-shadow groups are left split
/// until the full `Kernel11` interpolation rule is ported; collapsing them to a
/// representative point would violate Yap's exact-object replay boundary.
fn coalesce_lowered_events(events: &mut Vec<LoweredKernel12Event>) {
    sort_lowered_events(events);
    let mut coalesced = Vec::new();
    let mut start = 0;
    while start < events.len() {
        let mut end = start + 1;
        while end < events.len() && events[end].edge_face == events[start].edge_face {
            end += 1;
        }

        let mut sign_sum = 0;
        let mut identical = true;
        for event in &events[start..end] {
            sign_sum += event.sign;
            if !same_lowered_construction(&events[start], event) {
                identical = false;
            }
        }

        if identical {
            if sign_sum != 0 {
                let mut event = events[start].clone();
                event.sign = sign_sum;
                coalesced.push(event);
            }
        } else {
            coalesced.extend(events[start..end].iter().cloned());
        }
        start = end;
    }
    *events = coalesced;
}

fn same_lowered_construction(left: &LoweredKernel12Event, right: &LoweredKernel12Event) -> bool {
    compare_reals(&left.parameter, &right.parameter).value() == Some(Ordering::Equal)
        && compare_reals(&left.point.x, &right.point.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.point.y, &right.point.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.point.z, &right.point.z).value() == Some(Ordering::Equal)
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::SegmentPlaneParameterRatio;
    use crate::exact::mesh::ExactMesh;

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn edge_face() -> ExactBoolMeshEdgeFacePair {
        ExactBoolMeshEdgeFacePair {
            face_pair: super::super::ExactBoolMeshFacePair {
                left_face: 0,
                right_face: 0,
            },
            edge_side: ExactBoolMeshSide::Left,
            edge: [0, 1],
            face_side: ExactBoolMeshSide::Right,
            face: 0,
        }
    }

    fn proper_event(endpoint_sides: [Option<PlaneSide>; 2]) -> ExactBoolMeshKernel12Event {
        ExactBoolMeshKernel12Event {
            edge_face: edge_face(),
            relation: SegmentPlaneRelation::ProperCrossing,
            point: Some(Point3::new(
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(1),
            )),
            parameter: Some(
                (super::super::ExactReal::from(1) / super::super::ExactReal::from(2)).unwrap(),
            ),
            parameter_ratio: Some(SegmentPlaneParameterRatio {
                numerator: super::super::ExactReal::from(1),
                denominator: super::super::ExactReal::from(2),
            }),
            construction_failure: None,
            endpoint_sides,
        }
    }

    #[test]
    fn coalesces_identical_edge_face_contributions() {
        let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
        let lowering = lower_kernel12_events(
            &[
                proper_event([Some(PlaneSide::Below), Some(PlaneSide::Above)]),
                proper_event([Some(PlaneSide::Below), Some(PlaneSide::Above)]),
            ],
            &left,
            &right,
        );

        assert_eq!(lowering.p1q2, vec![edge_face()]);
        assert_eq!(lowering.x12, vec![2]);
        assert_eq!(lowering.v12.len(), 1);
        assert_eq!(lowering.source_edge_events.len(), 1);
    }

    #[test]
    fn drops_zero_sum_identical_edge_face_contributions() {
        let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
        let lowering = lower_kernel12_events(
            &[
                proper_event([Some(PlaneSide::Below), Some(PlaneSide::Above)]),
                proper_event([Some(PlaneSide::Above), Some(PlaneSide::Below)]),
            ],
            &left,
            &right,
        );

        assert!(lowering.p1q2.is_empty());
        assert!(lowering.x12.is_empty());
        assert!(lowering.v12.is_empty());
        assert!(lowering.source_edge_events.is_empty());
    }
}
