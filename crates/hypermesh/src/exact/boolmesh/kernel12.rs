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
use std::collections::BTreeSet;

use hyperlimit::{PlaneSide, Point3, compare_reals};

use crate::exact::mesh::ExactMesh;

use super::kernel_frame::{ExactBoolMeshKernelFrame, build_kernel_frame};
use super::kernel12_boundary::{EndpointShadowLocation, classify_endpoint_shadow};
use super::kernel12_intersect::{
    ExactKernel12IntersectHit, ExactKernel12IntersectTables, intersect12_exact,
};
use super::kernel12_op::{ExactKernel12Hit, ExactKernel12Input, kernel12_op};
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
    let left_frame = build_kernel_frame(left_mesh);
    let right_frame = build_kernel_frame(right_mesh);
    let intersect12 = intersect12_exact(left_mesh, right_mesh);
    let mut used_intersect12_hits = seed_intersect12_hits(&intersect12, &mut left, &mut right);

    for event in events {
        let lowered =
            match lower_intersect12_replay(event, &intersect12, &mut used_intersect12_hits) {
                Intersect12Replay::Lowered(lowered) => Some(lowered),
                Intersect12Replay::AlreadyConsumed => None,
                Intersect12Replay::Missing => {
                    lower_accumulator_replay(event, &left_frame, &right_frame).or_else(|| {
                        match event.relation {
                            SegmentPlaneRelation::ProperCrossing => lower_proper_crossing(event),
                            SegmentPlaneRelation::EndpointOnPlane => {
                                lower_strict_endpoint_shadow(event, left_mesh, right_mesh)
                            }
                            _ => None,
                        }
                    })
                }
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

/// Seed lowering from the exact boolmesh `intersect12` rows.
///
/// This is the direct boolmesh path: rows found by the exact broad loop and
/// `Kernel12::op` accumulator already carry the boolmesh row key, signed
/// multiplicity, witness point, and source-edge parameter.  Retained graph
/// events are still useful as fallbacks while boundary and coplanar discovery
/// are being finished, but they should not be the primary row source once the
/// accumulator loop can replay the row itself.  Trusting the row only after it
/// carries exact construction witnesses follows Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).
fn seed_intersect12_hits(
    tables: &ExactKernel12IntersectTables,
    left: &mut Vec<LoweredKernel12Event>,
    right: &mut Vec<LoweredKernel12Event>,
) -> BTreeSet<(u8, usize)> {
    let mut used = BTreeSet::new();
    for (index, hit) in tables.p1q2.iter().enumerate() {
        used.insert((0, index));
        left.push(lower_intersect12_hit(hit));
    }
    for (index, hit) in tables.p2q1.iter().enumerate() {
        used.insert((1, index));
        right.push(lower_intersect12_hit(hit));
    }
    used
}

fn lower_intersect12_hit(hit: &ExactKernel12IntersectHit) -> LoweredKernel12Event {
    LoweredKernel12Event {
        edge_face: hit.edge_face,
        sign: hit.sign,
        point: hit.point.clone(),
        parameter: hit.parameter.clone(),
    }
}

enum Intersect12Replay {
    Lowered(LoweredKernel12Event),
    AlreadyConsumed,
    Missing,
}

/// Replay a retained event through the exact boolmesh `intersect12` loop.
///
/// This is stricter than calling `Kernel12::op` directly per retained event:
/// the loop owns boolmesh's actual row cardinality.  Legacy `intersect12`
/// emits at most one row per forward source halfedge/opposite face candidate
/// after exact AABB scheduling and `Kernel12::op` accumulation.  If multiple
/// retained graph events map to the same row, only the first can consume it;
/// later matches are represented by that row and must not be re-lowered by the
/// older segment/plane shortcut.  That row-level replay follows Yap's
/// exact-object discipline from "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997): the retained event must agree with
/// the exact boolmesh accumulator point and source-edge parameter before
/// topology tables are mutated.
fn lower_intersect12_replay(
    event: &ExactBoolMeshKernel12Event,
    tables: &ExactKernel12IntersectTables,
    used_hits: &mut BTreeSet<(u8, usize)>,
) -> Intersect12Replay {
    let Some((table, side)) = intersect12_table_for_event(event, tables) else {
        return Intersect12Replay::Missing;
    };
    for (index, hit) in table.iter().enumerate() {
        if !intersect12_hit_matches_event(event, hit) {
            continue;
        }
        let key = (side, index);
        if used_hits.contains(&key) {
            return Intersect12Replay::AlreadyConsumed;
        }
        used_hits.insert(key);
        return Intersect12Replay::Lowered(LoweredKernel12Event {
            edge_face: hit.edge_face,
            sign: hit.sign,
            point: hit.point.clone(),
            parameter: hit.parameter.clone(),
        });
    }
    Intersect12Replay::Missing
}

fn intersect12_table_for_event<'a>(
    event: &ExactBoolMeshKernel12Event,
    tables: &'a ExactKernel12IntersectTables,
) -> Option<(&'a [ExactKernel12IntersectHit], u8)> {
    match (event.edge_face.edge_side, event.edge_face.face_side) {
        (ExactBoolMeshSide::Left, ExactBoolMeshSide::Right) => Some((&tables.p1q2, 0)),
        (ExactBoolMeshSide::Right, ExactBoolMeshSide::Left) => Some((&tables.p2q1, 1)),
        _ => None,
    }
}

fn intersect12_hit_matches_event(
    event: &ExactBoolMeshKernel12Event,
    hit: &ExactKernel12IntersectHit,
) -> bool {
    let Some(point) = event.point.as_ref() else {
        return false;
    };
    let Some(parameter) = event.parameter.as_ref() else {
        return false;
    };
    event.edge_face.face_pair == hit.edge_face.face_pair
        && event.edge_face.edge_side == hit.edge_face.edge_side
        && event.edge_face.face_side == hit.edge_face.face_side
        && event.edge_face.face == hit.edge_face.face
        && same_point(point, &hit.point)
        && event_parameter_matches_intersect12_hit(event.edge_face.edge, parameter, hit)
}

fn event_parameter_matches_intersect12_hit(
    event_edge: [usize; 2],
    event_parameter: &super::ExactReal,
    hit: &ExactKernel12IntersectHit,
) -> bool {
    if event_edge == hit.edge_face.edge {
        compare_reals(event_parameter, &hit.parameter).value() == Some(Ordering::Equal)
    } else if event_edge == [hit.edge_face.edge[1], hit.edge_face.edge[0]] {
        let reversed = super::ExactReal::from(1) - &hit.parameter;
        compare_reals(event_parameter, &reversed).value() == Some(Ordering::Equal)
    } else {
        false
    }
}

/// Replay one retained exact event through the ported boolmesh accumulator.
///
/// This is the first normal-workspace consumer of the exact `Kernel12::op`
/// port.  The replay keeps the boolmesh algorithmic contract: one source
/// halfedge plus one opposite face feeds the `Kernel02` endpoint shadows and
/// `Kernel11` edge shadows, and their signed sum becomes the row inserted into
/// `p1q2`/`p2q1`.  We accept the replay only when the accumulator reconstructs
/// the same exact point already retained by discovery; otherwise lowering falls
/// back to the narrower certified subcases.  The guard is deliberate Yap-style
/// exact-object replay, not a tolerance: see Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997).  The accumulator being
/// replayed is the legacy boolmesh `boolean03::kernel12` control flow derived
/// from the triangle-intersection pipeline of Moller (1997) and Guigue and
/// Devillers (2003).
fn lower_accumulator_replay(
    event: &ExactBoolMeshKernel12Event,
    left_frame: &ExactBoolMeshKernelFrame,
    right_frame: &ExactBoolMeshKernelFrame,
) -> Option<LoweredKernel12Event> {
    let point = event.point.as_ref()?;
    let parameter = event.parameter.clone()?;
    let hit = replay_kernel12_event(event, left_frame, right_frame)?;
    if !same_point(&hit.point, point) {
        return None;
    }
    Some(LoweredKernel12Event {
        edge_face: event.edge_face,
        sign: hit.sign,
        point: hit.point,
        parameter,
    })
}

fn replay_kernel12_event(
    event: &ExactBoolMeshKernel12Event,
    left_frame: &ExactBoolMeshKernelFrame,
    right_frame: &ExactBoolMeshKernelFrame,
) -> Option<ExactKernel12Hit> {
    let (source_frame, source_face, opposite_face, fwd) =
        match (event.edge_face.edge_side, event.edge_face.face_side) {
            (ExactBoolMeshSide::Left, ExactBoolMeshSide::Right) => (
                left_frame,
                event.edge_face.face_pair.left_face,
                event.edge_face.face,
                true,
            ),
            (ExactBoolMeshSide::Right, ExactBoolMeshSide::Left) => (
                right_frame,
                event.edge_face.face_pair.right_face,
                event.edge_face.face,
                false,
            ),
            _ => return None,
        };
    let source_halfedge =
        source_frame.source_halfedge_for_face_edge(source_face, event.edge_face.edge)?;
    let expand = super::ExactReal::from(1);
    let input = ExactKernel12Input {
        ps_p: &left_frame.points,
        ps_q: &right_frame.points,
        hs_p: &left_frame.halfedges,
        hs_q: &right_frame.halfedges,
        ns_p: &left_frame.expansion_normals,
        ns_q: &right_frame.expansion_normals,
        expand: &expand,
        fwd,
    };
    kernel12_op(&input, source_halfedge, opposite_face)
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
    if classify_endpoint_shadow(&point, event.edge_face, left_mesh, right_mesh)?
        != EndpointShadowLocation::StrictInterior
    {
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
        && same_point(&left.point, &right.point)
}

fn same_point(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

fn sort_lowered_events(events: &mut [LoweredKernel12Event]) {
    events.sort_by(|left, right| {
        (
            left.edge_face.source_halfedge,
            left.edge_face.edge[0],
            left.edge_face.edge[1],
            left.edge_face.face,
            left.edge_face.face_pair.left_face,
            left.edge_face.face_pair.right_face,
        )
            .cmp(&(
                right.edge_face.source_halfedge,
                right.edge_face.edge[0],
                right.edge_face.edge[1],
                right.edge_face.face,
                right.edge_face.face_pair.left_face,
                right.edge_face.face_pair.right_face,
            ))
    });
}

#[cfg(feature = "internal-fuzzing")]
pub(super) fn internal_fuzz_probe(selector: u8) -> bool {
    let top = 5 + i64::from(selector % 2);
    let left = ExactMesh::new_with_policy(
        vec![
            crate::exact::mesh::ExactPoint3::new(
                super::ExactReal::from(1),
                super::ExactReal::from(1),
                super::ExactReal::from(0),
            ),
            crate::exact::mesh::ExactPoint3::new(
                super::ExactReal::from(1),
                super::ExactReal::from(1),
                super::ExactReal::from(top),
            ),
            crate::exact::mesh::ExactPoint3::new(
                super::ExactReal::from(2),
                super::ExactReal::from(1),
                super::ExactReal::from(0),
            ),
        ],
        vec![crate::exact::mesh::Triangle([0, 1, 2])],
        crate::exact::SourceProvenance::exact("exact boolmesh accumulator replay fuzz fixture"),
        crate::exact::validation::ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic accumulator replay source fixture must import");
    let right = ExactMesh::new_with_policy(
        vec![
            crate::exact::mesh::ExactPoint3::new(
                super::ExactReal::from(0),
                super::ExactReal::from(0),
                super::ExactReal::from(4),
            ),
            crate::exact::mesh::ExactPoint3::new(
                super::ExactReal::from(4),
                super::ExactReal::from(0),
                super::ExactReal::from(4),
            ),
            crate::exact::mesh::ExactPoint3::new(
                super::ExactReal::from(0),
                super::ExactReal::from(4),
                super::ExactReal::from(4),
            ),
        ],
        vec![crate::exact::mesh::Triangle([0, 1, 2])],
        crate::exact::SourceProvenance::exact("exact boolmesh accumulator replay face fixture"),
        crate::exact::validation::ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic accumulator replay opposite fixture must import");
    let event = ExactBoolMeshKernel12Event {
        edge_face: ExactBoolMeshEdgeFacePair {
            face_pair: super::ExactBoolMeshFacePair {
                left_face: 0,
                right_face: 0,
            },
            edge_side: ExactBoolMeshSide::Left,
            source_halfedge: 0,
            edge: [0, 1],
            face_side: ExactBoolMeshSide::Right,
            face: 0,
        },
        relation: SegmentPlaneRelation::ProperCrossing,
        point: Some(Point3::new(
            super::ExactReal::from(1),
            super::ExactReal::from(1),
            super::ExactReal::from(4),
        )),
        parameter: Some((super::ExactReal::from(4) / super::ExactReal::from(top)).unwrap()),
        parameter_ratio: None,
        construction_failure: None,
        endpoint_sides: [None, None],
    };
    let lowering = lower_kernel12_events(&[event], &left, &right);
    lowering.p1q2.len() == 1
        && lowering.p2q1.is_empty()
        && lowering.x12 == vec![1]
        && lowering.source_edge_events.len() == 1
        && same_point(
            &lowering.v12[0],
            &Point3::new(
                super::ExactReal::from(1),
                super::ExactReal::from(1),
                super::ExactReal::from(4),
            ),
        )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::SegmentPlaneParameterRatio;
    use crate::exact::SourceProvenance;
    use crate::exact::mesh::{ExactMesh, ExactPoint3, Triangle};
    use crate::exact::validation::ValidationPolicy;

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
            source_halfedge: 0,
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

    fn endpoint_event(
        point: Point3,
        parameter: super::super::ExactReal,
        endpoint_sides: [Option<PlaneSide>; 2],
    ) -> ExactBoolMeshKernel12Event {
        ExactBoolMeshKernel12Event {
            edge_face: edge_face(),
            relation: SegmentPlaneRelation::EndpointOnPlane,
            point: Some(point),
            parameter: Some(parameter),
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides,
        }
    }

    fn open_accumulator_replay_meshes() -> (ExactMesh, ExactMesh) {
        let left = ExactMesh::new_with_policy(
            vec![
                ExactPoint3::new(
                    super::super::ExactReal::from(1),
                    super::super::ExactReal::from(1),
                    super::super::ExactReal::from(0),
                ),
                ExactPoint3::new(
                    super::super::ExactReal::from(1),
                    super::super::ExactReal::from(1),
                    super::super::ExactReal::from(5),
                ),
                ExactPoint3::new(
                    super::super::ExactReal::from(2),
                    super::super::ExactReal::from(1),
                    super::super::ExactReal::from(5),
                ),
            ],
            vec![Triangle([0, 1, 2])],
            SourceProvenance::exact("exact boolmesh accumulator replay test source"),
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::new_with_policy(
            vec![
                ExactPoint3::new(
                    super::super::ExactReal::from(0),
                    super::super::ExactReal::from(0),
                    super::super::ExactReal::from(4),
                ),
                ExactPoint3::new(
                    super::super::ExactReal::from(4),
                    super::super::ExactReal::from(0),
                    super::super::ExactReal::from(4),
                ),
                ExactPoint3::new(
                    super::super::ExactReal::from(0),
                    super::super::ExactReal::from(4),
                    super::super::ExactReal::from(4),
                ),
            ],
            vec![Triangle([0, 1, 2])],
            SourceProvenance::exact("exact boolmesh accumulator replay test opposite"),
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        (left, right)
    }

    fn empty_mesh() -> ExactMesh {
        ExactMesh::new_with_policy(
            Vec::new(),
            Vec::new(),
            SourceProvenance::exact("empty exact boolmesh kernel12 fallback fixture"),
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
    }

    fn accumulator_replay_event() -> ExactBoolMeshKernel12Event {
        ExactBoolMeshKernel12Event {
            edge_face: edge_face(),
            relation: SegmentPlaneRelation::ProperCrossing,
            point: Some(Point3::new(
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(4),
            )),
            parameter: Some(
                (super::super::ExactReal::from(4) / super::super::ExactReal::from(5)).unwrap(),
            ),
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides: [None, None],
        }
    }

    fn reversed_accumulator_replay_event() -> ExactBoolMeshKernel12Event {
        let mut event = accumulator_replay_event();
        event.edge_face.edge = [1, 0];
        event.parameter =
            Some((super::super::ExactReal::from(1) / super::super::ExactReal::from(5)).unwrap());
        event
    }

    #[test]
    fn coalesces_identical_edge_face_contributions() {
        let left = empty_mesh();
        let right = empty_mesh();
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
        let left = empty_mesh();
        let right = empty_mesh();
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

    #[test]
    fn intersect12_loop_lowers_boundary_endpoint_shadow_rows() {
        let left = tetrahedron_i64([2, 0, 0], [2, 0, 2], [3, 1, 1], [1, 1, 1]);
        let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
        let lowering = lower_kernel12_events(
            &[endpoint_event(
                Point3::new(
                    super::super::ExactReal::from(2),
                    super::super::ExactReal::from(0),
                    super::super::ExactReal::from(0),
                ),
                super::super::ExactReal::from(0),
                [Some(PlaneSide::On), Some(PlaneSide::Above)],
            )],
            &left,
            &right,
        );

        assert!(
            !lowering.p1q2.is_empty() || !lowering.p2q1.is_empty(),
            "the exact intersect12 loop should now replay Kernel11 boundary shadow rows directly"
        );
        assert_eq!(lowering.p1q2.len(), lowering.x12.len());
        assert_eq!(lowering.p1q2.len(), lowering.v12.len());
        assert_eq!(lowering.p2q1.len(), lowering.x21.len());
        assert_eq!(lowering.p2q1.len(), lowering.v21.len());
        assert_eq!(
            lowering.source_edge_events.len(),
            lowering.p1q2.len() + lowering.p2q1.len()
        );
    }

    #[test]
    fn accumulator_replay_lowers_event_without_legacy_side_shortcut() {
        let (left, right) = open_accumulator_replay_meshes();
        let lowering = lower_kernel12_events(&[accumulator_replay_event()], &left, &right);

        assert_eq!(lowering.p1q2, vec![edge_face()]);
        assert_eq!(lowering.p2q1, Vec::new());
        assert_eq!(lowering.x12, vec![1]);
        assert!(lowering.x21.is_empty());
        assert_eq!(
            lowering.v12,
            vec![Point3::new(
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(4),
            )]
        );
        assert_eq!(lowering.source_edge_events.len(), 1);
        assert_eq!(lowering.source_edge_events[0].tail, 0);
        assert_eq!(lowering.source_edge_events[0].head, 1);
    }

    #[test]
    fn intersect12_replay_normalizes_reversed_event_edge_to_forward_row() {
        let (left, right) = open_accumulator_replay_meshes();
        let lowering = lower_kernel12_events(&[reversed_accumulator_replay_event()], &left, &right);

        assert_eq!(lowering.p1q2, vec![edge_face()]);
        assert_eq!(lowering.x12, vec![1]);
        assert_eq!(lowering.source_edge_events.len(), 1);
        assert_eq!(lowering.source_edge_events[0].tail, 0);
        assert_eq!(lowering.source_edge_events[0].head, 1);
        assert_eq!(
            compare_reals(
                &lowering.source_edge_events[0].parameter,
                &(super::super::ExactReal::from(4) / super::super::ExactReal::from(5)).unwrap()
            )
            .value(),
            Some(Ordering::Equal)
        );
    }

    #[test]
    fn intersect12_replay_does_not_duplicate_consumed_boolmesh_rows() {
        let (left, right) = open_accumulator_replay_meshes();
        let lowering = lower_kernel12_events(
            &[
                accumulator_replay_event(),
                reversed_accumulator_replay_event(),
                accumulator_replay_event(),
            ],
            &left,
            &right,
        );

        assert_eq!(lowering.p1q2, vec![edge_face()]);
        assert_eq!(lowering.x12, vec![1]);
        assert_eq!(lowering.v12.len(), 1);
        assert_eq!(lowering.source_edge_events.len(), 1);
    }

    #[test]
    fn accumulator_replay_rejects_inconsistent_retained_point() {
        let (left, right) = open_accumulator_replay_meshes();
        let mut event = accumulator_replay_event();
        event.point = Some(Point3::new(
            super::super::ExactReal::from(1),
            super::super::ExactReal::from(1),
            super::super::ExactReal::from(3),
        ));
        let lowering = lower_kernel12_events(&[event], &left, &right);

        assert_eq!(lowering.p1q2, vec![edge_face()]);
        assert!(lowering.p2q1.is_empty());
        assert_eq!(lowering.x12, vec![1]);
        assert!(lowering.x21.is_empty());
        assert_eq!(
            lowering.v12,
            vec![Point3::new(
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(1),
                super::super::ExactReal::from(4),
            )]
        );
        assert!(lowering.v21.is_empty());
        assert_eq!(lowering.source_edge_events.len(), 1);
    }
}
