//! Exact coplanar split rows for boolmesh `kernel12`.
//!
//! Legacy boolmesh does not split coplanar edge intervals in a separate report
//! layer: `boolean03::kernel12` still emits signed `p1q2`/`p2q1` rows, and
//! `boolean45::pair_up` then orders those rows along the source halfedge.  This
//! module ports the exact coplanar split-plan point and interval subcases into
//! that same row stream.  The row payload is exact edge-parameter evidence, not
//! a primitive-coordinate reconstruction.
//!
//! The replay boundary follows Yap, "Towards Exact Geometric Computation,"
//! *Computational Geometry* 7.1-2 (1997): the retained split points and
//! interval endpoints must carry exact source-edge parameters and source
//! halfedge ownership before they become topology rows.  The tail/head
//! convention intentionally matches the boolmesh `boolean45::pair_up` signed
//! event model.

#![allow(dead_code)]

use std::cmp::Ordering;

use hyperlimit::{Point3, SegmentIntersection, compare_reals};

use crate::exact::mesh::ExactMesh;

use super::{
    ExactBoolMeshEdgeFacePair, ExactBoolMeshFacePair, ExactBoolMeshPointConstruction,
    ExactBoolMeshSide, ExactReal, Kernel12CoplanarEvidence, normalize_boolmesh_source_edge,
};

/// One exact coplanar split point lowered as a boolmesh `kernel12` row.
#[derive(Clone, Debug, PartialEq)]
pub(super) struct ExactCoplanarKernel12Row {
    /// Boolmesh edge/face ownership for `p1q2` or `p2q1`.
    pub(super) edge_face: ExactBoolMeshEdgeFacePair,
    /// Signed `x12`/`x21` contribution.
    pub(super) sign: i32,
    /// Exact endpoint copied into `v12`/`v21`.
    pub(super) point: Point3,
    /// Exact source-edge parameter consumed by `pair_up`.
    pub(super) parameter: ExactReal,
    /// Replayable source construction for the inserted endpoint.
    pub(super) point_construction: ExactBoolMeshPointConstruction,
}

/// Lower coplanar edge split points and intervals into direct boolmesh rows.
///
/// Each certified point contact contributes a row on both participating source
/// edges.  Each certified interval contributes two rows on the left source edge
/// and two rows on the right source edge.  Within each interval source edge the
/// lower parameter is positive and the upper parameter is negative; downstream
/// this becomes a head followed by a tail, matching boolmesh's signed event
/// pairing convention after `boolean45` applies the operation coefficient.
pub(super) fn lower_coplanar_split_rows(
    evidence: &[Kernel12CoplanarEvidence],
    left: &ExactMesh,
    right: &ExactMesh,
) -> Vec<ExactCoplanarKernel12Row> {
    let mut rows = Vec::new();
    for evidence in evidence {
        let Kernel12CoplanarEvidence::Edge {
            face_pair,
            left_edge,
            right_edge,
            relation,
            points,
            interval,
            ..
        } = evidence
        else {
            continue;
        };
        match relation {
            SegmentIntersection::EndpointTouch => {
                for point in points {
                    push_point_side_row(
                        &mut rows,
                        left,
                        *face_pair,
                        ExactBoolMeshSide::Left,
                        *left_edge,
                        face_pair.left_face,
                        face_pair.right_face,
                        &point.point,
                        &point.left_parameter,
                        CoplanarPointSign::EndpointTouch,
                    );
                    push_point_side_row(
                        &mut rows,
                        right,
                        *face_pair,
                        ExactBoolMeshSide::Right,
                        *right_edge,
                        face_pair.right_face,
                        face_pair.left_face,
                        &point.point,
                        &point.right_parameter,
                        CoplanarPointSign::EndpointTouch,
                    );
                }
            }
            SegmentIntersection::Proper => {
                for point in points {
                    push_point_side_row(
                        &mut rows,
                        left,
                        *face_pair,
                        ExactBoolMeshSide::Left,
                        *left_edge,
                        face_pair.left_face,
                        face_pair.right_face,
                        &point.point,
                        &point.left_parameter,
                        CoplanarPointSign::Fixed(1),
                    );
                    push_point_side_row(
                        &mut rows,
                        right,
                        *face_pair,
                        ExactBoolMeshSide::Right,
                        *right_edge,
                        face_pair.right_face,
                        face_pair.left_face,
                        &point.point,
                        &point.right_parameter,
                        CoplanarPointSign::Fixed(1),
                    );
                }
            }
            SegmentIntersection::CollinearOverlap | SegmentIntersection::Identical => {
                let Some(interval) = interval else {
                    continue;
                };
                push_interval_side_rows(
                    &mut rows,
                    left,
                    *face_pair,
                    ExactBoolMeshSide::Left,
                    *left_edge,
                    face_pair.left_face,
                    face_pair.right_face,
                    [
                        (
                            &interval.endpoints[0].point,
                            &interval.endpoints[0].left_parameter,
                        ),
                        (
                            &interval.endpoints[1].point,
                            &interval.endpoints[1].left_parameter,
                        ),
                    ],
                );
                push_interval_side_rows(
                    &mut rows,
                    right,
                    *face_pair,
                    ExactBoolMeshSide::Right,
                    *right_edge,
                    face_pair.right_face,
                    face_pair.left_face,
                    [
                        (
                            &interval.endpoints[0].point,
                            &interval.endpoints[0].right_parameter,
                        ),
                        (
                            &interval.endpoints[1].point,
                            &interval.endpoints[1].right_parameter,
                        ),
                    ],
                );
            }
            SegmentIntersection::Disjoint => {}
        }
    }
    rows.sort_by(row_order);
    rows.dedup_by(|right, left| {
        right.edge_face == left.edge_face
            && points_equal(&right.point, &left.point)
            && compare_reals(&right.parameter, &left.parameter).value() == Some(Ordering::Equal)
            && right.sign == left.sign
    });
    rows
}

#[allow(clippy::too_many_arguments)]
fn push_point_side_row(
    rows: &mut Vec<ExactCoplanarKernel12Row>,
    mesh: &ExactMesh,
    face_pair: ExactBoolMeshFacePair,
    side: ExactBoolMeshSide,
    edge: [usize; 2],
    source_face: usize,
    opposite_face: usize,
    point: &Point3,
    parameter: &ExactReal,
    sign: CoplanarPointSign,
) {
    let Some((edge, source_face, source_halfedge, Some(parameter), _)) =
        normalize_boolmesh_source_edge(
            mesh,
            source_face,
            edge,
            Some(parameter.clone()),
            [None, None],
        )
    else {
        return;
    };
    let face_pair = match side {
        ExactBoolMeshSide::Left => ExactBoolMeshFacePair {
            left_face: source_face,
            right_face: face_pair.right_face,
        },
        ExactBoolMeshSide::Right => ExactBoolMeshFacePair {
            left_face: face_pair.left_face,
            right_face: source_face,
        },
    };
    let face_side = match side {
        ExactBoolMeshSide::Left => ExactBoolMeshSide::Right,
        ExactBoolMeshSide::Right => ExactBoolMeshSide::Left,
    };
    let sign = sign.resolve(&parameter);
    rows.push(ExactCoplanarKernel12Row {
        edge_face: ExactBoolMeshEdgeFacePair {
            face_pair,
            edge_side: side,
            source_halfedge,
            edge,
            face_side,
            face: opposite_face,
        },
        sign,
        point: point.clone(),
        parameter: parameter.clone(),
        point_construction: ExactBoolMeshPointConstruction::EdgeParameter {
            side,
            tail: edge[0],
            head: edge[1],
            parameter,
        },
    });
}

/// Signed point-row policy after boolmesh source-edge normalization.
///
/// Proper intersections keep the positive `Kernel12::op` point-row sign that
/// legacy boolmesh records before operation coefficients are applied.  Endpoint
/// touches are the degenerate coplanar counterpart of the same event stream:
/// after [`normalize_boolmesh_source_edge`] chooses the boolmesh halfedge row,
/// a touch at the normalized head is the leaving event and a touch elsewhere is
/// the entering event.  Keeping that topological sign separate from coordinate
/// equality follows Yap, "Towards Exact Geometric Computation,"
/// *Computational Geometry* 7.1-2 (1997), and preserves the signed row model
/// consumed by boolmesh `boolean45::pair_up`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum CoplanarPointSign {
    Fixed(i32),
    EndpointTouch,
}

impl CoplanarPointSign {
    fn resolve(self, parameter: &ExactReal) -> i32 {
        match self {
            Self::Fixed(sign) => sign,
            Self::EndpointTouch => {
                if compare_reals(parameter, &ExactReal::from(1)).value() == Some(Ordering::Equal) {
                    -1
                } else {
                    1
                }
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn push_interval_side_rows(
    rows: &mut Vec<ExactCoplanarKernel12Row>,
    mesh: &ExactMesh,
    face_pair: ExactBoolMeshFacePair,
    side: ExactBoolMeshSide,
    edge: [usize; 2],
    source_face: usize,
    opposite_face: usize,
    endpoints: [(&Point3, &ExactReal); 2],
) {
    let mut endpoints = endpoints.map(|(point, parameter)| {
        let normalized_parameter = if edge[0] > edge[1] {
            ExactReal::from(1) - parameter
        } else {
            parameter.clone()
        };
        (point, parameter, normalized_parameter)
    });
    endpoints.sort_by(|left, right| {
        compare_reals(&left.2, &right.2)
            .value()
            .unwrap_or(Ordering::Equal)
    });
    if compare_reals(&endpoints[0].2, &endpoints[1].2).value() != Some(Ordering::Less) {
        return;
    }
    for (index, (point, parameter, _)) in endpoints.into_iter().enumerate() {
        push_point_side_row(
            rows,
            mesh,
            face_pair,
            side,
            edge,
            source_face,
            opposite_face,
            point,
            parameter,
            CoplanarPointSign::Fixed(if index == 0 { 1 } else { -1 }),
        );
    }
}

fn row_order(left: &ExactCoplanarKernel12Row, right: &ExactCoplanarKernel12Row) -> Ordering {
    side_order(left.edge_face.edge_side)
        .cmp(&side_order(right.edge_face.edge_side))
        .then_with(|| {
            left.edge_face
                .source_halfedge
                .cmp(&right.edge_face.source_halfedge)
        })
        .then_with(|| left.edge_face.face.cmp(&right.edge_face.face))
        .then_with(|| {
            compare_reals(&left.parameter, &right.parameter)
                .value()
                .unwrap_or(Ordering::Equal)
        })
}

fn side_order(side: ExactBoolMeshSide) -> u8 {
    match side {
        ExactBoolMeshSide::Left => 0,
        ExactBoolMeshSide::Right => 1,
    }
}

fn points_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::graph::{CoplanarEdgeInterval, CoplanarEdgeSplitPoint};

    fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
        ExactMesh::from_i64_triangles(
            &[
                a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap()
    }

    fn p3(x: i64, y: i64, z: i64) -> Point3 {
        Point3::new(ExactReal::from(x), ExactReal::from(y), ExactReal::from(z))
    }

    fn split_point(
        point: Point3,
        left_parameter: i64,
        right_parameter: i64,
    ) -> CoplanarEdgeSplitPoint {
        CoplanarEdgeSplitPoint {
            point,
            left_parameter: ExactReal::from(left_parameter),
            right_parameter: ExactReal::from(right_parameter),
        }
    }

    #[test]
    fn lowers_interval_endpoints_onto_both_boolmesh_source_edges() {
        let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let right = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, -1]);
        let evidence = Kernel12CoplanarEvidence::Edge {
            face_pair: ExactBoolMeshFacePair {
                left_face: 0,
                right_face: 0,
            },
            left_edge: [1, 0],
            right_edge: [1, 0],
            relation: SegmentIntersection::CollinearOverlap,
            points: Vec::new(),
            interval: Some(CoplanarEdgeInterval {
                endpoints: [
                    split_point(p3(1, 0, 0), 0, 0),
                    split_point(p3(0, 0, 0), 1, 1),
                ],
            }),
        };

        let rows = lower_coplanar_split_rows(&[evidence], &left, &right);

        assert_eq!(rows.len(), 4);
        assert_eq!(rows[0].edge_face.edge_side, ExactBoolMeshSide::Left);
        assert_eq!(rows[0].sign, 1);
        assert_eq!(rows[1].edge_face.edge_side, ExactBoolMeshSide::Left);
        assert_eq!(rows[1].sign, -1);
        assert_eq!(rows[2].edge_face.edge_side, ExactBoolMeshSide::Right);
        assert_eq!(rows[2].sign, 1);
        assert_eq!(rows[3].edge_face.edge_side, ExactBoolMeshSide::Right);
        assert_eq!(rows[3].sign, -1);
        assert!(matches!(
            rows[0].point_construction,
            ExactBoolMeshPointConstruction::EdgeParameter { .. }
        ));
    }

    #[test]
    fn lowers_endpoint_touch_points_onto_both_boolmesh_source_edges() {
        let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let right = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, -1]);
        let evidence = Kernel12CoplanarEvidence::Edge {
            face_pair: ExactBoolMeshFacePair {
                left_face: 0,
                right_face: 0,
            },
            left_edge: [1, 0],
            right_edge: [1, 0],
            relation: SegmentIntersection::EndpointTouch,
            points: vec![split_point(p3(0, 0, 0), 1, 1)],
            interval: None,
        };

        let rows = lower_coplanar_split_rows(&[evidence], &left, &right);

        assert_eq!(rows.len(), 2);
        assert_eq!(rows[0].edge_face.edge_side, ExactBoolMeshSide::Left);
        assert_eq!(rows[1].edge_face.edge_side, ExactBoolMeshSide::Right);
        assert!(rows.iter().all(|row| row.sign == 1));
    }

    #[test]
    fn endpoint_touch_at_normalized_head_is_leaving_row() {
        let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let right = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, -1]);
        let evidence = Kernel12CoplanarEvidence::Edge {
            face_pair: ExactBoolMeshFacePair {
                left_face: 0,
                right_face: 0,
            },
            left_edge: [1, 0],
            right_edge: [1, 0],
            relation: SegmentIntersection::EndpointTouch,
            points: vec![split_point(p3(1, 0, 0), 0, 0)],
            interval: None,
        };

        let rows = lower_coplanar_split_rows(&[evidence], &left, &right);

        assert_eq!(rows.len(), 2);
        assert!(rows.iter().all(|row| row.sign == -1));
    }

    #[test]
    fn preserves_opposite_signed_rows_at_shared_interval_endpoint() {
        let left = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
        let right = tetrahedron_i64([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, -1]);
        let evidence = [
            Kernel12CoplanarEvidence::Edge {
                face_pair: ExactBoolMeshFacePair {
                    left_face: 0,
                    right_face: 0,
                },
                left_edge: [1, 0],
                right_edge: [1, 0],
                relation: SegmentIntersection::CollinearOverlap,
                points: Vec::new(),
                interval: Some(CoplanarEdgeInterval {
                    endpoints: [
                        split_point(p3(1, 0, 0), 0, 0),
                        split_point(p3(0, 0, 0), 1, 1),
                    ],
                }),
            },
            Kernel12CoplanarEvidence::Edge {
                face_pair: ExactBoolMeshFacePair {
                    left_face: 0,
                    right_face: 0,
                },
                left_edge: [1, 0],
                right_edge: [1, 0],
                relation: SegmentIntersection::Proper,
                points: vec![split_point(p3(1, 0, 0), 0, 0)],
                interval: None,
            },
        ];

        let rows = lower_coplanar_split_rows(&evidence, &left, &right);

        assert_eq!(rows.len(), 6);
        for side in [ExactBoolMeshSide::Left, ExactBoolMeshSide::Right] {
            let endpoint_rows = rows
                .iter()
                .filter(|row| {
                    row.edge_face.edge_side == side && points_equal(&row.point, &p3(1, 0, 0))
                })
                .collect::<Vec<_>>();
            assert_eq!(endpoint_rows.len(), 2);
            assert!(endpoint_rows.iter().any(|row| row.sign == 1));
            assert!(endpoint_rows.iter().any(|row| row.sign == -1));
        }
    }
}
