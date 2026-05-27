#![cfg(feature = "exact")]

use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::Point3;
use hypermesh::exact::{
    CoplanarArrangementOperation, ExactMesh, ExactPoint3, ExactReportValidationError,
    FaceRegionPlaneRelation, MeshArtifactBlocker, MeshArtifactFaceRecord, MeshArtifactManifest,
    MeshArtifactRole, MeshArtifactSourceKind, MeshArtifactVertexRecord, MeshCoordinateEvidence,
    MeshFacePairClassification, MeshNumericAdapterContract, MeshTopologyEvidence, PredicateUse,
    SegmentPlaneRelation, SourceProvenance, Triangle, ValidationPolicy,
    arrange_coplanar_affine_surface_difference, arrange_coplanar_affine_surface_intersection,
    arrange_coplanar_affine_surface_union,
    arrange_coplanar_convex_surface_component_holed_difference,
    arrange_coplanar_convex_surface_component_union, arrange_coplanar_convex_surface_difference,
    arrange_coplanar_convex_surface_holed_difference, arrange_coplanar_convex_surface_intersection,
    arrange_coplanar_convex_surface_multi_difference,
    arrange_coplanar_convex_surface_multi_holed_difference,
    arrange_coplanar_convex_surface_multi_union, arrange_coplanar_convex_surface_union,
    arrange_coplanar_orthogonal_surface_difference,
    arrange_coplanar_orthogonal_surface_intersection, arrange_coplanar_orthogonal_surface_union,
    arrange_coplanar_surface_component_difference,
    arrange_coplanar_surface_component_holed_difference,
    arrange_coplanar_surface_component_holed_intersection,
    arrange_coplanar_surface_component_holed_union,
    arrange_coplanar_surface_component_intersection, arrange_coplanar_surface_component_union,
    arrange_coplanar_surface_cutter_hole_contact_difference,
    arrange_coplanar_surface_multi_component_intersection,
    arrange_coplanar_surface_multi_component_union, arrange_coplanar_surface_multi_difference,
    arrange_coplanar_surface_point_touch_difference, arrange_coplanar_surface_point_touch_union,
    arrange_coplanar_surface_side_cutter_difference,
    arrange_single_triangle_coplanar_holed_difference, arrange_single_triangle_coplanar_union,
    audit_exact_mesh, build_intersection_graph, certify_boundary_touching_report,
    certify_convex_solid, certify_coplanar_convex_surface_containment,
    certify_coplanar_convex_surface_equivalence, certify_coplanar_convex_surface_report,
    certify_coplanar_surface_boundary_touch, certify_coplanar_surface_mesh_containment,
    certify_coplanar_volumetric_cell_evidence, certify_exact_mesh_proposal,
    certify_open_surface_disjoint_report, certify_planar_arrangement_evidence,
    certify_planar_arrangement_report, certify_refinement_report, certify_same_surface_report,
    certify_single_triangle_coplanar_containment,
    certify_single_triangle_coplanar_containment_report, certify_winding_readiness_report,
    checked_classify_face_regions_against_opposite_planes, classify_coplanar_triangles,
    classify_mesh_face_pair, classify_mesh_face_pairs,
    classify_mesh_triangle_against_retained_face_plane,
    classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, classify_triangle_triangle,
    difference_single_triangle_coplanar_surfaces, inspect_f64_mesh_input, inspect_i64_mesh_input,
    intersect_closed_convex_solids, intersect_segment_with_face_plane,
    intersect_segment_with_retained_face_plane, intersect_single_triangle_coplanar_surfaces,
    mesh_artifact_from_exact_mesh, mesh_artifact_from_exact_mesh_proposal,
    subtract_closed_convex_solids_single_cap, union_single_triangle_coplanar_surfaces,
};
use hyperreal::Real;

#[cfg(feature = "exact-triangulation")]
fn rect_surface_i64(rectangles: &[(i64, i64, i64, i64)]) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 12);
    let mut indices = Vec::with_capacity(rectangles.len() * 6);
    for (rectangle, &(x0, y0, x1, y1)) in rectangles.iter().enumerate() {
        let base = rectangle * 4;
        coordinates.extend_from_slice(&[
            x0, y0, 0, //
            x1, y0, 0, //
            x1, y1, 0, //
            x0, y1, 0,
        ]);
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn fan_rect_surface_i64(rectangles: &[(i64, i64, i64, i64)]) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 15);
    let mut indices = Vec::with_capacity(rectangles.len() * 12);
    for (rectangle, &(x0, y0, x1, y1)) in rectangles.iter().enumerate() {
        let base = rectangle * 5;
        assert_eq!((x0 + x1) % 2, 0);
        assert_eq!((y0 + y1) % 2, 0);
        let cx = (x0 + x1) / 2;
        let cy = (y0 + y1) / 2;
        coordinates.extend_from_slice(&[x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y1, 0, cx, cy, 0]);
        indices.extend_from_slice(&[
            base,
            base + 1,
            base + 4,
            base + 1,
            base + 2,
            base + 4,
            base + 2,
            base + 3,
            base + 4,
            base + 3,
            base,
            base + 4,
        ]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn affine_rect_surface_i64(
    rectangles: &[(i64, i64, i64, i64)],
    origin: (i64, i64, i64),
    basis_u: (i64, i64, i64),
    basis_v: (i64, i64, i64),
) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 12);
    let mut indices = Vec::with_capacity(rectangles.len() * 6);
    let lift = |u: i64, v: i64| -> [i64; 3] {
        [
            origin.0 + u * basis_u.0 + v * basis_v.0,
            origin.1 + u * basis_u.1 + v * basis_v.1,
            origin.2 + u * basis_u.2 + v * basis_v.2,
        ]
    };
    for (rectangle, &(u0, v0, u1, v1)) in rectangles.iter().enumerate() {
        let base = rectangle * 4;
        for point in [lift(u0, v0), lift(u1, v0), lift(u1, v1), lift(u0, v1)] {
            coordinates.extend_from_slice(&point);
        }
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn affine_fan_rect_surface_i64(
    rectangles: &[(i64, i64, i64, i64)],
    origin: (i64, i64, i64),
    basis_u: (i64, i64, i64),
    basis_v: (i64, i64, i64),
) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 15);
    let mut indices = Vec::with_capacity(rectangles.len() * 12);
    let lift = |u: i64, v: i64| -> [i64; 3] {
        [
            origin.0 + u * basis_u.0 + v * basis_v.0,
            origin.1 + u * basis_u.1 + v * basis_v.1,
            origin.2 + u * basis_u.2 + v * basis_v.2,
        ]
    };
    for (rectangle, &(u0, v0, u1, v1)) in rectangles.iter().enumerate() {
        let base = rectangle * 5;
        assert_eq!((u0 + u1) % 2, 0);
        assert_eq!((v0 + v1) % 2, 0);
        for point in [
            lift(u0, v0),
            lift(u1, v0),
            lift(u1, v1),
            lift(u0, v1),
            lift((u0 + u1) / 2, (v0 + v1) / 2),
        ] {
            coordinates.extend_from_slice(&point);
        }
        indices.extend_from_slice(&[
            base,
            base + 1,
            base + 4,
            base + 1,
            base + 2,
            base + 4,
            base + 2,
            base + 3,
            base + 4,
            base + 3,
            base,
            base + 4,
        ]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn fan_surface_mesh_from_points(points: &[Point3]) -> ExactMesh {
    let vertices = points
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = (1..points.len() - 1)
        .map(|index| Triangle([0, index, index + 1]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("bench retained-ring fan surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], apex[0],
            apex[1], apex[2],
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_opposite_diagonal_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], apex[0],
            apex[1], apex[2],
        ],
        &[
            0, 1, 3, 1, 2, 3, //
            0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_quad_fan_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], center[0],
            center[1], center[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4, //
            0, 5, 1, 1, 5, 2, 2, 5, 3, 3, 5, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_two_branch_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    p: [i64; 3],
    q: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], p[0], p[1],
            p[2], q[0], q[1], q[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 1, 4, 1, 5, 4, 1, 2, 5, 2, 3, 5, 3, 4, 5, 3, 0, 4, //
            0, 6, 1, 1, 6, 2, 2, 6, 3, 3, 6, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn l_prism_pair_i64() -> (ExactMesh, ExactMesh) {
    let vertices = &[
        0, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0, 2, 4, 0, 0, 4, 0, 0, 0, 5, 4, 0, 5, 4, 2, 5, 2, 2, 5,
        2, 4, 5, 0, 4, 5,
    ];
    let left = ExactMesh::from_i64_triangles(
        vertices,
        &[
            0, 3, 1, 1, 3, 2, 0, 5, 3, 3, 5, 4, //
            6, 7, 8, 6, 8, 9, 6, 9, 11, 9, 10, 11, //
            0, 1, 7, 0, 7, 6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, //
            3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5, 0, 6, 5, 6, 11,
        ],
    )
    .unwrap();
    let right_vertices = &[
        0, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0, 2, 4, 0, 0, 4, 0, 0, 0, -5, 4, 0, -5, 4, 2, -5, 2, 2,
        -5, 2, 4, -5, 0, 4, -5,
    ];
    let right = ExactMesh::from_i64_triangles(
        right_vertices,
        &[
            0, 1, 2, 0, 2, 3, 0, 3, 5, 3, 4, 5, //
            6, 9, 7, 7, 9, 8, 6, 11, 9, 9, 11, 10, //
            0, 7, 1, 0, 6, 7, 1, 8, 2, 1, 7, 8, 2, 9, 3, 2, 8, 9, //
            3, 10, 4, 3, 9, 10, 4, 11, 5, 4, 10, 11, 5, 6, 0, 5, 11, 6,
        ],
    )
    .unwrap();
    (left, right)
}

#[cfg(feature = "exact-triangulation")]
fn upward_l_prism_i64(points: [[i64; 2]; 6], top_z: i64) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            0,
            points[1][0],
            points[1][1],
            0,
            points[2][0],
            points[2][1],
            0,
            points[3][0],
            points[3][1],
            0,
            points[4][0],
            points[4][1],
            0,
            points[5][0],
            points[5][1],
            0,
            points[0][0],
            points[0][1],
            top_z,
            points[1][0],
            points[1][1],
            top_z,
            points[2][0],
            points[2][1],
            top_z,
            points[3][0],
            points[3][1],
            top_z,
            points[4][0],
            points[4][1],
            top_z,
            points[5][0],
            points[5][1],
            top_z,
        ],
        &[
            0, 3, 1, 1, 3, 2, 0, 5, 3, 3, 5, 4, //
            6, 7, 8, 6, 8, 9, 6, 9, 11, 9, 10, 11, //
            0, 1, 7, 0, 7, 6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, //
            3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5, 0, 6, 5, 6, 11,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upward_pentagonal_pyramid_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    e: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], e[0], e[1],
            e[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, //
            0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 0, 5,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_pentagonal_pyramid_fan_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    e: [i64; 3],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], e[0], e[1],
            e[2], center[0], center[1], center[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 0, 5, //
            0, 6, 1, 1, 6, 2, 2, 6, 3, 3, 6, 4, 4, 6, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upward_hexagonal_pyramid_i64(points: [[i64; 3]; 6], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, //
            0, 1, 6, 1, 2, 6, 2, 3, 6, 3, 4, 6, 4, 5, 6, 5, 0, 6,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_hexagonal_pyramid_fan_i64(
    points: [[i64; 3]; 6],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 6, 1, 2, 6, 2, 3, 6, 3, 4, 6, 4, 5, 6, 5, 0, 6, //
            0, 7, 1, 1, 7, 2, 2, 7, 3, 3, 7, 4, 4, 7, 5, 5, 7, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upward_heptagonal_pyramid_i64(points: [[i64; 3]; 7], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, 0, 6, 5, //
            0, 1, 7, 1, 2, 7, 2, 3, 7, 3, 4, 7, 4, 5, 7, 5, 6, 7, 6, 0, 7,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_heptagonal_pyramid_fan_i64(
    points: [[i64; 3]; 7],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 7, 1, 2, 7, 2, 3, 7, 3, 4, 7, 4, 5, 7, 5, 6, 7, 6, 0, 7, //
            0, 8, 1, 1, 8, 2, 2, 8, 3, 3, 8, 4, 4, 8, 5, 5, 8, 6, 6, 8, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upward_octagonal_pyramid_i64(points: [[i64; 3]; 8], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, 0, 6, 5, 0, 7, 6, //
            0, 1, 8, 1, 2, 8, 2, 3, 8, 3, 4, 8, 4, 5, 8, 5, 6, 8, 6, 7, 8, 7, 0, 8,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_octagonal_pyramid_fan_i64(
    points: [[i64; 3]; 8],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 8, 1, 2, 8, 2, 3, 8, 3, 4, 8, 4, 5, 8, 5, 6, 8, 6, 7, 8, 7, 0, 8, //
            0, 9, 1, 1, 9, 2, 2, 9, 3, 3, 9, 4, 4, 9, 5, 5, 9, 6, 6, 9, 7, 7, 9, 0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upward_nonagonal_pyramid_i64(points: [[i64; 3]; 9], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            points[8][0],
            points[8][1],
            points[8][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, 0, 6, 5, 0, 7, 6, 0, 8, 7, //
            0, 1, 9, 1, 2, 9, 2, 3, 9, 3, 4, 9, 4, 5, 9, 5, 6, 9, 6, 7, 9, 7, 8, 9, 8, 0, 9,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_nonagonal_pyramid_fan_i64(
    points: [[i64; 3]; 9],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            points[8][0],
            points[8][1],
            points[8][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 9, 1, 2, 9, 2, 3, 9, 3, 4, 9, 4, 5, 9, 5, 6, 9, 6, 7, 9, 7, 8, 9, 8, 0,
            9, //
            0, 10, 1, 1, 10, 2, 2, 10, 3, 3, 10, 4, 4, 10, 5, 5, 10, 6, 6, 10, 7, 7, 10, 8, 8, 10,
            0,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upward_polygonal_pyramid_i64(points: &[[i64; 3]], apex: [i64; 3]) -> ExactMesh {
    assert!(points.len() >= 3);
    let apex_index = points.len();
    let mut coordinates = Vec::with_capacity((points.len() + 1) * 3);
    for point in points {
        coordinates.extend_from_slice(point);
    }
    coordinates.extend_from_slice(&apex);
    let mut indices = Vec::with_capacity((points.len() - 2 + points.len()) * 3);
    for index in 1..points.len() - 1 {
        indices.extend([0, index + 1, index]);
    }
    for index in 0..points.len() {
        indices.extend([index, (index + 1) % points.len(), apex_index]);
    }
    ExactMesh::from_i64_triangles(&coordinates, &indices).unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn downward_polygonal_pyramid_fan_i64(
    points: &[[i64; 3]],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    assert!(points.len() >= 3);
    let center_index = points.len();
    let apex_index = points.len() + 1;
    let mut coordinates = Vec::with_capacity((points.len() + 2) * 3);
    for point in points {
        coordinates.extend_from_slice(point);
    }
    coordinates.extend_from_slice(&center);
    coordinates.extend_from_slice(&apex);
    let mut indices = Vec::with_capacity(points.len() * 6);
    for index in 0..points.len() {
        indices.extend([index, (index + 1) % points.len(), center_index]);
    }
    for index in 0..points.len() {
        indices.extend([index, apex_index, (index + 1) % points.len()]);
    }
    ExactMesh::from_i64_triangles(&coordinates, &indices).unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upward_square_pyramid_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], apex[0],
            apex[1], apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, //
            0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn combine_exact_meshes(meshes: &[ExactMesh], label: &'static str) -> ExactMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for mesh in meshes {
        let offset = vertices.len();
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(mesh.triangles().iter().map(|triangle| {
            Triangle([
                triangle.0[0] + offset,
                triangle.0[1] + offset,
                triangle.0[2] + offset,
            ])
        }));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::CLOSED,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn combine_open_exact_meshes(meshes: &[ExactMesh], label: &'static str) -> ExactMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for mesh in meshes {
        let offset = vertices.len();
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(mesh.triangles().iter().map(|triangle| {
            Triangle([
                triangle.0[0] + offset,
                triangle.0[1] + offset,
                triangle.0[2] + offset,
            ])
        }));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn base_fan_tetrahedron_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    center: [i64; 3],
    d: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], center[0], center[1], center[2],
            d[0], d[1], d[2],
        ],
        &[0, 1, 3, 1, 2, 3, 2, 0, 3, 0, 2, 4, 2, 1, 4, 1, 0, 4],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn upper_base_fan_tetrahedron_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    center: [i64; 3],
    d: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], center[0], center[1], center[2],
            d[0], d[1], d[2],
        ],
        &[0, 3, 1, 1, 3, 2, 2, 3, 0, 0, 1, 4, 1, 2, 4, 2, 0, 4],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn affine_box_i64(
    min: [i64; 3],
    max: [i64; 3],
    origin: [i64; 3],
    basis_u: [i64; 3],
    basis_v: [i64; 3],
    basis_w: [i64; 3],
) -> ExactMesh {
    let corners = [
        [min[0], min[1], min[2]],
        [max[0], min[1], min[2]],
        [max[0], max[1], min[2]],
        [min[0], max[1], min[2]],
        [min[0], min[1], max[2]],
        [max[0], min[1], max[2]],
        [max[0], max[1], max[2]],
        [min[0], max[1], max[2]],
    ];
    let mut coordinates = Vec::with_capacity(24);
    for [u, v, w] in corners {
        coordinates.extend_from_slice(&[
            origin[0] + u * basis_u[0] + v * basis_v[0] + w * basis_w[0],
            origin[1] + u * basis_u[1] + v * basis_v[1] + w * basis_w[1],
            origin[2] + u * basis_u[2] + v * basis_v[2] + w * basis_w[2],
        ]);
    }
    ExactMesh::from_i64_triangles(
        &coordinates,
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn top_subdivided_axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    let mid_x = (min[0] + max[0]) / 2;
    let mid_y = (min[1] + max[1]) / 2;
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2], mid_x, mid_y, max[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 8, 5, 6, 8, 6, 7, 8, 7, 4, 8, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6,
            5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .unwrap()
}

fn exact_tetrahedron_validation(c: &mut Criterion) {
    let pos = vec![
        0.0, 0.0, 0.0, //
        1.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, //
        0.0, 0.0, 1.0,
    ];
    let idx = vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];

    c.bench_function("exact_tetrahedron_validation", |b| {
        b.iter(|| {
            let input_report = inspect_f64_mesh_input(&pos, &idx);
            let input_validation = input_report.validate();
            let input_readiness = input_report.readiness();
            let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
            let points = mesh
                .vertices()
                .iter()
                .map(|point| point.to_hyperlimit_point())
                .collect::<Vec<_>>();
            let triangles = mesh
                .triangles()
                .iter()
                .map(|triangle| triangle.0)
                .collect::<Vec<_>>();
            let validation = mesh.facts().validate();
            let source_validation = mesh.facts().validate_against_sources(&points, &triangles);
            let source_provenance_validation = mesh.provenance().source.validate();
            let predicate_validations = mesh
                .provenance()
                .predicates
                .iter()
                .map(|predicate| predicate.validate())
                .collect::<Vec<_>>();
            let provenance_validation = mesh.provenance().validate();
            let retained_state = mesh.validate_retained_state();
            let audit = audit_exact_mesh(&mesh);
            let audit_validation = match &audit {
                Ok(report) => report.validate_against_mesh(&mesh).is_ok(),
                Err(_) => false,
            };
            let audit_freshness = match &audit {
                Ok(report) => Some(report.freshness_against_mesh(&mesh)),
                Err(_) => None,
            };
            let approximate_view = mesh.approximate_f64_view();
            let consumer_readiness = mesh.consumer_readiness();
            let handoff_package = mesh.handoff_package();
            let handoff_package_validation = match &handoff_package {
                Ok(package) => package.validate_against_mesh(&mesh).is_ok(),
                Err(_) => false,
            };
            let handoff_package_internal_validation = match &handoff_package {
                Ok(package) => package.validate_internal().is_ok(),
                Err(_) => false,
            };
            let handoff_package_freshness = match &handoff_package {
                Ok(package) => Some(package.freshness_against_mesh(&mesh)),
                Err(_) => None,
            };
            let handoff_package_surface_domain = match &handoff_package {
                Ok(package) => package
                    .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Surface)
                    .is_ok(),
                Err(_) => false,
            };
            let handoff_package_available_domains = match &handoff_package {
                Ok(package) => package.available_domains(),
                Err(_) => Vec::new(),
            };
            let handoff_package_exact_domains = match &handoff_package {
                Ok(package) => package.exact_geometry_domains(),
                Err(_) => Vec::new(),
            };
            let handoff_package_lossy_domains = match &handoff_package {
                Ok(package) => package.lossy_adapter_domains(),
                Err(_) => Vec::new(),
            };
            let handoff_package_domain_summary = match &handoff_package {
                Ok(package) => Some(package.domain_summary()),
                Err(_) => None,
            };
            let handoff_package_domain_summary_validation =
                match (&handoff_package, &handoff_package_domain_summary) {
                    (Ok(package), Some(summary)) => {
                        summary.validate_against_package(package).is_ok()
                    }
                    _ => false,
                };
            let handoff_package_domain_summary_mesh_validation =
                match (&handoff_package, &handoff_package_domain_summary) {
                    (Ok(package), Some(summary)) => {
                        summary.validate_against_mesh(package, &mesh).is_ok()
                    }
                    _ => false,
                };
            let handoff_package_domain_summary_flags =
                match (&handoff_package, &handoff_package_domain_summary) {
                    (Ok(package), Some(summary)) => Some((
                        summary.has_exact_geometry(),
                        summary.has_lossy_adapter(),
                        summary.has_domain(hypermesh::exact::ExactMeshConsumerDomain::Surface),
                        summary.require_exact_geometry().is_ok(),
                        summary.require_lossy_adapter().is_ok(),
                        summary.preferred_exact_geometry_domain(),
                        summary.require_preferred_exact_geometry_domain().ok(),
                        summary
                            .require_preferred_exact_geometry_domain_against_package(package)
                            .ok(),
                        summary
                            .require_preferred_exact_geometry_domain_against_mesh(package, &mesh)
                            .ok(),
                        summary
                            .preferred_exact_geometry_report_against_package(package)
                            .map(|report| report.domain())
                            .ok(),
                        summary
                            .preferred_exact_geometry_report_against_mesh(package, &mesh)
                            .map(|report| report.domain())
                            .ok(),
                        summary
                            .require_domain_against_package(
                                package,
                                hypermesh::exact::ExactMeshConsumerDomain::Surface,
                            )
                            .is_ok(),
                        summary
                            .require_domain_against_mesh(
                                package,
                                &mesh,
                                hypermesh::exact::ExactMeshConsumerDomain::Surface,
                            )
                            .is_ok(),
                    )),
                    _ => None,
                };
            let handoff_package_surface_domain_replay = match &handoff_package {
                Ok(package) => package
                    .require_domain_against_mesh(
                        &mesh,
                        hypermesh::exact::ExactMeshConsumerDomain::Surface,
                    )
                    .is_ok(),
                Err(_) => false,
            };
            let handoff_package_surface_report = match &handoff_package {
                Ok(package) => package
                    .domain_report_against_mesh(
                        &mesh,
                        hypermesh::exact::ExactMeshConsumerDomain::Surface,
                    )
                    .map(|report| {
                        (
                            report.domain(),
                            report.domain().is_exact_geometry(),
                            report.domain().is_lossy_adapter(),
                            report.audit().face_count,
                            report.audit().vertex_count,
                        )
                    })
                    .ok(),
                Err(_) => None,
            };
            let handoff_package_solid_domain = match &handoff_package {
                Ok(package) => package
                    .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid)
                    .is_ok(),
                Err(_) => false,
            };
            let consumer_readiness_validation = match &consumer_readiness {
                Ok(report) => report.validate_against_mesh(&mesh).is_ok(),
                Err(_) => false,
            };
            let consumer_readiness_freshness = match &consumer_readiness {
                Ok(report) => Some(report.freshness_against_mesh(&mesh)),
                Err(_) => None,
            };
            let surface_handoff = mesh.surface_handoff();
            let surface_handoff_validation = match &surface_handoff {
                Ok(handoff) => handoff.validate_against_mesh(&mesh).is_ok(),
                Err(_) => false,
            };
            let surface_handoff_freshness = match &surface_handoff {
                Ok(handoff) => Some(handoff.freshness_against_mesh(&mesh)),
                Err(_) => None,
            };
            let approximate_view_validation = match &approximate_view {
                Ok(view) => view.validate_against_mesh(&mesh).is_ok(),
                Err(_) => false,
            };
            let approximate_view_freshness = match &approximate_view {
                Ok(view) => Some(view.freshness_against_mesh(&mesh)),
                Err(_) => None,
            };
            (
                mesh,
                validation,
                source_validation,
                source_provenance_validation,
                predicate_validations,
                provenance_validation,
                retained_state,
                audit,
                audit_validation,
                audit_freshness,
                input_report,
                input_validation,
                input_readiness,
                consumer_readiness,
                handoff_package,
                consumer_readiness_validation,
                consumer_readiness_freshness,
                handoff_package_validation,
                handoff_package_internal_validation,
                handoff_package_freshness,
                handoff_package_surface_domain,
                handoff_package_available_domains,
                handoff_package_exact_domains,
                handoff_package_lossy_domains,
                handoff_package_domain_summary,
                handoff_package_domain_summary_validation,
                handoff_package_domain_summary_mesh_validation,
                handoff_package_domain_summary_flags,
                handoff_package_surface_domain_replay,
                handoff_package_surface_report,
                handoff_package_solid_domain,
                surface_handoff,
                surface_handoff_validation,
                surface_handoff_freshness,
                approximate_view,
                approximate_view_validation,
                approximate_view_freshness,
            )
        })
    });
}

fn exact_mesh_proposal_acceptance(c: &mut Criterion) {
    let exact = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let lossy = ExactMesh::from_f64_triangles_with_policy(
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let external = ExactMesh::new_with_policy(
        vec![
            ExactPoint3::new(Real::from(0), Real::from(0), Real::from(0)),
            ExactPoint3::new(Real::from(1), Real::from(0), Real::from(0)),
            ExactPoint3::new(Real::from(0), Real::from(1), Real::from(0)),
        ],
        vec![Triangle([0, 1, 2])],
        SourceProvenance::external_adapter("bench external proposal"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_mesh_proposal_acceptance", |b| {
        b.iter(|| {
            let exact_report = certify_exact_mesh_proposal(&exact).unwrap();
            let lossy_report = certify_exact_mesh_proposal(&lossy).unwrap();
            let external_report = certify_exact_mesh_proposal(&external).unwrap();
            (
                exact_report.validate_against_mesh(&exact),
                lossy_report.validate_against_mesh(&lossy),
                external_report.validate_against_mesh(&external),
                exact_report,
                lossy_report,
                external_report,
            )
        })
    });
    c.bench_function("shared_mesh_artifact_report", |b| {
        b.iter(|| {
            let exact_report = mesh_artifact_from_exact_mesh(&exact).unwrap();
            let lossy_report = mesh_artifact_from_exact_mesh(&lossy).unwrap();
            let external_report = mesh_artifact_from_exact_mesh(&external).unwrap();
            let external_proposal = certify_exact_mesh_proposal(&external).unwrap();
            let proposal_report =
                mesh_artifact_from_exact_mesh_proposal(&external, &external_proposal).unwrap();
            let malicious_vertices = (0..3)
                .map(|index| MeshArtifactVertexRecord {
                    index,
                    coordinate_evidence: MeshCoordinateEvidence::CertifiedDerivedExact,
                })
                .collect::<Vec<_>>();
            let malicious_faces = vec![MeshArtifactFaceRecord {
                index: 0,
                vertices: vec![0, 1, 2],
                topology_evidence: MeshTopologyEvidence::DerivedExactSurfaceHandoff,
            }];
            let relabeled_preview = MeshArtifactManifest::new(
                MeshArtifactSourceKind::SdfSurfaceNetsPreview,
                "bench relabeled surface-nets preview",
                1,
                MeshArtifactRole::DerivedHandoff,
                MeshNumericAdapterContract::exact(MeshCoordinateEvidence::CertifiedDerivedExact),
                malicious_vertices,
                malicious_faces,
            )
            .report();
            (
                exact_report.validation_handoff_ready,
                lossy_report.numeric_contract.lossy_adapter_route,
                external_report.source_kind,
                proposal_report.validation_handoff_ready,
                relabeled_preview
                    .blockers
                    .contains(&MeshArtifactBlocker::PreviewOrExportSource),
            )
        })
    });
}

fn exact_face_plane_fact_retention(c: &mut Criterion) {
    let pos = vec![
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let idx = vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];

    c.bench_function("exact_face_plane_fact_retention", |b| {
        b.iter(|| {
            let input_report = inspect_i64_mesh_input(&pos, &idx);
            let input_validation = input_report.validate();
            let input_readiness = input_report.readiness();
            let mesh = ExactMesh::from_i64_triangles(&pos, &idx).unwrap();
            let points = mesh
                .vertices()
                .iter()
                .map(|point| point.to_hyperlimit_point())
                .collect::<Vec<_>>();
            let triangles = mesh
                .triangles()
                .iter()
                .map(|triangle| triangle.0)
                .collect::<Vec<_>>();
            mesh.facts().validate().unwrap();
            mesh.facts()
                .validate_against_sources(&points, &triangles)
                .unwrap();
            mesh.provenance().source.validate().unwrap();
            for predicate in &mesh.provenance().predicates {
                predicate.validate().unwrap();
            }
            mesh.provenance().validate().unwrap();
            mesh.validate_retained_state().unwrap();
            let audit = mesh.audit().unwrap();
            audit.validate_against_mesh(&mesh).unwrap();
            let handoff = mesh.solid_handoff().unwrap();
            let handoff_freshness = handoff.freshness_against_mesh(&mesh);
            handoff.validate_against_mesh(&mesh).unwrap();
            let surface_handoff = mesh.surface_handoff().unwrap();
            let surface_handoff_freshness = surface_handoff.freshness_against_mesh(&mesh);
            surface_handoff.validate_against_mesh(&mesh).unwrap();
            let consumer_readiness = mesh.consumer_readiness().unwrap();
            let consumer_readiness_freshness = consumer_readiness.freshness_against_mesh(&mesh);
            consumer_readiness.validate_against_mesh(&mesh).unwrap();
            let handoff_package = mesh.handoff_package().unwrap();
            let handoff_package_freshness = handoff_package.freshness_against_mesh(&mesh);
            handoff_package.validate_internal().unwrap();
            handoff_package.validate_against_mesh(&mesh).unwrap();
            let handoff_package_available_domains = handoff_package.available_domains();
            let handoff_package_exact_domains = handoff_package.exact_geometry_domains();
            let handoff_package_lossy_domains = handoff_package.lossy_adapter_domains();
            let handoff_package_domain_summary = handoff_package.domain_summary();
            handoff_package_domain_summary
                .validate_against_package(&handoff_package)
                .unwrap();
            handoff_package_domain_summary
                .validate_against_mesh(&handoff_package, &mesh)
                .unwrap();
            let handoff_package_domain_summary_flags = (
                handoff_package_domain_summary.has_exact_geometry(),
                handoff_package_domain_summary.has_lossy_adapter(),
                handoff_package_domain_summary
                    .has_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid),
                handoff_package_domain_summary
                    .require_closed_volume()
                    .is_ok(),
                handoff_package_domain_summary.preferred_exact_geometry_domain(),
                handoff_package_domain_summary
                    .require_preferred_exact_geometry_domain()
                    .ok(),
                handoff_package_domain_summary
                    .require_preferred_exact_geometry_domain_against_package(&handoff_package)
                    .ok(),
                handoff_package_domain_summary
                    .require_preferred_exact_geometry_domain_against_mesh(&handoff_package, &mesh)
                    .ok(),
                handoff_package_domain_summary
                    .preferred_exact_geometry_report_against_package(&handoff_package)
                    .map(|report| report.domain())
                    .ok(),
                handoff_package_domain_summary
                    .preferred_exact_geometry_report_against_mesh(&handoff_package, &mesh)
                    .map(|report| report.domain())
                    .ok(),
                handoff_package_domain_summary
                    .require_domain_against_mesh(
                        &handoff_package,
                        &mesh,
                        hypermesh::exact::ExactMeshConsumerDomain::Solid,
                    )
                    .is_ok(),
            );
            handoff_package
                .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Surface)
                .unwrap();
            handoff_package
                .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid)
                .unwrap();
            handoff_package
                .require_domain_against_mesh(
                    &mesh,
                    hypermesh::exact::ExactMeshConsumerDomain::Solid,
                )
                .unwrap();
            handoff_package
                .require_preferred_exact_geometry_domain()
                .unwrap();
            handoff_package
                .require_preferred_exact_geometry_domain_against_mesh(&mesh)
                .unwrap();
            let _ = handoff_package.preferred_exact_geometry_report().unwrap();
            let _ = handoff_package
                .preferred_exact_geometry_report_against_mesh(&mesh)
                .unwrap();
            let _ = handoff_package
                .domain_report_against_mesh(&mesh, hypermesh::exact::ExactMeshConsumerDomain::Solid)
                .unwrap();
            let approximate_view = mesh.approximate_f64_view().unwrap();
            let approximate_view_freshness = approximate_view.freshness_against_mesh(&mesh);
            approximate_view.validate_against_mesh(&mesh).unwrap();
            (
                input_report,
                input_validation,
                input_readiness,
                consumer_readiness,
                handoff_package,
                handoff_package_available_domains,
                handoff_package_exact_domains,
                handoff_package_lossy_domains,
                handoff_package_domain_summary,
                handoff_package_domain_summary_flags,
                approximate_view,
                surface_handoff,
                consumer_readiness_freshness,
                handoff_package_freshness,
                handoff_freshness,
                surface_handoff_freshness,
                approximate_view_freshness,
                mesh.facts()
                    .faces
                    .iter()
                    .map(|face| (face.plane.normal.clone(), face.plane.offset.clone()))
                    .collect::<Vec<_>>(),
            )
        })
    });
}

fn exact_bounds_candidate_generation(c: &mut Criterion) {
    let left = hypermesh::exact::ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = hypermesh::exact::ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 2, 0, 0, 1, 1, 0],
        &[0, 1, 2],
        hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_bounds_candidate_generation", |b| {
        b.iter(|| {
            let left_points = left
                .vertices()
                .iter()
                .map(|point| point.to_hyperlimit_point())
                .collect::<Vec<_>>();
            let left_triangles = left
                .triangles()
                .iter()
                .map(|triangle| triangle.0)
                .collect::<Vec<_>>();
            let right_points = right
                .vertices()
                .iter()
                .map(|point| point.to_hyperlimit_point())
                .collect::<Vec<_>>();
            let right_triangles = right
                .triangles()
                .iter()
                .map(|triangle| triangle.0)
                .collect::<Vec<_>>();
            let left_validation = left
                .bounds()
                .validate(left.vertices().len(), left.triangles().len());
            let left_source_validation = left
                .bounds()
                .validate_against_sources(&left_points, &left_triangles);
            let left_mesh_source_validation = left
                .bounds()
                .mesh
                .as_ref()
                .map(|bounds| bounds.validate_against_points(&left_points));
            let left_face_source_validations = left
                .bounds()
                .faces
                .iter()
                .zip(left_triangles.iter())
                .map(|(bounds, triangle)| {
                    bounds.validate_against_triangle([
                        &left_points[triangle[0]],
                        &left_points[triangle[1]],
                        &left_points[triangle[2]],
                    ])
                })
                .collect::<Vec<_>>();
            let right_validation = right
                .bounds()
                .validate(right.vertices().len(), right.triangles().len());
            let right_source_validation = right
                .bounds()
                .validate_against_sources(&right_points, &right_triangles);
            let right_mesh_source_validation = right
                .bounds()
                .mesh
                .as_ref()
                .map(|bounds| bounds.validate_against_points(&right_points));
            let right_face_source_validations = right
                .bounds()
                .faces
                .iter()
                .zip(right_triangles.iter())
                .map(|(bounds, triangle)| {
                    bounds.validate_against_triangle([
                        &right_points[triangle[0]],
                        &right_points[triangle[1]],
                        &right_points[triangle[2]],
                    ])
                })
                .collect::<Vec<_>>();
            (
                left.bounds().candidate_face_pairs(right.bounds()),
                left_validation,
                left_source_validation,
                left_mesh_source_validation,
                left_face_source_validations,
                right_validation,
                right_source_validation,
                right_mesh_source_validation,
                right_face_source_validations,
            )
        })
    });
}

fn exact_support_dop_witness_refresh(c: &mut Criterion) {
    let mesh = hypermesh::exact::ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 3, 0],
        &[0, 1, 2],
        hypermesh::exact::ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let axes = hypermesh::exact::SupportDopAxis3::kdop26_axes();
    let mut changed_points = mesh
        .vertices()
        .iter()
        .map(|point| point.to_hyperlimit_point())
        .collect::<Vec<_>>();
    changed_points[2] = Point3::new(Real::from(4), Real::from(5), Real::from(0));

    c.bench_function("exact_support_dop_witness_refresh", |b| {
        b.iter(|| {
            let mut support = hypermesh::exact::support_dop_for_mesh(&mesh, &axes).unwrap();
            let source_validation = support.validate_against_mesh(&mesh);
            let refresh = support.refresh_for_changed_vertices(&changed_points, &[2]);
            let refreshed_validation = support.validate_against_points(&changed_points);
            (support, source_validation, refresh, refreshed_validation)
        })
    });
}

fn exact_segment_plane_intersection(c: &mut Criterion) {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, -1),
        p3(0, 0, 1),
    ];

    c.bench_function("exact_segment_plane_intersection", |b| {
        b.iter(|| {
            let event = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);
            let validation = event.validate();
            let source_validation = event.validate_against_sources(
                &points[0], &points[1], &points[2], &points[3], &points[4],
            );
            (event, validation, source_validation)
        })
    });
}

fn exact_retained_segment_plane_intersection(c: &mut Criterion) {
    let plane = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let p0 = p3(0, 0, -1);
    let p1 = p3(0, 0, 1);

    c.bench_function("exact_retained_segment_plane_intersection", |b| {
        b.iter(|| {
            let event =
                intersect_segment_with_retained_face_plane(&plane.facts().faces[0].plane, &p0, &p1);
            let validation = event.validate();
            (event, validation)
        })
    });
}

fn exact_triangle_triangle_classifier(c: &mut Criterion) {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(0, 0, -1),
        p3(2, 0, 1),
        p3(0, 2, 1),
    ];

    c.bench_function("exact_triangle_triangle_classifier", |b| {
        b.iter(|| {
            let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
            let validation = classification.validate();
            let source_validation =
                classification.validate_against_sources(&points, [0, 1, 2], [3, 4, 5]);
            let plane_source_validation = classification
                .right_against_left_plane
                .validate_against_sources(&points, [0, 1, 2], [3, 4, 5]);
            (
                classification,
                validation,
                source_validation,
                plane_source_validation,
            )
        })
    });
}

fn exact_retained_face_plane_classifier(c: &mut Criterion) {
    let plane = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let query = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -2, 1, 0, -2, 0, 1, -2],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_retained_face_plane_classifier", |b| {
        b.iter(|| {
            let classification =
                classify_mesh_triangle_against_retained_face_plane(&plane, 0, &query, 0).unwrap();
            let validation = classification.validate();
            (classification, validation)
        })
    });
}

fn exact_coplanar_triangle_classifier(c: &mut Criterion) {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(1, 0, 0),
        p3(3, 0, 0),
        p3(1, 2, 0),
    ];

    c.bench_function("exact_coplanar_triangle_classifier", |b| {
        b.iter(|| {
            let classification = classify_coplanar_triangles(&points, [0, 1, 2], [3, 4, 5]);
            let validation = classification.validate();
            let source_validation =
                classification.validate_against_sources(&points, [0, 1, 2], [3, 4, 5]);
            (classification, validation, source_validation)
        })
    });
}

fn exact_mesh_face_pair_classifier(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_mesh_face_pair_classifier", |b| {
        b.iter(|| {
            let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
            let validation = classification.validate();
            let source_validation = classification.validate_against_sources(&left, &right);
            (classification, validation, source_validation)
        })
    });
}

fn exact_mesh_face_pair_retained_plane_rejection(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 2, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 2, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_mesh_face_pair_retained_plane_rejection", |b| {
        b.iter(|| {
            let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
            let validation = classification.validate();
            let source_validation = classification.validate_against_sources(&left, &right);
            (classification, validation, source_validation)
        })
    });
}

fn exact_mesh_face_pair_batch(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_mesh_face_pair_batch", |b| {
        b.iter(|| {
            let classifications = classify_mesh_face_pairs(&left, &right).unwrap();
            let validations = classifications
                .iter()
                .map(MeshFacePairClassification::validate)
                .collect::<Vec<_>>();
            let source_validations = classifications
                .iter()
                .map(|classification| classification.validate_against_sources(&left, &right))
                .collect::<Vec<_>>();
            (classifications, validations, source_validations)
        })
    });
}

fn exact_intersection_graph_events(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_intersection_graph_events", |b| {
        b.iter(|| {
            let graph = build_intersection_graph(&left, &right).unwrap();
            let validation = graph.validate();
            let source_validation = graph.validate_against_sources(&left, &right);
            let pair_source_validations = graph
                .face_pairs
                .iter()
                .map(|pair| pair.validate_against_sources(&left, &right))
                .collect::<Vec<_>>();
            (
                graph,
                validation,
                source_validation,
                pair_source_validations,
            )
        })
    });
}

fn exact_coplanar_overlap_graph_handoff(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 3, 0, 0, 1, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_coplanar_overlap_graph_handoff", |b| {
        b.iter(|| {
            let graph = build_intersection_graph(&left, &right).unwrap();
            let overlap_graphs = graph.coplanar_overlap_graphs();
            let split_plan = graph.coplanar_overlap_split_plan(&left, &right).unwrap();
            let readiness = graph
                .coplanar_arrangement_readiness_report(&left, &right)
                .unwrap();
            let mut relabeled_graph = graph.clone();
            relabeled_graph.face_pairs[0].left_face = usize::MAX;
            let validations = overlap_graphs
                .iter()
                .map(|overlap| overlap.validate())
                .collect::<Vec<_>>();
            let source_validations = overlap_graphs
                .iter()
                .map(|overlap| overlap.validate_against_sources(&left, &right))
                .collect::<Vec<_>>();
            let split_graph_source_validations = split_plan
                .graphs
                .iter()
                .map(|graph| graph.validate_against_sources(&left, &right))
                .collect::<Vec<_>>();
            (
                graph.validate(),
                graph.validate_against_meshes(&left, &right),
                graph.validate_against_sources(&left, &right),
                overlap_graphs,
                validations,
                source_validations,
                split_plan.validate_against_meshes(&left, &right),
                split_plan.validate_against_sources(&left, &right),
                split_graph_source_validations,
                readiness.validate(),
                readiness.validate_against_sources(&left, &right),
                relabeled_graph
                    .coplanar_arrangement_readiness_report(&left, &right)
                    .unwrap_err(),
                split_plan,
                readiness,
            )
        })
    });
}

fn exact_planar_arrangement_evidence(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 4, 0, 0, 0, 4, 0, //
            8, 0, 0, 12, 0, 0, 8, 4, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 0, 0, 5, 0, 0, 1, 4, 0, //
            8, 4, 0, 12, 4, 0, 8, 8, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    c.bench_function("exact_planar_arrangement_evidence", |b| {
        b.iter(|| {
            let report = certify_planar_arrangement_evidence(&left, &right).unwrap();
            let validation = report.validate();
            let source_validation = report.validate_against_sources(&left, &right);
            let freshness = report.freshness_against_sources(&left, &right);
            let needs_general_arrangement = report.obstacle.requires_general_arrangement();
            let branch_side_counts = (
                report.left_branch_point_count,
                report.right_branch_point_count,
                report.mixed_side_branch_point_count,
            );
            (
                report,
                validation,
                source_validation,
                freshness,
                needs_general_arrangement,
                branch_side_counts,
            )
        })
    });
}

fn exact_coplanar_volumetric_cell_evidence(c: &mut Criterion) {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = tetrahedron_i64([1, 1, 0], [5, 1, 0], [1, 5, 0], [1, 1, 4]);

    c.bench_function("exact_coplanar_volumetric_cell_evidence", |b| {
        b.iter(|| {
            let report = certify_coplanar_volumetric_cell_evidence(&left, &right).unwrap();
            let validation = report.validate();
            let source_validation = report.validate_against_sources(&left, &right);
            let freshness = report.freshness_against_sources(&left, &right);
            let requires_cells = report.obstacle.requires_coplanar_volumetric_cells();
            let coplanar_side_counts = (
                report.opposite_side_coplanar_overlapping_pairs,
                report.same_side_coplanar_overlapping_pairs,
                report.undecided_side_coplanar_overlapping_pairs,
            );
            (
                report,
                validation,
                source_validation,
                freshness,
                requires_cells,
                coplanar_side_counts,
            )
        })
    });
}

fn exact_graph_vertex_merge(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();

    c.bench_function("exact_graph_vertex_merge", |b| {
        b.iter(|| graph.graph_vertex_plan())
    });
    c.bench_function("exact_checked_graph_vertex_merge", |b| {
        b.iter(|| graph.checked_graph_vertex_plan().unwrap())
    });
}

fn exact_split_topology_plan(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();

    c.bench_function("exact_split_topology_plan", |b| {
        b.iter(|| graph.split_topology_plan())
    });
    c.bench_function("exact_checked_split_topology_plan", |b| {
        b.iter(|| graph.checked_split_topology_plan().unwrap())
    });
}

fn exact_face_split_plan(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();

    c.bench_function("exact_face_split_plan", |b| {
        b.iter(|| graph.face_split_plan())
    });
}

fn exact_split_plan_validation(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();
    let edge_split_plan = graph.edge_split_plan();
    let topology_plan = graph.split_topology_plan();
    let face_plan = graph.face_split_plan();

    c.bench_function("exact_split_plan_validation", |b| {
        b.iter(|| {
            (
                edge_split_plan.validate(),
                edge_split_plan.validate_against_sources(&left, &right),
                graph.graph_vertex_plan().validate(),
                graph
                    .graph_vertex_plan()
                    .validate_against_sources(&left, &right),
                topology_plan.validate(),
                topology_plan.validate_against_sources(&left, &right),
                face_plan.validate_against_topology(&topology_plan),
                face_plan.validate_against_sources(&left, &right),
            )
        })
    });
}

fn exact_face_split_geometry_plan(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();

    c.bench_function("exact_face_split_geometry_plan", |b| {
        b.iter(|| graph.face_split_geometry_plan(&left, &right).unwrap())
    });
}

fn exact_face_split_geometry_incidence(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();

    c.bench_function("exact_face_split_geometry_incidence", |b| {
        b.iter(|| {
            let incidence = geometry.validate_boundary_incidence(&left, &right);
            let source = geometry.validate_against_sources(&left, &right);
            (incidence, source)
        })
    });
}

fn exact_face_region_plan(c: &mut Criterion) {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 0, 2, 0, //
            20, 0, 0, 22, 0, 0, 20, 2, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();

    c.bench_function("exact_face_region_plan", |b| {
        b.iter(|| {
            let region_plan = geometry.region_plan(&left, &right);
            let classifications =
                checked_classify_face_regions_against_opposite_planes(&region_plan, &left, &right)
                    .unwrap();
            let classification_validations = classifications
                .iter()
                .map(|classification| classification.validate())
                .collect::<Vec<_>>();
            let classification_source_validations = classifications
                .iter()
                .map(|classification| classification.validate_against_sources(&left, &right))
                .collect::<Vec<_>>();
            (
                region_plan.graph_vertex_references(),
                {
                    let report = region_plan.validate(&left, &right);
                    let report_validation = report.validate();
                    let source_report = region_plan.validate_against_sources(&left, &right);
                    let source_report_validation = source_report.validate();
                    (
                        report,
                        report_validation,
                        source_report,
                        source_report_validation,
                    )
                },
                classifications,
                classification_validations,
                classification_source_validations,
            )
        })
    });
}

fn exact_face_region_earcut(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 0, 2, 0, //
                20, 0, 0, 22, 0, 0, 20, 2, 0,
            ],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, -1, 2, 0, 1, 0, 2, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let graph = build_intersection_graph(&left, &right).unwrap();
        let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
        let region_plan = geometry.region_plan(&left, &right);

        c.bench_function("exact_face_region_earcut", |b| {
            b.iter(|| {
                let triangulations =
                    hypermesh::exact::checked_triangulate_face_regions_with_earcut(
                        &region_plan,
                        &left,
                        &right,
                    )
                    .unwrap();
                hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
                    &triangulations,
                    hypermesh::exact::ExactRegionSelection::KeepAll,
                )
                .unwrap()
            })
        });
        c.bench_function("exact_face_region_earcut_source_replay", |b| {
            b.iter(|| {
                let triangulations =
                    hypermesh::exact::checked_triangulate_face_regions_with_earcut(
                        &region_plan,
                        &left,
                        &right,
                    )
                    .unwrap();
                triangulations
                    .iter()
                    .map(|triangulation| triangulation.validate_against_sources(&left, &right))
                    .collect::<Vec<_>>()
            })
        });
        let triangulations = hypermesh::exact::checked_triangulate_face_regions_with_earcut(
            &region_plan,
            &left,
            &right,
        )
        .unwrap();
        let assembly = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
            &triangulations,
            hypermesh::exact::ExactRegionSelection::KeepAll,
        )
        .unwrap();
        c.bench_function("exact_boolean_assembly_materialization", |b| {
            b.iter(|| {
                assembly
                    .to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
                    .unwrap()
            })
        });
        c.bench_function("exact_boolean_assembly_source_replay", |b| {
            b.iter(|| {
                assembly.validate_against_sources(
                    &left,
                    &right,
                    hypermesh::exact::ExactRegionSelection::KeepAll,
                )
            })
        });
        c.bench_function(
            "exact_boolean_assembly_source_checked_materialization",
            |b| {
                b.iter(|| {
                    assembly
                        .checked_to_exact_mesh_with_sources(
                            &left,
                            &right,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap()
                })
            },
        );
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_face_interior_steiner_provenance(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let mesh = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let point2 = |x: i64, y: i64| {
            hypertri::ExactPoint::new(
                hypermesh::exact::ExactReal::from(x),
                hypermesh::exact::ExactReal::from(y),
            )
        };
        let triangulation = hypermesh::exact::FaceRegionTriangulation {
            side: hypermesh::exact::MeshSide::Left,
            face: 0,
            projection: hypermesh::exact::CoplanarProjection::Xy,
            boundary: vec![
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(4, 0, 0),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 4, 0),
                },
                hypermesh::exact::FaceSplitBoundaryNode::FaceInterior { point: p3(1, 1, 0) },
            ],
            vertices: vec![point2(0, 0), point2(4, 0), point2(0, 4), point2(1, 1)],
            triangles: vec![0, 1, 3, 0, 3, 2],
        };
        let crossing_points = vec![point2(0, 0), point2(2, 2), point2(0, 2), point2(2, 0)];
        let crossing_constraints = vec![
            hypertri::Constraint::new(0, 1),
            hypertri::Constraint::new(2, 3),
        ];
        let target = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 12],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let boundary_centroid_triangulation = hypermesh::exact::FaceRegionTriangulation {
            side: hypermesh::exact::MeshSide::Left,
            face: 0,
            projection: hypermesh::exact::CoplanarProjection::Xy,
            boundary: vec![
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(2, 1, 1),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(14, 1, 1),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(1, 14, 1),
                },
            ],
            vertices: vec![point2(2, 1), point2(14, 1), point2(1, 14)],
            triangles: vec![0, 1, 2],
        };

        c.bench_function("exact_face_interior_steiner_provenance", |b| {
            b.iter(|| {
                let cdt = hypertri::cdt::constrained_delaunay(
                    &crossing_points,
                    &crossing_constraints,
                )
                .unwrap();
                let assembly =
                    hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
                        std::slice::from_ref(&triangulation),
                        hypermesh::exact::ExactRegionSelection::KeepAll,
                        &mesh,
                        &mesh,
                    )
                    .unwrap();
                (
                    cdt.points().len(),
                    cdt.constraint_edges().len(),
                    triangulation.validate(),
                    assembly.validate_source_face_incidence(&mesh, &mesh),
                    assembly.checked_to_exact_mesh_with_sources(
                        &mesh,
                        &mesh,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    ),
                    hypermesh::exact::classify_triangulated_region_triangle_against_closed_mesh(
                        &boundary_centroid_triangulation,
                        [0, 1, 2],
                        &target,
                    ),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_volumetric_witness_lattice(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let point2 = |x: i64, y: i64| {
            hypertri::ExactPoint::new(
                hypermesh::exact::ExactReal::from(x),
                hypermesh::exact::ExactReal::from(y),
            )
        };
        let target = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 12],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let triangulation = hypermesh::exact::FaceRegionTriangulation {
            side: hypermesh::exact::MeshSide::Left,
            face: 0,
            projection: hypermesh::exact::CoplanarProjection::Xy,
            boundary: vec![
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(2, 1, 1),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(14, 1, 1),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(1, 14, 1),
                },
            ],
            vertices: vec![point2(2, 1), point2(14, 1), point2(1, 14)],
            triangles: vec![0, 1, 2],
        };
        let exhausted_boundary = hypermesh::exact::FaceRegionTriangulation {
            side: hypermesh::exact::MeshSide::Left,
            face: 0,
            projection: hypermesh::exact::CoplanarProjection::Xy,
            boundary: vec![
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(1, 1, 0),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(5, 1, 0),
                },
                hypermesh::exact::FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(1, 5, 0),
                },
            ],
            vertices: vec![point2(1, 1), point2(5, 1), point2(1, 5)],
            triangles: vec![0, 1, 2],
        };

        c.bench_function("exact_volumetric_witness_lattice_boundary_retry", |b| {
            b.iter(|| {
                let classification =
                    hypermesh::exact::classify_triangulated_region_triangle_against_closed_mesh(
                        &triangulation,
                        [0, 1, 2],
                        &target,
                    )
                    .unwrap();
                classification.representative_witness.validate().unwrap();
                let exhausted =
                    hypermesh::exact::classify_triangulated_region_triangle_against_closed_mesh(
                        &exhausted_boundary,
                        [0, 1, 2],
                        &target,
                    )
                    .unwrap();
                assert_eq!(
                    exhausted.witness_attempts.len(),
                    hypermesh::exact::EXACT_TRIANGLE_INTERIOR_WITNESSES.len()
                );
                (
                    classification.validate_against_sources(&triangulation, &target),
                    exhausted.validate_against_sources(&exhausted_boundary, &target),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_selected_regions(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 0, 2, 0, //
                20, 0, 0, 22, 0, 0, 20, 2, 0,
            ],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, -1, 2, 0, 1, 0, 2, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_selected_regions", |b| {
            b.iter(|| {
                let result = hypermesh::exact::boolean_selected_regions(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
                )
                .unwrap();
                let validation = result.validate();
                let source_validation = result.validate_against_sources(&left, &right);
                (result, validation, source_validation)
            })
        });
        c.bench_function("exact_selected_region_source_replay_validation", |b| {
            b.iter(|| {
                let result = hypermesh::exact::boolean_selected_regions(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
                )
                .unwrap();
                result.validate_against_sources(&left, &right)
            })
        });
        c.bench_function("exact_selected_region_mesh_handoff", |b| {
            b.iter(|| {
                let mesh = hypermesh::exact::build_selected_region_mesh(
                    &left,
                    &right,
                    hypermesh::exact::ExactRegionSelection::KeepAll,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let validation = mesh.validate_retained_state();
                (mesh, validation)
            })
        });
        c.bench_function("exact_open_surface_arrangement_union_difference", |b| {
            b.iter(|| {
                let preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                )
                .unwrap();
                let result = hypermesh::exact::boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let validation = result.validate_operation_against_sources(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::ALLOW_BOUNDARY,
                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                );
                let difference_preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                )
                .unwrap();
                let difference = hypermesh::exact::boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let difference_validation = difference.validate_operation_against_sources(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::ALLOW_BOUNDARY,
                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                );
                (
                    preflight,
                    result,
                    validation,
                    difference_preflight,
                    difference,
                    difference_validation,
                )
            })
        });
        c.bench_function("exact_selected_region_duplicate_validation", |b| {
            b.iter(|| {
                let mut result = hypermesh::exact::boolean_selected_regions(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
                )
                .unwrap();
                result
                    .region_classifications
                    .push(result.region_classifications[0].clone());
                assert_eq!(
                    result.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::DuplicateRegionClassification
                );
            })
        });
        c.bench_function(
            "exact_selected_region_duplicate_triangulation_validation",
            |b| {
                b.iter(|| {
                    let mut result = hypermesh::exact::boolean_selected_regions(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
                    )
                    .unwrap();
                    result.triangulations.push(result.triangulations[0].clone());
                    assert_eq!(
                        result.validate().unwrap_err(),
                        hypermesh::exact::ExactReportValidationError::DuplicateRegionTriangulation
                    );
                })
            },
        );
        c.bench_function("exact_selected_region_mesh_parity_validation", |b| {
            b.iter(|| {
                let mut result = hypermesh::exact::boolean_selected_regions(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
                )
                .unwrap();
                let mut mesh_vertices = result.mesh.vertices().to_vec();
                mesh_vertices[0] = ExactPoint3::new(Real::from(99), Real::from(0), Real::from(0));
                result.mesh = ExactMesh::new_with_policy(
                    mesh_vertices,
                    result.mesh.triangles().to_vec(),
                    SourceProvenance::exact("bench selected-region mesh vertex payload"),
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                assert_eq!(
                    result.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::OutputMeshAssemblyMismatch
                );

                let mut result = hypermesh::exact::boolean_selected_regions(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
                )
                .unwrap();
                let mesh_triangles = result
                    .mesh
                    .triangles()
                    .iter()
                    .enumerate()
                    .map(|(index, triangle)| {
                        if index == 0 {
                            let [a, b, c] = triangle.0;
                            Triangle([a, c, b])
                        } else {
                            *triangle
                        }
                    })
                    .collect::<Vec<_>>();
                result.mesh = ExactMesh::new_with_policy(
                    result.mesh.vertices().to_vec(),
                    mesh_triangles,
                    SourceProvenance::exact("bench selected-region mesh payload"),
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                assert_eq!(
                    result.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::OutputMeshAssemblyMismatch
                );
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_selected_region_undecided_validation(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 0, 2, 0, //
                20, 0, 0, 22, 0, 0, 20, 2, 0,
            ],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, -1, 2, 0, 1, 0, 2, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mut result = hypermesh::exact::boolean_selected_regions(
            &left,
            &right,
            hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
        )
        .unwrap();
        let classification = result
            .region_classifications
            .first_mut()
            .expect("fixture must produce selected-region side facts");
        classification.relation = FaceRegionPlaneRelation::Unknown;
        classification.node_sides.fill(None);

        c.bench_function("exact_selected_region_undecided_validation", |b| {
            b.iter(|| {
                assert_eq!(
                    result.validate().unwrap_err(),
                    ExactReportValidationError::RegionClassificationNotProofProducing
                );
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_selected_region_preflight(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 0, 2, 0, //
                20, 0, 0, 22, 0, 0, 20, 2, 0,
            ],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, -1, 2, 0, 1, 0, 2, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_selected_region_preflight", |b| {
            b.iter(|| {
                let preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::SelectedRegions(
                        hypermesh::exact::ExactRegionSelection::KeepAll,
                    ),
                )
                .unwrap();
                (preflight.validate(), preflight)
            })
        });
        c.bench_function("exact_preflight_orphan_event_validation", |b| {
            b.iter(|| {
                let mut preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::SelectedRegions(
                        hypermesh::exact::ExactRegionSelection::KeepAll,
                    ),
                )
                .unwrap();
                preflight.retained_face_pairs = 0;
                preflight.retained_events = 1;
                preflight.region_count = 0;
                preflight.region_classifications.clear();
                assert_eq!(
                    preflight.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
                );
            })
        });
        c.bench_function("exact_preflight_region_count_validation", |b| {
            b.iter(|| {
                let mut preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::SelectedRegions(
                        hypermesh::exact::ExactRegionSelection::KeepAll,
                    ),
                )
                .unwrap();
                preflight.region_count += 1;
                assert_eq!(
                    preflight.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::RegionCountMismatch
                );
            })
        });
        c.bench_function("exact_preflight_duplicate_region_validation", |b| {
            b.iter(|| {
                let mut preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::SelectedRegions(
                        hypermesh::exact::ExactRegionSelection::KeepAll,
                    ),
                )
                .unwrap();
                preflight
                    .region_classifications
                    .push(preflight.region_classifications[0].clone());
                assert_eq!(
                    preflight.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::DuplicateRegionClassification
                );
            })
        });
        c.bench_function("exact_blocker_relation_evidence_validation", |b| {
            b.iter(|| {
                let report = hypermesh::exact::ExactRefinementReport {
                    operation: hypermesh::exact::ExactBooleanOperation::Union,
                    status: hypermesh::exact::ExactRefinementStatus::Required,
                    graph_had_unknowns: false,
                    retained_face_pairs: 1,
                    retained_events: 1,
                    blocker: Some(hypermesh::exact::ExactBooleanBlocker {
                        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
                        candidate_pairs: 0,
                        coplanar_overlapping_pairs: 0,
                        coplanar_touching_pairs: 0,
                        unknown_pairs: 0,
                        construction_failed_events: 1,
                    }),
                };
                assert_eq!(
                    report.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
                );
            })
        });
        c.bench_function("exact_preflight_empty_pair_validation", |b| {
            b.iter(|| {
                let mut preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::SelectedRegions(
                        hypermesh::exact::ExactRegionSelection::KeepAll,
                    ),
                )
                .unwrap();
                preflight.retained_face_pairs = 1;
                preflight.retained_events = 0;
                preflight.region_count = 0;
                preflight.region_classifications.clear();
                assert_eq!(
                    preflight.validate().unwrap_err(),
                    hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
                );
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_preflight(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 0, 2, 0, //
                20, 0, 0, 22, 0, 0, 20, 2, 0,
            ],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, -1, 2, 0, 1, 0, 2, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_preflight", |b| {
            b.iter(|| {
                let preflight = hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                )
                .unwrap();
                let winding = certify_winding_readiness_report(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                )
                .unwrap();
                let refinement = certify_refinement_report(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                )
                .unwrap();
                (
                    preflight.validate_against_sources(&left, &right),
                    preflight
                        .blocker
                        .as_ref()
                        .map(|blocker| blocker.validate_against_sources(&left, &right)),
                    preflight.validate(),
                    preflight,
                    refinement.validate_against_sources(&left, &right),
                    refinement
                        .blocker
                        .as_ref()
                        .map(|blocker| blocker.validate_against_sources(&left, &right)),
                    refinement.validate(),
                    refinement,
                    winding.validate_against_sources(&left, &right),
                    winding.freshness_against_sources(&left, &right),
                    winding.blocker.validate_against_sources(&left, &right),
                    winding.validate(),
                    winding,
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_winding_readiness_undecided_validation(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 0, 2, 0, //
                20, 0, 0, 22, 0, 0, 20, 2, 0,
            ],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, -1, 2, 0, 1, 0, 2, 1],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mut report = certify_winding_readiness_report(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
        )
        .unwrap();
        let classification = report
            .region_classifications
            .first_mut()
            .expect("fixture must produce a winding-ready region classification");
        classification.relation = FaceRegionPlaneRelation::Unknown;
        classification.node_sides.fill(None);
        classification.predicates = classification
            .node_sides
            .iter()
            .map(|_| PredicateUse::from_certificate(hyperlimit::PredicateCertificate::Unknown))
            .collect();

        c.bench_function("exact_winding_readiness_undecided_validation", |b| {
            b.iter(|| {
                assert_eq!(
                    report.validate().unwrap_err(),
                    ExactReportValidationError::RegionClassificationNotProofProducing
                );
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_closed_mesh_winding_parity(c: &mut Criterion) {
    let mesh = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            10, 0, 0, //
            0, 10, 0, //
            0, 0, 10,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let inside = p3(1, 1, 1);
    let outside = p3(20, 20, 20);
    let boundary = p3(0, 0, 0);

    c.bench_function("exact_closed_mesh_winding_parity", |b| {
        b.iter(|| {
            let inside_report =
                hypermesh::exact::classify_point_against_closed_mesh_winding_report(&inside, &mesh);
            let outside_report =
                hypermesh::exact::classify_point_against_closed_mesh_winding_report(
                    &outside, &mesh,
                );
            let boundary_report =
                hypermesh::exact::classify_point_against_closed_mesh_winding_report(
                    &boundary, &mesh,
                );
            (
                inside_report.validate_against_sources(&inside, &mesh),
                outside_report.validate_against_sources(&outside, &mesh),
                boundary_report.validate_against_sources(&boundary, &mesh),
                inside_report,
                outside_report,
                boundary_report,
            )
        })
    });
}

fn exact_boolean_winding_shortcuts(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let outer = ExactMesh::from_i64_triangles(
            &[
                0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10, //
                30, 0, 0, 40, 0, 0, 30, 10, 0, 30, 0, 10,
            ],
            &[
                0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3, //
                4, 6, 5, 4, 5, 7, 5, 6, 7, 6, 4, 7,
            ],
        )
        .unwrap();
        let contained = ExactMesh::from_i64_triangles(
            &[1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 2],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let separated = ExactMesh::from_i64_triangles(
            &[15, 1, 1, 16, 1, 1, 15, 2, 1, 15, 1, 2],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolean_winding_containment_shortcut", |b| {
            b.iter(|| {
                let report =
                    hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(
                        &contained, &outer,
                    );
                let preflight = hypermesh::exact::preflight_boolean_exact(
                    &contained,
                    &outer,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                )
                .unwrap();
                let result = hypermesh::exact::boolean_exact(
                    &contained,
                    &outer,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                    ValidationPolicy::CLOSED,
                )
                .unwrap();
                (
                    report.validate_against_sources(&contained, &outer),
                    preflight.validate_against_sources(&contained, &outer),
                    result.validate_against_sources(&contained, &outer),
                    report,
                    preflight,
                    result,
                )
            })
        });
        c.bench_function("exact_boolean_winding_separation_shortcut", |b| {
            b.iter(|| {
                let report =
                    hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(
                        &separated, &outer,
                    );
                let preflight = hypermesh::exact::preflight_boolean_exact(
                    &outer,
                    &separated,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                )
                .unwrap();
                let result = hypermesh::exact::boolean_exact(
                    &outer,
                    &separated,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                    ValidationPolicy::CLOSED,
                )
                .unwrap();
                (
                    report.validate_against_sources(&separated, &outer),
                    preflight.validate_against_sources(&outer, &separated),
                    result.validate_against_sources(&outer, &separated),
                    report,
                    preflight,
                    result,
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_boundary_preflight(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 0, 0, 4, 0, 0, 2, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_boundary_preflight", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact_with_boundary_policy(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
                    )
                    .unwrap(),
                    certify_boundary_touching_report(&left, &right).unwrap(),
                )
            })
        });

        let closed_left = ExactMesh::from_i64_triangles(
            &[
                0, 0, -2, 2, 0, -2, 2, 2, -2, 0, 2, -2, 0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0,
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap();
        let closed_right = top_subdivided_axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);

        c.bench_function("exact_closed_coplanar_contact_boundary_policy", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::preflight_boolean_exact(
                        &closed_left,
                        &closed_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    certify_boundary_touching_report(&closed_left, &closed_right).unwrap(),
                    certify_planar_arrangement_report(
                        &closed_left,
                        &closed_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact_with_boundary_policy(
                        &closed_left,
                        &closed_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
                    )
                    .unwrap(),
                )
            })
        });

        let vertex_left = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let vertex_right = ExactMesh::from_i64_triangles(
            &[0, 0, 0, -2, 0, 0, 0, -2, 0, 0, 0, -2],
            &[0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0],
        )
        .unwrap();

        c.bench_function("exact_closed_vertex_contact_union_shortcut", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::preflight_boolean_exact(
                        &vertex_left,
                        &vertex_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &vertex_left,
                        &vertex_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &vertex_left,
                                &vertex_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    })
                    .unwrap(),
                    certify_boundary_touching_report(&vertex_left, &vertex_right).unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &vertex_left,
                        &vertex_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &vertex_left,
                        &vertex_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &vertex_left,
                                &vertex_right,
                                hypermesh::exact::ExactBooleanOperation::Intersection,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    })
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &vertex_left,
                        &vertex_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &vertex_left,
                        &vertex_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &vertex_left,
                                &vertex_right,
                                hypermesh::exact::ExactBooleanOperation::Difference,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    })
                    .unwrap(),
                )
            })
        });

        let edge_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let edge_right = axis_aligned_box_i64([2, 2, 0], [4, 4, 2]);

        c.bench_function("exact_closed_edge_contact_union_shortcut", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::preflight_boolean_exact(
                        &edge_left,
                        &edge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &edge_left,
                        &edge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &edge_left,
                                &edge_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    })
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_same_surface(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let vertices = [
            0, 0, 0, //
            1, 0, 0, //
            0, 1, 0, //
            0, 0, 1,
        ];
        let mesh = ExactMesh::from_i64_triangles(&vertices, &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3])
            .unwrap();
        let reversed =
            ExactMesh::from_i64_triangles(&vertices, &[0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0])
                .unwrap();
        let shifted = ExactMesh::from_i64_triangles(
            &[
                0, 0, 1, //
                1, 0, 1, //
                0, 1, 1, //
                0, 0, 2,
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolean_same_surface", |b| {
            b.iter(|| {
                let report = certify_same_surface_report(&mesh, &reversed);
                let source_validation = report.validate_against_sources(&mesh, &reversed);
                (
                    report,
                    source_validation,
                    hypermesh::exact::boolean_exact(
                        &mesh,
                        &reversed,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                )
            })
        });
        c.bench_function("exact_same_surface_source_replay_validation", |b| {
            b.iter(|| {
                certify_same_surface_report(&mesh, &reversed)
                    .validate_against_sources(&mesh, &reversed)
            })
        });
        c.bench_function("exact_same_surface_rejection_validation", |b| {
            b.iter(|| {
                let mut report = certify_same_surface_report(&mesh, &shifted);
                report.right_to_left.push(0);
                assert_eq!(
                    report.validate().unwrap_err(),
                    ExactReportValidationError::StatusEvidenceMismatch
                );
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_equivalence(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let vertices = &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0];
        let left = ExactMesh::from_i64_triangles_with_policy(
            vertices,
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            vertices,
            &[0, 1, 3, 1, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_convex_surface_equivalence", |b| {
            b.iter(|| {
                (
                    certify_coplanar_convex_surface_equivalence(&left, &right)
                        .map(|report| report.validate()),
                    certify_coplanar_convex_surface_equivalence(&left, &right)
                        .map(|report| report.validate_against_sources(&left, &right)),
                    certify_coplanar_convex_surface_report(&left, &right).validate(),
                    certify_coplanar_convex_surface_report(&left, &right)
                        .validate_against_sources(&left, &right),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_containment(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let inner = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_convex_surface_containment", |b| {
            b.iter(|| {
                (
                    certify_coplanar_convex_surface_containment(&outer, &inner)
                        .map(|report| report.validate()),
                    certify_coplanar_convex_surface_containment(&outer, &inner)
                        .map(|report| report.validate_against_sources(&outer, &inner)),
                    certify_coplanar_convex_surface_report(&outer, &inner).validate(),
                    certify_coplanar_convex_surface_report(&outer, &inner)
                        .validate_against_sources(&outer, &inner),
                    hypermesh::exact::boolean_exact(
                        &outer,
                        &inner,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_arrangement_union(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function(
            "exact_boolean_coplanar_convex_surface_arrangement_union",
            |b| {
                b.iter(|| {
                    let arrangement = arrange_coplanar_convex_surface_union(&left, &right);
                    (
                        arrangement.as_ref().map(|output| {
                            output.validate_against_sources(
                                &left,
                                &right,
                                CoplanarArrangementOperation::Union,
                            )
                        }),
                        arrangement.as_ref().map(|output| output.validate()),
                        arrangement,
                        hypermesh::exact::preflight_boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                    )
                })
            },
        );
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_multi_union(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
                10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 0, 0, 3, 0, 0, 3, 2, 0, 1, 2, 0, //
                11, 0, 0, 13, 0, 0, 13, 2, 0, 11, 2, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let edge_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let edge_touch_right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let bridge_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
                4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0, //
                10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let bridge_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 0, 0, 5, 0, 0, 5, 2, 0, 1, 2, 0, //
                11, 0, 0, 13, 0, 0, 13, 2, 0, 11, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let single_bridge_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
                4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let single_bridge_right = ExactMesh::from_i64_triangles_with_policy(
            &[1, 0, 0, 5, 0, 0, 5, 2, 0, 1, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let edge_bridge_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
                4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let edge_bridge_right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrect_bridge_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 1, 2, 0, //
                4, 0, 0, 6, 0, 0, 5, 2, 0, 4, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrect_bridge_right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_loop_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                3, 2, 0, 4, -2, 0, 5, 2, 0, //
                2, 5, 0, -2, 4, 0, 2, 3, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_loop_right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_multi_loop_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                3, 2, 0, 4, -2, 0, 5, 2, 0, //
                2, 5, 0, -2, 4, 0, 2, 3, 0, //
                -7, -5, 0, -5, -5, 0, -6, -3, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, //
                6, 7, 8,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_multi_loop_right = nonconvex_loop_right.clone();
        let point_touch_right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let vertex_edge_point_touch_right = ExactMesh::from_i64_triangles_with_policy(
            &[1, 2, 0, 3, 3, 0, 3, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 4, 0, 7, 4, 0, 6, 6, 0, 10, 8, 0, 10, 12, 0, 0, 12, 0,
            ],
            &[
                0, 1, 2, //
                0, 2, 3, //
                0, 3, 4, //
                0, 4, 7, //
                7, 4, 5, //
                7, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_point_touch_right = ExactMesh::from_i64_triangles_with_policy(
            &[10, 12, 0, 12, 12, 0, 12, 14, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_edge_touch_right = ExactMesh::from_i64_triangles_with_policy(
            &[4, 12, 0, 8, 12, 0, 8, 14, 0, 4, 14, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mixed_contact_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
                8, 4, 0, 10, 4, 0, 10, 6, 0, 8, 6, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mixed_contact_right = ExactMesh::from_i64_triangles_with_policy(
            &[4, 0, 0, 8, 0, 0, 8, 4, 0, 4, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mixed_overlap_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 6, 0, 0, 6, 4, 0, 0, 4, 0, //
                8, 4, 0, 10, 4, 0, 10, 6, 0, 8, 6, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let mixed_overlap_right = ExactMesh::from_i64_triangles_with_policy(
            &[4, 0, 0, 8, 0, 0, 8, 4, 0, 4, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let annular_union_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 4, 0, 0, 2, 0, 2, 0, 0, 4, 0, 0, //
                0, -4, 0, 0, -2, 0, -2, 0, 0, -4, 0, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let annular_union_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 0, 0, 2, 0, 0, 0, -2, 0, 0, -4, 0, //
                -4, 0, 0, -2, 0, 0, 0, 2, 0, 0, 4, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let disconnected_annular_union_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 4, 0, 0, 2, 0, 2, 0, 0, 4, 0, 0, //
                0, -4, 0, 0, -2, 0, -2, 0, 0, -4, 0, 0, //
                12, 4, 0, 12, 2, 0, 14, 0, 0, 16, 0, 0, //
                12, -4, 0, 12, -2, 0, 10, 0, 0, 8, 0, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11, 12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let disconnected_annular_union_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 0, 0, 2, 0, 0, 0, -2, 0, 0, -4, 0, //
                -4, 0, 0, -2, 0, 0, 0, 2, 0, 0, 4, 0, //
                16, 0, 0, 14, 0, 0, 12, -2, 0, 12, -4, 0, //
                8, 0, 0, 10, 0, 0, 12, 2, 0, 12, 4, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10, 8, 10, 11, 12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let two_disk_annular_union_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 4, 0, -4, 0, 0, 0, -4, 0, //
                0, -2, 0, -2, 0, 0, 0, 2, 0,
            ],
            &[0, 1, 4, 0, 4, 5, 1, 2, 3, 1, 3, 4],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let two_disk_annular_union_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, -4, 0, 4, 0, 0, 0, 4, 0, //
                0, 2, 0, 2, 0, 0, 0, -2, 0,
            ],
            &[1, 2, 3, 1, 3, 4, 0, 1, 4, 0, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let overlapping_annular_union_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                -1, 5, 0, 0, 2, 0, 2, 0, 0, 5, -1, 0, //
                1, -5, 0, 0, -2, 0, -2, 0, 0, -5, 1, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let overlapping_annular_union_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                5, 1, 0, 2, 0, 0, 0, -2, 0, 1, -5, 0, //
                -5, -1, 0, -2, 0, 0, 0, 2, 0, -1, 5, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_overlapping_annular_union_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                -1, 5, 0, 0, 2, 0, 2, 0, 0, 5, -1, 0, 2, 1, 0, //
                1, -5, 0, 0, -2, 0, -2, 0, 0, -5, 1, 0,
            ],
            &[4, 0, 1, 4, 1, 2, 4, 2, 3, 5, 6, 7, 5, 7, 8],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_overlapping_annular_union_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                5, 1, 0, 2, 0, 0, 0, -2, 0, 1, -5, 0, //
                -5, -1, 0, -2, 0, 0, 0, 2, 0, -1, 5, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let point_branch_annular_union_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                -1, 5, 0, 0, 2, 0, 2, 0, 0, 5, -1, 0, //
                1, -5, 0, 0, -2, 0, -2, 0, 0, -5, 1, 0, //
                5, 1, 0, 7, 1, 0, 6, 3, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let point_branch_annular_union_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                5, 1, 0, 2, 0, 0, 0, -2, 0, 1, -5, 0, //
                -5, -1, 0, -2, 0, 0, 0, 2, 0, -1, 5, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_convex_surface_multi_union", |b| {
            b.iter(|| {
                let arrangement = arrange_coplanar_convex_surface_multi_union(&left, &right);
                let edge_touch_arrangement =
                    arrange_coplanar_convex_surface_union(&edge_touch_left, &edge_touch_right);
                let bridge_arrangement =
                    arrange_coplanar_convex_surface_multi_union(&bridge_left, &bridge_right);
                let single_bridge_arrangement = arrange_coplanar_convex_surface_component_union(
                    &single_bridge_left,
                    &single_bridge_right,
                );
                let edge_bridge_arrangement = arrange_coplanar_convex_surface_component_union(
                    &edge_bridge_left,
                    &edge_bridge_right,
                );
                let nonrect_bridge_arrangement = arrange_coplanar_convex_surface_component_union(
                    &nonrect_bridge_left,
                    &nonrect_bridge_right,
                );
                let nonconvex_loop_arrangement = arrange_coplanar_surface_component_union(
                    &nonconvex_loop_left,
                    &nonconvex_loop_right,
                );
                let nonconvex_multi_loop_arrangement =
                    arrange_coplanar_surface_multi_component_union(
                        &nonconvex_multi_loop_left,
                        &nonconvex_multi_loop_right,
                    );
                let point_touch_arrangement = arrange_coplanar_surface_point_touch_union(
                    &edge_touch_left,
                    &point_touch_right,
                );
                let vertex_edge_point_touch_arrangement =
                    arrange_coplanar_surface_point_touch_union(
                        &edge_touch_left,
                        &vertex_edge_point_touch_right,
                    );
                let nonconvex_point_touch_arrangement = arrange_coplanar_surface_point_touch_union(
                    &nonconvex_point_touch_left,
                    &nonconvex_point_touch_right,
                );
                let nonconvex_edge_touch_arrangement = arrange_coplanar_surface_component_union(
                    &nonconvex_point_touch_left,
                    &nonconvex_edge_touch_right,
                );
                let nonconvex_edge_touch_boundary = certify_coplanar_surface_boundary_touch(
                    &nonconvex_point_touch_left,
                    &nonconvex_edge_touch_right,
                );
                let mixed_contact_arrangement = arrange_coplanar_surface_point_touch_union(
                    &mixed_contact_left,
                    &mixed_contact_right,
                );
                let mixed_overlap_arrangement = arrange_coplanar_surface_point_touch_union(
                    &mixed_overlap_left,
                    &mixed_overlap_right,
                );
                let annular_arrangement = arrange_coplanar_surface_component_holed_union(
                    &annular_union_left,
                    &annular_union_right,
                );
                let disconnected_annular_arrangement =
                    arrange_coplanar_surface_component_holed_union(
                        &disconnected_annular_union_left,
                        &disconnected_annular_union_right,
                    );
                let two_disk_annular_arrangement = arrange_coplanar_surface_component_holed_union(
                    &two_disk_annular_union_left,
                    &two_disk_annular_union_right,
                );
                let overlapping_annular_arrangement =
                    arrange_coplanar_surface_component_holed_union(
                        &overlapping_annular_union_left,
                        &overlapping_annular_union_right,
                    );
                let nonconvex_overlapping_annular_arrangement =
                    arrange_coplanar_surface_component_holed_union(
                        &nonconvex_overlapping_annular_union_left,
                        &nonconvex_overlapping_annular_union_right,
                    );
                let point_branch_annular_arrangement =
                    arrange_coplanar_surface_component_holed_union(
                        &point_branch_annular_union_left,
                        &point_branch_annular_union_right,
                    );
                (
                    arrangement
                        .as_ref()
                        .map(|output| output.validate_union_against_sources(&left, &right)),
                    arrangement.as_ref().map(|output| output.validate()),
                    arrangement,
                    edge_touch_arrangement.as_ref().map(|output| {
                        output.validate_against_sources(
                            &edge_touch_left,
                            &edge_touch_right,
                            CoplanarArrangementOperation::Union,
                        )
                    }),
                    edge_touch_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    edge_touch_arrangement,
                    bridge_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(&bridge_left, &bridge_right)
                    }),
                    bridge_arrangement.as_ref().map(|output| output.validate()),
                    bridge_arrangement,
                    single_bridge_arrangement.as_ref().map(|output| {
                        output.validate_against_sources(
                            &single_bridge_left,
                            &single_bridge_right,
                            CoplanarArrangementOperation::Union,
                        )
                    }),
                    single_bridge_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    single_bridge_arrangement,
                    edge_bridge_arrangement.as_ref().map(|output| {
                        output.validate_against_sources(
                            &edge_bridge_left,
                            &edge_bridge_right,
                            CoplanarArrangementOperation::Union,
                        )
                    }),
                    edge_bridge_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    edge_bridge_arrangement,
                    nonrect_bridge_arrangement.as_ref().map(|output| {
                        output.validate_against_sources(
                            &nonrect_bridge_left,
                            &nonrect_bridge_right,
                            CoplanarArrangementOperation::Union,
                        )
                    }),
                    nonrect_bridge_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    nonrect_bridge_arrangement,
                    nonconvex_loop_arrangement.as_ref().map(|output| {
                        output.validate_component_union_against_sources(
                            &nonconvex_loop_left,
                            &nonconvex_loop_right,
                        )
                    }),
                    nonconvex_loop_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    nonconvex_loop_arrangement,
                    nonconvex_multi_loop_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &nonconvex_multi_loop_left,
                            &nonconvex_multi_loop_right,
                        )
                    }),
                    nonconvex_multi_loop_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    nonconvex_multi_loop_arrangement,
                    point_touch_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(&edge_touch_left, &point_touch_right)
                    }),
                    point_touch_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    point_touch_arrangement,
                    vertex_edge_point_touch_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &edge_touch_left,
                            &vertex_edge_point_touch_right,
                        )
                    }),
                    vertex_edge_point_touch_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    vertex_edge_point_touch_arrangement,
                    nonconvex_point_touch_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &nonconvex_point_touch_left,
                            &nonconvex_point_touch_right,
                        )
                    }),
                    nonconvex_point_touch_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    nonconvex_point_touch_arrangement,
                    nonconvex_edge_touch_arrangement.as_ref().map(|output| {
                        output.validate_component_union_against_sources(
                            &nonconvex_point_touch_left,
                            &nonconvex_edge_touch_right,
                        )
                    }),
                    nonconvex_edge_touch_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    nonconvex_edge_touch_arrangement,
                    nonconvex_edge_touch_boundary,
                    mixed_contact_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &mixed_contact_left,
                            &mixed_contact_right,
                        )
                    }),
                    mixed_contact_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    mixed_contact_arrangement,
                    mixed_overlap_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &mixed_overlap_left,
                            &mixed_overlap_right,
                        )
                    }),
                    mixed_overlap_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    mixed_overlap_arrangement,
                    annular_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &annular_union_left,
                            &annular_union_right,
                        )
                    }),
                    annular_arrangement.as_ref().map(|output| output.validate()),
                    annular_arrangement,
                    disconnected_annular_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &disconnected_annular_union_left,
                            &disconnected_annular_union_right,
                        )
                    }),
                    disconnected_annular_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    disconnected_annular_arrangement,
                    two_disk_annular_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &two_disk_annular_union_left,
                            &two_disk_annular_union_right,
                        )
                    }),
                    two_disk_annular_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    two_disk_annular_arrangement,
                    overlapping_annular_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &overlapping_annular_union_left,
                            &overlapping_annular_union_right,
                        )
                    }),
                    overlapping_annular_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    overlapping_annular_arrangement,
                    nonconvex_overlapping_annular_arrangement
                        .as_ref()
                        .map(|output| {
                            output.validate_union_against_sources(
                                &nonconvex_overlapping_annular_union_left,
                                &nonconvex_overlapping_annular_union_right,
                            )
                        }),
                    nonconvex_overlapping_annular_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    nonconvex_overlapping_annular_arrangement,
                    point_branch_annular_arrangement.as_ref().map(|output| {
                        output.validate_union_against_sources(
                            &point_branch_annular_union_left,
                            &point_branch_annular_union_right,
                        )
                    }),
                    point_branch_annular_arrangement
                        .as_ref()
                        .map(|output| output.validate()),
                    point_branch_annular_arrangement,
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &edge_touch_left,
                        &edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &bridge_left,
                        &bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &single_bridge_left,
                        &single_bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &edge_bridge_left,
                        &edge_bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonrect_bridge_left,
                        &nonrect_bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_loop_left,
                        &nonconvex_loop_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_multi_loop_left,
                        &nonconvex_multi_loop_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &edge_touch_left,
                        &point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &edge_touch_left,
                        &point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &edge_touch_left,
                        &point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &edge_touch_left,
                        &vertex_edge_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &mixed_contact_left,
                        &mixed_contact_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &mixed_overlap_left,
                        &mixed_overlap_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &edge_touch_left,
                        &edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &bridge_left,
                        &bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &single_bridge_left,
                        &single_bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &edge_bridge_left,
                        &edge_bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonrect_bridge_left,
                        &nonrect_bridge_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_loop_left,
                        &nonconvex_loop_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_multi_loop_left,
                        &nonconvex_multi_loop_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &edge_touch_left,
                        &point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &edge_touch_left,
                        &point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &edge_touch_left,
                        &point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &edge_touch_left,
                        &vertex_edge_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_point_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_point_touch_left,
                        &nonconvex_edge_touch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &mixed_contact_left,
                        &mixed_contact_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &mixed_overlap_left,
                        &mixed_overlap_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_intersection(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_convex_surface_intersection", |b| {
            b.iter(|| {
                let arrangement = arrange_coplanar_convex_surface_intersection(&left, &right);
                (
                    arrangement.as_ref().map(|output| {
                        output.validate_against_sources(
                            &left,
                            &right,
                            CoplanarArrangementOperation::Intersection,
                        )
                    }),
                    arrangement.as_ref().map(|output| output.validate()),
                    arrangement,
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_arrangement_difference(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function(
            "exact_boolean_coplanar_convex_surface_arrangement_difference",
            |b| {
                b.iter(|| {
                    let arrangement = arrange_coplanar_convex_surface_difference(&left, &right);
                    (
                        arrangement.as_ref().map(|output| {
                            output.validate_against_sources(
                                &left,
                                &right,
                                CoplanarArrangementOperation::Difference,
                            )
                        }),
                        arrangement.as_ref().map(|output| output.validate()),
                        arrangement,
                        hypermesh::exact::preflight_boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                    )
                })
            },
        );
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_multi_difference(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, -1, 0, 3, -1, 0, 3, 5, 0, 1, 5, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
                4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_right = ExactMesh::from_i64_triangles_with_policy(
            &[1, -1, 0, 3, -1, 0, 3, 3, 0, 1, 3, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
                4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0, //
                8, 0, 0, 10, 0, 0, 10, 2, 0, 8, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, -1, 0, 3, -1, 0, 3, 3, 0, 1, 3, 0, //
                5, -1, 0, 7, -1, 0, 7, 3, 0, 5, 3, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_component_multi_cutter_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 6, 0, 0, 6, 2, 0, 0, 2, 0, //
                10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_component_multi_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, -1, 0, 2, -1, 0, 2, 3, 0, 1, 3, 0, //
                4, -1, 0, 5, -1, 0, 5, 3, 0, 4, 3, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let partial_height_multi_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, //
                4, -1, 0, 5, -1, 0, 5, 3, 0, 4, 3, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrectangular_multi_cutter_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
                20, 0, 0, 22, 0, 0, 22, 2, 0, 20, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrectangular_multi_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[-1, -1, 0, 3, -1, 0, -1, 3, 0, 7, 11, 0, 11, 7, 0, 11, 11, 0],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_multi_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                -1, -1, 0, 3, -1, 0, -1, 3, 0, //
                8, 4, 0, 11, 4, 0, 11, 6, 0, 8, 6, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrectilinear_channel_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrectilinear_channel_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
                -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrectilinear_channel_consumed_only_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 5, 0, 2, 5, 0, 2, 6, 0, 1, 6, 0, //
                8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
                -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let crossing_side_cutter_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let crossing_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                -4, 8, 0, 34, 18, 0, 34, 22, 0, -4, 12, 0, //
                8, -4, 0, 12, -4, 0, 22, 34, 0, 18, 34, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let crossing_side_cutter_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
                26, 2, 0, 28, 2, 0, 28, 4, 0, 26, 4, 0, //
                26, 26, 0, 28, 26, 0, 28, 28, 0, 26, 28, 0, //
                2, 26, 0, 4, 26, 0, 4, 28, 0, 2, 28, 0, //
                -4, 8, 0, 34, 18, 0, 34, 22, 0, -4, 12, 0, //
                8, -4, 0, 12, -4, 0, 22, 34, 0, 18, 34, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19, //
                20, 21, 22, 20, 22, 23,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let crossing_side_cutter_straddling_hole_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                12, 12, 0, 18, 12, 0, 18, 18, 0, 12, 18, 0, //
                -4, 8, 0, 34, 18, 0, 34, 22, 0, -4, 12, 0, //
                8, -4, 0, 12, -4, 0, 22, 34, 0, 18, 34, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_holed_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
                20, 0, 0, 22, 0, 0, 22, 2, 0, 20, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_holed_cut_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                8, -1, 0, 11, -1, 0, 11, 11, 0, 8, 11, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_holed_multi_cut_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                4, -1, 0, 5, -1, 0, 5, 11, 0, 4, 11, 0, //
                8, -1, 0, 11, -1, 0, 11, 11, 0, 8, 11, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_holed_partial_height_multi_cut_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
                    4, 0, 0, 5, 0, 0, 5, 5, 0, 4, 5, 0, //
                    8, -1, 0, 11, -1, 0, 11, 11, 0, 8, 11, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let component_holed_corner_cut_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
                -1, -1, 0, 3, -1, 0, -1, 3, 0, //
                7, 11, 0, 11, 7, 0, 11, 11, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, //
                7, 8, 9,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let single_component_holed_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_nested_small_hole = ExactMesh::from_i64_triangles_with_policy(
            &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_nested_large_hole = ExactMesh::from_i64_triangles_with_policy(
            &[3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_nested_left = arrange_coplanar_convex_surface_holed_difference(
            &single_component_holed_left,
            &same_outer_nested_small_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_nested_right = arrange_coplanar_convex_surface_holed_difference(
            &single_component_holed_left,
            &same_outer_nested_large_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_disjoint_multi_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0, //
                7, 7, 0, 9, 7, 0, 9, 9, 0, 7, 9, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_disjoint_multi_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &single_component_holed_left,
                &same_outer_disjoint_multi_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_disjoint_single_hole = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_disjoint_single_right = arrange_coplanar_convex_surface_holed_difference(
            &single_component_holed_left,
            &same_outer_disjoint_single_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_crossing_single_hole = ExactMesh::from_i64_triangles_with_policy(
            &[5, 3, 0, 8, 3, 0, 8, 6, 0, 5, 6, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_crossing_single_right = arrange_coplanar_convex_surface_holed_difference(
            &single_component_holed_left,
            &same_outer_crossing_single_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_nonrect_component_left_hole = ExactMesh::from_i64_triangles_with_policy(
            &[6, 1, 0, 9, 1, 0, 9, 5, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_nonrect_component_right_hole = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 8, 2, 0, 8, 8, 0, 2, 8, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_nonrect_component_left = arrange_coplanar_convex_surface_holed_difference(
            &single_component_holed_left,
            &same_outer_nonrect_component_left_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_nonrect_component_right = arrange_coplanar_convex_surface_holed_difference(
            &single_component_holed_left,
            &same_outer_nonrect_component_right_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_mixed_component_outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 14, 0, 0, 14, 14, 0, 0, 14, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_mixed_component_left_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 3, 0, 5, 3, 0, 5, 5, 0, 1, 5, 0, //
                10, 8, 0, 13, 8, 0, 10, 11, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_mixed_component_right_hole = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 12, 2, 0, 12, 12, 0, 2, 12, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_mixed_component_left =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_mixed_component_outer,
                &same_outer_mixed_component_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_mixed_component_right = arrange_coplanar_convex_surface_holed_difference(
            &same_outer_mixed_component_outer,
            &same_outer_mixed_component_right_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_simple_component_origin = (0, 0, 0);
        let same_outer_simple_component_basis_u = (2, 1, 0);
        let same_outer_simple_component_basis_v = (-1, 2, 0);
        let same_outer_simple_component_outer = affine_rect_surface_i64(
            &[(0, 0, 14, 14)],
            same_outer_simple_component_origin,
            same_outer_simple_component_basis_u,
            same_outer_simple_component_basis_v,
        );
        let same_outer_simple_component_left_hole = affine_rect_surface_i64(
            &[(7, 4, 13, 12)],
            same_outer_simple_component_origin,
            same_outer_simple_component_basis_u,
            same_outer_simple_component_basis_v,
        );
        let same_outer_simple_component_right_hole = affine_rect_surface_i64(
            &[(3, 2, 12, 5), (8, 5, 12, 10)],
            same_outer_simple_component_origin,
            same_outer_simple_component_basis_u,
            same_outer_simple_component_basis_v,
        );
        let same_outer_simple_component_left = arrange_coplanar_affine_surface_difference(
            &same_outer_simple_component_outer,
            &same_outer_simple_component_left_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_simple_component_right = arrange_coplanar_affine_surface_difference(
            &same_outer_simple_component_outer,
            &same_outer_simple_component_right_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_crossing_multi_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0, //
                5, 3, 0, 8, 3, 0, 8, 6, 0, 5, 6, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_crossing_multi_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &single_component_holed_left,
                &same_outer_crossing_multi_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_split_multi_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
        let same_outer_orthogonal_split_multi_right_hole =
            rect_surface_i64(&[(4, 4, 16, 8), (4, 8, 8, 16)]);
        let same_outer_orthogonal_split_multi_left_hole = rect_surface_i64(&[(6, 4, 8, 16)]);
        let same_outer_orthogonal_split_multi_left =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_split_multi_outer,
                &same_outer_orthogonal_split_multi_left_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_split_multi_right =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_split_multi_outer,
                &same_outer_orthogonal_split_multi_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_common_left_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
                6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_common_right_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
                6, 1, 0, 8, 1, 0, 8, 3, 0, 6, 3, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_common_left = arrange_coplanar_convex_surface_multi_holed_difference(
            &single_component_holed_left,
            &same_outer_common_left_holes,
        )
        .unwrap()
        .mesh;
        let same_outer_common_right = arrange_coplanar_convex_surface_multi_holed_difference(
            &single_component_holed_left,
            &same_outer_common_right_holes,
        )
        .unwrap()
        .mesh;
        let same_outer_partial_union_right_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                3, 2, 0, 5, 2, 0, 5, 4, 0, 3, 4, 0, //
                6, 1, 0, 8, 1, 0, 8, 3, 0, 6, 3, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_partial_union_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &single_component_holed_left,
                &same_outer_partial_union_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_nonrect_partial_union_right_holes =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    3, 2, 0, 5, 2, 0, 5, 4, 0, //
                    6, 1, 0, 8, 1, 0, 8, 3, 0, 6, 3, 0,
                ],
                &[0, 1, 2, 3, 4, 5, 3, 5, 6],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let same_outer_nonrect_partial_union_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &single_component_holed_left,
                &same_outer_nonrect_partial_union_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_retained_union_outer = rect_surface_i64(&[(0, 0, 10, 10)]);
        let same_outer_orthogonal_retained_union_left_hole =
            rect_surface_i64(&[(2, 2, 6, 6), (6, 2, 8, 4)]);
        let same_outer_orthogonal_retained_union_right_hole = rect_surface_i64(&[(4, 3, 9, 7)]);
        let same_outer_orthogonal_retained_union_left =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_retained_union_outer,
                &same_outer_orthogonal_retained_union_left_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_retained_union_right =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_retained_union_outer,
                &same_outer_orthogonal_retained_union_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_bridge_intersection_left_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
                6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_bridge_intersection_right_hole = ExactMesh::from_i64_triangles_with_policy(
            &[3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_bridge_intersection_nonrect_right_hole =
            ExactMesh::from_i64_triangles_with_policy(
                &[3, 3, 0, 7, 3, 0, 8, 8, 0, 3, 7, 0],
                &[0, 1, 2, 0, 2, 3],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let same_outer_bridge_intersection_left =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &single_component_holed_left,
                &same_outer_bridge_intersection_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_bridge_intersection_right =
            arrange_coplanar_convex_surface_holed_difference(
                &single_component_holed_left,
                &same_outer_bridge_intersection_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_bridge_intersection_nonrect_right =
            arrange_coplanar_convex_surface_holed_difference(
                &single_component_holed_left,
                &same_outer_bridge_intersection_nonrect_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_intersection_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
        let same_outer_orthogonal_intersection_left_hole =
            rect_surface_i64(&[(4, 4, 12, 8), (4, 8, 8, 16)]);
        let same_outer_orthogonal_intersection_right_hole = rect_surface_i64(&[(8, 6, 16, 10)]);
        let same_outer_orthogonal_intersection_left =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_intersection_outer,
                &same_outer_orthogonal_intersection_left_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_intersection_right =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_intersection_outer,
                &same_outer_orthogonal_intersection_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_island_intersection_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
        let same_outer_orthogonal_island_intersection_left_holes =
            rect_surface_i64(&[(4, 4, 13, 8), (4, 8, 8, 17)]);
        let same_outer_orthogonal_island_intersection_right_holes =
            rect_surface_i64(&[(12, 4, 16, 13), (7, 12, 16, 16)]);
        let same_outer_orthogonal_island_intersection_left =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_island_intersection_outer,
                &same_outer_orthogonal_island_intersection_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_island_intersection_right =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_island_intersection_outer,
                &same_outer_orthogonal_island_intersection_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_point_branch_island_intersection_outer = rect_surface_i64(&[(0, 0, 5, 5)]);
        let same_outer_point_branch_island_intersection_holes = rect_surface_i64(&[
            (1, 2, 2, 3),
            (1, 3, 2, 4),
            (2, 1, 3, 2),
            (2, 3, 3, 4),
            (3, 1, 4, 2),
            (3, 2, 4, 3),
            (3, 3, 4, 4),
        ]);
        let same_outer_point_branch_island_intersection_source =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_point_branch_island_intersection_outer,
                &same_outer_point_branch_island_intersection_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_point_branch_island_consumed_hole = rect_surface_i64(&[(2, 2, 3, 3)]);
        let same_outer_point_branch_island_consumed_right =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_point_branch_island_intersection_outer,
                &same_outer_point_branch_island_consumed_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_point_branch_island_clipped_outer = rect_surface_i64(&[(0, 0, 10, 10)]);
        let same_outer_point_branch_island_clipped_holes = rect_surface_i64(&[
            (2, 4, 4, 6),
            (2, 6, 4, 8),
            (4, 2, 6, 4),
            (4, 6, 6, 8),
            (6, 2, 8, 4),
            (6, 4, 8, 6),
            (6, 6, 8, 8),
        ]);
        let same_outer_point_branch_island_clipped_source =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_point_branch_island_clipped_outer,
                &same_outer_point_branch_island_clipped_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_point_branch_island_clipped_right_hole = rect_surface_i64(&[(5, 4, 7, 6)]);
        let same_outer_point_branch_island_clipped_right =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_point_branch_island_clipped_outer,
                &same_outer_point_branch_island_clipped_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_multi_source_island_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
        let same_outer_multi_source_island_owner_hole = rect_surface_i64(&[(4, 4, 16, 16)]);
        let same_outer_multi_source_island_shell =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_multi_source_island_outer,
                &same_outer_multi_source_island_owner_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_multi_source_island = rect_surface_i64(&[(8, 8, 12, 12)]);
        let same_outer_multi_source_island_source = combine_open_exact_meshes(
            &[
                same_outer_multi_source_island_shell,
                same_outer_multi_source_island,
            ],
            "bench same-outer multi-hole clipped source island",
        );
        let same_outer_multi_source_island_right_holes =
            rect_surface_i64(&[(6, 6, 10, 10), (11, 11, 14, 14)]);
        let same_outer_multi_source_island_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_multi_source_island_outer,
                &same_outer_multi_source_island_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_split_source_island_right_hole = rect_surface_i64(&[(9, 8, 11, 12)]);
        let same_outer_split_source_island_right =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_multi_source_island_outer,
                &same_outer_split_source_island_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_holed_source_island_outer = rect_surface_i64(&[(0, 0, 24, 24)]);
        let same_outer_holed_source_island_owner_hole = rect_surface_i64(&[(4, 4, 20, 20)]);
        let same_outer_holed_source_island_shell =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_holed_source_island_outer,
                &same_outer_holed_source_island_owner_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_holed_source_island_component_outer = rect_surface_i64(&[(8, 8, 18, 18)]);
        let same_outer_holed_source_island_component_hole = rect_surface_i64(&[(10, 10, 14, 14)]);
        let same_outer_holed_source_island_component =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_holed_source_island_component_outer,
                &same_outer_holed_source_island_component_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_holed_source_island_source = combine_open_exact_meshes(
            &[
                same_outer_holed_source_island_shell,
                same_outer_holed_source_island_component,
            ],
            "bench same-outer holed source island",
        );
        let same_outer_holed_source_island_right_holes =
            rect_surface_i64(&[(11, 11, 12, 12), (15, 15, 17, 17)]);
        let same_outer_holed_source_island_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_holed_source_island_outer,
                &same_outer_holed_source_island_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_clipped_holed_source_island_outer = rect_surface_i64(&[(0, 0, 24, 24)]);
        let same_outer_clipped_holed_source_island_owner_hole = rect_surface_i64(&[(4, 4, 20, 20)]);
        let same_outer_clipped_holed_source_island_shell =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_clipped_holed_source_island_outer,
                &same_outer_clipped_holed_source_island_owner_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_clipped_holed_source_island_component_outer =
            rect_surface_i64(&[(8, 8, 18, 18)]);
        let same_outer_clipped_holed_source_island_component_hole =
            rect_surface_i64(&[(10, 10, 12, 12)]);
        let same_outer_clipped_holed_source_island_component =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_clipped_holed_source_island_component_outer,
                &same_outer_clipped_holed_source_island_component_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_clipped_holed_source_island_source = combine_open_exact_meshes(
            &[
                same_outer_clipped_holed_source_island_shell,
                same_outer_clipped_holed_source_island_component,
            ],
            "bench same-outer clipped holed source island",
        );
        let same_outer_clipped_holed_source_island_right_holes =
            rect_surface_i64(&[(16, 8, 22, 18), (13, 13, 15, 15)]);
        let same_outer_clipped_holed_source_island_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_clipped_holed_source_island_outer,
                &same_outer_clipped_holed_source_island_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_split_holed_source_island_outer = rect_surface_i64(&[(0, 0, 24, 24)]);
        let same_outer_split_holed_source_island_owner_hole = rect_surface_i64(&[(4, 4, 20, 20)]);
        let same_outer_split_holed_source_island_shell =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_split_holed_source_island_outer,
                &same_outer_split_holed_source_island_owner_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_split_holed_source_island_component_outer =
            rect_surface_i64(&[(8, 8, 18, 18)]);
        let same_outer_split_holed_source_island_component_hole =
            rect_surface_i64(&[(9, 10, 11, 12)]);
        let same_outer_split_holed_source_island_component =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_split_holed_source_island_component_outer,
                &same_outer_split_holed_source_island_component_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_split_holed_source_island_source = combine_open_exact_meshes(
            &[
                same_outer_split_holed_source_island_shell,
                same_outer_split_holed_source_island_component,
            ],
            "bench same-outer split holed source island",
        );
        let same_outer_split_holed_source_island_right_holes =
            rect_surface_i64(&[(12, 8, 14, 18), (15, 13, 17, 15)]);
        let same_outer_split_holed_source_island_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_split_holed_source_island_outer,
                &same_outer_split_holed_source_island_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_affine_island_intersection_origin = (0, 0, 0);
        let same_outer_affine_island_intersection_basis_u = (2, 1, 0);
        let same_outer_affine_island_intersection_basis_v = (-1, 2, 0);
        let same_outer_affine_island_intersection_outer = affine_rect_surface_i64(
            &[(0, 0, 20, 20)],
            same_outer_affine_island_intersection_origin,
            same_outer_affine_island_intersection_basis_u,
            same_outer_affine_island_intersection_basis_v,
        );
        let same_outer_affine_island_intersection_left_holes = affine_rect_surface_i64(
            &[(4, 4, 13, 8), (4, 8, 8, 17)],
            same_outer_affine_island_intersection_origin,
            same_outer_affine_island_intersection_basis_u,
            same_outer_affine_island_intersection_basis_v,
        );
        let same_outer_affine_island_intersection_right_holes = affine_rect_surface_i64(
            &[(12, 4, 16, 13), (7, 12, 16, 16)],
            same_outer_affine_island_intersection_origin,
            same_outer_affine_island_intersection_basis_u,
            same_outer_affine_island_intersection_basis_v,
        );
        let same_outer_affine_island_intersection_left =
            arrange_coplanar_affine_surface_difference(
                &same_outer_affine_island_intersection_outer,
                &same_outer_affine_island_intersection_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_affine_island_intersection_right =
            arrange_coplanar_affine_surface_difference(
                &same_outer_affine_island_intersection_outer,
                &same_outer_affine_island_intersection_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_simple_intersection_origin = (0, 0, 0);
        let same_outer_simple_intersection_basis_u = (2, 1, 0);
        let same_outer_simple_intersection_basis_v = (-1, 2, 0);
        let same_outer_simple_intersection_outer = affine_rect_surface_i64(
            &[(0, 0, 20, 20)],
            same_outer_simple_intersection_origin,
            same_outer_simple_intersection_basis_u,
            same_outer_simple_intersection_basis_v,
        );
        let same_outer_simple_intersection_left_hole = affine_rect_surface_i64(
            &[(4, 4, 12, 8), (4, 8, 8, 16)],
            same_outer_simple_intersection_origin,
            same_outer_simple_intersection_basis_u,
            same_outer_simple_intersection_basis_v,
        );
        let same_outer_simple_intersection_right_hole = affine_rect_surface_i64(
            &[(6, 6, 16, 10)],
            same_outer_simple_intersection_origin,
            same_outer_simple_intersection_basis_u,
            same_outer_simple_intersection_basis_v,
        );
        let same_outer_simple_intersection_left = arrange_coplanar_affine_surface_difference(
            &same_outer_simple_intersection_outer,
            &same_outer_simple_intersection_left_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_simple_intersection_right = arrange_coplanar_affine_surface_difference(
            &same_outer_simple_intersection_outer,
            &same_outer_simple_intersection_right_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_disconnected_intersection_outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_disconnected_intersection_left_holes =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    2, 2, 0, 5, 2, 0, 5, 5, 0, 2, 5, 0, //
                    12, 12, 0, 15, 12, 0, 15, 15, 0, 12, 15, 0,
                ],
                &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let same_outer_disconnected_intersection_right_holes =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    4, 2, 0, 7, 2, 0, 7, 5, 0, 4, 5, 0, //
                    14, 12, 0, 17, 12, 0, 17, 15, 0, 14, 15, 0,
                ],
                &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let same_outer_disconnected_intersection_left =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_disconnected_intersection_outer,
                &same_outer_disconnected_intersection_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_disconnected_intersection_right =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_disconnected_intersection_outer,
                &same_outer_disconnected_intersection_right_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_partial_component_outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 12, 0, 0, 12, 12, 0, 0, 12, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_partial_component_left_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
                8, 1, 0, 11, 1, 0, 11, 5, 0, 8, 5, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_nonrect_partial_component_left_holes =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
                    8, 1, 0, 11, 1, 0, 11, 5, 0,
                ],
                &[0, 1, 2, 0, 2, 3, 4, 5, 6],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let same_outer_partial_component_right_hole = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 10, 2, 0, 10, 10, 0, 2, 10, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_partial_component_left =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_partial_component_outer,
                &same_outer_partial_component_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_nonrect_partial_component_left =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_partial_component_outer,
                &same_outer_nonrect_partial_component_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_partial_component_right = arrange_coplanar_convex_surface_holed_difference(
            &same_outer_partial_component_outer,
            &same_outer_partial_component_right_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_mixed_partial_component_outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 14, 0, 0, 14, 14, 0, 0, 14, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_mixed_partial_component_left_holes =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    6, 6, 0, 7, 6, 0, 7, 7, 0, 6, 7, 0, //
                    1, 3, 0, 5, 3, 0, 5, 5, 0, 1, 5, 0, //
                    10, 8, 0, 13, 8, 0, 10, 11, 0,
                ],
                &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let same_outer_mixed_partial_component_right_hole =
            ExactMesh::from_i64_triangles_with_policy(
                &[2, 2, 0, 12, 2, 0, 12, 12, 0, 2, 12, 0],
                &[0, 1, 2, 0, 2, 3],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let same_outer_mixed_partial_component_left =
            arrange_coplanar_convex_surface_multi_holed_difference(
                &same_outer_mixed_partial_component_outer,
                &same_outer_mixed_partial_component_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_mixed_partial_component_right =
            arrange_coplanar_convex_surface_holed_difference(
                &same_outer_mixed_partial_component_outer,
                &same_outer_mixed_partial_component_right_hole,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_partial_component_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
        let same_outer_orthogonal_partial_component_right_hole =
            rect_surface_i64(&[(4, 4, 16, 8), (4, 8, 8, 16)]);
        let same_outer_orthogonal_partial_component_left_holes =
            rect_surface_i64(&[(12, 5, 14, 7), (6, 10, 10, 14)]);
        let same_outer_orthogonal_partial_component_left =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_partial_component_outer,
                &same_outer_orthogonal_partial_component_left_holes,
            )
            .unwrap()
            .mesh;
        let same_outer_orthogonal_partial_component_right =
            arrange_coplanar_orthogonal_surface_difference(
                &same_outer_orthogonal_partial_component_outer,
                &same_outer_orthogonal_partial_component_right_hole,
            )
            .unwrap()
            .mesh;
        let nonrectilinear_channel_holed_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrectilinear_channel_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 17, 0, 4, 17, 0, 4, 19, 0, 2, 19, 0, //
                15, 4, 0, 17, 4, 0, 17, 6, 0, 15, 6, 0, //
                8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
                -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0, -2, 12, 0, 4, 11, 0, 5, 15, 0, -2, 16, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrectilinear_channel_consumed_hole_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 17, 0, 4, 17, 0, 4, 19, 0, 2, 19, 0, //
                15, 4, 0, 17, 4, 0, 17, 6, 0, 15, 6, 0, //
                1, 5, 0, 2, 5, 0, 2, 6, 0, 1, 6, 0, //
                8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
                -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0, -2, 12, 0, 4, 11, 0, 5, 15, 0, -2, 16, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_holed_contact_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
                -1, 5, 0, 4, 5, 0, 4, 6, 0, -1, 6, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_consumed_hole_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0, //
                30, 0, 0, 40, 0, 0, 40, 10, 0, 30, 10, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_consumed_hole_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
                33, 3, 0, 35, 3, 0, 35, 5, 0, 33, 5, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_no_hole_consumed_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0, //
                30, 0, 0, 50, 0, 0, 50, 20, 0, 30, 20, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_no_hole_consumed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
                38, 8, 0, 42, 10, 0, 38, 12, 0, //
                30, 9, 0, 40, 8, 0, 40, 12, 0, 30, 11, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, //
                10, 11, 12, 10, 12, 13,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrect_contact_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrect_contact_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 9, 0, 8, 10, 0, 6, 8, 0, //
                0, 8, 0, 8, 10, 0, 0, 12, 0,
            ],
            &[0, 2, 1, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let straddling_contact_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let independent_openings_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
                12, 4, 0, 16, 6, 0, 12, 8, 0, //
                20, 5, 0, 14, 4, 0, 14, 8, 0, 20, 7, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, //
                10, 11, 12, 10, 12, 13,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let contact_opening_holed_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let contact_opening_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
                15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let independent_openings_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
                12, 4, 0, 16, 6, 0, 12, 8, 0, //
                20, 5, 0, 14, 4, 0, 14, 8, 0, 20, 7, 0, //
                3, 15, 0, 5, 15, 0, 5, 17, 0, 3, 17, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, //
                10, 11, 12, 10, 12, 13, //
                14, 15, 16, 14, 16, 17,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let connected_multi_cutter_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0, //
                -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
                -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multiple_side_cutter_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
                -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
                -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
                11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let consumed_hole_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
                4, 8, 0, 5, 8, 0, 5, 9, 0, 4, 9, 0, //
                -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
                -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
                11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let single_consumed_hole_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
                4, 8, 0, 5, 8, 0, 5, 9, 0, 4, 9, 0, //
                -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let single_consumed_only_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 8, 0, 5, 8, 0, 5, 9, 0, 4, 9, 0, //
                -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let straddling_hole_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
                8, 12, 0, 10, 12, 0, 10, 14, 0, 8, 14, 0, //
                -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
                -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
                11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let split_straddling_hole_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
                9, 10, 0, 11, 10, 0, 11, 14, 0, 9, 14, 0, //
                -2, 8, 0, 10, 8, 0, 10, 12, 0, -2, 12, 0, //
                10, 8, 0, 22, 8, 0, 22, 12, 0, 10, 13, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let split_all_consumed_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                9, 10, 0, 11, 10, 0, 11, 14, 0, 9, 14, 0, //
                -2, 8, 0, 10, 8, 0, 10, 12, 0, -2, 12, 0, //
                10, 8, 0, 22, 8, 0, 22, 12, 0, 10, 13, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let single_split_consumed_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
                9, 10, 0, 11, 10, 0, 11, 11, 0, 9, 11, 0, //
                -2, 8, 0, 22, 8, 0, 22, 15, 0, -2, 12, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_branch_consumed_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_branch_consumed_with_retained_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 2, 0, 6, 2, 0, 6, 4, 0, 4, 4, 0, //
                4, 14, 0, 6, 14, 0, 6, 16, 0, 4, 16, 0, //
                4, 26, 0, 6, 26, 0, 6, 28, 0, 4, 28, 0, //
                13, 9, 0, 17, 9, 0, 17, 13, 0, 13, 13, 0, //
                -2, 7, 0, 15, 7, 0, 15, 11, 0, -2, 11, 0, //
                15, 7, 0, 32, 7, 0, 32, 11, 0, 15, 11, 0, //
                13, 21, 0, 17, 21, 0, 17, 25, 0, 13, 25, 0, //
                -2, 19, 0, 15, 19, 0, 15, 23, 0, -2, 23, 0, //
                15, 19, 0, 32, 19, 0, 32, 23, 0, 15, 23, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19, //
                20, 21, 22, 20, 22, 23, //
                24, 25, 26, 24, 26, 27, //
                28, 29, 30, 28, 30, 31, //
                32, 33, 34, 32, 34, 35,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_branch_all_consumed_side_cutter_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    13, 9, 0, 17, 9, 0, 17, 13, 0, 13, 13, 0, //
                    -2, 7, 0, 15, 7, 0, 15, 11, 0, -2, 11, 0, //
                    15, 7, 0, 32, 7, 0, 32, 11, 0, 15, 11, 0, //
                    13, 21, 0, 17, 21, 0, 17, 25, 0, 13, 25, 0, //
                    -2, 19, 0, 15, 19, 0, 15, 23, 0, -2, 23, 0, //
                    15, 19, 0, 32, 19, 0, 32, 23, 0, 15, 23, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15, //
                    16, 17, 18, 16, 18, 19, //
                    20, 21, 22, 20, 22, 23,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let branch_group_consumed_with_retained_holes = ExactMesh::from_i64_triangles_with_policy(
            &[
                3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
                25, 3, 0, 27, 3, 0, 27, 5, 0, 25, 5, 0, //
                3, 25, 0, 5, 25, 0, 5, 27, 0, 3, 27, 0, //
                25, 25, 0, 27, 25, 0, 27, 27, 0, 25, 27, 0, //
                12, 12, 0, 18, 12, 0, 18, 18, 0, 12, 18, 0, //
                -2, 14, 0, 14, 14, 0, 14, 16, 0, -2, 16, 0, //
                16, 14, 0, 32, 14, 0, 32, 16, 0, 16, 16, 0, //
                14, -2, 0, 16, -2, 0, 16, 14, 0, 14, 14, 0, //
                14, 16, 0, 16, 16, 0, 16, 32, 0, 14, 32, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19, //
                20, 21, 22, 20, 22, 23, //
                24, 25, 26, 24, 26, 27, //
                28, 29, 30, 28, 30, 31, //
                32, 33, 34, 32, 34, 35,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let branch_group_all_consumed_side_cutter_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    12, 12, 0, 18, 12, 0, 18, 18, 0, 12, 18, 0, //
                    -2, 14, 0, 14, 14, 0, 14, 16, 0, -2, 16, 0, //
                    16, 14, 0, 32, 14, 0, 32, 16, 0, 16, 16, 0, //
                    14, -2, 0, 16, -2, 0, 16, 14, 0, 14, 14, 0, //
                    14, 16, 0, 16, 16, 0, 16, 32, 0, 14, 32, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15, //
                    16, 17, 18, 16, 18, 19,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let mixed_consumed_hole_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                6, 8, 0, 8, 9, 0, 8, 11, 0, 6, 12, 0, //
                0, 9, 0, 6, 8, 0, 6, 12, 0, 0, 11, 0, //
                11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10, //
                11, 12, 13, 11, 13, 14,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let side_cutter_no_hole_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
                -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
                11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let single_side_cutter_no_hole_right = ExactMesh::from_i64_triangles_with_policy(
            &[-2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let point_touch_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let vertex_edge_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let vertex_edge_point_touch_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                10, -2, 0, 14, -2, 0, 12, 8, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_vertex_edge_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 20, 0, 0, 20, 20, 0, 12, 20, 0, 12, 12, 0, 8, 12, 0, 8, 20, 0, 0, 20, 0,
                20, 12, 0, 0, 12, 0,
            ],
            &[
                0, 1, 8, 0, 8, 4, 0, 4, 5, 0, 5, 9, //
                9, 5, 6, 9, 6, 7, //
                4, 8, 2, 4, 2, 3,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_vertex_edge_point_touch_side_cutter_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    -2, 8, 0, 12, 8, 0, 12, 10, 0, -2, 10, 0, //
                    10, -2, 0, 14, -2, 0, 12, 8, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let point_touch_consumed_hole_side_cutter_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    2, 5, 0, 3, 5, 0, 3, 6, 0, 2, 6, 0, //
                    -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                    10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let point_touch_straddling_hole_side_cutter_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
                    -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                    10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let grouped_point_touch_straddling_hole_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let grouped_point_touch_straddling_hole_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
                12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let orthogonal_grouped_point_touch_straddling_hole_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                    -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                    12, 12, 0, 16, 12, 0, 16, 32, 0, 12, 32, 0, //
                    12, -2, 0, 16, -2, 0, 16, 8, 0, 12, 8, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let grouped_point_touch_straddling_retained_hole_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
                    11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                    -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                    12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
                    12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15, //
                    16, 17, 18, 16, 18, 19,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let orthogonal_grouped_point_touch_straddling_retained_hole_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
                    11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                    -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                    12, 12, 0, 16, 12, 0, 16, 32, 0, 12, 32, 0, //
                    12, -2, 0, 16, -2, 0, 16, 8, 0, 12, 8, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15, //
                    16, 17, 18, 16, 18, 19,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let multi_component_grouped_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0, //
                40, 0, 0, 50, 0, 0, 50, 10, 0, 40, 10, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_grouped_point_touch_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
                12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0, //
                43, 3, 0, 45, 3, 0, 45, 5, 0, 43, 5, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0, //
                30, 0, 0, 40, 0, 0, 40, 10, 0, 30, 10, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let point_touch_component_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 1, 0, 4, 1, 0, 4, 3, 0, 2, 3, 0, //
                -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let point_touch_straddling_component_holed_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    2, 1, 0, 4, 1, 0, 4, 3, 0, 2, 3, 0, //
                    7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
                    -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                    10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let multi_component_point_touch_straddling_holed_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
                    -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                    10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0, //
                    33, 3, 0, 35, 3, 0, 35, 5, 0, 33, 5, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_point_touch_side_cutter_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 20, 0, 0, 20, 20, 0, 12, 20, 0, 12, 12, 0, 8, 12, 0, 8, 20, 0, 0, 20, 0,
                20, 12, 0, 0, 12, 0,
            ],
            &[
                0, 1, 8, 0, 8, 4, 0, 4, 5, 0, 5, 9, //
                9, 5, 6, 9, 6, 7, //
                4, 8, 2, 4, 2, 3,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let multi_component_nonconvex_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 20, 0, 0, 20, 20, 0, 12, 20, 0, 12, 12, 0, 8, 12, 0, 8, 20, 0, 0, 20, 0,
                20, 12, 0, 0, 12, 0, //
                30, 0, 0, 40, 0, 0, 40, 10, 0, 30, 10, 0,
            ],
            &[
                0, 1, 8, 0, 8, 4, 0, 4, 5, 0, 5, 9, //
                9, 5, 6, 9, 6, 7, //
                4, 8, 2, 4, 2, 3, //
                10, 11, 12, 10, 12, 13,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_point_touch_component_holed_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    3, 1, 0, 5, 1, 0, 5, 3, 0, 3, 3, 0, //
                    -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                    10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_point_touch_straddling_component_holed_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    3, 1, 0, 5, 1, 0, 5, 3, 0, 3, 3, 0, //
                    7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
                    -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                    10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_grouped_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 30, 0, 0, 30, 26, 0, 30, 30, 0, 22, 30, 0, 22, 26, 0, 20, 26, 0, 20, 30,
                0, 0, 30, 0, 0, 26, 0,
            ],
            &[
                0, 1, 2, //
                0, 2, 5, //
                0, 5, 6, //
                0, 6, 9, //
                9, 6, 7, //
                9, 7, 8, //
                5, 2, 3, //
                5, 3, 4,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_grouped_point_touch_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
                12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_grouped_point_touch_retained_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
                    11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                    -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                    12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
                    12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15, //
                    16, 17, 18, 16, 18, 19,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let multi_component_nonconvex_grouped_point_touch_left =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    0, 0, 0, 30, 0, 0, 30, 26, 0, 30, 30, 0, 22, 30, 0, 22, 26, 0, 20, 26, 0, 20,
                    30, 0, 0, 30, 0, 0, 26, 0, //
                    40, 0, 0, 50, 0, 0, 50, 10, 0, 40, 10, 0,
                ],
                &[
                    0, 1, 2, //
                    0, 2, 5, //
                    0, 5, 6, //
                    0, 6, 9, //
                    9, 6, 7, //
                    9, 7, 8, //
                    5, 2, 3, //
                    5, 3, 4, //
                    10, 11, 12, 10, 12, 13,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let multi_component_nonconvex_grouped_point_touch_right =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                    -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                    12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
                    12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0, //
                    43, 3, 0, 45, 3, 0, 45, 5, 0, 43, 5, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11, //
                    12, 13, 14, 12, 14, 15, //
                    16, 17, 18, 16, 18, 19,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let incidental_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 8, 0, 8, 8, 0, 8, 12, 0, 0, 12, 0, //
                0, 11, 0, 10, 12, 0, 0, 15, 0, //
                8, 12, 0, 0, 14, 0, 0, 18, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, //
                7, 8, 9,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_opening_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
                10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_opening_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0, //
                10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 10, 0, 0, 10, 4, 0, 7, 4, 0, 6, 6, 0, 10, 8, 0, 10, 12, 0, 0, 12, 0,
            ],
            &[
                0, 1, 2, //
                0, 2, 3, //
                0, 3, 4, //
                0, 4, 7, //
                7, 4, 5, //
                7, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_opening = ExactMesh::from_i64_triangles_with_policy(
            &[2, 12, 0, 5, 9, 0, 7, 10, 0, 4, 12, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_hole = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 3, 2, 0, 2, 3, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_opening_and_hole = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 3, 2, 0, 2, 3, 0, //
                2, 12, 0, 5, 9, 0, 7, 10, 0, 4, 12, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_crossing_opening = ExactMesh::from_i64_triangles_with_policy(
            &[4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_crossing_opening_consumed_hole =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    5, 9, 0, 7, 9, 0, 7, 11, 0, 5, 11, 0, //
                    4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_split_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 30, 0, 0, 30, 10, 0, 10, 10, 0, 10, 30, 0, 0, 30, 0],
            &[
                0, 1, 2, //
                0, 2, 3, //
                0, 3, 5, //
                3, 4, 5,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_split_crossing_consumed_hole = ExactMesh::from_i64_triangles_with_policy(
            &[
                7, 12, 0, 9, 12, 0, 9, 14, 0, 7, 14, 0, //
                8, -2, 0, 12, -2, 0, 12, 32, 0, 8, 32, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_split_crossing_consumed_and_retained_holes =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    20, 2, 0, 22, 2, 0, 22, 4, 0, 20, 4, 0, //
                    7, 12, 0, 9, 12, 0, 9, 14, 0, 7, 14, 0, //
                    8, -2, 0, 12, -2, 0, 12, 32, 0, 8, 32, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7, //
                    8, 9, 10, 8, 10, 11,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_source_overlapping_crossing_openings =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0, //
                    2, 8, 0, 8, 8, 0, 8, 14, 0, 2, 14, 0,
                ],
                &[
                    0, 1, 2, 0, 2, 3, //
                    4, 5, 6, 4, 6, 7,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_source_incidental_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 20, 0, 0, 20, 8, 0, 16, 8, 0, 16, 12, 0, 20, 12, 0, 20, 20, 0, 0, 20, 0,
            ],
            &[
                0, 1, 2, //
                0, 2, 3, //
                0, 3, 4, //
                0, 4, 7, //
                7, 4, 5, //
                7, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_incidental_openings = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 8, 0, 8, 8, 0, 8, 12, 0, 0, 12, 0, //
                0, 11, 0, 10, 12, 0, 0, 15, 0, //
                8, 12, 0, 0, 14, 0, 0, 18, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, //
                7, 8, 9,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_crossing_opening_and_hole = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 3, 2, 0, 2, 3, 0, //
                4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_source_overlapping_crossing_openings_and_hole =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    2, 2, 0, 3, 2, 0, 2, 3, 0, //
                    4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0, //
                    2, 8, 0, 8, 8, 0, 8, 14, 0, 2, 14, 0,
                ],
                &[
                    0, 1, 2, //
                    3, 4, 5, 3, 5, 6, //
                    7, 8, 9, 7, 9, 10,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_source_clipped_straddling_hole_and_retained_hole =
            ExactMesh::from_i64_triangles_with_policy(
                &[
                    2, 2, 0, 3, 2, 0, 2, 3, 0, //
                    5, 9, 0, 7, 9, 0, 7, 11, 0, 5, 11, 0, //
                    4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0,
                ],
                &[
                    0, 1, 2, //
                    3, 4, 5, 3, 5, 6, //
                    7, 8, 9, 7, 9, 10,
                ],
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        let nonconvex_source_straddling_hole = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 3, 2, 0, 2, 3, 0, //
                5, 10, 0, 7, 10, 0, 7, 11, 0, 5, 11, 0, //
                2, 8, 0, 6, 8, 0, 6, 12, 0, 2, 12, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let affine_contact_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 4, 0, 18, 18, 0, -2, 14, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let affine_contact_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                7, 5, 0, 12, 6, 0, 8, 9, 0, //
                5, 1, 0, 10, 2, 0, 8, 7, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let pairwise_overlap_graph_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 4, 0, 11, 7, 0, 8, 8, 0, //
                12, 10, 0, 11, 12, 0, 14, 12, 0, //
                0, 6, 0, 10, 6, 0, 12, 10, 0, 10, 14, 0, 0, 14, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, //
                6, 7, 8, 6, 8, 9, 6, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let triple_overlap_graph_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                8, 9, 0, 12, 9, 0, 8, 13, 0, //
                0, 8, 0, 10, 8, 0, 12, 10, 0, 10, 12, 0, 0, 12, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, //
                6, 7, 8, 6, 8, 9, 6, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonrect_contact_chain_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                8, 8, 0, 12, 10, 0, 8, 12, 0, //
                6, 8, 0, 8, 9, 0, 8, 11, 0, 6, 12, 0, //
                0, 9, 0, 6, 8, 0, 6, 12, 0, 0, 11, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let incidental_point_group_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                -1, 8, 0, 8, 8, 0, 8, 12, 0, -1, 12, 0, //
                6, 9, 0, 10, 10, 0, 6, 11, 0, //
                8, 10, 0, 12, 8, 0, 12, 12, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, //
                7, 8, 9,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_holed_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_holed_right = ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 4, 2, 0, 3, 4, 0, //
                8, 8, 0, 24, 4, 0, 24, 12, 0,
            ],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function(
            "exact_boolean_coplanar_convex_surface_multi_difference",
            |b| {
                b.iter(|| {
                    let arrangement =
                        arrange_coplanar_convex_surface_multi_difference(&left, &right);
                    (
                        arrangement
                            .as_ref()
                            .map(|output| output.validate_against_sources(&left, &right)),
                        arrangement.as_ref().map(|output| output.validate()),
                        arrangement,
                        hypermesh::exact::preflight_boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_multi_difference(
                            &component_left,
                            &component_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(&component_left, &component_right)
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &component_left,
                            &component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &component_left,
                            &component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_multi_difference(
                            &multi_component_left,
                            &multi_component_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_component_left,
                                &multi_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_left,
                            &multi_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_left,
                            &multi_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_multi_difference(
                            &same_component_multi_cutter_left,
                            &same_component_multi_cutter_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &same_component_multi_cutter_left,
                                &same_component_multi_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_component_multi_cutter_left,
                            &same_component_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &same_component_multi_cutter_left,
                            &same_component_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_multi_difference(
                            &nonrectangular_multi_cutter_left,
                            &nonrectangular_multi_cutter_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonrectangular_multi_cutter_left,
                                &nonrectangular_multi_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrectangular_multi_cutter_left,
                            &nonrectangular_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrectangular_multi_cutter_left,
                            &nonrectangular_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &nonrectangular_multi_cutter_left,
                            &nonconvex_multi_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonrectangular_multi_cutter_left,
                                &nonconvex_multi_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrectangular_multi_cutter_left,
                            &nonconvex_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrectangular_multi_cutter_left,
                            &nonconvex_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &nonrectilinear_channel_left,
                            &nonrectilinear_channel_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonrectilinear_channel_left,
                                &nonrectilinear_channel_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrectilinear_channel_left,
                            &nonrectilinear_channel_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrectilinear_channel_left,
                            &nonrectilinear_channel_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &nonrectilinear_channel_left,
                            &nonrectilinear_channel_consumed_only_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonrectilinear_channel_left,
                                &nonrectilinear_channel_consumed_only_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrectilinear_channel_left,
                            &nonrectilinear_channel_consumed_only_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrectilinear_channel_left,
                            &nonrectilinear_channel_consumed_only_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &crossing_side_cutter_left,
                                &crossing_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &crossing_side_cutter_left,
                                &crossing_side_cutter_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_straddling_hole_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &crossing_side_cutter_left,
                                &crossing_side_cutter_straddling_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_straddling_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &crossing_side_cutter_left,
                            &crossing_side_cutter_straddling_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &same_component_multi_cutter_left,
                            &partial_height_multi_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &same_component_multi_cutter_left,
                                &partial_height_multi_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_component_multi_cutter_left,
                            &partial_height_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &same_component_multi_cutter_left,
                            &partial_height_multi_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &component_holed_left,
                            &component_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &component_holed_left,
                                &component_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &component_holed_left,
                            &component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &component_holed_left,
                            &component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_component_holed_difference(
                            &same_outer_nested_left,
                            &same_outer_nested_right,
                        )
                        .map(|output| {
                            output.validate_surface_difference_against_sources(
                                &same_outer_nested_left,
                                &same_outer_nested_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_nested_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_difference(
                            &same_outer_partial_component_left,
                            &same_outer_partial_component_right,
                        )
                        .map(|output| {
                            output.validate_surface_difference_against_sources(
                                &same_outer_partial_component_left,
                                &same_outer_partial_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_partial_component_left,
                            &same_outer_partial_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_difference(
                            &same_outer_nonrect_partial_component_left,
                            &same_outer_partial_component_right,
                        )
                        .map(|output| {
                            output.validate_surface_difference_against_sources(
                                &same_outer_nonrect_partial_component_left,
                                &same_outer_partial_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nonrect_partial_component_left,
                            &same_outer_partial_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_difference(
                            &same_outer_mixed_partial_component_left,
                            &same_outer_mixed_partial_component_right,
                        )
                        .map(|output| {
                            output.validate_surface_difference_against_sources(
                                &same_outer_mixed_partial_component_left,
                                &same_outer_mixed_partial_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_mixed_partial_component_left,
                            &same_outer_mixed_partial_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_difference(
                            &same_outer_orthogonal_partial_component_left,
                            &same_outer_orthogonal_partial_component_right,
                        )
                        .map(|output| {
                            output.validate_surface_difference_against_sources(
                                &same_outer_orthogonal_partial_component_left,
                                &same_outer_orthogonal_partial_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_orthogonal_partial_component_left,
                            &same_outer_orthogonal_partial_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_nested_left,
                            &same_outer_nested_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_nested_left,
                                &same_outer_nested_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_nested_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_bridge_intersection_left,
                            &same_outer_bridge_intersection_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_bridge_intersection_left,
                                &same_outer_bridge_intersection_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_bridge_intersection_left,
                            &same_outer_bridge_intersection_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_bridge_intersection_left,
                            &same_outer_bridge_intersection_nonrect_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_bridge_intersection_left,
                                &same_outer_bridge_intersection_nonrect_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_bridge_intersection_left,
                            &same_outer_bridge_intersection_nonrect_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_orthogonal_intersection_left,
                            &same_outer_orthogonal_intersection_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_orthogonal_intersection_left,
                                &same_outer_orthogonal_intersection_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_orthogonal_intersection_left,
                            &same_outer_orthogonal_intersection_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_orthogonal_island_intersection_left,
                            &same_outer_orthogonal_island_intersection_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_orthogonal_island_intersection_left,
                                &same_outer_orthogonal_island_intersection_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_orthogonal_island_intersection_left,
                            &same_outer_orthogonal_island_intersection_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_point_branch_island_intersection_source,
                            &same_outer_point_branch_island_intersection_outer,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_point_branch_island_intersection_source,
                                &same_outer_point_branch_island_intersection_outer,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_point_branch_island_intersection_source,
                            &same_outer_point_branch_island_intersection_outer,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_point_branch_island_intersection_source,
                            &same_outer_point_branch_island_consumed_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_point_branch_island_intersection_source,
                                &same_outer_point_branch_island_consumed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_point_branch_island_intersection_source,
                            &same_outer_point_branch_island_consumed_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_point_branch_island_clipped_source,
                            &same_outer_point_branch_island_clipped_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_point_branch_island_clipped_source,
                                &same_outer_point_branch_island_clipped_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_point_branch_island_clipped_source,
                            &same_outer_point_branch_island_clipped_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_multi_source_island_source,
                            &same_outer_multi_source_island_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_multi_source_island_source,
                                &same_outer_multi_source_island_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_multi_source_island_source,
                            &same_outer_multi_source_island_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_multi_source_island_source,
                            &same_outer_split_source_island_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_multi_source_island_source,
                                &same_outer_split_source_island_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_multi_source_island_source,
                            &same_outer_split_source_island_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_holed_source_island_source,
                            &same_outer_holed_source_island_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_holed_source_island_source,
                                &same_outer_holed_source_island_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_holed_source_island_source,
                            &same_outer_holed_source_island_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_clipped_holed_source_island_source,
                            &same_outer_clipped_holed_source_island_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_clipped_holed_source_island_source,
                                &same_outer_clipped_holed_source_island_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_clipped_holed_source_island_source,
                            &same_outer_clipped_holed_source_island_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_split_holed_source_island_source,
                            &same_outer_split_holed_source_island_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_split_holed_source_island_source,
                                &same_outer_split_holed_source_island_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_split_holed_source_island_source,
                            &same_outer_split_holed_source_island_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_affine_island_intersection_left,
                            &same_outer_affine_island_intersection_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_affine_island_intersection_left,
                                &same_outer_affine_island_intersection_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_affine_island_intersection_left,
                            &same_outer_affine_island_intersection_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_simple_intersection_left,
                            &same_outer_simple_intersection_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_simple_intersection_left,
                                &same_outer_simple_intersection_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_simple_intersection_left,
                            &same_outer_simple_intersection_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_disconnected_intersection_left,
                            &same_outer_disconnected_intersection_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_disconnected_intersection_left,
                                &same_outer_disconnected_intersection_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_disconnected_intersection_left,
                            &same_outer_disconnected_intersection_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_intersection(
                            &same_outer_nested_left,
                            &same_outer_crossing_single_right,
                        )
                        .map(|output| {
                            output.validate_intersection_against_sources(
                                &same_outer_nested_left,
                                &same_outer_crossing_single_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_crossing_single_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_multi_difference(
                            &same_outer_nested_left,
                            &same_outer_disjoint_multi_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &same_outer_nested_left,
                                &same_outer_disjoint_multi_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_disjoint_multi_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_multi_difference(
                            &same_outer_nested_left,
                            &same_outer_crossing_multi_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &same_outer_nested_left,
                                &same_outer_crossing_multi_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_crossing_multi_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_multi_difference(
                            &same_outer_orthogonal_split_multi_left,
                            &same_outer_orthogonal_split_multi_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &same_outer_orthogonal_split_multi_left,
                                &same_outer_orthogonal_split_multi_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_orthogonal_split_multi_left,
                            &same_outer_orthogonal_split_multi_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &same_outer_nested_left,
                            &same_outer_disjoint_single_right,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &same_outer_nested_left,
                                &same_outer_disjoint_single_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_disjoint_single_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &same_outer_nested_left,
                            &same_outer_crossing_single_right,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &same_outer_nested_left,
                                &same_outer_crossing_single_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_crossing_single_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &same_outer_nonrect_component_left,
                            &same_outer_nonrect_component_right,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &same_outer_nonrect_component_left,
                                &same_outer_nonrect_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nonrect_component_left,
                            &same_outer_nonrect_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &same_outer_mixed_component_left,
                            &same_outer_mixed_component_right,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &same_outer_mixed_component_left,
                                &same_outer_mixed_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_mixed_component_left,
                            &same_outer_mixed_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &same_outer_simple_component_left,
                            &same_outer_simple_component_right,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &same_outer_simple_component_left,
                                &same_outer_simple_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_simple_component_left,
                            &same_outer_simple_component_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_union(
                            &same_outer_simple_component_left,
                            &same_outer_simple_component_right,
                        )
                        .map(|output| {
                            output.validate_union_against_sources(
                                &same_outer_simple_component_left,
                                &same_outer_simple_component_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_simple_component_left,
                            &same_outer_simple_component_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_union(
                            &same_outer_nested_left,
                            &same_outer_disjoint_single_right,
                        )
                        .map(|output| {
                            output.validate_component_union_against_sources(
                                &same_outer_nested_left,
                                &same_outer_disjoint_single_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_nested_left,
                            &same_outer_disjoint_single_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_union(
                            &same_outer_common_left,
                            &same_outer_common_right,
                        )
                        .map(|output| {
                            output.validate_union_against_sources(
                                &same_outer_common_left,
                                &same_outer_common_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_common_left,
                            &same_outer_common_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_union(
                            &same_outer_common_left,
                            &same_outer_partial_union_right,
                        )
                        .map(|output| {
                            output.validate_union_against_sources(
                                &same_outer_common_left,
                                &same_outer_partial_union_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_common_left,
                            &same_outer_partial_union_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_union(
                            &same_outer_common_left,
                            &same_outer_nonrect_partial_union_right,
                        )
                        .map(|output| {
                            output.validate_union_against_sources(
                                &same_outer_common_left,
                                &same_outer_nonrect_partial_union_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_common_left,
                            &same_outer_nonrect_partial_union_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_holed_union(
                            &same_outer_orthogonal_retained_union_left,
                            &same_outer_orthogonal_retained_union_right,
                        )
                        .map(|output| {
                            output.validate_union_against_sources(
                                &same_outer_orthogonal_retained_union_left,
                                &same_outer_orthogonal_retained_union_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &same_outer_orthogonal_retained_union_left,
                            &same_outer_orthogonal_retained_union_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &component_holed_left,
                            &component_holed_cut_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &component_holed_left,
                                &component_holed_cut_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &component_holed_left,
                            &component_holed_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &component_holed_left,
                            &component_holed_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &component_holed_left,
                            &component_holed_multi_cut_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &component_holed_left,
                                &component_holed_multi_cut_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &component_holed_left,
                            &component_holed_multi_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &component_holed_left,
                            &component_holed_multi_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &component_holed_left,
                            &component_holed_partial_height_multi_cut_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &component_holed_left,
                                &component_holed_partial_height_multi_cut_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &component_holed_left,
                            &component_holed_partial_height_multi_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &component_holed_left,
                            &component_holed_partial_height_multi_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &component_holed_left,
                            &component_holed_corner_cut_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &component_holed_left,
                                &component_holed_corner_cut_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &component_holed_left,
                            &component_holed_corner_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &component_holed_left,
                            &component_holed_corner_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &single_component_holed_left,
                            &component_holed_cut_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &single_component_holed_left,
                                &component_holed_cut_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &single_component_holed_left,
                            &component_holed_cut_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonrectilinear_channel_holed_left,
                            &nonrectilinear_channel_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonrectilinear_channel_holed_left,
                                &nonrectilinear_channel_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrectilinear_channel_holed_left,
                            &nonrectilinear_channel_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonrectilinear_channel_holed_left,
                            &nonrectilinear_channel_consumed_hole_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonrectilinear_channel_holed_left,
                                &nonrectilinear_channel_consumed_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrectilinear_channel_holed_left,
                            &nonrectilinear_channel_consumed_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &multi_component_consumed_hole_left,
                            &multi_component_consumed_hole_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_component_consumed_hole_left,
                                &multi_component_consumed_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_consumed_hole_left,
                            &multi_component_consumed_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_consumed_hole_left,
                            &multi_component_consumed_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &multi_component_no_hole_consumed_left,
                            &multi_component_no_hole_consumed_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_component_no_hole_consumed_left,
                                &multi_component_no_hole_consumed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_no_hole_consumed_left,
                            &multi_component_no_hole_consumed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_no_hole_consumed_left,
                            &multi_component_no_hole_consumed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_holed_left,
                            &nonconvex_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_holed_left,
                                &nonconvex_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_holed_left,
                            &nonconvex_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_holed_left,
                            &nonconvex_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &single_component_holed_left,
                            &component_holed_contact_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &single_component_holed_left,
                                &component_holed_contact_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &single_component_holed_left,
                            &component_holed_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &single_component_holed_left,
                            &component_holed_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &nonrect_contact_left,
                            &nonrect_contact_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &nonrect_contact_left,
                                &nonrect_contact_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrect_contact_left,
                            &nonrect_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrect_contact_left,
                            &nonrect_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &nonrect_contact_left,
                            &straddling_contact_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &nonrect_contact_left,
                                &straddling_contact_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrect_contact_left,
                            &straddling_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrect_contact_left,
                            &straddling_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &nonrect_contact_left,
                            &independent_openings_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &nonrect_contact_left,
                                &independent_openings_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrect_contact_left,
                            &independent_openings_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrect_contact_left,
                            &independent_openings_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &contact_opening_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &contact_opening_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &contact_opening_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &contact_opening_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &independent_openings_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &independent_openings_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &independent_openings_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &independent_openings_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &connected_multi_cutter_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &connected_multi_cutter_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &connected_multi_cutter_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &connected_multi_cutter_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &multiple_side_cutter_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &multiple_side_cutter_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &multiple_side_cutter_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &multiple_side_cutter_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &single_consumed_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &single_consumed_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &single_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &single_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &contact_opening_holed_left,
                            &single_consumed_only_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &contact_opening_holed_left,
                                &single_consumed_only_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &single_consumed_only_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &single_consumed_only_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &multi_component_point_touch_left,
                            &single_consumed_only_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_component_point_touch_left,
                                &single_consumed_only_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_point_touch_left,
                            &single_consumed_only_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_point_touch_left,
                            &single_consumed_only_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &consumed_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &consumed_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &straddling_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &straddling_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &split_straddling_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &split_straddling_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &split_straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &split_straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &contact_opening_holed_left,
                            &split_all_consumed_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &contact_opening_holed_left,
                                &split_all_consumed_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &split_all_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &split_all_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &single_split_consumed_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &single_split_consumed_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &single_split_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &single_split_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &multi_branch_consumed_left,
                            &multi_branch_consumed_with_retained_holes,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_branch_consumed_left,
                                &multi_branch_consumed_with_retained_holes,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_branch_consumed_left,
                            &multi_branch_consumed_with_retained_holes,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_branch_consumed_left,
                            &multi_branch_consumed_with_retained_holes,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &multi_branch_consumed_left,
                            &multi_branch_all_consumed_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_branch_consumed_left,
                                &multi_branch_all_consumed_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_branch_consumed_left,
                            &multi_branch_all_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_branch_consumed_left,
                            &multi_branch_all_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &multi_branch_consumed_left,
                            &branch_group_consumed_with_retained_holes,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_branch_consumed_left,
                                &branch_group_consumed_with_retained_holes,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_branch_consumed_left,
                            &branch_group_consumed_with_retained_holes,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_branch_consumed_left,
                            &branch_group_consumed_with_retained_holes,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &multi_branch_consumed_left,
                            &branch_group_all_consumed_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_branch_consumed_left,
                                &branch_group_all_consumed_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_branch_consumed_left,
                            &branch_group_all_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_branch_consumed_left,
                            &branch_group_all_consumed_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &contact_opening_holed_left,
                            &mixed_consumed_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &contact_opening_holed_left,
                                &mixed_consumed_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &mixed_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &mixed_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_side_cutter_difference(
                            &contact_opening_holed_left,
                            &single_side_cutter_no_hole_right,
                        )
                        .map(|output| {
                            output.validate_side_cutter_difference_against_sources(
                                &contact_opening_holed_left,
                                &single_side_cutter_no_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &single_side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &single_side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_side_cutter_difference(
                            &contact_opening_holed_left,
                            &side_cutter_no_hole_right,
                        )
                        .map(|output| {
                            output.validate_side_cutter_difference_against_sources(
                                &contact_opening_holed_left,
                                &side_cutter_no_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &multi_component_point_touch_left,
                            &single_side_cutter_no_hole_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_component_point_touch_left,
                                &single_side_cutter_no_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_point_touch_left,
                            &single_side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_point_touch_left,
                            &single_side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_multi_difference(
                            &multi_component_point_touch_left,
                            &side_cutter_no_hole_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_component_point_touch_left,
                                &side_cutter_no_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_point_touch_left,
                            &side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_point_touch_left,
                            &side_cutter_no_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &contact_opening_holed_left,
                            &point_touch_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &contact_opening_holed_left,
                                &point_touch_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &vertex_edge_point_touch_left,
                            &vertex_edge_point_touch_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &vertex_edge_point_touch_left,
                                &vertex_edge_point_touch_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &vertex_edge_point_touch_left,
                            &vertex_edge_point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &vertex_edge_point_touch_left,
                            &vertex_edge_point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &nonconvex_vertex_edge_point_touch_left,
                            &nonconvex_vertex_edge_point_touch_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonconvex_vertex_edge_point_touch_left,
                                &nonconvex_vertex_edge_point_touch_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_vertex_edge_point_touch_left,
                            &nonconvex_vertex_edge_point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_vertex_edge_point_touch_left,
                            &nonconvex_vertex_edge_point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &contact_opening_holed_left,
                            &point_touch_consumed_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &contact_opening_holed_left,
                                &point_touch_consumed_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &contact_opening_holed_left,
                            &point_touch_straddling_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &contact_opening_holed_left,
                                &point_touch_straddling_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &grouped_point_touch_straddling_hole_left,
                            &grouped_point_touch_straddling_hole_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &grouped_point_touch_straddling_hole_left,
                                &grouped_point_touch_straddling_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &grouped_point_touch_straddling_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &grouped_point_touch_straddling_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &grouped_point_touch_straddling_hole_left,
                            &orthogonal_grouped_point_touch_straddling_hole_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &grouped_point_touch_straddling_hole_left,
                                &orthogonal_grouped_point_touch_straddling_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &orthogonal_grouped_point_touch_straddling_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &orthogonal_grouped_point_touch_straddling_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &grouped_point_touch_straddling_hole_left,
                            &grouped_point_touch_straddling_retained_hole_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &grouped_point_touch_straddling_hole_left,
                                &grouped_point_touch_straddling_retained_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &grouped_point_touch_straddling_retained_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &grouped_point_touch_straddling_retained_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &grouped_point_touch_straddling_hole_left,
                            &orthogonal_grouped_point_touch_straddling_retained_hole_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &grouped_point_touch_straddling_hole_left,
                                &orthogonal_grouped_point_touch_straddling_retained_hole_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &orthogonal_grouped_point_touch_straddling_retained_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &grouped_point_touch_straddling_hole_left,
                            &orthogonal_grouped_point_touch_straddling_retained_hole_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &multi_component_grouped_point_touch_left,
                            &multi_component_grouped_point_touch_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_component_grouped_point_touch_left,
                                &multi_component_grouped_point_touch_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_grouped_point_touch_left,
                            &multi_component_grouped_point_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_grouped_point_touch_left,
                            &multi_component_grouped_point_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &multi_component_point_touch_left,
                            &point_touch_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_component_point_touch_left,
                                &point_touch_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_point_touch_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_point_touch_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &multi_component_point_touch_left,
                            &multi_component_point_touch_straddling_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_component_point_touch_left,
                                &multi_component_point_touch_straddling_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_point_touch_left,
                            &multi_component_point_touch_straddling_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_point_touch_left,
                            &multi_component_point_touch_straddling_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &point_touch_component_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &point_touch_component_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &contact_opening_holed_left,
                            &point_touch_straddling_component_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &contact_opening_holed_left,
                                &point_touch_straddling_component_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_straddling_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &point_touch_straddling_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonconvex_point_touch_side_cutter_left,
                                &point_touch_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_consumed_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonconvex_point_touch_side_cutter_left,
                                &point_touch_consumed_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_consumed_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_straddling_hole_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonconvex_point_touch_side_cutter_left,
                                &point_touch_straddling_hole_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &point_touch_straddling_hole_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &nonconvex_grouped_point_touch_left,
                            &nonconvex_grouped_point_touch_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonconvex_grouped_point_touch_left,
                                &nonconvex_grouped_point_touch_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_grouped_point_touch_left,
                            &nonconvex_grouped_point_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_grouped_point_touch_left,
                            &nonconvex_grouped_point_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_grouped_point_touch_left,
                            &nonconvex_grouped_point_touch_retained_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_grouped_point_touch_left,
                                &nonconvex_grouped_point_touch_retained_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_grouped_point_touch_left,
                            &nonconvex_grouped_point_touch_retained_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_grouped_point_touch_left,
                            &nonconvex_grouped_point_touch_retained_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &multi_component_nonconvex_grouped_point_touch_left,
                            &multi_component_nonconvex_grouped_point_touch_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_component_nonconvex_grouped_point_touch_left,
                                &multi_component_nonconvex_grouped_point_touch_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_nonconvex_grouped_point_touch_left,
                            &multi_component_nonconvex_grouped_point_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_nonconvex_grouped_point_touch_left,
                            &multi_component_nonconvex_grouped_point_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_point_touch_difference(
                            &multi_component_nonconvex_point_touch_left,
                            &point_touch_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &multi_component_nonconvex_point_touch_left,
                                &point_touch_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_nonconvex_point_touch_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_nonconvex_point_touch_left,
                            &point_touch_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &multi_component_nonconvex_point_touch_left,
                            &multi_component_point_touch_straddling_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &multi_component_nonconvex_point_touch_left,
                                &multi_component_point_touch_straddling_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &multi_component_nonconvex_point_touch_left,
                            &multi_component_point_touch_straddling_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &multi_component_nonconvex_point_touch_left,
                            &multi_component_point_touch_straddling_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_point_touch_side_cutter_left,
                            &nonconvex_point_touch_component_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_point_touch_side_cutter_left,
                                &nonconvex_point_touch_component_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &nonconvex_point_touch_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &nonconvex_point_touch_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_point_touch_side_cutter_left,
                            &nonconvex_point_touch_straddling_component_holed_right,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_point_touch_side_cutter_left,
                                &nonconvex_point_touch_straddling_component_holed_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &nonconvex_point_touch_straddling_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_point_touch_side_cutter_left,
                            &nonconvex_point_touch_straddling_component_holed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_side_cutter_difference(
                            &contact_opening_holed_left,
                            &incidental_side_cutter_right,
                        )
                        .map(|output| {
                            output.validate_side_cutter_difference_against_sources(
                                &contact_opening_holed_left,
                                &incidental_side_cutter_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &contact_opening_holed_left,
                            &incidental_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &contact_opening_holed_left,
                            &incidental_side_cutter_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_component_difference(
                            &component_opening_left,
                            &component_opening_right,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &component_opening_left,
                                &component_opening_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &component_opening_left,
                            &component_opening_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &component_opening_left,
                            &component_opening_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_component_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_opening,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_opening,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_opening,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_opening,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_hole,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_opening_and_hole,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_opening_and_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_opening_and_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_crossing_opening,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_crossing_opening,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_crossing_opening,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_crossing_opening_consumed_hole,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_crossing_opening_consumed_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_crossing_opening_consumed_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_multi_difference(
                            &nonconvex_split_left,
                            &nonconvex_split_crossing_consumed_hole,
                        )
                        .map(|output| {
                            output.validate_difference_against_sources(
                                &nonconvex_split_left,
                                &nonconvex_split_crossing_consumed_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_split_left,
                            &nonconvex_split_crossing_consumed_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_split_left,
                            &nonconvex_split_crossing_consumed_and_retained_holes,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_split_left,
                                &nonconvex_split_crossing_consumed_and_retained_holes,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_split_left,
                            &nonconvex_split_crossing_consumed_and_retained_holes,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_overlapping_crossing_openings,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_overlapping_crossing_openings,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_overlapping_crossing_openings,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_component_difference(
                            &nonconvex_source_incidental_left,
                            &nonconvex_source_incidental_openings,
                        )
                        .map(|output| {
                            output.validate_component_difference_against_sources(
                                &nonconvex_source_incidental_left,
                                &nonconvex_source_incidental_openings,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_incidental_left,
                            &nonconvex_source_incidental_openings,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonconvex_source_incidental_left,
                            &nonconvex_source_incidental_openings,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_crossing_opening_and_hole,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_crossing_opening_and_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_crossing_opening_and_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_overlapping_crossing_openings_and_hole,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_overlapping_crossing_openings_and_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_overlapping_crossing_openings_and_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_clipped_straddling_hole_and_retained_hole,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_clipped_straddling_hole_and_retained_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_clipped_straddling_hole_and_retained_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_convex_surface_component_holed_difference(
                            &nonconvex_source_left,
                            &nonconvex_source_straddling_hole,
                        )
                        .map(|output| {
                            output.validate_against_sources(
                                &nonconvex_source_left,
                                &nonconvex_source_straddling_hole,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonconvex_source_left,
                            &nonconvex_source_straddling_hole,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &affine_contact_left,
                            &affine_contact_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &affine_contact_left,
                                &affine_contact_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &affine_contact_left,
                            &affine_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &affine_contact_left,
                            &affine_contact_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &nonrect_contact_left,
                            &pairwise_overlap_graph_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &nonrect_contact_left,
                                &pairwise_overlap_graph_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrect_contact_left,
                            &pairwise_overlap_graph_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrect_contact_left,
                            &pairwise_overlap_graph_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &nonrect_contact_left,
                            &triple_overlap_graph_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &nonrect_contact_left,
                                &triple_overlap_graph_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrect_contact_left,
                            &triple_overlap_graph_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrect_contact_left,
                            &triple_overlap_graph_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &nonrect_contact_left,
                            &nonrect_contact_chain_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &nonrect_contact_left,
                                &nonrect_contact_chain_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrect_contact_left,
                            &nonrect_contact_chain_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrect_contact_left,
                            &nonrect_contact_chain_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                        arrange_coplanar_surface_cutter_hole_contact_difference(
                            &nonrect_contact_left,
                            &incidental_point_group_right,
                        )
                        .map(|output| {
                            output.validate_cutter_hole_contact_difference_against_sources(
                                &nonrect_contact_left,
                                &incidental_point_group_right,
                            )
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &nonrect_contact_left,
                            &incidental_point_group_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &nonrect_contact_left,
                            &incidental_point_group_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                    )
                })
            },
        );
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_convex_surface_multi_holed_difference(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, 6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function(
            "exact_boolean_coplanar_convex_surface_multi_holed_difference",
            |b| {
                b.iter(|| {
                    let arrangement =
                        arrange_coplanar_convex_surface_multi_holed_difference(&left, &right);
                    (
                        arrangement
                            .as_ref()
                            .map(|output| output.validate_against_sources(&left, &right)),
                        arrangement.as_ref().map(|output| output.validate()),
                        arrangement,
                        hypermesh::exact::preflight_boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                    )
                })
            },
        );
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_orthogonal_surface_cells(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let l_left = rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)]);
        let l_right = rect_surface_i64(&[(2, 2, 4, 4)]);
        let fan_l_left = fan_rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)]);
        let intersection_left = rect_surface_i64(&[(0, 0, 6, 2), (0, 2, 2, 6)]);
        let intersection_right = rect_surface_i64(&[(0, 0, 6, 6)]);
        let holed_left = rect_surface_i64(&[(0, 0, 10, 10), (10, 0, 12, 2)]);
        let holed_right = rect_surface_i64(&[(2, 2, 4, 4)]);
        let nested_left = rect_surface_i64(&[(0, 0, 10, 10)]);
        let nested_right =
            rect_surface_i64(&[(2, 2, 8, 4), (2, 6, 8, 8), (2, 4, 4, 6), (6, 4, 8, 6)]);
        let graph_left = rect_surface_i64(&[(0, 0, 12, 10)]);
        let graph_right =
            rect_surface_i64(&[(3, 3, 5, 5), (7, 3, 9, 5), (5, 4, 7, 5), (-1, 4, 3, 5)]);
        let overlap_source_left = rect_surface_i64(&[(0, 0, 4, 6), (2, 2, 8, 4)]);
        let overlap_source_right = rect_surface_i64(&[(8, 2, 10, 4)]);
        let rectangular_overlap_left = rect_surface_i64(&[(0, 0, 20, 20)]);
        let rectangular_overlap_right = rect_surface_i64(&[(8, 8, 12, 12), (0, 9, 10, 11)]);
        let branch_left = rect_surface_i64(&[(0, 0, 4, 4)]);
        let branch_right = rect_surface_i64(&[(0, 2, 2, 4), (2, 0, 4, 2)]);
        let hole_branch_left = rect_surface_i64(&[(0, 0, 5, 5)]);
        let hole_branch_right = rect_surface_i64(&[
            (1, 2, 2, 3),
            (1, 3, 2, 4),
            (2, 1, 3, 2),
            (2, 3, 3, 4),
            (3, 1, 4, 2),
            (3, 2, 4, 3),
            (3, 3, 4, 4),
        ]);
        let retained_outer = vec![
            p3(0, 0, 0),
            p3(6, 0, 0),
            p3(6, 1, 0),
            p3(1, 1, 0),
            p3(1, 5, 0),
            p3(6, 5, 0),
            p3(6, 6, 0),
            p3(0, 6, 0),
        ];
        let orthogonal_fan_rejection = hypermesh::exact::CoplanarOrthogonalSurfaceArrangement {
            projection: hypermesh::exact::CoplanarProjection::Xy,
            operation: hypermesh::exact::CoplanarOrthogonalSurfaceOperation::Union,
            components: vec![hypermesh::exact::CoplanarOrthogonalSurfaceComponent {
                outer: retained_outer.clone(),
                holes: Vec::new(),
            }],
            mesh: fan_surface_mesh_from_points(&retained_outer),
        };

        c.bench_function("exact_boolean_coplanar_orthogonal_surface_cells", |b| {
            b.iter(|| {
                let union = arrange_coplanar_orthogonal_surface_union(&l_left, &l_right);
                let fan_union = arrange_coplanar_orthogonal_surface_union(&fan_l_left, &l_right);
                let intersection = arrange_coplanar_orthogonal_surface_intersection(
                    &intersection_left,
                    &intersection_right,
                );
                let difference =
                    arrange_coplanar_orthogonal_surface_difference(&holed_left, &holed_right);
                let nested_difference =
                    arrange_coplanar_orthogonal_surface_difference(&nested_left, &nested_right);
                let graph_difference =
                    arrange_coplanar_orthogonal_surface_difference(&graph_left, &graph_right);
                let graph_contact_fallback =
                    arrange_coplanar_surface_cutter_hole_contact_difference(
                        &graph_left,
                        &graph_right,
                    );
                let overlap_union = arrange_coplanar_orthogonal_surface_union(
                    &overlap_source_left,
                    &overlap_source_right,
                );
                let rectangular_overlap_difference = arrange_coplanar_orthogonal_surface_difference(
                    &rectangular_overlap_left,
                    &rectangular_overlap_right,
                );
                let branch_difference =
                    arrange_coplanar_orthogonal_surface_difference(&branch_left, &branch_right);
                let hole_branch_difference = arrange_coplanar_orthogonal_surface_difference(
                    &hole_branch_left,
                    &hole_branch_right,
                );
                let union_result = hypermesh::exact::boolean_exact(
                    &l_left,
                    &l_right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let intersection_result = hypermesh::exact::boolean_exact(
                    &intersection_left,
                    &intersection_right,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let difference_result = hypermesh::exact::boolean_exact(
                    &holed_left,
                    &holed_right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let nested_difference_result = hypermesh::exact::boolean_exact(
                    &nested_left,
                    &nested_right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let overlap_union_result = hypermesh::exact::boolean_exact(
                    &overlap_source_left,
                    &overlap_source_right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let rectangular_overlap_difference_result = hypermesh::exact::boolean_exact(
                    &rectangular_overlap_left,
                    &rectangular_overlap_right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let branch_difference_result = hypermesh::exact::boolean_exact(
                    &branch_left,
                    &branch_right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                let hole_branch_difference_result = hypermesh::exact::boolean_exact(
                    &hole_branch_left,
                    &hole_branch_right,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                (
                    union
                        .as_ref()
                        .map(|output| output.validate_against_sources(&l_left, &l_right)),
                    union.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &l_left,
                        &l_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| {
                        (
                            report.validate(),
                            report.validate_against_sources(&l_left, &l_right),
                        )
                    }),
                    union_result.validate_operation_against_sources(
                        &l_left,
                        &l_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    fan_union
                        .as_ref()
                        .map(|output| output.validate_against_sources(&fan_l_left, &l_right)),
                    fan_union.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &fan_l_left,
                        &l_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| {
                        (
                            report.validate(),
                            report.validate_against_sources(&fan_l_left, &l_right),
                        )
                    }),
                    intersection.as_ref().map(|output| {
                        output.validate_against_sources(&intersection_left, &intersection_right)
                    }),
                    intersection.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &intersection_left,
                        &intersection_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    intersection_result.validate_operation_against_sources(
                        &intersection_left,
                        &intersection_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&holed_left, &holed_right)),
                    difference.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &holed_left,
                        &holed_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    difference_result.validate_operation_against_sources(
                        &holed_left,
                        &holed_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    nested_difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&nested_left, &nested_right)),
                    nested_difference.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &nested_left,
                        &nested_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    nested_difference_result.validate_operation_against_sources(
                        &nested_left,
                        &nested_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    graph_contact_fallback.is_none(),
                    graph_difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&graph_left, &graph_right)),
                    graph_difference.as_ref().map(|output| output.validate()),
                    orthogonal_fan_rejection.validate().is_err(),
                    hypermesh::exact::preflight_boolean_exact(
                        &graph_left,
                        &graph_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    overlap_union.as_ref().map(|output| {
                        output.validate_against_sources(&overlap_source_left, &overlap_source_right)
                    }),
                    overlap_union.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &overlap_source_left,
                        &overlap_source_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    overlap_union_result.validate_operation_against_sources(
                        &overlap_source_left,
                        &overlap_source_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    rectangular_overlap_difference.as_ref().map(|output| {
                        output.validate_against_sources(
                            &rectangular_overlap_left,
                            &rectangular_overlap_right,
                        )
                    }),
                    rectangular_overlap_difference
                        .as_ref()
                        .map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &rectangular_overlap_left,
                        &rectangular_overlap_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    rectangular_overlap_difference_result.validate_operation_against_sources(
                        &rectangular_overlap_left,
                        &rectangular_overlap_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    branch_difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&branch_left, &branch_right)),
                    branch_difference.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &branch_left,
                        &branch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    branch_difference_result.validate_operation_against_sources(
                        &branch_left,
                        &branch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    hole_branch_difference.as_ref().map(|output| {
                        output.validate_against_sources(&hole_branch_left, &hole_branch_right)
                    }),
                    hole_branch_difference
                        .as_ref()
                        .map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &hole_branch_left,
                        &hole_branch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hole_branch_difference_result.validate_operation_against_sources(
                        &hole_branch_left,
                        &hole_branch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_affine_surface_cells(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let origin = (0, 0, 0);
        let basis_u = (2, 1, 0);
        let basis_v = (-1, 2, 0);
        let l_left =
            affine_rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)], origin, basis_u, basis_v);
        let l_right = affine_rect_surface_i64(&[(2, 2, 4, 4)], origin, basis_u, basis_v);
        let fan_l_left =
            affine_fan_rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)], origin, basis_u, basis_v);
        let intersection_left =
            affine_rect_surface_i64(&[(0, 0, 6, 2), (0, 2, 2, 6)], origin, basis_u, basis_v);
        let intersection_right = affine_rect_surface_i64(&[(0, 0, 6, 6)], origin, basis_u, basis_v);
        let holed_left =
            affine_rect_surface_i64(&[(0, 0, 10, 10), (10, 0, 12, 2)], origin, basis_u, basis_v);
        let holed_right = affine_rect_surface_i64(&[(2, 2, 4, 4)], origin, basis_u, basis_v);
        let nested_left = affine_rect_surface_i64(&[(0, 0, 10, 10)], origin, basis_u, basis_v);
        let nested_right = affine_rect_surface_i64(
            &[(2, 2, 8, 4), (2, 6, 8, 8), (2, 4, 4, 6), (6, 4, 8, 6)],
            origin,
            basis_u,
            basis_v,
        );
        let branch_left = affine_rect_surface_i64(&[(0, 0, 4, 4)], origin, basis_u, basis_v);
        let branch_right =
            affine_rect_surface_i64(&[(0, 2, 2, 4), (2, 0, 4, 2)], origin, basis_u, basis_v);
        let hole_branch_left = affine_rect_surface_i64(&[(0, 0, 5, 5)], origin, basis_u, basis_v);
        let hole_branch_right = affine_rect_surface_i64(
            &[
                (1, 2, 2, 3),
                (1, 3, 2, 4),
                (2, 1, 3, 2),
                (2, 3, 3, 4),
                (3, 1, 4, 2),
                (3, 2, 4, 3),
                (3, 3, 4, 4),
            ],
            origin,
            basis_u,
            basis_v,
        );
        let lift = |u: i32, v: i32| p3(2 * u - v, u + 2 * v, 0);
        let affine_outer = vec![
            lift(0, 0),
            lift(6, 0),
            lift(6, 1),
            lift(1, 1),
            lift(1, 5),
            lift(6, 5),
            lift(6, 6),
            lift(0, 6),
        ];
        let affine_fan_rejection = hypermesh::exact::CoplanarAffineSurfaceArrangement {
            basis: hypermesh::exact::CoplanarAffineSurfaceBasis {
                projection: hypermesh::exact::CoplanarProjection::Xy,
                origin: p3(0, 0, 0),
                basis_u: p3(2, 1, 0),
                basis_v: p3(-1, 2, 0),
            },
            operation: hypermesh::exact::CoplanarOrthogonalSurfaceOperation::Union,
            components: vec![hypermesh::exact::CoplanarOrthogonalSurfaceComponent {
                outer: affine_outer.clone(),
                holes: Vec::new(),
            }],
            mesh: fan_surface_mesh_from_points(&affine_outer),
        };

        c.bench_function("exact_boolean_coplanar_affine_surface_cells", |b| {
            b.iter(|| {
                let union = arrange_coplanar_affine_surface_union(&l_left, &l_right);
                let fan_union = arrange_coplanar_affine_surface_union(&fan_l_left, &l_right);
                let intersection = arrange_coplanar_affine_surface_intersection(
                    &intersection_left,
                    &intersection_right,
                );
                let difference =
                    arrange_coplanar_affine_surface_difference(&holed_left, &holed_right);
                let nested_difference =
                    arrange_coplanar_affine_surface_difference(&nested_left, &nested_right);
                let branch_difference =
                    arrange_coplanar_affine_surface_difference(&branch_left, &branch_right);
                let hole_branch_difference = arrange_coplanar_affine_surface_difference(
                    &hole_branch_left,
                    &hole_branch_right,
                );
                let union_result = hypermesh::exact::boolean_exact(
                    &l_left,
                    &l_right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap();
                (
                    union
                        .as_ref()
                        .map(|output| output.validate_against_sources(&l_left, &l_right)),
                    hypermesh::exact::preflight_boolean_exact(
                        &l_left,
                        &l_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    union_result.validate_operation_against_sources(
                        &l_left,
                        &l_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    fan_union
                        .as_ref()
                        .map(|output| output.validate_against_sources(&fan_l_left, &l_right)),
                    fan_union.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &fan_l_left,
                        &l_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| {
                        (
                            report.validate(),
                            report.validate_against_sources(&fan_l_left, &l_right),
                        )
                    }),
                    intersection.as_ref().map(|output| {
                        output.validate_against_sources(&intersection_left, &intersection_right)
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &intersection_left,
                        &intersection_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&holed_left, &holed_right)),
                    affine_fan_rejection.validate().is_err(),
                    hypermesh::exact::preflight_boolean_exact(
                        &holed_left,
                        &holed_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    nested_difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&nested_left, &nested_right)),
                    hypermesh::exact::preflight_boolean_exact(
                        &nested_left,
                        &nested_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    branch_difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&branch_left, &branch_right)),
                    branch_difference.as_ref().map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &branch_left,
                        &branch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hole_branch_difference.as_ref().map(|output| {
                        output.validate_against_sources(&hole_branch_left, &hole_branch_right)
                    }),
                    hole_branch_difference
                        .as_ref()
                        .map(|output| output.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &hole_branch_left,
                        &hole_branch_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_affine_box_cells(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let origin = [0, 0, 0];
        let basis_u = [2, 1, 0];
        let basis_v = [-1, 2, 0];
        let basis_w = [0, 1, 2];
        let left = affine_box_i64([0, 0, 0], [2, 2, 2], origin, basis_u, basis_v, basis_w);
        let right = affine_box_i64([1, 1, 0], [3, 3, 2], origin, basis_u, basis_v, basis_w);
        let affine_complex = hypermesh::exact::boolean_exact(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .mesh;
        let affine_cutter = affine_box_i64([2, 0, 0], [3, 2, 2], origin, basis_u, basis_v, basis_w);
        let affine_outer = affine_box_i64([0, 0, 0], [8, 8, 8], origin, basis_u, basis_v, basis_w);
        let affine_cavity = affine_box_i64([2, 2, 2], [6, 6, 6], origin, basis_u, basis_v, basis_w);
        let affine_hollow = hypermesh::exact::boolean_exact(
            &affine_outer,
            &affine_cavity,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .mesh;
        let affine_floating =
            affine_box_i64([3, 3, 3], [5, 5, 5], origin, basis_u, basis_v, basis_w);
        let discovered_left = affine_complex.clone();
        let discovered_right_a =
            affine_box_i64([2, 0, 0], [4, 2, 2], origin, basis_u, basis_v, basis_w);
        let discovered_right_b =
            affine_box_i64([3, 1, 0], [5, 3, 2], origin, basis_u, basis_v, basis_w);
        let discovered_right = hypermesh::exact::boolean_exact(
            &discovered_right_a,
            &discovered_right_b,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .mesh;

        c.bench_function("exact_boolean_affine_box_cells", |b| {
            b.iter(|| {
                let union = hypermesh::exact::materialize_affine_box_union(
                    &left,
                    &right,
                    ValidationPolicy::CLOSED,
                )
                .unwrap();
                let intersection = hypermesh::exact::materialize_affine_box_intersection(
                    &left,
                    &right,
                    ValidationPolicy::CLOSED,
                )
                .unwrap();
                let difference = hypermesh::exact::materialize_affine_box_difference(
                    &left,
                    &right,
                    ValidationPolicy::CLOSED,
                )
                .unwrap();
                (
                    union
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &right)),
                    intersection
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &right)),
                    difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &right)),
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap()
                    .validate_operation_against_sources(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap()
                    .validate_operation_against_sources(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap()
                    .validate_operation_against_sources(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                )
            })
        });

        c.bench_function("exact_boolean_affine_orthogonal_solid_cells", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::materialize_affine_orthogonal_solid_union(
                        &affine_complex,
                        &affine_cutter,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap()
                    .map(|output| output.validate_against_sources(&affine_complex, &affine_cutter)),
                    hypermesh::exact::preflight_boolean_exact(
                        &affine_complex,
                        &affine_cutter,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &affine_complex,
                        &affine_cutter,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap()
                    .validate_operation_against_sources(
                        &affine_complex,
                        &affine_cutter,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    hypermesh::exact::boolean_exact(
                        &affine_complex,
                        &affine_cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap()
                    .validate_operation_against_sources(
                        &affine_complex,
                        &affine_cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                    hypermesh::exact::preflight_boolean_exact(
                        &affine_hollow,
                        &affine_floating,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &affine_hollow,
                        &affine_floating,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap()
                    .validate_operation_against_sources(
                        &affine_hollow,
                        &affine_floating,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                        hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                    ),
                )
            })
        });

        c.bench_function(
            "exact_boolean_affine_orthogonal_solid_frame_discovery",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_affine_orthogonal_solid_union(
                            &discovered_left,
                            &discovered_right,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap()
                        .map(|output| {
                            output.validate_against_sources(&discovered_left, &discovered_right)
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &discovered_left,
                            &discovered_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &discovered_left,
                            &discovered_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap()
                        .validate_operation_against_sources(
                            &discovered_left,
                            &discovered_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                            ValidationPolicy::CLOSED,
                            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                        ),
                        hypermesh::exact::boolean_exact(
                            &discovered_left,
                            &discovered_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap()
                        .validate_operation_against_sources(
                            &discovered_left,
                            &discovered_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                        ),
                    )
                })
            },
        );
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_convex_solid_classification(c: &mut Criterion) {
    let outer = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            10, 0, 0, //
            0, 10, 0, //
            0, 0, 10,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let inner = ExactMesh::from_i64_triangles(
        &[
            1, 1, 1, //
            2, 1, 1, //
            1, 2, 1, //
            1, 1, 2,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    c.bench_function("exact_convex_solid_classification", |b| {
        b.iter(|| {
            let facts = certify_convex_solid(&outer);
            let report = classify_mesh_vertices_against_convex_solid_report(&inner, &outer);
            let facts_validation = facts.validate();
            let facts_source_validation = facts.validate_against_source(&outer);
            let report_validation = report.validate();
            let report_source_validation = report.validate_against_sources(&inner, &outer);
            (
                facts,
                facts_validation,
                facts_source_validation,
                classify_mesh_vertices_against_convex_solid(&inner, &outer),
                report,
                report_validation,
                report_source_validation,
            )
        })
    });
    c.bench_function("exact_convex_solid_source_replay_validation", |b| {
        b.iter(|| {
            classify_mesh_vertices_against_convex_solid_report(&inner, &outer)
                .validate_against_sources(&inner, &outer)
        })
    });
}

fn exact_boolean_coplanar_surface_containment(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let inner = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 2, 1, 0, 1, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_surface_containment", |b| {
            b.iter(|| {
                (
                    certify_single_triangle_coplanar_containment(&outer, &inner),
                    certify_single_triangle_coplanar_containment_report(&outer, &inner),
                    hypermesh::exact::boolean_exact(
                        &outer,
                        &inner,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
        c.bench_function("exact_coplanar_containment_status_validation", |b| {
            b.iter(|| {
                let mut report =
                    certify_single_triangle_coplanar_containment_report(&inner, &outer);
                report.status =
                    hypermesh::exact::CoplanarSurfaceContainmentStatus::DisjointOrUnknown;
                assert_eq!(
                    report.validate().unwrap_err(),
                    hypermesh::exact::CoplanarSurfaceContainmentReportError::StatusRelationMismatch
                );
            })
        });
        c.bench_function("exact_coplanar_containment_source_replay", |b| {
            b.iter(|| {
                let report = certify_single_triangle_coplanar_containment_report(&inner, &outer);
                report.validate_against_sources(&inner, &outer)
            })
        });

        let holed_outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let holed_inner = ExactMesh::from_i64_triangles_with_policy(
            &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let annulus = arrange_coplanar_convex_surface_holed_difference(&holed_outer, &holed_inner)
            .unwrap()
            .mesh;
        let cover = ExactMesh::from_i64_triangles_with_policy(
            &[-1, -1, 0, 12, -1, 0, 11, 11, 0, -1, 12, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        c.bench_function("exact_coplanar_mesh_containment_holed_area_replay", |b| {
            b.iter(|| {
                (
                    certify_coplanar_surface_mesh_containment(&annulus, &cover),
                    hypermesh::exact::boolean_exact(
                        &annulus,
                        &cover,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_open_surface_disjoint(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 4, 2, 0, 2, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_open_surface_disjoint", |b| {
            b.iter(|| {
                let report = certify_open_surface_disjoint_report(&left, &right).unwrap();
                let source_validation = report.validate_against_sources(&left, &right);
                (
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    report,
                    source_validation,
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_policy_report_refinement_blocker_validation(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let freshness_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let freshness_right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        c.bench_function("exact_policy_report_refinement_blocker_validation", |b| {
            b.iter(|| {
                let open = hypermesh::exact::ExactOpenSurfaceDisjointReport {
                    status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphUnknowns,
                    left_open_surface: true,
                    right_open_surface: true,
                    graph_had_unknowns: true,
                    retained_face_pairs: 1,
                    retained_events: 1,
                    blocker: hypermesh::exact::ExactBooleanBlocker {
                        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
                        candidate_pairs: 0,
                        coplanar_overlapping_pairs: 0,
                        coplanar_touching_pairs: 0,
                        unknown_pairs: 1,
                        construction_failed_events: 0,
                    },
                };
                let mut boundary = hypermesh::exact::ExactBoundaryTouchingReport {
                    status: hypermesh::exact::ExactBoundaryTouchingStatus::GraphUnknowns,
                    graph_had_unknowns: true,
                    retained_face_pairs: 1,
                    retained_events: 1,
                    blocker: open.blocker.clone(),
                };
                let planar = hypermesh::exact::ExactPlanarArrangementReport {
                    operation: hypermesh::exact::ExactBooleanOperation::Union,
                    status: hypermesh::exact::ExactPlanarArrangementStatus::GraphUnknowns,
                    graph_had_unknowns: true,
                    retained_face_pairs: 1,
                    retained_events: 1,
                    blocker: open.blocker.clone(),
                    arrangement_readiness: None,
                };
                let winding = hypermesh::exact::ExactWindingReadinessReport {
                    operation: hypermesh::exact::ExactBooleanOperation::Union,
                    status: hypermesh::exact::ExactWindingReadinessStatus::GraphUnknowns,
                    graph_had_unknowns: true,
                    retained_face_pairs: 1,
                    retained_events: 1,
                    region_count: 0,
                    region_classifications: Vec::new(),
                    blocker: open.blocker.clone(),
                    arrangement_readiness: None,
                    coplanar_volumetric_evidence: None,
                };
                let valid = (
                    open.validate(),
                    boundary.validate(),
                    planar.validate(),
                    planar.freshness_against_sources(&freshness_left, &freshness_right),
                    winding.validate(),
                    winding.freshness_against_sources(&freshness_left, &freshness_right),
                );
                boundary.blocker.kind =
                    hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy;
                let invalid_kind = boundary.validate().unwrap_err();
                let mut stale_resolved = hypermesh::exact::ExactOpenSurfaceDisjointReport {
                    status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::Certified,
                    left_open_surface: true,
                    right_open_surface: true,
                    graph_had_unknowns: false,
                    retained_face_pairs: 0,
                    retained_events: 0,
                    blocker: hypermesh::exact::ExactBooleanBlocker {
                        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
                        candidate_pairs: 0,
                        coplanar_overlapping_pairs: 0,
                        coplanar_touching_pairs: 0,
                        unknown_pairs: 0,
                        construction_failed_events: 1,
                    },
                };
                let invalid_refinement = stale_resolved.validate().unwrap_err();
                stale_resolved.blocker.construction_failed_events = 0;
                let valid_resolved = stale_resolved.validate();
                (valid, invalid_kind, invalid_refinement, valid_resolved)
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_surface_intersection(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 5, 1, 0, 1, 5, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let split_left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0, 10, 0, 0, 14, 0, 0, 10, 4, 0],
            &[0, 1, 2, 3, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let bridge_right = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 13, 1, 0, 1, 3, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
                8, 0, 0, 12, 0, 0, 12, 4, 0, 8, 4, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let component_right = ExactMesh::from_i64_triangles_with_policy(
            &[2, 1, 0, 10, 1, 0, 10, 3, 0, 2, 3, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_intersection_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, 1, 2, 0, 0, 2, 0, 0, 1, 0,
            ],
            &[0, 1, 2, 0, 2, 3, 0, 3, 6, 6, 3, 4, 6, 4, 5],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_intersection_right = ExactMesh::from_i64_triangles_with_policy(
            &[-1, -1, 0, 6, -1, 0, -1, 6, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_multi_intersection_left = ExactMesh::from_i64_triangles_with_policy(
            &[
                0, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, 1, 2, 0, 0, 2, 0, 0, 1, 0, //
                5, 0, 0, 7, 0, 0, 7, 1, 0, 6, 1, 0, 6, 2, 0, 5, 2, 0, 5, 1, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, 0, 3, 6, 6, 3, 4, 6, 4, 5, //
                7, 8, 9, 7, 9, 10, 7, 10, 13, 13, 10, 11, 13, 11, 12,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let nonconvex_multi_intersection_right = ExactMesh::from_i64_triangles_with_policy(
            &[-1, -1, 0, 12, -1, 0, -1, 6, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let holed_intersection_outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let holed_intersection_hole = ExactMesh::from_i64_triangles_with_policy(
            &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let holed_intersection_annulus = arrange_coplanar_convex_surface_holed_difference(
            &holed_intersection_outer,
            &holed_intersection_hole,
        )
        .unwrap()
        .mesh;
        let holed_intersection_clipper = ExactMesh::from_i64_triangles_with_policy(
            &[2, 1, 0, 9, 2, 0, 8, 9, 0, 1, 8, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 10, 1, 0, 9, 10, 0, -1, 9, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_left_hole = ExactMesh::from_i64_triangles_with_policy(
            &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_right_hole = ExactMesh::from_i64_triangles_with_policy(
            &[6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0],
            &[0, 1, 2, 0, 2, 3],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let same_outer_left_annulus = arrange_coplanar_convex_surface_holed_difference(
            &same_outer_outer,
            &same_outer_left_hole,
        )
        .unwrap()
        .mesh;
        let same_outer_right_annulus = arrange_coplanar_convex_surface_holed_difference(
            &same_outer_outer,
            &same_outer_right_hole,
        )
        .unwrap()
        .mesh;

        c.bench_function("exact_boolean_coplanar_surface_intersection", |b| {
            b.iter(|| {
                let intersection = intersect_single_triangle_coplanar_surfaces(&left, &right);
                (
                    intersection
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &right)),
                    intersection.as_ref().map(|output| output.validate()),
                    intersection,
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    {
                        let report = certify_planar_arrangement_report(
                            &left,
                            &right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap();
                        let freshness = report.freshness_against_sources(&left, &right);
                        (report, freshness)
                    },
                )
            })
        });
        c.bench_function("exact_simple_loop_orientation_validation", |b| {
            b.iter(|| {
                let mut union = arrange_single_triangle_coplanar_union(&left, &right)
                    .expect("fixture should produce a simple-loop arrangement");
                let valid = union.validate();
                union.polygon.reverse();
                let invalid = union.validate().unwrap_err();
                (valid, invalid)
            })
        });
        c.bench_function("exact_planar_readiness_count_validation", |b| {
            b.iter(|| {
                let report = certify_planar_arrangement_report(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                )
                .unwrap();
                let valid = report.validate();
                let mut invalid_report = report;
                if let Some(readiness) = invalid_report.arrangement_readiness.as_mut() {
                    readiness.graph_count += 1;
                    readiness.touching_graphs += 1;
                }
                let invalid = invalid_report.validate().unwrap_err();
                let freshness = invalid_report.freshness_against_sources(&left, &right);
                (valid, invalid, freshness)
            })
        });
        c.bench_function("exact_multi_component_intersection_validation", |b| {
            b.iter(|| {
                let multi = hypermesh::exact::arrange_coplanar_convex_surface_multi_intersection(
                    &split_left,
                    &bridge_right,
                )
                .unwrap();
                let report = certify_planar_arrangement_report(
                    &split_left,
                    &bridge_right,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                )
                .unwrap();
                let winding = certify_winding_readiness_report(
                    &split_left,
                    &bridge_right,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                )
                .unwrap();
                (
                    multi.validate_intersection_against_sources(&split_left, &bridge_right),
                    multi.validate(),
                    report.validate_against_sources(&split_left, &bridge_right),
                    report.validate(),
                    winding.validate_against_sources(&split_left, &bridge_right),
                    winding.validate(),
                    hypermesh::exact::arrange_coplanar_convex_surface_multi_intersection(
                        &component_left,
                        &component_right,
                    )
                    .map(|output| {
                        output.validate_intersection_against_sources(
                            &component_left,
                            &component_right,
                        )
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &component_left,
                        &component_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    arrange_coplanar_surface_component_intersection(
                        &nonconvex_intersection_left,
                        &nonconvex_intersection_right,
                    )
                    .map(|output| {
                        output.validate_intersection_against_sources(
                            &nonconvex_intersection_left,
                            &nonconvex_intersection_right,
                        )
                    }),
                    arrange_coplanar_surface_multi_component_intersection(
                        &nonconvex_intersection_left,
                        &nonconvex_intersection_right,
                    )
                    .is_none(),
                    arrange_coplanar_surface_component_intersection(
                        &nonconvex_multi_intersection_left,
                        &nonconvex_multi_intersection_right,
                    )
                    .is_none(),
                    arrange_coplanar_surface_multi_component_intersection(
                        &nonconvex_multi_intersection_left,
                        &nonconvex_multi_intersection_right,
                    )
                    .map(|output| {
                        output.validate_intersection_against_sources(
                            &nonconvex_multi_intersection_left,
                            &nonconvex_multi_intersection_right,
                        )
                    }),
                    arrange_coplanar_surface_component_holed_intersection(
                        &holed_intersection_annulus,
                        &holed_intersection_clipper,
                    )
                    .map(|output| {
                        output.validate_intersection_against_sources(
                            &holed_intersection_annulus,
                            &holed_intersection_clipper,
                        )
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &holed_intersection_annulus,
                        &holed_intersection_clipper,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    arrange_coplanar_surface_component_holed_intersection(
                        &same_outer_left_annulus,
                        &same_outer_right_annulus,
                    )
                    .map(|output| {
                        output.validate_intersection_against_sources(
                            &same_outer_left_annulus,
                            &same_outer_right_annulus,
                        )
                    }),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_multi_intersection_left,
                        &nonconvex_multi_intersection_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nonconvex_intersection_left,
                        &nonconvex_intersection_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_surface_convex_union(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[0, 1, 0, 1, 0, 0, 1, 1, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_surface_convex_union", |b| {
            b.iter(|| {
                let union = union_single_triangle_coplanar_surfaces(&left, &right);
                (
                    union
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &right)),
                    union.as_ref().map(|output| output.validate()),
                    union,
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_surface_corner_difference(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let removed_right = ExactMesh::from_i64_triangles_with_policy(
            &[-1, -1, 0, 2, -1, 0, -1, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let remaining_right = ExactMesh::from_i64_triangles_with_policy(
            &[-3, 1, 0, 8, -1, 0, -3, 6, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function(
            "exact_boolean_coplanar_surface_corner_removed_difference",
            |b| {
                b.iter(|| {
                    let difference =
                        difference_single_triangle_coplanar_surfaces(&left, &removed_right);
                    (
                        difference
                            .as_ref()
                            .map(|output| output.validate_against_sources(&left, &removed_right)),
                        difference.as_ref().map(|output| output.validate()),
                        difference,
                        hypermesh::exact::boolean_exact(
                            &left,
                            &removed_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_coplanar_surface_corner_remaining_difference",
            |b| {
                b.iter(|| {
                    let difference =
                        difference_single_triangle_coplanar_surfaces(&left, &remaining_right);
                    (
                        difference
                            .as_ref()
                            .map(|output| output.validate_against_sources(&left, &remaining_right)),
                        difference.as_ref().map(|output| output.validate()),
                        difference,
                        hypermesh::exact::boolean_exact(
                            &left,
                            &remaining_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::ALLOW_BOUNDARY,
                        )
                        .unwrap(),
                    )
                })
            },
        );
        c.bench_function("exact_boolean_coplanar_surface_corner_difference", |b| {
            b.iter(|| {
                let difference =
                    difference_single_triangle_coplanar_surfaces(&left, &removed_right);
                (
                    difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &removed_right)),
                    difference.as_ref().map(|output| output.validate()),
                    difference,
                    hypermesh::exact::boolean_exact(
                        &left,
                        &removed_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_surface_arrangement_union(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles_with_policy(
            &[1, -1, 0, 5, 3, 0, 1, 3, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_surface_arrangement_union", |b| {
            b.iter(|| {
                let union = arrange_single_triangle_coplanar_union(&left, &right);
                (
                    union.as_ref().map(|output| {
                        output.validate_against_sources(
                            &left,
                            &right,
                            CoplanarArrangementOperation::Union,
                        )
                    }),
                    union.as_ref().map(|output| output.validate()),
                    union,
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_coplanar_surface_holed_difference(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let outer = ExactMesh::from_i64_triangles_with_policy(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        let inner = ExactMesh::from_i64_triangles_with_policy(
            &[1, 1, 0, 2, 1, 0, 1, 2, 0],
            &[0, 1, 2],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();

        c.bench_function("exact_boolean_coplanar_surface_holed_difference", |b| {
            b.iter(|| {
                let difference = arrange_single_triangle_coplanar_holed_difference(&outer, &inner);
                (
                    difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&outer, &inner)),
                    difference.as_ref().map(|output| output.validate()),
                    difference,
                    hypermesh::exact::boolean_exact(
                        &outer,
                        &inner,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_convex_containment(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let outer = ExactMesh::from_i64_triangles(
            &[
                0, 0, 0, //
                10, 0, 0, //
                0, 10, 0, //
                0, 0, 10,
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let inner = ExactMesh::from_i64_triangles(
            &[
                1, 1, 1, //
                2, 1, 1, //
                1, 2, 1, //
                1, 1, 2,
            ],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolean_convex_containment", |b| {
            b.iter(|| {
                hypermesh::exact::boolean_exact(
                    &outer,
                    &inner,
                    hypermesh::exact::ExactBooleanOperation::Difference,
                    ValidationPolicy::CLOSED,
                )
                .unwrap()
            })
        });
        c.bench_function("exact_boolean_result_operation_replay", |b| {
            b.iter(|| {
                let result = hypermesh::exact::boolean_exact(
                    &outer,
                    &inner,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::CLOSED,
                )
                .unwrap();
                result.validate_operation_against_sources(
                    &outer,
                    &inner,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::CLOSED,
                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_convex_intersection(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles(
            &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolean_convex_intersection", |b| {
            b.iter(|| {
                let intersection = intersect_closed_convex_solids(&left, &right);
                (
                    intersection.as_ref().map(|output| output.validate()),
                    intersection
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &right)),
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    ),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    ),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_volumetric_winding_materialization(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles(
            &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let adjacent_left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let adjacent_right = tetrahedron_i64([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, -4]);
        let adjacent_fan_right =
            base_fan_tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [1, 1, 0], [0, 0, -4]);
        let adjacent_dual_fan_left =
            upper_base_fan_tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [1, 1, 0], [0, 0, 4]);
        let adjacent_dual_fan_right =
            base_fan_tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [2, 1, 0], [0, 0, -4]);
        let adjacent_quad_left =
            upward_square_pyramid_i64([0, 0, 0], [4, 0, 0], [4, 4, 0], [0, 4, 0], [2, 2, 4]);
        let adjacent_quad_right = downward_square_pyramid_opposite_diagonal_i64(
            [0, 0, 0],
            [4, 0, 0],
            [4, 4, 0],
            [0, 4, 0],
            [2, 2, -4],
        );
        let adjacent_quad_fan_right = downward_square_pyramid_quad_fan_i64(
            [0, 0, 0],
            [4, 0, 0],
            [4, 4, 0],
            [0, 4, 0],
            [2, 2, 0],
            [2, 2, -4],
        );
        let adjacent_two_branch_left =
            upward_square_pyramid_i64([0, 0, 0], [6, 0, 0], [6, 6, 0], [0, 6, 0], [3, 3, 5]);
        let adjacent_two_branch_right = downward_square_pyramid_two_branch_i64(
            [0, 0, 0],
            [6, 0, 0],
            [6, 6, 0],
            [0, 6, 0],
            [2, 3, 0],
            [4, 3, 0],
            [3, 3, -5],
        );
        let (adjacent_l_prism_left, adjacent_l_prism_right) = l_prism_pair_i64();
        let adjacent_pentagon_left = upward_pentagonal_pyramid_i64(
            [0, 0, 0],
            [4, 0, 0],
            [5, 3, 0],
            [2, 5, 0],
            [-1, 3, 0],
            [2, 2, 4],
        );
        let adjacent_pentagon_fan_right = downward_pentagonal_pyramid_fan_i64(
            [0, 0, 0],
            [4, 0, 0],
            [5, 3, 0],
            [2, 5, 0],
            [-1, 3, 0],
            [2, 2, 0],
            [2, 2, -4],
        );
        let adjacent_hexagon_boundary = [
            [0, 0, 0],
            [4, 0, 0],
            [6, 3, 0],
            [4, 6, 0],
            [0, 6, 0],
            [-2, 3, 0],
        ];
        let adjacent_hexagon_left =
            upward_hexagonal_pyramid_i64(adjacent_hexagon_boundary, [2, 3, 5]);
        let adjacent_hexagon_fan_right =
            downward_hexagonal_pyramid_fan_i64(adjacent_hexagon_boundary, [2, 3, 0], [2, 3, -5]);
        let adjacent_heptagon_boundary = [
            [0, 0, 0],
            [4, 0, 0],
            [7, 3, 0],
            [5, 6, 0],
            [2, 8, 0],
            [-1, 6, 0],
            [-3, 3, 0],
        ];
        let adjacent_heptagon_left =
            upward_heptagonal_pyramid_i64(adjacent_heptagon_boundary, [2, 4, 6]);
        let adjacent_heptagon_fan_right =
            downward_heptagonal_pyramid_fan_i64(adjacent_heptagon_boundary, [2, 4, 0], [2, 4, -6]);
        let adjacent_octagon_boundary = [
            [0, 0, 0],
            [4, 0, 0],
            [7, 2, 0],
            [8, 5, 0],
            [5, 8, 0],
            [1, 9, 0],
            [-2, 6, 0],
            [-3, 3, 0],
        ];
        let adjacent_octagon_left =
            upward_octagonal_pyramid_i64(adjacent_octagon_boundary, [2, 4, 7]);
        let adjacent_octagon_fan_right =
            downward_octagonal_pyramid_fan_i64(adjacent_octagon_boundary, [2, 4, 0], [2, 4, -7]);
        let adjacent_nonagon_boundary = [
            [0, 0, 0],
            [4, 0, 0],
            [7, 2, 0],
            [9, 4, 0],
            [8, 7, 0],
            [5, 9, 0],
            [2, 10, 0],
            [-1, 8, 0],
            [-3, 4, 0],
        ];
        let adjacent_nonagon_left =
            upward_nonagonal_pyramid_i64(adjacent_nonagon_boundary, [2, 4, 8]);
        let adjacent_nonagon_fan_right =
            downward_nonagonal_pyramid_fan_i64(adjacent_nonagon_boundary, [2, 4, 0], [2, 4, -8]);
        let adjacent_decagon_boundary = [
            [0, 0, 0],
            [4, 0, 0],
            [8, 2, 0],
            [10, 5, 0],
            [9, 8, 0],
            [6, 10, 0],
            [2, 11, 0],
            [-1, 9, 0],
            [-3, 6, 0],
            [-2, 2, 0],
        ];
        let adjacent_decagon_left =
            upward_polygonal_pyramid_i64(&adjacent_decagon_boundary, [3, 5, 9]);
        let adjacent_decagon_fan_right =
            downward_polygonal_pyramid_fan_i64(&adjacent_decagon_boundary, [3, 5, 0], [3, 5, -9]);
        let contained_adjacent_left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
        let contained_adjacent_right = tetrahedron_i64([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -3]);
        let contained_multi_left = combine_exact_meshes(
            &[
                tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]),
                tetrahedron_i64([20, 0, 0], [28, 0, 0], [20, 8, 0], [20, 0, 8]),
            ],
            "bench contained-face adjacent two-container fixture",
        );
        let contained_multi_right = combine_exact_meshes(
            &[
                tetrahedron_i64([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -3]),
                tetrahedron_i64([21, 1, 0], [21, 2, 0], [22, 1, 0], [21, 1, -3]),
            ],
            "bench contained-face adjacent two-cap fixture",
        );
        let contained_same_face_left = tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
        let contained_same_face_right = combine_exact_meshes(
            &[
                tetrahedron_i64([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -3]),
                tetrahedron_i64([2, 4, 0], [2, 5, 0], [3, 4, 0], [2, 4, -3]),
            ],
            "bench contained-face adjacent same-face two-hole fixture",
        );
        let contained_component_hole_left =
            tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
        let contained_component_hole_right =
            downward_square_pyramid_i64([1, 1, 0], [3, 1, 0], [3, 3, 0], [1, 3, 0], [2, 2, -3]);
        let contained_multi_face_left =
            upward_square_pyramid_i64([0, 0, 0], [8, 0, 0], [8, 8, 0], [0, 8, 0], [4, 4, 5]);
        let contained_multi_face_right =
            downward_square_pyramid_i64([3, 2, 0], [6, 2, 0], [6, 5, 0], [3, 5, 0], [4, 3, -3]);
        let contained_independent_multi_face_left = combine_exact_meshes(
            &[
                upward_square_pyramid_i64([0, 0, 0], [8, 0, 0], [8, 8, 0], [0, 8, 0], [4, 4, 5]),
                upward_square_pyramid_i64(
                    [20, 0, 0],
                    [28, 0, 0],
                    [28, 8, 0],
                    [20, 8, 0],
                    [24, 4, 5],
                ),
            ],
            "bench contained-face adjacent independent multi-face containers",
        );
        let contained_independent_multi_face_right = combine_exact_meshes(
            &[
                downward_square_pyramid_i64([3, 2, 0], [6, 2, 0], [6, 5, 0], [3, 5, 0], [4, 3, -3]),
                downward_square_pyramid_i64(
                    [23, 2, 0],
                    [26, 2, 0],
                    [26, 5, 0],
                    [23, 5, 0],
                    [24, 3, -3],
                ),
            ],
            "bench contained-face adjacent independent multi-face caps",
        );

        let graph = build_intersection_graph(&left, &right).unwrap();

        c.bench_function("exact_boolean_volumetric_winding_materialization", |b| {
            b.iter(|| {
                let cells =
                    hypermesh::exact::triangulate_all_face_cells_with_cdt(&graph, &left, &right)
                        .unwrap();
                (
                    cells,
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::certify_winding_readiness_report(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &left,
                                &right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &left,
                                &right,
                                hypermesh::exact::ExactBooleanOperation::Difference,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                )
            })
        });

        c.bench_function("exact_boolean_full_face_adjacent_union", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::materialize_full_face_adjacent_union(
                        &adjacent_left,
                        &adjacent_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(&adjacent_left, &adjacent_right)
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &adjacent_left,
                        &adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &adjacent_left,
                        &adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &adjacent_left,
                                &adjacent_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &adjacent_left,
                        &adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &adjacent_left,
                        &adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &adjacent_left,
                                &adjacent_right,
                                hypermesh::exact::ExactBooleanOperation::Intersection,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &adjacent_left,
                        &adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &adjacent_left,
                        &adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &adjacent_left,
                                &adjacent_right,
                                hypermesh::exact::ExactBooleanOperation::Difference,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                )
            })
        });

        c.bench_function("exact_boolean_full_face_adjacent_fan_patch_union", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::materialize_full_face_adjacent_union(
                        &adjacent_left,
                        &adjacent_fan_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(&adjacent_left, &adjacent_fan_right)
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &adjacent_left,
                        &adjacent_fan_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &adjacent_left,
                        &adjacent_fan_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &adjacent_left,
                                &adjacent_fan_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &adjacent_left,
                        &adjacent_fan_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &adjacent_left,
                        &adjacent_fan_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &adjacent_left,
                                &adjacent_fan_right,
                                hypermesh::exact::ExactBooleanOperation::Intersection,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &adjacent_left,
                        &adjacent_fan_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &adjacent_left,
                        &adjacent_fan_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &adjacent_left,
                                &adjacent_fan_right,
                                hypermesh::exact::ExactBooleanOperation::Difference,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                )
            })
        });

        c.bench_function(
            "exact_boolean_full_face_adjacent_dual_fan_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_dual_fan_left,
                            &adjacent_dual_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_dual_fan_left,
                                    &adjacent_dual_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &adjacent_dual_fan_left,
                            &adjacent_dual_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .map(|report| report.validate()),
                        hypermesh::exact::boolean_exact(
                            &adjacent_dual_fan_left,
                            &adjacent_dual_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_dual_fan_left,
                                    &adjacent_dual_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_dual_fan_left,
                            &adjacent_dual_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_dual_fan_left,
                                    &adjacent_dual_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Intersection,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_cross_diagonal_quad_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_quad_left,
                            &adjacent_quad_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(&adjacent_quad_left, &adjacent_quad_right)
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_quad_left,
                            &adjacent_quad_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_quad_left,
                                    &adjacent_quad_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_quad_fan_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_quad_left,
                            &adjacent_quad_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_quad_left,
                                    &adjacent_quad_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_quad_left,
                            &adjacent_quad_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_quad_left,
                                    &adjacent_quad_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_two_branch_square_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_two_branch_left,
                            &adjacent_two_branch_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_two_branch_left,
                                    &adjacent_two_branch_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_two_branch_left,
                            &adjacent_two_branch_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_two_branch_left,
                                    &adjacent_two_branch_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_nonconvex_l_prism_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_l_prism_left,
                            &adjacent_l_prism_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_l_prism_left,
                                    &adjacent_l_prism_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_l_prism_left,
                            &adjacent_l_prism_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_l_prism_left,
                                    &adjacent_l_prism_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_pentagon_fan_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_pentagon_left,
                            &adjacent_pentagon_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_pentagon_left,
                                    &adjacent_pentagon_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_pentagon_left,
                            &adjacent_pentagon_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_pentagon_left,
                                    &adjacent_pentagon_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_hexagon_fan_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_hexagon_left,
                            &adjacent_hexagon_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_hexagon_left,
                                    &adjacent_hexagon_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_hexagon_left,
                            &adjacent_hexagon_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_hexagon_left,
                                    &adjacent_hexagon_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_heptagon_fan_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_heptagon_left,
                            &adjacent_heptagon_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_heptagon_left,
                                    &adjacent_heptagon_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_heptagon_left,
                            &adjacent_heptagon_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_heptagon_left,
                                    &adjacent_heptagon_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_octagon_fan_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_octagon_left,
                            &adjacent_octagon_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_octagon_left,
                                    &adjacent_octagon_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_octagon_left,
                            &adjacent_octagon_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_octagon_left,
                                    &adjacent_octagon_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_nonagon_fan_patch_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_nonagon_left,
                            &adjacent_nonagon_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_nonagon_left,
                                    &adjacent_nonagon_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_nonagon_left,
                            &adjacent_nonagon_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_nonagon_left,
                                    &adjacent_nonagon_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );
        c.bench_function(
            "exact_boolean_full_face_adjacent_decagon_component_disk_union",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_full_face_adjacent_union(
                            &adjacent_decagon_left,
                            &adjacent_decagon_fan_right,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|union| {
                            union
                                .validate_against_sources(
                                    &adjacent_decagon_left,
                                    &adjacent_decagon_fan_right,
                                )
                                .unwrap();
                            union.mesh.triangles().len()
                        }),
                        hypermesh::exact::boolean_exact(
                            &adjacent_decagon_left,
                            &adjacent_decagon_fan_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &adjacent_decagon_left,
                                    &adjacent_decagon_fan_right,
                                    hypermesh::exact::ExactBooleanOperation::Union,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        }),
                    )
                })
            },
        );

        c.bench_function("exact_boolean_contained_face_adjacent_union", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::materialize_contained_face_adjacent_union(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(
                                &contained_adjacent_left,
                                &contained_adjacent_right,
                            )
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::certify_boundary_touching_report(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::preflight_boolean_exact(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_adjacent_left,
                                &contained_adjacent_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_adjacent_left,
                                &contained_adjacent_right,
                                hypermesh::exact::ExactBooleanOperation::Intersection,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::preflight_boolean_exact(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .map(|report| report.validate()),
                    hypermesh::exact::boolean_exact(
                        &contained_adjacent_left,
                        &contained_adjacent_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_adjacent_left,
                                &contained_adjacent_right,
                                hypermesh::exact::ExactBooleanOperation::Difference,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::materialize_contained_face_adjacent_union(
                        &contained_multi_left,
                        &contained_multi_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(&contained_multi_left, &contained_multi_right)
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::boolean_exact(
                        &contained_multi_left,
                        &contained_multi_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_multi_left,
                                &contained_multi_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::materialize_contained_face_adjacent_union(
                        &contained_same_face_left,
                        &contained_same_face_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(
                                &contained_same_face_left,
                                &contained_same_face_right,
                            )
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::boolean_exact(
                        &contained_same_face_left,
                        &contained_same_face_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_same_face_left,
                                &contained_same_face_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::materialize_contained_face_adjacent_union(
                        &contained_component_hole_left,
                        &contained_component_hole_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(
                                &contained_component_hole_left,
                                &contained_component_hole_right,
                            )
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::boolean_exact(
                        &contained_component_hole_left,
                        &contained_component_hole_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_component_hole_left,
                                &contained_component_hole_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::materialize_contained_face_adjacent_union(
                        &contained_multi_face_left,
                        &contained_multi_face_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(
                                &contained_multi_face_left,
                                &contained_multi_face_right,
                            )
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::boolean_exact(
                        &contained_multi_face_left,
                        &contained_multi_face_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_multi_face_left,
                                &contained_multi_face_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                    hypermesh::exact::materialize_contained_face_adjacent_union(
                        &contained_independent_multi_face_left,
                        &contained_independent_multi_face_right,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|union| {
                        union
                            .validate_against_sources(
                                &contained_independent_multi_face_left,
                                &contained_independent_multi_face_right,
                            )
                            .unwrap();
                        union.mesh.triangles().len()
                    }),
                    hypermesh::exact::boolean_exact(
                        &contained_independent_multi_face_left,
                        &contained_independent_multi_face_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .map(|result| {
                        result
                            .validate_operation_against_sources(
                                &contained_independent_multi_face_left,
                                &contained_independent_multi_face_right,
                                hypermesh::exact::ExactBooleanOperation::Union,
                                ValidationPolicy::CLOSED,
                                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                            )
                            .unwrap();
                        result.mesh.triangles().len()
                    }),
                )
            })
        });

        let coplanar_left = ExactMesh::from_i64_triangles(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2,
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap();
        let coplanar_right = top_subdivided_axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let non_rectilinear_coplanar_left =
            tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
        let non_rectilinear_coplanar_right =
            tetrahedron_i64([1, 1, 0], [5, 1, 0], [1, 5, 0], [1, 1, 4]);
        let nonconvex_coplanar_left =
            upward_l_prism_i64([[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]], 5);
        let nonconvex_coplanar_right = tetrahedron_i64([1, 1, 0], [7, 1, 0], [1, 7, 0], [1, 1, 5]);
        let boundary_contained_convex_outer =
            tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
        let boundary_contained_convex_inner =
            tetrahedron_i64([1, 1, 0], [3, 1, 0], [1, 3, 0], [1, 1, 2]);
        let boundary_contained_nonconvex_outer =
            upward_l_prism_i64([[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]], 8);
        let boundary_contained_nonconvex_inner = axis_aligned_box_i64([1, 3, 4], [2, 4, 8]);

        let slab_right = ExactMesh::from_i64_triangles(
            &[
                1, 0, 0, 3, 0, 0, 3, 2, 0, 1, 2, 0, 1, 0, 2, 3, 0, 2, 3, 2, 2, 1, 2, 2,
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap();
        let split_left = axis_aligned_box_i64([0, 0, 0], [4, 2, 2]);
        let split_right = axis_aligned_box_i64([1, 0, 0], [3, 2, 2]);
        let nested_left = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
        let nested_right = axis_aligned_box_i64([1, 1, 1], [3, 3, 3]);
        let contained_outer = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
        let contained_inner = axis_aligned_box_i64([1, 1, 1], [3, 3, 3]);
        let boundary_contained_inner = axis_aligned_box_i64([0, 1, 1], [2, 3, 3]);
        let cell_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let cell_right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let fan_cell_left = top_subdivided_axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let fan_cell_right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
        let orthogonal_complex = hypermesh::exact::boolean_exact(
            &cell_left,
            &cell_right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .mesh;
        let orthogonal_cutter = axis_aligned_box_i64([2, 0, 0], [3, 2, 2]);
        let orthogonal_outer = axis_aligned_box_i64([0, 0, 0], [8, 8, 8]);
        let orthogonal_cavity = axis_aligned_box_i64([2, 2, 2], [6, 6, 6]);
        let orthogonal_hollow = hypermesh::exact::boolean_exact(
            &orthogonal_outer,
            &orthogonal_cavity,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .mesh;
        let orthogonal_floating = axis_aligned_box_i64([3, 3, 3], [5, 5, 5]);
        let face_touch_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
        let face_touch_right = axis_aligned_box_i64([2, 0, 0], [4, 2, 2]);

        c.bench_function(
            "exact_boolean_axis_aligned_box_slab_union_difference",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::preflight_boolean_exact(
                            &coplanar_left,
                            &slab_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &coplanar_left,
                            &slab_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &coplanar_left,
                            &slab_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &coplanar_left,
                            &slab_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                    )
                })
            },
        );

        c.bench_function("exact_boolean_axis_aligned_box_split_difference", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::preflight_boolean_exact(
                        &split_left,
                        &split_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &split_left,
                        &split_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                )
            })
        });

        c.bench_function("exact_boolean_axis_aligned_box_nested_difference", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::preflight_boolean_exact(
                        &nested_left,
                        &nested_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &nested_left,
                        &nested_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                )
            })
        });

        c.bench_function(
            "exact_boolean_axis_aligned_box_containment_union_empty_difference",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::preflight_boolean_exact(
                            &contained_outer,
                            &contained_inner,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &contained_outer,
                            &contained_inner,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &contained_inner,
                            &contained_outer,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &contained_inner,
                            &contained_outer,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &boundary_contained_inner,
                            &contained_outer,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &boundary_contained_inner,
                            &contained_outer,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                    )
                })
            },
        );

        c.bench_function(
            "exact_boolean_axis_aligned_box_face_touch_union_difference",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::preflight_boolean_exact(
                            &face_touch_left,
                            &face_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &face_touch_left,
                            &face_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &face_touch_left,
                            &face_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &face_touch_left,
                            &face_touch_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                    )
                })
            },
        );

        c.bench_function(
            "exact_boolean_axis_aligned_box_cell_union_difference",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::preflight_boolean_exact(
                            &cell_left,
                            &cell_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &cell_left,
                            &cell_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &cell_left,
                            &cell_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &cell_left,
                            &cell_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &cell_left,
                            &cell_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &cell_left,
                            &cell_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                    )
                })
            },
        );

        c.bench_function("exact_boolean_axis_aligned_orthogonal_solid_cells", |b| {
            b.iter(|| {
                (
                    hypermesh::exact::preflight_boolean_exact(
                        &orthogonal_complex,
                        &orthogonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &orthogonal_complex,
                        &orthogonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &orthogonal_complex,
                        &orthogonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &orthogonal_complex,
                        &orthogonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &orthogonal_complex,
                        &orthogonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &orthogonal_complex,
                        &orthogonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &fan_cell_left,
                        &fan_cell_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &fan_cell_left,
                        &fan_cell_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &fan_cell_left,
                        &fan_cell_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &fan_cell_left,
                        &fan_cell_right,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &fan_cell_left,
                        &fan_cell_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &fan_cell_left,
                        &fan_cell_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                    hypermesh::exact::preflight_boolean_exact(
                        &orthogonal_hollow,
                        &orthogonal_floating,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                    )
                    .unwrap(),
                    hypermesh::exact::boolean_exact(
                        &orthogonal_hollow,
                        &orthogonal_floating,
                        hypermesh::exact::ExactBooleanOperation::Intersection,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap(),
                )
            })
        });

        c.bench_function(
            "exact_boolean_coplanar_volumetric_cell_materialization",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::preflight_boolean_exact(
                            &coplanar_left,
                            &coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap(),
                        hypermesh::exact::certify_winding_readiness_report(
                            &coplanar_left,
                            &coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &coplanar_left,
                            &coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &coplanar_left,
                            &coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &coplanar_left,
                            &coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &coplanar_left,
                            &coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &coplanar_left,
                            &coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                    )
                })
            },
        );

        let mut non_rectilinear_wrong_operation = hypermesh::exact::boolean_exact(
            &non_rectilinear_coplanar_left,
            &non_rectilinear_coplanar_right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        non_rectilinear_wrong_operation.kind =
            hypermesh::exact::ExactBooleanResultKind::WindingMaterialized {
                operation: hypermesh::exact::ExactBooleanOperation::Difference,
            };
        let mut non_rectilinear_wrong_orientation = hypermesh::exact::boolean_exact(
            &non_rectilinear_coplanar_left,
            &non_rectilinear_coplanar_right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        if let Some(triangle) = non_rectilinear_wrong_orientation
            .assembly
            .triangles
            .first_mut()
        {
            triangle.orientation = hypermesh::exact::ExactOutputTriangleOrientation::ReverseSource;
        }

        c.bench_function(
            "exact_boolean_non_rectilinear_coplanar_volumetric_cells",
            |b| {
                b.iter(|| {
                    let preflight = hypermesh::exact::preflight_boolean_exact(
                        &non_rectilinear_coplanar_left,
                        &non_rectilinear_coplanar_right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap();
                    let preflight_retains_evidence = preflight
                        .coplanar_volumetric_evidence
                        .as_ref()
                        .is_some_and(|evidence| {
                            evidence.obstacle.requires_coplanar_volumetric_cells()
                        });
                    (
                        preflight,
                        preflight_retains_evidence,
                        hypermesh::exact::boolean_exact(
                            &non_rectilinear_coplanar_left,
                            &non_rectilinear_coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &non_rectilinear_coplanar_left,
                            &non_rectilinear_coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &non_rectilinear_coplanar_left,
                            &non_rectilinear_coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        non_rectilinear_wrong_operation.validate().is_err(),
                        non_rectilinear_wrong_orientation.validate().is_err(),
                    )
                })
            },
        );

        c.bench_function(
            "exact_boolean_nonconvex_coplanar_volumetric_fan_split",
            |b| {
                b.iter(|| {
                    let preflight = hypermesh::exact::preflight_boolean_exact(
                        &nonconvex_coplanar_left,
                        &nonconvex_coplanar_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    )
                    .unwrap();
                    let result = hypermesh::exact::boolean_exact(
                        &nonconvex_coplanar_left,
                        &nonconvex_coplanar_right,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    )
                    .unwrap();
                    result
                        .validate_operation_against_sources(
                            &nonconvex_coplanar_left,
                            &nonconvex_coplanar_right,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                        )
                        .unwrap();
                    (preflight, result.mesh.triangles().len())
                })
            },
        );

        c.bench_function(
            "exact_boolean_convex_boundary_containment_coplanar_cells",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::preflight_boolean_exact(
                            &boundary_contained_convex_outer,
                            &boundary_contained_convex_inner,
                            hypermesh::exact::ExactBooleanOperation::Union,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &boundary_contained_convex_outer,
                            &boundary_contained_convex_inner,
                            hypermesh::exact::ExactBooleanOperation::Union,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &boundary_contained_convex_outer,
                            &boundary_contained_convex_inner,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &boundary_contained_convex_outer,
                            &boundary_contained_convex_inner,
                            hypermesh::exact::ExactBooleanOperation::Intersection,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::preflight_boolean_exact(
                            &boundary_contained_convex_outer,
                            &boundary_contained_convex_inner,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &boundary_contained_convex_outer,
                            &boundary_contained_convex_inner,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &boundary_contained_convex_inner,
                            &boundary_contained_convex_outer,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .unwrap(),
                    )
                })
            },
        );

        c.bench_function(
            "exact_boolean_nonconvex_boundary_containment_difference",
            |b| {
                b.iter(|| {
                    (
                        hypermesh::exact::materialize_contained_boundary_difference(
                            &boundary_contained_nonconvex_outer,
                            &boundary_contained_nonconvex_inner,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|difference| {
                            difference
                                .validate_against_sources(
                                    &boundary_contained_nonconvex_outer,
                                    &boundary_contained_nonconvex_inner,
                                )
                                .unwrap();
                            difference.mesh.triangles().len()
                        }),
                        hypermesh::exact::preflight_boolean_exact(
                            &boundary_contained_nonconvex_outer,
                            &boundary_contained_nonconvex_inner,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                        )
                        .unwrap(),
                        hypermesh::exact::boolean_exact(
                            &boundary_contained_nonconvex_outer,
                            &boundary_contained_nonconvex_inner,
                            hypermesh::exact::ExactBooleanOperation::Difference,
                            ValidationPolicy::CLOSED,
                        )
                        .map(|result| {
                            result
                                .validate_operation_against_sources(
                                    &boundary_contained_nonconvex_outer,
                                    &boundary_contained_nonconvex_inner,
                                    hypermesh::exact::ExactBooleanOperation::Difference,
                                    ValidationPolicy::CLOSED,
                                    hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
                                )
                                .unwrap();
                            result.mesh.triangles().len()
                        })
                        .unwrap(),
                    )
                })
            },
        );
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolean_convex_single_cap_difference(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let cutter = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolean_convex_single_cap_difference", |b| {
            b.iter(|| {
                let difference = subtract_closed_convex_solids_single_cap(&left, &cutter);
                (
                    difference.as_ref().map(|output| output.validate()),
                    difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&left, &cutter)),
                    hypermesh::exact::preflight_boolean_exact(
                        &left,
                        &cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    ),
                    hypermesh::exact::boolean_exact(
                        &left,
                        &cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    ),
                )
            })
        });

        let cube = ExactMesh::from_i64_triangles(
            &[
                0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2,
            ],
            &[
                0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2,
                7, 6, 3, 0, 4, 3, 4, 7,
            ],
        )
        .unwrap();
        let polygonal_cutter = ExactMesh::from_i64_triangles(
            &[-10, -10, -10, 23, -10, -10, -10, 23, -10, -10, -10, 23],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolean_convex_polygonal_cap_difference", |b| {
            b.iter(|| {
                let difference = subtract_closed_convex_solids_single_cap(&cube, &polygonal_cutter);
                (
                    difference.as_ref().map(|output| output.validate()),
                    difference
                        .as_ref()
                        .map(|output| output.validate_against_sources(&cube, &polygonal_cutter)),
                    hypermesh::exact::preflight_boolean_exact(
                        &cube,
                        &polygonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                    ),
                    hypermesh::exact::boolean_exact(
                        &cube,
                        &polygonal_cutter,
                        hypermesh::exact::ExactBooleanOperation::Difference,
                        ValidationPolicy::CLOSED,
                    ),
                )
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolmesh_bounds_disjoint_port(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles(
            &[10, 0, 0, 12, 0, 0, 10, 2, 0, 10, 0, 2],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolmesh_bounds_disjoint_port", |b| {
            b.iter(|| {
                let workspace = hypermesh::exact::exact_boolmesh_workspace(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                );
                workspace.validate_against_sources(&left, &right).unwrap();
                let size_output = workspace.boolean45.as_ref().unwrap();
                let execution = hypermesh::exact::execute_exact_boolmesh_bounds_disjoint(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                    ValidationPolicy::CLOSED,
                )
                .unwrap();
                execution.validate_against_sources(&left, &right).unwrap();
                execution.mesh.triangles().len()
                    + size_output.face_halfedge_offsets.len()
                    + size_output.vertices_from_left
                    + size_output.vertices_from_right
                    + size_output.vertex_allocation.output_vertex_origins.len()
                    + size_output.new_edge_vertices.source_edge_runs.len()
                    + size_output.new_edge_vertices.face_pair_runs.len()
                    + size_output.partial_source_edges.source_edge_runs.len()
                    + size_output.new_face_pair_edges.face_pair_runs.len()
                    + size_output.whole_source_edges.source_edge_runs.len()
                    + size_output.halfedge_assembly.output_halfedges.len()
                    + size_output.halfedge_assembly.emitted_pairs
                    + size_output.face_loop_assembly.loops.len()
                    + size_output.loop_triangulation.triangulations.len()
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn exact_boolmesh_kernel12_port(c: &mut Criterion) {
    #[cfg(feature = "exact-triangulation")]
    {
        let left = ExactMesh::from_i64_triangles(
            &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();
        let right = ExactMesh::from_i64_triangles(
            &[1, 1, -1, 3, 1, 3, 1, 3, 3, 3, 3, 1],
            &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        )
        .unwrap();

        c.bench_function("exact_boolmesh_kernel12_port", |b| {
            b.iter(|| {
                let workspace = hypermesh::exact::exact_boolmesh_workspace(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Intersection,
                );
                workspace.validate_against_sources(&left, &right).unwrap();
                assert_eq!(
                    workspace.blocker.as_ref().map(|blocker| blocker.stage),
                    Some(hypermesh::exact::ExactBoolMeshKernelStage::Kernel03)
                );
                workspace
                    .kernel12_events
                    .iter()
                    .filter(|event| {
                        matches!(
                            event.relation,
                            SegmentPlaneRelation::ProperCrossing
                                | SegmentPlaneRelation::EndpointOnPlane
                        )
                    })
                    .count()
                    + workspace.boolean03.p1q2.len()
                    + workspace.boolean03.p2q1.len()
                    + workspace.pair_up.source_edge_runs.len()
                    + workspace
                        .boolean45
                        .as_ref()
                        .map(|stage| {
                            stage.face_halfedge_offsets.len()
                                + stage.inserted_intersection_vertices
                                + stage.source_face_to_output_face.len()
                                + stage.vertex_allocation.output_vertex_origins.len()
                                + stage.new_edge_vertices.source_edge_runs.len()
                                + stage.new_edge_vertices.face_pair_runs.len()
                                + stage.partial_source_edges.source_edge_runs.len()
                                + stage.new_face_pair_edges.face_pair_runs.len()
                                + stage.whole_source_edges.source_edge_runs.len()
                                + stage.halfedge_assembly.output_halfedges.len()
                                + stage.halfedge_assembly.emitted_pairs
                                + stage.face_loop_assembly.loops.len()
                                + stage.loop_triangulation.triangulations.len()
                        })
                        .unwrap_or(0)
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

fn legacy_boolean_adapter_report(c: &mut Criterion) {
    #[cfg(feature = "legacy-boolean")]
    {
        let left = hypermesh::prelude::Manifold::new(
            &[
                -0.866025, -1.0, 0.5, 0.0, -1.0, -1.0, 0.866025, -1.0, 0.5, 0.0, 1.0, 0.0,
            ],
            &[0, 3, 1, 1, 2, 0, 1, 3, 2, 2, 3, 0],
        )
        .unwrap();
        let right = hypermesh::prelude::Manifold::new(
            &[
                -1.0, -0.866025, 0.5, -1.0, 0.0, -1.0, -1.0, 0.866025, 0.5, 1.0, 0.0, 0.0,
            ],
            &[1, 3, 0, 1, 0, 2, 2, 3, 1, 0, 3, 2],
        )
        .unwrap();

        c.bench_function("legacy_boolean_adapter_report", |b| {
            b.iter(|| {
                let result = hypermesh::prelude::compute_boolean_with_report(
                    &left,
                    &right,
                    hypermesh::prelude::OpType::Subtract,
                )
                .unwrap();
                result.validate_operation_against_inputs(
                    &left,
                    &right,
                    hypermesh::prelude::OpType::Subtract,
                )
            })
        });
    }
    #[cfg(not(feature = "legacy-boolean"))]
    {
        let _ = c;
    }
}

criterion_group!(
    benches,
    exact_tetrahedron_validation,
    exact_mesh_proposal_acceptance,
    exact_face_plane_fact_retention,
    exact_bounds_candidate_generation,
    exact_support_dop_witness_refresh,
    exact_segment_plane_intersection,
    exact_retained_segment_plane_intersection,
    exact_triangle_triangle_classifier,
    exact_retained_face_plane_classifier,
    exact_coplanar_triangle_classifier,
    exact_mesh_face_pair_classifier,
    exact_mesh_face_pair_retained_plane_rejection,
    exact_mesh_face_pair_batch,
    exact_intersection_graph_events,
    exact_coplanar_overlap_graph_handoff,
    exact_planar_arrangement_evidence,
    exact_coplanar_volumetric_cell_evidence,
    exact_graph_vertex_merge,
    exact_split_topology_plan,
    exact_face_split_plan,
    exact_split_plan_validation,
    exact_face_split_geometry_plan,
    exact_face_split_geometry_incidence,
    exact_face_region_plan,
    exact_face_region_earcut,
    exact_face_interior_steiner_provenance,
    exact_volumetric_witness_lattice,
    exact_boolean_selected_regions,
    exact_selected_region_undecided_validation,
    exact_selected_region_preflight,
    exact_boolean_preflight,
    exact_winding_readiness_undecided_validation,
    exact_closed_mesh_winding_parity,
    exact_boolean_winding_shortcuts,
    exact_boolean_boundary_preflight,
    exact_boolean_same_surface,
    exact_boolean_coplanar_convex_surface_equivalence,
    exact_boolean_coplanar_convex_surface_containment,
    exact_boolean_coplanar_convex_surface_arrangement_union,
    exact_boolean_coplanar_convex_surface_multi_union,
    exact_boolean_coplanar_convex_surface_intersection,
    exact_boolean_coplanar_convex_surface_arrangement_difference,
    exact_boolean_coplanar_convex_surface_multi_difference,
    exact_boolean_coplanar_convex_surface_multi_holed_difference,
    exact_boolean_coplanar_orthogonal_surface_cells,
    exact_boolean_coplanar_affine_surface_cells,
    exact_boolean_affine_box_cells,
    exact_convex_solid_classification,
    exact_boolean_coplanar_surface_containment,
    exact_boolean_open_surface_disjoint,
    exact_policy_report_refinement_blocker_validation,
    exact_boolean_coplanar_surface_intersection,
    exact_boolean_coplanar_surface_convex_union,
    exact_boolean_coplanar_surface_arrangement_union,
    exact_boolean_coplanar_surface_corner_difference,
    exact_boolean_coplanar_surface_holed_difference,
    exact_boolean_convex_containment,
    exact_boolean_convex_intersection,
    exact_boolean_volumetric_winding_materialization,
    exact_boolean_convex_single_cap_difference,
    exact_boolmesh_bounds_disjoint_port,
    exact_boolmesh_kernel12_port,
    legacy_boolean_adapter_report
);
criterion_main!(benches);

fn p3(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}
