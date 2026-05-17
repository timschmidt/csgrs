#![cfg(feature = "exact")]

use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::Point3;
use hypermesh::exact::{
    ExactMesh, ValidationPolicy, build_intersection_graph, classify_coplanar_triangles,
    classify_face_regions_against_opposite_planes, classify_mesh_face_pair,
    classify_mesh_face_pairs, classify_triangle_triangle, intersect_segment_with_face_plane,
};
use hyperreal::Real;

fn exact_tetrahedron_validation(c: &mut Criterion) {
    let pos = vec![
        0.0, 0.0, 0.0, //
        1.0, 0.0, 0.0, //
        0.0, 1.0, 0.0, //
        0.0, 0.0, 1.0,
    ];
    let idx = vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];

    c.bench_function("exact_tetrahedron_validation", |b| {
        b.iter(|| ExactMesh::from_f64_triangles(&pos, &idx).unwrap())
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
        b.iter(|| left.bounds().candidate_face_pairs(right.bounds()))
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
        b.iter(|| intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]))
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
        b.iter(|| classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]))
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
        b.iter(|| classify_coplanar_triangles(&points, [0, 1, 2], [3, 4, 5]))
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
        b.iter(|| classify_mesh_face_pair(&left, 0, &right, 0).unwrap())
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
        b.iter(|| classify_mesh_face_pairs(&left, &right).unwrap())
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
        b.iter(|| build_intersection_graph(&left, &right).unwrap())
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
    let topology_plan = graph.split_topology_plan();
    let face_plan = graph.face_split_plan();

    c.bench_function("exact_split_plan_validation", |b| {
        b.iter(|| {
            (
                topology_plan.validate(),
                face_plan.validate_against_topology(&topology_plan),
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
        b.iter(|| geometry.validate_boundary_incidence(&left, &right))
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
            (
                region_plan.graph_vertex_references(),
                region_plan.validate(&left, &right),
                classify_face_regions_against_opposite_planes(&region_plan, &left, &right),
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
                let triangulations = hypermesh::exact::triangulate_face_regions_with_earcut(
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
                hypermesh::exact::boolean_selected_regions(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
                )
                .unwrap()
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
                hypermesh::exact::preflight_boolean_exact(
                    &left,
                    &right,
                    hypermesh::exact::ExactBooleanOperation::Union,
                )
                .unwrap()
            })
        });
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

criterion_group!(
    benches,
    exact_tetrahedron_validation,
    exact_bounds_candidate_generation,
    exact_segment_plane_intersection,
    exact_triangle_triangle_classifier,
    exact_coplanar_triangle_classifier,
    exact_mesh_face_pair_classifier,
    exact_mesh_face_pair_batch,
    exact_intersection_graph_events,
    exact_graph_vertex_merge,
    exact_split_topology_plan,
    exact_face_split_plan,
    exact_split_plan_validation,
    exact_face_split_geometry_plan,
    exact_face_split_geometry_incidence,
    exact_face_region_plan,
    exact_face_region_earcut,
    exact_boolean_selected_regions,
    exact_boolean_preflight
);
criterion_main!(benches);

fn p3(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}
