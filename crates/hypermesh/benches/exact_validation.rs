#![cfg(feature = "exact")]

use criterion::{Criterion, criterion_group, criterion_main};
use hyperlimit::Point3;
use hypermesh::exact::{
    ExactMesh, ExactReportValidationError, FaceRegionPlaneRelation, MeshFacePairClassification,
    PredicateUse, ValidationPolicy, arrange_coplanar_convex_surface_difference,
    arrange_coplanar_convex_surface_intersection, arrange_coplanar_convex_surface_multi_difference,
    arrange_coplanar_convex_surface_union, arrange_single_triangle_coplanar_holed_difference,
    arrange_single_triangle_coplanar_union, build_intersection_graph,
    certify_boundary_touching_report, certify_convex_solid,
    certify_coplanar_convex_surface_containment, certify_coplanar_convex_surface_equivalence,
    certify_coplanar_convex_surface_report, certify_open_surface_disjoint_report,
    certify_planar_arrangement_report, certify_refinement_report, certify_same_surface_report,
    certify_single_triangle_coplanar_containment,
    certify_single_triangle_coplanar_containment_report, certify_winding_readiness_report,
    checked_classify_face_regions_against_opposite_planes, classify_coplanar_triangles,
    classify_mesh_face_pair, classify_mesh_face_pairs,
    classify_mesh_triangle_against_retained_face_plane,
    classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, classify_triangle_triangle,
    difference_single_triangle_coplanar_surfaces, intersect_segment_with_face_plane,
    intersect_segment_with_retained_face_plane, intersect_single_triangle_coplanar_surfaces,
    union_single_triangle_coplanar_surfaces,
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
        b.iter(|| {
            let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
            let validation = mesh.facts().validate();
            let retained_state = mesh.validate_retained_state();
            (mesh, validation, retained_state)
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
            let mesh = ExactMesh::from_i64_triangles(&pos, &idx).unwrap();
            mesh.facts().validate().unwrap();
            mesh.validate_retained_state().unwrap();
            mesh.facts()
                .faces
                .iter()
                .map(|face| (face.plane.normal.clone(), face.plane.offset.clone()))
                .collect::<Vec<_>>()
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
            let left_validation = left
                .bounds()
                .validate(left.vertices().len(), left.triangles().len());
            let right_validation = right
                .bounds()
                .validate(right.vertices().len(), right.triangles().len());
            (
                left.bounds().candidate_face_pairs(right.bounds()),
                left_validation,
                right_validation,
            )
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
            (event, validation)
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
            (classification, validation)
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
            (classification, validation)
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
            (classification, validation)
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
            (classification, validation)
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
            (classifications, validations)
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
            (graph, validation)
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
            let validations = overlap_graphs
                .iter()
                .map(|overlap| overlap.validate())
                .collect::<Vec<_>>();
            (
                graph.validate(),
                graph.validate_against_meshes(&left, &right),
                overlap_graphs,
                validations,
                split_plan.validate_against_meshes(&left, &right),
                readiness.validate(),
                split_plan,
                readiness,
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
                graph.graph_vertex_plan().validate(),
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
            let classifications =
                checked_classify_face_regions_against_opposite_planes(&region_plan, &left, &right)
                    .unwrap();
            let classification_validations = classifications
                .iter()
                .map(|classification| classification.validate())
                .collect::<Vec<_>>();
            (
                region_plan.graph_vertex_references(),
                {
                    let report = region_plan.validate(&left, &right);
                    let report_validation = report.validate();
                    (report, report_validation)
                },
                classifications,
                classification_validations,
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
                (result, validation)
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
                    preflight.validate(),
                    preflight,
                    refinement.validate(),
                    refinement,
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
                (
                    certify_same_surface_report(&mesh, &reversed),
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
                    certify_coplanar_convex_surface_report(&left, &right).validate(),
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
                    certify_coplanar_convex_surface_report(&outer, &inner).validate(),
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

        c.bench_function(
            "exact_boolean_coplanar_convex_surface_multi_difference",
            |b| {
                b.iter(|| {
                    let arrangement =
                        arrange_coplanar_convex_surface_multi_difference(&left, &right);
                    (
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
            let report_validation = report.validate();
            (
                facts,
                facts_validation,
                classify_mesh_vertices_against_convex_solid(&inner, &outer),
                report,
                report_validation,
            )
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
                (
                    hypermesh::exact::boolean_exact(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                        ValidationPolicy::ALLOW_BOUNDARY,
                    )
                    .unwrap(),
                    certify_open_surface_disjoint_report(&left, &right).unwrap(),
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
                let valid = (open.validate(), boundary.validate(), planar.validate());
                boundary.blocker.kind =
                    hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy;
                let invalid = boundary.validate().unwrap_err();
                (valid, invalid)
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

        c.bench_function("exact_boolean_coplanar_surface_intersection", |b| {
            b.iter(|| {
                let intersection = intersect_single_triangle_coplanar_surfaces(&left, &right);
                (
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
                    certify_planar_arrangement_report(
                        &left,
                        &right,
                        hypermesh::exact::ExactBooleanOperation::Union,
                    )
                    .unwrap(),
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
                (valid, invalid)
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
    }
    #[cfg(not(feature = "exact-triangulation"))]
    {
        let _ = c;
    }
}

criterion_group!(
    benches,
    exact_tetrahedron_validation,
    exact_face_plane_fact_retention,
    exact_bounds_candidate_generation,
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
    exact_graph_vertex_merge,
    exact_split_topology_plan,
    exact_face_split_plan,
    exact_split_plan_validation,
    exact_face_split_geometry_plan,
    exact_face_split_geometry_incidence,
    exact_face_region_plan,
    exact_face_region_earcut,
    exact_boolean_selected_regions,
    exact_selected_region_undecided_validation,
    exact_selected_region_preflight,
    exact_boolean_preflight,
    exact_winding_readiness_undecided_validation,
    exact_boolean_boundary_preflight,
    exact_boolean_same_surface,
    exact_boolean_coplanar_convex_surface_equivalence,
    exact_boolean_coplanar_convex_surface_containment,
    exact_boolean_coplanar_convex_surface_arrangement_union,
    exact_boolean_coplanar_convex_surface_intersection,
    exact_boolean_coplanar_convex_surface_arrangement_difference,
    exact_boolean_coplanar_convex_surface_multi_difference,
    exact_convex_solid_classification,
    exact_boolean_coplanar_surface_containment,
    exact_boolean_open_surface_disjoint,
    exact_policy_report_refinement_blocker_validation,
    exact_boolean_coplanar_surface_intersection,
    exact_boolean_coplanar_surface_convex_union,
    exact_boolean_coplanar_surface_arrangement_union,
    exact_boolean_coplanar_surface_corner_difference,
    exact_boolean_coplanar_surface_holed_difference,
    exact_boolean_convex_containment
);
criterion_main!(benches);

fn p3(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}
