#![no_main]

use hyperlimit::Point3;
use hypermesh::exact::{ExactMesh, audit_exact_mesh, build_intersection_graph, classify_coplanar_triangles, classify_mesh_face_pair, classify_mesh_face_pairs, classify_mesh_triangle_against_retained_face_plane, classify_triangle_triangle, inspect_i64_mesh_input, intersect_segment_with_face_plane, intersect_segment_with_retained_face_plane};

use hypermesh::exact::region::{classify_face_regions_against_opposite_planes};


use hyperreal::Real;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut pos = Vec::new();
    let mut idx = Vec::new();

    for chunk in data.chunks_exact(8).take(96) {
        pos.push(i64::from_le_bytes(chunk.try_into().unwrap()));
    }

    for chunk in data.chunks_exact(2).skip(96).take(192) {
        idx.push(u16::from_le_bytes(chunk.try_into().unwrap()) as usize);
    }

    let input_report = inspect_i64_mesh_input(&pos, &idx);
    input_report.validate().unwrap();
    let _ = input_report.readiness();

    if let Ok(mesh) = ExactMesh::from_i64_triangles(&pos, &idx) {
        mesh.validate_retained_state().unwrap();
        let audit = audit_exact_mesh(&mesh).unwrap();
        audit.validate_against_mesh(&mesh).unwrap();
        assert_eq!(
            audit.freshness_against_mesh(&mesh),
            hypermesh::exact::ExactMeshAuditFreshness::Current
        );
        let readiness = mesh.consumer_readiness().unwrap();
        assert_eq!(
            readiness.freshness_against_mesh(&mesh),
            hypermesh::exact::ExactMeshConsumerReadinessFreshness::Current
        );
        readiness.validate_against_mesh(&mesh).unwrap();
        let package = mesh.handoff_package().unwrap();
        assert_eq!(
            package.freshness_against_mesh(&mesh),
            hypermesh::exact::ExactMeshHandoffPackageFreshness::Current
        );
        package.validate_internal().unwrap();
        package.validate_against_mesh(&mesh).unwrap();
        let surface_domain = hypermesh::exact::ExactMeshConsumerDomain::Surface;
        let solid_domain = hypermesh::exact::ExactMeshConsumerDomain::Solid;
        let approximate_domain = hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View;
        assert_eq!(
            package.has_domain(surface_domain),
            package.surface.is_some()
        );
        assert_eq!(package.has_domain(solid_domain), package.solid.is_some());
        assert_eq!(
            package.has_domain(approximate_domain),
            package.approximate_f64_view.is_some()
        );
        assert_eq!(
            package.available_domains(),
            [
                (surface_domain, package.surface.is_some()),
                (solid_domain, package.solid.is_some()),
                (approximate_domain, package.approximate_f64_view.is_some()),
            ]
            .into_iter()
            .filter_map(|(domain, present)| present.then_some(domain))
            .collect::<Vec<_>>()
        );
        assert!(
            package
                .exact_geometry_domains()
                .iter()
                .all(|domain| domain.is_exact_geometry())
        );
        assert!(
            package
                .lossy_adapter_domains()
                .iter()
                .all(|domain| domain.is_lossy_adapter())
        );
        let domain_summary = package.domain_summary();
        assert_eq!(
            domain_summary.has_exact_geometry(),
            !package.exact_geometry_domains().is_empty()
        );
        assert_eq!(
            domain_summary.has_lossy_adapter(),
            !package.lossy_adapter_domains().is_empty()
        );
        assert_eq!(
            domain_summary.has_domain(surface_domain),
            package.has_domain(surface_domain)
        );
        if let Some(preferred_domain) = domain_summary.preferred_exact_geometry_domain() {
            assert!(preferred_domain.is_exact_geometry());
            assert_eq!(
                domain_summary
                    .require_preferred_exact_geometry_domain()
                    .unwrap(),
                preferred_domain
            );
            assert_eq!(
                domain_summary
                    .require_preferred_exact_geometry_domain_against_package(&package)
                    .unwrap(),
                preferred_domain
            );
            assert_eq!(
                domain_summary
                    .require_preferred_exact_geometry_domain_against_mesh(&package, &mesh)
                    .unwrap(),
                preferred_domain
            );
            let preferred_summary_report = domain_summary
                .preferred_exact_geometry_report_against_mesh(&package, &mesh)
                .unwrap();
            assert_eq!(preferred_summary_report.domain(), preferred_domain);
            assert_eq!(preferred_summary_report.audit(), &package.audit);
            domain_summary.require_exact_geometry().unwrap();
        } else {
            assert!(
                domain_summary
                    .require_preferred_exact_geometry_domain()
                    .is_err()
            );
            assert!(domain_summary.require_exact_geometry().is_err());
        }
        if package.has_domain(surface_domain) {
            domain_summary.require_domain(surface_domain).unwrap();
            domain_summary
                .require_domain_against_package(&package, surface_domain)
                .unwrap();
            domain_summary
                .require_domain_against_mesh(&package, &mesh, surface_domain)
                .unwrap();
        } else {
            assert!(domain_summary.require_domain(surface_domain).is_err());
            assert!(
                domain_summary
                    .require_domain_against_package(&package, surface_domain)
                    .is_err()
            );
            assert!(
                domain_summary
                    .require_domain_against_mesh(&package, &mesh, surface_domain)
                    .is_err()
            );
        }
        if package.has_domain(approximate_domain) {
            domain_summary.require_lossy_adapter().unwrap();
        } else {
            assert!(domain_summary.require_lossy_adapter().is_err());
        }
        assert_eq!(
            domain_summary.available_domains,
            package.available_domains()
        );
        assert_eq!(
            domain_summary.exact_geometry_domains,
            package.exact_geometry_domains()
        );
        domain_summary.validate_against_package(&package).unwrap();
        domain_summary
            .validate_against_mesh(&package, &mesh)
            .unwrap();
        assert_eq!(
            domain_summary.freshness_against_package(&package),
            hypermesh::exact::ExactMeshDomainSummaryFreshness::Current
        );
        assert_eq!(
            domain_summary.freshness_against_mesh(&package, &mesh),
            hypermesh::exact::ExactMeshDomainSummaryFreshness::Current
        );
        if package.has_domain(approximate_domain) {
            package.require_domain(approximate_domain).unwrap();
        } else {
            assert!(package.require_domain(approximate_domain).is_err());
        }
        if let Some(preferred_domain) = package.preferred_exact_geometry_domain() {
            assert_eq!(
                package.require_preferred_exact_geometry_domain().unwrap(),
                preferred_domain
            );
            assert_eq!(
                package
                    .require_preferred_exact_geometry_domain_against_mesh(&mesh)
                    .unwrap(),
                preferred_domain
            );
            let preferred_package_report = package
                .preferred_exact_geometry_report_against_mesh(&mesh)
                .unwrap();
            assert_eq!(preferred_package_report.domain(), preferred_domain);
            assert_eq!(preferred_package_report.audit(), &package.audit);
        } else {
            assert!(package.require_preferred_exact_geometry_domain().is_err());
            assert!(
                package
                    .require_preferred_exact_geometry_domain_against_mesh(&mesh)
                    .is_err()
            );
        }
        if package.has_domain(surface_domain) {
            package.require_domain(surface_domain).unwrap();
            package
                .require_domain_against_mesh(&mesh, surface_domain)
                .unwrap();
            let report = package
                .domain_report_against_mesh(&mesh, surface_domain)
                .unwrap();
            assert_eq!(report.domain(), surface_domain);
            assert!(report.domain().is_exact_geometry());
            assert!(!report.domain().is_closed_volume());
            assert_eq!(report.audit(), &package.audit);
        } else {
            assert!(package.require_domain(surface_domain).is_err());
            assert!(
                package
                    .require_domain_against_mesh(&mesh, surface_domain)
                    .is_err()
            );
            assert!(
                package
                    .domain_report_against_mesh(&mesh, surface_domain)
                    .is_err()
            );
        }
        let _ = mesh.solid_handoff().map(|handoff| {
            assert_eq!(
                handoff.freshness_against_mesh(&mesh),
                hypermesh::exact::ExactSolidHandoffFreshness::Current
            );
            handoff.validate_against_mesh(&mesh)
        });
        let _ = mesh.surface_handoff().map(|handoff| {
            assert_eq!(
                handoff.freshness_against_mesh(&mesh),
                hypermesh::exact::ExactSurfaceHandoffFreshness::Current
            );
            handoff.validate_against_mesh(&mesh)
        });
        let _ = mesh.approximate_f64_view().map(|view| {
            assert_eq!(
                view.freshness_against_mesh(&mesh),
                hypermesh::exact::ApproximateMeshF64ViewFreshness::Current
            );
            view.validate_against_mesh(&mesh)
        });
        assert_eq!(mesh.facts().faces.len(), mesh.triangles().len());
        mesh.facts().validate().unwrap();
        for face in &mesh.facts().faces {
            let _ = (&face.plane.normal, &face.plane.offset);
        }
        let _ = mesh
            .bounds()
            .validate(mesh.vertices().len(), mesh.triangles().len());
        let _ = mesh.bounds().candidate_face_pairs(mesh.bounds());
        let axes = hypermesh::exact::SupportDopAxis3::kdop26_axes();
        let support = hypermesh::exact::support_dop_for_mesh(&mesh, &axes);
        if mesh.vertices().is_empty() {
            assert!(support.is_err());
        } else {
            let support = support.unwrap();
            support.validate_against_mesh(&mesh).unwrap();
            assert_eq!(
                support.expansion.kind,
                hypermesh::exact::SupportDopExpansionKind::None
            );
        }
        if mesh.facts().mesh.closed_manifold && !mesh.triangles().is_empty() {
            let boundary_vertex = mesh.triangles()[0].0[0];
            let boundary_point = mesh.vertices()[boundary_vertex].clone();
            let point_winding = hypermesh::exact::classify_point_against_closed_mesh_winding_report(
                &boundary_point,
                &mesh,
            );
            point_winding
                .validate_against_sources(&boundary_point, &mesh)
                .unwrap();
            assert_eq!(
                point_winding.relation,
                hypermesh::exact::ClosedMeshWindingRelation::Boundary
            );
            let mesh_winding =
                hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(
                    &mesh, &mesh,
                );
            mesh_winding.validate_against_sources(&mesh, &mesh).unwrap();
            assert_eq!(
                mesh_winding.relation,
                hypermesh::exact::ClosedMeshWindingMeshRelation::BoundaryOrMixed
            );
        }
        if !mesh.triangles().is_empty() {
            if mesh.vertices().len() >= 2 {
                let p0 = mesh.vertices()[0].clone();
                let p1 = mesh.vertices()[1].clone();
                let _ = intersect_segment_with_retained_face_plane(
                    &mesh.facts().faces[0].plane,
                    &p0,
                    &p1,
                )
                .validate();
            }
            let _ = classify_mesh_face_pair(&mesh, 0, &mesh, 0)
                .map(|classification| classification.validate());
            let _ = classify_mesh_triangle_against_retained_face_plane(&mesh, 0, &mesh, 0)
                .map(|classification| classification.validate());
            let _ = classify_mesh_face_pairs(&mesh, &mesh).map(|classifications| {
                for classification in classifications {
                    let _ = classification.validate();
                }
            });
            if let Ok(graph) = build_intersection_graph(&mesh, &mesh) {
                let _ = graph.validate();
                let _ = graph.validate_against_sources(&mesh, &mesh);
                for pair in &graph.face_pairs {
                    let _ = pair.validate_against_sources(&mesh, &mesh);
                }
                for overlap in graph.coplanar_overlap_graphs() {
                    let _ = overlap.validate();
                    let _ = overlap.validate_against_sources(&mesh, &mesh);
                }
                let _ = graph.coplanar_overlap_split_plan(&mesh, &mesh).map(|plan| {
                    let _ = plan.validate();
                    let _ = plan.validate_against_sources(&mesh, &mesh);
                    for graph in &plan.graphs {
                        let _ = graph.validate_against_sources(&mesh, &mesh);
                    }
                });
                let _ = graph
                    .coplanar_arrangement_readiness_report(&mesh, &mesh)
                    .map(|report| report.validate());
                let edge_split_plan = graph.edge_split_plan();
                let _ = edge_split_plan
                    .validate_against_sources(&mesh, &mesh)
                    .validate();
                let graph_vertex_plan = graph.graph_vertex_plan();
                let _ = graph_vertex_plan
                    .validate_against_sources(&mesh, &mesh)
                    .validate();
                let topology_plan = graph.split_topology_plan();
                let _ = topology_plan.validate().validate();
                let _ = topology_plan
                    .validate_against_sources(&mesh, &mesh)
                    .validate();
                let _ = graph.checked_graph_vertex_plan();
                let _ = graph.checked_split_topology_plan();
                let _ = graph.checked_face_split_plan();
                let face_plan = graph.face_split_plan();
                let _ = face_plan
                    .validate_against_topology(&topology_plan)
                    .validate();
                let _ = face_plan.validate_against_sources(&mesh, &mesh).validate();
                if let Ok(geometry_plan) = graph.face_split_geometry_plan(&mesh, &mesh) {
                    let _ = geometry_plan
                        .validate_boundary_incidence(&mesh, &mesh)
                        .validate();
                    let _ = geometry_plan
                        .validate_against_sources(&mesh, &mesh)
                        .validate();
                    let region_plan = geometry_plan.region_plan(&mesh, &mesh);
                    let _ = region_plan.validate(&mesh, &mesh).validate();
                    let _ = region_plan
                        .validate_against_sources(&mesh, &mesh)
                        .validate();
                    let classifications =
                        classify_face_regions_against_opposite_planes(&region_plan, &mesh, &mesh);
                    for classification in classifications {
                        let _ = classification.validate();
                    }
                                        let _ =
                        hypermesh::exact::region::checked_classify_face_regions_against_opposite_planes(
                            &region_plan,
                            &mesh,
                            &mesh,
                        )
                        .map(|classifications| {
                            for classification in classifications {
                                let _ = classification.validate();
                            }
                        });
                                        {
                        if let Ok(triangulations) =
                            hypermesh::exact::region::checked_triangulate_face_regions_with_earcut(
                                &region_plan,
                                &mesh,
                                &mesh,
                            )
                        {
                            for triangulation in &triangulations {
                                let _ = triangulation.validate();
                            }
                            let _ =
                                hypermesh::exact::region::ExactBooleanAssemblyPlan::from_region_triangulations(
                                    &triangulations,
                                    hypermesh::exact::region::ExactRegionSelection::KeepAll,
                                );
                        }
                    }
                }
            }
        }
    }

    if pos.len() >= 15 {
        let points = pos
            .chunks_exact(3)
            .take(5)
            .map(|coords| {
                Point3::new(
                    Real::from(coords[0]),
                    Real::from(coords[1]),
                    Real::from(coords[2]),
                )
                .clone()
            })
            .collect::<Vec<_>>();
        let _ = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]).validate();
        let axes = hypermesh::exact::SupportDopAxis3::orthogonal_axes();
        if let Ok(mut support) = hypermesh::exact::SupportDop3::from_points(&points, &axes) {
            support.validate_against_points(&points).unwrap();
            let _ = support.refresh_for_changed_vertices(&points, &[0]).unwrap();
        }
    }

    if pos.len() >= 18 {
        let points = pos
            .chunks_exact(3)
            .take(6)
            .map(|coords| {
                Point3::new(
                    Real::from(coords[0]),
                    Real::from(coords[1]),
                    Real::from(coords[2]),
                )
                .clone()
            })
            .collect::<Vec<_>>();
        let _ = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]).validate();
        let _ = classify_coplanar_triangles(&points, [0, 1, 2], [3, 4, 5]).validate();
    }
});
