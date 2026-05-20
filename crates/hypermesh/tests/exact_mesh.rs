#![cfg(feature = "exact")]

use hyperlimit::{PlaneSide, Point3, compare_reals};
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::CoplanarProjection;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::checked_classify_face_regions_against_opposite_planes;
#[cfg(not(feature = "exact-triangulation"))]
use hypermesh::exact::classify_face_regions_against_opposite_planes;
use hypermesh::exact::{
    AabbIntersectionKind, ApproximationPolicy, ConstructionProvenance, CoplanarTriangleRelation,
    DiagnosticKind, EdgeSplit, EdgeSplitPoint, ExactAabb3, ExactEdgeSplitPlan,
    ExactFaceSplitGeometryPlan, ExactFaceSplitPlan, ExactGraphVertex, ExactGraphVertexPlan,
    ExactGraphVertexUse, ExactIntersectionGraph, ExactMesh, ExactReal, ExactSplitTopologyPlan,
    FacePairEvents, FaceRegionBoundary, FaceSplitBoundaryChain, FaceSplitBoundaryNode,
    FaceSplitEdge, FaceSplitGeometry, FaceSplitPlan, IntersectionEvent, MeshFacePairRelation,
    MeshSide, MeshSource, SegmentPlaneRelation, Severity, SourceProvenance, SplitEdgeChain,
    SplitEdgeNode, SplitPlanDiagnosticKind, TrianglePlaneRelation, TriangleTriangleRelation,
    ValidationPolicy, VertexLinkKind, audit_exact_mesh, build_intersection_graph,
    certify_convex_solid, classify_coplanar_triangles, classify_mesh_face_pair,
    classify_mesh_face_pairs, classify_mesh_triangle_against_retained_face_plane,
    classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, classify_point_against_convex_solid,
    classify_point_against_convex_solid_report, classify_triangle_against_face_plane,
    classify_triangle_triangle, exact_mesh_consumer_readiness, exact_mesh_handoff_package,
    exact_solid_handoff, exact_surface_handoff, inspect_f64_mesh_input, inspect_i64_mesh_input,
    intersect_segment_with_face_plane, intersect_segment_with_retained_face_plane,
    validate_triangles, validate_triangles_with_policy,
};
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::{ExactPoint3, Triangle};
use hyperreal::Real;
use proptest::prelude::*;
use std::cmp::Ordering;

fn tetrahedron() -> (Vec<f64>, Vec<usize>) {
    (
        vec![
            0.0, 0.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, //
            0.0, 0.0, 1.0,
        ],
        vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
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

#[test]
fn exact_mesh_audit_report_replays_retained_state() {
    let (pos, idx) = tetrahedron();
    let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
    let report = audit_exact_mesh(&mesh).unwrap();

    assert_eq!(report.vertex_count, 4);
    assert_eq!(report.face_count, 4);
    assert_eq!(report.edge_count, 6);
    assert!(report.closed_manifold);
    assert!(report.fixed_coordinates_exact_rational);
    assert_eq!(report.validation_policy, ValidationPolicy::CLOSED);
    assert!(report.all_predicates_proof_producing());
    assert_eq!(
        report.freshness_against_mesh(&mesh),
        hypermesh::exact::ExactMeshAuditFreshness::Current
    );
    report.validate_against_mesh(&mesh).unwrap();

    let mut stale = report.clone();
    stale.edge_count += 1;
    assert_eq!(
        stale.validate_against_mesh(&mesh).unwrap_err(),
        hypermesh::exact::ExactMeshAuditError::CountMismatch {
            field: "edge_count",
            expected: 6,
            actual: 7,
        }
    );
    assert_eq!(
        stale.freshness_against_mesh(&mesh),
        hypermesh::exact::ExactMeshAuditFreshness::StaleCounts
    );

    let mut relabeled = report;
    relabeled.source_label.push_str(" stale");
    assert_eq!(
        relabeled.validate_against_mesh(&mesh).unwrap_err(),
        hypermesh::exact::ExactMeshAuditError::SourceLabelMismatch
    );
    assert_eq!(
        relabeled.freshness_against_mesh(&mesh),
        hypermesh::exact::ExactMeshAuditFreshness::StaleSource
    );

    let mut stale_version = audit_exact_mesh(&mesh).unwrap();
    stale_version.construction_version += 1;
    assert_eq!(
        stale_version.validate_against_mesh(&mesh).unwrap_err(),
        hypermesh::exact::ExactMeshAuditError::ConstructionVersionMismatch {
            expected: mesh.provenance().construction_version,
            actual: 2,
        }
    );
    assert_eq!(
        stale_version.freshness_against_mesh(&mesh),
        hypermesh::exact::ExactMeshAuditFreshness::StaleConstructionVersion
    );

    let mut stale_policy = audit_exact_mesh(&mesh).unwrap();
    stale_policy.validation_policy = ValidationPolicy::ALLOW_BOUNDARY;
    assert_eq!(
        stale_policy.validate_against_mesh(&mesh).unwrap_err(),
        hypermesh::exact::ExactMeshAuditError::ValidationPolicyMismatch {
            expected: ValidationPolicy::CLOSED,
            actual: ValidationPolicy::ALLOW_BOUNDARY,
        }
    );
    assert_eq!(
        stale_policy.freshness_against_mesh(&mesh),
        hypermesh::exact::ExactMeshAuditFreshness::StaleValidationPolicy
    );
}

#[test]
fn exact_solid_handoff_accepts_only_closed_exact_solids() {
    let (pos, idx) = tetrahedron();
    let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
    let report = exact_solid_handoff(&mesh).unwrap();

    assert_eq!(report.audit.face_count, 4);
    assert_eq!(report.retained_face_planes, 4);
    assert!(report.retained_mesh_bounds);
    assert!(report.nonempty_topology);
    assert!(report.proof_predicate_ready);
    assert!(!report.source_is_exact());
    assert_eq!(
        report.freshness_against_mesh(&mesh),
        hypermesh::exact::ExactSolidHandoffFreshness::Current
    );
    report.validate_against_mesh(&mesh).unwrap();
    mesh.solid_handoff().unwrap();

    let mut stale = report.clone();
    stale.retained_face_planes += 1;
    assert_eq!(
        stale.freshness_against_mesh(&mesh),
        hypermesh::exact::ExactSolidHandoffFreshness::StaleReport
    );

    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(open.validation_policy(), ValidationPolicy::ALLOW_BOUNDARY);
    assert_eq!(
        audit_exact_mesh(&open).unwrap().validation_policy,
        ValidationPolicy::ALLOW_BOUNDARY
    );
    assert_eq!(
        exact_solid_handoff(&open).unwrap_err(),
        hypermesh::exact::ExactSolidHandoffError::NotClosedManifold
    );

    let exact = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            1, 0, 0, //
            0, 1, 0, //
            0, 0, 1,
        ],
        &idx,
    )
    .unwrap();
    assert!(exact_solid_handoff(&exact).unwrap().source_is_exact());
}

#[test]
fn exact_surface_handoff_accepts_open_and_closed_exact_surfaces() {
    let (pos, idx) = tetrahedron();
    let closed = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
    let closed_report = exact_surface_handoff(&closed).unwrap();

    assert_eq!(closed_report.audit.face_count, 4);
    assert_eq!(closed_report.retained_face_planes, 4);
    assert!(closed_report.retained_mesh_bounds);
    assert!(closed_report.nonempty_topology);
    assert!(closed_report.closed_manifold);
    assert_eq!(closed_report.validation_policy, ValidationPolicy::CLOSED);
    assert!(closed_report.proof_predicate_ready);
    assert!(!closed_report.boundary_allowed());
    assert!(!closed_report.source_is_exact());
    assert_eq!(
        closed_report.freshness_against_mesh(&closed),
        hypermesh::exact::ExactSurfaceHandoffFreshness::Current
    );
    closed_report.validate_against_mesh(&closed).unwrap();
    closed.surface_handoff().unwrap();

    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_report = open.surface_handoff().unwrap();
    assert_eq!(open_report.audit.face_count, 1);
    assert_eq!(open_report.retained_face_planes, 1);
    assert!(!open_report.closed_manifold);
    assert!(open_report.boundary_allowed());
    assert!(open_report.source_is_exact());
    assert_eq!(
        open_report.freshness_against_mesh(&open),
        hypermesh::exact::ExactSurfaceHandoffFreshness::Current
    );
    open_report.validate_against_mesh(&open).unwrap();

    let mut stale = open_report.clone();
    stale.validation_policy = ValidationPolicy::CLOSED;
    assert_eq!(
        stale.freshness_against_mesh(&open),
        hypermesh::exact::ExactSurfaceHandoffFreshness::StaleReport
    );
    assert_eq!(
        stale.validate_against_mesh(&open).unwrap_err(),
        hypermesh::exact::ExactSurfaceHandoffError::ReportMismatch {
            field: "exact_surface_handoff",
        }
    );
}

#[test]
fn exact_mesh_consumer_readiness_keeps_domains_separate() {
    let (pos, idx) = tetrahedron();
    let closed = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
    let closed_report = exact_mesh_consumer_readiness(&closed).unwrap();

    assert!(closed_report.surface_handoff_ready);
    assert!(closed_report.solid_handoff_ready);
    assert!(closed_report.approximate_f64_view_ready);
    assert!(closed_report.nonempty_topology);
    assert!(closed_report.closed_manifold);
    assert!(!closed_report.boundary_allowed);
    assert!(closed_report.exact_rational_coordinates);
    assert!(!closed_report.exact_source);
    assert_eq!(
        closed_report.freshness_against_mesh(&closed),
        hypermesh::exact::ExactMeshConsumerReadinessFreshness::Current
    );
    closed_report.validate_against_mesh(&closed).unwrap();
    closed.consumer_readiness().unwrap();

    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_report = open.consumer_readiness().unwrap();
    assert!(open_report.surface_handoff_ready);
    assert!(!open_report.solid_handoff_ready);
    assert!(open_report.approximate_f64_view_ready);
    assert!(open_report.nonempty_topology);
    assert!(!open_report.closed_manifold);
    assert!(open_report.boundary_allowed);
    assert!(open_report.exact_source);

    let mut stale = open_report.clone();
    stale.solid_handoff_ready = true;
    assert_eq!(
        stale.validate_against_mesh(&open).unwrap_err(),
        hypermesh::exact::ExactMeshConsumerReadinessError::ReportMismatch {
            field: "exact_mesh_consumer_readiness",
        }
    );
    assert_eq!(
        stale.freshness_against_mesh(&open),
        hypermesh::exact::ExactMeshConsumerReadinessFreshness::StaleReport
    );
}

#[test]
fn exact_mesh_handoff_package_replays_optional_domains() {
    let (pos, idx) = tetrahedron();
    let closed = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
    let closed_package = exact_mesh_handoff_package(&closed).unwrap();

    assert!(hypermesh::exact::ExactMeshConsumerDomain::Surface.is_exact_geometry());
    assert!(hypermesh::exact::ExactMeshConsumerDomain::Solid.is_exact_geometry());
    assert!(hypermesh::exact::ExactMeshConsumerDomain::Solid.is_closed_volume());
    assert!(!hypermesh::exact::ExactMeshConsumerDomain::Surface.is_closed_volume());
    assert!(hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View.is_lossy_adapter());
    assert!(!hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View.is_exact_geometry());

    assert!(closed_package.readiness.surface_handoff_ready);
    assert!(closed_package.readiness.solid_handoff_ready);
    assert!(closed_package.surface.is_some());
    assert!(closed_package.solid.is_some());
    assert!(closed_package.approximate_f64_view.is_some());
    assert_eq!(
        closed_package.available_domains(),
        vec![
            hypermesh::exact::ExactMeshConsumerDomain::Surface,
            hypermesh::exact::ExactMeshConsumerDomain::Solid,
            hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View,
        ]
    );
    assert_eq!(
        closed_package.exact_geometry_domains(),
        vec![
            hypermesh::exact::ExactMeshConsumerDomain::Surface,
            hypermesh::exact::ExactMeshConsumerDomain::Solid,
        ]
    );
    assert_eq!(
        closed_package.lossy_adapter_domains(),
        vec![hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View]
    );
    let closed_summary = closed_package.domain_summary();
    assert_eq!(closed_summary.exact_geometry_count, 2);
    assert_eq!(closed_summary.lossy_adapter_count, 1);
    assert!(closed_summary.closed_volume_ready);
    assert!(closed_summary.has_exact_geometry());
    assert!(closed_summary.has_lossy_adapter());
    assert!(closed_summary.has_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid));
    assert_eq!(
        closed_summary.preferred_exact_geometry_domain(),
        Some(hypermesh::exact::ExactMeshConsumerDomain::Solid)
    );
    assert_eq!(
        closed_summary
            .require_preferred_exact_geometry_domain()
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Solid
    );
    assert_eq!(
        closed_package.preferred_exact_geometry_domain(),
        Some(hypermesh::exact::ExactMeshConsumerDomain::Solid)
    );
    assert_eq!(
        closed_summary
            .require_preferred_exact_geometry_domain_against_package(&closed_package)
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Solid
    );
    assert_eq!(
        closed_summary
            .require_preferred_exact_geometry_domain_against_mesh(&closed_package, &closed)
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Solid
    );
    assert_eq!(
        closed_package
            .require_preferred_exact_geometry_domain()
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Solid
    );
    assert_eq!(
        closed_package
            .require_preferred_exact_geometry_domain_against_mesh(&closed)
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Solid
    );
    assert!(matches!(
        closed_package.preferred_exact_geometry_report().unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Solid(_)
    ));
    assert!(matches!(
        closed_package
            .preferred_exact_geometry_report_against_mesh(&closed)
            .unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Solid(_)
    ));
    assert!(matches!(
        closed_summary
            .preferred_exact_geometry_report_against_package(&closed_package)
            .unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Solid(_)
    ));
    assert!(matches!(
        closed_summary
            .preferred_exact_geometry_report_against_mesh(&closed_package, &closed)
            .unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Solid(_)
    ));
    closed_summary
        .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid)
        .unwrap();
    closed_summary.require_exact_geometry().unwrap();
    closed_summary.require_lossy_adapter().unwrap();
    closed_summary.require_closed_volume().unwrap();
    assert_eq!(
        closed_summary.available_domains,
        closed_package.available_domains()
    );
    closed_summary
        .validate_against_package(&closed_package)
        .unwrap();
    closed_summary
        .validate_against_mesh(&closed_package, &closed)
        .unwrap();
    closed_summary
        .require_domain_against_package(
            &closed_package,
            hypermesh::exact::ExactMeshConsumerDomain::Solid,
        )
        .unwrap();
    closed_summary
        .require_domain_against_mesh(
            &closed_package,
            &closed,
            hypermesh::exact::ExactMeshConsumerDomain::Solid,
        )
        .unwrap();
    assert_eq!(
        closed_summary.freshness_against_package(&closed_package),
        hypermesh::exact::ExactMeshDomainSummaryFreshness::Current
    );
    assert_eq!(
        closed_summary.freshness_against_mesh(&closed_package, &closed),
        hypermesh::exact::ExactMeshDomainSummaryFreshness::Current
    );
    assert!(closed_package.has_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid));
    closed_package
        .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Surface)
        .unwrap();
    closed_package
        .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid)
        .unwrap();
    closed_package
        .require_domain(hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View)
        .unwrap();
    closed_package
        .require_domain_against_mesh(&closed, hypermesh::exact::ExactMeshConsumerDomain::Solid)
        .unwrap();
    let closed_solid = closed_package
        .domain_report(hypermesh::exact::ExactMeshConsumerDomain::Solid)
        .unwrap();
    assert!(matches!(
        closed_solid,
        hypermesh::exact::ExactMeshDomainReportRef::Solid(_)
    ));
    assert_eq!(
        closed_solid.domain(),
        hypermesh::exact::ExactMeshConsumerDomain::Solid
    );
    assert_eq!(closed_solid.audit(), &closed_package.audit);
    let closed_view = closed_package
        .domain_report_against_mesh(
            &closed,
            hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View,
        )
        .unwrap();
    assert!(matches!(
        closed_view,
        hypermesh::exact::ExactMeshDomainReportRef::ApproximateF64View(_)
    ));
    assert_eq!(
        closed_view.domain(),
        hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View
    );
    assert!(closed_view.domain().is_lossy_adapter());
    assert_eq!(closed_view.audit(), &closed_package.audit);
    assert_eq!(
        closed_package.freshness_against_mesh(&closed),
        hypermesh::exact::ExactMeshHandoffPackageFreshness::Current
    );
    closed_package.validate_against_mesh(&closed).unwrap();
    closed.handoff_package().unwrap();

    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_package = open.handoff_package().unwrap();
    assert!(open_package.readiness.surface_handoff_ready);
    assert!(!open_package.readiness.solid_handoff_ready);
    assert!(open_package.surface.is_some());
    assert!(open_package.solid.is_none());
    assert!(open_package.approximate_f64_view.is_some());
    assert_eq!(
        open_package.available_domains(),
        vec![
            hypermesh::exact::ExactMeshConsumerDomain::Surface,
            hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View,
        ]
    );
    assert_eq!(
        open_package.exact_geometry_domains(),
        vec![hypermesh::exact::ExactMeshConsumerDomain::Surface]
    );
    assert_eq!(
        open_package.lossy_adapter_domains(),
        vec![hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View]
    );
    let open_summary = open_package.domain_summary();
    assert_eq!(open_summary.exact_geometry_count, 1);
    assert_eq!(open_summary.lossy_adapter_count, 1);
    assert!(!open_summary.closed_volume_ready);
    assert!(open_summary.has_exact_geometry());
    assert!(open_summary.has_lossy_adapter());
    assert!(!open_summary.has_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid));
    assert_eq!(
        open_summary.preferred_exact_geometry_domain(),
        Some(hypermesh::exact::ExactMeshConsumerDomain::Surface)
    );
    assert_eq!(
        open_summary
            .require_preferred_exact_geometry_domain()
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Surface
    );
    assert_eq!(
        open_package.preferred_exact_geometry_domain(),
        Some(hypermesh::exact::ExactMeshConsumerDomain::Surface)
    );
    assert_eq!(
        open_summary
            .require_preferred_exact_geometry_domain_against_package(&open_package)
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Surface
    );
    assert_eq!(
        open_summary
            .require_preferred_exact_geometry_domain_against_mesh(&open_package, &open)
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Surface
    );
    assert_eq!(
        open_package
            .require_preferred_exact_geometry_domain()
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Surface
    );
    assert_eq!(
        open_package
            .require_preferred_exact_geometry_domain_against_mesh(&open)
            .unwrap(),
        hypermesh::exact::ExactMeshConsumerDomain::Surface
    );
    assert!(matches!(
        open_package.preferred_exact_geometry_report().unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Surface(_)
    ));
    assert!(matches!(
        open_package
            .preferred_exact_geometry_report_against_mesh(&open)
            .unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Surface(_)
    ));
    assert!(matches!(
        open_summary
            .preferred_exact_geometry_report_against_package(&open_package)
            .unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Surface(_)
    ));
    assert!(matches!(
        open_summary
            .preferred_exact_geometry_report_against_mesh(&open_package, &open)
            .unwrap(),
        hypermesh::exact::ExactMeshDomainReportRef::Surface(_)
    ));
    open_summary
        .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Surface)
        .unwrap();
    open_summary.require_exact_geometry().unwrap();
    open_summary.require_lossy_adapter().unwrap();
    assert_eq!(
        open_summary.require_closed_volume().unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::MissingClosedVolume
    );
    assert_eq!(
        open_summary
            .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid)
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::MissingDomain {
            domain: hypermesh::exact::ExactMeshConsumerDomain::Solid,
        }
    );
    assert_eq!(
        open_summary
            .require_domain_against_package(
                &open_package,
                hypermesh::exact::ExactMeshConsumerDomain::Solid,
            )
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::MissingDomain {
            domain: hypermesh::exact::ExactMeshConsumerDomain::Solid,
        }
    );
    assert_eq!(
        open_summary
            .require_domain_against_mesh(
                &open_package,
                &open,
                hypermesh::exact::ExactMeshConsumerDomain::Solid,
            )
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::MissingDomain {
            domain: hypermesh::exact::ExactMeshConsumerDomain::Solid,
        }
    );
    assert_eq!(
        open_summary.exact_geometry_domains,
        open_package.exact_geometry_domains()
    );
    open_summary
        .validate_against_package(&open_package)
        .unwrap();
    let mut stale_summary = open_summary.clone();
    stale_summary.closed_volume_ready = true;
    assert_eq!(
        stale_summary
            .validate_against_package(&open_package)
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::SummaryMismatch {
            field: "closed_volume_ready",
        }
    );
    assert_eq!(
        stale_summary.freshness_against_package(&open_package),
        hypermesh::exact::ExactMeshDomainSummaryFreshness::StaleSummary
    );
    assert_eq!(
        stale_summary.freshness_against_mesh(&open_package, &open),
        hypermesh::exact::ExactMeshDomainSummaryFreshness::StaleSummary
    );
    assert!(!open_package.has_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid));
    open_package.validate_internal().unwrap();
    open_package
        .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Surface)
        .unwrap();
    open_package
        .require_domain(hypermesh::exact::ExactMeshConsumerDomain::ApproximateF64View)
        .unwrap();
    assert_eq!(
        open_package
            .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid)
            .unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::MissingDomain {
            domain: hypermesh::exact::ExactMeshConsumerDomain::Solid,
        }
    );
    assert_eq!(
        open_package
            .domain_report(hypermesh::exact::ExactMeshConsumerDomain::Solid)
            .unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::MissingDomain {
            domain: hypermesh::exact::ExactMeshConsumerDomain::Solid,
        }
    );
    let open_surface = open_package
        .domain_report_against_mesh(&open, hypermesh::exact::ExactMeshConsumerDomain::Surface)
        .unwrap();
    assert!(matches!(
        open_surface,
        hypermesh::exact::ExactMeshDomainReportRef::Surface(_)
    ));
    assert_eq!(
        open_surface.domain(),
        hypermesh::exact::ExactMeshConsumerDomain::Surface
    );
    assert!(open_surface.domain().is_exact_geometry());
    assert!(!open_surface.domain().is_closed_volume());
    assert_eq!(open_surface.audit(), &open_package.audit);
    assert_eq!(
        open_package
            .require_domain_against_mesh(&open, hypermesh::exact::ExactMeshConsumerDomain::Solid)
            .unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::MissingDomain {
            domain: hypermesh::exact::ExactMeshConsumerDomain::Solid,
        }
    );

    let mut contradictory = open_package.clone();
    contradictory.readiness.solid_handoff_ready = true;
    assert_eq!(
        contradictory.validate_internal().unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::InternalMismatch {
            field: "readiness.solid_handoff_ready",
        }
    );
    assert_eq!(
        contradictory.validate_against_mesh(&open).unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::InternalMismatch {
            field: "readiness.solid_handoff_ready",
        }
    );

    let mut stale = open_package.clone();
    stale.solid = closed_package.solid;
    assert!(
        stale
            .require_domain(hypermesh::exact::ExactMeshConsumerDomain::Solid)
            .is_ok()
    );
    assert_eq!(
        stale.validate_internal().unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::InternalMismatch {
            field: "readiness.solid_handoff_ready",
        }
    );
    assert_eq!(
        stale
            .require_domain_against_mesh(&open, hypermesh::exact::ExactMeshConsumerDomain::Solid)
            .unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::InternalMismatch {
            field: "readiness.solid_handoff_ready",
        }
    );

    let mut stale_replay_only = open_package.clone();
    stale_replay_only.audit.face_count += 1;
    stale_replay_only.readiness.audit.face_count += 1;
    if let Some(surface) = &mut stale_replay_only.surface {
        surface.audit.face_count += 1;
    }
    if let Some(view) = &mut stale_replay_only.approximate_f64_view {
        view.audit.face_count += 1;
    }
    assert!(stale_replay_only.validate_internal().is_ok());
    assert_eq!(
        open_summary
            .validate_against_mesh(&stale_replay_only, &open)
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::Package(
            hypermesh::exact::ExactMeshHandoffPackageError::PackageMismatch {
                field: "exact_mesh_handoff_package",
            }
        )
    );
    assert_eq!(
        open_summary
            .require_domain_against_mesh(
                &stale_replay_only,
                &open,
                hypermesh::exact::ExactMeshConsumerDomain::Surface,
            )
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::Package(
            hypermesh::exact::ExactMeshHandoffPackageError::PackageMismatch {
                field: "exact_mesh_handoff_package",
            }
        )
    );
    assert_eq!(
        open_summary
            .require_preferred_exact_geometry_domain_against_mesh(&stale_replay_only, &open)
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::Package(
            hypermesh::exact::ExactMeshHandoffPackageError::PackageMismatch {
                field: "exact_mesh_handoff_package",
            }
        )
    );
    assert_eq!(
        open_summary
            .preferred_exact_geometry_report_against_mesh(&stale_replay_only, &open)
            .unwrap_err(),
        hypermesh::exact::ExactMeshDomainSummaryError::Package(
            hypermesh::exact::ExactMeshHandoffPackageError::PackageMismatch {
                field: "exact_mesh_handoff_package",
            }
        )
    );
    assert_eq!(
        stale_replay_only
            .require_domain_against_mesh(&open, hypermesh::exact::ExactMeshConsumerDomain::Surface)
            .unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::PackageMismatch {
            field: "exact_mesh_handoff_package",
        }
    );
    assert_eq!(
        stale_replay_only
            .preferred_exact_geometry_report_against_mesh(&open)
            .unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::PackageMismatch {
            field: "exact_mesh_handoff_package",
        }
    );
    assert_eq!(
        stale_replay_only
            .require_preferred_exact_geometry_domain_against_mesh(&open)
            .unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::PackageMismatch {
            field: "exact_mesh_handoff_package",
        }
    );
    assert_eq!(
        stale.validate_against_mesh(&open).unwrap_err(),
        hypermesh::exact::ExactMeshHandoffPackageError::InternalMismatch {
            field: "readiness.solid_handoff_ready",
        }
    );
    assert_eq!(
        stale.freshness_against_mesh(&open),
        hypermesh::exact::ExactMeshHandoffPackageFreshness::StalePackage
    );
}

#[test]
fn approximate_f64_view_replays_against_exact_mesh() {
    let (pos, idx) = tetrahedron();
    let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();
    let view = mesh.approximate_f64_view().unwrap();

    assert!(view.lossy_view);
    assert_eq!(view.positions, pos);
    assert_eq!(view.indices, idx);
    assert_eq!(view.exported_coordinates, pos.len());
    assert_eq!(
        view.freshness_against_mesh(&mesh),
        hypermesh::exact::ApproximateMeshF64ViewFreshness::Current
    );
    view.validate_against_mesh(&mesh).unwrap();

    let mut stale_coordinate = view.clone();
    stale_coordinate.positions[0] = 42.0;
    assert_eq!(
        stale_coordinate.validate_against_mesh(&mesh).unwrap_err(),
        hypermesh::exact::ApproximateMeshF64ViewError::CoordinateReplayMismatch { coordinate: 0 }
    );
    assert_eq!(
        stale_coordinate.freshness_against_mesh(&mesh),
        hypermesh::exact::ApproximateMeshF64ViewFreshness::StaleCoordinate
    );

    let mut stale_index = view.clone();
    stale_index.indices[0] = 3;
    assert_eq!(
        stale_index.validate_against_mesh(&mesh).unwrap_err(),
        hypermesh::exact::ApproximateMeshF64ViewError::IndexReplayMismatch { index: 0 }
    );
    assert_eq!(
        stale_index.freshness_against_mesh(&mesh),
        hypermesh::exact::ApproximateMeshF64ViewFreshness::StaleIndex
    );

    let mut relabeled = view;
    relabeled.lossy_view = false;
    assert_eq!(
        relabeled.validate_against_mesh(&mesh).unwrap_err(),
        hypermesh::exact::ApproximateMeshF64ViewError::MissingLossyViewFlag
    );
    assert_eq!(
        relabeled.freshness_against_mesh(&mesh),
        hypermesh::exact::ApproximateMeshF64ViewFreshness::MissingLossyFlag
    );
}

#[test]
fn lossy_f64_mesh_input_report_keeps_adapter_edge_explicit() {
    let (pos, idx) = tetrahedron();
    let report = inspect_f64_mesh_input(&pos, &idx);

    assert!(report.edge_ready());
    assert_eq!(
        report.readiness(),
        hypermesh::exact::LossyF64MeshInputReadiness::Ready
    );
    assert_eq!(report.vertex_count, Some(4));
    assert_eq!(report.face_count, Some(4));
    assert_eq!(report.exact_dyadic_coordinates, pos.len());
    assert_eq!(report.checked_indices, idx.len());
    assert!(report.diagnostics.is_empty());
    report.validate().unwrap();
    assert_eq!(ExactMesh::inspect_f64_triangles(&pos, &idx), report);

    let malformed = inspect_f64_mesh_input(&[0.0, f64::NAN, 1.0, 2.0], &[0, 0, 7, 1]);
    assert!(!malformed.edge_ready());
    assert_eq!(
        malformed.readiness(),
        hypermesh::exact::LossyF64MeshInputReadiness::InvalidCoordinateArity
    );
    assert_eq!(malformed.vertex_count, None);
    assert_eq!(malformed.face_count, None);
    assert!(
        malformed
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::VertexBufferArity)
    );
    assert!(
        malformed
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexBufferArity)
    );
    assert!(
        malformed
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::NonFiniteCoordinate)
    );
    malformed.validate().unwrap();
}

#[test]
fn exact_i64_mesh_input_report_keeps_exact_edge_explicit() {
    let pos = vec![
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let idx = vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
    let report = inspect_i64_mesh_input(&pos, &idx);

    assert!(report.edge_ready());
    assert_eq!(
        report.readiness(),
        hypermesh::exact::ExactI64MeshInputReadiness::Ready
    );
    assert_eq!(report.vertex_count, Some(4));
    assert_eq!(report.face_count, Some(4));
    assert_eq!(report.exact_integer_coordinates, pos.len());
    assert_eq!(report.checked_indices, idx.len());
    assert!(report.diagnostics.is_empty());
    report.validate().unwrap();
    assert_eq!(ExactMesh::inspect_i64_triangles(&pos, &idx), report);

    let malformed = inspect_i64_mesh_input(&[0, 0, 0, 1], &[0, 0, 7, 1]);
    assert!(!malformed.edge_ready());
    assert_eq!(
        malformed.readiness(),
        hypermesh::exact::ExactI64MeshInputReadiness::InvalidCoordinateArity
    );
    assert_eq!(malformed.vertex_count, None);
    assert_eq!(malformed.face_count, None);
    assert!(
        malformed
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::VertexBufferArity)
    );
    assert!(
        malformed
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexBufferArity)
    );
    malformed.validate().unwrap();

    let out_of_range = inspect_i64_mesh_input(&pos, &[0, 1, 8]);
    assert!(!out_of_range.edge_ready());
    assert_eq!(
        out_of_range.readiness(),
        hypermesh::exact::ExactI64MeshInputReadiness::InvalidIndex
    );
    assert!(
        out_of_range
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
    out_of_range.validate().unwrap();
}

#[test]
fn exact_mesh_accepts_closed_tetrahedron_with_certified_facts() {
    let (pos, idx) = tetrahedron();
    let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();

    assert_eq!(mesh.facts().mesh.vertex_count, 4);
    assert_eq!(mesh.facts().mesh.face_count, 4);
    assert_eq!(mesh.facts().mesh.edge_count, 6);
    assert_eq!(mesh.facts().mesh.euler_characteristic, 2);
    assert!(mesh.facts().mesh.closed_manifold);
    assert!(mesh.facts().mesh.fixed_coordinates_exact_rational);
    assert!(
        mesh.facts()
            .vertices
            .iter()
            .all(|vertex| vertex.link == VertexLinkKind::Circle)
    );
    mesh.facts().validate().unwrap();
    let points = mesh
        .vertices()
        .iter()
        .map(|vertex| vertex.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let triangles = mesh
        .triangles()
        .iter()
        .map(|triangle| triangle.0)
        .collect::<Vec<_>>();
    mesh.facts()
        .validate_against_sources(&points, &triangles)
        .unwrap();
    let shifted = [p3(10, 0, 0), p3(11, 0, 0), p3(10, 1, 0), p3(10, 0, 1)];
    assert_eq!(
        mesh.facts()
            .validate_against_sources(&shifted, &triangles)
            .unwrap_err(),
        hypermesh::exact::MeshFactsValidationError::SourceReplayMismatch
    );
    mesh.validate_retained_state().unwrap();
    let base_plane = &mesh.facts().faces[0].plane;
    assert_eq!(
        compare_reals(&base_plane.normal[0], &ExactReal::from(0)).value(),
        Some(Ordering::Equal)
    );
    assert_eq!(
        compare_reals(&base_plane.normal[1], &ExactReal::from(0)).value(),
        Some(Ordering::Equal)
    );
    assert_eq!(
        compare_reals(&base_plane.normal[2], &ExactReal::from(-1)).value(),
        Some(Ordering::Equal)
    );
    assert_eq!(
        compare_reals(&base_plane.offset, &ExactReal::from(0)).value(),
        Some(Ordering::Equal)
    );
    assert!(
        mesh.provenance()
            .predicates
            .iter()
            .all(|predicate| predicate.is_proof_producing())
    );
}

#[test]
fn exact_closed_mesh_winding_classifies_points_without_epsilon() {
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

    let inside =
        hypermesh::exact::classify_point_against_closed_mesh_winding_report(&p3(1, 1, 1), &mesh);
    assert_eq!(
        inside.relation,
        hypermesh::exact::ClosedMeshWindingRelation::Inside
    );
    inside
        .validate_against_sources(&p3(1, 1, 1), &mesh)
        .unwrap();
    assert!(inside.crossings % 2 == 1);

    let outside =
        hypermesh::exact::classify_point_against_closed_mesh_winding_report(&p3(20, 20, 20), &mesh);
    assert_eq!(
        outside.relation,
        hypermesh::exact::ClosedMeshWindingRelation::Outside
    );
    outside
        .validate_against_sources(&p3(20, 20, 20), &mesh)
        .unwrap();
    assert!(outside.crossings.is_multiple_of(2));

    let boundary =
        hypermesh::exact::classify_point_against_closed_mesh_winding_report(&p3(0, 0, 0), &mesh);
    assert_eq!(
        boundary.relation,
        hypermesh::exact::ClosedMeshWindingRelation::Boundary
    );
    boundary
        .validate_against_sources(&p3(0, 0, 0), &mesh)
        .unwrap();
    assert_ne!(boundary.boundary_hits, 0);
}

#[test]
fn exact_closed_mesh_winding_reports_not_closed_and_stale_artifacts() {
    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 0, 10, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let report =
        hypermesh::exact::classify_point_against_closed_mesh_winding_report(&p3(1, 1, 0), &open);
    assert_eq!(
        report.relation,
        hypermesh::exact::ClosedMeshWindingRelation::NotClosed
    );
    report
        .validate_against_sources(&p3(1, 1, 0), &open)
        .unwrap();

    let closed = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    assert_eq!(
        report
            .validate_against_sources(&p3(1, 1, 1), &closed)
            .unwrap_err(),
        hypermesh::exact::WindingReportError::SourceReplayMismatch
    );
}

#[test]
fn exact_nonconvex_closed_mesh_winding_classifies_subject_vertices() {
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
    assert_eq!(
        certify_convex_solid(&outer).convexity,
        hypermesh::exact::ConvexSolidClassification::NonConvex
    );
    let inner = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let report =
        hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(&inner, &outer);
    assert_eq!(
        report.relation,
        hypermesh::exact::ClosedMeshWindingMeshRelation::StrictlyInside
    );
    assert_eq!(report.subject_vertex_count, inner.vertices().len());
    assert!(
        report
            .vertices
            .iter()
            .all(|vertex| vertex.relation == hypermesh::exact::ClosedMeshWindingRelation::Inside)
    );
    report.validate_against_sources(&inner, &outer).unwrap();

    let mut stale = report;
    stale.vertices[0].crossings = 0;
    assert_eq!(
        stale.validate().unwrap_err(),
        hypermesh::exact::WindingReportError::StatusEvidenceMismatch
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_use_winding_for_nonconvex_no_intersection_containment() {
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
    let inner = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedWindingContainment
    );
    preflight.validate_against_sources(&inner, &outer).unwrap();

    let intersection = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::WindingContainment
        }
    );
    assert_eq!(intersection.mesh.vertices(), inner.vertices());
    assert_eq!(intersection.mesh.triangles(), inner.triangles());
    intersection
        .validate_against_sources(&inner, &outer)
        .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(union.mesh.vertices(), outer.vertices());
    assert_eq!(union.mesh.triangles(), outer.triangles());

    let difference = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert!(difference.mesh.vertices().is_empty());
    assert!(difference.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_use_winding_for_nonconvex_aabb_overlap_separation() {
    let left = ExactMesh::from_i64_triangles(
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
    assert_eq!(
        certify_convex_solid(&left).convexity,
        hypermesh::exact::ConvexSolidClassification::NonConvex
    );
    let right = ExactMesh::from_i64_triangles(
        &[15, 1, 1, 16, 1, 1, 15, 2, 1, 15, 1, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    assert_ne!(
        left.bounds()
            .mesh
            .as_ref()
            .unwrap()
            .classify_intersection(right.bounds().mesh.as_ref().unwrap())
            .value(),
        Some(hypermesh::exact::AabbIntersectionKind::Disjoint)
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedWindingSeparated
    );
    preflight.validate_against_sources(&left, &right).unwrap();

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::WindingSeparated
        }
    );
    assert!(intersection.mesh.vertices().is_empty());
    assert!(intersection.mesh.triangles().is_empty());
    intersection
        .validate_against_sources(&left, &right)
        .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(
        union.mesh.vertices().len(),
        left.vertices().len() + right.vertices().len()
    );
    assert_eq!(
        union.mesh.triangles().len(),
        left.triangles().len() + right.triangles().len()
    );

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(difference.mesh.vertices(), left.vertices());
    assert_eq!(difference.mesh.triangles(), left.triangles());
}

#[test]
fn exact_mesh_lifts_integer_grid_without_lossy_source() {
    let pos = vec![
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let idx = vec![0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
    let mesh = ExactMesh::from_i64_triangles(&pos, &idx).unwrap();

    assert!(mesh.facts().mesh.closed_manifold);
    assert!(mesh.facts().mesh.fixed_coordinates_exact_rational);
    mesh.bounds()
        .validate(mesh.vertices().len(), mesh.triangles().len())
        .unwrap();
    mesh.facts().validate().unwrap();
    mesh.validate_retained_state().unwrap();
    assert_eq!(mesh.provenance().source.label, "flat i64 triangle mesh");
}

#[test]
fn exact_provenance_validation_rejects_inconsistent_artifacts() {
    let empty_label = ConstructionProvenance::new(SourceProvenance {
        source: MeshSource::Exact,
        label: String::new(),
        approximation: ApproximationPolicy::ExactOnly,
    });
    assert_eq!(
        empty_label.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::EmptySourceLabel
    );

    let lossy_as_exact = ConstructionProvenance::new(SourceProvenance {
        source: MeshSource::LossyF64,
        label: "bad source policy".to_string(),
        approximation: ApproximationPolicy::ExactOnly,
    });
    assert_eq!(
        lossy_as_exact.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::SourceApproximationMismatch
    );

    let exact_as_edge = ConstructionProvenance::new(SourceProvenance {
        source: MeshSource::Exact,
        label: "bad exact policy".to_string(),
        approximation: ApproximationPolicy::EdgeOnly,
    });
    assert_eq!(
        exact_as_edge.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::SourceApproximationMismatch
    );

    SourceProvenance::exact("direct exact provenance")
        .validate()
        .unwrap();
    assert_eq!(
        SourceProvenance {
            source: MeshSource::LossyF64,
            label: "bad direct lossy policy".to_string(),
            approximation: ApproximationPolicy::ExplicitApproximateDecision,
        }
        .validate()
        .unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::LossySourcePolicyMismatch
    );
    SourceProvenance::legacy_boolmesh_adapter("reported legacy adapter")
        .validate()
        .unwrap();
    assert_eq!(
        SourceProvenance {
            source: MeshSource::LegacyBoolmeshAdapter,
            label: "legacy edge cannot pretend to be display-only".to_string(),
            approximation: ApproximationPolicy::EdgeOnly,
        }
        .validate()
        .unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::LegacyAdapterPolicyMismatch
    );
    SourceProvenance::external_adapter("reported external adapter")
        .validate()
        .unwrap();
    assert_eq!(
        SourceProvenance {
            source: MeshSource::ExternalAdapter,
            label: "external edge cannot opt into topology".to_string(),
            approximation: ApproximationPolicy::ExplicitApproximateDecision,
        }
        .validate()
        .unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::ExternalAdapterPolicyMismatch
    );

    let predicate = hypermesh::exact::PredicateUse::from_certificate(
        hyperlimit::PredicateCertificate::ExactRealFact,
    );
    predicate.validate().unwrap();

    let mut stale_stage = predicate;
    stale_stage.stage = hyperlimit::PredicatePrecisionStage::CertifiedFilter;
    assert_eq!(
        stale_stage.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::PredicateMetadataMismatch
    );

    let mut stale_semantics = predicate;
    stale_semantics.semantics = hyperlimit::PredicateApiSemantics::ApproximationDeferring;
    assert_eq!(
        stale_semantics.validate().unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::PredicateMetadataMismatch
    );

    assert_eq!(
        hypermesh::exact::PredicateUse::from_certificate(hyperlimit::PredicateCertificate::Unknown)
            .validate()
            .unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::NonProofProducingPredicate
    );

    assert_eq!(
        ConstructionProvenance::with_version(SourceProvenance::exact("zero version"), 0)
            .validate()
            .unwrap_err(),
        hypermesh::exact::ConstructionProvenanceValidationError::InvalidConstructionVersion
    );
}

#[test]
fn exact_mesh_fact_validation_rejects_inconsistent_artifacts() {
    let (pos, idx) = tetrahedron();
    let mesh = ExactMesh::from_f64_triangles(&pos, &idx).unwrap();

    let mut bad_summary = mesh.facts().clone();
    bad_summary.mesh.edge_count += 1;
    assert_eq!(
        bad_summary.validate().unwrap_err(),
        hypermesh::exact::MeshFactsValidationError::SummaryLengthMismatch {
            field: "edge_count",
            expected: 6,
            actual: 7,
        }
    );

    let mut bad_face = mesh.facts().clone();
    bad_face.faces[0].oriented.directed_edges[0] = [1, 0];
    assert_eq!(
        bad_face.validate().unwrap_err(),
        hypermesh::exact::MeshFactsValidationError::FaceDirectedEdgesMismatch {
            face: 0,
            expected: [[0, 2], [2, 1], [1, 0]],
            actual: [[1, 0], [2, 1], [1, 0]],
        }
    );

    let mut bad_edge = mesh.facts().clone();
    bad_edge.edges[0].directed_uses = [2, 0];
    assert_eq!(
        bad_edge.validate().unwrap_err(),
        hypermesh::exact::MeshFactsValidationError::EdgeUseMismatch {
            edge: [0, 1],
            expected_directed_uses: [1, 1],
            actual_directed_uses: [2, 0],
            expected_incident_faces: 2,
            actual_incident_faces: 2,
        }
    );
}

#[test]
fn exact_bounds_validation_rejects_inconsistent_artifacts() {
    let points = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
    let triangles = [[0, 1, 2]];
    let bounds = hypermesh::exact::MeshBounds::from_triangles(&points, &triangles);
    bounds
        .mesh
        .as_ref()
        .unwrap()
        .validate_against_points(&points)
        .unwrap();
    bounds.faces[0]
        .validate_against_triangle([&points[0], &points[1], &points[2]])
        .unwrap();
    bounds
        .validate_against_sources(&points, &triangles)
        .unwrap();
    let shifted = [p3(2, 0, 0), p3(3, 0, 0), p3(2, 1, 0)];
    assert_eq!(
        bounds
            .mesh
            .as_ref()
            .unwrap()
            .validate_against_points(&shifted)
            .unwrap_err(),
        hypermesh::exact::BoundsValidationError::SourceReplayMismatch
    );
    assert_eq!(
        bounds.faces[0]
            .validate_against_triangle([&shifted[0], &shifted[1], &shifted[2]])
            .unwrap_err(),
        hypermesh::exact::BoundsValidationError::SourceReplayMismatch
    );
    assert_eq!(
        bounds
            .validate_against_sources(&shifted, &triangles)
            .unwrap_err(),
        hypermesh::exact::BoundsValidationError::SourceReplayMismatch
    );

    let inverted = ExactAabb3 {
        min: p3(1, 0, 0),
        max: p3(0, 0, 0),
    };
    assert_eq!(
        inverted.validate().unwrap_err(),
        hypermesh::exact::BoundsValidationError::InvertedAxis
    );

    let missing_mesh = hypermesh::exact::MeshBounds {
        mesh: None,
        faces: Vec::new(),
    };
    assert_eq!(
        missing_mesh.validate(1, 0).unwrap_err(),
        hypermesh::exact::BoundsValidationError::MissingMeshBounds
    );
}

#[test]
fn exact_bounds_reject_disjoint_face_pairs_without_narrow_phase() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert_eq!(
        left.bounds()
            .mesh
            .as_ref()
            .unwrap()
            .classify_intersection(right.bounds().mesh.as_ref().unwrap())
            .value(),
        Some(AabbIntersectionKind::Disjoint)
    );
    assert!(
        left.bounds()
            .candidate_face_pairs(right.bounds())
            .is_empty()
    );
}

#[test]
fn exact_bounds_keep_touching_faces_as_candidates() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 2, 0, 0, 1, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert_eq!(
        left.bounds().faces[0]
            .classify_intersection(&right.bounds().faces[0])
            .value(),
        Some(AabbIntersectionKind::Touching)
    );
    assert_eq!(
        left.bounds().candidate_face_pairs(right.bounds()),
        vec![[0, 0]]
    );
}

#[test]
fn exact_bounds_can_retain_symbolic_unknown_relation() {
    let zero = ExactReal::from(0);
    let one = ExactReal::from(1);
    let pi = ExactReal::pi();
    let left = ExactAabb3 {
        min: Point3::new(zero.clone(), zero.clone(), zero.clone()),
        max: Point3::new(pi.clone(), one.clone(), one.clone()),
    };
    let right = ExactAabb3 {
        min: Point3::new(one.clone(), zero.clone(), zero.clone()),
        max: Point3::new(one.clone() + pi, one.clone(), one),
    };

    let relation = left.classify_intersection(&right);
    assert!(
        relation
            .value()
            .is_none_or(AabbIntersectionKind::needs_narrow_phase)
    );
}

#[test]
fn exact_support_dop_tracks_witnesses_and_replays_mesh() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 3, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let axes = hypermesh::exact::SupportDopAxis3::kdop26_axes();
    let support = hypermesh::exact::support_dop_for_mesh(&mesh, &axes).unwrap();

    assert_eq!(support.vertex_count, 3);
    assert_eq!(support.slabs.len(), axes.len());
    assert_eq!(
        support.expansion.kind,
        hypermesh::exact::SupportDopExpansionKind::None
    );
    assert_eq!(support.expansion.expanded_slabs, 0);
    support.validate_against_mesh(&mesh).unwrap();

    let x_slab = support
        .slabs
        .iter()
        .find(|slab| slab.axis.direction == [1, 0, 0])
        .unwrap();
    assert_eq!(x_slab.min.vertex, 0);
    assert_eq!(x_slab.max.vertex, 1);
    assert_real_eq(&x_slab.min.distance, &ExactReal::from(0));
    assert_real_eq(&x_slab.max.distance, &ExactReal::from(2));

    let xy_slab = support
        .slabs
        .iter()
        .find(|slab| slab.axis.direction == [1, 1, 0])
        .unwrap();
    assert_eq!(xy_slab.max.vertex, 2);
    assert_real_eq(&xy_slab.max.distance, &ExactReal::from(3));
}

#[test]
fn exact_support_dop_reports_lossy_adapter_expansion_boundary() {
    let mesh = ExactMesh::from_f64_triangles_with_policy(
        &[0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 1.0, 0.0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let axes = hypermesh::exact::SupportDopAxis3::orthogonal_axes();
    let support = hypermesh::exact::support_dop_for_mesh(&mesh, &axes).unwrap();

    assert_eq!(
        support.expansion.kind,
        hypermesh::exact::SupportDopExpansionKind::LossyAdapter
    );
    assert_eq!(support.expansion.expanded_slabs, axes.len());
    support.expansion.validate().unwrap();
    support.validate_against_mesh(&mesh).unwrap();

    let x_slab = support
        .slabs
        .iter()
        .find(|slab| slab.axis.direction == [1, 0, 0])
        .unwrap();
    assert_real_eq(
        &x_slab.conservative_min_distance(&support.expansion),
        &x_slab.min.distance,
    );
    assert_real_eq(
        &x_slab.conservative_max_distance(&support.expansion),
        &x_slab.max.distance,
    );
}

#[test]
fn exact_support_dop_refresh_rebuilds_only_invalidated_witness_axes() {
    let axes = hypermesh::exact::SupportDopAxis3::orthogonal_axes();
    let mut points = vec![p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
    let mut support = hypermesh::exact::SupportDop3::from_points(&points, &axes).unwrap();

    points[2] = p3(2, 2, 0);
    let refresh = support.refresh_for_changed_vertices(&points, &[2]).unwrap();
    assert_eq!(refresh.axis_count, axes.len());
    assert_eq!(refresh.invalidated_witness_axes, 1);
    assert_eq!(refresh.axes_rebuilt, 1);
    assert_eq!(refresh.axes_extended, 1);
    assert_eq!(refresh.axes_unchanged, 1);
    support.validate_against_points(&points).unwrap();

    let x_slab = support
        .slabs
        .iter()
        .find(|slab| slab.axis.direction == [1, 0, 0])
        .unwrap();
    let y_slab = support
        .slabs
        .iter()
        .find(|slab| slab.axis.direction == [0, 1, 0])
        .unwrap();
    assert_eq!(x_slab.max.vertex, 2);
    assert_eq!(y_slab.max.vertex, 2);
    assert_real_eq(&x_slab.max.distance, &ExactReal::from(2));
    assert_real_eq(&y_slab.max.distance, &ExactReal::from(2));
}

#[test]
fn exact_support_dop_validation_rejects_stale_or_malformed_artifacts() {
    let axes = hypermesh::exact::SupportDopAxis3::orthogonal_axes();
    let mut points = vec![p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
    let support = hypermesh::exact::SupportDop3::from_points(&points, &axes).unwrap();

    points[1] = p3(5, 0, 0);
    assert_eq!(
        support.validate_against_points(&points).unwrap_err(),
        hypermesh::exact::SupportDopValidationError::WitnessPointMismatch
    );

    let bad_axis = hypermesh::exact::SupportDopAxis3::new([0, 0, 0]);
    assert_eq!(
        hypermesh::exact::SupportDop3::from_points(&[p3(0, 0, 0)], &[bad_axis]).unwrap_err(),
        hypermesh::exact::SupportDopValidationError::ZeroAxis
    );

    let malformed = hypermesh::exact::SupportDopExpansionReport {
        kind: hypermesh::exact::SupportDopExpansionKind::None,
        axis_count: 1,
        expanded_slabs: 1,
        expansion: ExactReal::from(0),
    };
    assert_eq!(
        malformed.validate().unwrap_err(),
        hypermesh::exact::SupportDopValidationError::ExpansionKindMismatch
    );
}

#[test]
fn exact_narrow_phase_classifies_triangle_plane_side() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, 1),
        p3(1, 0, 1),
        p3(0, 1, 1),
    ];
    let classification = classify_triangle_against_face_plane(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(
        classification.relation,
        TrianglePlaneRelation::StrictlyBelow
    );
    classification.validate().unwrap();
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_narrow_phase_reuses_retained_face_plane_facts() {
    let plane = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let below = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -2, 1, 0, -2, 0, 1, -2],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let retained =
        classify_mesh_triangle_against_retained_face_plane(&plane, 0, &below, 0).unwrap();
    let points = plane
        .vertices()
        .iter()
        .chain(below.vertices())
        .map(|point| point.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let predicate = classify_triangle_against_face_plane(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(retained.relation, predicate.relation);
    assert_eq!(retained.vertex_sides, predicate.vertex_sides);
    assert_eq!(retained.relation, TrianglePlaneRelation::StrictlyAbove);
    retained.validate().unwrap();
    predicate.validate().unwrap();
    assert!(retained.predicates.is_empty());
}

#[test]
fn exact_narrow_phase_classifies_coplanar_and_straddling_triangles() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(1, 1, 0),
        p3(0, 0, 1),
        p3(0, 0, -1),
    ];

    let coplanar = classify_triangle_against_face_plane(&points, [0, 1, 2], [3, 4, 5]);
    let straddling = classify_triangle_against_face_plane(&points, [0, 1, 2], [0, 6, 7]);

    assert_eq!(coplanar.relation, TrianglePlaneRelation::Coplanar);
    assert_eq!(straddling.relation, TrianglePlaneRelation::Straddling);
    coplanar.validate().unwrap();
    straddling.validate().unwrap();
    assert!(coplanar.all_proof_producing());
    assert!(straddling.all_proof_producing());
}

#[test]
fn exact_narrow_phase_validation_rejects_inconsistent_plane_artifacts() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, 1),
        p3(1, 0, 1),
        p3(0, 1, 1),
    ];
    let replayed = classify_triangle_against_face_plane(&points, [0, 1, 2], [3, 4, 5]);
    replayed
        .validate_against_sources(&points, [0, 1, 2], [3, 4, 5])
        .unwrap();
    assert_eq!(
        replayed
            .validate_against_sources(&points, [3, 4, 5], [0, 1, 2])
            .unwrap_err(),
        hypermesh::exact::TrianglePlaneValidationError::SourceReplayMismatch
    );

    let classification = hypermesh::exact::TrianglePlaneClassification {
        relation: TrianglePlaneRelation::Coplanar,
        vertex_sides: [
            Some(PlaneSide::Above),
            Some(PlaneSide::Above),
            Some(PlaneSide::Above),
        ],
        predicates: Vec::new(),
    };

    assert_eq!(
        classification.validate().unwrap_err(),
        hypermesh::exact::TrianglePlaneValidationError::RelationMismatch
    );
}

#[test]
fn exact_segment_plane_constructs_proper_crossing_as_ratio() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, -1),
        p3(0, 0, 1),
    ];

    let event = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);

    assert_eq!(event.relation, SegmentPlaneRelation::ProperCrossing);
    event.validate().unwrap();
    event
        .validate_against_sources(&points[0], &points[1], &points[2], &points[3], &points[4])
        .unwrap();
    assert_eq!(
        event
            .validate_against_sources(&points[0], &points[1], &points[2], &points[4], &points[3])
            .unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::SourceReplayMismatch
    );
    assert_eq!(
        event.endpoint_sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
    );
    assert!(event.all_proof_producing());
    assert_real_eq(event.parameter.as_ref().unwrap(), &half());
    let ratio = event.parameter_ratio.as_ref().unwrap();
    assert_real_eq(
        &(&ratio.numerator / &ratio.denominator).expect("nonzero crossing denominator"),
        &half(),
    );
    let point = event.point.as_ref().unwrap();
    assert_real_eq(&point.x, &ExactReal::from(0));
    assert_real_eq(&point.y, &ExactReal::from(0));
    assert_real_eq(&point.z, &ExactReal::from(0));
}

#[test]
fn exact_segment_plane_reuses_retained_face_plane_for_crossing() {
    let plane = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let p0 = p3(0, 0, -1);
    let p1 = p3(0, 0, 1);
    let retained =
        intersect_segment_with_retained_face_plane(&plane.facts().faces[0].plane, &p0, &p1);
    let predicate = intersect_segment_with_face_plane(
        &[p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0), p0, p1],
        [0, 1, 2],
        [3, 4],
    );

    assert_eq!(retained.relation, predicate.relation);
    assert_eq!(retained.endpoint_sides, predicate.endpoint_sides);
    assert_eq!(retained.relation, SegmentPlaneRelation::ProperCrossing);
    retained.validate().unwrap();
    predicate.validate().unwrap();
    assert!(retained.predicates.is_empty());
    assert_real_eq(retained.parameter.as_ref().unwrap(), &half());
    let ratio = retained.parameter_ratio.as_ref().unwrap();
    assert_real_eq(
        &(&ratio.numerator / &ratio.denominator).expect("nonzero retained crossing denominator"),
        &half(),
    );
    let point = retained.point.as_ref().unwrap();
    assert_real_eq(&point.x, &ExactReal::from(0));
    assert_real_eq(&point.y, &ExactReal::from(0));
    assert_real_eq(&point.z, &ExactReal::from(0));
}

#[test]
fn exact_segment_plane_classifies_endpoint_coplanar_and_disjoint_cases() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(2, 0, 0),
        p3(2, 0, 1),
        p3(0, 0, 2),
        p3(1, 0, 2),
        p3(0, 0, 0),
        p3(1, 1, 0),
    ];

    let endpoint = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);
    assert_eq!(endpoint.relation, SegmentPlaneRelation::EndpointOnPlane);
    endpoint.validate().unwrap();
    assert_eq!(endpoint.endpoint_on_plane, Some(0));
    assert_real_eq(endpoint.parameter.as_ref().unwrap(), &ExactReal::from(0));

    let disjoint = intersect_segment_with_face_plane(&points, [0, 1, 2], [5, 6]);
    assert_eq!(disjoint.relation, SegmentPlaneRelation::Disjoint);
    disjoint.validate().unwrap();
    assert!(disjoint.point.is_none());

    let coplanar = intersect_segment_with_face_plane(&points, [0, 1, 2], [7, 8]);
    assert_eq!(coplanar.relation, SegmentPlaneRelation::Coplanar);
    coplanar.validate().unwrap();
    assert!(coplanar.parameter.is_none());
}

#[test]
fn exact_segment_plane_validation_rejects_inconsistent_artifacts() {
    let invalid_crossing = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ProperCrossing,
        point: Some(p3(0, 0, 0)),
        parameter: Some(half()),
        parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
            numerator: ExactReal::from(1),
            denominator: ExactReal::from(2),
        }),
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        invalid_crossing.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::ProperCrossingSideFactsMismatch
    );

    let invalid_disjoint = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::Disjoint,
        point: Some(p3(0, 0, 0)),
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        invalid_disjoint.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::UnexpectedConstruction
    );

    let out_of_range_crossing = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ProperCrossing,
        point: Some(p3(0, 0, 0)),
        parameter: Some(ExactReal::from(2)),
        parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
            numerator: ExactReal::from(2),
            denominator: ExactReal::from(1),
        }),
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        out_of_range_crossing.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::ProperCrossingParameterOutOfRange
    );

    let endpoint_that_is_really_coplanar = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::EndpointOnPlane,
        point: Some(p3(0, 0, 0)),
        parameter: Some(ExactReal::from(0)),
        parameter_ratio: None,
        endpoint_on_plane: Some(0),
        endpoint_sides: [Some(PlaneSide::On), Some(PlaneSide::On)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        endpoint_that_is_really_coplanar.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::EndpointSideFactsMismatch
    );

    let mismatched_ratio = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ProperCrossing,
        point: Some(p3(0, 0, 0)),
        parameter: Some(half()),
        parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
            numerator: ExactReal::from(2),
            denominator: ExactReal::from(3),
        }),
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        mismatched_ratio.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::ProperCrossingRatioMismatch
    );

    let failed_without_reason = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ConstructionFailed,
        point: None,
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: None,
    };
    assert_eq!(
        failed_without_reason.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::MissingConstructionFailureReason
    );

    let failed_with_reason = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::ConstructionFailed,
        point: None,
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: Vec::new(),
        construction_failure: Some(
            hypermesh::exact::SegmentPlaneConstructionFailure::ZeroDenominator,
        ),
    };
    failed_with_reason.validate().unwrap();

    let disjoint_with_failure_reason = hypermesh::exact::SegmentPlaneIntersection {
        relation: SegmentPlaneRelation::Disjoint,
        point: None,
        parameter: None,
        parameter_ratio: None,
        endpoint_on_plane: None,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        predicates: Vec::new(),
        construction_failure: Some(
            hypermesh::exact::SegmentPlaneConstructionFailure::ParameterDivisionFailed,
        ),
    };
    assert_eq!(
        disjoint_with_failure_reason.validate().unwrap_err(),
        hypermesh::exact::SegmentPlaneValidationError::UnexpectedConstructionFailureReason
    );
}

#[test]
fn exact_triangle_triangle_rejects_plane_separated_pair() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(0, 0, 2),
        p3(1, 0, 2),
        p3(0, 1, 2),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(
        classification.relation,
        TriangleTriangleRelation::SeparatedByFirstPlane
    );
    classification.validate().unwrap();
    classification
        .validate_against_sources(&points, [0, 1, 2], [3, 4, 5])
        .unwrap();
    assert_eq!(
        classification
            .validate_against_sources(&points, [3, 4, 5], [0, 1, 2])
            .unwrap_err(),
        hypermesh::exact::TriangleTriangleValidationError::SourceReplayMismatch
    );
    assert!(classification.right_edge_events.is_empty());
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_triangle_triangle_keeps_coplanar_overlap_for_later_graph() {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(1, 0, 0),
        p3(3, 0, 0),
        p3(1, 2, 0),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(
        classification.relation,
        TriangleTriangleRelation::CoplanarOverlapping
    );
    assert_eq!(
        classification.coplanar.as_ref().unwrap().relation,
        CoplanarTriangleRelation::Overlapping
    );
    assert!(classification.right_edge_events.is_empty());
    assert!(classification.left_edge_events.is_empty());
    classification.validate().unwrap();
    classification
        .validate_against_sources(&points, [0, 1, 2], [3, 4, 5])
        .unwrap();
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_coplanar_triangle_classifier_distinguishes_disjoint_touching_and_overlap() {
    let disjoint_points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(3, 0, 0),
        p3(5, 0, 0),
        p3(3, 2, 0),
    ];
    let touching_points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(2, 0, 0),
        p3(4, 0, 0),
        p3(2, 2, 0),
    ];
    let overlapping_points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(1, 0, 0),
        p3(3, 0, 0),
        p3(1, 2, 0),
    ];

    let disjoint = classify_coplanar_triangles(&disjoint_points, [0, 1, 2], [3, 4, 5]);
    let touching = classify_coplanar_triangles(&touching_points, [0, 1, 2], [3, 4, 5]);
    let overlapping = classify_coplanar_triangles(&overlapping_points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(disjoint.relation, CoplanarTriangleRelation::Disjoint);
    assert_eq!(touching.relation, CoplanarTriangleRelation::Touching);
    assert_eq!(overlapping.relation, CoplanarTriangleRelation::Overlapping);
    disjoint.validate().unwrap();
    touching.validate().unwrap();
    overlapping.validate().unwrap();
    disjoint
        .validate_against_sources(&disjoint_points, [0, 1, 2], [3, 4, 5])
        .unwrap();
    touching
        .validate_against_sources(&touching_points, [0, 1, 2], [3, 4, 5])
        .unwrap();
    overlapping
        .validate_against_sources(&overlapping_points, [0, 1, 2], [3, 4, 5])
        .unwrap();
    assert_eq!(
        overlapping
            .validate_against_sources(&disjoint_points, [0, 1, 2], [3, 4, 5])
            .unwrap_err(),
        hypermesh::exact::CoplanarTriangleValidationError::SourceReplayMismatch
    );
}

#[test]
fn exact_coplanar_triangle_validation_rejects_inconsistent_artifacts() {
    let no_projection = hypermesh::exact::CoplanarTriangleClassification {
        projection: None,
        relation: CoplanarTriangleRelation::Overlapping,
        edge_intersections: Vec::new(),
        right_vertices_in_left: [None, None, None],
        left_vertices_in_right: [None, None, None],
        predicates: Vec::new(),
    };
    assert_eq!(
        no_projection.validate().unwrap_err(),
        hypermesh::exact::CoplanarTriangleValidationError::DecidedRelationWithoutProjection
    );

    let missing_edges = hypermesh::exact::CoplanarTriangleClassification {
        projection: Some(hypermesh::exact::CoplanarProjection::Xy),
        relation: CoplanarTriangleRelation::Disjoint,
        edge_intersections: Vec::new(),
        right_vertices_in_left: [
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
        ],
        left_vertices_in_right: [
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
            Some(hyperlimit::TriangleLocation::Outside),
        ],
        predicates: Vec::new(),
    };
    assert_eq!(
        missing_edges.validate().unwrap_err(),
        hypermesh::exact::CoplanarTriangleValidationError::MissingEdgeIntersections
    );
}

#[test]
fn exact_triangle_triangle_retains_segment_plane_events_for_candidates() {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(0, 0, -1),
        p3(2, 0, 1),
        p3(0, 2, 1),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(classification.relation, TriangleTriangleRelation::Candidate);
    classification.validate().unwrap();
    assert_eq!(classification.right_edge_events.len(), 3);
    assert!(
        classification
            .right_edge_events
            .iter()
            .filter(|event| event.relation == SegmentPlaneRelation::ProperCrossing)
            .count()
            >= 2
    );
    assert!(classification.all_proof_producing());
}

#[test]
fn exact_triangle_triangle_validation_rejects_inconsistent_artifacts() {
    let points = [
        p3(0, 0, 0),
        p3(2, 0, 0),
        p3(0, 2, 0),
        p3(0, 0, -1),
        p3(2, 0, 1),
        p3(0, 2, 1),
    ];
    let mut candidate = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
    candidate.right_edge_events.clear();

    assert_eq!(
        candidate.validate().unwrap_err(),
        hypermesh::exact::TriangleTriangleValidationError::CandidateEdgeEventCountMismatch
    );

    let mut separated = classify_triangle_triangle(
        &[
            p3(0, 0, 0),
            p3(1, 0, 0),
            p3(0, 1, 0),
            p3(0, 0, 2),
            p3(1, 0, 2),
            p3(0, 1, 2),
        ],
        [0, 1, 2],
        [3, 4, 5],
    );
    separated.relation = TriangleTriangleRelation::Candidate;

    assert_eq!(
        separated.validate().unwrap_err(),
        hypermesh::exact::TriangleTriangleValidationError::PlaneRelationMismatch
    );
}

#[test]
fn exact_mesh_face_pair_classifier_uses_bounds_before_triangle_predicates() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(
        classification.relation,
        MeshFacePairRelation::BoundsDisjoint
    );
    classification.validate().unwrap();
    classification
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        classification
            .validate_against_sources(&left, &left)
            .unwrap_err(),
        hypermesh::exact::MeshFacePairValidationError::SourceReplayMismatch
    );
    assert!(!classification.needs_graph_construction());
    assert!(classification.triangle.is_none());
}

#[test]
fn exact_mesh_face_pair_classifier_uses_retained_planes_before_triangle_predicates() {
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

    assert!(
        left.bounds().faces[0]
            .classify_intersection(&right.bounds().faces[0])
            .value()
            .is_none_or(AabbIntersectionKind::needs_narrow_phase)
    );

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(
        classification.relation,
        MeshFacePairRelation::PlaneSeparated
    );
    classification.validate().unwrap();
    classification
        .validate_against_sources(&left, &right)
        .unwrap();
    assert!(!classification.needs_graph_construction());
    assert!(classification.triangle.is_none());
}

#[test]
fn exact_mesh_face_pair_classifier_retains_triangle_candidates() {
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

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(classification.relation, MeshFacePairRelation::Candidate);
    classification.validate().unwrap();
    classification
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        classification
            .validate_against_sources(&right, &left)
            .unwrap_err(),
        hypermesh::exact::MeshFacePairValidationError::SourceReplayMismatch
    );
    assert!(classification.needs_graph_construction());
    let triangle = classification.triangle.as_ref().unwrap();
    assert_eq!(triangle.right_edge_events.len(), 3);
    assert_eq!(triangle.left_edge_events.len(), 3);
    assert!(
        triangle
            .right_edge_events
            .iter()
            .chain(&triangle.left_edge_events)
            .all(|event| event.predicates.is_empty())
    );
}

#[test]
fn exact_mesh_face_pair_classifier_rejects_coplanar_disjoint_pairs() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 4, 3, 0, 3, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let classification = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    assert_eq!(
        classification.relation,
        MeshFacePairRelation::PlaneSeparated
    );
    classification.validate().unwrap();
    assert!(!classification.needs_graph_construction());
    assert_eq!(
        classification.triangle.as_ref().unwrap().relation,
        TriangleTriangleRelation::CoplanarDisjoint
    );
}

#[test]
fn exact_edge_split_validation_rejects_missing_and_noncrossing_side_facts() {
    let split_plan = ExactEdgeSplitPlan {
        splits: vec![EdgeSplit {
            side: MeshSide::Left,
            edge: [0, 1],
            points: vec![
                EdgeSplitPoint {
                    face_pair: [0, 0],
                    plane_face: 0,
                    parameter: half(),
                    parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                        numerator: ExactReal::from(1),
                        denominator: ExactReal::from(2),
                    },
                    point: p3(0, 0, 0),
                    endpoint_sides: [None, Some(PlaneSide::Below)],
                },
                EdgeSplitPoint {
                    face_pair: [0, 0],
                    plane_face: 0,
                    parameter: half(),
                    parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                        numerator: ExactReal::from(2),
                        denominator: ExactReal::from(3),
                    },
                    point: p3(0, 0, 0),
                    endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
                },
            ],
        }],
        unknown_orderings: 1,
    };

    let report = split_plan.validate();

    assert!(!report.is_valid());
    report.validate().unwrap();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
    }));
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnknownOrdering)
    );
}

#[test]
fn exact_split_plan_report_validation_rejects_malformed_diagnostics() {
    let empty_message = hypermesh::exact::SplitPlanValidationReport {
        diagnostics: vec![hypermesh::exact::SplitPlanDiagnostic {
            kind: SplitPlanDiagnosticKind::UnknownOrdering,
            message: "   ".to_string(),
            side: None,
            face: None,
            edge: None,
            graph_vertex: None,
        }],
    };
    assert_eq!(
        empty_message.validate().unwrap_err(),
        hypermesh::exact::SplitPlanReportValidationError::EmptyMessage
    );

    let missing_edge = hypermesh::exact::SplitPlanValidationReport {
        diagnostics: vec![hypermesh::exact::SplitPlanDiagnostic {
            kind: SplitPlanDiagnosticKind::WrongChainEnd,
            message: "chain end is not retained".to_string(),
            side: Some(MeshSide::Left),
            face: None,
            edge: None,
            graph_vertex: None,
        }],
    };
    assert_eq!(
        missing_edge.validate().unwrap_err(),
        hypermesh::exact::SplitPlanReportValidationError::MissingEdge
    );

    let missing_graph_vertex = hypermesh::exact::SplitPlanValidationReport {
        diagnostics: vec![hypermesh::exact::SplitPlanDiagnostic {
            kind: SplitPlanDiagnosticKind::MissingFaceSplitSourceUse,
            message: "source use missing".to_string(),
            side: Some(MeshSide::Right),
            face: Some(2),
            edge: Some([0, 1]),
            graph_vertex: None,
        }],
    };
    assert_eq!(
        missing_graph_vertex.validate().unwrap_err(),
        hypermesh::exact::SplitPlanReportValidationError::MissingGraphVertex
    );
}

#[test]
fn exact_checked_topology_plan_rejects_invalid_edge_split_handoff() {
    let graph = ExactIntersectionGraph {
        face_pairs: vec![FacePairEvents {
            left_face: 0,
            right_face: 0,
            relation: MeshFacePairRelation::Candidate,
            projection: None,
            events: vec![IntersectionEvent::SegmentPlane {
                segment_side: MeshSide::Left,
                edge: [0, 1],
                plane_side: MeshSide::Right,
                plane_face: 0,
                relation: SegmentPlaneRelation::ProperCrossing,
                point: Some(p3(0, 0, 0)),
                parameter: Some(half()),
                parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(1),
                    denominator: ExactReal::from(2),
                }),
                construction_failure: None,
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
            }],
        }],
    };

    assert_eq!(
        graph.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::InvalidSegmentPlaneEvent
    );
    let report = graph.checked_split_topology_plan().unwrap_err();

    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
    let face_report = graph.checked_face_split_plan().unwrap_err();
    assert!(face_report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
}

#[test]
fn exact_intersection_graph_validation_rejects_inconsistent_events() {
    let rejected_pair = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::PlaneSeparated,
        projection: None,
        events: vec![IntersectionEvent::Unknown],
    };
    assert_eq!(
        rejected_pair.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::RejectedPairHasEvents
    );

    let missing_projection = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::CoplanarOverlapping,
        projection: None,
        events: vec![IntersectionEvent::CoplanarVertex {
            vertex_side: MeshSide::Left,
            vertex: 0,
            triangle_side: MeshSide::Right,
            triangle_face: 0,
            location: hyperlimit::TriangleLocation::Inside,
        }],
    };
    assert_eq!(
        missing_projection.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::CoplanarPairMissingProjection
    );

    let disjoint_segment = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::Disjoint,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
        }],
    };
    assert_eq!(
        disjoint_segment.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::DisjointSegmentPlaneEvent
    );

    let failed_without_reason = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::ConstructionFailed,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        }],
    };
    assert_eq!(
        failed_without_reason.validate().unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::InvalidSegmentPlaneEvent
    );

    let failed_with_reason = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::ConstructionFailed,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: Some(
                hypermesh::exact::SegmentPlaneConstructionFailure::ParameterDivisionFailed,
            ),
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
        }],
    };
    failed_with_reason.validate().unwrap();

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
    let source_valid_graph = ExactIntersectionGraph {
        face_pairs: vec![FacePairEvents {
            left_face: 0,
            right_face: 0,
            relation: MeshFacePairRelation::Candidate,
            projection: None,
            events: vec![IntersectionEvent::SegmentPlane {
                segment_side: MeshSide::Left,
                edge: [0, 1],
                plane_side: MeshSide::Right,
                plane_face: 0,
                relation: SegmentPlaneRelation::ProperCrossing,
                point: Some(p3(1, 0, 0)),
                parameter: Some(half()),
                parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(1),
                    denominator: ExactReal::from(2),
                }),
                construction_failure: None,
                endpoint_sides: [Some(PlaneSide::Below), Some(PlaneSide::Above)],
            }],
        }],
    };
    source_valid_graph
        .validate_against_meshes(&left, &right)
        .unwrap();

    let mut bad_face = source_valid_graph.clone();
    bad_face.face_pairs[0].left_face = usize::MAX;
    assert_eq!(
        bad_face.validate_against_meshes(&left, &right).unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::FaceIndexOutOfRange
    );

    let mut bad_vertex = source_valid_graph.clone();
    if let IntersectionEvent::SegmentPlane { edge, .. } = &mut bad_vertex.face_pairs[0].events[0] {
        *edge = [0, usize::MAX];
    }
    assert_eq!(
        bad_vertex
            .validate_against_meshes(&left, &right)
            .unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::EventSourceOutOfRange
    );

    let mut relabeled_edge = source_valid_graph;
    if let IntersectionEvent::SegmentPlane { edge, .. } =
        &mut relabeled_edge.face_pairs[0].events[0]
    {
        *edge = [0, 0];
    }
    assert_eq!(
        relabeled_edge
            .validate_against_meshes(&left, &right)
            .unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::EventSourceMismatch
    );
}

#[test]
fn exact_graph_vertex_plan_retains_and_validates_source_side_facts() {
    let graph = ExactIntersectionGraph {
        face_pairs: vec![FacePairEvents {
            left_face: 0,
            right_face: 0,
            relation: MeshFacePairRelation::Candidate,
            projection: None,
            events: vec![IntersectionEvent::SegmentPlane {
                segment_side: MeshSide::Left,
                edge: [0, 1],
                plane_side: MeshSide::Right,
                plane_face: 0,
                relation: SegmentPlaneRelation::ProperCrossing,
                point: Some(p3(0, 0, 0)),
                parameter: Some(half()),
                parameter_ratio: Some(hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(1),
                    denominator: ExactReal::from(2),
                }),
                construction_failure: None,
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
            }],
        }],
    };

    graph.validate().unwrap();
    let vertex_plan = graph.checked_graph_vertex_plan().unwrap();

    assert_eq!(vertex_plan.source_use_count(), 1);
    assert!(vertex_plan.validate().is_valid());
    assert_eq!(vertex_plan.vertices[0].uses[0].parameter, half());
    assert_eq!(
        vertex_plan.vertices[0].uses[0].endpoint_sides,
        [Some(PlaneSide::Above), Some(PlaneSide::Below)]
    );
    assert_real_eq(
        &(&vertex_plan.vertices[0].uses[0].parameter_ratio.numerator
            / &vertex_plan.vertices[0].uses[0].parameter_ratio.denominator)
            .expect("graph vertex use ratio denominator should be nonzero"),
        &half(),
    );
}

#[test]
fn exact_graph_vertex_validation_rejects_unresolved_and_bad_source_facts() {
    let vertex_plan = ExactGraphVertexPlan {
        vertices: vec![
            ExactGraphVertex {
                point: p3(0, 0, 0),
                uses: Vec::new(),
            },
            ExactGraphVertex {
                point: p3(1, 0, 0),
                uses: vec![
                    ExactGraphVertexUse {
                        side: MeshSide::Left,
                        edge: [0, 1],
                        face_pair: [0, 0],
                        plane_face: 0,
                        parameter: half(),
                        parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                            numerator: ExactReal::from(1),
                            denominator: ExactReal::from(2),
                        },
                        endpoint_sides: [None, Some(PlaneSide::Below)],
                    },
                    ExactGraphVertexUse {
                        side: MeshSide::Right,
                        edge: [2, 3],
                        face_pair: [0, 0],
                        plane_face: 0,
                        parameter: half(),
                        parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                            numerator: ExactReal::from(2),
                            denominator: ExactReal::from(3),
                        },
                        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Above)],
                    },
                ],
            },
        ],
        unresolved_equalities: 1,
    };

    let report = vertex_plan.validate();

    assert!(!report.is_valid());
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnresolvedEquality)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::EmptyGraphVertexUses)
    );
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::NonCrossingEndpointSideFacts
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
    }));
}

#[test]
fn exact_mesh_face_pair_validation_rejects_inconsistent_scheduler_records() {
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
    let valid_candidate = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();

    let missing_triangle = hypermesh::exact::MeshFacePairClassification {
        left_face: 0,
        right_face: 0,
        bounds: valid_candidate.bounds,
        triangle: None,
        relation: MeshFacePairRelation::Candidate,
    };
    assert_eq!(
        missing_triangle.validate().unwrap_err(),
        hypermesh::exact::MeshFacePairValidationError::MissingTriangleClassification
    );

    let mut candidate = valid_candidate.triangle.unwrap();
    candidate.left_edge_events.clear();
    let bad_candidate = hypermesh::exact::MeshFacePairClassification {
        left_face: 0,
        right_face: 0,
        bounds: valid_candidate.bounds,
        triangle: Some(candidate),
        relation: MeshFacePairRelation::Candidate,
    };
    assert_eq!(
        bad_candidate.validate().unwrap_err(),
        hypermesh::exact::MeshFacePairValidationError::CandidateMissingEdgeEvents
    );
}

#[test]
fn exact_intersection_graph_records_noncoplanar_split_events() {
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

    let graph = build_intersection_graph(&left, &right).unwrap();

    graph.validate().unwrap();
    graph.validate_against_sources(&left, &right).unwrap();
    graph.face_pairs[0]
        .validate_against_sources(&left, &right)
        .unwrap();
    let mut stale_graph = graph.clone();
    stale_graph.face_pairs[0].right_face = usize::MAX;
    assert_eq!(
        stale_graph
            .validate_against_sources(&left, &right)
            .unwrap_err(),
        hypermesh::exact::IntersectionGraphValidationError::FaceIndexOutOfRange
    );
    assert_eq!(graph.face_pairs.len(), 1);
    assert!(!graph.has_unknowns());
    assert!(graph.face_pairs[0].events.iter().any(|event| matches!(
        event,
        IntersectionEvent::SegmentPlane {
            relation: SegmentPlaneRelation::ProperCrossing,
            point: Some(_),
            parameter: Some(_),
            endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)]
                | [Some(PlaneSide::Below), Some(PlaneSide::Above)],
            ..
        }
    )));

    let split_plan = graph.edge_split_plan();
    assert_eq!(split_plan.unknown_orderings, 0);
    assert!(split_plan.validate().is_valid());
    assert!(
        split_plan
            .validate_against_sources(&left, &right)
            .is_valid()
    );
    assert!(split_plan.point_count() >= 2);
    assert!(split_plan.splits.iter().all(|split| {
        split.points.iter().all(|point| {
            real_between_unit(&point.parameter)
                && matches!(
                    point.endpoint_sides,
                    [Some(PlaneSide::Above), Some(PlaneSide::Below)]
                        | [Some(PlaneSide::Below), Some(PlaneSide::Above)]
                )
        })
    }));
    let mut relabeled_split_plan = split_plan.clone();
    relabeled_split_plan.splits.reverse();
    let relabeled_split_report = relabeled_split_plan.validate_against_sources(&left, &right);
    if relabeled_split_plan != split_plan {
        assert_eq!(
            relabeled_split_report
                .diagnostics
                .first()
                .map(|diagnostic| diagnostic.kind),
            Some(hypermesh::exact::SplitPlanDiagnosticKind::SourceReplayMismatch)
        );
        assert!(relabeled_split_report.validate().is_ok());
    }

    let vertex_plan = graph.graph_vertex_plan();
    assert_eq!(vertex_plan.unresolved_equalities, 0);
    assert!(vertex_plan.validate().is_valid());
    assert!(
        vertex_plan
            .validate_against_sources(&left, &right)
            .is_valid()
    );
    assert!(vertex_plan.vertices.len() <= split_plan.point_count());
    assert!(
        vertex_plan
            .vertices
            .iter()
            .all(|vertex| !vertex.uses.is_empty())
    );
    let mut relabeled_vertex_plan = vertex_plan.clone();
    relabeled_vertex_plan.vertices.reverse();
    let relabeled_vertex_report = relabeled_vertex_plan.validate_against_sources(&left, &right);
    if relabeled_vertex_plan != vertex_plan {
        assert_eq!(
            relabeled_vertex_report
                .diagnostics
                .first()
                .map(|diagnostic| diagnostic.kind),
            Some(hypermesh::exact::SplitPlanDiagnosticKind::SourceReplayMismatch)
        );
        assert!(relabeled_vertex_report.validate().is_ok());
    }

    let topology_plan = graph.split_topology_plan();
    let checked_topology_plan = graph.checked_split_topology_plan().unwrap();
    assert_eq!(checked_topology_plan, topology_plan);
    assert_eq!(topology_plan.unresolved_equalities, 0);
    assert_eq!(topology_plan.unresolved_vertex_lookups, 0);
    assert_eq!(topology_plan.unknown_orderings, 0);
    assert!(topology_plan.validate().is_valid());
    assert!(!topology_plan.edge_chains.is_empty());
    assert_eq!(
        topology_plan.referenced_graph_vertices(),
        split_plan.point_count()
    );
    assert!(
        topology_plan
            .validate_against_sources(&left, &right)
            .is_valid()
    );
    let mut relabeled_topology_plan = topology_plan.clone();
    relabeled_topology_plan.edge_chains.reverse();
    let relabeled_topology_report = relabeled_topology_plan.validate_against_sources(&left, &right);
    if relabeled_topology_plan != topology_plan {
        assert_eq!(
            relabeled_topology_report
                .diagnostics
                .first()
                .map(|diagnostic| diagnostic.kind),
            Some(hypermesh::exact::SplitPlanDiagnosticKind::SourceReplayMismatch)
        );
        assert!(relabeled_topology_report.validate().is_ok());
    }
    assert!(
        topology_plan
            .edge_chains
            .iter()
            .all(|chain| chain.nodes.len() >= 3)
    );

    let face_plan = graph.face_split_plan();
    let checked_face_plan = graph.checked_face_split_plan().unwrap();
    assert_eq!(checked_face_plan, face_plan);
    assert!(!face_plan.faces.is_empty());
    assert!(face_plan.graph_vertex_references() >= topology_plan.referenced_graph_vertices());
    assert!(face_plan.faces.iter().all(|face| !face.edges.is_empty()));
    assert!(
        face_plan
            .validate_against_topology(&topology_plan)
            .is_valid()
    );
    assert!(face_plan.validate_against_sources(&left, &right).is_valid());
    let mut relabeled_face_plan = face_plan.clone();
    relabeled_face_plan.faces.reverse();
    let relabeled_face_report = relabeled_face_plan.validate_against_sources(&left, &right);
    if relabeled_face_plan != face_plan {
        assert_eq!(
            relabeled_face_report
                .diagnostics
                .first()
                .map(|diagnostic| diagnostic.kind),
            Some(hypermesh::exact::SplitPlanDiagnosticKind::SourceReplayMismatch)
        );
        assert!(relabeled_face_report.validate().is_ok());
    }

    let geometry_plan = graph.face_split_geometry_plan(&left, &right).unwrap();
    assert_eq!(geometry_plan.faces.len(), face_plan.faces.len());
    assert_eq!(
        geometry_plan.graph_vertex_references(),
        face_plan.graph_vertex_references()
    );
    assert!(geometry_plan.faces.iter().all(|face| {
        !face.boundary_chains.is_empty()
            && face.boundary_chains.iter().all(|chain| {
                chain.nodes.len() >= 3
                    && matches!(
                        chain.nodes.first(),
                        Some(FaceSplitBoundaryNode::OriginalVertex { .. })
                    )
                    && matches!(
                        chain.nodes.last(),
                        Some(FaceSplitBoundaryNode::OriginalVertex { .. })
                    )
                    && chain
                        .nodes
                        .iter()
                        .any(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
            })
    }));
    assert!(
        geometry_plan
            .validate_boundary_incidence(&left, &right)
            .is_valid()
    );
    assert!(
        geometry_plan
            .validate_against_sources(&left, &right)
            .is_valid()
    );
    let mut relabeled_geometry_plan = geometry_plan.clone();
    relabeled_geometry_plan.faces[0].triangle.swap(0, 1);
    let relabeled_geometry_report = relabeled_geometry_plan.validate_against_sources(&left, &right);
    assert_eq!(
        relabeled_geometry_report
            .diagnostics
            .first()
            .map(|diagnostic| diagnostic.kind),
        Some(hypermesh::exact::SplitPlanDiagnosticKind::SourceReplayMismatch)
    );
    assert!(relabeled_geometry_report.validate().is_ok());

    let region_plan = geometry_plan.region_plan(&left, &right);
    assert_eq!(region_plan.regions.len(), geometry_plan.faces.len());
    assert_eq!(
        region_plan.graph_vertex_references(),
        geometry_plan.graph_vertex_references()
    );
    assert!(region_plan.validate(&left, &right).is_valid());
    assert!(
        region_plan
            .validate_against_sources(&left, &right)
            .is_valid()
    );
    let mut stale_region_plan = region_plan.clone();
    stale_region_plan.regions[0].face = usize::MAX;
    let stale_region_report = stale_region_plan.validate_against_sources(&left, &right);
    assert!(!stale_region_report.is_valid());
    assert!(stale_region_report.validate().is_ok());
    let mut relabeled_region_plan = region_plan.clone();
    relabeled_region_plan.regions[0].triangle.swap(0, 1);
    let relabeled_region_report = relabeled_region_plan.validate_against_sources(&left, &right);
    assert_eq!(
        relabeled_region_report
            .diagnostics
            .first()
            .map(|diagnostic| diagnostic.kind),
        Some(hypermesh::exact::SplitPlanDiagnosticKind::SourceReplayMismatch)
    );
    assert!(relabeled_region_report.validate().is_ok());
    assert!(region_plan.regions.iter().all(|region| {
        region.boundary.len() >= 4
            && region
                .boundary
                .iter()
                .any(|node| matches!(node, FaceSplitBoundaryNode::GraphVertex { .. }))
    }));

    #[cfg(feature = "exact-triangulation")]
    let region_classifications =
        checked_classify_face_regions_against_opposite_planes(&region_plan, &left, &right).unwrap();
    #[cfg(not(feature = "exact-triangulation"))]
    let region_classifications =
        classify_face_regions_against_opposite_planes(&region_plan, &left, &right);
    assert_eq!(
        region_classifications.len(),
        region_plan.regions.len() * right.triangles().len()
    );
    assert!(
        region_classifications
            .iter()
            .all(|classification| classification.all_proof_producing()
                && classification.validate().is_ok())
    );
    #[cfg(feature = "exact-triangulation")]
    {
        let first_classification = region_classifications
            .first()
            .expect("intersecting triangles produce at least one region classification");
        first_classification
            .validate_against_sources(&left, &right)
            .unwrap();
        let mut stale_classification = first_classification.clone();
        stale_classification.plane_face = usize::MAX;
        assert_eq!(
            stale_classification
                .validate_against_sources(&left, &right)
                .unwrap_err(),
            hypermesh::exact::FaceRegionPlaneValidationError::SourceReplayMismatch
        );
        assert_eq!(
            first_classification
                .validate_against_sources(&right, &left)
                .unwrap_err(),
            hypermesh::exact::FaceRegionPlaneValidationError::SourceReplayMismatch
        );
    }
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_face_region_plane_validation_rejects_inconsistent_artifacts() {
    let same_side = hypermesh::exact::FaceRegionPlaneClassification {
        region_side: MeshSide::Left,
        region_face: 0,
        plane_side: MeshSide::Left,
        plane_face: 0,
        relation: hypermesh::exact::FaceRegionPlaneRelation::Coplanar,
        node_sides: vec![Some(PlaneSide::On), Some(PlaneSide::On)],
        predicates: vec![
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::ExactRealFact,
            ),
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::ExactRealFact,
            ),
        ],
    };
    assert_eq!(
        same_side.validate().unwrap_err(),
        hypermesh::exact::FaceRegionPlaneValidationError::SameRegionAndPlaneSide {
            region_side: MeshSide::Left,
            plane_side: MeshSide::Left,
        }
    );

    let empty = hypermesh::exact::FaceRegionPlaneClassification {
        region_side: MeshSide::Left,
        region_face: 0,
        plane_side: MeshSide::Right,
        plane_face: 0,
        relation: hypermesh::exact::FaceRegionPlaneRelation::Coplanar,
        node_sides: Vec::new(),
        predicates: Vec::new(),
    };
    assert_eq!(
        empty.validate().unwrap_err(),
        hypermesh::exact::FaceRegionPlaneValidationError::EmptyNodeSides
    );

    let mut mismatched = hypermesh::exact::FaceRegionPlaneClassification {
        region_side: MeshSide::Left,
        region_face: 0,
        plane_side: MeshSide::Right,
        plane_face: 0,
        relation: hypermesh::exact::FaceRegionPlaneRelation::StrictlyAbove,
        node_sides: vec![Some(PlaneSide::Above), Some(PlaneSide::Below)],
        predicates: vec![
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::ExactRealFact,
            ),
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::ExactRealFact,
            ),
        ],
    };
    assert_eq!(
        mismatched.validate().unwrap_err(),
        hypermesh::exact::FaceRegionPlaneValidationError::RelationMismatch {
            expected: hypermesh::exact::FaceRegionPlaneRelation::Straddling,
            actual: hypermesh::exact::FaceRegionPlaneRelation::StrictlyAbove,
        }
    );

    mismatched.relation = hypermesh::exact::FaceRegionPlaneRelation::Straddling;
    mismatched.predicates.pop();
    assert_eq!(
        mismatched.validate().unwrap_err(),
        hypermesh::exact::FaceRegionPlaneValidationError::PredicateCountMismatch {
            expected: 2,
            actual: 1,
        }
    );

    let undecided = hypermesh::exact::FaceRegionPlaneClassification {
        region_side: MeshSide::Left,
        region_face: 0,
        plane_side: MeshSide::Right,
        plane_face: 0,
        relation: hypermesh::exact::FaceRegionPlaneRelation::Unknown,
        node_sides: vec![None, Some(PlaneSide::Above)],
        predicates: vec![
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::Unknown,
            ),
            hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::ExactRealFact,
            ),
        ],
    };
    undecided.validate().unwrap();
    assert!(!undecided.is_decided_and_proof_producing());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_face_region_triangulates_through_feature_gated_hypertri() {
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
    let graph = build_intersection_graph(&left, &right).unwrap();
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
    let region_plan = geometry.region_plan(&left, &right);

    let triangulations =
        hypermesh::exact::checked_triangulate_face_regions_with_earcut(&region_plan, &left, &right)
            .unwrap();

    assert_eq!(triangulations.len(), region_plan.regions.len());
    assert!(triangulations.iter().all(|triangulation| {
        triangulation.triangles.len() % 3 == 0
            && triangulation
                .triangles
                .iter()
                .all(|&index| index < triangulation.vertices.len())
    }));
    triangulations[0]
        .validate_against_sources(&left, &right)
        .unwrap();
    let mut stale_triangulation = triangulations[0].clone();
    stale_triangulation.face = usize::MAX;
    assert!(
        stale_triangulation
            .validate_against_sources(&left, &right)
            .is_err()
    );
    assert!(
        triangulations[0]
            .validate_against_sources(&right, &left)
            .is_err()
    );

    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepAll,
    )
    .unwrap();
    assembly
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactRegionSelection::KeepAll,
        )
        .unwrap();
    assert!(
        assembly
            .validate_against_sources(
                &left,
                &right,
                hypermesh::exact::ExactRegionSelection::KeepLeft,
            )
            .is_err()
    );
    let mut stale_assembly = assembly.clone();
    stale_assembly.triangles[0].source_face = usize::MAX;
    assert!(
        stale_assembly
            .validate_against_sources(
                &left,
                &right,
                hypermesh::exact::ExactRegionSelection::KeepAll,
            )
            .is_err()
    );

    assert!(!assembly.vertices.is_empty());
    assert!(!assembly.triangles.is_empty());
    assert!(assembly.validate().is_ok());
    assert!(assembly.triangles.iter().all(|triangle| {
        triangle
            .vertices
            .iter()
            .all(|&vertex| vertex < assembly.vertices.len())
    }));

    let left_only = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepLeft,
    )
    .unwrap();
    let right_only = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepRight,
    )
    .unwrap();
    assert_eq!(
        left_only.triangles.len() + right_only.triangles.len(),
        assembly.triangles.len()
    );
    assert!(
        left_only
            .triangles
            .iter()
            .all(|triangle| triangle.source_side == MeshSide::Left)
    );
    assert!(
        right_only
            .triangles
            .iter()
            .all(|triangle| triangle.source_side == MeshSide::Right)
    );
    assembly
        .validate_source_face_incidence(&left, &right)
        .unwrap();

    let output = assembly
        .checked_to_exact_mesh_with_sources(&left, &right, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap();
    assert_eq!(output.vertices().len(), assembly.vertices.len());
    assert_eq!(output.triangles().len(), assembly.triangles.len());
    assert_eq!(
        output.provenance().source.label,
        "exact boolean assembly plan"
    );

    let pipelined = hypermesh::exact::build_selected_region_mesh(
        &left,
        &right,
        hypermesh::exact::ExactRegionSelection::KeepAll,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(pipelined.triangles().len(), output.triangles().len());
    assert_eq!(pipelined.vertices().len(), output.vertices().len());
    pipelined.validate_retained_state().unwrap();

    let boolean = hypermesh::exact::boolean_selected_regions(
        &left,
        &right,
        hypermesh::exact::ExactBooleanPolicy::KEEP_ALL_BOUNDARY,
    )
    .unwrap();
    boolean.validate().unwrap();
    boolean.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        boolean.kind,
        hypermesh::exact::ExactBooleanResultKind::SelectedRegions {
            selection: hypermesh::exact::ExactRegionSelection::KeepAll
        }
    );
    assert!(!boolean.graph_had_unknowns);
    assert_eq!(boolean.mesh.triangles().len(), output.triangles().len());
    assert_eq!(
        boolean.region_classifications.len(),
        region_plan.regions.len() * right.triangles().len()
    );

    let exact = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::SelectedRegions(
            hypermesh::exact::ExactRegionSelection::KeepAll,
        ),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    exact.validate().unwrap();
    exact.validate_against_sources(&left, &right).unwrap();
    assert_eq!(exact.mesh.triangles().len(), output.triangles().len());

    assert_eq!(
        boolean.validate_against_sources(&right, &left).unwrap_err(),
        hypermesh::exact::ExactReportValidationError::OutputSourceReplayMismatch
    );

    let mut bad_result = boolean.clone();
    bad_result.kind = hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
        shortcut: hypermesh::exact::ExactBooleanShortcutKind::BoundsDisjoint,
    };
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::ShortcutResultHasAssemblyArtifacts
    );

    let mut bad_result = boolean.clone();
    bad_result.kind = hypermesh::exact::ExactBooleanResultKind::SelectedRegions {
        selection: hypermesh::exact::ExactRegionSelection::KeepLeft,
    };
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SelectedRegionAssemblyViolatesSelection
    );

    let mut bad_result = boolean.clone();
    bad_result.graph_had_unknowns = true;
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SelectedRegionResultHasUnknownGraph
    );

    let mut bad_result = boolean.clone();
    bad_result.region_classifications.clear();
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::MissingRegionFacts
    );

    let mut bad_result = boolean.clone();
    bad_result.region_classifications[0].relation =
        hypermesh::exact::FaceRegionPlaneRelation::Unknown;
    bad_result.region_classifications[0].node_sides.fill(None);
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::RegionClassificationNotProofProducing
    );

    let mut bad_result = boolean.clone();
    bad_result
        .region_classifications
        .push(bad_result.region_classifications[0].clone());
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::DuplicateRegionClassification
    );

    let mut bad_result = boolean.clone();
    bad_result.triangulations.clear();
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::MissingRegionFacts
    );

    let mut bad_result = boolean.clone();
    bad_result
        .triangulations
        .push(bad_result.triangulations[0].clone());
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::DuplicateRegionTriangulation
    );

    let mut bad_result = boolean.clone();
    bad_result.triangulations[0].face = usize::MAX;
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::UnclassifiedRegionTriangulation
    );

    let mut bad_result = boolean.clone();
    let mut orphaned = bad_result.region_classifications[0].clone();
    orphaned.region_face = usize::MAX;
    bad_result.region_classifications.push(orphaned);
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::OrphanedRegionClassification
    );

    let mut bad_result = boolean.clone();
    bad_result.assembly.triangles[0].source_face = usize::MAX;
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::UntriangulatedAssemblyRegion
    );

    let mut bad_result = boolean.clone();
    let source_vertex = bad_result.assembly.triangles[0].vertices[0];
    let point = Point3::new(
        ExactReal::from(99),
        ExactReal::from(98),
        ExactReal::from(97),
    );
    bad_result.assembly.vertices[source_vertex].point = point.clone();
    bad_result.assembly.vertices[source_vertex].source = FaceSplitBoundaryNode::OriginalVertex {
        vertex: usize::MAX,
        point,
    };
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::AssemblyVertexOutsideTriangulation
    );

    let mut bad_result = boolean.clone();
    bad_result
        .assembly
        .vertices
        .push(bad_result.assembly.vertices[0].clone());
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::UnreferencedAssemblyVertex
    );

    let mut bad_result = boolean.clone();
    bad_result.assembly.vertices[0].point = p3(99, 0, 0);
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidAssembly
    );

    let mut bad_result = boolean.clone();
    let mut mesh_vertices = bad_result.mesh.vertices().to_vec();
    mesh_vertices[0] = ExactPoint3::new(Real::from(99), Real::from(0), Real::from(0));
    bad_result.mesh = ExactMesh::new_with_policy(
        mesh_vertices,
        bad_result.mesh.triangles().to_vec(),
        SourceProvenance::exact("adversarial selected-region mesh vertex payload"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::OutputMeshAssemblyMismatch
    );

    let mut bad_result = boolean.clone();
    let mut mesh_triangles = bad_result.mesh.triangles().to_vec();
    mesh_triangles[0].0.swap(1, 2);
    bad_result.mesh = ExactMesh::new_with_policy(
        bad_result.mesh.vertices().to_vec(),
        mesh_triangles,
        SourceProvenance::exact("adversarial selected-region mesh payload"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        bad_result.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::OutputMeshAssemblyMismatch
    );

    let unsupported = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap_err();
    assert!(
        unsupported
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::UnsupportedExactOperation)
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresCertifiedWinding
    );
    assert!(!preflight.graph_had_unknowns);
    assert!(preflight.retained_face_pairs > 0);
    assert!(preflight.retained_events > 0);
    assert_eq!(preflight.region_count, region_plan.regions.len());
    assert_eq!(
        preflight.region_classifications.len(),
        region_plan.regions.len() * right.triangles().len()
    );
    assert!(
        preflight
            .region_classifications
            .iter()
            .all(|classification| classification.all_proof_producing())
    );
    let blocker = preflight.blocker.as_ref().unwrap();
    assert_eq!(
        blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding
    );
    assert!(blocker.candidate_pairs > 0);
    blocker.validate_against_sources(&left, &right).unwrap();
    let mut stale_blocker = blocker.clone();
    stale_blocker.candidate_pairs += 1;
    assert_eq!(
        stale_blocker
            .validate_against_sources(&left, &right)
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SourceReplayMismatch
    );

    let selected_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::SelectedRegions(
            hypermesh::exact::ExactRegionSelection::KeepAll,
        ),
    )
    .unwrap();
    selected_preflight.validate().unwrap();
    selected_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        selected_preflight
            .validate_against_sources(&right, &left)
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SourceReplayMismatch
    );
    assert_eq!(
        selected_preflight.support,
        hypermesh::exact::ExactBooleanSupport::SelectedRegionPolicy
    );
    assert!(selected_preflight.blocker.is_none());
    assert_eq!(selected_preflight.region_count, region_plan.regions.len());
    assert_eq!(
        selected_preflight.region_classifications.len(),
        preflight.region_classifications.len()
    );
    let mut blocked_selected_preflight = selected_preflight.clone();
    blocked_selected_preflight.blocker = Some(hypermesh::exact::ExactBooleanBlocker {
        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
        candidate_pairs: 1,
        coplanar_overlapping_pairs: 0,
        coplanar_touching_pairs: 0,
        unknown_pairs: 0,
        construction_failed_events: 0,
    });
    assert_eq!(
        blocked_selected_preflight.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );
    let mut mismatched_region_count = selected_preflight.clone();
    mismatched_region_count.region_count += 1;
    assert_eq!(
        mismatched_region_count.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::RegionCountMismatch
    );
    let mut duplicated_region_classification = selected_preflight.clone();
    duplicated_region_classification
        .region_classifications
        .push(duplicated_region_classification.region_classifications[0].clone());
    assert_eq!(
        duplicated_region_classification.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::DuplicateRegionClassification
    );
    let mut event_without_pair_selected_preflight = selected_preflight.clone();
    event_without_pair_selected_preflight.retained_face_pairs = 0;
    event_without_pair_selected_preflight.retained_events = 1;
    event_without_pair_selected_preflight.region_count = 0;
    event_without_pair_selected_preflight
        .region_classifications
        .clear();
    assert_eq!(
        event_without_pair_selected_preflight
            .validate()
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );
    let mut undecided_selected_preflight = selected_preflight;
    undecided_selected_preflight.region_classifications[0].relation =
        hypermesh::exact::FaceRegionPlaneRelation::Unknown;
    undecided_selected_preflight.region_classifications[0]
        .node_sides
        .fill(None);
    assert_eq!(
        undecided_selected_preflight.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::RegionClassificationNotProofProducing
    );

    let refinement_report = hypermesh::exact::certify_refinement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    refinement_report.validate().unwrap();
    refinement_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        refinement_report.status,
        hypermesh::exact::ExactRefinementStatus::NotRequired
    );
    assert!(!refinement_report.graph_had_unknowns);
    assert!(refinement_report.blocker.is_none());

    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    winding_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert!(winding_report.is_ready());
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::Ready
    );
    assert!(!winding_report.graph_had_unknowns);
    assert_eq!(winding_report.region_count, region_plan.regions.len());
    assert_eq!(
        winding_report.region_classifications.len(),
        preflight.region_classifications.len()
    );
    assert!(winding_report.all_proof_producing());
    assert_eq!(
        winding_report.blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding
    );
    let mut mismatched_winding_region_count = winding_report.clone();
    mismatched_winding_region_count.region_count += 1;
    assert_eq!(
        mismatched_winding_region_count.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::RegionCountMismatch
    );
    let mut duplicated_winding_classification = winding_report.clone();
    duplicated_winding_classification
        .region_classifications
        .push(duplicated_winding_classification.region_classifications[0].clone());
    assert_eq!(
        duplicated_winding_classification.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::DuplicateRegionClassification
    );
    winding_report
        .blocker
        .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding)
        .unwrap();
    let bad_winding_blocker = hypermesh::exact::ExactBooleanBlocker {
        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
        candidate_pairs: 0,
        coplanar_overlapping_pairs: 1,
        coplanar_touching_pairs: 0,
        unknown_pairs: 0,
        construction_failed_events: 0,
    };
    assert_eq!(
        bad_winding_blocker
            .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding),
        Err(hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts)
    );
    let construction_failure_blocker = hypermesh::exact::ExactBooleanBlocker {
        kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
        candidate_pairs: 1,
        coplanar_overlapping_pairs: 0,
        coplanar_touching_pairs: 0,
        unknown_pairs: 0,
        construction_failed_events: 1,
    };
    construction_failure_blocker
        .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement)
        .unwrap();
    assert_eq!(
        construction_failure_blocker
            .validate_for_kind(hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding),
        Err(hypermesh::exact::ExactReportValidationError::WrongBlockerKind)
    );
    let required_refinement = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: Some(construction_failure_blocker.clone()),
    };
    required_refinement.validate().unwrap();
    let missing_refinement_blocker = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: None,
    };
    assert_eq!(
        missing_refinement_blocker.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::MissingBlocker
    );
    let retained_event_count_mismatch = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 0,
        blocker: Some(construction_failure_blocker.clone()),
    };
    assert_eq!(
        retained_event_count_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );
    let retained_pair_count_mismatch = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::Required,
        graph_had_unknowns: true,
        retained_face_pairs: 0,
        retained_events: 1,
        blocker: Some(hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 1,
            construction_failed_events: 0,
        }),
    };
    assert_eq!(
        retained_pair_count_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );
    let retained_pair_without_relation_evidence = hypermesh::exact::ExactRefinementReport {
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
        retained_pair_without_relation_evidence
            .validate()
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );
    let not_required_refinement_with_orphan_event = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::NotRequired,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 1,
        blocker: None,
    };
    assert_eq!(
        not_required_refinement_with_orphan_event
            .validate()
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );
    let not_required_refinement_with_empty_pair = hypermesh::exact::ExactRefinementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactRefinementStatus::NotRequired,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 0,
        blocker: None,
    };
    assert_eq!(
        not_required_refinement_with_empty_pair
            .validate()
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let certified_selected_preflight = hypermesh::exact::ExactBooleanPreflight {
        operation: hypermesh::exact::ExactBooleanOperation::SelectedRegions(
            hypermesh::exact::ExactRegionSelection::KeepAll,
        ),
        support: hypermesh::exact::ExactBooleanSupport::CertifiedBoundsDisjoint,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: None,
        arrangement_readiness: None,
    };
    assert_eq!(
        certified_selected_preflight.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let certified_with_graph_evidence = hypermesh::exact::ExactBooleanPreflight {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        support: hypermesh::exact::ExactBooleanSupport::CertifiedBoundsDisjoint,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: None,
        arrangement_readiness: None,
    };
    assert_eq!(
        certified_with_graph_evidence.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let selected_policy_for_named_operation = hypermesh::exact::ExactBooleanPreflight {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        support: hypermesh::exact::ExactBooleanSupport::SelectedRegionPolicy,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: None,
        arrangement_readiness: None,
    };
    assert_eq!(
        selected_policy_for_named_operation.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let winding_preflight_without_pairs = hypermesh::exact::ExactBooleanPreflight {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        support: hypermesh::exact::ExactBooleanSupport::RequiresCertifiedWinding,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 1,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: Some(hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        }),
        arrangement_readiness: None,
    };
    assert_eq!(
        winding_preflight_without_pairs.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_region_triangulation_rejects_projected_source_drift() {
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
    let graph = build_intersection_graph(&left, &right).unwrap();
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
    let region_plan = geometry.region_plan(&left, &right);
    let mut triangulations =
        hypermesh::exact::checked_triangulate_face_regions_with_earcut(&region_plan, &left, &right)
            .unwrap();

    triangulations[0].vertices[0] = hypertri::ExactPoint::new(Real::from(99), Real::from(99));

    let error = triangulations[0].validate().unwrap_err();
    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));

    let assembly_error = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations(
        &triangulations,
        hypermesh::exact::ExactRegionSelection::KeepAll,
    )
    .unwrap_err();
    assert!(matches!(
        assembly_error,
        hypertri::Error::InvalidInput { .. }
    ));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_region_triangulation_rejects_exactly_collinear_output_triangle() {
    let triangulation = hypermesh::exact::FaceRegionTriangulation {
        side: MeshSide::Left,
        face: 0,
        projection: CoplanarProjection::Xy,
        boundary: vec![
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: p3(0, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 1,
                point: p3(1, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 2,
                point: p3(2, 0, 0),
            },
        ],
        vertices: vec![
            hypertri::ExactPoint::new(Real::from(0), Real::from(0)),
            hypertri::ExactPoint::new(Real::from(1), Real::from(0)),
            hypertri::ExactPoint::new(Real::from(2), Real::from(0)),
        ],
        triangles: vec![0, 1, 2],
    };

    let error = triangulation.validate().unwrap_err();
    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_region_triangulation_accepts_face_interior_steiner_witnesses() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let triangulation = hypermesh::exact::FaceRegionTriangulation {
        side: MeshSide::Left,
        face: 0,
        projection: CoplanarProjection::Xy,
        boundary: vec![
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: p3(0, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 1,
                point: p3(4, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 2,
                point: p3(0, 4, 0),
            },
            FaceSplitBoundaryNode::FaceInterior { point: p3(1, 1, 0) },
        ],
        vertices: vec![
            hypertri::ExactPoint::new(Real::from(0), Real::from(0)),
            hypertri::ExactPoint::new(Real::from(4), Real::from(0)),
            hypertri::ExactPoint::new(Real::from(0), Real::from(4)),
            hypertri::ExactPoint::new(Real::from(1), Real::from(1)),
        ],
        triangles: vec![0, 1, 3, 0, 3, 2],
    };

    triangulation.validate().unwrap();
    let assembly =
        hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
            std::slice::from_ref(&triangulation),
            hypermesh::exact::ExactRegionSelection::KeepAll,
            &mesh,
            &mesh,
        )
        .unwrap();
    assembly
        .validate_source_face_incidence(&mesh, &mesh)
        .unwrap();
    let materialized = assembly
        .checked_to_exact_mesh_with_sources(&mesh, &mesh, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap();
    assert_eq!(materialized.vertices().len(), 4);
    assert_eq!(materialized.triangles().len(), 2);

    let mut off_plane = triangulation;
    off_plane.boundary[3] = FaceSplitBoundaryNode::FaceInterior { point: p3(1, 1, 1) };
    off_plane.validate().unwrap();
    let bad = hypermesh::exact::ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        std::slice::from_ref(&off_plane),
        hypermesh::exact::ExactRegionSelection::KeepAll,
        &mesh,
        &mesh,
    )
    .unwrap();
    assert!(bad.validate_source_face_incidence(&mesh, &mesh).is_err());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_volumetric_classification_retries_boundary_centroid_representative() {
    let target = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 12],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let triangulation = hypermesh::exact::FaceRegionTriangulation {
        side: MeshSide::Left,
        face: 0,
        projection: CoplanarProjection::Xy,
        boundary: vec![
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: p3(2, 1, 1),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 1,
                point: p3(14, 1, 1),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 2,
                point: p3(1, 14, 1),
            },
        ],
        vertices: vec![
            hypertri::ExactPoint::new(Real::from(2), Real::from(1)),
            hypertri::ExactPoint::new(Real::from(14), Real::from(1)),
            hypertri::ExactPoint::new(Real::from(1), Real::from(14)),
        ],
        triangles: vec![0, 1, 2],
    };

    let centroid = Point3::new(rational(17, 3), rational(16, 3), ExactReal::from(1));
    let centroid_report =
        hypermesh::exact::classify_point_against_closed_mesh_winding_report(&centroid, &target);
    assert_eq!(
        centroid_report.relation,
        hypermesh::exact::ClosedMeshWindingRelation::Boundary
    );
    centroid_report
        .validate_against_sources(&centroid, &target)
        .unwrap();

    let classification =
        hypermesh::exact::classify_triangulated_region_triangle_against_closed_mesh(
            &triangulation,
            [0, 1, 2],
            &target,
        )
        .unwrap();
    assert_eq!(
        classification.relation,
        hypermesh::exact::ExactVolumetricRegionRelation::Inside
    );
    assert_real_eq(&classification.representative.x, &rational(19, 4));
    assert_real_eq(&classification.representative.y, &rational(17, 4));
    assert_real_eq(&classification.representative.z, &ExactReal::from(1));
    classification
        .validate_against_sources(&triangulation, &target)
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_checked_assembly_materialization_rejects_invalid_triangle_indices() {
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![hypermesh::exact::ExactOutputVertex {
            point: p3(0, 0, 0),
            source: FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: p3(0, 0, 0),
            },
        }],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 0],
            source_side: MeshSide::Left,
            source_face: 0,
            orientation: hypermesh::exact::ExactOutputTriangleOrientation::PreserveSource,
        }],
    };

    let error = assembly
        .checked_to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap_err();

    assert!(
        error
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_validation_rejects_output_vertex_source_mismatch() {
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(1, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(1, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(1, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 1, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 1, 0),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 2],
            source_side: MeshSide::Left,
            source_face: 0,
            orientation: hypermesh::exact::ExactOutputTriangleOrientation::PreserveSource,
        }],
    };

    let error = assembly.validate().unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
    let materialization_error = assembly
        .to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap_err();
    assert!(
        materialization_error
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_validation_rejects_unreferenced_output_vertex() {
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(1, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(1, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 1, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 1, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(2, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 3,
                    point: p3(2, 0, 0),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 2],
            source_side: MeshSide::Left,
            source_face: 0,
            orientation: hypermesh::exact::ExactOutputTriangleOrientation::PreserveSource,
        }],
    };

    let error = assembly.validate().unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
    let materialization_error = assembly
        .to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap_err();
    assert!(
        materialization_error
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_validation_rejects_distinct_handles_with_same_point() {
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 1, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 1, 0),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 2],
            source_side: MeshSide::Left,
            source_face: 0,
            orientation: hypermesh::exact::ExactOutputTriangleOrientation::PreserveSource,
        }],
    };

    let error = assembly.validate().unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_source_face_incidence_rejects_off_plane_output() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(1, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(1, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 1, 1),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 1, 1),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 1, 2],
            source_side: MeshSide::Left,
            source_face: 0,
            orientation: hypermesh::exact::ExactOutputTriangleOrientation::PreserveSource,
        }],
    };

    let error = assembly
        .validate_source_face_incidence(&mesh, &mesh)
        .unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
    let materialize_error = assembly
        .checked_to_exact_mesh_with_sources(&mesh, &mesh, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap_err();
    assert!(
        materialize_error
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_assembly_source_face_incidence_rejects_reversed_output_orientation() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let assembly = hypermesh::exact::ExactBooleanAssemblyPlan {
        vertices: vec![
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(2, 0, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 1,
                    point: p3(2, 0, 0),
                },
            },
            hypermesh::exact::ExactOutputVertex {
                point: p3(0, 2, 0),
                source: FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 2,
                    point: p3(0, 2, 0),
                },
            },
        ],
        triangles: vec![hypermesh::exact::ExactOutputTriangle {
            vertices: [0, 2, 1],
            source_side: MeshSide::Left,
            source_face: 0,
            orientation: hypermesh::exact::ExactOutputTriangleOrientation::PreserveSource,
        }],
    };

    assembly.validate().unwrap();
    let error = assembly
        .validate_source_face_incidence(&mesh, &mesh)
        .unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
    let materialize_error = assembly
        .checked_to_exact_mesh_with_sources(&mesh, &mesh, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap_err();
    assert!(
        materialize_error
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_checked_region_triangulation_rejects_invalid_region_before_earcut() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let region_plan = hypermesh::exact::ExactFaceRegionPlan {
        regions: vec![FaceRegionBoundary {
            side: MeshSide::Left,
            face: 0,
            triangle: [0, 1, 2],
            boundary: vec![
                FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
                FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: p3(0, 0, 0),
                },
            ],
        }],
    };

    let error =
        hypermesh::exact::checked_triangulate_face_regions_with_earcut(&region_plan, &mesh, &mesh)
            .unwrap_err();

    assert!(matches!(error, hypertri::Error::InvalidInput { .. }));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_certified_aabb_disjoint_meshes() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::BoundsDisjoint
        }
    );
    let mut bad_shortcut = union.clone();
    bad_shortcut.graph_had_unknowns = true;
    assert_eq!(
        bad_shortcut.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::ShortcutResultHasUnknownGraph
    );
    assert_eq!(union.mesh.triangles().len(), 2);
    assert_eq!(union.mesh.vertices().len(), 6);

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());
    assert!(intersection.mesh.vertices().is_empty());

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles(), left.triangles());
    assert_eq!(difference.mesh.vertices(), left.vertices());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedBoundsDisjoint
    );
    assert_eq!(preflight.retained_face_pairs, 0);
    assert!(preflight.region_classifications.is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_preflight_reports_boundary_touching_policy_gap() {
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

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresBoundaryPolicy
    );
    assert!(!preflight.graph_had_unknowns);
    assert_eq!(preflight.retained_face_pairs, 1);
    assert!(preflight.retained_events > 0);
    assert_eq!(preflight.region_count, 0);
    assert!(preflight.region_classifications.is_empty());
    let blocker = preflight.blocker.as_ref().unwrap();
    assert_eq!(
        blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy
    );
    assert_eq!(blocker.coplanar_touching_pairs, 1);
    assert_eq!(blocker.candidate_pairs, 0);
    assert_eq!(blocker.coplanar_overlapping_pairs, 0);
    blocker.validate_against_sources(&left, &right).unwrap();
    let boundary_report =
        hypermesh::exact::certify_boundary_touching_report(&left, &right).unwrap();
    boundary_report.validate().unwrap();
    boundary_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert!(boundary_report.is_certified());
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::Certified
    );
    assert!(!boundary_report.graph_had_unknowns);
    assert_eq!(boundary_report.retained_face_pairs, 1);
    assert_eq!(boundary_report.blocker.coplanar_touching_pairs, 1);
    assert_eq!(boundary_report.blocker.candidate_pairs, 0);
    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    winding_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::BoundaryPolicyRequired
    );
    assert_eq!(
        winding_report.blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy
    );

    let unsupported = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap_err();
    assert!(
        unsupported
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::UnsupportedExactOperation)
    );

    let union = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: hypermesh::exact::ExactBooleanOperation::Union
        }
    );
    assert_eq!(union.mesh.triangles().len(), 2);
    assert_eq!(union.mesh.vertices().len(), 6);

    let intersection = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let difference = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles(), left.triangles());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_closed_coplanar_overlap_contact_uses_boundary_policy_not_planar_cells() {
    let left = axis_aligned_box_i64([0, 0, -2], [2, 2, 0]);
    let right = top_subdivided_axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);

    let graph = build_intersection_graph(&left, &right).unwrap();
    assert!(!graph.has_unknowns());
    assert!(graph.face_pairs.iter().any(|pair| {
        pair.relation == hypermesh::exact::MeshFacePairRelation::CoplanarOverlapping
    }));
    assert!(graph.face_pairs.iter().all(|pair| {
        pair.relation != hypermesh::exact::MeshFacePairRelation::Candidate
            || pair.events.iter().all(|event| {
                !matches!(
                    event,
                    IntersectionEvent::SegmentPlane {
                        relation: SegmentPlaneRelation::ProperCrossing
                            | SegmentPlaneRelation::ConstructionFailed
                            | SegmentPlaneRelation::Unknown,
                        ..
                    } | IntersectionEvent::Unknown
                )
            })
    }));

    let left_in_right =
        hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(&left, &right);
    let right_in_left =
        hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(&right, &left);
    left_in_right
        .validate_against_sources(&left, &right)
        .unwrap();
    right_in_left
        .validate_against_sources(&right, &left)
        .unwrap();
    assert!(
        left_in_right.vertices.iter().all(|vertex| {
            vertex.relation != hypermesh::exact::ClosedMeshWindingRelation::Inside
        })
    );
    assert!(
        right_in_left.vertices.iter().all(|vertex| {
            vertex.relation != hypermesh::exact::ClosedMeshWindingRelation::Inside
        })
    );

    let boundary_report =
        hypermesh::exact::certify_boundary_touching_report(&left, &right).unwrap();
    boundary_report.validate().unwrap();
    boundary_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::Certified
    );
    assert!(boundary_report.blocker.coplanar_overlapping_pairs > 0);

    let planar_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    planar_report.validate().unwrap();
    assert_eq!(
        planar_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::BoundaryPolicyRequired
    );
    assert_eq!(
        planar_report.blocker.kind,
        hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresBoundaryPolicy
    );
    assert!(preflight.blocker.unwrap().coplanar_overlapping_pairs > 0);

    assert!(
        hypermesh::exact::boolean_exact(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .is_err()
    );
    let union = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: hypermesh::exact::ExactBooleanOperation::Union
        }
    );
    assert_eq!(
        union.mesh.triangles().len(),
        left.triangles().len() + right.triangles().len()
    );

    let intersection = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let difference = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles(), left.triangles());
    assert_eq!(difference.mesh.vertices(), left.vertices());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_closed_vertex_touch_contact_uses_boundary_policy() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, -2, 0, 0, 0, -2, 0, 0, 0, -2],
        &[0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0],
    )
    .unwrap();

    let graph = build_intersection_graph(&left, &right).unwrap();
    assert!(!graph.has_unknowns());
    assert!(
        graph
            .face_pairs
            .iter()
            .any(|pair| { pair.relation == hypermesh::exact::MeshFacePairRelation::Candidate })
    );
    assert!(graph.face_pairs.iter().all(|pair| {
        pair.relation != hypermesh::exact::MeshFacePairRelation::Candidate
            || pair.events.iter().all(|event| {
                !matches!(
                    event,
                    IntersectionEvent::SegmentPlane {
                        relation: SegmentPlaneRelation::ProperCrossing
                            | SegmentPlaneRelation::ConstructionFailed
                            | SegmentPlaneRelation::Unknown,
                        ..
                    } | IntersectionEvent::Unknown
                )
            })
    }));

    let left_in_right =
        hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(&left, &right);
    let right_in_left =
        hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(&right, &left);
    left_in_right
        .validate_against_sources(&left, &right)
        .unwrap();
    right_in_left
        .validate_against_sources(&right, &left)
        .unwrap();
    assert!(
        left_in_right.vertices.iter().all(|vertex| {
            vertex.relation != hypermesh::exact::ClosedMeshWindingRelation::Inside
        })
    );
    assert!(
        right_in_left.vertices.iter().all(|vertex| {
            vertex.relation != hypermesh::exact::ClosedMeshWindingRelation::Inside
        })
    );
    assert!(
        left_in_right.vertices.iter().any(|vertex| {
            vertex.relation == hypermesh::exact::ClosedMeshWindingRelation::Boundary
        }) || right_in_left.vertices.iter().any(|vertex| {
            vertex.relation == hypermesh::exact::ClosedMeshWindingRelation::Boundary
        })
    );

    let boundary_report =
        hypermesh::exact::certify_boundary_touching_report(&left, &right).unwrap();
    boundary_report.validate().unwrap();
    boundary_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::Certified
    );
    assert!(boundary_report.blocker.candidate_pairs > 0);
    assert_eq!(boundary_report.blocker.coplanar_overlapping_pairs, 0);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresBoundaryPolicy
    );

    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::BoundaryPolicyRequired
    );

    let union = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.mesh.triangles().len(),
        left.triangles().len() + right.triangles().len()
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_face_touching_boxes_materialize_regularized_union_only() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([2, 0, 0], [4, 2, 2]);

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_preflight.validate().unwrap();
    union_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
    );
    assert!(union_preflight.blocker.is_none());

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    union.validate().unwrap();
    union
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxUnion
        }
    );
    assert_eq!(union.mesh.vertices().len(), 8);
    assert_eq!(union.mesh.triangles().len(), 12);
    assert!(union.mesh.facts().mesh.closed_manifold);
    let bounds = union.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&bounds.min.x, &ExactReal::from(0));
    assert_real_eq(&bounds.max.x, &ExactReal::from(4));

    let difference_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    difference_preflight.validate().unwrap();
    difference_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        difference_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
    );
    assert!(difference_preflight.blocker.is_none());
    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    difference.validate().unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxDifference
        }
    );
    assert_eq!(difference.mesh.vertices(), left.vertices());
    assert_eq!(difference.mesh.triangles(), left.triangles());

    assert!(
        hypermesh::exact::intersect_closed_convex_solids(&left, &right).is_none(),
        "zero-volume face contact must not be certified as a closed-convex solid intersection"
    );
    let intersection_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection_preflight.validate().unwrap();
    intersection_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        intersection_preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresBoundaryPolicy
    );
    let intersection = hypermesh::exact::boolean_exact_with_boundary_policy(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        hypermesh::exact::ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let edge_touching = axis_aligned_box_i64([2, 2, 0], [4, 4, 2]);
    assert!(hypermesh::exact::intersect_closed_convex_solids(&left, &edge_touching).is_none());
    let edge_union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &edge_touching,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    edge_union_preflight.validate().unwrap();
    edge_union_preflight
        .validate_against_sources(&left, &edge_touching)
        .unwrap();
    assert_eq!(
        edge_union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresBoundaryPolicy
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_boxes_materialize_slab_union_and_difference() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([1, 0, 0], [3, 2, 2]);

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_preflight.validate().unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
    );
    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    union.validate().unwrap();
    union
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxUnion
        }
    );
    let union_bounds = union.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&union_bounds.min.x, &ExactReal::from(0));
    assert_real_eq(&union_bounds.max.x, &ExactReal::from(3));
    assert!(union.mesh.facts().mesh.closed_manifold);

    let difference_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    difference_preflight.validate().unwrap();
    assert_eq!(
        difference_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
    );
    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    difference.validate().unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxDifference
        }
    );
    let difference_bounds = difference.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&difference_bounds.min.x, &ExactReal::from(0));
    assert_real_eq(&difference_bounds.max.x, &ExactReal::from(1));
    assert!(difference.mesh.facts().mesh.closed_manifold);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_box_difference_materializes_two_components() {
    let left = axis_aligned_box_i64([0, 0, 0], [4, 2, 2]);
    let right = axis_aligned_box_i64([1, 0, 0], [3, 2, 2]);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxMultiDifference
    );

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    difference.validate().unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxMultiDifference
        }
    );
    assert_eq!(difference.mesh.vertices().len(), 16);
    assert_eq!(difference.mesh.triangles().len(), 24);
    assert!(difference.mesh.facts().mesh.closed_manifold);
    let bounds = difference.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&bounds.min.x, &ExactReal::from(0));
    assert_real_eq(&bounds.max.x, &ExactReal::from(4));

    let left_component_vertices = difference
        .mesh
        .vertices()
        .iter()
        .filter(|vertex| {
            compare_reals(&vertex.coordinates().0[0], &ExactReal::from(1)).value()
                != Some(Ordering::Greater)
        })
        .count();
    let right_component_vertices = difference
        .mesh
        .vertices()
        .iter()
        .filter(|vertex| {
            compare_reals(&vertex.coordinates().0[0], &ExactReal::from(3)).value()
                != Some(Ordering::Less)
        })
        .count();
    assert_eq!(left_component_vertices, 8);
    assert_eq!(right_component_vertices, 8);

    let mut relabeled = difference.clone();
    relabeled.kind = hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
        shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxDifference,
    };
    assert!(
        relabeled
            .validate_operation_against_sources(
                &left,
                &right,
                hypermesh::exact::ExactBooleanOperation::Difference,
                ValidationPolicy::CLOSED,
                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err()
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_box_difference_materializes_nested_cavity() {
    let left = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
    let right = axis_aligned_box_i64([1, 1, 1], [3, 3, 3]);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxNestedDifference
    );
    assert!(preflight.blocker.is_none());

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    difference.validate().unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxNestedDifference
        }
    );
    assert_eq!(difference.mesh.vertices().len(), 16);
    assert_eq!(difference.mesh.triangles().len(), 24);
    assert!(difference.mesh.facts().mesh.closed_manifold);
    let bounds = difference.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&bounds.min.x, &ExactReal::from(0));
    assert_real_eq(&bounds.max.x, &ExactReal::from(4));
    assert!(
        difference
            .mesh
            .vertices()
            .iter()
            .any(|vertex| real_eq(&vertex.coordinates().0[0], &ExactReal::from(1)))
    );
    assert!(
        difference
            .mesh
            .vertices()
            .iter()
            .any(|vertex| real_eq(&vertex.coordinates().0[0], &ExactReal::from(3)))
    );

    let mut stale = preflight.clone();
    stale.support = hypermesh::exact::ExactBooleanSupport::CertifiedConvexContainment;
    assert!(stale.validate_against_sources(&left, &right).is_err());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_box_union_materializes_containment_shortcut() {
    let outer = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
    let inner = axis_aligned_box_i64([1, 1, 1], [3, 3, 3]);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&outer, &inner).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
    );

    let union = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    union.validate().unwrap();
    union
        .validate_operation_against_sources(
            &outer,
            &inner,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxUnion
        }
    );
    assert_eq!(union.mesh.vertices().len(), 8);
    assert_eq!(union.mesh.triangles().len(), 12);
    assert!(union.mesh.facts().mesh.closed_manifold);
    let bounds = union.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&bounds.min.x, &ExactReal::from(0));
    assert_real_eq(&bounds.max.x, &ExactReal::from(4));
    assert_real_eq(&bounds.min.y, &ExactReal::from(0));
    assert_real_eq(&bounds.max.y, &ExactReal::from(4));
    assert_real_eq(&bounds.min.z, &ExactReal::from(0));
    assert_real_eq(&bounds.max.z, &ExactReal::from(4));

    let mut stale = preflight.clone();
    stale.support = hypermesh::exact::ExactBooleanSupport::CertifiedConvexContainment;
    assert!(stale.validate_against_sources(&outer, &inner).is_err());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_box_difference_materializes_empty_contained_left() {
    let inner = axis_aligned_box_i64([1, 1, 1], [3, 3, 3]);
    let outer = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&inner, &outer).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
    );

    let difference = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    difference.validate().unwrap();
    difference
        .validate_operation_against_sources(
            &inner,
            &outer,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxEmptyDifference
        }
    );
    assert!(difference.mesh.vertices().is_empty());
    assert!(difference.mesh.triangles().is_empty());

    let boundary_touching_inner = axis_aligned_box_i64([0, 1, 1], [2, 3, 3]);
    let boundary_touching = hypermesh::exact::preflight_boolean_exact(
        &boundary_touching_inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    boundary_touching.validate().unwrap();
    boundary_touching
        .validate_against_sources(&boundary_touching_inner, &outer)
        .unwrap();
    assert_eq!(
        boundary_touching.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
    );
    let boundary_touching_difference = hypermesh::exact::boolean_exact(
        &boundary_touching_inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    boundary_touching_difference
        .validate_operation_against_sources(
            &boundary_touching_inner,
            &outer,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(boundary_touching_difference.mesh.triangles().is_empty());

    let protruding_overlap = axis_aligned_box_i64([-1, 1, 1], [2, 3, 3]);
    let protruding = hypermesh::exact::preflight_boolean_exact(
        &protruding_overlap,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    protruding.validate().unwrap();
    assert_ne!(
        protruding.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
    );

    let mut stale = preflight.clone();
    stale.support = hypermesh::exact::ExactBooleanSupport::CertifiedConvexContainment;
    assert!(stale.validate_against_sources(&inner, &outer).is_err());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_box_difference_materializes_l_cell_complex() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxCellDifference
    );

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    difference.validate().unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxCellDifference
        }
    );
    assert_eq!(difference.mesh.vertices().len(), 16);
    assert_eq!(difference.mesh.triangles().len(), 28);
    assert!(difference.mesh.facts().mesh.closed_manifold);

    let mut stale = preflight.clone();
    stale.support = hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxDifference;
    assert!(stale.validate_against_sources(&left, &right).is_err());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_box_union_materializes_orthogonal_cell_complex() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxCellUnion
    );

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    union.validate().unwrap();
    union
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxCellUnion
        }
    );
    assert!(union.mesh.triangles().len() > left.triangles().len() + right.triangles().len());
    assert!(union.mesh.facts().mesh.closed_manifold);
    let bounds = union.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&bounds.min.x, &ExactReal::from(0));
    assert_real_eq(&bounds.max.x, &ExactReal::from(3));
    assert_real_eq(&bounds.min.y, &ExactReal::from(0));
    assert_real_eq(&bounds.max.y, &ExactReal::from(3));
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_axis_aligned_coplanar_volumetric_box_intersection_materializes_box_shortcut() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxIntersection
    );
    assert!(preflight.blocker.is_none());

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    intersection.validate().unwrap();
    intersection
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::AxisAlignedBoxIntersection
        }
    );
    assert_eq!(intersection.mesh.vertices().len(), 8);
    assert_eq!(intersection.mesh.triangles().len(), 12);
    assert!(intersection.mesh.facts().mesh.closed_manifold);
    let bounds = intersection.mesh.bounds().mesh.as_ref().unwrap();
    assert_real_eq(&bounds.min.x, &ExactReal::from(1));
    assert_real_eq(&bounds.max.x, &ExactReal::from(2));
    assert_real_eq(&bounds.min.y, &ExactReal::from(1));
    assert_real_eq(&bounds.max.y, &ExactReal::from(2));
    assert_real_eq(&bounds.min.z, &ExactReal::from(0));
    assert_real_eq(&bounds.max.z, &ExactReal::from(2));

    let mut stale = preflight.clone();
    stale.support = hypermesh::exact::ExactBooleanSupport::CertifiedConvexIntersection;
    assert!(stale.validate_against_sources(&left, &right).is_err());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_mixed_coplanar_volumetric_overlap_materializes_from_face_cells() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2,
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .unwrap();
    let right = top_subdivided_axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);

    let graph = build_intersection_graph(&left, &right).unwrap();
    assert!(!graph.has_unknowns());
    assert!(graph.face_pairs.iter().any(|pair| {
        pair.relation == hypermesh::exact::MeshFacePairRelation::CoplanarOverlapping
    }));
    assert!(graph.face_pairs.iter().any(|pair| {
        pair.relation == hypermesh::exact::MeshFacePairRelation::Candidate
            && pair.events.iter().any(|event| {
                matches!(
                    event,
                    IntersectionEvent::SegmentPlane {
                        relation: SegmentPlaneRelation::ProperCrossing,
                        ..
                    }
                )
            })
    }));

    let boundary_report =
        hypermesh::exact::certify_boundary_touching_report(&left, &right).unwrap();
    boundary_report.validate().unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::NotBoundaryOnly
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedWindingMaterialized
    );
    assert!(preflight.blocker.is_none());
    assert!(preflight.region_count > 0);

    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    winding_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::Ready
    );
    assert!(winding_report.region_count > 0);

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    result.validate().unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::WindingMaterialized {
            operation: hypermesh::exact::ExactBooleanOperation::Union
        }
    );
    assert!(result.mesh.facts().mesh.closed_manifold);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_single_triangle_coplanar_containment() {
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

    assert_eq!(
        hypermesh::exact::certify_single_triangle_coplanar_containment(&inner, &outer),
        Some(hypermesh::exact::CoplanarSurfaceContainment::LeftInsideRight)
    );
    assert_eq!(
        hypermesh::exact::certify_single_triangle_coplanar_containment(&outer, &inner),
        Some(hypermesh::exact::CoplanarSurfaceContainment::RightInsideLeft)
    );
    let containment =
        hypermesh::exact::certify_single_triangle_coplanar_containment_report(&inner, &outer);
    containment.validate().unwrap();
    containment
        .validate_against_sources(&inner, &outer)
        .unwrap();
    assert_eq!(
        containment
            .validate_against_sources(&outer, &inner)
            .unwrap_err(),
        hypermesh::exact::CoplanarSurfaceContainmentReportError::SourceReplayMismatch
    );
    assert_eq!(
        containment.status,
        hypermesh::exact::CoplanarSurfaceContainmentStatus::Certified(
            hypermesh::exact::CoplanarSurfaceContainment::LeftInsideRight
        )
    );
    assert_eq!(
        containment.triangle.as_ref().unwrap().relation,
        TriangleTriangleRelation::CoplanarOverlapping
    );
    assert_eq!(
        containment.coplanar.as_ref().unwrap().relation,
        CoplanarTriangleRelation::Overlapping
    );
    assert!(containment.all_proof_producing());
    let mut mislabeled_containment = containment.clone();
    mislabeled_containment.status = hypermesh::exact::CoplanarSurfaceContainmentStatus::Certified(
        hypermesh::exact::CoplanarSurfaceContainment::RightInsideLeft,
    );
    assert_eq!(
        mislabeled_containment.validate().unwrap_err(),
        hypermesh::exact::CoplanarSurfaceContainmentReportError::StatusRelationMismatch
    );
    let mut mislabeled_disjoint = containment.clone();
    mislabeled_disjoint.status =
        hypermesh::exact::CoplanarSurfaceContainmentStatus::DisjointOrUnknown;
    assert_eq!(
        mislabeled_disjoint.validate().unwrap_err(),
        hypermesh::exact::CoplanarSurfaceContainmentReportError::StatusRelationMismatch
    );

    let union = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), outer.triangles());
    assert_eq!(union.mesh.vertices(), outer.vertices());

    let intersection = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), inner.triangles());
    assert_eq!(intersection.mesh.vertices(), inner.vertices());

    let empty_difference = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(empty_difference.mesh.triangles().is_empty());
    assert!(empty_difference.mesh.vertices().is_empty());

    let holed_difference = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    holed_difference.validate().unwrap();
    assert_eq!(
        holed_difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceHoledDifference
        }
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
    );
    assert_eq!(preflight.retained_face_pairs, 0);
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_surface_containment_report_retains_rejection_state() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 1, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let report =
        hypermesh::exact::certify_single_triangle_coplanar_containment_report(&left, &right);
    report.validate().unwrap();
    assert_eq!(
        report.status,
        hypermesh::exact::CoplanarSurfaceContainmentStatus::NotCoplanar
    );
    assert!(report.triangle.is_some());
    assert!(report.coplanar.is_none());
    assert!(report.all_proof_producing());

    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
        &[0, 1, 2, 1, 3, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let not_single =
        hypermesh::exact::certify_single_triangle_coplanar_containment_report(&open, &right);
    not_single.validate().unwrap();
    assert_eq!(
        not_single.status,
        hypermesh::exact::CoplanarSurfaceContainmentStatus::NotSingleTriangle
    );
    assert!(not_single.triangle.is_none());
    assert!(not_single.coplanar.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_open_surface_disjoint_with_overlapping_bounds() {
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

    assert_ne!(
        left.bounds()
            .mesh
            .as_ref()
            .unwrap()
            .classify_intersection(right.bounds().mesh.as_ref().unwrap())
            .value(),
        Some(hypermesh::exact::AabbIntersectionKind::Disjoint)
    );

    let retained = classify_mesh_face_pairs(&left, &right).unwrap();
    assert!(retained.is_empty());
    let report = hypermesh::exact::certify_open_surface_disjoint_report(&left, &right).unwrap();
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert!(report.is_certified());
    assert_eq!(
        report.status,
        hypermesh::exact::ExactOpenSurfaceDisjointStatus::Certified
    );
    assert!(report.left_open_surface);
    assert!(report.right_open_surface);
    assert!(!report.graph_had_unknowns);
    assert_eq!(report.retained_face_pairs, 0);
    assert_eq!(report.retained_events, 0);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedOpenSurfaceDisjoint
    );
    assert!(preflight.blocker.is_none());

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::OpenSurfaceDisjoint
        }
    );
    assert_eq!(union.mesh.triangles().len(), 2);
    assert_eq!(union.mesh.vertices().len(), 6);

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles(), left.triangles());
    assert_eq!(difference.mesh.vertices(), left.vertices());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_graph_shortcut_reports_retain_rejection_state() {
    let closed = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            1, 0, 0, //
            0, 1, 0, //
            0, 0, 1,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let open = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 11, 0, 0, 10, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_report =
        hypermesh::exact::certify_open_surface_disjoint_report(&closed, &open).unwrap();
    assert_eq!(
        open_report.status,
        hypermesh::exact::ExactOpenSurfaceDisjointStatus::NotOpenSurface
    );
    assert!(!open_report.left_open_surface);
    assert!(open_report.right_open_surface);

    let touching = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 2, 0, 0, 1, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let boundary_report =
        hypermesh::exact::certify_boundary_touching_report(&open, &touching).unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::NotBoundaryOnly
    );
    assert!(!boundary_report.is_certified());

    let impossible_open_report = hypermesh::exact::ExactOpenSurfaceDisjointReport {
        status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphHasFacePairs,
        left_open_surface: true,
        right_open_surface: true,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        impossible_open_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let impossible_boundary_report = hypermesh::exact::ExactBoundaryTouchingReport {
        status: hypermesh::exact::ExactBoundaryTouchingStatus::Certified,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 1,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        impossible_boundary_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let impossible_planar_report = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactPlanarArrangementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 1,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        impossible_planar_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let impossible_winding_report = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::GraphUnknowns,
        graph_had_unknowns: true,
        retained_face_pairs: 0,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsRefinement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 1,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        impossible_winding_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );

    let unknown_open_status_mismatch = hypermesh::exact::ExactOpenSurfaceDisjointReport {
        status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::Certified,
        left_open_surface: true,
        right_open_surface: true,
        graph_had_unknowns: true,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        unknown_open_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );

    let unknown_open_wrong_blocker = hypermesh::exact::ExactOpenSurfaceDisjointReport {
        status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphUnknowns,
        left_open_surface: true,
        right_open_surface: true,
        graph_had_unknowns: true,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 1,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        unknown_open_wrong_blocker.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::WrongBlockerKind
    );

    let construction_failed_open_report = hypermesh::exact::ExactOpenSurfaceDisjointReport {
        status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphHasFacePairs,
        left_open_surface: true,
        right_open_surface: true,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 1,
        },
    };
    assert_eq!(
        construction_failed_open_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let unknown_boundary_status_mismatch = hypermesh::exact::ExactBoundaryTouchingReport {
        status: hypermesh::exact::ExactBoundaryTouchingStatus::Certified,
        graph_had_unknowns: true,
        retained_face_pairs: 1,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 1,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        unknown_boundary_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );

    let unknown_boundary_wrong_blocker = hypermesh::exact::ExactBoundaryTouchingReport {
        status: hypermesh::exact::ExactBoundaryTouchingStatus::GraphUnknowns,
        graph_had_unknowns: true,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 1,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        unknown_boundary_wrong_blocker.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::WrongBlockerKind
    );

    let construction_failed_boundary_report = hypermesh::exact::ExactBoundaryTouchingReport {
        status: hypermesh::exact::ExactBoundaryTouchingStatus::NotBoundaryOnly,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 1,
        },
    };
    assert_eq!(
        construction_failed_boundary_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let unknown_planar_status_mismatch = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactPlanarArrangementStatus::GraphUnknowns,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        unknown_planar_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );

    let unknown_planar_wrong_blocker = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactPlanarArrangementStatus::GraphUnknowns,
        graph_had_unknowns: true,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 1,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        unknown_planar_wrong_blocker.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::WrongBlockerKind
    );

    let construction_failed_planar_report = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactPlanarArrangementStatus::NoPositiveOverlap,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 1,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        construction_failed_planar_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let unknown_winding_status_mismatch = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::Ready,
        graph_had_unknowns: true,
        retained_face_pairs: 1,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        unknown_winding_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::GraphUnknownStatusMismatch
    );

    let construction_failed_winding_report = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::NoNontrivialOverlap,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 1,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        construction_failed_winding_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let undecided_winding_region = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::Ready,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        region_count: 1,
        region_classifications: vec![hypermesh::exact::FaceRegionPlaneClassification {
            region_side: MeshSide::Left,
            region_face: 0,
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: hypermesh::exact::FaceRegionPlaneRelation::Unknown,
            node_sides: vec![None],
            predicates: vec![hypermesh::exact::PredicateUse::from_certificate(
                hyperlimit::PredicateCertificate::Unknown,
            )],
        }],
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        undecided_winding_region.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::RegionClassificationNotProofProducing
    );

    let open_precondition_mismatch = hypermesh::exact::ExactOpenSurfaceDisjointReport {
        status: hypermesh::exact::ExactOpenSurfaceDisjointStatus::NotOpenSurface,
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
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        open_precondition_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let boundary_status_mismatch = hypermesh::exact::ExactBoundaryTouchingReport {
        status: hypermesh::exact::ExactBoundaryTouchingStatus::NotBoundaryOnly,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 1,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
    };
    assert_eq!(
        boundary_status_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let planar_operation_mismatch = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactPlanarArrangementStatus::NotNamedOperation,
        graph_had_unknowns: false,
        retained_face_pairs: 0,
        retained_events: 0,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        planar_operation_mismatch.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let planar_required_without_readiness = hypermesh::exact::ExactPlanarArrangementReport {
        operation: hypermesh::exact::ExactBooleanOperation::Intersection,
        status: hypermesh::exact::ExactPlanarArrangementStatus::Required,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement,
            candidate_pairs: 0,
            coplanar_overlapping_pairs: 1,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        planar_required_without_readiness.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::MissingArrangementReadiness
    );

    let winding_selected_ready = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::SelectedRegions(
            hypermesh::exact::ExactRegionSelection::KeepAll,
        ),
        status: hypermesh::exact::ExactWindingReadinessStatus::Ready,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        winding_selected_ready.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let winding_no_overlap_with_pairs = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::NoNontrivialOverlap,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        winding_no_overlap_with_pairs.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let coplanar_volumetric_wrong_blocker = hypermesh::exact::ExactWindingReadinessReport {
        operation: hypermesh::exact::ExactBooleanOperation::Union,
        status: hypermesh::exact::ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired,
        graph_had_unknowns: false,
        retained_face_pairs: 1,
        retained_events: 1,
        region_count: 0,
        region_classifications: Vec::new(),
        blocker: hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 1,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        },
        arrangement_readiness: None,
    };
    assert_eq!(
        coplanar_volumetric_wrong_blocker.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::WrongBlockerKind
    );

    let coplanar_volumetric_without_coplanar_evidence =
        hypermesh::exact::ExactWindingReadinessReport {
            operation: hypermesh::exact::ExactBooleanOperation::Union,
            status: hypermesh::exact::ExactWindingReadinessStatus::CoplanarVolumetricCellsRequired,
            graph_had_unknowns: false,
            retained_face_pairs: 1,
            retained_events: 1,
            region_count: 0,
            region_classifications: Vec::new(),
            blocker: hypermesh::exact::ExactBooleanBlocker {
                kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsCoplanarVolumetricCells,
                candidate_pairs: 1,
                coplanar_overlapping_pairs: 0,
                coplanar_touching_pairs: 0,
                unknown_pairs: 0,
                construction_failed_events: 0,
            },
            arrangement_readiness: None,
        };
    assert_eq!(
        coplanar_volumetric_without_coplanar_evidence
            .validate()
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidBlockerCounts
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_intersect_partially_overlapping_coplanar_triangles() {
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

    let clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("partial coplanar overlap should produce a positive-area polygon");
    assert_eq!(clipped.polygon.len(), 3);
    assert_eq!(clipped.mesh.triangles().len(), 1);
    clipped.validate().unwrap();
    clipped.validate_against_sources(&left, &right).unwrap();
    assert!(clipped.validate_against_sources(&right, &left).is_err());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    assert!(preflight.blocker.is_none());

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceIntersection
        }
    );
    assert_eq!(intersection.mesh.triangles().len(), 1);
    assert_eq!(intersection.mesh.vertices().len(), 3);

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion
        }
    );

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_preflight.validate().unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    assert!(union_preflight.blocker.is_none());

    let union_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_report.validate().unwrap();
    union_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        union_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    assert_eq!(union_report.retained_face_pairs, 1);
    let mut corrupted_union_report = union_report.clone();
    if let Some(readiness) = corrupted_union_report.arrangement_readiness.as_mut() {
        readiness.graph_count += 1;
        readiness.touching_graphs += 1;
    }
    assert_eq!(
        corrupted_union_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::ArrangementReadinessMismatch
    );

    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    winding_report.validate().unwrap();
    winding_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
    );
    let mut corrupted_winding_report = winding_report.clone();
    if let Some(readiness) = corrupted_winding_report.arrangement_readiness.as_mut() {
        readiness.graph_count += 1;
        readiness.touching_graphs += 1;
    }
    assert_eq!(
        corrupted_winding_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::ArrangementReadinessMismatch
    );

    let intersection_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection_report.validate().unwrap();
    assert_eq!(
        intersection_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    assert!(intersection_report.arrangement_readiness.is_some());

    let difference_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        difference_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    assert!(difference_preflight.blocker.is_none());
    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    difference.validate().unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceArrangementDifference
        }
    );

    let mut reversed_loop = hypermesh::exact::arrange_single_triangle_coplanar_union(&left, &right)
        .expect("fixture should produce a simple-loop arrangement");
    reversed_loop.polygon.reverse();
    reversed_loop.mesh =
        surface_mesh_from_polygon(&reversed_loop.polygon, "reversed simple-loop fixture").unwrap();
    assert!(reversed_loop.validate().is_err());
}

#[cfg(feature = "exact-triangulation")]
fn surface_mesh_from_polygon(polygon: &[Point3], label: &'static str) -> Result<ExactMesh, String> {
    let vertices = polygon
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    let triangles = (1..polygon.len().saturating_sub(1))
        .map(|index| Triangle([0, index, index + 1]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .map_err(|error| format!("{error:?}"))
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_multi_component_coplanar_intersection_materializes_before_winding() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0, 10, 0, 0, 14, 0, 0, 10, 4, 0],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 13, 1, 0, 1, 3, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_intersection(&left, &right).is_none()
    );
    let multi = hypermesh::exact::arrange_coplanar_convex_surface_multi_intersection(&left, &right)
        .expect("disconnected coplanar clips should materialize as retained components");
    multi.validate().unwrap();
    multi
        .validate_intersection_against_sources(&left, &right)
        .unwrap();
    assert_eq!(multi.polygons.len(), 2);

    let report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    assert!(report.blocker.coplanar_overlapping_pairs > 0);
    report
        .blocker
        .validate_against_sources(&left, &right)
        .unwrap();
    assert!(
        report
            .arrangement_readiness
            .as_ref()
            .unwrap()
            .needs_planar_cells()
    );

    let winding = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    winding.validate().unwrap();
    winding.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        winding.status,
        hypermesh::exact::ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_surface_outputs_validate_public_artifacts() {
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
    let mut clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("partial coplanar overlap should produce a positive-area polygon");
    clipped.validate().unwrap();
    clipped.polygon[1] = clipped.polygon[0].clone();
    let duplicate = clipped.validate().unwrap_err();
    assert!(
        duplicate
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );

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
    let mut union = hypermesh::exact::union_single_triangle_coplanar_surfaces(&left, &right)
        .expect("diagonal-adjacent triangles should union into a square");
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    union.polygon.push(p3(2, 2, 0));
    let drift = union.validate().unwrap_err();
    assert!(
        drift
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_union_materializes_convex_edge_touching_square() {
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

    let union = hypermesh::exact::union_single_triangle_coplanar_surfaces(&left, &right)
        .expect("diagonal-adjacent triangles should union into a square");
    assert_eq!(union.polygon.len(), 4);
    assert_eq!(union.mesh.triangles().len(), 2);
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert!(union.validate_against_sources(&left, &left).is_err());
    let mut nonconvex_union = union.clone();
    nonconvex_union.polygon = vec![p3(0, 0, 0), p3(3, 0, 0), p3(1, 1, 0), p3(0, 3, 0)];
    nonconvex_union.mesh = fan_mesh_from_points(&nonconvex_union.polygon);
    assert!(nonconvex_union.validate().is_err());

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceConvexUnion
        }
    );
    assert_eq!(result.mesh.vertices().len(), 4);
    assert_eq!(result.mesh.triangles().len(), 2);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceConvexUnion
    );
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_union_materializes_simple_planar_arrangement() {
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

    assert!(hypermesh::exact::union_single_triangle_coplanar_surfaces(&left, &right).is_none());
    let arrangement = hypermesh::exact::arrange_single_triangle_coplanar_union(&left, &right)
        .expect("simple single-loop triangle union should materialize");
    arrangement.validate().unwrap();
    arrangement
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::CoplanarArrangementOperation::Union,
        )
        .unwrap();
    assert!(
        arrangement
            .validate_against_sources(
                &left,
                &right,
                hypermesh::exact::CoplanarArrangementOperation::Difference,
            )
            .is_err()
    );
    assert!(arrangement.polygon.len() >= 4);
    assert_eq!(arrangement.mesh.vertices().len(), arrangement.polygon.len());
    assert_eq!(
        arrangement.mesh.triangles().len(),
        arrangement.polygon.len() - 2
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceArrangementUnion
        }
    );

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    assert!(union_preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_difference_materializes_one_corner_cut() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[-1, -1, 0, 2, -1, 0, -1, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let difference = hypermesh::exact::difference_single_triangle_coplanar_surfaces(&left, &right)
        .expect("one strict corner cut should produce a convex difference polygon");
    assert_eq!(difference.polygon.len(), 4);
    assert_eq!(difference.mesh.vertices().len(), 4);
    assert_eq!(difference.mesh.triangles().len(), 2);
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert!(difference.validate_against_sources(&right, &left).is_err());
    let mut nonconvex_difference = difference.clone();
    nonconvex_difference.polygon = vec![p3(0, 0, 0), p3(3, 0, 0), p3(1, 1, 0), p3(0, 3, 0)];
    nonconvex_difference.mesh = fan_mesh_from_points(&nonconvex_difference.polygon);
    assert!(nonconvex_difference.validate().is_err());

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(result.mesh.vertices().len(), 4);
    assert_eq!(result.mesh.triangles().len(), 2);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
    );
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_difference_materializes_remaining_corner_cut() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[-3, 1, 0, 8, -1, 0, -3, 6, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let difference = hypermesh::exact::difference_single_triangle_coplanar_surfaces(&left, &right)
        .expect("one strict remaining corner should produce a convex difference triangle");
    assert_eq!(difference.polygon.len(), 3);
    assert_eq!(difference.mesh.vertices().len(), 3);
    assert_eq!(difference.mesh.triangles().len(), 1);
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(result.mesh.vertices().len(), 3);
    assert_eq!(result.mesh.triangles().len(), 1);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceCornerDifference
    );
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_difference_materializes_contained_hole_case() {
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

    assert!(
        hypermesh::exact::difference_single_triangle_coplanar_surfaces(&outer, &inner).is_none()
    );
    let holed = hypermesh::exact::arrange_single_triangle_coplanar_holed_difference(&outer, &inner)
        .expect("contained triangle difference should materialize one hole");
    assert_eq!(holed.outer.len(), 3);
    assert_eq!(holed.hole.len(), 3);
    assert_eq!(holed.mesh.vertices().len(), 6);
    assert!(!holed.mesh.triangles().is_empty());
    holed.validate().unwrap();
    holed.validate_against_sources(&outer, &inner).unwrap();
    assert!(holed.validate_against_sources(&inner, &outer).is_err());
    let mut reversed_outer = holed.clone();
    reversed_outer.outer.reverse();
    assert!(reversed_outer.validate().is_err());
    let mut reversed_hole = holed.clone();
    reversed_hole.hole.reverse();
    assert!(reversed_hole.validate().is_err());
    let mut reversed_mesh = holed.clone();
    reversed_mesh.mesh = reverse_mesh_triangles(&reversed_mesh.mesh);
    assert!(reversed_mesh.validate().is_err());
    let mut filled_hole_mesh = holed.clone();
    filled_hole_mesh.mesh =
        mesh_with_filled_hole_triangle(&filled_hole_mesh.mesh, holed.outer.len());
    assert!(filled_hole_mesh.validate().is_err());
    let mut repeated_outer_point = holed.clone();
    repeated_outer_point.outer[2] = repeated_outer_point.outer[0].clone();
    assert!(repeated_outer_point.validate().is_err());
    let mut hole_on_boundary = holed.clone();
    hole_on_boundary.hole[0] = hole_on_boundary.outer[0].clone();
    assert!(hole_on_boundary.validate().is_err());
    let mut partial_holed_mesh = holed.clone();
    let retained_points = partial_holed_mesh
        .outer
        .iter()
        .chain(&partial_holed_mesh.hole)
        .cloned()
        .collect::<Vec<_>>();
    partial_holed_mesh.mesh = partial_mesh_from_points(&retained_points);
    assert!(partial_holed_mesh.validate().is_err());
    if let Some(mesh) = retained_ring_crossing_mesh(&holed.mesh) {
        let mut crossing_ring_mesh = holed.clone();
        crossing_ring_mesh.mesh = mesh;
        assert!(crossing_ring_mesh.validate().is_err());
    }

    let result = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceHoledDifference
        }
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceHoledDifference
    );
    assert!(preflight.blocker.is_none());
    let report = hypermesh::exact::certify_planar_arrangement_report(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    report.validate().unwrap();
    assert_eq!(
        report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_intersection_handles_quadrilateral_clip() {
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

    let clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("quadrilateral overlap should produce a positive-area polygon");
    assert_eq!(clipped.polygon.len(), 4);
    assert_eq!(clipped.mesh.triangles().len(), 2);
    assert_eq!(clipped.mesh.vertices().len(), 4);
    clipped.validate().unwrap();

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles().len(), 2);
    assert_eq!(intersection.mesh.vertices().len(), 4);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_triangle_intersection_simplifies_edge_aligned_overlap() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 2, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let clipped = hypermesh::exact::intersect_single_triangle_coplanar_surfaces(&left, &right)
        .expect("edge-aligned overlap should produce the smaller triangle");
    assert_eq!(clipped.polygon.len(), 3);
    assert_eq!(clipped.mesh.triangles().len(), 1);
    assert_eq!(clipped.mesh.vertices().len(), 3);
    clipped.validate().unwrap();
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_structurally_identical_meshes() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), mesh.triangles());
    assert_eq!(union.mesh.vertices(), mesh.vertices());

    let intersection = hypermesh::exact::boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), mesh.triangles());
    assert_eq!(intersection.mesh.vertices(), mesh.vertices());

    let difference = hypermesh::exact::boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(difference.mesh.triangles().is_empty());
    assert!(difference.mesh.vertices().is_empty());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &mesh,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedIdentical
    );
    assert_eq!(preflight.region_count, 0);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_reversed_same_indexed_surface() {
    let vertices = [
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let mesh =
        ExactMesh::from_i64_triangles(&vertices, &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3]).unwrap();
    let reversed =
        ExactMesh::from_i64_triangles(&vertices, &[0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0]).unwrap();

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedSameSurface
    );
    assert_eq!(preflight.retained_events, 0);
    let report = hypermesh::exact::certify_same_surface_report(&mesh, &reversed);
    report.validate().unwrap();
    assert!(report.is_certified());
    assert_eq!(report.left_to_right, vec![0, 1, 2, 3]);
    assert_eq!(report.right_to_left, vec![0, 1, 2, 3]);
    assert_eq!(report.left_triangles, report.right_triangles);
    assert!(report.all_proof_producing());

    let union = hypermesh::exact::boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), mesh.triangles());

    let intersection = hypermesh::exact::boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), mesh.triangles());

    let difference = hypermesh::exact::boolean_exact(
        &mesh,
        &reversed,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert!(difference.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_reindexed_same_surface() {
    let left_vertices = [
        0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        0, 0, 1,
    ];
    let right_vertices = [
        0, 0, 1, //
        0, 0, 0, //
        0, 1, 0, //
        1, 0, 0,
    ];
    let left = ExactMesh::from_i64_triangles(&left_vertices, &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3])
        .unwrap();
    let right =
        ExactMesh::from_i64_triangles(&right_vertices, &[1, 3, 2, 1, 0, 3, 3, 0, 2, 2, 0, 1])
            .unwrap();

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedSameSurface
    );
    let report = hypermesh::exact::certify_same_surface_report(&left, &right);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert!(report.is_certified());
    assert_eq!(report.left_to_right, vec![1, 3, 2, 0]);
    assert_eq!(report.right_to_left, vec![3, 0, 2, 1]);
    assert_eq!(report.left_triangles, report.right_triangles);
    assert!(report.all_proof_producing());
    assert_eq!(
        report.validate_against_sources(&right, &left).unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SourceReplayMismatch
    );

    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(intersection.mesh.vertices(), left.vertices());
    assert_eq!(intersection.mesh.triangles(), left.triangles());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_coplanar_convex_surface_retriangulation() {
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

    assert!(!hypermesh::exact::certify_same_surface_report(&left, &right).is_certified());
    let certificate = hypermesh::exact::certify_coplanar_convex_surface_equivalence(&left, &right)
        .expect("same square with opposite diagonals should certify by exact hull/area");
    certificate.validate().unwrap();
    certificate.validate_against_sources(&left, &right).unwrap();
    let shifted = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        certificate
            .validate_against_sources(&left, &shifted)
            .is_err()
    );
    assert_eq!(certificate.polygon.len(), 4);
    let mut reversed_hull = certificate.clone();
    reversed_hull.polygon.reverse();
    assert!(reversed_hull.validate().is_err());
    let mut repeated_hull_point = certificate.clone();
    repeated_hull_point.polygon[1] = repeated_hull_point.polygon[0].clone();
    assert!(repeated_hull_point.validate().is_err());
    let mut nonconvex_hull = certificate.clone();
    nonconvex_hull.polygon = vec![p3(0, 0, 0), p3(2, 0, 0), p3(1, 1, 0), p3(0, 2, 0)];
    assert!(nonconvex_hull.validate().is_err());
    let report = hypermesh::exact::certify_coplanar_convex_surface_report(&left, &right);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert!(report.is_certified());
    assert_eq!(
        report.status,
        hypermesh::exact::CoplanarConvexSurfaceReportStatus::Equivalent
    );
    assert!(report.equivalence.is_some());
    assert!(report.containment.is_none());
    let mut stale_report = report.clone();
    stale_report
        .equivalence
        .as_mut()
        .unwrap()
        .polygon
        .rotate_left(1);
    assert_eq!(
        stale_report
            .validate_against_sources(&left, &right)
            .unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::SourceReplayMismatch
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceEquivalence
    );
    assert!(preflight.blocker.is_none());

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    union.validate().unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceEquivalence
        }
    );
    assert_eq!(union.mesh.triangles(), left.triangles());

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(difference.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_coplanar_convex_surface_containment() {
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

    let certificate = hypermesh::exact::certify_coplanar_convex_surface_containment(&outer, &inner)
        .expect("inner square should certify inside outer square");
    certificate.validate().unwrap();
    certificate
        .validate_against_sources(&outer, &inner)
        .unwrap();
    assert!(
        certificate
            .validate_against_sources(&inner, &outer)
            .is_err()
    );
    assert_eq!(
        certificate.relation,
        hypermesh::exact::CoplanarConvexSurfaceContainment::RightInsideLeft
    );
    let mut reversed_left_hull = certificate.clone();
    reversed_left_hull.left_hull.reverse();
    assert!(reversed_left_hull.validate().is_err());
    let mut repeated_right_hull_point = certificate.clone();
    repeated_right_hull_point.right_hull[1] = repeated_right_hull_point.right_hull[0].clone();
    assert!(repeated_right_hull_point.validate().is_err());
    let mut outside_right_hull = certificate.clone();
    outside_right_hull.right_hull =
        vec![p3(10, 10, 0), p3(11, 10, 0), p3(11, 11, 0), p3(10, 11, 0)];
    assert!(outside_right_hull.validate().is_err());
    let report = hypermesh::exact::certify_coplanar_convex_surface_report(&outer, &inner);
    report.validate().unwrap();
    report.validate_against_sources(&outer, &inner).unwrap();
    assert_eq!(
        report.status,
        hypermesh::exact::CoplanarConvexSurfaceReportStatus::Contained(
            hypermesh::exact::CoplanarConvexSurfaceContainment::RightInsideLeft
        )
    );
    assert!(report.equivalence.is_none());
    assert!(report.containment.is_some());
    assert_eq!(
        report.validate_against_sources(&inner, &outer).unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::SourceReplayMismatch
    );

    let holed = hypermesh::exact::arrange_coplanar_convex_surface_holed_difference(&outer, &inner)
        .expect("outer minus inner convex sheets should materialize one hole");
    holed.validate().unwrap();
    holed.validate_against_sources(&outer, &inner).unwrap();
    assert!(holed.validate_against_sources(&inner, &outer).is_err());
    assert_eq!(holed.outer.len(), 4);
    assert_eq!(holed.hole.len(), 4);
    assert_eq!(holed.mesh.vertices().len(), 8);
    let mut reversed_convex_outer = holed.clone();
    reversed_convex_outer.outer.reverse();
    assert!(reversed_convex_outer.validate().is_err());
    let mut reversed_convex_hole = holed.clone();
    reversed_convex_hole.hole.reverse();
    assert!(reversed_convex_hole.validate().is_err());
    let mut reversed_convex_mesh = holed.clone();
    reversed_convex_mesh.mesh = reverse_mesh_triangles(&reversed_convex_mesh.mesh);
    assert!(reversed_convex_mesh.validate().is_err());
    let mut filled_convex_hole_mesh = holed.clone();
    filled_convex_hole_mesh.mesh =
        mesh_with_filled_hole_triangle(&filled_convex_hole_mesh.mesh, holed.outer.len());
    assert!(filled_convex_hole_mesh.validate().is_err());
    let mut repeated_hole_point = holed.clone();
    repeated_hole_point.hole[1] = repeated_hole_point.hole[0].clone();
    assert!(repeated_hole_point.validate().is_err());
    let mut boundary_touching_hole = holed.clone();
    boundary_touching_hole.hole[0] = boundary_touching_hole.outer[0].clone();
    assert!(boundary_touching_hole.validate().is_err());
    let mut nonconvex_hole = holed.clone();
    nonconvex_hole.hole = vec![p3(1, 1, 0), p3(3, 1, 0), p3(1, 2, 0), p3(2, 3, 0)];
    assert!(nonconvex_hole.validate().is_err());
    let mut partial_convex_holed_mesh = holed.clone();
    let retained_points = partial_convex_holed_mesh
        .outer
        .iter()
        .chain(&partial_convex_holed_mesh.hole)
        .cloned()
        .collect::<Vec<_>>();
    partial_convex_holed_mesh.mesh = partial_mesh_from_points(&retained_points);
    assert!(partial_convex_holed_mesh.validate().is_err());
    if let Some(mesh) = retained_ring_crossing_mesh(&holed.mesh) {
        let mut crossing_ring_mesh = holed.clone();
        crossing_ring_mesh.mesh = mesh;
        assert!(crossing_ring_mesh.validate().is_err());
    }

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceHoledDifference
    );

    let difference = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    difference.validate().unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceHoledDifference
        }
    );
    assert!(!difference.mesh.triangles().is_empty());

    let intersection = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), inner.triangles());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_union_materializes_simple_loop() {
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

    assert!(hypermesh::exact::certify_coplanar_convex_surface_equivalence(&left, &right).is_none());
    assert!(hypermesh::exact::certify_coplanar_convex_surface_containment(&left, &right).is_none());
    let union = hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &right)
        .expect("overlapping convex sheets should materialize one simple union loop");
    union.validate().unwrap();
    union
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::CoplanarArrangementOperation::Union,
        )
        .unwrap();
    assert!(
        union
            .validate_against_sources(
                &left,
                &right,
                hypermesh::exact::CoplanarArrangementOperation::Difference,
            )
            .is_err()
    );
    assert_eq!(union.polygon.len(), 8);
    assert!(!union.mesh.triangles().is_empty());
    let mut self_intersecting_union = union.clone();
    self_intersecting_union.polygon = vec![
        p3(0, 0, 0),
        p3(4, 4, 0),
        p3(0, 4, 0),
        p3(4, 0, 0),
        p3(5, 0, 0),
        p3(6, 0, 0),
        p3(6, 1, 0),
        p3(5, 1, 0),
    ];
    assert!(self_intersecting_union.validate().is_err());
    let mut partial_union_mesh = union.clone();
    partial_union_mesh.mesh = partial_mesh_from_points(&partial_union_mesh.polygon);
    assert!(partial_union_mesh.validate().is_err());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
    );
    assert!(preflight.blocker.is_none());

    let arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    arrangement_report.validate().unwrap();
    arrangement_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    let winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    winding_report.validate().unwrap();
    winding_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion
        }
    );
    assert_eq!(result.mesh.vertices().len(), union.mesh.vertices().len());

    let union_arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_arrangement_report.validate().unwrap();
    assert_eq!(
        union_arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );

    let intersection = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection.validate().unwrap();
    assert_eq!(
        intersection.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
    );

    let intersection_output =
        hypermesh::exact::arrange_coplanar_convex_surface_intersection(&left, &right)
            .expect("overlapping convex sheets should materialize their convex intersection");
    intersection_output.validate().unwrap();
    intersection_output
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::CoplanarArrangementOperation::Intersection,
        )
        .unwrap();
    assert_eq!(intersection_output.polygon.len(), 4);
    let intersection_result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    intersection_result.validate().unwrap();
    assert_eq!(
        intersection_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection
        }
    );

    let intersection_arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection_arrangement_report.validate().unwrap();
    assert_eq!(
        intersection_arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    let intersection_winding_report = hypermesh::exact::certify_winding_readiness_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    intersection_winding_report.validate().unwrap();
    assert_eq!(
        intersection_winding_report.status,
        hypermesh::exact::ExactWindingReadinessStatus::PlanarArrangementAlreadyMaterialized
    );

    let difference_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    difference_preflight.validate().unwrap();
    assert_eq!(
        difference_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
    );
    assert!(difference_preflight.blocker.is_none());
    let difference_arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    difference_arrangement_report.validate().unwrap();
    assert_eq!(
        difference_arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_union_materializes_full_edge_touching_rectangles() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &right)
        .expect("full-edge touching convex sheets should replay as one exact rectangle");
    union.validate().unwrap();
    union
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::CoplanarArrangementOperation::Union,
        )
        .unwrap();
    assert_eq!(union.polygon.len(), 4);
    assert!(
        union
            .polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(0)))
    );
    assert!(
        union
            .polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(4)))
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion
        }
    );

    let point_touching_right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &point_touching_right)
            .is_none()
    );
    let point_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &point_touching_right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    point_preflight.validate().unwrap();
    assert_eq!(
        point_preflight.support,
        hypermesh::exact::ExactBooleanSupport::RequiresBoundaryPolicy
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_union_materializes_multiple_components() {
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

    assert!(hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &right).is_none());
    let union = hypermesh::exact::arrange_coplanar_convex_surface_multi_union(&left, &right)
        .expect("two disjoint convex union clusters should retain two output components");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    union.validate_union_against_sources(&right, &left).unwrap();
    assert_eq!(union.polygons.len(), 2);
    assert!(union.polygons.iter().all(|polygon| polygon.len() == 4));
    assert_eq!(union.mesh.vertices().len(), 8);
    assert_eq!(union.mesh.triangles().len(), 4);

    let mut reversed_component = union.clone();
    reversed_component.polygons[0].reverse();
    assert!(reversed_component.validate().is_err());
    let mut cross_component_mesh = union.clone();
    cross_component_mesh.mesh =
        mesh_with_cross_component_triangle(&cross_component_mesh.mesh, union.polygons[0].len());
    assert!(cross_component_mesh.validate().is_err());
    let mut shared_component_point = union.clone();
    shared_component_point.polygons[1][0] = shared_component_point.polygons[0][0].clone();
    assert!(shared_component_point.validate().is_err());
    let mut stale_component = union.clone();
    stale_component.polygons[0][1] = p3(4, 0, 0);
    assert!(
        stale_component
            .validate_union_against_sources(&left, &right)
            .is_err()
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion
    );
    assert!(preflight.blocker.is_none());

    let arrangement_report = hypermesh::exact::certify_planar_arrangement_report(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    arrangement_report.validate().unwrap();
    assert_eq!(
        arrangement_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::AlreadyMaterialized
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiUnion
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_union_materializes_bridged_strip_cluster() {
    let left = ExactMesh::from_i64_triangles_with_policy(
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
    let right = ExactMesh::from_i64_triangles_with_policy(
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

    assert!(hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &right).is_none());
    let union = hypermesh::exact::arrange_coplanar_convex_surface_multi_union(&left, &right)
        .expect("bridge strip cluster plus far cluster should retain two exact output loops");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    union.validate_union_against_sources(&right, &left).unwrap();
    assert_eq!(union.polygons.len(), 2);
    assert!(union.polygons.iter().all(|polygon| polygon.len() == 4));
    assert!(union.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(0)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(6)))
    }));
    assert!(union.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(10)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(13)))
    }));

    let mut stale = union.clone();
    stale.polygons[0][1] = p3(99, 0, 0);
    assert!(stale.validate_union_against_sources(&left, &right).is_err());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_union_materializes_single_bridged_strip_cluster() {
    let left = ExactMesh::from_i64_triangles_with_policy(
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
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 5, 0, 0, 5, 2, 0, 1, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &right).is_none());
    assert!(hypermesh::exact::arrange_coplanar_convex_surface_multi_union(&left, &right).is_none());
    let union = hypermesh::exact::arrange_coplanar_convex_surface_component_union(&left, &right)
        .expect("two separated rectangles plus exact bridge should form one retained strip loop");
    union.validate().unwrap();
    union
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::CoplanarArrangementOperation::Union,
        )
        .unwrap();
    assert_eq!(union.polygon.len(), 4);
    assert!(
        union
            .polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(0)))
    );
    assert!(
        union
            .polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(6)))
    );

    let mut stale = union.clone();
    stale.polygon[0] = p3(99, 0, 0);
    assert!(
        stale
            .validate_against_sources(
                &left,
                &right,
                hypermesh::exact::CoplanarArrangementOperation::Union,
            )
            .is_err()
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_union_materializes_edge_touching_strip_cluster() {
    let left = ExactMesh::from_i64_triangles_with_policy(
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
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(hypermesh::exact::arrange_coplanar_convex_surface_union(&left, &right).is_none());
    assert!(hypermesh::exact::arrange_coplanar_convex_surface_multi_union(&left, &right).is_none());
    let union = hypermesh::exact::arrange_coplanar_convex_surface_component_union(&left, &right)
        .expect("full-edge touching rectangles should replay as one exact strip loop");
    union.validate().unwrap();
    union
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::CoplanarArrangementOperation::Union,
        )
        .unwrap();
    assert_eq!(union.polygon.len(), 4);
    assert!(
        union
            .polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(0)))
    );
    assert!(
        union
            .polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(6)))
    );
    assert!(
        union
            .polygon
            .iter()
            .all(|point| real_eq(&point.y, &ExactReal::from(0))
                || real_eq(&point.y, &ExactReal::from(2)))
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementUnion
        }
    );

    let point_touching_right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_component_union(
            &left,
            &point_touching_right
        )
        .is_none()
    );
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_multi_union(&left, &point_touching_right)
            .is_none()
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_difference_materializes_simple_loop() {
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

    let difference = hypermesh::exact::arrange_coplanar_convex_surface_difference(&left, &right)
        .expect("overlapping convex sheets should materialize one simple difference loop");
    difference.validate().unwrap();
    difference
        .validate_against_sources(
            &left,
            &right,
            hypermesh::exact::CoplanarArrangementOperation::Difference,
        )
        .unwrap();
    assert_eq!(difference.polygon.len(), 6);
    assert!(!difference.mesh.triangles().is_empty());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
    );
    assert!(preflight.blocker.is_none());

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceArrangementDifference
        }
    );
    assert_eq!(
        result.mesh.vertices().len(),
        difference.mesh.vertices().len()
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_difference_materializes_multiple_components() {
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

    assert!(hypermesh::exact::arrange_coplanar_convex_surface_difference(&left, &right).is_none());
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_holed_difference(&left, &right).is_none()
    );
    let difference =
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(&left, &right)
            .expect("convex strip cut should produce two exact output components");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert!(difference.validate_against_sources(&right, &left).is_err());
    assert_eq!(difference.polygons.len(), 2);
    assert!(difference.polygons.iter().all(|polygon| polygon.len() == 4));
    assert_eq!(difference.mesh.vertices().len(), 8);
    assert_eq!(difference.mesh.triangles().len(), 4);

    let mut reversed_component = difference.clone();
    reversed_component.polygons[0].reverse();
    assert!(reversed_component.validate().is_err());
    let mut reversed_multi_mesh = difference.clone();
    reversed_multi_mesh.mesh = reverse_mesh_triangles(&reversed_multi_mesh.mesh);
    assert!(reversed_multi_mesh.validate().is_err());
    let mut cross_component_mesh = difference.clone();
    cross_component_mesh.mesh = mesh_with_cross_component_triangle(
        &cross_component_mesh.mesh,
        difference.polygons[0].len(),
    );
    assert!(cross_component_mesh.validate().is_err());
    let mut repeated_component_point = difference.clone();
    repeated_component_point.polygons[0][1] = repeated_component_point.polygons[0][0].clone();
    assert!(repeated_component_point.validate().is_err());
    let mut shared_component_point = difference.clone();
    shared_component_point.polygons[1][0] = shared_component_point.polygons[0][0].clone();
    assert!(shared_component_point.validate().is_err());
    let mut nonconvex_component = difference.clone();
    nonconvex_component.polygons[0] = vec![p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0), p3(0, 4, 0)];
    assert!(nonconvex_component.validate().is_err());
    let mut overlapping_components = difference.clone();
    overlapping_components.polygons[1] = vec![p3(0, 1, 0), p3(2, 1, 0), p3(2, 3, 0), p3(0, 3, 0)];
    assert!(overlapping_components.validate().is_err());
    let mut crossing_components = difference.clone();
    crossing_components.polygons[1] = vec![p3(-1, 1, 0), p3(2, 1, 0), p3(2, 3, 0), p3(-1, 3, 0)];
    assert!(crossing_components.validate().is_err());
    let mut partial_multi_mesh = difference.clone();
    let retained_points = partial_multi_mesh
        .polygons
        .iter()
        .flat_map(|polygon| polygon.iter().cloned())
        .collect::<Vec<_>>();
    partial_multi_mesh.mesh = partial_mesh_from_points(&retained_points);
    assert!(partial_multi_mesh.validate().is_err());
    if let Some(mesh) = boundary_mismatched_mesh(&difference.mesh) {
        let mut mismatched_boundary = difference.clone();
        mismatched_boundary.mesh = mesh;
        assert!(mismatched_boundary.validate().is_err());
    }

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
    );
    assert!(preflight.blocker.is_none());

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiDifference
        }
    );
    assert_eq!(result.mesh.vertices().len(), 8);
    assert_eq!(result.mesh.triangles().len(), 4);
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_difference_materializes_left_component_cut() {
    let left = ExactMesh::from_i64_triangles_with_policy(
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
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, 0, 3, -1, 0, 3, 3, 0, 1, 3, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(hypermesh::exact::arrange_coplanar_convex_surface_difference(&left, &right).is_none());
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_holed_difference(&left, &right).is_none()
    );
    let difference =
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(&left, &right)
            .expect("component-wise convex cut should retain cut and untouched left loops");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert_eq!(difference.polygons.len(), 2);
    assert!(difference.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(0)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(1)))
    }));
    assert!(difference.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(4)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(6)))
    }));

    let mut stale = difference.clone();
    stale.polygons[0][0] = p3(99, 0, 0);
    assert!(stale.validate_against_sources(&left, &right).is_err());

    let boundary_bridge = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(&left, &boundary_bridge)
            .is_none()
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiDifference
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_difference_materializes_multiple_component_cuts() {
    let left = ExactMesh::from_i64_triangles_with_policy(
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
    let right = ExactMesh::from_i64_triangles_with_policy(
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

    assert!(hypermesh::exact::arrange_coplanar_convex_surface_difference(&left, &right).is_none());
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_holed_difference(&left, &right).is_none()
    );
    let difference =
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(&left, &right)
            .expect("two independent right cutters should retain two cuts and one untouched loop");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert_eq!(difference.polygons.len(), 3);
    assert!(difference.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(0)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(1)))
    }));
    assert!(difference.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(4)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(5)))
    }));
    assert!(difference.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(8)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(10)))
    }));

    let two_cutters_one_component = ExactMesh::from_i64_triangles_with_policy(
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
    let wide_left = ExactMesh::from_i64_triangles_with_policy(
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
    let multi_cutter = hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(
        &wide_left,
        &two_cutters_one_component,
    )
    .expect("two full-span rectangular cutters should split one left component");
    multi_cutter.validate().unwrap();
    multi_cutter
        .validate_against_sources(&wide_left, &two_cutters_one_component)
        .unwrap();
    assert_eq!(multi_cutter.polygons.len(), 4);
    assert!(multi_cutter.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(0)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(1)))
    }));
    assert!(multi_cutter.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(2)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(4)))
    }));
    assert!(multi_cutter.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(5)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(6)))
    }));

    let corner_cutter_left = ExactMesh::from_i64_triangles_with_policy(
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
    let nonrectangular_corner_cutters = ExactMesh::from_i64_triangles_with_policy(
        &[-1, -1, 0, 3, -1, 0, -1, 3, 0, 7, 11, 0, 11, 7, 0, 11, 11, 0],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let nonrectangular_multi_cutter =
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(
            &corner_cutter_left,
            &nonrectangular_corner_cutters,
        )
        .expect("sequential exact corner cutters should retain convex remnants");
    nonrectangular_multi_cutter.validate().unwrap();
    nonrectangular_multi_cutter
        .validate_against_sources(&corner_cutter_left, &nonrectangular_corner_cutters)
        .unwrap();
    assert_eq!(nonrectangular_multi_cutter.polygons.len(), 2);
    assert!(
        nonrectangular_multi_cutter
            .polygons
            .iter()
            .any(|polygon| polygon.len() == 6)
    );
    assert_eq!(nonrectangular_multi_cutter.mesh.vertices().len(), 10);
    let nonrectangular_preflight = hypermesh::exact::preflight_boolean_exact(
        &corner_cutter_left,
        &nonrectangular_corner_cutters,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    nonrectangular_preflight.validate().unwrap();
    nonrectangular_preflight
        .validate_against_sources(&corner_cutter_left, &nonrectangular_corner_cutters)
        .unwrap();
    assert_eq!(
        nonrectangular_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
    );
    let nonrectangular_result = hypermesh::exact::boolean_exact(
        &corner_cutter_left,
        &nonrectangular_corner_cutters,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    nonrectangular_result.validate().unwrap();
    assert_eq!(
        nonrectangular_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiDifference
        }
    );

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
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(
            &corner_cutter_left,
            &nonconvex_multi_cutter_right,
        )
        .is_none()
    );
    let nonconvex_multi_cutter = hypermesh::exact::arrange_coplanar_surface_multi_difference(
        &corner_cutter_left,
        &nonconvex_multi_cutter_right,
    )
    .expect("nonconvex simple loop plus a far component should materialize");
    nonconvex_multi_cutter.validate().unwrap();
    nonconvex_multi_cutter
        .validate_difference_against_sources(&corner_cutter_left, &nonconvex_multi_cutter_right)
        .unwrap();
    assert_eq!(nonconvex_multi_cutter.polygons.len(), 2);
    assert!(
        nonconvex_multi_cutter
            .polygons
            .iter()
            .any(|polygon| polygon.len() == 9)
    );
    assert_eq!(nonconvex_multi_cutter.mesh.vertices().len(), 13);
    let mut reversed_nonconvex = nonconvex_multi_cutter.clone();
    reversed_nonconvex.polygons[0].reverse();
    assert!(reversed_nonconvex.validate().is_err());
    let nonconvex_preflight = hypermesh::exact::preflight_boolean_exact(
        &corner_cutter_left,
        &nonconvex_multi_cutter_right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    nonconvex_preflight.validate().unwrap();
    nonconvex_preflight
        .validate_against_sources(&corner_cutter_left, &nonconvex_multi_cutter_right)
        .unwrap();
    assert_eq!(
        nonconvex_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    let nonconvex_result = hypermesh::exact::boolean_exact(
        &corner_cutter_left,
        &nonconvex_multi_cutter_right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    nonconvex_result.validate().unwrap();
    assert_eq!(
        nonconvex_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceMultiDifference
        }
    );

    let partial_height_cutters = ExactMesh::from_i64_triangles_with_policy(
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
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(
            &wide_left,
            &partial_height_cutters,
        )
        .is_none()
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiDifference
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_difference_materializes_multiple_holes() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 2, 1, 0, 1, 2, 0, //
            7, 7, 0, 8, 7, 0, 7, 8, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_holed_difference(&left, &right).is_none()
    );
    let difference =
        hypermesh::exact::arrange_coplanar_convex_surface_multi_holed_difference(&left, &right)
            .expect("two contained triangle islands should materialize two retained holes");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert!(difference.validate_against_sources(&right, &left).is_err());
    assert_eq!(difference.outer.len(), 4);
    assert_eq!(difference.holes.len(), 2);
    assert!(difference.holes.iter().all(|hole| hole.len() == 3));
    assert_eq!(difference.mesh.vertices().len(), 10);
    assert!(!difference.mesh.triangles().is_empty());

    let mut reversed_hole = difference.clone();
    reversed_hole.holes[0].reverse();
    assert!(reversed_hole.validate().is_err());
    let mut shared_hole_point = difference.clone();
    shared_hole_point.holes[1][0] = shared_hole_point.holes[0][0].clone();
    assert!(shared_hole_point.validate().is_err());
    let mut outside_hole = difference.clone();
    outside_hole.holes[0] = vec![p3(11, 1, 0), p3(12, 1, 0), p3(11, 2, 0)];
    assert!(outside_hole.validate().is_err());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiHoledDifference
    );
    assert!(preflight.blocker.is_none());

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiHoledDifference
        }
    );
    assert_eq!(
        result.mesh.vertices().len(),
        difference.mesh.vertices().len()
    );
    assert_eq!(
        result.mesh.triangles().len(),
        difference.mesh.triangles().len()
    );

    let square_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let square_difference =
        hypermesh::exact::arrange_coplanar_convex_surface_multi_holed_difference(
            &left,
            &square_holes,
        )
        .expect("two connected square components should materialize two retained holes");
    square_difference.validate().unwrap();
    square_difference
        .validate_against_sources(&left, &square_holes)
        .unwrap();
    assert_eq!(square_difference.outer.len(), 4);
    assert_eq!(square_difference.holes.len(), 2);
    assert!(square_difference.holes.iter().all(|hole| hole.len() == 4));
    assert_eq!(square_difference.mesh.vertices().len(), 12);

    let square_result = hypermesh::exact::boolean_exact(
        &left,
        &square_holes,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    square_result.validate().unwrap();
    assert_eq!(
        square_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceMultiHoledDifference
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_difference_materializes_component_holes() {
    let left = ExactMesh::from_i64_triangles_with_policy(
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
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_multi_holed_difference(&left, &right)
            .is_none()
    );
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_multi_difference(&left, &right).is_none()
    );
    let difference =
        hypermesh::exact::arrange_coplanar_convex_surface_component_holed_difference(&left, &right)
            .expect("one holed component plus one untouched component should materialize");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert_eq!(difference.components.len(), 2);
    assert!(
        difference
            .components
            .iter()
            .any(|component| component.holes.len() == 2)
    );
    assert!(
        difference
            .components
            .iter()
            .any(|component| component.holes.is_empty())
    );
    assert_eq!(difference.mesh.vertices().len(), 16);

    let mut reversed_hole = difference.clone();
    let holed = reversed_hole
        .components
        .iter_mut()
        .find(|component| !component.holes.is_empty())
        .unwrap();
    holed.holes[0].reverse();
    assert!(reversed_hole.validate().is_err());

    let hole_and_cut = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            8, -1, 0, 11, -1, 0, 11, 11, 0, 8, 11, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let mixed_cut = hypermesh::exact::arrange_coplanar_convex_surface_component_holed_difference(
        &left,
        &hole_and_cut,
    )
    .expect("one partial cut plus strict holes should assign holes to output remnants");
    mixed_cut.validate().unwrap();
    mixed_cut
        .validate_against_sources(&left, &hole_and_cut)
        .unwrap();
    assert_eq!(mixed_cut.components.len(), 2);
    assert!(
        mixed_cut
            .components
            .iter()
            .any(|component| component.holes.len() == 1)
    );
    assert_eq!(mixed_cut.mesh.vertices().len(), 12);

    let mixed_result = hypermesh::exact::boolean_exact(
        &left,
        &hole_and_cut,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    mixed_result.validate().unwrap();
    assert_eq!(
        mixed_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::
                CoplanarConvexSurfaceComponentHoledDifference
        }
    );

    let single_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let single_mixed =
        hypermesh::exact::arrange_coplanar_convex_surface_component_holed_difference(
            &single_left,
            &hole_and_cut,
        )
        .expect("a single holed component with one side cut should materialize");
    single_mixed.validate().unwrap();
    single_mixed
        .validate_against_sources(&single_left, &hole_and_cut)
        .unwrap();
    assert_eq!(single_mixed.components.len(), 1);
    assert_eq!(single_mixed.components[0].holes.len(), 1);
    assert_eq!(single_mixed.mesh.vertices().len(), 8);
    let single_mixed_preflight = hypermesh::exact::preflight_boolean_exact(
        &single_left,
        &hole_and_cut,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    single_mixed_preflight.validate().unwrap();
    single_mixed_preflight
        .validate_against_sources(&single_left, &hole_and_cut)
        .unwrap();
    assert_eq!(
        single_mixed_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let hole_and_two_cuts = ExactMesh::from_i64_triangles_with_policy(
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
    let multi_cut_holed =
        hypermesh::exact::arrange_coplanar_convex_surface_component_holed_difference(
            &left,
            &hole_and_two_cuts,
        )
        .expect("full-span rectangle cutters should assign holes to retained remnants");
    multi_cut_holed.validate().unwrap();
    multi_cut_holed
        .validate_against_sources(&left, &hole_and_two_cuts)
        .unwrap();
    assert_eq!(multi_cut_holed.components.len(), 3);
    assert!(
        multi_cut_holed
            .components
            .iter()
            .any(|component| component.holes.len() == 1)
    );
    assert_eq!(multi_cut_holed.mesh.vertices().len(), 16);

    let hole_and_corner_cuts = ExactMesh::from_i64_triangles_with_policy(
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
    let nonrectangular_holed =
        hypermesh::exact::arrange_coplanar_convex_surface_component_holed_difference(
            &left,
            &hole_and_corner_cuts,
        )
        .expect("sequential exact corner cutters should retain a holed convex remnant");
    nonrectangular_holed.validate().unwrap();
    nonrectangular_holed
        .validate_against_sources(&left, &hole_and_corner_cuts)
        .unwrap();
    assert_eq!(nonrectangular_holed.components.len(), 2);
    assert!(
        nonrectangular_holed
            .components
            .iter()
            .any(|component| component.outer.len() == 6 && component.holes.len() == 1)
    );
    assert_eq!(nonrectangular_holed.mesh.vertices().len(), 14);
    let nonrectangular_holed_result = hypermesh::exact::boolean_exact(
        &left,
        &hole_and_corner_cuts,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    nonrectangular_holed_result.validate().unwrap();
    assert_eq!(
        nonrectangular_holed_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::
                CoplanarConvexSurfaceComponentHoledDifference
        }
    );

    let hole_and_partial_height_cuts = ExactMesh::from_i64_triangles_with_policy(
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
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_component_holed_difference(
            &left,
            &hole_and_partial_height_cuts,
        )
        .is_none()
    );

    let cutter_hole_contact = ExactMesh::from_i64_triangles_with_policy(
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
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_component_holed_difference(
            &single_left,
            &cutter_hole_contact,
        )
        .is_none()
    );
    let contact_difference =
        hypermesh::exact::arrange_coplanar_surface_cutter_hole_contact_difference(
            &single_left,
            &cutter_hole_contact,
        )
        .expect("side cutter touching a retained hole should open one exact nonconvex loop");
    contact_difference.validate().unwrap();
    contact_difference
        .validate_cutter_hole_contact_difference_against_sources(&single_left, &cutter_hole_contact)
        .unwrap();
    assert_eq!(contact_difference.polygon.len(), 10);
    assert!(
        contact_difference
            .polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(6)))
    );
    let mut reversed_contact = contact_difference.clone();
    reversed_contact.polygon.reverse();
    assert!(reversed_contact.validate().is_err());
    let mut convex_relabel = contact_difference.clone();
    convex_relabel.polygon = vec![p3(0, 0, 0), p3(10, 0, 0), p3(10, 10, 0), p3(0, 10, 0)];
    assert!(
        convex_relabel
            .validate_cutter_hole_contact_difference_against_sources(
                &single_left,
                &cutter_hole_contact,
            )
            .is_err()
    );
    let contact_preflight = hypermesh::exact::preflight_boolean_exact(
        &single_left,
        &cutter_hole_contact,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    contact_preflight.validate().unwrap();
    contact_preflight
        .validate_against_sources(&single_left, &cutter_hole_contact)
        .unwrap();
    assert_eq!(
        contact_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    let contact_result = hypermesh::exact::boolean_exact(
        &single_left,
        &cutter_hole_contact,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    contact_result
        .validate_operation_against_sources(
            &single_left,
            &cutter_hole_contact,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        contact_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::
                CoplanarSurfaceCutterHoleContactDifference
        }
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::
                CoplanarConvexSurfaceComponentHoledDifference
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_orthogonal_surface_cells_materialize_nonconvex_outputs() {
    let l_left = rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)]);
    let l_right = rect_surface_i64(&[(2, 2, 4, 4)]);
    assert!(hypermesh::exact::arrange_coplanar_convex_surface_union(&l_left, &l_right).is_none());
    let l_union = hypermesh::exact::arrange_coplanar_orthogonal_surface_union(&l_left, &l_right)
        .expect("full-edge L-shaped rectangle union should materialize as orthogonal cells");
    l_union.validate().unwrap();
    l_union.validate_against_sources(&l_left, &l_right).unwrap();
    assert_eq!(l_union.components.len(), 1);
    assert_eq!(l_union.components[0].holes.len(), 0);
    assert_eq!(l_union.components[0].outer.len(), 8);
    let mut reversed_union = l_union.clone();
    reversed_union.components[0].outer.reverse();
    assert!(reversed_union.validate().is_err());
    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &l_left,
        &l_right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_preflight.validate().unwrap();
    union_preflight
        .validate_against_sources(&l_left, &l_right)
        .unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion
    );
    let union_result = hypermesh::exact::boolean_exact(
        &l_left,
        &l_right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    union_result.validate().unwrap();
    assert_eq!(
        union_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceUnion
        }
    );

    let intersection_left = rect_surface_i64(&[(0, 0, 6, 2), (0, 2, 2, 6)]);
    let intersection_right = rect_surface_i64(&[(0, 0, 6, 6)]);
    let intersection = hypermesh::exact::arrange_coplanar_orthogonal_surface_intersection(
        &intersection_left,
        &intersection_right,
    )
    .expect("nonconvex rectangular source intersection should replay from cells");
    intersection.validate().unwrap();
    intersection
        .validate_against_sources(&intersection_left, &intersection_right)
        .unwrap();
    assert_eq!(intersection.components.len(), 1);
    assert_eq!(intersection.components[0].outer.len(), 6);
    let intersection_result = hypermesh::exact::boolean_exact(
        &intersection_left,
        &intersection_right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    intersection_result.validate().unwrap();
    assert_eq!(
        intersection_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceIntersection
        }
    );

    let holed_left = rect_surface_i64(&[(0, 0, 10, 10), (10, 0, 12, 2)]);
    let holed_right = rect_surface_i64(&[(2, 2, 4, 4)]);
    let holed_difference =
        hypermesh::exact::arrange_coplanar_orthogonal_surface_difference(&holed_left, &holed_right)
            .expect("nonconvex outer with an exact rectangular hole should materialize");
    holed_difference.validate().unwrap();
    holed_difference
        .validate_against_sources(&holed_left, &holed_right)
        .unwrap();
    assert_eq!(holed_difference.components.len(), 1);
    assert_eq!(holed_difference.components[0].holes.len(), 1);
    assert!(holed_difference.components[0].outer.len() > 4);
    let mut wrong_operation = holed_difference.clone();
    wrong_operation.operation = hypermesh::exact::CoplanarOrthogonalSurfaceOperation::Union;
    assert!(
        wrong_operation
            .validate_against_sources(&holed_left, &holed_right)
            .is_err()
    );
    let holed_result = hypermesh::exact::boolean_exact(
        &holed_left,
        &holed_right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    holed_result.validate().unwrap();
    assert_eq!(
        holed_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut:
                hypermesh::exact::ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceDifference
        }
    );

    let graph_left = rect_surface_i64(&[(0, 0, 12, 10)]);
    let graph_right = rect_surface_i64(&[(3, 3, 5, 5), (7, 3, 9, 5), (5, 4, 7, 5), (-1, 4, 3, 5)]);
    assert!(
        hypermesh::exact::arrange_coplanar_surface_cutter_hole_contact_difference(
            &graph_left,
            &graph_right,
        )
        .is_none()
    );
    let graph_difference =
        hypermesh::exact::arrange_coplanar_orthogonal_surface_difference(&graph_left, &graph_right)
            .expect("multi-rectangle cutter/hole contact graph should replay through cells");
    graph_difference.validate().unwrap();
    graph_difference
        .validate_against_sources(&graph_left, &graph_right)
        .unwrap();
    assert_eq!(graph_difference.components.len(), 1);
    assert!(graph_difference.components[0].holes.is_empty());
    assert!(graph_difference.components[0].outer.len() > 10);
    let graph_preflight = hypermesh::exact::preflight_boolean_exact(
        &graph_left,
        &graph_right,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    graph_preflight.validate().unwrap();
    assert_eq!(
        graph_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_coplanar_convex_surface_report_rejects_inconsistent_artifacts() {
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
    let equivalence = hypermesh::exact::certify_coplanar_convex_surface_equivalence(&left, &right)
        .expect("fixture should certify equivalent convex sheets");

    let rejected_with_certificate = hypermesh::exact::CoplanarConvexSurfaceReport {
        status: hypermesh::exact::CoplanarConvexSurfaceReportStatus::NotCertified,
        equivalence: Some(equivalence.clone()),
        containment: None,
    };
    assert_eq!(
        rejected_with_certificate.validate().unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::UnexpectedCertificate
    );

    let missing_equivalence = hypermesh::exact::CoplanarConvexSurfaceReport {
        status: hypermesh::exact::CoplanarConvexSurfaceReportStatus::Equivalent,
        equivalence: None,
        containment: None,
    };
    assert_eq!(
        missing_equivalence.validate().unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::MissingEquivalenceCertificate
    );

    let invalid_equivalence = hypermesh::exact::CoplanarConvexSurfaceReport {
        status: hypermesh::exact::CoplanarConvexSurfaceReportStatus::Equivalent,
        equivalence: Some(hypermesh::exact::CoplanarConvexSurfaceEquivalence {
            left_area2: hypermesh::exact::ExactReal::from(0),
            ..equivalence
        }),
        containment: None,
    };
    assert_eq!(
        invalid_equivalence.validate().unwrap_err(),
        hypermesh::exact::CoplanarConvexSurfaceReportError::InvalidEquivalenceCertificate
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_same_surface_report_retains_rejection_state() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let shifted = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 1, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let shifted_report = hypermesh::exact::certify_same_surface_report(&left, &shifted);
    shifted_report.validate().unwrap();
    shifted_report
        .validate_against_sources(&left, &shifted)
        .unwrap();
    assert_eq!(
        shifted_report.status,
        hypermesh::exact::ExactSameSurfaceStatus::VertexCoordinateMismatch
    );
    assert!(!shifted_report.predicates.is_empty());
    assert!(shifted_report.all_proof_producing());
    let mut corrupted_shifted_report = shifted_report.clone();
    corrupted_shifted_report.right_to_left.push(0);
    assert_eq!(
        corrupted_shifted_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );

    let different_topology = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0],
        &[0, 1, 2, 1, 3, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let count_report = hypermesh::exact::certify_same_surface_report(&left, &different_topology);
    count_report.validate().unwrap();
    count_report
        .validate_against_sources(&left, &different_topology)
        .unwrap();
    assert_eq!(
        count_report.status,
        hypermesh::exact::ExactSameSurfaceStatus::VertexCountMismatch
    );
    assert!(count_report.predicates.is_empty());
    let mut corrupted_count_report = count_report;
    corrupted_count_report.left_to_right.push(0);
    assert_eq!(
        corrupted_count_report.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::StatusEvidenceMismatch
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_multi_component_coplanar_intersection_materializes_component_hulls() {
    let left = ExactMesh::from_i64_triangles_with_policy(
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
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 1, 0, 10, 1, 0, 10, 3, 0, 2, 3, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_intersection(&left, &right).is_none()
    );
    let multi = hypermesh::exact::arrange_coplanar_convex_surface_multi_intersection(&left, &right)
        .expect("one convex right component should clip two retained left components");
    multi.validate().unwrap();
    multi
        .validate_intersection_against_sources(&left, &right)
        .unwrap();
    assert_eq!(multi.polygons.len(), 2);
    assert!(multi.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(2)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(4)))
    }));
    assert!(multi.polygons.iter().any(|polygon| {
        polygon
            .iter()
            .any(|point| real_eq(&point.x, &ExactReal::from(8)))
            && polygon
                .iter()
                .any(|point| real_eq(&point.x, &ExactReal::from(10)))
    }));

    let touching_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 8, 0, 0, 8, 4, 0, 4, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        hypermesh::exact::arrange_coplanar_convex_surface_multi_intersection(
            &left,
            &touching_right,
        )
        .is_none()
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarConvexSurfaceIntersection
        }
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_empty_operands() {
    let empty =
        ExactMesh::from_i64_triangles_with_policy(&[], &[], ValidationPolicy::ALLOW_BOUNDARY)
            .unwrap();
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let union = hypermesh::exact::boolean_exact(
        &empty,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(union.mesh.triangles(), mesh.triangles());

    let intersection = hypermesh::exact::boolean_exact(
        &mesh,
        &empty,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let left_empty_difference = hypermesh::exact::boolean_exact(
        &empty,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(left_empty_difference.mesh.triangles().is_empty());

    let right_empty_difference = hypermesh::exact::boolean_exact(
        &mesh,
        &empty,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(right_empty_difference.mesh.triangles(), mesh.triangles());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &empty,
        &mesh,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedEmptyOperand
    );
    assert_eq!(preflight.retained_events, 0);
}

#[test]
fn exact_convex_solid_facts_classify_points_and_vertex_sets() {
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

    let facts = certify_convex_solid(&outer);
    assert!(facts.is_certified_convex());
    assert!(facts.all_proof_producing());
    facts.validate().unwrap();
    facts.validate_against_source(&outer).unwrap();
    assert_eq!(
        classify_point_against_convex_solid(&p3(1, 1, 1), &outer),
        hypermesh::exact::ConvexSolidPointRelation::Inside
    );
    assert_eq!(
        classify_point_against_convex_solid(&p3(0, 0, 0), &outer),
        hypermesh::exact::ConvexSolidPointRelation::Boundary
    );
    assert_eq!(
        classify_point_against_convex_solid(&p3(11, 1, 1), &outer),
        hypermesh::exact::ConvexSolidPointRelation::Outside
    );
    let inside = classify_point_against_convex_solid_report(&p3(1, 1, 1), &outer);
    assert_eq!(
        inside.relation,
        hypermesh::exact::ConvexSolidPointRelation::Inside
    );
    assert_eq!(inside.predicates.len(), outer.triangles().len());
    assert!(inside.all_proof_producing());
    inside.validate().unwrap();
    inside
        .validate_against_sources(&p3(1, 1, 1), &outer)
        .unwrap();
    assert_eq!(
        inside
            .validate_against_sources(&p3(11, 1, 1), &outer)
            .unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::SourceReplayMismatch
    );

    let outside = classify_point_against_convex_solid_report(&p3(11, 1, 1), &outer);
    assert_eq!(
        outside.relation,
        hypermesh::exact::ConvexSolidPointRelation::Outside
    );
    assert!(!outside.predicates.is_empty());
    assert!(outside.predicates.len() <= outer.triangles().len());
    assert!(outside.all_proof_producing());
    outside.validate().unwrap();
    outside
        .validate_against_sources(&p3(11, 1, 1), &outer)
        .unwrap();

    assert_eq!(
        classify_mesh_vertices_against_convex_solid(&inner, &outer),
        hypermesh::exact::ConvexSolidMeshRelation::StrictlyInside
    );
    let containment = classify_mesh_vertices_against_convex_solid_report(&inner, &outer);
    assert_eq!(
        containment.relation,
        hypermesh::exact::ConvexSolidMeshRelation::StrictlyInside
    );
    assert!(containment.solid_facts.is_certified_convex());
    assert_eq!(containment.vertices.len(), inner.vertices().len());
    assert!(containment.all_proof_producing());
    containment.validate().unwrap();
    containment
        .validate_against_sources(&inner, &outer)
        .unwrap();
    assert_eq!(
        containment
            .validate_against_sources(&outer, &inner)
            .unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::SourceReplayMismatch
    );

    assert_eq!(
        classify_mesh_vertices_against_convex_solid(&outer, &inner),
        hypermesh::exact::ConvexSolidMeshRelation::Outside
    );
    let separated = classify_mesh_vertices_against_convex_solid_report(&outer, &inner);
    assert_eq!(
        separated.relation,
        hypermesh::exact::ConvexSolidMeshRelation::Outside
    );
    assert_eq!(separated.vertices.len(), outer.vertices().len());
    assert!(separated.all_proof_producing());
    separated.validate().unwrap();
    separated.validate_against_sources(&outer, &inner).unwrap();
}

#[test]
fn exact_convex_solid_reports_retain_not_certified_state() {
    let open = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let subject = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 1, 0, 1, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let point = classify_point_against_convex_solid_report(&p3(0, 0, 0), &open);
    assert_eq!(
        point.relation,
        hypermesh::exact::ConvexSolidPointRelation::NotCertifiedConvex
    );
    assert!(point.predicates.is_empty());
    point.validate().unwrap();

    let mesh = classify_mesh_vertices_against_convex_solid_report(&subject, &open);
    assert_eq!(
        mesh.relation,
        hypermesh::exact::ConvexSolidMeshRelation::NotCertifiedConvex
    );
    assert_eq!(
        mesh.solid_facts.orientation,
        hypermesh::exact::ClosedMeshOrientation::NotClosed
    );
    assert!(mesh.vertices.is_empty());
    mesh.validate().unwrap();
    mesh.validate_against_sources(&subject, &open).unwrap();

    let (closed_vertices, closed_triangles) = tetrahedron();
    let closed = ExactMesh::from_f64_triangles(&closed_vertices, &closed_triangles).unwrap();
    let closed_facts = certify_convex_solid(&closed);
    assert_eq!(
        closed_facts.validate_against_source(&open).unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::SourceReplayMismatch
    );
}

#[test]
fn exact_convex_solid_report_validation_rejects_inconsistent_artifacts() {
    let facts = hypermesh::exact::ConvexSolidFacts {
        orientation: hypermesh::exact::ClosedMeshOrientation::NotClosed,
        convexity: hypermesh::exact::ConvexSolidClassification::Convex,
        predicates: Vec::new(),
    };
    assert_eq!(
        facts.validate().unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::NotClosedStateMismatch
    );

    let point = hypermesh::exact::ConvexSolidPointClassification {
        relation: hypermesh::exact::ConvexSolidPointRelation::NotCertifiedConvex,
        predicates: vec![hypermesh::exact::PredicateUse::from_certificate(
            hyperlimit::PredicateCertificate::ExactRealFact,
        )],
    };
    assert_eq!(
        point.validate().unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::NonCertifiedPointHasPredicates
    );

    let solid_facts = hypermesh::exact::ConvexSolidFacts {
        orientation: hypermesh::exact::ClosedMeshOrientation::Positive,
        convexity: hypermesh::exact::ConvexSolidClassification::Convex,
        predicates: Vec::new(),
    };
    let mesh = hypermesh::exact::ConvexSolidMeshClassification {
        relation: hypermesh::exact::ConvexSolidMeshRelation::StrictlyInside,
        solid_facts,
        vertices: vec![hypermesh::exact::ConvexSolidPointClassification {
            relation: hypermesh::exact::ConvexSolidPointRelation::Outside,
            predicates: Vec::new(),
        }],
    };
    assert_eq!(
        mesh.validate().unwrap_err(),
        hypermesh::exact::ConvexSolidReportError::MeshRelationMismatch
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_handle_certified_convex_containment() {
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

    let union = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexContainment
        }
    );
    assert_eq!(union.mesh.triangles(), outer.triangles());
    assert_eq!(
        union.mesh.provenance().source.label,
        "exact convex containment union keeps outer left"
    );
    union
        .validate_operation_against_sources(
            &outer,
            &inner,
            hypermesh::exact::ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        union
            .validate_operation_against_sources(
                &outer,
                &inner,
                hypermesh::exact::ExactBooleanOperation::Intersection,
                ValidationPolicy::CLOSED,
                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SourceReplayMismatch
    );
    let mut relabeled_union = union.clone();
    relabeled_union.kind = hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
        shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexSeparated,
    };
    assert_eq!(
        relabeled_union
            .validate_operation_against_sources(
                &outer,
                &inner,
                hypermesh::exact::ExactBooleanOperation::Union,
                ValidationPolicy::CLOSED,
                hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::SourceReplayMismatch
    );
    let preflight = hypermesh::exact::preflight_boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedConvexContainment
    );
    assert_eq!(preflight.retained_events, 0);

    let intersection = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(intersection.mesh.triangles(), inner.triangles());

    let difference = hypermesh::exact::boolean_exact(
        &outer,
        &inner,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(difference.mesh.triangles().len(), 8);
    assert!(difference.mesh.facts().mesh.closed_manifold);

    let empty_difference = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert!(empty_difference.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_materialize_partial_convex_intersection() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[
            1, 1, 1, //
            5, 1, 1, //
            1, 5, 1, //
            1, 1, 5,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    assert!(
        hypermesh::exact::classify_mesh_vertices_against_convex_solid(&right, &left)
            != hypermesh::exact::ConvexSolidMeshRelation::StrictlyInside
    );
    let intersection = hypermesh::exact::intersect_closed_convex_solids(&left, &right)
        .expect("overlapping convex tetrahedra should clip to an exact convex solid");
    intersection.validate().unwrap();
    intersection
        .validate_against_sources(&left, &right)
        .unwrap();
    assert!(
        intersection
            .validate_against_sources(&right, &left)
            .is_err()
    );
    assert!(intersection.mesh.facts().mesh.closed_manifold);
    assert_eq!(intersection.mesh.vertices().len(), 4);
    assert_eq!(intersection.mesh.triangles().len(), 4);
    let mut stale_intersection = intersection.clone();
    stale_intersection.mesh = reverse_mesh_triangles(&stale_intersection.mesh);
    stale_intersection.validate().unwrap();
    assert!(
        stale_intersection
            .validate_against_sources(&left, &right)
            .is_err()
    );

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedConvexIntersection
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexIntersection
        }
    );
    assert_eq!(result.mesh.triangles().len(), 4);

    let union_preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
    )
    .unwrap();
    union_preflight.validate().unwrap();
    assert_eq!(
        union_preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedWindingMaterialized
    );
    let graph = build_intersection_graph(&left, &right).unwrap();
    let (_cell_region_plan, cell_triangulations) =
        hypermesh::exact::triangulate_all_face_cells_with_cdt(&graph, &left, &right)
            .unwrap()
            .expect("overlapping tetrahedra should produce exact face cells");
    let left_cut_face = cell_triangulations
        .iter()
        .find(|triangulation| triangulation.side == MeshSide::Left && triangulation.face == 2)
        .expect("left cut face should be triangulated into cells");
    assert_eq!(left_cut_face.triangles.len() / 3, 7);
    let cell_classifications =
        hypermesh::exact::classify_triangulated_regions_against_opposite_meshes(
            &cell_triangulations,
            &left,
            &right,
        )
        .unwrap();
    assert!(cell_classifications.iter().any(|classification| {
        classification.region_side == MeshSide::Left
            && classification.region_face == 2
            && classification.relation == hypermesh::exact::ExactVolumetricRegionRelation::Inside
    }));
    assert!(cell_classifications.iter().any(|classification| {
        classification.region_side == MeshSide::Left
            && classification.region_face == 2
            && classification.relation == hypermesh::exact::ExactVolumetricRegionRelation::Outside
    }));

    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::WindingMaterialized {
            operation: hypermesh::exact::ExactBooleanOperation::Union
        }
    );
    assert!(union.mesh.facts().mesh.closed_manifold);
    assert!(!union.volumetric_classifications.is_empty());
    assert!(
        union
            .volumetric_classifications
            .iter()
            .all(|classification| classification.relation.is_strictly_decided())
    );
    let mut missing_volumetric = union.clone();
    missing_volumetric.volumetric_classifications.clear();
    assert_eq!(
        missing_volumetric.validate().unwrap_err(),
        hypermesh::exact::ExactReportValidationError::MissingVolumetricClassifications
    );
    let mut stale_volumetric = union.clone();
    stale_volumetric.volumetric_classifications[0].representative = Point3::new(
        ExactReal::from(99),
        ExactReal::from(99),
        ExactReal::from(99),
    );
    assert_eq!(
        stale_volumetric
            .validate_against_sources(&left, &right)
            .unwrap_err(),
        hypermesh::exact::ExactReportValidationError::InvalidVolumetricClassification(
            hypermesh::exact::ExactVolumetricRegionError::SourceReplayMismatch
        )
    );

    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            hypermesh::exact::ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            hypermesh::exact::ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::WindingMaterialized {
            operation: hypermesh::exact::ExactBooleanOperation::Difference
        }
    );
    assert!(
        difference
            .assembly
            .triangles
            .iter()
            .any(|triangle| triangle.orientation
                == hypermesh::exact::ExactOutputTriangleOrientation::ReverseSource)
    );
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_materialize_single_cap_convex_difference() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let cutter = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            1, 0, 0, //
            0, 1, 0, //
            0, 0, 1,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let difference = hypermesh::exact::subtract_closed_convex_solids_single_cap(&left, &cutter)
        .expect("small tetra at the origin should remove one triangular cap");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &cutter).unwrap();
    assert_eq!(difference.mesh.vertices().len(), 6);
    assert_eq!(difference.mesh.triangles().len(), 8);
    assert!(difference.mesh.facts().mesh.closed_manifold);
    assert!(difference.validate_against_sources(&cutter, &left).is_err());

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &left,
        &cutter,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedConvexSingleCapDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &cutter,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexSingleCapDifference
        }
    );
    assert_eq!(
        result.mesh.triangles().len(),
        difference.mesh.triangles().len()
    );

    let unsupported_reverse = hypermesh::exact::boolean_exact(
        &cutter,
        &left,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    );
    assert!(unsupported_reverse.is_err());
}

#[cfg(feature = "exact-triangulation")]
#[test]
fn exact_named_booleans_materialize_polygonal_cap_convex_difference() {
    let cube = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            2, 0, 0, //
            2, 2, 0, //
            0, 2, 0, //
            0, 0, 2, //
            2, 0, 2, //
            2, 2, 2, //
            0, 2, 2,
        ],
        &[
            0, 2, 1, 0, 3, 2, // bottom
            4, 5, 6, 4, 6, 7, // top
            0, 1, 5, 0, 5, 4, // y-min
            1, 2, 6, 1, 6, 5, // x-max
            2, 3, 7, 2, 7, 6, // y-max
            3, 0, 4, 3, 4, 7, // x-min
        ],
    )
    .unwrap();
    let cutter = ExactMesh::from_i64_triangles(
        &[
            -10, -10, -10, //
            23, -10, -10, //
            -10, 23, -10, //
            -10, -10, 23,
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let difference = hypermesh::exact::subtract_closed_convex_solids_single_cap(&cube, &cutter)
        .expect("one large tetrahedron face should cut a hexagonal cap from the cube");
    difference.validate().unwrap();
    difference.validate_against_sources(&cube, &cutter).unwrap();
    assert_eq!(difference.mesh.vertices().len(), 15);
    assert_eq!(difference.mesh.triangles().len(), 26);
    assert!(difference.mesh.facts().mesh.closed_manifold);

    let preflight = hypermesh::exact::preflight_boolean_exact(
        &cube,
        &cutter,
        hypermesh::exact::ExactBooleanOperation::Difference,
    )
    .unwrap();
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedConvexSingleCapDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &cube,
        &cutter,
        hypermesh::exact::ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    result.validate().unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexSingleCapDifference
        }
    );
    assert_eq!(
        result.mesh.triangles().len(),
        difference.mesh.triangles().len()
    );
}

#[test]
fn exact_split_plan_validation_rejects_unresolved_and_malformed_topology() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(0, 0, 0),
            uses: Vec::new(),
        }],
        edge_chains: vec![SplitEdgeChain {
            side: MeshSide::Left,
            edge: [0, 1],
            nodes: vec![
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Right,
                    vertex: 0,
                },
                SplitEdgeNode::GraphVertex { graph_vertex: 7 },
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Left,
                    vertex: 2,
                },
            ],
        }],
        unresolved_vertex_lookups: 1,
        unresolved_equalities: 1,
        unknown_orderings: 1,
    };

    let report = topology_plan.validate();

    assert!(!report.is_valid());
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnknownOrdering)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::UnresolvedEquality)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::GraphVertexOutOfRange)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::WrongChainEnd)
    );
}

#[test]
fn exact_split_topology_validation_rechecks_graph_vertex_source_facts() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(1, 0, 0),
            uses: vec![ExactGraphVertexUse {
                side: MeshSide::Left,
                edge: [0, 1],
                face_pair: [0, 0],
                plane_face: 0,
                parameter: half(),
                parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(2),
                    denominator: ExactReal::from(3),
                },
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
            }],
        }],
        edge_chains: vec![SplitEdgeChain {
            side: MeshSide::Left,
            edge: [0, 1],
            nodes: vec![
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Left,
                    vertex: 0,
                },
                SplitEdgeNode::GraphVertex { graph_vertex: 0 },
                SplitEdgeNode::OriginalVertex {
                    side: MeshSide::Left,
                    vertex: 1,
                },
            ],
        }],
        unresolved_vertex_lookups: 0,
        unresolved_equalities: 0,
        unknown_orderings: 0,
    };

    let report = topology_plan.validate();

    assert!(!report.is_valid());
    report.validate().unwrap();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
            && diagnostic.graph_vertex == Some(0)
    }));
}

#[test]
fn exact_face_split_plan_validation_rejects_duplicate_and_unmatched_edges() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(0, 0, 0),
            uses: Vec::new(),
        }],
        edge_chains: Vec::new(),
        unresolved_vertex_lookups: 0,
        unresolved_equalities: 0,
        unknown_orderings: 0,
    };
    let face_plan = ExactFaceSplitPlan {
        faces: vec![FaceSplitPlan {
            side: MeshSide::Left,
            face: 0,
            edges: vec![
                FaceSplitEdge {
                    edge: [0, 1],
                    graph_vertices: vec![0],
                },
                FaceSplitEdge {
                    edge: [0, 1],
                    graph_vertices: vec![3],
                },
            ],
        }],
    };

    let report = face_plan.validate_against_topology(&topology_plan);

    assert!(!report.is_valid());
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::DuplicateFaceSplitEdge)
    );
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == SplitPlanDiagnosticKind::GraphVertexOutOfRange)
    );
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingFaceSplitSourceUse
    }));
}

#[test]
fn exact_face_split_plan_validation_rechecks_source_construction_facts() {
    let topology_plan = ExactSplitTopologyPlan {
        graph_vertices: vec![ExactGraphVertex {
            point: p3(1, 0, 0),
            uses: vec![ExactGraphVertexUse {
                side: MeshSide::Left,
                edge: [0, 1],
                face_pair: [0, 0],
                plane_face: 0,
                parameter: half(),
                parameter_ratio: hypermesh::exact::SegmentPlaneParameterRatio {
                    numerator: ExactReal::from(2),
                    denominator: ExactReal::from(3),
                },
                endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
            }],
        }],
        edge_chains: Vec::new(),
        unresolved_vertex_lookups: 0,
        unresolved_equalities: 0,
        unknown_orderings: 0,
    };
    let face_plan = ExactFaceSplitPlan {
        faces: vec![FaceSplitPlan {
            side: MeshSide::Left,
            face: 0,
            edges: vec![FaceSplitEdge {
                edge: [0, 1],
                graph_vertices: vec![0],
            }],
        }],
    };

    let report = face_plan.validate_against_topology(&topology_plan);

    assert!(!report.is_valid());
    report.validate().unwrap();
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::InvalidConstructionRatio
            && diagnostic.graph_vertex == Some(0)
    }));
    assert!(!report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::MissingFaceSplitSourceUse
    }));
}

#[test]
fn exact_face_split_geometry_validation_rejects_off_plane_boundary_node() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let geometry = ExactFaceSplitGeometryPlan {
        faces: vec![FaceSplitGeometry {
            side: MeshSide::Left,
            face: 0,
            triangle: [0, 1, 2],
            boundary_chains: vec![FaceSplitBoundaryChain {
                edge: [0, 1],
                nodes: vec![FaceSplitBoundaryNode::GraphVertex {
                    graph_vertex: 0,
                    point: p3(1, 0, 1),
                }],
            }],
        }],
    };

    let report = geometry.validate_boundary_incidence(&mesh, &mesh);

    assert!(!report.is_valid());
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::BoundaryNodeOffFacePlane
    }));
}

#[test]
fn exact_face_region_validation_rejects_duplicate_boundary_nodes() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let point = p3(0, 0, 0);
    let region_plan = hypermesh::exact::ExactFaceRegionPlan {
        regions: vec![FaceRegionBoundary {
            side: MeshSide::Left,
            face: 0,
            triangle: [0, 1, 2],
            boundary: vec![
                FaceSplitBoundaryNode::OriginalVertex {
                    vertex: 0,
                    point: point.clone(),
                },
                FaceSplitBoundaryNode::OriginalVertex { vertex: 0, point },
            ],
        }],
    };

    let report = region_plan.validate(&mesh, &mesh);

    assert!(!report.is_valid());
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::EmptyOrShortRegionBoundary
    }));
    assert!(report.diagnostics.iter().any(|diagnostic| {
        diagnostic.kind == SplitPlanDiagnosticKind::DuplicateConsecutiveRegionNode
    }));
}

#[test]
fn exact_intersection_graph_records_coplanar_edge_and_vertex_events() {
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

    let graph = build_intersection_graph(&left, &right).unwrap();

    graph.validate().unwrap();
    graph.validate_against_sources(&left, &right).unwrap();
    graph.face_pairs[0]
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(graph.face_pairs.len(), 1);
    assert_eq!(
        graph.face_pairs[0].relation,
        MeshFacePairRelation::CoplanarOverlapping
    );
    assert!(graph.face_pairs[0].projection.is_some());
    assert!(
        graph.face_pairs[0]
            .events
            .iter()
            .any(|event| matches!(event, IntersectionEvent::CoplanarEdge { .. }))
    );
    assert!(
        graph.face_pairs[0]
            .events
            .iter()
            .any(|event| matches!(event, IntersectionEvent::CoplanarVertex { .. }))
    );
    let overlap_graph = graph.face_pairs[0]
        .coplanar_overlap_graph()
        .expect("coplanar pair should expose an overlap graph");
    overlap_graph.validate().unwrap();
    overlap_graph
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(overlap_graph.left_face, 0);
    assert_eq!(overlap_graph.right_face, 0);
    assert_eq!(
        overlap_graph.relation,
        MeshFacePairRelation::CoplanarOverlapping
    );
    assert!(!overlap_graph.edge_overlaps.is_empty());
    assert!(!overlap_graph.vertex_overlaps.is_empty());
    assert_eq!(graph.coplanar_overlap_graphs(), vec![overlap_graph.clone()]);
    let mut stale_overlap_graph = overlap_graph.clone();
    stale_overlap_graph.left_face = usize::MAX;
    assert_eq!(
        stale_overlap_graph
            .validate_against_sources(&left, &right)
            .unwrap_err(),
        hypermesh::exact::CoplanarOverlapGraphValidationError::SourceReplayMismatch
    );
    let split_plan = graph.coplanar_overlap_split_plan(&left, &right).unwrap();
    split_plan.validate().unwrap();
    split_plan.validate_against_sources(&left, &right).unwrap();
    assert_eq!(split_plan.graphs.len(), 1);
    split_plan.graphs[0]
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(split_plan.graphs[0].left_face, 0);
    assert_eq!(split_plan.graphs[0].right_face, 0);
    assert!(
        split_plan.graphs[0]
            .edge_splits
            .iter()
            .any(|split| split.interval_overlap || !split.points.is_empty())
    );
    assert!(
        split_plan.graphs[0]
            .edge_splits
            .iter()
            .filter(|split| split.interval_overlap)
            .all(|split| split.interval.as_ref().is_some_and(|interval| {
                compare_reals(
                    &interval.endpoints[0].left_parameter,
                    &interval.endpoints[1].left_parameter,
                )
                .value()
                    == Some(Ordering::Less)
            }))
    );
    let mut stale_split_plan = split_plan.clone();
    stale_split_plan.graphs[0].left_face = usize::MAX;
    assert_eq!(
        stale_split_plan
            .validate_against_sources(&left, &right)
            .unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::SourceReplayMismatch
    );

    let readiness = graph
        .coplanar_arrangement_readiness_report(&left, &right)
        .unwrap();
    readiness.validate().unwrap();
    readiness.validate_against_sources(&left, &right).unwrap();
    assert!(readiness.needs_planar_cells());
    assert_eq!(
        readiness.status,
        hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells
    );
    assert_eq!(readiness.graph_count, 1);
    assert_eq!(readiness.overlapping_graphs, 1);
    assert!(readiness.edge_overlap_count > 0);
    let separated = ExactMesh::from_i64_triangles_with_policy(
        &[5, 0, 0, 7, 0, 0, 5, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        readiness
            .validate_against_sources(&left, &separated)
            .unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::SourceReplayMismatch
    );

    let mut relabeled_graph = graph.clone();
    relabeled_graph.face_pairs[0].left_face = usize::MAX;
    assert!(relabeled_graph.validate().is_ok());
    assert!(
        relabeled_graph
            .coplanar_arrangement_readiness_report(&left, &right)
            .is_err()
    );

    let mut relabeled_edge_graph = graph.clone();
    if let Some(IntersectionEvent::CoplanarEdge { left_edge, .. }) = relabeled_edge_graph
        .face_pairs
        .first_mut()
        .and_then(|pair| {
            pair.events
                .iter_mut()
                .find(|event| matches!(event, IntersectionEvent::CoplanarEdge { .. }))
        })
    {
        *left_edge = [usize::MAX, usize::MAX];
        assert!(relabeled_edge_graph.validate().is_ok());
        assert!(
            relabeled_edge_graph
                .coplanar_arrangement_readiness_report(&left, &right)
                .is_err()
        );
    }
}

#[test]
fn exact_coplanar_overlap_graph_validation_rejects_malformed_records() {
    let empty = hypermesh::exact::CoplanarOverlapGraph {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::CoplanarTouching,
        projection: hypermesh::exact::CoplanarProjection::Xy,
        edge_overlaps: Vec::new(),
        vertex_overlaps: Vec::new(),
    };
    assert_eq!(
        empty.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapGraphValidationError::EmptyOverlapGraph
    );

    let disjoint_edge = hypermesh::exact::CoplanarOverlapGraph {
        edge_overlaps: vec![hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::Disjoint,
        }],
        ..empty.clone()
    };
    assert_eq!(
        disjoint_edge.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapGraphValidationError::DisjointEdgeOverlap
    );

    let same_side_vertex = hypermesh::exact::CoplanarOverlapGraph {
        edge_overlaps: Vec::new(),
        vertex_overlaps: vec![hypermesh::exact::CoplanarVertexOverlap {
            vertex_side: MeshSide::Left,
            vertex: 0,
            triangle_side: MeshSide::Left,
            triangle_face: 0,
            location: hyperlimit::TriangleLocation::Inside,
        }],
        ..empty
    };
    assert_eq!(
        same_side_vertex.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapGraphValidationError::SameSideVertexOverlap
    );
}

#[test]
fn exact_coplanar_arrangement_readiness_validation_rejects_bad_counts() {
    let mut no_overlap = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NoCoplanarOverlap,
        graph_count: 0,
        overlapping_graphs: 0,
        touching_graphs: 0,
        edge_overlap_count: 0,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 0,
        interval_endpoint_count: 0,
    };
    no_overlap.validate().unwrap();

    no_overlap.edge_overlap_count = 1;
    assert_eq!(
        no_overlap.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::NoOverlapWithEvidence
    );

    let mismatch = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells,
        graph_count: 2,
        overlapping_graphs: 1,
        touching_graphs: 0,
        edge_overlap_count: 1,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 0,
        interval_endpoint_count: 0,
    };
    assert_eq!(
        mismatch.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::GraphCountMismatch
    );

    let missing_overlap = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells,
        graph_count: 1,
        overlapping_graphs: 0,
        touching_graphs: 1,
        edge_overlap_count: 1,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 0,
        interval_endpoint_count: 0,
    };
    assert_eq!(
        missing_overlap.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::NeedsCellsMissingOverlap
    );

    let impossible_split_summary = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells,
        graph_count: 1,
        overlapping_graphs: 1,
        touching_graphs: 0,
        edge_overlap_count: 1,
        vertex_overlap_count: 0,
        point_split_count: 1,
        interval_overlap_count: 1,
        interval_endpoint_count: 2,
    };
    assert_eq!(
        impossible_split_summary.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::SplitCountExceedsEdgeEvidence
    );

    let impossible_interval_endpoint_count = hypermesh::exact::CoplanarArrangementReadinessReport {
        status: hypermesh::exact::CoplanarArrangementReadinessStatus::NeedsPlanarCells,
        graph_count: 1,
        overlapping_graphs: 1,
        touching_graphs: 0,
        edge_overlap_count: 1,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 1,
        interval_endpoint_count: 1,
    };
    assert_eq!(
        impossible_interval_endpoint_count.validate().unwrap_err(),
        hypermesh::exact::CoplanarArrangementReadinessValidationError::IntervalEndpointCountMismatch
    );
}

#[test]
fn exact_coplanar_overlap_split_validation_rejects_malformed_records() {
    let point = Point3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(0));
    let missing_point = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::Proper,
        },
        points: Vec::new(),
        interval_overlap: false,
        interval: None,
    };
    assert_eq!(
        missing_point.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::MissingPointConstruction
    );

    let missing_interval = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::CollinearOverlap,
        },
        points: Vec::new(),
        interval_overlap: false,
        interval: None,
    };
    assert_eq!(
        missing_interval.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::MissingIntervalConstruction
    );

    let proper_with_endpoint_parameter = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::Proper,
        },
        points: vec![hypermesh::exact::CoplanarEdgeSplitPoint {
            point: point.clone(),
            left_parameter: ExactReal::from(0),
            right_parameter: half(),
        }],
        interval_overlap: false,
        interval: None,
    };
    assert_eq!(
        proper_with_endpoint_parameter.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::ProperCrossingEndpointParameter
    );

    let endpoint_with_interior_parameters = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::EndpointTouch,
        },
        points: vec![hypermesh::exact::CoplanarEdgeSplitPoint {
            point,
            left_parameter: half(),
            right_parameter: half(),
        }],
        interval_overlap: false,
        interval: None,
    };
    assert_eq!(
        endpoint_with_interior_parameters.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::EndpointTouchWithoutEndpointParameter
    );

    let missing_interval_endpoints = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::Identical,
        },
        points: Vec::new(),
        interval_overlap: true,
        interval: None,
    };
    assert_eq!(
        missing_interval_endpoints.validate().unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::MissingIntervalEndpoints
    );

    let corrupted_point = hypermesh::exact::CoplanarEdgeSplitConstruction {
        overlap: hypermesh::exact::CoplanarEdgeOverlap {
            left_edge: [0, 1],
            right_edge: [2, 3],
            relation: hyperlimit::SegmentIntersection::Proper,
        },
        points: vec![hypermesh::exact::CoplanarEdgeSplitPoint {
            point: Point3::new(ExactReal::from(2), ExactReal::from(0), ExactReal::from(0)),
            left_parameter: half(),
            right_parameter: half(),
        }],
        interval_overlap: false,
        interval: None,
    };
    let left_edge = [
        Point3::new(ExactReal::from(0), ExactReal::from(0), ExactReal::from(0)),
        Point3::new(ExactReal::from(1), ExactReal::from(0), ExactReal::from(0)),
    ];
    let right_edge = [
        Point3::new(half(), ExactReal::from(-1), ExactReal::from(0)),
        Point3::new(half(), ExactReal::from(1), ExactReal::from(0)),
    ];
    assert_eq!(
        corrupted_point
            .validate_against_edges(left_edge, right_edge)
            .unwrap_err(),
        hypermesh::exact::CoplanarOverlapSplitValidationError::SplitPointDoesNotMatchLeftParameter
    );
}

#[test]
#[cfg(feature = "exact-triangulation")]
fn exact_coplanar_split_plan_replays_interval_endpoints_against_sources() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
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

    let graph = build_intersection_graph(&left, &right).unwrap();
    let split_plan = graph.coplanar_overlap_split_plan(&left, &right).unwrap();
    split_plan.validate_against_meshes(&left, &right).unwrap();

    let mut corrupted = split_plan.clone();
    let interval = corrupted
        .graphs
        .iter_mut()
        .flat_map(|graph| graph.edge_splits.iter_mut())
        .find_map(|split| split.interval.as_mut())
        .expect("overlapping collinear edge interval");
    interval.endpoints[0].point =
        Point3::new(ExactReal::from(-1), ExactReal::from(0), ExactReal::from(0));

    let err = corrupted
        .validate_against_meshes(&left, &right)
        .unwrap_err();
    assert!(err.diagnostics.iter().any(|diagnostic| {
        diagnostic
            .message
            .contains("SplitPointDoesNotMatchLeftParameter")
    }));
}

#[test]
fn exact_mesh_face_pair_classifier_rejects_out_of_range_faces() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let err = classify_mesh_face_pair(&mesh, 1, &mesh, 0).unwrap_err();
    assert!(
        err.diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[test]
fn exact_mesh_face_pair_batch_retains_only_graph_construction_pairs() {
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

    let retained = classify_mesh_face_pairs(&left, &right).unwrap();

    assert_eq!(retained.len(), 1);
    assert_eq!(retained[0].left_face, 0);
    assert_eq!(retained[0].right_face, 0);
    assert_eq!(retained[0].relation, MeshFacePairRelation::Candidate);
}

#[test]
fn exact_mesh_rejects_non_finite_lossy_input_before_predicates() {
    let (mut pos, idx) = tetrahedron();
    pos[2] = f64::NAN;

    let err = ExactMesh::from_f64_triangles(&pos, &idx).unwrap_err();
    assert_eq!(err.diagnostics[0].kind, DiagnosticKind::NonFiniteCoordinate);
    assert_eq!(err.diagnostics[0].severity, Severity::Error);
    assert_eq!(err.diagnostics[0].coordinate, Some(2));
}

#[test]
fn exact_mesh_rejects_out_of_range_indices_without_panicking() {
    let (pos, mut idx) = tetrahedron();
    idx[4] = 99;

    let err = ExactMesh::from_f64_triangles(&pos, &idx).unwrap_err();
    assert!(
        err.diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::IndexOutOfBounds)
    );
}

#[test]
fn exact_validation_reports_boundary_and_duplicate_directed_edges() {
    let points = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0), p3(1, 1, 0)];
    let triangles = [[0, 1, 2], [1, 3, 2]];
    let report = validate_triangles(&points, &triangles);

    assert!(!report.is_valid());
    assert_eq!(report.facts.mesh.boundary_edges, 4);
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::BoundaryEdge)
    );
}

#[test]
fn exact_validation_boundary_policy_allows_disk_links() {
    let points = [p3(0, 0, 0), p3(1, 0, 0), p3(0, 1, 0)];
    let report =
        validate_triangles_with_policy(&points, &[[0, 1, 2]], ValidationPolicy::ALLOW_BOUNDARY);

    assert!(report.is_valid());
    assert!(!report.facts.mesh.closed_manifold);
    assert_eq!(report.facts.mesh.boundary_edges, 3);
    assert!(
        report
            .facts
            .vertices
            .iter()
            .all(|vertex| vertex.link == VertexLinkKind::Disk)
    );
}

#[test]
fn exact_mesh_boundary_policy_constructs_open_mesh_explicitly() {
    let pos = vec![0, 0, 0, 1, 0, 0, 0, 1, 0];
    let idx = vec![0, 1, 2];
    let mesh =
        ExactMesh::from_i64_triangles_with_policy(&pos, &idx, ValidationPolicy::ALLOW_BOUNDARY)
            .unwrap();

    assert_eq!(mesh.facts().mesh.boundary_edges, 3);
    assert!(!mesh.facts().mesh.closed_manifold);
}

#[test]
fn exact_validation_reports_bow_tie_vertex_link() {
    let points = [
        p3(0, 0, 0),
        p3(1, 0, 0),
        p3(0, 1, 0),
        p3(-1, 0, 0),
        p3(0, 0, 1),
        p3(0, 1, 1),
        p3(0, 0, 2),
    ];
    let report = validate_triangles(&points, &[[0, 1, 2], [0, 2, 3], [0, 4, 5], [0, 5, 6]]);

    assert_eq!(report.facts.vertices[0].link, VertexLinkKind::NonManifold);
    assert_eq!(report.facts.mesh.non_manifold_vertices, 1);
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::NonManifoldVertexLink)
    );
}

#[test]
fn exact_validation_rejects_collinear_triangle_without_epsilon() {
    let points = [p3(0, 0, 0), p3(1, 1, 1), p3(2, 2, 2)];
    let report = validate_triangles(&points, &[[0, 1, 2]]);

    assert_eq!(report.facts.mesh.degenerate_triangles, 1);
    assert!(
        report
            .diagnostics
            .iter()
            .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
    );
}

proptest! {
    #[test]
    fn malformed_f64_imports_never_panic(
        coords in proptest::collection::vec(any::<f64>(), 0..80),
        indices in proptest::collection::vec(any::<usize>(), 0..80),
    ) {
        let _ = ExactMesh::from_f64_triangles(&coords, &indices);
    }

    #[test]
    fn repeated_triangle_vertices_are_rejected(a in 0usize..8, b in 0usize..8) {
        let points = vec![
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
        ];
        let idx = vec![a % 3, b % 3, a % 3];
        let err = ExactMesh::from_f64_triangles(&points, &idx).unwrap_err();
        prop_assert!(
            err.diagnostics
                .iter()
                .any(|diagnostic| diagnostic.kind == DiagnosticKind::DegenerateTriangle)
        );
    }

    #[test]
    fn generated_integer_vertical_segments_cross_z_plane_exactly(x in -16i32..16, y in -16i32..16, h in 1i32..16) {
        let points = [
            p3(0, 0, 0),
            p3(1, 0, 0),
            p3(0, 1, 0),
            p3(x, y, -h),
            p3(x, y, h),
        ];
        let event = intersect_segment_with_face_plane(&points, [0, 1, 2], [3, 4]);

        prop_assert_eq!(event.relation, SegmentPlaneRelation::ProperCrossing);
        prop_assert!(event.validate().is_ok());
        prop_assert!(event.all_proof_producing());
        prop_assert!(real_eq(event.parameter.as_ref().unwrap(), &half()));
        let point = event.point.as_ref().unwrap();
        prop_assert!(real_eq(&point.x, &ExactReal::from(x)));
        prop_assert!(real_eq(&point.y, &ExactReal::from(y)));
        prop_assert!(real_eq(&point.z, &ExactReal::from(0)));
    }

    #[test]
    fn generated_triangle_pairs_with_straddling_vertex_remain_candidates(h in 1i32..16) {
        let points = [
            p3(0, 0, 0),
            p3(2, 0, 0),
            p3(0, 2, 0),
            p3(0, 0, -h),
            p3(2, 0, h),
            p3(0, 2, h),
        ];
        let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

        prop_assert_eq!(classification.relation, TriangleTriangleRelation::Candidate);
        prop_assert!(classification.validate().is_ok());
        prop_assert!(classification.all_proof_producing());
        prop_assert_eq!(classification.right_edge_events.len(), 3);
    }
}

fn p3(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

#[cfg(feature = "exact-triangulation")]
fn partial_mesh_from_points(points: &[Point3]) -> ExactMesh {
    assert!(points.len() >= 3);
    let vertices = points
        .iter()
        .map(|point| ExactPoint3::new(point.x.clone(), point.y.clone(), point.z.clone()))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        vec![Triangle([0, 1, 2])],
        SourceProvenance::exact("adversarial partial surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

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
fn fan_mesh_from_points(points: &[Point3]) -> ExactMesh {
    assert!(points.len() >= 3);
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
        SourceProvenance::exact("adversarial fan surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn reverse_mesh_triangles(mesh: &ExactMesh) -> ExactMesh {
    let triangles = mesh
        .triangles()
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.0;
            Triangle([a, c, b])
        })
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        triangles,
        SourceProvenance::exact("adversarial reversed surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn mesh_with_cross_component_triangle(
    mesh: &ExactMesh,
    second_component_start: usize,
) -> ExactMesh {
    assert!(second_component_start < mesh.vertices().len());
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([0, 2, second_component_start])],
        SourceProvenance::exact("adversarial cross-component surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn mesh_with_filled_hole_triangle(mesh: &ExactMesh, hole_start: usize) -> ExactMesh {
    assert!(hole_start + 2 < mesh.vertices().len());
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([hole_start, hole_start + 1, hole_start + 2])],
        SourceProvenance::exact("adversarial filled-hole surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
}

#[cfg(feature = "exact-triangulation")]
fn boundary_mismatched_mesh(mesh: &ExactMesh) -> Option<ExactMesh> {
    if mesh.vertices().len() < 4 {
        return None;
    }
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([0, 1, 2]), Triangle([0, 3, 1])],
        SourceProvenance::exact("adversarial mismatched retained surface boundary"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn retained_ring_crossing_mesh(mesh: &ExactMesh) -> Option<ExactMesh> {
    if mesh.vertices().len() <= 6 {
        return None;
    }
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([0, 1, 6]), Triangle([0, 6, 3])],
        SourceProvenance::exact("adversarial retained ring crossing surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn half() -> ExactReal {
    (ExactReal::from(1) / ExactReal::from(2)).expect("nonzero denominator")
}

#[cfg(feature = "exact-triangulation")]
fn rational(numerator: i64, denominator: i64) -> ExactReal {
    (ExactReal::from(numerator) / ExactReal::from(denominator)).expect("nonzero denominator")
}

fn assert_real_eq(left: &ExactReal, right: &ExactReal) {
    assert!(real_eq(left, right), "expected {left} == {right}");
}

fn real_eq(left: &ExactReal, right: &ExactReal) -> bool {
    compare_reals(left, right).value() == Some(Ordering::Equal)
}

fn real_between_unit(value: &ExactReal) -> bool {
    let zero = ExactReal::from(0);
    let one = ExactReal::from(1);
    matches!(
        compare_reals(value, &zero).value(),
        Some(Ordering::Greater | Ordering::Equal)
    ) && matches!(
        compare_reals(value, &one).value(),
        Some(Ordering::Less | Ordering::Equal)
    )
}
