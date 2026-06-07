use hyperlimit::{Point2, Point3, SourceProvenance};
use hypermesh::{
    AffineOrthogonalSolidFreshness, ApproximateMeshF64ViewFreshness,
    AxisAlignedOrthogonalSolidFreshness, ClosedMeshOrientation,
    ContainedFaceAdjacentUnionFreshness, ConvexSolidMeshRelation, ConvexSolidPointRelation,
    ConvexSolidReportFreshness, CoplanarArrangementReadinessFreshness,
    CoplanarOverlapGraphFreshness, CoplanarOverlapSplitFreshness,
    CoplanarVolumetricCellEvidenceFreshness, ExactAdjacentUnionCompletionStatus, ExactArrangement,
    ExactArrangement2dBoundaryPolicy, ExactArrangement2dRegion, ExactArrangement2dRegionRing,
    ExactArrangement2dSetOperation, ExactArrangementFreshness, ExactBooleanBlockerKind,
    ExactBooleanOperation, ExactBooleanPolicy, ExactBooleanResult, ExactBooleanResultKind,
    ExactBoundaryBooleanPolicy, ExactBoundaryTouchingStatus, ExactI64MeshInputReadiness,
    ExactLabeledCellComplexFreshness, ExactMesh, ExactMeshConsumerDomain,
    ExactMeshDomainSummaryFreshness, ExactMeshHandoffPackageFreshness, ExactMeshProposalAcceptance,
    ExactMeshProposalSourceKind, ExactOpenSurfaceDisjointStatus, ExactPlanarArrangementStatus,
    ExactRefinementStatus, ExactRegionSelection, ExactRegularizationPolicy, ExactReportFreshness,
    ExactSameSurfaceStatus, ExactSelectedCellComplexFreshness, ExactSimplifiedCellComplexFreshness,
    ExactVolumetricRegionFreshness, ExactVolumetricRegionRelation, ExactWindingReadinessStatus,
    FaceRegionPlaneRelation, FullFaceAdjacentUnionFreshness, IntersectionGraphFreshness,
    MeshArtifactBlocker, MeshArtifactManifest, MeshArtifactRole, MeshArtifactSourceKind,
    MeshCoordinateEvidence, MeshFacePairFreshness, MeshFacePairRelation,
    MeshFacePairValidationError, SplitPlanFreshness, TriangleTriangleFreshness,
    TriangleTriangleRelation, ValidationPolicy, WindingReportFreshness, approximate_mesh_f64_view,
    boolean_exact, boolean_exact_with_boundary_policy, boolean_selected_regions,
    build_exact_arrangement2d_overlay, build_exact_arrangement2d_overlay_with_boundary_policy,
    build_intersection_graph, certify_adjacent_union_completion_report,
    certify_boundary_touching_report, certify_convex_solid,
    certify_coplanar_volumetric_cell_evidence, certify_exact_mesh_proposal,
    certify_open_surface_disjoint_report, certify_planar_arrangement_report,
    certify_refinement_report, certify_same_surface_report,
    certify_volumetric_boundary_closure_report, certify_winding_readiness_report,
    certify_winding_readiness_report_with_boundary_policy,
    certify_winding_readiness_report_with_validation,
    checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut, classify_mesh_face_pair,
    classify_mesh_vertices_against_closed_mesh_winding_report,
    classify_mesh_vertices_against_convex_solid_report,
    classify_point_against_closed_mesh_winding_report, classify_point_against_convex_solid_report,
    classify_triangle_triangle, exact_arrangement_boolean_attempt_report,
    exact_arrangement_boolean_attempt_report_with_validation, exact_mesh_consumer_readiness,
    exact_mesh_handoff_package, inspect_i64_mesh_input,
    materialize_adjacent_union_completion_boolean, materialize_affine_orthogonal_solid_boolean,
    materialize_affine_orthogonal_solid_difference,
    materialize_affine_orthogonal_solid_intersection, materialize_arrangement_cell_complex_boolean,
    materialize_axis_aligned_orthogonal_solid_boolean,
    materialize_axis_aligned_orthogonal_solid_difference,
    materialize_axis_aligned_orthogonal_solid_intersection,
    materialize_axis_aligned_orthogonal_solid_union, materialize_boundary_touching_policy_boolean,
    materialize_bounds_disjoint_boolean, materialize_closed_boundary_touching_regularized_boolean,
    materialize_closed_convex_boolean, materialize_closed_no_volume_overlap_regularized_boolean,
    materialize_closed_regularized_lower_dimensional_boolean,
    materialize_closed_same_surface_boolean, materialize_closed_winding_containment_boolean,
    materialize_closed_winding_separated_boolean, materialize_contained_face_adjacent_union,
    materialize_coplanar_mesh_overlay_arrangement, materialize_empty_operand_boolean,
    materialize_full_face_adjacent_union, materialize_identical_mesh_boolean,
    materialize_mixed_dimensional_regularized_solid_boolean, materialize_open_surface_arrangement,
    materialize_open_surface_disjoint_boolean, materialize_same_surface_boolean,
    materialize_volumetric_winding_arrangement, mesh_artifact_from_exact_mesh,
    mesh_artifact_from_exact_mesh_proposal, preflight_boolean_exact,
    preflight_boolean_exact_with_boundary_policy, triangulate_all_face_cells_with_cdt,
    validate_face_cell_cdt_against_sources,
};
use hyperreal::Real;

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn p2(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn rational_point(numerators: [i64; 3], denominator: i64) -> Point3 {
    let denominator = Real::from(denominator);
    Point3::new(
        (Real::from(numerators[0]) / &denominator).expect("nonzero denominator"),
        (Real::from(numerators[1]) / &denominator).expect("nonzero denominator"),
        (Real::from(numerators[2]) / &denominator).expect("nonzero denominator"),
    )
}

fn tetra(offset: [i64; 3]) -> ExactMesh {
    let [ox, oy, oz] = offset;
    ExactMesh::new(
        vec![
            p(ox, oy, oz),
            p(ox + 1, oy, oz),
            p(ox, oy + 1, oz),
            p(ox, oy, oz + 1),
        ],
        vec![
            hypermesh::Triangle([0, 2, 1]),
            hypermesh::Triangle([0, 1, 3]),
            hypermesh::Triangle([1, 2, 3]),
            hypermesh::Triangle([2, 0, 3]),
        ],
        SourceProvenance::exact("test tetra"),
    )
    .unwrap()
}

fn tetra_from_corners(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap()
}

fn axis_aligned_box(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
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

fn combine_exact_meshes(left: &ExactMesh, right: &ExactMesh, label: &'static str) -> ExactMesh {
    let right_offset = left.vertices().len();
    ExactMesh::new(
        left.vertices()
            .iter()
            .chain(right.vertices())
            .cloned()
            .collect(),
        left.triangles()
            .iter()
            .copied()
            .chain(right.triangles().iter().map(|triangle| {
                let [a, b, c] = triangle.0;
                hypermesh::Triangle([a + right_offset, b + right_offset, c + right_offset])
            }))
            .collect(),
        SourceProvenance::exact(label),
    )
    .unwrap()
}

#[test]
fn exact_arrangement2d_boundary_policy_is_publicly_available() {
    let rings = [
        ExactArrangement2dRegionRing::new(
            ExactArrangement2dRegion::Left,
            vec![p2(0, 0), p2(4, 0), p2(4, 2), p2(0, 2)],
        ),
        ExactArrangement2dRegionRing::new(
            ExactArrangement2dRegion::Right,
            vec![p2(2, 0), p2(6, 0), p2(6, 2), p2(2, 2)],
        ),
    ];

    let simplified =
        build_exact_arrangement2d_overlay(&rings, ExactArrangement2dSetOperation::Union);
    let preserved = build_exact_arrangement2d_overlay_with_boundary_policy(
        &rings,
        ExactArrangement2dSetOperation::Union,
        ExactArrangement2dBoundaryPolicy::PreserveCollinear,
    );

    assert!(simplified.is_complete(), "{:?}", simplified.blockers);
    assert!(preserved.is_complete(), "{:?}", preserved.blockers);
    assert_eq!(simplified.output_loops.len(), 1);
    assert_eq!(preserved.output_loops.len(), 1);
    assert_eq!(simplified.output_loops[0].points.len(), 4);
    assert_eq!(preserved.output_loops[0].points.len(), 8);
    assert_eq!(simplified.output_loops[0].signed_area_twice, Real::from(24));
    assert_eq!(preserved.output_loops[0].signed_area_twice, Real::from(24));
}

#[test]
fn exact_face_region_stage_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let stale_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, -1, -1, 8, 3, 1, 8, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_intersection_graph(&left, &right).unwrap();
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
    let regions = geometry.region_plan(&left, &right);

    let classifications =
        checked_classify_face_regions_against_opposite_planes(&regions, &left, &right).unwrap();
    let triangulations =
        checked_triangulate_face_regions_with_earcut(&regions, &left, &right).unwrap();

    assert!(!classifications.is_empty());
    assert!(!triangulations.is_empty());
    classifications[0]
        .validate_against_sources(&left, &right)
        .unwrap();
    triangulations[0]
        .validate_against_sources(&left, &right)
        .unwrap();
    assert!(
        classifications[0]
            .validate_against_sources(&left, &stale_right)
            .is_err()
    );
    assert!(
        triangulations[0]
            .validate_against_sources(&left, &stale_right)
            .is_err()
    );

    let mut stale_classification = classifications[0].clone();
    stale_classification.relation = match stale_classification.relation {
        FaceRegionPlaneRelation::StrictlyAbove => FaceRegionPlaneRelation::StrictlyBelow,
        _ => FaceRegionPlaneRelation::StrictlyAbove,
    };
    assert!(stale_classification.validate().is_err());
}

fn skew_affine_box(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    let p = |u: i64, v: i64, w: i64| [u + 10 * v, v, w];
    let corners = [
        p(min[0], min[1], min[2]),
        p(max[0], min[1], min[2]),
        p(max[0], max[1], min[2]),
        p(min[0], max[1], min[2]),
        p(min[0], min[1], max[2]),
        p(max[0], min[1], max[2]),
        p(max[0], max[1], max[2]),
        p(min[0], max[1], max[2]),
    ];
    ExactMesh::from_i64_triangles(
        &corners
            .iter()
            .flat_map(|point| point.iter().copied())
            .collect::<Vec<_>>(),
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .unwrap()
}

#[test]
fn exact_mesh_construction_retains_valid_public_facts() {
    let mesh = tetra([0, 0, 0]);

    mesh.validate_retained_state().unwrap();
    assert_eq!(mesh.facts().mesh.vertex_count, 4);
    assert_eq!(mesh.facts().mesh.face_count, 4);
    assert!(mesh.facts().mesh.closed_manifold);
    assert!(
        mesh.facts()
            .faces
            .iter()
            .all(|face| face.triangle.non_degenerate)
    );

    let report = inspect_i64_mesh_input(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    );
    let readiness = report.readiness();
    assert_eq!(readiness, ExactI64MeshInputReadiness::Ready);
    assert!(report.edge_ready());
}

#[test]
fn exact_mesh_proposal_and_artifact_reports_are_publicly_replayable() {
    let exact = tetra([0, 0, 0]);
    let proposal = certify_exact_mesh_proposal(&exact).unwrap();

    proposal.validate().unwrap();
    proposal.validate_against_mesh(&exact).unwrap();
    assert_eq!(
        proposal.source_kind,
        ExactMeshProposalSourceKind::ExactConstruction
    );
    assert_eq!(
        proposal.acceptance,
        ExactMeshProposalAcceptance::ExactInputReplayed
    );

    let mut stale_proposal = proposal.clone();
    stale_proposal.source_label.push_str(" stale");
    assert!(stale_proposal.validate_against_mesh(&exact).is_err());

    let artifact = mesh_artifact_from_exact_mesh(&exact).unwrap();
    assert_eq!(artifact.source_kind, MeshArtifactSourceKind::HypermeshExact);
    assert_eq!(artifact.role, MeshArtifactRole::SolidHandoff);
    assert!(artifact.validation_handoff_ready, "{:?}", artifact.blockers);
    assert!(artifact.blockers.is_empty());

    let proposal_artifact = mesh_artifact_from_exact_mesh_proposal(&exact, &proposal).unwrap();
    assert_eq!(proposal_artifact, artifact);

    let lossy = ExactMesh::from_f64_triangles(
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let lossy_proposal = certify_exact_mesh_proposal(&lossy).unwrap();
    lossy_proposal.validate_against_mesh(&lossy).unwrap();
    assert_eq!(
        lossy_proposal.source_kind,
        ExactMeshProposalSourceKind::LossyPrimitiveFloatProposal
    );
    assert_eq!(
        lossy_proposal.acceptance,
        ExactMeshProposalAcceptance::ProposalAcceptedAfterExactReplay
    );

    let lossy_artifact = mesh_artifact_from_exact_mesh(&lossy).unwrap();
    assert_eq!(
        lossy_artifact.source_kind,
        MeshArtifactSourceKind::HypermeshLossyF64Replay
    );
    assert!(lossy_artifact.validation_handoff_ready);
    assert!(lossy_artifact.numeric_contract.primitive_float_lowering);
    assert!(lossy_artifact.numeric_contract.lossy_adapter_route);
    assert_eq!(
        lossy_artifact.numeric_contract.coordinate_evidence,
        MeshCoordinateEvidence::ExactDyadicFromLossyFloat
    );

    let preview = MeshArtifactManifest::sdf_surface_nets_preview("preview", 3, 1).report();
    assert!(preview.preview_only);
    assert!(!preview.validation_handoff_ready);
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::PreviewOrExportOnly)
    );
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::PreviewOrExportSource)
    );
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::MissingExactCoordinateReplay)
    );
    assert!(
        preview
            .blockers
            .contains(&MeshArtifactBlocker::MissingExactTopologyReplay)
    );
}

#[test]
fn exact_mesh_handoff_package_domains_are_publicly_replayable() {
    let solid = tetra([0, 0, 0]);
    let package = exact_mesh_handoff_package(&solid).unwrap();

    package.validate_internal().unwrap();
    package.validate_against_mesh(&solid).unwrap();
    assert_eq!(
        package.freshness_against_mesh(&solid),
        ExactMeshHandoffPackageFreshness::Current
    );
    assert!(package.has_domain(ExactMeshConsumerDomain::Surface));
    assert!(package.has_domain(ExactMeshConsumerDomain::Solid));
    assert!(package.has_domain(ExactMeshConsumerDomain::ApproximateF64View));
    assert_eq!(
        package
            .require_preferred_exact_geometry_domain_against_mesh(&solid)
            .unwrap(),
        ExactMeshConsumerDomain::Solid
    );
    let preferred = package
        .preferred_exact_geometry_report_against_mesh(&solid)
        .unwrap();
    assert_eq!(preferred.domain(), ExactMeshConsumerDomain::Solid);
    assert_eq!(preferred.audit(), &package.audit);

    let summary = package.domain_summary();
    summary.validate_against_mesh(&package, &solid).unwrap();
    assert_eq!(
        summary.preferred_exact_geometry_domain(),
        Some(ExactMeshConsumerDomain::Solid)
    );
    summary
        .require_domain_against_mesh(
            &package,
            &solid,
            ExactMeshConsumerDomain::ApproximateF64View,
        )
        .unwrap();
    assert!(summary.require_lossy_adapter().is_ok());
    assert!(summary.require_closed_volume().is_ok());

    let stale_source = tetra([2, 0, 0]);
    assert_eq!(
        package.freshness_against_mesh(&stale_source),
        ExactMeshHandoffPackageFreshness::StalePackage
    );
    assert!(
        package
            .domain_report_against_mesh(&stale_source, ExactMeshConsumerDomain::Solid)
            .is_err()
    );
    assert_eq!(
        summary.freshness_against_mesh(&package, &stale_source),
        ExactMeshDomainSummaryFreshness::InvalidPackage
    );

    let mut stale_summary = summary.clone();
    stale_summary.lossy_adapter_count = 0;
    assert_eq!(
        stale_summary.freshness_against_package(&package),
        ExactMeshDomainSummaryFreshness::StaleSummary
    );
    assert!(stale_summary.validate_against_package(&package).is_err());

    let view = approximate_mesh_f64_view(&solid).unwrap();
    view.validate_against_mesh(&solid).unwrap();
    let mut stale_view = view.clone();
    stale_view.positions[0] = 42.0;
    assert_eq!(
        stale_view.freshness_against_mesh(&solid),
        ApproximateMeshF64ViewFreshness::StaleCoordinate
    );
    let mut relabeled_view = view.clone();
    relabeled_view.lossy_view = false;
    assert_eq!(
        relabeled_view.freshness_against_mesh(&solid),
        ApproximateMeshF64ViewFreshness::MissingLossyFlag
    );

    let open_surface = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_readiness = exact_mesh_consumer_readiness(&open_surface).unwrap();
    assert!(open_readiness.surface_handoff_ready);
    assert!(!open_readiness.solid_handoff_ready);
    assert!(open_readiness.boundary_allowed);

    let open_package = exact_mesh_handoff_package(&open_surface).unwrap();
    assert!(open_package.has_domain(ExactMeshConsumerDomain::Surface));
    assert!(!open_package.has_domain(ExactMeshConsumerDomain::Solid));
    assert!(open_package.has_domain(ExactMeshConsumerDomain::ApproximateF64View));
    assert_eq!(
        open_package.preferred_exact_geometry_domain(),
        Some(ExactMeshConsumerDomain::Surface)
    );
    assert!(
        open_package
            .require_domain(ExactMeshConsumerDomain::Solid)
            .is_err()
    );
    let open_summary = open_package.domain_summary();
    assert!(!open_summary.closed_volume_ready);
    assert!(open_summary.require_closed_volume().is_err());

    let lossy = ExactMesh::from_f64_triangles_with_policy(
        &[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let lossy_readiness = exact_mesh_consumer_readiness(&lossy).unwrap();
    assert!(lossy_readiness.surface_handoff_ready);
    assert!(!lossy_readiness.solid_handoff_ready);
    assert!(!lossy_readiness.exact_source);

    let lossy_package = exact_mesh_handoff_package(&lossy).unwrap();
    assert!(lossy_package.has_domain(ExactMeshConsumerDomain::Surface));
    assert!(!lossy_package.has_domain(ExactMeshConsumerDomain::Solid));
    assert!(lossy_package.has_domain(ExactMeshConsumerDomain::ApproximateF64View));
    assert_eq!(
        lossy_package
            .require_preferred_exact_geometry_domain()
            .unwrap(),
        ExactMeshConsumerDomain::Surface
    );
}

#[test]
fn exact_winding_reports_classify_freshness_publicly() {
    let target = tetra([0, 0, 0]);
    let shifted = tetra([3, 0, 0]);
    let point = rational_point([1, 1, 1], 4);

    let point_report = classify_point_against_closed_mesh_winding_report(&point, &target);
    point_report
        .validate_against_sources(&point, &target)
        .unwrap();
    assert_eq!(
        point_report.freshness_against_sources(&point, &target),
        WindingReportFreshness::Current
    );
    let mut stale_point_report = point_report.clone();
    stale_point_report.axis = None;
    assert_eq!(
        stale_point_report.freshness_against_sources(&point, &target),
        WindingReportFreshness::StaleAxisEvidence
    );
    assert_eq!(
        point_report.freshness_against_sources(&point, &shifted),
        WindingReportFreshness::SourceReplayMismatch
    );

    let mesh_report = classify_mesh_vertices_against_closed_mesh_winding_report(&shifted, &target);
    mesh_report
        .validate_against_sources(&shifted, &target)
        .unwrap();
    assert_eq!(
        mesh_report.freshness_against_sources(&shifted, &target),
        WindingReportFreshness::Current
    );
    let mut stale_mesh_report = mesh_report.clone();
    stale_mesh_report.subject_vertex_count += 1;
    assert_eq!(
        stale_mesh_report.freshness_against_sources(&shifted, &target),
        WindingReportFreshness::StaleCounts
    );
    assert_eq!(
        mesh_report.freshness_against_sources(&target, &shifted),
        WindingReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn exact_convex_reports_classify_freshness_publicly() {
    let solid = tetra([0, 0, 0]);
    let shifted = tetra([3, 0, 0]);
    let point = rational_point([1, 1, 1], 4);

    let facts = certify_convex_solid(&solid);
    facts.validate_against_source(&solid).unwrap();
    assert_eq!(
        facts.freshness_against_source(&solid),
        ConvexSolidReportFreshness::Current
    );
    let mut stale_facts = facts.clone();
    stale_facts.orientation = ClosedMeshOrientation::NotClosed;
    assert_eq!(
        stale_facts.freshness_against_source(&solid),
        ConvexSolidReportFreshness::StaleSolidFacts
    );

    let point_report = classify_point_against_convex_solid_report(&point, &solid);
    point_report
        .validate_against_sources(&point, &solid)
        .unwrap();
    assert_eq!(
        point_report.freshness_against_sources(&point, &solid),
        ConvexSolidReportFreshness::Current
    );
    let mut stale_point_report = point_report.clone();
    stale_point_report.relation = ConvexSolidPointRelation::NotCertifiedConvex;
    assert_eq!(
        stale_point_report.freshness_against_sources(&point, &solid),
        ConvexSolidReportFreshness::StalePointEvidence
    );
    assert_eq!(
        point_report.freshness_against_sources(&point, &shifted),
        ConvexSolidReportFreshness::SourceReplayMismatch
    );

    let mesh_report = classify_mesh_vertices_against_convex_solid_report(&shifted, &solid);
    mesh_report
        .validate_against_sources(&shifted, &solid)
        .unwrap();
    assert_eq!(
        mesh_report.freshness_against_sources(&shifted, &solid),
        ConvexSolidReportFreshness::Current
    );
    let mut stale_mesh_report = mesh_report.clone();
    stale_mesh_report.relation = ConvexSolidMeshRelation::StrictlyInside;
    assert_eq!(
        stale_mesh_report.freshness_against_sources(&shifted, &solid),
        ConvexSolidReportFreshness::StaleMeshEvidence
    );
    assert_eq!(
        mesh_report.freshness_against_sources(&solid, &shifted),
        ConvexSolidReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn exact_affine_orthogonal_solid_materializer_is_publicly_replayable() {
    let left = skew_affine_box([0, 0, 0], [2, 2, 2]);
    let right = skew_affine_box([1, 1, 1], [3, 3, 3]);
    let separated_right = skew_affine_box([4, 4, 4], [5, 5, 5]);

    let arrangement =
        materialize_affine_orthogonal_solid_intersection(&left, &right, ValidationPolicy::CLOSED)
            .unwrap()
            .expect("skew affine boxes should materialize by exact affine orthogonal replay");
    arrangement.validate().unwrap();
    arrangement.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        arrangement.freshness_against_sources(&left, &right),
        AffineOrthogonalSolidFreshness::Current
    );
    assert!(arrangement.mesh.facts().mesh.closed_manifold);
    assert!(!arrangement.mesh.triangles().is_empty());

    let difference =
        materialize_affine_orthogonal_solid_difference(&left, &right, ValidationPolicy::CLOSED)
            .unwrap()
            .expect("skew affine boxes should materialize exact affine difference");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        difference.freshness_against_sources(&left, &right),
        AffineOrthogonalSolidFreshness::Current
    );
    assert_eq!(
        difference.freshness_against_sources(&left, &separated_right),
        AffineOrthogonalSolidFreshness::SourceReplayMismatch
    );
    assert!(difference.mesh.facts().mesh.closed_manifold);

    let mut invalid_basis = arrangement.clone();
    invalid_basis.basis.basis_u = p(0, 0, 0);
    assert_eq!(
        invalid_basis.freshness_against_sources(&left, &right),
        AffineOrthogonalSolidFreshness::InvalidOutput
    );

    assert_eq!(
        arrangement.freshness_against_sources(&left, &separated_right),
        AffineOrthogonalSolidFreshness::SourceReplayMismatch
    );

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = materialize_affine_orthogonal_solid_boolean(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("skew affine boxes should materialize as a boolean result");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_ok(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch,
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }
}

#[test]
fn exact_axis_aligned_orthogonal_solid_materializer_is_publicly_replayable() {
    let horizontal = axis_aligned_box([0, 0, 0], [2, 1, 1]);
    let vertical = axis_aligned_box([0, 1, 0], [1, 2, 1]);
    let left = materialize_axis_aligned_orthogonal_solid_union(
        &horizontal,
        &vertical,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("adjacent exact boxes should materialize an L-shaped orthogonal solid")
    .mesh;
    let right = axis_aligned_box([1, 0, 0], [3, 1, 1]);
    let separated_right = axis_aligned_box([5, 0, 0], [6, 1, 1]);

    let arrangement = materialize_axis_aligned_orthogonal_solid_intersection(
        &left,
        &right,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("L solid and box should materialize by exact orthogonal cells");
    arrangement.validate().unwrap();
    arrangement.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        arrangement.freshness_against_sources(&left, &right),
        AxisAlignedOrthogonalSolidFreshness::Current
    );
    assert!(arrangement.selected_cells > 0);
    assert!(arrangement.mesh.facts().mesh.closed_manifold);
    assert!(!arrangement.mesh.triangles().is_empty());

    let difference = materialize_axis_aligned_orthogonal_solid_difference(
        &left,
        &right,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("L solid and box should materialize exact orthogonal difference");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        difference.freshness_against_sources(&left, &right),
        AxisAlignedOrthogonalSolidFreshness::Current
    );
    assert_eq!(
        difference.freshness_against_sources(&left, &separated_right),
        AxisAlignedOrthogonalSolidFreshness::SourceReplayMismatch
    );
    assert!(difference.selected_cells > 0);
    assert!(difference.mesh.facts().mesh.closed_manifold);

    let mut stale_selected_count = arrangement.clone();
    stale_selected_count.selected_cells += 1;
    assert_eq!(
        stale_selected_count.freshness_against_sources(&left, &right),
        AxisAlignedOrthogonalSolidFreshness::SourceReplayMismatch
    );

    let mut invalid_output = arrangement.clone();
    invalid_output.mesh = tetra([0, 0, 0]);
    assert_eq!(
        invalid_output.freshness_against_sources(&left, &right),
        AxisAlignedOrthogonalSolidFreshness::InvalidOutput
    );

    assert_eq!(
        arrangement.freshness_against_sources(&left, &separated_right),
        AxisAlignedOrthogonalSolidFreshness::SourceReplayMismatch
    );

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = materialize_axis_aligned_orthogonal_solid_boolean(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("L solid and box should materialize as a boolean result");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_ok(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch,
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }
}

#[test]
fn exact_coplanar_volumetric_cell_evidence_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = tetra_from_corners([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);

    let report = certify_coplanar_volumetric_cell_evidence(&left, &right).unwrap();
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        report.freshness_against_sources(&left, &right),
        CoplanarVolumetricCellEvidenceFreshness::Current
    );
    assert!(report.obstacle.requires_coplanar_volumetric_cells());
    assert!(report.positive_area_coplanar_overlapping_pairs > 0);
    assert!(report.same_side_coplanar_overlapping_pairs > 0);

    let mut stale_counts = report.clone();
    stale_counts.retained_face_pair_count += 1;
    assert_eq!(
        stale_counts.freshness_against_sources(&left, &right),
        CoplanarVolumetricCellEvidenceFreshness::StaleFacePairCounts
    );

    let separated_right = tetra([10, 0, 0]);
    assert_eq!(
        report.freshness_against_sources(&left, &separated_right),
        CoplanarVolumetricCellEvidenceFreshness::SourceReplayMismatch
    );
}

#[test]
fn exact_closed_convex_boolean_materializer_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetra_from_corners([2, 2, 2], [8, -1, -1], [-1, 8, -1], [3, 2, 0]);
    let stale_open_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for (operation, shortcut) in [
        (
            ExactBooleanOperation::Union,
            hypermesh::ExactBooleanShortcutKind::ConvexUnion,
        ),
        (
            ExactBooleanOperation::Intersection,
            hypermesh::ExactBooleanShortcutKind::ConvexIntersection,
        ),
        (
            ExactBooleanOperation::Difference,
            hypermesh::ExactBooleanShortcutKind::ConvexDifference,
        ),
    ] {
        let result =
            materialize_closed_convex_boolean(&left, &right, operation, ValidationPolicy::CLOSED)
                .unwrap()
                .expect("nonorthogonal closed convex solids should materialize directly");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut
            }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &stale_open_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_ok(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch,
            "{operation:?}: {stale_output:?}"
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }

    let separated_left = tetra_from_corners([0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]);
    let separated_right = tetra_from_corners([1, 1, 1], [3, 1, 1], [1, 3, 1], [1, 1, 3]);
    let preflight = preflight_boolean_exact(
        &separated_left,
        &separated_right,
        ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedConvexSeparated,
        "{preflight:?}"
    );
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&separated_left, &separated_right)
        .unwrap();
    let separated = materialize_closed_convex_boolean(
        &separated_left,
        &separated_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("separated closed convex intersection should retain convex relation provenance");
    assert_eq!(
        separated.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Intersection,
            shortcut: hypermesh::ExactBooleanShortcutKind::ConvexSeparated
        }
    );
    separated.validate().unwrap();
    separated
        .validate_against_sources(&separated_left, &separated_right)
        .unwrap();
    let mut stale_separated_output = separated.clone();
    stale_separated_output.mesh = separated_left.clone();
    assert!(
        stale_separated_output.validate().is_ok(),
        "{stale_separated_output:?}"
    );
    assert_eq!(
        stale_separated_output.freshness_against_sources(&separated_left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch,
        "{stale_separated_output:?}"
    );
    separated
        .validate_operation_against_sources(
            &separated_left,
            &separated_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    let dispatched = boolean_exact(
        &separated_left,
        &separated_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(dispatched.kind, separated.kind);

    let contained_on_boundary = tetra_from_corners([1, 1, 0], [2, 1, 0], [1, 2, 0], [1, 1, 1]);
    let container = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let preflight = preflight_boolean_exact(
        &contained_on_boundary,
        &container,
        ExactBooleanOperation::Difference,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedConvexContainment,
        "{preflight:?}"
    );
    assert!(
        preflight.retained_face_pairs > 0,
        "boundary-contained convex relation should retain graph evidence: {preflight:?}"
    );
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&contained_on_boundary, &container)
        .unwrap();
    let containment = materialize_closed_convex_boolean(
        &contained_on_boundary,
        &container,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("boundary-contained closed convex difference should certify as empty");
    assert_eq!(
        containment.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Difference,
            shortcut: hypermesh::ExactBooleanShortcutKind::ConvexContainment
        }
    );
    containment.validate().unwrap();
    containment
        .validate_against_sources(&contained_on_boundary, &container)
        .unwrap();
    let mut stale_containment_output = containment.clone();
    stale_containment_output.mesh = container.clone();
    assert!(
        stale_containment_output.validate().is_ok(),
        "{stale_containment_output:?}"
    );
    assert_eq!(
        stale_containment_output.freshness_against_sources(&contained_on_boundary, &container),
        ExactReportFreshness::SourceReplayMismatch,
        "{stale_containment_output:?}"
    );
    containment
        .validate_operation_against_sources(
            &contained_on_boundary,
            &container,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(containment.mesh.triangles().is_empty());

    assert!(
        materialize_closed_convex_boolean(
            &axis_aligned_box([0, 0, 0], [2, 2, 2]),
            &axis_aligned_box([1, 1, 1], [3, 3, 3]),
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );
}

#[test]
fn exact_full_face_adjacent_union_is_publicly_replayable() {
    let left = axis_aligned_box([0, 0, 0], [1, 1, 1]);
    let right = axis_aligned_box([1, 0, 0], [2, 1, 1]);

    let union = materialize_full_face_adjacent_union(&left, &right, ValidationPolicy::CLOSED)
        .expect("full-face adjacent boxes should materialize as a welded union");
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.freshness_against_sources(&left, &right),
        FullFaceAdjacentUnionFreshness::Current
    );
    assert!(union.mesh.facts().mesh.closed_manifold);
    assert!(!union.shared_faces.is_empty() || !union.shared_patches.is_empty());
    assert!(!union.mesh.triangles().is_empty());

    let mut invalid_shared_faces = union.clone();
    invalid_shared_faces.shared_faces.clear();
    invalid_shared_faces.shared_patches.clear();
    assert_eq!(
        invalid_shared_faces.freshness_against_sources(&left, &right),
        FullFaceAdjacentUnionFreshness::InvalidSharedFaces
    );

    let mut invalid_output = union.clone();
    invalid_output.mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        invalid_output.freshness_against_sources(&left, &right),
        FullFaceAdjacentUnionFreshness::InvalidOutput
    );

    let separated_right = axis_aligned_box([3, 0, 0], [4, 1, 1]);
    assert_eq!(
        union.freshness_against_sources(&left, &separated_right),
        FullFaceAdjacentUnionFreshness::SourceReplayMismatch
    );
}

#[test]
fn adjacent_union_completion_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = combine_exact_meshes(&left_a, &left_b, "test disconnected full-face fixture");
    let right = tetra_from_corners([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, -4]);
    let separated_right = tetra_from_corners([20, 0, 0], [24, 0, 0], [20, 4, 0], [20, 0, 4]);

    let report =
        certify_adjacent_union_completion_report(&left, &right, ExactBooleanOperation::Union)
            .unwrap();
    assert_eq!(
        report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace
    );
    assert!(report.is_certified());
    assert!(report.full_face_shared_faces + report.full_face_shared_patches > 0);
    assert_eq!(report.contained_faces, 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        report.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        report.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let result = materialize_adjacent_union_completion_boolean(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("non-axis full-face adjacent solids should complete as a boolean union");
    assert_eq!(
        result.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
        }
    );
    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    let mut stale_output = result.clone();
    stale_output.mesh = left.clone();
    assert!(stale_output.validate().is_ok(), "{stale_output:?}");
    assert_eq!(
        stale_output.freshness_against_sources(&left, &right),
        ExactReportFreshness::SourceReplayMismatch,
        "{stale_output:?}"
    );
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(result.mesh.facts().mesh.closed_manifold);

    assert!(
        materialize_adjacent_union_completion_boolean(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );
    let intersection_report = certify_adjacent_union_completion_report(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
    )
    .unwrap();
    assert_eq!(
        intersection_report.status,
        ExactAdjacentUnionCompletionStatus::NotUnion
    );
    assert!(!intersection_report.is_certified());
    intersection_report.validate().unwrap();
    assert!(
        materialize_adjacent_union_completion_boolean(
            &axis_aligned_box([0, 0, 0], [1, 1, 1]),
            &axis_aligned_box([1, 0, 0], [2, 1, 1]),
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );

    let crossing_right = tetra_from_corners([1, 1, -1], [5, 1, -1], [1, 5, -1], [1, 1, 3]);
    let crossing_report = certify_adjacent_union_completion_report(
        &left,
        &crossing_right,
        ExactBooleanOperation::Union,
    )
    .unwrap();
    assert_eq!(
        crossing_report.status,
        ExactAdjacentUnionCompletionStatus::NoAdjacencyCertificate
    );
    assert_eq!(
        crossing_report.blocker.kind,
        ExactBooleanBlockerKind::NeedsWinding
    );
    assert!(crossing_report.blocker.candidate_pairs > 0);
    crossing_report.validate().unwrap();
    crossing_report
        .validate_against_sources(&left, &crossing_right)
        .unwrap();

    let mut stale_crossing = crossing_report;
    stale_crossing.blocker.kind = ExactBooleanBlockerKind::NeedsBoundaryPolicy;
    assert!(stale_crossing.validate().is_err());
}

#[test]
fn exact_open_surface_arrangement_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, -1, -1, 8, 3, 1, 8, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        if !matches!(operation, ExactBooleanOperation::Intersection) {
            let closed_attempt = exact_arrangement_boolean_attempt_report_with_validation(
                &left,
                &right,
                operation,
                ExactRegularizationPolicy::REGULARIZED_SOLID,
                ValidationPolicy::CLOSED,
            )
            .unwrap();
            assert_eq!(closed_attempt.output_validation, ValidationPolicy::CLOSED);
            assert!(
                matches!(
                    closed_attempt.decline,
                    Some(hypermesh::ExactArrangementBooleanDecline::OutputValidation)
                ),
                "{operation:?}: {closed_attempt:?}"
            );
            assert!(
                closed_attempt.output_vertices > 0,
                "{operation:?}: {closed_attempt:?}"
            );
            assert!(
                closed_attempt.output_triangles > 0,
                "{operation:?}: {closed_attempt:?}"
            );
            closed_attempt.validate().unwrap();
            closed_attempt
                .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
                .unwrap();
            assert_eq!(
                closed_attempt.freshness_against_sources_with_validation(
                    &left,
                    &right,
                    ValidationPolicy::CLOSED,
                ),
                ExactReportFreshness::Current
            );
            assert_eq!(
                closed_attempt.freshness_against_sources_with_validation(
                    &left,
                    &right,
                    ValidationPolicy::ALLOW_BOUNDARY,
                ),
                ExactReportFreshness::SourceReplayMismatch
            );
            assert!(
                materialize_open_surface_arrangement(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::CLOSED,
                )
                .unwrap()
                .is_none(),
                "{operation:?} should decline direct materialization when open-surface output cannot satisfy CLOSED validation"
            );
        }

        let attempt = exact_arrangement_boolean_attempt_report(
            &left,
            &right,
            operation,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
        assert_eq!(
            attempt.materialized_shortcut,
            Some(hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex),
            "{operation:?}: {attempt:?}"
        );
        assert_eq!(attempt.output_validation, ValidationPolicy::ALLOW_BOUNDARY);
        attempt.validate().unwrap();
        attempt.validate_against_sources(&left, &right).unwrap();
        attempt
            .validate_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        assert_eq!(
            attempt.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            ),
            ExactReportFreshness::Current
        );
        assert_eq!(
            attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::SourceReplayMismatch
        );

        let result = materialize_open_surface_arrangement(
            &left,
            &right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("crossing open surfaces should materialize by exact arrangement");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::OpenSurfaceArrangement { operation }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert!(!result.region_classifications.is_empty());
        assert!(!result.triangulations.is_empty());
        if matches!(operation, ExactBooleanOperation::Intersection) {
            assert!(result.mesh.triangles().is_empty());
        } else {
            assert!(!result.mesh.triangles().is_empty());
        }
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
    }

    let union = materialize_open_surface_arrangement(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .unwrap();
    let difference = materialize_open_surface_arrangement(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .unwrap();
    let mut stale_materialization = union.clone();
    stale_materialization.assembly = difference.assembly;
    stale_materialization.mesh = difference.mesh;
    assert!(
        stale_materialization.validate().is_ok(),
        "{stale_materialization:?}"
    );
    assert_eq!(
        stale_materialization.freshness_against_sources(&left, &right),
        ExactReportFreshness::SourceReplayMismatch,
        "{stale_materialization:?}"
    );
}

#[test]
fn arrangement_attempt_output_validation_is_publicly_replayable() {
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

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
    ] {
        let closed_attempt = exact_arrangement_boolean_attempt_report_with_validation(
            &left,
            &right,
            operation,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(closed_attempt.output_validation, ValidationPolicy::CLOSED);
        assert!(
            closed_attempt.materialized_shortcut.is_none(),
            "{operation:?}: {closed_attempt:?}"
        );
        assert!(
            matches!(
                closed_attempt.decline,
                Some(hypermesh::ExactArrangementBooleanDecline::OutputValidation)
            ),
            "{operation:?}: {closed_attempt:?}"
        );
        assert!(
            closed_attempt.output_vertices > 0,
            "{operation:?}: {closed_attempt:?}"
        );
        if operation == ExactBooleanOperation::Union {
            assert!(
                closed_attempt.output_triangles > 0,
                "{operation:?}: {closed_attempt:?}"
            );
        }
        closed_attempt.validate().unwrap();
        closed_attempt
            .validate_against_sources(&left, &right)
            .unwrap();
        closed_attempt
            .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
            .unwrap();
        assert_eq!(
            closed_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::Current
        );
        assert_eq!(
            closed_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            ),
            ExactReportFreshness::SourceReplayMismatch
        );

        let boundary_attempt = exact_arrangement_boolean_attempt_report_with_validation(
            &left,
            &right,
            operation,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap();
        assert_eq!(
            boundary_attempt.output_validation,
            ValidationPolicy::ALLOW_BOUNDARY
        );
        assert_eq!(
            boundary_attempt.materialized_shortcut,
            Some(hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex),
            "{operation:?}: {boundary_attempt:?}"
        );
        assert!(
            boundary_attempt.decline.is_none(),
            "{operation:?}: {boundary_attempt:?}"
        );
        boundary_attempt.validate().unwrap();
        boundary_attempt
            .validate_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .unwrap();
        assert_eq!(
            boundary_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::ALLOW_BOUNDARY,
            ),
            ExactReportFreshness::Current
        );
        assert_eq!(
            boundary_attempt.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::SourceReplayMismatch
        );
    }
}

#[test]
fn exact_selected_region_boolean_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, -1, -1, 8, 3, 1, 8, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let policy = ExactBooleanPolicy {
        selection: ExactRegionSelection::KeepAll,
        validation: ValidationPolicy::ALLOW_BOUNDARY,
        reject_unknowns: true,
    };

    let result = boolean_selected_regions(&left, &right, policy).unwrap();

    assert_eq!(
        result.kind,
        ExactBooleanResultKind::SelectedRegions {
            selection: ExactRegionSelection::KeepAll
        }
    );
    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    assert!(!result.region_classifications.is_empty());
    assert!(!result.triangulations.is_empty());
    assert!(!result.assembly.triangles.is_empty());
    assert!(!result.mesh.triangles().is_empty());
    assert_eq!(
        result.mesh.validation_policy(),
        ValidationPolicy::ALLOW_BOUNDARY
    );

    let mut stale_kind = result.clone();
    stale_kind.kind = ExactBooleanResultKind::SelectedRegions {
        selection: ExactRegionSelection::KeepLeft,
    };
    assert!(stale_kind.validate_against_sources(&left, &right).is_err());

    let keep_left = boolean_selected_regions(
        &left,
        &right,
        ExactBooleanPolicy {
            selection: ExactRegionSelection::KeepLeft,
            validation: ValidationPolicy::ALLOW_BOUNDARY,
            reject_unknowns: true,
        },
    )
    .unwrap();
    let mut stale_materialization = result.clone();
    stale_materialization.assembly = keep_left.assembly;
    stale_materialization.mesh = keep_left.mesh;
    assert!(
        stale_materialization.validate().is_ok(),
        "{stale_materialization:?}"
    );
    assert_eq!(
        stale_materialization.freshness_against_sources(&left, &right),
        ExactReportFreshness::SourceReplayMismatch,
        "{stale_materialization:?}"
    );
}

#[test]
fn exact_coplanar_mesh_overlay_arrangement_is_publicly_replayable() {
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
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, 2, 0, 12, 2, 0, 12, 6, 0, 8, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = materialize_coplanar_mesh_overlay_arrangement(
            &left,
            &right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("overlapping coplanar surfaces should materialize by exact overlay");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
            }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert!(!result.mesh.triangles().is_empty());
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch
        );
    }

    let identical = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        materialize_coplanar_mesh_overlay_arrangement(
            &identical,
            &identical,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .is_none(),
        "direct coplanar overlay wrapper should yield to the public identical shortcut"
    );
}

#[test]
fn lower_dimensional_regularized_boolean_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let disjoint_right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 0, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let closed_right = tetra([0, 0, 0]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = hypermesh::preflight_boolean_exact_with_validation(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            preflight.support,
            hypermesh::ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid
        );
        preflight.validate().unwrap();
        preflight
            .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
            .unwrap();
        assert!(
            preflight
                .validate_against_sources_with_validation(
                    &left,
                    &closed_right,
                    ValidationPolicy::CLOSED,
                )
                .is_err()
        );

        let readiness = certify_winding_readiness_report_with_validation(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            readiness.status,
            ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
            "{operation:?}: {readiness:?}"
        );
        assert_eq!(
            readiness.blocker.kind,
            ExactBooleanBlockerKind::NeedsWinding,
            "{operation:?}: {readiness:?}"
        );
        assert_eq!(readiness.retained_face_pairs, 0);
        assert_eq!(readiness.retained_events, 0);
        assert_eq!(readiness.region_count, 0);
        readiness.validate().unwrap();
        readiness
            .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
            .unwrap();
        assert_eq!(
            readiness.freshness_against_sources_with_validation(
                &left,
                &right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::Current
        );
        assert_eq!(
            readiness.freshness_against_sources_with_validation(
                &left,
                &closed_right,
                ValidationPolicy::CLOSED,
            ),
            ExactReportFreshness::SourceReplayMismatch
        );

        let result = materialize_closed_regularized_lower_dimensional_boolean(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("lower-dimensional operands should regularize to exact empty solid output");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            }
        );
        assert!(result.mesh.triangles().is_empty());
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &closed_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let disjoint_preflight = hypermesh::preflight_boolean_exact_with_validation(
            &left,
            &disjoint_right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            disjoint_preflight.support,
            hypermesh::ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid,
            "{operation:?}: {disjoint_preflight:?}"
        );
        disjoint_preflight
            .validate_against_sources_with_validation(
                &left,
                &disjoint_right,
                ValidationPolicy::CLOSED,
            )
            .unwrap();
        let disjoint_readiness = certify_winding_readiness_report_with_validation(
            &left,
            &disjoint_right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            disjoint_readiness.status,
            ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
            "{operation:?}: {disjoint_readiness:?}"
        );
        assert!(
            materialize_bounds_disjoint_boolean(
                &left,
                &disjoint_right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .is_none(),
            "{operation:?} should yield to closed lower-dimensional provenance"
        );
        let disjoint_result =
            boolean_exact(&left, &disjoint_right, operation, ValidationPolicy::CLOSED).unwrap();
        assert_eq!(
            disjoint_result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            },
            "{operation:?}: {disjoint_result:?}"
        );
        assert!(disjoint_result.mesh.triangles().is_empty());
        assert!(disjoint_result.mesh.facts().mesh.closed_manifold);
        disjoint_result
            .validate_operation_against_sources(
                &left,
                &disjoint_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }
}

#[test]
fn mixed_dimensional_regularized_solid_boolean_is_publicly_replayable() {
    let solid = axis_aligned_box([0, 0, 0], [4, 4, 4]);
    let sheet = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 1, 3, 1, 1, 1, 3, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let stale_solid = axis_aligned_box([10, 0, 0], [14, 4, 4]);

    for (left, right, stale_left, stale_right, solid_is_left) in [
        (&solid, &sheet, &stale_solid, &sheet, true),
        (&sheet, &solid, &sheet, &stale_solid, false),
    ] {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let result = materialize_mixed_dimensional_regularized_solid_boolean(
                left,
                right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .expect("mixed-dimensional regularized solid should materialize");
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: hypermesh::ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
                }
            );
            result.validate().unwrap();
            result.validate_against_sources(left, right).unwrap();
            assert_eq!(
                result.freshness_against_sources(left, right),
                ExactReportFreshness::Current
            );
            let keeps_solid = matches!(operation, ExactBooleanOperation::Union)
                || (solid_is_left && matches!(operation, ExactBooleanOperation::Difference));
            let expected_stale_freshness = if keeps_solid {
                ExactReportFreshness::SourceReplayMismatch
            } else {
                ExactReportFreshness::Current
            };
            assert_eq!(
                result.freshness_against_sources(stale_left, stale_right),
                expected_stale_freshness
            );
            result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();

            if keeps_solid {
                assert!(result.mesh.facts().mesh.closed_manifold);
                assert!(!result.mesh.triangles().is_empty());
            } else {
                assert!(
                    result.mesh.triangles().is_empty(),
                    "{operation:?}: {result:?}"
                );
            }
        }
    }

    let lower_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let lower_right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        materialize_mixed_dimensional_regularized_solid_boolean(
            &lower_left,
            &lower_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );
    assert!(
        materialize_closed_regularized_lower_dimensional_boolean(
            &lower_left,
            &lower_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_some()
    );

    let disjoint_sheet = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 0, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    for (left, right, solid_is_left) in [
        (&solid, &disjoint_sheet, true),
        (&disjoint_sheet, &solid, false),
    ] {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let preflight = hypermesh::preflight_boolean_exact_with_validation(
                left,
                right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap();
            assert_eq!(
                preflight.support,
                hypermesh::ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid,
                "{operation:?}: {preflight:?}"
            );
            let readiness = certify_winding_readiness_report_with_validation(
                left,
                right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert!(
                materialize_bounds_disjoint_boolean(
                    left,
                    right,
                    operation,
                    ValidationPolicy::CLOSED,
                )
                .unwrap()
                .is_none(),
                "{operation:?} should yield to mixed-dimensional regularized provenance"
            );
            let result = boolean_exact(left, right, operation, ValidationPolicy::CLOSED).unwrap();
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: hypermesh::ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid
                },
                "{operation:?}: {result:?}"
            );
            result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
            let keeps_solid = matches!(operation, ExactBooleanOperation::Union)
                || (solid_is_left && matches!(operation, ExactBooleanOperation::Difference));
            assert_eq!(
                result.mesh.triangles().is_empty(),
                !keeps_solid,
                "{operation:?}: {result:?}"
            );

            assert!(
                materialize_mixed_dimensional_regularized_solid_boolean(
                    left,
                    right,
                    operation,
                    ValidationPolicy::ALLOW_BOUNDARY,
                )
                .unwrap()
                .is_none(),
                "{operation:?} should yield to bounds-disjoint provenance for boundary-valid output"
            );
            let boundary_preflight = preflight_boolean_exact(left, right, operation).unwrap();
            assert_eq!(
                boundary_preflight.support,
                hypermesh::ExactBooleanSupport::CertifiedBoundsDisjoint,
                "{operation:?}: {boundary_preflight:?}"
            );
            let boundary_result =
                boolean_exact(left, right, operation, ValidationPolicy::ALLOW_BOUNDARY).unwrap();
            assert_eq!(
                boundary_result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint
                },
                "{operation:?}: {boundary_result:?}"
            );
            boundary_result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    ValidationPolicy::ALLOW_BOUNDARY,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
        }
    }
}

#[test]
fn boundary_touching_policy_boolean_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 0, 2, 0, 2, 2, 2],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[5, 0, 0, 7, 0, 0, 5, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        assert!(
            materialize_boundary_touching_policy_boolean(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap()
            .is_none()
        );

        let result = materialize_boundary_touching_policy_boolean(
            &left,
            &right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap()
        .expect("certified boundary-only contact should materialize under explicit policy");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::BoundaryPolicyShortcut { operation }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
    }
}

#[test]
fn closed_boundary_touching_regularized_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = combine_exact_meshes(
        &left_a,
        &left_b,
        "test disconnected closed boundary fixture",
    );
    let right = tetra_from_corners([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
    let separated_right = tetra_from_corners([100, 0, 0], [104, 0, 0], [100, 4, 0], [100, 0, 4]);

    for (operation, support, shortcut) in [
        (
            ExactBooleanOperation::Union,
            hypermesh::ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
        ),
        (
            ExactBooleanOperation::Intersection,
            hypermesh::ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
        ),
        (
            ExactBooleanOperation::Difference,
            hypermesh::ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        ),
    ] {
        let preflight = preflight_boolean_exact(&left, &right, operation).unwrap();
        assert_eq!(preflight.support, support, "{operation:?}: {preflight:?}");
        assert!(
            preflight.retained_face_pairs > 0,
            "closed boundary-touching shortcut should retain graph evidence: {operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        let result = materialize_closed_boundary_touching_regularized_boolean(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("closed boundary-only contact should materialize by exact regularization");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut
            }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        if operation == ExactBooleanOperation::Intersection {
            let mut stale_output = result.clone();
            stale_output.mesh = left.clone();
            assert!(stale_output.validate().is_ok(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::SourceReplayMismatch,
                "{stale_output:?}"
            );
        }
        if operation == ExactBooleanOperation::Difference {
            let mut stale_output = result.clone();
            stale_output.mesh = right.clone();
            assert!(stale_output.validate().is_ok(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::SourceReplayMismatch,
                "{stale_output:?}"
            );
        }
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }
}

#[test]
fn closed_no_volume_overlap_regularized_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = combine_exact_meshes(
        &left_a,
        &left_b,
        "test disconnected positive-area boundary fixture",
    );
    let right = tetra_from_corners([2, 0, 0], [6, 0, 0], [2, 4, 0], [2, 0, -4]);
    let separated_right = tetra_from_corners([20, 0, 0], [24, 0, 0], [20, 4, 0], [20, 0, -4]);

    let evidence = certify_coplanar_volumetric_cell_evidence(&left, &right).unwrap();
    evidence.validate().unwrap();
    evidence.validate_against_sources(&left, &right).unwrap();
    assert!(evidence.positive_area_coplanar_overlapping_pairs > 0);

    for (operation, support, shortcut) in [
        (
            ExactBooleanOperation::Union,
            hypermesh::ExactBooleanSupport::CertifiedArrangementCellComplex,
            hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex,
        ),
        (
            ExactBooleanOperation::Intersection,
            hypermesh::ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
        ),
        (
            ExactBooleanOperation::Difference,
            hypermesh::ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        ),
    ] {
        let preflight = preflight_boolean_exact(&left, &right, operation).unwrap();
        assert_eq!(preflight.support, support, "{operation:?}: {preflight:?}");
        assert!(
            preflight.retained_face_pairs > 0,
            "positive-area no-volume shortcut should retain graph evidence: {operation:?}: {preflight:?}"
        );
        assert_eq!(
            preflight.coplanar_volumetric_evidence.as_ref(),
            Some(&evidence),
            "{operation:?}: positive-area no-volume shortcut should retain source-aware boundary-only evidence"
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        if matches!(
            operation,
            ExactBooleanOperation::Intersection | ExactBooleanOperation::Difference
        ) {
            let readiness = certify_winding_readiness_report(&left, &right, operation).unwrap();
            assert_eq!(
                readiness.status,
                hypermesh::ExactWindingReadinessStatus::ClosedBoundaryTouchingAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            assert_eq!(
                readiness.coplanar_volumetric_evidence.as_ref(),
                Some(&evidence),
                "{operation:?}: no-volume readiness should retain consumed source-aware evidence"
            );
            readiness.validate().unwrap();
            readiness.validate_against_sources(&left, &right).unwrap();
        }

        assert!(
            materialize_closed_boundary_touching_regularized_boolean(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .is_none()
        );
        let result = materialize_closed_no_volume_overlap_regularized_boolean(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .unwrap_or_else(|| {
            panic!(
                "{operation:?}: positive-area boundary-only contact should materialize by exact no-volume overlap"
            )
        });
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut
            }
        );
        result.validate().unwrap();
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        if operation == ExactBooleanOperation::Union {
            let mut stale_output = result.clone();
            stale_output.mesh = left.clone();
            assert!(stale_output.validate().is_ok(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::SourceReplayMismatch,
                "{stale_output:?}"
            );
        }
        assert_eq!(
            result.freshness_against_sources(&left, &separated_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        if operation == ExactBooleanOperation::Union {
            assert_eq!(
                result.mesh.triangles().len(),
                left.triangles().len() + right.triangles().len()
            );
            assert!(result.mesh.facts().mesh.closed_manifold);
        }
    }
}

#[test]
fn closed_winding_shortcuts_are_publicly_replayable() {
    let separated_left_a = tetra_from_corners([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);
    let separated_left_b = tetra_from_corners([10, 0, 0], [11, 0, 0], [10, 1, 0], [10, 0, 1]);
    let separated_left = combine_exact_meshes(
        &separated_left_a,
        &separated_left_b,
        "test disconnected closed winding separated fixture",
    );
    let separated_right = tetra_from_corners([5, 0, 0], [6, 0, 0], [5, 1, 0], [5, 0, 1]);
    let intersecting_right = tetra_from_corners([0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = materialize_closed_winding_separated_boolean(
            &separated_left,
            &separated_right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("empty graph separated solids should materialize by exact winding");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::ClosedWindingSeparated
            }
        );
        result.validate().unwrap();
        result
            .validate_against_sources(&separated_left, &separated_right)
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&separated_left, &separated_right),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&separated_left, &intersecting_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        if operation == ExactBooleanOperation::Intersection {
            let mut stale_output = result.clone();
            stale_output.mesh = separated_left.clone();
            assert!(stale_output.validate().is_ok(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&separated_left, &separated_right),
                ExactReportFreshness::SourceReplayMismatch,
                "{stale_output:?}"
            );
        }
        result
            .validate_operation_against_sources(
                &separated_left,
                &separated_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }

    let outer = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let disjoint_shell = tetra_from_corners([20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]);
    let container = combine_exact_meshes(
        &outer,
        &disjoint_shell,
        "test disconnected closed winding containment fixture",
    );
    let contained = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let uncontained = tetra_from_corners([30, 0, 0], [31, 0, 0], [30, 1, 0], [30, 0, 1]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let result = materialize_closed_winding_containment_boolean(
            &container,
            &contained,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("empty graph contained solids should materialize by exact winding");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::ClosedWindingContainment
            }
        );
        result.validate().unwrap();
        result
            .validate_against_sources(&container, &contained)
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&container, &contained),
            ExactReportFreshness::Current
        );
        assert_eq!(
            result.freshness_against_sources(&container, &uncontained),
            ExactReportFreshness::SourceReplayMismatch
        );
        if operation == ExactBooleanOperation::Difference {
            let mut stale_output = result.clone();
            stale_output.mesh = container.clone();
            assert!(stale_output.validate().is_ok(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&container, &contained),
                ExactReportFreshness::SourceReplayMismatch,
                "{stale_output:?}"
            );
        }
        result
            .validate_operation_against_sources(
                &container,
                &contained,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
    }
}

#[test]
fn exact_volumetric_winding_arrangement_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4, //
            2, 2, 3,
        ],
        &[
            0, 2, 1, //
            1, 2, 3, //
            2, 0, 3, //
            0, 1, 4, //
            1, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let separated_right = tetra([10, 10, 10]);

    let graph = build_intersection_graph(&left, &right).unwrap();
    let (cell_regions, cell_triangulations) =
        triangulate_all_face_cells_with_cdt(&graph, &left, &right)
            .unwrap()
            .expect("overlapping closed solids should expose exact CDT face cells");
    assert_eq!(
        cell_regions.regions.len(),
        left.triangles().len() + right.triangles().len()
    );
    assert_eq!(cell_triangulations.len(), cell_regions.regions.len());
    assert!(cell_regions.validate(&left, &right).is_valid());
    validate_face_cell_cdt_against_sources(&cell_regions, &cell_triangulations, &left, &right)
        .unwrap();
    assert!(
        validate_face_cell_cdt_against_sources(
            &cell_regions,
            &cell_triangulations,
            &left,
            &separated_right
        )
        .is_err()
    );

    let preflight = hypermesh::preflight_boolean_exact_with_validation(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedArrangementCellComplex,
        "{preflight:?}"
    );
    preflight.validate().unwrap();
    preflight
        .validate_against_sources_with_validation(&left, &right, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap();

    let readiness = certify_winding_readiness_report_with_validation(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        readiness.status,
        ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
        "{readiness:?}"
    );
    assert_eq!(
        readiness.retained_face_pairs,
        graph.face_pairs.len(),
        "{readiness:?}"
    );
    assert_eq!(readiness.retained_events, graph.event_count());
    assert_eq!(readiness.region_count, 0);
    readiness.validate().unwrap();
    readiness
        .validate_against_sources_with_validation(&left, &right, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap();
    assert_eq!(
        readiness.freshness_against_sources_with_validation(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
        ExactReportFreshness::Current
    );
    assert_eq!(
        readiness.freshness_against_sources_with_validation(
            &left,
            &separated_right,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
        ExactReportFreshness::SourceReplayMismatch
    );

    let result = materialize_volumetric_winding_arrangement(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .expect("overlapping closed solids should materialize by exact volumetric winding");

    assert_eq!(
        result.kind,
        ExactBooleanResultKind::ArrangementCellComplexMaterialized {
            operation: ExactBooleanOperation::Union
        }
    );

    let closed_attempt = exact_arrangement_boolean_attempt_report_with_validation(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(closed_attempt.output_validation, ValidationPolicy::CLOSED);
    assert_eq!(
        closed_attempt.decline,
        Some(hypermesh::ExactArrangementBooleanDecline::OutputValidation),
        "{closed_attempt:?}"
    );
    assert_eq!(closed_attempt.output_vertices, result.mesh.vertices().len());
    assert_eq!(
        closed_attempt.output_triangles,
        result.mesh.triangles().len()
    );
    closed_attempt.validate().unwrap();
    closed_attempt
        .validate_against_sources_with_validation(&left, &right, ValidationPolicy::CLOSED)
        .unwrap();
    assert_eq!(
        closed_attempt.freshness_against_sources_with_validation(
            &left,
            &right,
            ValidationPolicy::CLOSED,
        ),
        ExactReportFreshness::Current
    );
    assert_eq!(
        closed_attempt.freshness_against_sources_with_validation(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
        ),
        ExactReportFreshness::SourceReplayMismatch
    );

    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    assert!(!result.region_classifications.is_empty());
    assert!(!result.triangulations.is_empty());
    assert!(!result.volumetric_classifications.is_empty());
    assert!(!result.assembly.triangles.is_empty());
    assert!(!result.mesh.triangles().is_empty());
    if result.volumetric_classifications.len() > 1 {
        let mut stale_volumetric_order = result.clone();
        stale_volumetric_order.volumetric_classifications.swap(0, 1);
        assert!(
            stale_volumetric_order.validate().is_ok(),
            "{stale_volumetric_order:?}"
        );
        assert_eq!(
            stale_volumetric_order.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch,
            "{stale_volumetric_order:?}"
        );
    }
    assert_eq!(
        result.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let convex_left = tetra_from_corners([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let convex_right = tetra_from_corners([1, 1, 1], [5, 1, 2], [1, 5, 1], [2, 1, 5]);
    assert!(
        materialize_volumetric_winding_arrangement(
            &convex_left,
            &convex_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none(),
        "direct volumetric wrapper should yield when public replay is a convex shortcut"
    );
}

#[test]
fn exact_volumetric_winding_coplanar_cap_is_publicly_certified() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4, //
            2, 2, 3,
        ],
        &[
            0, 2, 1, //
            1, 2, 3, //
            2, 0, 3, //
            0, 1, 4, //
            1, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap();
    let right = tetra_from_corners([-1, 1, 0], [3, 1, 0], [-1, 5, 0], [-1, 1, 4]);

    for operation in [
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let closure = certify_volumetric_boundary_closure_report(&left, &right, operation).unwrap();
        assert_eq!(
            closure.status,
            hypermesh::ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
            "{operation:?}: {closure:?}"
        );
        closure.validate().unwrap();
        closure.validate_against_sources(&left, &right).unwrap();

        let preflight = preflight_boolean_exact(&left, &right, operation).unwrap();
        assert_eq!(
            preflight.support,
            hypermesh::ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        let readiness = certify_winding_readiness_report(&left, &right, operation).unwrap();
        assert_eq!(
            readiness.status,
            ExactWindingReadinessStatus::ArrangementCellComplexAlreadyMaterialized,
            "{operation:?}: {readiness:?}"
        );
        readiness.validate().unwrap();
        readiness.validate_against_sources(&left, &right).unwrap();

        let result = materialize_volumetric_winding_arrangement(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("exact coplanar boundary cap should materialize closed volumetric output");
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex,
            },
            "{operation:?}: {result:?}"
        );
        result.validate().unwrap();
        assert!(
            result.mesh.facts().mesh.closed_manifold || result.mesh.triangles().is_empty(),
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );
        result.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current,
            "{operation:?}: {result:?}"
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_ok(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::SourceReplayMismatch,
            "{operation:?}: {stale_output:?}"
        );
    }
}

#[test]
fn arrangement_cell_complex_boolean_is_publicly_replayable() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4, //
            2, 2, 3,
        ],
        &[
            0, 2, 1, //
            1, 2, 3, //
            2, 0, 3, //
            0, 1, 4, //
            1, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let stale_right = tetra([10, 10, 10]);

    let result = materialize_arrangement_cell_complex_boolean(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .expect("certified arrangement cell-complex boolean should materialize");
    let replay = boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(result, replay);
    assert_eq!(
        result.kind,
        ExactBooleanResultKind::ArrangementCellComplexMaterialized {
            operation: ExactBooleanOperation::Union
        }
    );
    result.validate().unwrap();
    result.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        result.freshness_against_sources(&left, &stale_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(!result.region_classifications.is_empty());
    assert!(!result.triangulations.is_empty());
    assert!(!result.volumetric_classifications.is_empty());
    assert!(!result.assembly.triangles.is_empty());

    let horizontal = axis_aligned_box([0, 0, 0], [2, 1, 1]);
    let vertical = axis_aligned_box([0, 1, 0], [1, 2, 1]);
    let shortcut = materialize_arrangement_cell_complex_boolean(
        &horizontal,
        &vertical,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("orthogonal arrangement shortcut should materialize");
    assert_eq!(
        shortcut.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
        }
    );
    shortcut.validate().unwrap();
    shortcut
        .validate_against_sources(&horizontal, &vertical)
        .unwrap();
    shortcut
        .validate_operation_against_sources(
            &horizontal,
            &vertical,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let convex_left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let convex_right = tetra_from_corners([1, 1, 1], [5, 1, 1], [1, 5, 1], [1, 1, 5]);
    assert!(
        materialize_arrangement_cell_complex_boolean(
            &convex_left,
            &convex_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );
    assert!(
        materialize_closed_convex_boolean(
            &convex_left,
            &convex_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_some()
    );
}

#[test]
fn exact_contained_face_adjacent_union_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetra_from_corners([1, 1, 0], [1, 3, 0], [3, 1, 0], [1, 1, -2]);
    let separated_right = tetra([20, 0, 0]);

    let union = materialize_contained_face_adjacent_union(&left, &right, ValidationPolicy::CLOSED)
        .expect("contained coplanar cap should materialize as a holed union");
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.freshness_against_sources(&left, &right),
        ContainedFaceAdjacentUnionFreshness::Current
    );
    assert!(union.mesh.facts().mesh.closed_manifold);
    assert!(!union.contained_faces.is_empty());
    assert!(!union.containing_faces.is_empty());
    assert!(!union.mesh.triangles().is_empty());

    let mut invalid_output = union.clone();
    invalid_output.mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        invalid_output.freshness_against_sources(&left, &right),
        ContainedFaceAdjacentUnionFreshness::InvalidOutput
    );

    assert_eq!(
        union.freshness_against_sources(&left, &separated_right),
        ContainedFaceAdjacentUnionFreshness::SourceReplayMismatch
    );

    assert!(
        materialize_adjacent_union_completion_boolean(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );

    let disjoint_shell = tetra_from_corners([40, 0, 0], [41, 0, 0], [40, 1, 0], [40, 0, 1]);
    let container = combine_exact_meshes(
        &left,
        &disjoint_shell,
        "test disconnected contained-face fixture",
    );
    let completion_report =
        certify_adjacent_union_completion_report(&container, &right, ExactBooleanOperation::Union)
            .unwrap();
    assert_eq!(
        completion_report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
    );
    assert!(completion_report.is_certified());
    assert_eq!(
        completion_report.full_face_shared_faces + completion_report.full_face_shared_patches,
        0
    );
    assert!(completion_report.contained_faces > 0);
    assert!(completion_report.containing_faces > 0);
    assert!(completion_report.contained_containing_side.is_some());
    completion_report.validate().unwrap();
    completion_report
        .validate_against_sources(&container, &right)
        .unwrap();
    assert_eq!(
        completion_report.freshness_against_sources(&container, &right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        completion_report.freshness_against_sources(&container, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    let result = materialize_adjacent_union_completion_boolean(
        &container,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("contained-face adjacent solids should complete as a boolean union");
    assert_eq!(
        result.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
        }
    );
    result.validate().unwrap();
    result.validate_against_sources(&container, &right).unwrap();
    assert_eq!(
        result.freshness_against_sources(&container, &right),
        ExactReportFreshness::Current
    );
    let mut stale_output = result.clone();
    stale_output.mesh = container.clone();
    assert!(stale_output.validate().is_ok(), "{stale_output:?}");
    assert_eq!(
        stale_output.freshness_against_sources(&container, &right),
        ExactReportFreshness::SourceReplayMismatch,
        "{stale_output:?}"
    );
    assert_eq!(
        result.freshness_against_sources(&container, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );
    result
        .validate_operation_against_sources(
            &container,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
}

#[test]
fn exact_face_pair_classifier_rejects_disjoint_bounds_publicly() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    let pair = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
    assert_eq!(pair.relation, MeshFacePairRelation::BoundsDisjoint);
    assert!(pair.triangle.is_none());

    let graph = build_intersection_graph(&left, &right).unwrap();
    assert!(graph.face_pairs.is_empty());
}

#[test]
fn exact_triangle_classifier_reports_degenerate_coplanar_overlap() {
    let points = vec![
        p(0, 0, 0),
        p(2, 0, 0),
        p(0, 2, 0),
        p(0, 0, 0),
        p(1, 0, 0),
        p(0, 1, 0),
    ];

    let classification = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);
    assert_eq!(
        classification.relation,
        TriangleTriangleRelation::CoplanarOverlapping
    );
    assert!(classification.coplanar.is_some());
    assert!(classification.validate().is_ok());
    assert_eq!(
        classification.freshness_against_sources(&points, [0, 1, 2], [3, 4, 5]),
        TriangleTriangleFreshness::Current
    );
    let mut stale_classification = classification.clone();
    stale_classification.coplanar = None;
    assert_eq!(
        stale_classification.freshness_against_sources(&points, [0, 1, 2], [3, 4, 5]),
        TriangleTriangleFreshness::StaleCoplanarEvidence
    );
    let mut moved_points = points.clone();
    moved_points[4] = p(3, 0, 0);
    assert_eq!(
        classification.freshness_against_sources(&moved_points, [0, 1, 2], [3, 4, 5]),
        TriangleTriangleFreshness::SourceReplayMismatch
    );
}

#[test]
fn exact_face_pair_classifier_matches_local_triangle_report() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0, 9, 9, 9],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, -9, -9, -9],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let pair = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
    let points = vec![
        left.vertices()[0].clone(),
        left.vertices()[1].clone(),
        left.vertices()[2].clone(),
        right.vertices()[0].clone(),
        right.vertices()[1].clone(),
        right.vertices()[2].clone(),
    ];
    let direct = classify_triangle_triangle(&points, [0, 1, 2], [3, 4, 5]);

    assert_eq!(pair.relation, MeshFacePairRelation::CoplanarOverlapping);
    assert_eq!(pair.triangle.as_ref().unwrap(), &direct);
    pair.validate_against_sources(&left, &right).unwrap();

    let graph = build_intersection_graph(&left, &right).unwrap();
    let overlap = graph.coplanar_overlap_graphs().pop().unwrap();
    assert_eq!(
        overlap.freshness_against_sources(&left, &right),
        CoplanarOverlapGraphFreshness::Current
    );
    let mut invalid_overlap = overlap.clone();
    invalid_overlap.edge_overlaps.clear();
    invalid_overlap.vertex_overlaps.clear();
    assert_eq!(
        invalid_overlap.freshness_against_sources(&left, &right),
        CoplanarOverlapGraphFreshness::InvalidGraph
    );

    let split_plan = graph.coplanar_overlap_split_plan(&left, &right).unwrap();
    assert_eq!(
        split_plan.freshness_against_sources(&left, &right),
        CoplanarOverlapSplitFreshness::Current
    );
    let mut stale_split_plan = split_plan.clone();
    stale_split_plan.graphs.clear();
    assert_eq!(
        stale_split_plan.freshness_against_sources(&left, &right),
        CoplanarOverlapSplitFreshness::SourceReplayMismatch
    );

    let readiness = graph
        .coplanar_arrangement_readiness_report(&left, &right)
        .unwrap();
    assert_eq!(
        readiness.freshness_against_sources(&left, &right),
        CoplanarArrangementReadinessFreshness::Current
    );
    let mut invalid_readiness = readiness.clone();
    invalid_readiness.graph_count += 1;
    assert_eq!(
        invalid_readiness.freshness_against_sources(&left, &right),
        CoplanarArrangementReadinessFreshness::InvalidReadiness
    );
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[9, 0, 0, 10, 0, 0, 9, 1, 0, -9, -9, -9],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        readiness.freshness_against_sources(&left, &separated_right),
        CoplanarArrangementReadinessFreshness::SourceReplayMismatch
    );
}

#[test]
fn public_exact_blocker_reports_replay_remaining_decisions() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let overlapping_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[9, 0, 0, 10, 0, 0, 9, 1, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let refinement =
        certify_refinement_report(&left, &overlapping_right, ExactBooleanOperation::Union).unwrap();
    assert_eq!(refinement.status, ExactRefinementStatus::NotRequired);
    assert!(!refinement.is_required());
    refinement.validate().unwrap();
    refinement
        .validate_against_sources(&left, &overlapping_right)
        .unwrap();
    assert_eq!(
        refinement.freshness_against_sources(&left, &overlapping_right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        refinement.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let planar =
        certify_planar_arrangement_report(&left, &overlapping_right, ExactBooleanOperation::Union)
            .unwrap();
    assert_eq!(
        planar.status,
        ExactPlanarArrangementStatus::AlreadyMaterialized
    );
    assert!(!planar.is_required());
    planar.validate().unwrap();
    planar
        .validate_against_sources(&left, &overlapping_right)
        .unwrap();
    assert_eq!(
        planar.freshness_against_sources(&left, &overlapping_right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        planar.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let same_surface = certify_same_surface_report(&left, &left);
    assert_eq!(same_surface.status, ExactSameSurfaceStatus::Certified);
    assert!(same_surface.is_certified());
    same_surface.validate().unwrap();
    same_surface.validate_against_sources(&left, &left).unwrap();
    assert_eq!(
        same_surface.freshness_against_sources(&left, &left),
        ExactReportFreshness::Current
    );
    assert_eq!(
        same_surface.freshness_against_sources(&left, &separated_right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let parallel_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint = certify_open_surface_disjoint_report(&left, &parallel_right).unwrap();
    assert_eq!(
        open_disjoint.status,
        ExactOpenSurfaceDisjointStatus::Certified
    );
    assert!(open_disjoint.is_certified());
    open_disjoint.validate().unwrap();
    open_disjoint
        .validate_against_sources(&left, &parallel_right)
        .unwrap();
    assert_eq!(
        open_disjoint.freshness_against_sources(&left, &parallel_right),
        ExactReportFreshness::Current
    );
    assert_eq!(
        open_disjoint.freshness_against_sources(&left, &overlapping_right),
        ExactReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn open_surface_disjoint_report_classifies_retained_coplanar_overlap_blocker() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0],
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

    let report = certify_open_surface_disjoint_report(&left, &right).unwrap();

    assert_eq!(
        report.status,
        ExactOpenSurfaceDisjointStatus::GraphHasFacePairs
    );
    assert_eq!(
        report.blocker.kind,
        ExactBooleanBlockerKind::NeedsPlanarArrangement
    );
    assert!(report.blocker.coplanar_overlapping_pairs > 0);
    assert!(report.retained_face_pairs > 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let mut relabeled = report;
    relabeled.blocker.kind = ExactBooleanBlockerKind::NeedsBoundaryPolicy;
    assert_eq!(
        relabeled.validate(),
        Err(hypermesh::ExactReportValidationError::WrongBlockerKind)
    );
}

#[test]
fn planar_arrangement_report_classifies_noncoplanar_candidates_as_winding_blocker() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let report =
        certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Union).unwrap();

    assert_eq!(
        report.status,
        ExactPlanarArrangementStatus::NoPositiveOverlap
    );
    assert_eq!(report.blocker.kind, ExactBooleanBlockerKind::NeedsWinding);
    assert!(report.blocker.candidate_pairs > 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let mut stale = report;
    stale.blocker.kind = ExactBooleanBlockerKind::NeedsPlanarArrangement;
    assert!(stale.validate().is_err());
}

#[test]
fn exact_face_pair_candidate_retains_source_plane_split_events() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let pair = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
    assert_eq!(pair.relation, MeshFacePairRelation::Candidate);

    let triangle = pair.triangle.as_ref().unwrap();
    assert_eq!(triangle.relation, TriangleTriangleRelation::Candidate);
    assert_eq!(triangle.right_edge_events.len(), 3);
    assert_eq!(triangle.left_edge_events.len(), 3);
    assert!(
        triangle
            .right_edge_events
            .iter()
            .chain(&triangle.left_edge_events)
            .all(|event| event.predicates.is_empty())
    );
    pair.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        pair.freshness_against_sources(&left, &right),
        MeshFacePairFreshness::Current
    );
    let mut stale_pair = pair.clone();
    stale_pair.triangle = None;
    assert_eq!(
        stale_pair.freshness_against_sources(&left, &right),
        MeshFacePairFreshness::MissingTriangleEvidence
    );

    let graph = build_intersection_graph(&left, &right).unwrap();
    assert_eq!(
        graph.face_pairs[0].freshness_against_sources(&left, &right),
        IntersectionGraphFreshness::Current
    );
    assert_eq!(
        graph.freshness_against_sources(&left, &right),
        IntersectionGraphFreshness::Current
    );
    let mut stale_graph = graph.clone();
    stale_graph.face_pairs[0].events.clear();
    assert_eq!(
        stale_graph.freshness_against_sources(&left, &right),
        IntersectionGraphFreshness::InvalidGraph
    );

    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[9, -1, -1, 9, 3, 1, 9, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert_eq!(
        graph.freshness_against_sources(&left, &separated_right),
        IntersectionGraphFreshness::SourceReplayMismatch
    );
    assert_eq!(
        pair.freshness_against_sources(&left, &separated_right),
        MeshFacePairFreshness::SourceReplayMismatch
    );

    let edge_splits = graph.edge_split_plan();
    assert_eq!(
        edge_splits.freshness_against_sources(&left, &right),
        SplitPlanFreshness::Current
    );
    assert_eq!(
        edge_splits.freshness_against_sources(&left, &separated_right),
        SplitPlanFreshness::SourceReplayMismatch
    );
    let mut stale_edge_splits = edge_splits.clone();
    stale_edge_splits.unknown_orderings += 1;
    assert_eq!(
        stale_edge_splits.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );

    let graph_vertices = graph.graph_vertex_plan();
    assert_eq!(
        graph_vertices.freshness_against_sources(&left, &right),
        SplitPlanFreshness::Current
    );
    let topology = graph.split_topology_plan();
    assert_eq!(
        topology.freshness_against_sources(&left, &right),
        SplitPlanFreshness::Current
    );
    let face_plan = graph.face_split_plan();
    assert_eq!(
        face_plan.freshness_against_sources(&left, &right),
        SplitPlanFreshness::Current
    );
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
    assert_eq!(
        geometry.freshness_against_sources(&left, &right),
        SplitPlanFreshness::Current
    );
    let regions = geometry.region_plan(&left, &right);
    assert_eq!(
        regions.freshness_against_sources(&left, &right),
        SplitPlanFreshness::Current
    );

    let mut truncated = pair.clone();
    truncated.triangle.as_mut().unwrap().right_edge_events.pop();
    assert_eq!(
        truncated.validate(),
        Err(MeshFacePairValidationError::CandidateMissingEdgeEvents)
    );
}

#[test]
fn exact_boolean_public_shortcuts_handle_disjoint_operands() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
    assert!(!preflight.graph_had_unknowns);

    let union = boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .unwrap();
    assert_eq!(
        union.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint
        }
    );
    union.mesh.validate_retained_state().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert!(union.validate_against_sources(&left, &left).is_err());
    let mut relabeled = union.clone();
    relabeled.kind = ExactBooleanResultKind::CertifiedShortcut {
        operation: ExactBooleanOperation::Intersection,
        shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint,
    };
    assert!(
        relabeled
            .validate_operation_against_sources(
                &left,
                &right,
                ExactBooleanOperation::Union,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .is_err()
    );

    let intersection = boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(intersection.mesh.triangles().is_empty());
}

#[test]
fn trivial_boolean_materializers_are_publicly_replayable() {
    let empty = ExactMesh::new(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("test empty mesh"),
    )
    .unwrap();
    let solid = tetra([0, 0, 0]);
    let disjoint_solid = tetra([3, 0, 0]);
    let far_solid = tetra([20, 0, 0]);
    let farther_solid = tetra([30, 0, 0]);

    let open_identical_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_identical_right = open_identical_left.clone();
    let open_same_surface_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 0, 4, 0, 0, 0, 0],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 4, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 4, 0, 5, 0, 4, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_identical_alt_left = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 0, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_identical_alt_right = open_identical_alt_left.clone();
    let open_same_surface_alt_right = ExactMesh::from_i64_triangles_with_policy(
        &[14, 0, 0, 10, 4, 0, 10, 0, 0],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_alt_left = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 14, 0, 4, 10, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_disjoint_alt_right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 1, 14, 0, 5, 10, 4, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let assert_shortcut =
        |result: &ExactBooleanResult,
         left: &ExactMesh,
         right: &ExactMesh,
         stale_left: &ExactMesh,
         stale_right: &ExactMesh,
         operation: ExactBooleanOperation,
         validation: ValidationPolicy,
         shortcut: hypermesh::ExactBooleanShortcutKind| {
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut
                }
            );
            result.validate().unwrap();
            result.validate_against_sources(left, right).unwrap();
            assert_eq!(
                result.freshness_against_sources(left, right),
                ExactReportFreshness::Current
            );
            assert_eq!(
                result.freshness_against_sources(stale_left, stale_right),
                ExactReportFreshness::SourceReplayMismatch
            );
            result
                .validate_operation_against_sources(
                    left,
                    right,
                    operation,
                    validation,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap();
        };

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let empty_result =
            materialize_empty_operand_boolean(&empty, &solid, operation, ValidationPolicy::CLOSED)
                .unwrap()
                .expect("empty operand should materialize as an exact shortcut");
        assert_shortcut(
            &empty_result,
            &empty,
            &solid,
            &solid,
            &disjoint_solid,
            operation,
            ValidationPolicy::CLOSED,
            hypermesh::ExactBooleanShortcutKind::EmptyOperand,
        );
        if operation == ExactBooleanOperation::Union {
            assert_eq!(
                empty_result.freshness_against_sources(&empty, &disjoint_solid),
                ExactReportFreshness::SourceReplayMismatch
            );
        }

        let empty_open_result = boolean_exact(
            &empty,
            &open_disjoint_left,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_shortcut(
            &empty_open_result,
            &empty,
            &open_disjoint_left,
            &solid,
            &open_disjoint_left,
            operation,
            ValidationPolicy::CLOSED,
            hypermesh::ExactBooleanShortcutKind::EmptyOperand,
        );
        assert!(empty_open_result.mesh.triangles().is_empty());
        assert!(empty_open_result.mesh.facts().mesh.closed_manifold);

        let direct_empty_open = materialize_empty_operand_boolean(
            &empty,
            &open_disjoint_left,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("empty/open closed-output shortcut should materialize as empty");
        assert_eq!(direct_empty_open.kind, empty_open_result.kind);
        assert!(direct_empty_open.mesh.triangles().is_empty());
        assert!(
            materialize_closed_regularized_lower_dimensional_boolean(
                &empty,
                &open_disjoint_left,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .is_none(),
            "{operation:?} should preserve empty-operand provenance"
        );

        let open_empty_result = boolean_exact(
            &open_disjoint_left,
            &empty,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            open_empty_result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::EmptyOperand
            }
        );
        assert!(open_empty_result.mesh.triangles().is_empty());
        assert!(open_empty_result.mesh.facts().mesh.closed_manifold);
        open_empty_result
            .validate_operation_against_sources(
                &open_disjoint_left,
                &empty,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let disjoint_result = materialize_bounds_disjoint_boolean(
            &solid,
            &disjoint_solid,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("bounds-disjoint operands should materialize as an exact shortcut");
        assert_shortcut(
            &disjoint_result,
            &solid,
            &disjoint_solid,
            &solid,
            &solid,
            operation,
            ValidationPolicy::CLOSED,
            hypermesh::ExactBooleanShortcutKind::BoundsDisjoint,
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Difference
        ) {
            assert_eq!(
                disjoint_result.freshness_against_sources(&far_solid, &farther_solid),
                ExactReportFreshness::SourceReplayMismatch
            );
        }

        let identical_result = materialize_identical_mesh_boolean(
            &open_identical_left,
            &open_identical_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("identical open surfaces should materialize as an exact shortcut");
        assert_shortcut(
            &identical_result,
            &open_identical_left,
            &open_identical_right,
            &open_identical_left,
            &open_same_surface_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::ExactBooleanShortcutKind::Identical,
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
        ) {
            assert_eq!(
                identical_result
                    .freshness_against_sources(&open_identical_alt_left, &open_identical_alt_right),
                ExactReportFreshness::SourceReplayMismatch
            );
        }
        assert!(
            materialize_identical_mesh_boolean(
                &open_identical_left,
                &open_identical_right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .is_none(),
            "{operation:?} should yield to closed lower-dimensional regularization"
        );
        let closed_identical_result = boolean_exact(
            &open_identical_left,
            &open_identical_right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            closed_identical_result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            },
            "{operation:?}: {closed_identical_result:?}"
        );
        assert!(closed_identical_result.mesh.triangles().is_empty());
        assert!(closed_identical_result.mesh.facts().mesh.closed_manifold);
        closed_identical_result
            .validate_operation_against_sources(
                &open_identical_left,
                &open_identical_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let same_surface_result = materialize_same_surface_boolean(
            &open_identical_left,
            &open_same_surface_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("same-surface open meshes should materialize as an exact shortcut");
        assert_shortcut(
            &same_surface_result,
            &open_identical_left,
            &open_same_surface_right,
            &open_identical_left,
            &open_disjoint_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::ExactBooleanShortcutKind::SameSurface,
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Intersection
        ) {
            assert_eq!(
                same_surface_result.freshness_against_sources(
                    &open_identical_alt_left,
                    &open_same_surface_alt_right,
                ),
                ExactReportFreshness::SourceReplayMismatch
            );
        }
        assert!(
            materialize_same_surface_boolean(
                &open_identical_left,
                &open_same_surface_right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .is_none(),
            "{operation:?} should yield to closed lower-dimensional regularization"
        );
        let closed_same_surface_result = boolean_exact(
            &open_identical_left,
            &open_same_surface_right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap();
        assert_eq!(
            closed_same_surface_result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            },
            "{operation:?}: {closed_same_surface_result:?}"
        );
        assert!(closed_same_surface_result.mesh.triangles().is_empty());
        assert!(closed_same_surface_result.mesh.facts().mesh.closed_manifold);
        closed_same_surface_result
            .validate_operation_against_sources(
                &open_identical_left,
                &open_same_surface_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        let open_disjoint_result = materialize_open_surface_disjoint_boolean(
            &open_disjoint_left,
            &open_disjoint_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .expect("open surfaces with an empty exact graph should materialize as disjoint");
        assert_shortcut(
            &open_disjoint_result,
            &open_disjoint_left,
            &open_disjoint_right,
            &open_disjoint_left,
            &open_identical_left,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            hypermesh::ExactBooleanShortcutKind::OpenSurfaceDisjoint,
        );
        if matches!(
            operation,
            ExactBooleanOperation::Union | ExactBooleanOperation::Difference
        ) {
            assert_eq!(
                open_disjoint_result
                    .freshness_against_sources(&open_disjoint_alt_left, &open_disjoint_alt_right),
                ExactReportFreshness::SourceReplayMismatch
            );
        }
    }

    assert!(
        materialize_empty_operand_boolean(
            &solid,
            &disjoint_solid,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );
    assert!(
        materialize_same_surface_boolean(
            &open_identical_left,
            &open_identical_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .is_none()
    );
    assert!(
        materialize_open_surface_disjoint_boolean(
            &solid,
            &disjoint_solid,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );
}

#[test]
fn direct_boolean_materializers_yield_to_public_operation_replay() {
    type DirectBooleanMaterializer = fn(
        &ExactMesh,
        &ExactMesh,
        ExactBooleanOperation,
        ValidationPolicy,
    )
        -> Result<Option<ExactBooleanResult>, hypermesh::MeshError>;

    let materializers: &[(&str, DirectBooleanMaterializer)] = &[
        ("empty", materialize_empty_operand_boolean),
        ("bounds_disjoint", materialize_bounds_disjoint_boolean),
        ("identical", materialize_identical_mesh_boolean),
        ("same_surface", materialize_same_surface_boolean),
        (
            "closed_regularized_lower_dimensional",
            materialize_closed_regularized_lower_dimensional_boolean,
        ),
        (
            "mixed_dimensional_regularized_solid",
            materialize_mixed_dimensional_regularized_solid_boolean,
        ),
        (
            "open_surface_disjoint",
            materialize_open_surface_disjoint_boolean,
        ),
        (
            "closed_boundary_touching_regularized",
            materialize_closed_boundary_touching_regularized_boolean,
        ),
        (
            "closed_no_volume_overlap_regularized",
            materialize_closed_no_volume_overlap_regularized_boolean,
        ),
        (
            "closed_winding_separated",
            materialize_closed_winding_separated_boolean,
        ),
        (
            "closed_winding_containment",
            materialize_closed_winding_containment_boolean,
        ),
        ("closed_convex", materialize_closed_convex_boolean),
        (
            "axis_aligned_orthogonal_solid",
            materialize_axis_aligned_orthogonal_solid_boolean,
        ),
        (
            "affine_orthogonal_solid",
            materialize_affine_orthogonal_solid_boolean,
        ),
        (
            "adjacent_union_completion",
            materialize_adjacent_union_completion_boolean,
        ),
    ];

    let empty = ExactMesh::new(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("test empty direct replay audit"),
    )
    .unwrap();
    let solid = tetra([0, 0, 0]);
    let disjoint_solid = tetra([4, 0, 0]);
    let contained = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let container = combine_exact_meshes(
        &tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]),
        &tetra_from_corners([20, 0, 0], [21, 0, 0], [20, 1, 0], [20, 0, 1]),
        "test direct replay containment container",
    );
    let boundary_touching_right = tetra_from_corners([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
    let no_volume_right = tetra_from_corners([2, 0, 0], [6, 0, 0], [2, 4, 0], [2, 0, -4]);
    let open_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 4, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 1, 4, 0, 5, 0, 4, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let same_surface_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 0, 4, 0, 0, 0, 0],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let horizontal = axis_aligned_box([0, 0, 0], [2, 1, 1]);
    let vertical = axis_aligned_box([0, 1, 0], [1, 2, 1]);
    let orthogonal = materialize_axis_aligned_orthogonal_solid_union(
        &horizontal,
        &vertical,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("adjacent boxes should build an orthogonal audit fixture")
    .mesh;
    let orthogonal_right = axis_aligned_box([1, 0, 0], [3, 1, 1]);
    let affine_left = skew_affine_box([0, 0, 0], [2, 2, 2]);
    let affine_right = skew_affine_box([1, 1, 1], [3, 3, 3]);

    let fixtures: &[(&str, &ExactMesh, &ExactMesh)] = &[
        ("empty", &empty, &solid),
        ("bounds_disjoint_solids", &solid, &disjoint_solid),
        ("identical_open", &open_left, &open_left),
        ("same_surface_open", &open_left, &same_surface_right),
        ("open_disjoint", &open_left, &open_right),
        ("boundary_touching", &solid, &boundary_touching_right),
        ("positive_area_no_volume", &solid, &no_volume_right),
        ("winding_containment", &container, &contained),
        ("axis_orthogonal", &orthogonal, &orthogonal_right),
        ("affine_orthogonal", &affine_left, &affine_right),
    ];

    for (fixture_name, left, right) in fixtures {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            for validation in [ValidationPolicy::CLOSED, ValidationPolicy::ALLOW_BOUNDARY] {
                for (materializer_name, materializer) in materializers {
                    let Some(result) = materializer(left, right, operation, validation)
                        .unwrap_or_else(|error| {
                            panic!(
                                "{materializer_name} errored for {fixture_name} {operation:?} {validation:?}: {error:?}"
                            )
                        })
                    else {
                        continue;
                    };
                    result.validate().unwrap_or_else(|error| {
                        panic!(
                            "{materializer_name} produced invalid result for {fixture_name} {operation:?} {validation:?}: {error:?}"
                        )
                    });
                    result
                        .validate_operation_against_sources(
                            left,
                            right,
                            operation,
                            validation,
                            ExactBoundaryBooleanPolicy::Reject,
                        )
                        .unwrap_or_else(|error| {
                            let replay =
                                boolean_exact(left, right, operation, validation).unwrap_or_else(
                                    |replay_error| {
                                        panic!(
                                            "{materializer_name} produced unreplayable result for {fixture_name} {operation:?} {validation:?}: {error:?}; replay errored: {replay_error:?}; result={:?}",
                                            result.kind
                                        )
                                    },
                                );
                            panic!(
                                "{materializer_name} produced unreplayable result for {fixture_name} {operation:?} {validation:?}: {error:?}; result={:?}; replay={:?}",
                                result.kind, replay.kind
                            )
                        });
                }
            }
        }
    }
}

#[test]
fn closed_same_surface_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [14, 0, 0], [10, 4, 0], [10, 0, 4]);
    let left = combine_exact_meshes(&left_a, &left_b, "test disconnected same-surface left");
    let same_surface_a = ExactMesh::from_i64_triangles(
        &[
            4, 0, 0, //
            0, 0, 0, //
            0, 4, 0, //
            0, 0, 4,
        ],
        &[
            1, 2, 0, //
            1, 0, 3, //
            0, 2, 3, //
            2, 1, 3,
        ],
    )
    .unwrap();
    let same_surface_b = ExactMesh::from_i64_triangles(
        &[
            14, 0, 0, //
            10, 0, 0, //
            10, 4, 0, //
            10, 0, 4,
        ],
        &[
            1, 2, 0, //
            1, 0, 3, //
            0, 2, 3, //
            2, 1, 3,
        ],
    )
    .unwrap();
    let same_surface = combine_exact_meshes(
        &same_surface_a,
        &same_surface_b,
        "test disconnected same-surface right",
    );
    let stale_right = combine_exact_meshes(
        &tetra([20, 0, 0]),
        &tetra([30, 0, 0]),
        "test stale disconnected right",
    );

    for (right_index, right) in [&same_surface, &left].into_iter().enumerate() {
        for operation in [
            ExactBooleanOperation::Union,
            ExactBooleanOperation::Intersection,
            ExactBooleanOperation::Difference,
        ] {
            let result = materialize_closed_same_surface_boolean(
                &left,
                right,
                operation,
                ValidationPolicy::CLOSED,
            )
            .unwrap()
            .expect("closed same-surface solids should materialize through arrangement");
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
                }
            );
            result.validate().unwrap();
            result.validate_against_sources(&left, right).unwrap();
            assert_eq!(
                result.freshness_against_sources(&left, right),
                ExactReportFreshness::Current
            );
            let mut stale_output = result.clone();
            stale_output.mesh = stale_right.clone();
            assert!(stale_output.validate().is_ok(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, right),
                ExactReportFreshness::SourceReplayMismatch,
                "{stale_output:?}"
            );
            assert_eq!(
                result.freshness_against_sources(&left, &stale_right),
                ExactReportFreshness::SourceReplayMismatch
            );
            result
                .validate_operation_against_sources(
                    &left,
                    right,
                    operation,
                    ValidationPolicy::CLOSED,
                    ExactBoundaryBooleanPolicy::Reject,
                )
                .unwrap_or_else(|error| {
                    let replay = boolean_exact_with_boundary_policy(
                        &left,
                        right,
                        operation,
                        ValidationPolicy::CLOSED,
                        ExactBoundaryBooleanPolicy::Reject,
                    )
                    .unwrap();
                    panic!(
                        "right_index={right_index} operation={operation:?} error={error:?} result={:?} replay={:?}",
                        result.kind, replay.kind
                    );
                });
        }
    }

    let open_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let open_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 0, 4, 0, 0, 0, 0],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        materialize_closed_same_surface_boolean(
            &open_left,
            &open_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .unwrap()
        .is_none()
    );
    assert!(
        materialize_closed_same_surface_boolean(
            &left,
            &stale_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );

    let convex_left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let convex_same_surface = same_surface_a;
    assert!(
        materialize_closed_same_surface_boolean(
            &convex_left,
            &convex_same_surface,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_none()
    );
    assert!(
        materialize_closed_convex_boolean(
            &convex_left,
            &convex_same_surface,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .is_some()
    );
}

#[test]
fn exact_arrangement_public_path_reports_blockers_or_cells() {
    let left = tetra([0, 0, 0]);
    let right = tetra([1, 0, 0]);

    let arrangement = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    assert!(arrangement.validate_against_sources(&left, &right).is_ok());
    assert_eq!(
        arrangement.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactArrangementFreshness::Current
    );
    let mut stale_arrangement = arrangement.clone();
    stale_arrangement.vertices.pop();
    assert_eq!(
        stale_arrangement.freshness_against_sources_with_policy(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactArrangementFreshness::StaleArrangement
    );

    let labeled = arrangement
        .label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    assert_eq!(
        labeled.freshness_against_sources(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactLabeledCellComplexFreshness::Current
    );
    let mut stale_labeled = labeled.clone();
    stale_labeled.faces.pop();
    assert_eq!(
        stale_labeled.freshness_against_sources(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactLabeledCellComplexFreshness::StaleLabeledCells
    );

    let selected = labeled
        .select_with_policy(
            ExactBooleanOperation::Union,
            ExactRegularizationPolicy::REGULARIZED_SOLID,
        )
        .unwrap();
    assert_eq!(
        selected.freshness_against_sources(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactSelectedCellComplexFreshness::Current
    );
    let mut stale_selected = selected.clone();
    stale_selected.selected_faces.pop();
    assert_eq!(
        stale_selected.freshness_against_sources(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactSelectedCellComplexFreshness::StaleSelectedCells
    );

    let simplified = selected
        .simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
        .unwrap();
    assert_eq!(
        simplified.freshness_against_sources(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactSimplifiedCellComplexFreshness::Current
    );
    let mut stale_simplified = simplified.clone();
    stale_simplified.duplicate_cells_removed += 1;
    assert_eq!(
        stale_simplified.freshness_against_sources(
            &left,
            &right,
            ExactRegularizationPolicy::REGULARIZED_SOLID
        ),
        ExactSimplifiedCellComplexFreshness::StaleSimplifiedCells
    );

    let attempt = exact_arrangement_boolean_attempt_report(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .unwrap();
    attempt.validate().unwrap();
    attempt.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        attempt.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    let mut stale_attempt = attempt.clone();
    stale_attempt.arrangement_blockers += 1;
    assert_eq!(
        stale_attempt.freshness_against_sources(&left, &right),
        ExactReportFreshness::SourceReplayMismatch
    );
}

#[test]
fn exact_volumetric_region_reports_classify_freshness_publicly() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, //
            4, 0, 0, //
            0, 4, 0, //
            0, 0, 4, //
            2, 2, 3,
        ],
        &[
            0, 2, 1, //
            1, 2, 3, //
            2, 0, 3, //
            0, 1, 4, //
            1, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let result = boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(!result.volumetric_classifications.is_empty(), "{result:?}");
    let classification = result
        .volumetric_classifications
        .iter()
        .find(|classification| classification.relation != ExactVolumetricRegionRelation::Outside)
        .expect("overlapping fixture should retain a non-outside volumetric cell");
    let triangulation = result
        .triangulations
        .iter()
        .find(|triangulation| {
            triangulation.side == classification.region_side
                && triangulation.face == classification.region_face
                && triangulation
                    .triangles
                    .chunks_exact(3)
                    .any(|triangle| triangle == classification.triangle)
        })
        .expect("volumetric classification should reference a retained triangulation");
    let target = match classification.region_side {
        hypermesh::MeshSide::Left => &right,
        hypermesh::MeshSide::Right => &left,
    };
    assert_eq!(
        classification.freshness_against_sources(triangulation, target),
        ExactVolumetricRegionFreshness::Current
    );

    let mut stale_relation = classification.clone();
    stale_relation.relation = ExactVolumetricRegionRelation::Unknown;
    assert_eq!(
        stale_relation.freshness_against_sources(triangulation, target),
        ExactVolumetricRegionFreshness::StaleRelationEvidence
    );
    let mut stale_attempts = classification.clone();
    stale_attempts.witness_attempts.clear();
    assert_eq!(
        stale_attempts.freshness_against_sources(triangulation, target),
        ExactVolumetricRegionFreshness::InvalidRepresentativeEvidence
    );

    let shifted_target = tetra([10, 10, 10]);
    assert_eq!(
        classification.freshness_against_sources(triangulation, &shifted_target),
        ExactVolumetricRegionFreshness::SourceReplayMismatch
    );
}

#[test]
fn boundary_policy_remains_explicit_for_named_booleans() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 0, 2, 0, 2, 2, 2],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let report = certify_boundary_touching_report(&left, &right).unwrap();
    assert!(report.is_certified(), "{report:?}");
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::ExactBooleanSupport::RequiresBoundaryPolicy,
        "{preflight:?}"
    );
    let rejected_policy_preflight = preflight_boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    assert_eq!(rejected_policy_preflight, preflight);

    let policy_preflight = preflight_boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        policy_preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedBoundaryPolicyShortcut,
        "{policy_preflight:?}"
    );
    assert!(policy_preflight.blocker.is_none(), "{policy_preflight:?}");
    assert_eq!(
        policy_preflight.retained_face_pairs,
        report.retained_face_pairs
    );
    assert_eq!(policy_preflight.retained_events, report.retained_events);
    policy_preflight
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();
    assert_eq!(
        policy_preflight.freshness_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
        hypermesh::ExactReportFreshness::Current
    );
    assert!(
        policy_preflight
            .validate_against_sources(&left, &right)
            .is_err(),
        "strict replay should not certify a boundary-policy preflight"
    );

    let rejected_readiness = certify_winding_readiness_report_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    assert_eq!(
        rejected_readiness.status,
        ExactWindingReadinessStatus::BoundaryPolicyRequired,
        "{rejected_readiness:?}"
    );

    let policy_readiness = certify_winding_readiness_report_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        policy_readiness.status,
        ExactWindingReadinessStatus::BoundaryPolicyShortcutAlreadyMaterialized,
        "{policy_readiness:?}"
    );
    assert_eq!(
        policy_readiness.blocker.kind,
        ExactBooleanBlockerKind::NeedsBoundaryPolicy
    );
    assert_eq!(
        policy_readiness.retained_face_pairs,
        report.retained_face_pairs
    );
    assert_eq!(policy_readiness.retained_events, report.retained_events);
    policy_readiness.validate().unwrap();
    policy_readiness
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();
    assert_eq!(
        policy_readiness.freshness_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ),
        hypermesh::ExactReportFreshness::Current
    );
    assert!(
        policy_readiness
            .validate_against_sources(&left, &right)
            .is_err(),
        "strict replay should not certify a boundary-policy readiness report"
    );
    assert!(
        boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY
        )
        .is_err()
    );

    let projected = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        projected.kind,
        ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: ExactBooleanOperation::Union
        }
    );
    projected.mesh.validate_retained_state().unwrap();
    projected.validate_against_sources(&left, &right).unwrap();

    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[5, 0, 0, 7, 0, 0, 5, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        projected
            .validate_against_sources(&left, &separated_right)
            .is_err()
    );
    assert!(
        policy_preflight
            .validate_against_sources_with_boundary_policy(
                &left,
                &separated_right,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .is_err()
    );

    let closed_intersection_preflight = preflight_boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        closed_intersection_preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid,
        "{closed_intersection_preflight:?}"
    );
    closed_intersection_preflight
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();

    let closed_intersection_readiness = certify_winding_readiness_report_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        closed_intersection_readiness.status,
        ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
        "{closed_intersection_readiness:?}"
    );
    closed_intersection_readiness
        .validate_against_sources_with_boundary_policy(
            &left,
            &right,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();

    let closed_intersection = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    assert_eq!(
        closed_intersection.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Intersection,
            shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
        }
    );
    assert!(closed_intersection.mesh.triangles().is_empty());
    assert!(closed_intersection.mesh.facts().mesh.closed_manifold);
    closed_intersection
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Difference,
    ] {
        let closed_policy_preflight = preflight_boolean_exact_with_boundary_policy(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();
        assert_eq!(
            closed_policy_preflight.support,
            hypermesh::ExactBooleanSupport::CertifiedLowerDimensionalRegularizedSolid,
            "{operation:?}: {closed_policy_preflight:?}"
        );
        closed_policy_preflight
            .validate_against_sources_with_boundary_policy(
                &left,
                &right,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
        let closed_policy_readiness = certify_winding_readiness_report_with_boundary_policy(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();
        assert_eq!(
            closed_policy_readiness.status,
            ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
            "{operation:?}: {closed_policy_readiness:?}"
        );
        assert!(
            materialize_boundary_touching_policy_boolean(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap()
            .is_none(),
            "{operation:?} should remain blocked because preserving open shells cannot satisfy CLOSED output validation"
        );
        let closed_regularized = boolean_exact_with_boundary_policy(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .unwrap();
        assert_eq!(
            closed_regularized.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            },
            "{operation:?}: {closed_regularized:?}"
        );
    }
}

#[test]
fn boundary_touching_report_classifies_proper_crossing_as_winding_blocker() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let report = certify_boundary_touching_report(&left, &right).unwrap();

    assert_eq!(report.status, ExactBoundaryTouchingStatus::NotBoundaryOnly);
    assert_eq!(report.blocker.kind, ExactBooleanBlockerKind::NeedsWinding);
    assert!(report.blocker.candidate_pairs > 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let mut stale = report;
    stale.blocker.kind = ExactBooleanBlockerKind::NeedsBoundaryPolicy;
    assert!(stale.validate().is_err());
}
