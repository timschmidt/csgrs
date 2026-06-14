use hyperlimit::{Point2, Point3, SourceProvenance};
use hypermesh::{
    AffineOrthogonalSolidFreshness, ApproximateMeshF64ViewFreshness,
    AxisAlignedOrthogonalSolidFreshness, ClosedMeshOrientation, ContainedFaceAdjacentUnionError,
    ContainedFaceAdjacentUnionFreshness, ConvexSolidMeshRelation, ConvexSolidPointRelation,
    ConvexSolidReportFreshness, CoplanarArrangementReadinessFreshness,
    CoplanarOverlapGraphFreshness, CoplanarOverlapSplitFreshness,
    CoplanarVolumetricCellEvidenceFreshness, ExactAdjacentUnionCompletionStatus, ExactArrangement,
    ExactArrangement2dBoundaryPolicy, ExactArrangement2dRegion, ExactArrangement2dRegionRing,
    ExactArrangement2dSetOperation, ExactArrangementBlocker, ExactArrangementFreshness,
    ExactBooleanBlockerKind, ExactBooleanOperation, ExactBooleanRequest, ExactBooleanResult,
    ExactBooleanResultKind, ExactBoundaryBooleanPolicy, ExactBoundaryTouchingStatus,
    ExactI64MeshInputReadiness, ExactI64MeshInputReportValidationError,
    ExactLabeledCellComplexFreshness, ExactMesh, ExactMeshAuditError, ExactMeshConsumerDomain,
    ExactMeshConsumerReadinessError, ExactMeshDomainSummaryFreshness, ExactMeshHandoffPackageError,
    ExactMeshHandoffPackageFreshness, ExactMeshProposalAcceptance, ExactMeshProposalReportError,
    ExactMeshProposalSourceKind, ExactOpenSurfaceDisjointStatus, ExactOutputTriangleOrientation,
    ExactPlanarArrangementStatus, ExactRefinementStatus, ExactRegionOwnershipStatus,
    ExactRegionSelection, ExactRegularizationPolicy, ExactReportFreshness, ExactSameSurfaceStatus,
    ExactSelectedCellComplexFreshness, ExactSimplifiedCellComplexFreshness,
    ExactTopologyAssemblyStatus, ExactVolumetricRegionFreshness, ExactVolumetricRegionRelation,
    ExactWindingReadinessStatus, FaceRegionPlaneRelation, FullFaceAdjacentUnionFreshness,
    IntersectionGraphFreshness, LossyF64MeshInputReadiness, LossyF64MeshInputReportValidationError,
    MeshArtifactBlocker, MeshArtifactFaceRecord, MeshArtifactManifest, MeshArtifactReportError,
    MeshArtifactRole, MeshArtifactSourceKind, MeshArtifactVertexRecord, MeshCoordinateEvidence,
    MeshFacePairFreshness, MeshFacePairRelation, MeshFacePairValidationError, MeshTopologyEvidence,
    SplitPlanFreshness, TriangleTriangleFreshness, TriangleTriangleRelation, ValidationPolicy,
    WindingReportFreshness, approximate_mesh_f64_view, audit_exact_mesh,
    build_exact_arrangement2d_overlay, build_exact_arrangement2d_overlay_with_boundary_policy,
    build_intersection_graph, certify_convex_solid, certify_coplanar_volumetric_cell_evidence,
    certify_exact_mesh_proposal, checked_classify_face_regions_against_opposite_planes,
    checked_triangulate_face_regions_with_earcut, classify_mesh_face_pair,
    classify_mesh_vertices_against_closed_mesh_winding_report,
    classify_mesh_vertices_against_convex_solid_report,
    classify_point_against_closed_mesh_winding_report, classify_point_against_convex_solid_report,
    classify_triangle_triangle, exact_mesh_consumer_readiness, exact_mesh_handoff_package,
    inspect_f64_mesh_input, inspect_i64_mesh_input, materialize_affine_orthogonal_solid_difference,
    materialize_affine_orthogonal_solid_intersection, materialize_arrangement_cell_complex_boolean,
    materialize_axis_aligned_orthogonal_solid_difference,
    materialize_axis_aligned_orthogonal_solid_intersection,
    materialize_axis_aligned_orthogonal_solid_union, materialize_contained_face_adjacent_union,
    materialize_coplanar_mesh_overlay_arrangement, materialize_full_face_adjacent_union,
    materialize_open_surface_arrangement, materialize_volumetric_coplanar_boundary_closure_boolean,
    materialize_volumetric_winding_arrangement, mesh_artifact_from_exact_mesh,
    mesh_artifact_from_exact_mesh_proposal, triangulate_all_face_cells_with_cdt,
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

fn face_fan_box() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2, 1, 1, 0,
        ],
        &[
            0, 8, 1, //
            1, 8, 2, //
            2, 8, 3, //
            3, 8, 0, //
            4, 5, 6, //
            4, 6, 7, //
            0, 1, 5, //
            0, 5, 4, //
            1, 2, 6, //
            1, 6, 5, //
            2, 3, 7, //
            2, 7, 6, //
            3, 0, 4, //
            3, 4, 7,
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
fn exact_boolean_evaluation_materializes_certified_result_publicly() {
    let left = tetra([0, 0, 0]);
    let right = tetra([4, 0, 0]);
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );

    let evaluation = (request).evaluate(&left, &right).unwrap();

    evaluation.validate().unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        evaluation.freshness_against_sources(&left, &right),
        hypermesh::ExactReportFreshness::Current
    );
    assert!(evaluation.is_certified());
    assert!(evaluation.is_materialized());
    assert_eq!(evaluation.required_blocker_kind(), None);
    assert!(evaluation.preflight.is_certified());
    assert!(evaluation.result.as_ref().is_some_and(|result| {
        matches!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation: ExactBooleanOperation::Union,
                ..
            }
        )
    }));
    let stale_right = tetra([8, 0, 0]);
    assert!(
        evaluation
            .validate_against_sources(&left, &stale_right)
            .is_err()
    );
    assert_eq!(
        evaluation.freshness_against_sources(&left, &stale_right),
        hypermesh::ExactReportFreshness::SourceReplayMismatch
    );
    let mut relabeled_request = evaluation.clone();
    relabeled_request.request = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    );
    assert_eq!(
        relabeled_request.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_attempt_policy = evaluation.clone();
    stale_attempt_policy
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt")
        .output_validation = ValidationPolicy::CLOSED;
    assert_eq!(
        stale_attempt_policy.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut relabeled_support = evaluation.clone();
    relabeled_support.preflight.support = hypermesh::ExactBooleanSupport::CertifiedConvexUnion;
    relabeled_support.preflight.validate().unwrap();
    assert_eq!(
        relabeled_support.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut relabeled_winding_status = evaluation.clone();
    relabeled_winding_status
        .certifications
        .winding_readiness
        .status = ExactWindingReadinessStatus::EmptyOperandAlreadyMaterialized;
    relabeled_winding_status
        .certifications
        .winding_readiness
        .validate()
        .unwrap();
    assert_eq!(
        relabeled_winding_status.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
}

#[test]
fn exact_boolean_certifications_retain_region_ownership_report() {
    let left = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);
    let request = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED);

    let evaluation = request.evaluate(&left, &right).unwrap();

    evaluation.validate().unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    let ownership = evaluation
        .certifications
        .region_ownership
        .as_ref()
        .expect("named boolean certifications should retain region ownership");
    ownership.validate().unwrap();
    assert_eq!(ownership.status, ExactRegionOwnershipStatus::VolumeResolved);
    assert!(ownership.is_resolved());
    assert_eq!(ownership.volume_regions, 3);
    assert_eq!(ownership.shared_owned_volumes, 1);

    let mut missing_ownership = evaluation.clone();
    missing_ownership.certifications.region_ownership = None;
    assert_eq!(
        missing_ownership.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );

    let mut missing_topology = evaluation.clone();
    missing_topology.certifications.topology_assembly = None;
    assert_eq!(
        missing_topology.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
}

#[test]
fn exact_boolean_evaluation_materializes_boundary_policy_shortcut_by_default() {
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
    let request = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    );

    let evaluation = (request).evaluate(&left, &right).unwrap();

    evaluation.validate().unwrap();
    evaluation.validate_against_sources(&left, &right).unwrap();
    assert!(evaluation.is_certified());
    assert!(evaluation.is_materialized());
    assert_eq!(evaluation.required_blocker_kind(), None);
    assert!(evaluation.preflight.is_certified());
    assert!(evaluation.preflight.has_retained_exact_evidence());
    assert!(evaluation.result.as_ref().is_some_and(|result| {
        matches!(
            result.kind,
            ExactBooleanResultKind::BoundaryPolicyShortcut {
                operation: ExactBooleanOperation::Union
            }
        )
    }));
    let mut mixed_graph_snapshot = evaluation.clone();
    mixed_graph_snapshot
        .certifications
        .refinement
        .retained_face_pairs = 0;
    mixed_graph_snapshot
        .certifications
        .refinement
        .retained_events = 0;
    mixed_graph_snapshot
        .certifications
        .refinement
        .validate()
        .unwrap();
    assert_eq!(
        mixed_graph_snapshot.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut relabeled_winding_status = evaluation.clone();
    relabeled_winding_status
        .certifications
        .winding_readiness
        .status = ExactWindingReadinessStatus::BoundaryPolicyRequired;
    relabeled_winding_status
        .certifications
        .winding_readiness
        .validate()
        .unwrap();
    assert_eq!(
        relabeled_winding_status.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let rejected_request = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    );
    let rejected = (rejected_request).evaluate(&left, &right).unwrap();
    rejected.validate().unwrap();
    assert!(!rejected.is_certified());
    assert!(!rejected.is_materialized());
    let mut impossible_materialization = rejected.clone();
    impossible_materialization.result = evaluation.result.clone();
    assert_eq!(
        impossible_materialization.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
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

fn skew_affine_mesh_from_axis_aligned(mesh: &ExactMesh, label: &'static str) -> ExactMesh {
    let ten = Real::from(10);
    ExactMesh::new(
        mesh.vertices()
            .iter()
            .map(|point| {
                Point3::new(
                    point.x.clone() + &(point.y.clone() * &ten),
                    point.y.clone(),
                    point.z.clone(),
                )
            })
            .collect(),
        mesh.triangles().to_vec(),
        SourceProvenance::exact(label),
    )
    .unwrap()
}

fn axis_aligned_l_solid(offset: [i64; 3]) -> ExactMesh {
    let [ox, oy, oz] = offset;
    let horizontal = axis_aligned_box([ox, oy, oz], [ox + 2, oy + 1, oz + 1]);
    let vertical = axis_aligned_box([ox, oy + 1, oz], [ox + 1, oy + 2, oz + 1]);
    materialize_axis_aligned_orthogonal_solid_union(
        &horizontal,
        &vertical,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("test L solid should materialize")
    .mesh
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

    let audit = audit_exact_mesh(&mesh).unwrap();
    audit.validate().unwrap();
    audit.validate_against_mesh(&mesh).unwrap();

    let mut impossible_predicate_counts = audit.clone();
    impossible_predicate_counts.proof_predicates =
        impossible_predicate_counts.predicate_uses.saturating_add(1);
    assert_eq!(
        impossible_predicate_counts.validate(),
        Err(ExactMeshAuditError::InvalidPredicateCounts {
            predicate_uses: impossible_predicate_counts.predicate_uses,
            proof_predicates: impossible_predicate_counts.proof_predicates
        })
    );
    assert_eq!(
        impossible_predicate_counts.validate_against_mesh(&mesh),
        Err(ExactMeshAuditError::InvalidPredicateCounts {
            predicate_uses: impossible_predicate_counts.predicate_uses,
            proof_predicates: impossible_predicate_counts.proof_predicates
        })
    );

    let mut empty_topology_audit = audit.clone();
    empty_topology_audit.vertex_count = 0;
    assert_eq!(
        empty_topology_audit.validate(),
        Err(ExactMeshAuditError::EmptyTopology)
    );

    let mut empty_source_audit = audit.clone();
    empty_source_audit.source_label.clear();
    assert_eq!(
        empty_source_audit.validate(),
        Err(ExactMeshAuditError::EmptySourceLabel)
    );

    let mut invalid_version_audit = audit.clone();
    invalid_version_audit.construction_version = 0;
    assert_eq!(
        invalid_version_audit.validate(),
        Err(ExactMeshAuditError::InvalidConstructionVersion)
    );

    let report = inspect_i64_mesh_input(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    );
    let readiness = report.readiness();
    assert_eq!(readiness, ExactI64MeshInputReadiness::Ready);
    assert!(report.edge_ready());

    let mut missing_integer_evidence = report.clone();
    missing_integer_evidence.exact_integer_coordinates -= 1;
    assert_eq!(
        missing_integer_evidence.validate(),
        Err(ExactI64MeshInputReportValidationError::ExactCoordinateCountMismatch)
    );
    assert_eq!(
        missing_integer_evidence.readiness(),
        ExactI64MeshInputReadiness::InvalidReport
    );

    let mut missing_checked_indices = report.clone();
    missing_checked_indices.checked_indices -= 1;
    assert_eq!(
        missing_checked_indices.validate(),
        Err(ExactI64MeshInputReportValidationError::CheckedIndexCountMismatch)
    );

    let mut missing_arity_diagnostic = inspect_i64_mesh_input(&[0, 0], &[0, 1, 2]);
    missing_arity_diagnostic.diagnostics.clear();
    assert_eq!(
        missing_arity_diagnostic.validate(),
        Err(ExactI64MeshInputReportValidationError::MissingCoordinateArityDiagnostic)
    );

    let mut missing_index_arity_diagnostic = inspect_i64_mesh_input(&[0, 0, 0], &[0, 1]);
    missing_index_arity_diagnostic.diagnostics.clear();
    assert_eq!(
        missing_index_arity_diagnostic.validate(),
        Err(ExactI64MeshInputReportValidationError::MissingIndexArityDiagnostic)
    );

    let lossy_report =
        inspect_f64_mesh_input(&[0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0], &[0, 1, 2]);
    assert_eq!(lossy_report.readiness(), LossyF64MeshInputReadiness::Ready);

    let mut missing_dyadic_evidence = lossy_report.clone();
    missing_dyadic_evidence.exact_dyadic_coordinates -= 1;
    assert_eq!(
        missing_dyadic_evidence.validate(),
        Err(LossyF64MeshInputReportValidationError::ExactCoordinateCountMismatch)
    );
    assert_eq!(
        missing_dyadic_evidence.readiness(),
        LossyF64MeshInputReadiness::InvalidReport
    );

    let mut missing_float_diagnostic = inspect_f64_mesh_input(&[0.0, f64::NAN, 0.0], &[0, 1, 2]);
    assert_eq!(
        missing_float_diagnostic.readiness(),
        LossyF64MeshInputReadiness::InvalidCoordinate
    );
    missing_float_diagnostic.diagnostics.clear();
    assert_eq!(
        missing_float_diagnostic.validate(),
        Err(LossyF64MeshInputReportValidationError::ExactCoordinateCountMismatch)
    );
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

    let mut invalid_proposal_audit = proposal.clone();
    invalid_proposal_audit.audit.proof_predicates = invalid_proposal_audit
        .audit
        .predicate_uses
        .saturating_add(1);
    assert_eq!(
        invalid_proposal_audit.validate(),
        Err(ExactMeshProposalReportError::AuditReplay(
            ExactMeshAuditError::InvalidPredicateCounts {
                predicate_uses: invalid_proposal_audit.audit.predicate_uses,
                proof_predicates: invalid_proposal_audit.audit.proof_predicates
            }
        ))
    );

    let artifact = mesh_artifact_from_exact_mesh(&exact).unwrap();
    artifact.validate().unwrap();
    assert_eq!(artifact.source_kind, MeshArtifactSourceKind::HypermeshExact);
    assert_eq!(artifact.role, MeshArtifactRole::SolidHandoff);
    assert!(artifact.validation_handoff_ready, "{:?}", artifact.blockers);
    assert!(artifact.blockers.is_empty());

    let mut forged_handoff_ready = artifact.clone();
    forged_handoff_ready.validation_handoff_ready = false;
    assert_eq!(
        forged_handoff_ready.validate(),
        Err(MeshArtifactReportError::ReportMismatch {
            field: "validation_handoff_ready"
        })
    );

    let proposal_artifact = mesh_artifact_from_exact_mesh_proposal(&exact, &proposal).unwrap();
    proposal_artifact.validate().unwrap();
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
    lossy_artifact.validate().unwrap();
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
    preview.validate().unwrap();
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
    let mut preview_missing_blocker = preview.clone();
    preview_missing_blocker
        .blockers
        .retain(|blocker| *blocker != MeshArtifactBlocker::MissingExactCoordinateReplay);
    assert_eq!(
        preview_missing_blocker.validate(),
        Err(MeshArtifactReportError::MissingBlocker {
            blocker: MeshArtifactBlocker::MissingExactCoordinateReplay
        })
    );
    let mut duplicate_preview_blocker = preview.clone();
    duplicate_preview_blocker
        .blockers
        .push(MeshArtifactBlocker::PreviewOrExportOnly);
    assert_eq!(
        duplicate_preview_blocker.validate(),
        Err(MeshArtifactReportError::DuplicateBlocker {
            blocker: MeshArtifactBlocker::PreviewOrExportOnly
        })
    );

    let brep_triangle_handoff = |face| {
        MeshArtifactManifest::brep_exact_triangle_handoff(
            "brep exact triangle handoff",
            1,
            vec![
                MeshArtifactVertexRecord {
                    index: 0,
                    coordinate_evidence: MeshCoordinateEvidence::CertifiedDerivedExact,
                },
                MeshArtifactVertexRecord {
                    index: 1,
                    coordinate_evidence: MeshCoordinateEvidence::CertifiedDerivedExact,
                },
                MeshArtifactVertexRecord {
                    index: 2,
                    coordinate_evidence: MeshCoordinateEvidence::CertifiedDerivedExact,
                },
            ],
            vec![face],
        )
    };

    let repeated_vertex_handoff = brep_triangle_handoff(MeshArtifactFaceRecord {
        index: 0,
        vertices: vec![0, 1, 1],
        topology_evidence: MeshTopologyEvidence::DerivedExactSurfaceHandoff,
    })
    .report();
    repeated_vertex_handoff.validate().unwrap();
    assert!(!repeated_vertex_handoff.validation_handoff_ready);
    assert!(!repeated_vertex_handoff.topology_validation_replay_ready);
    assert!(
        repeated_vertex_handoff
            .blockers
            .contains(&MeshArtifactBlocker::FaceRepeatedVertex),
        "{repeated_vertex_handoff:?}"
    );

    let mut missing_vertex_record_manifest = brep_triangle_handoff(MeshArtifactFaceRecord {
        index: 0,
        vertices: vec![0, 1, 2],
        topology_evidence: MeshTopologyEvidence::DerivedExactSurfaceHandoff,
    });
    missing_vertex_record_manifest.declared_vertex_count += 1;
    let missing_vertex_record = missing_vertex_record_manifest.report();
    missing_vertex_record.validate().unwrap();
    assert!(!missing_vertex_record.validation_handoff_ready);
    assert!(!missing_vertex_record.coordinates_exact_replay_ready);
    assert!(
        missing_vertex_record
            .blockers
            .contains(&MeshArtifactBlocker::MissingOrMismatchedVertexRecords),
        "{missing_vertex_record:?}"
    );

    let mut stale_face_index_manifest = brep_triangle_handoff(MeshArtifactFaceRecord {
        index: 0,
        vertices: vec![0, 1, 2],
        topology_evidence: MeshTopologyEvidence::DerivedExactSurfaceHandoff,
    });
    stale_face_index_manifest.faces[0].index = 1;
    let stale_face_index = stale_face_index_manifest.report();
    stale_face_index.validate().unwrap();
    assert!(!stale_face_index.validation_handoff_ready);
    assert!(!stale_face_index.topology_validation_replay_ready);
    assert!(
        stale_face_index
            .blockers
            .contains(&MeshArtifactBlocker::FaceIndexMismatch),
        "{stale_face_index:?}"
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

    let mut invalid_readiness_package = package.clone();
    invalid_readiness_package.readiness.closed_manifold = false;
    assert!(matches!(
        invalid_readiness_package.validate_internal(),
        Err(ExactMeshHandoffPackageError::InternalMismatch { field: "readiness" })
    ));
    assert_eq!(
        invalid_readiness_package.freshness_against_mesh(&solid),
        ExactMeshHandoffPackageFreshness::StalePackage
    );

    let mut understated_surface_readiness = package.readiness.clone();
    understated_surface_readiness.surface_handoff_ready = false;
    assert_eq!(
        understated_surface_readiness.validate(),
        Err(ExactMeshConsumerReadinessError::ReportMismatch {
            field: "surface_handoff_ready"
        })
    );

    let mut stale_face_plane_readiness = package.readiness.clone();
    stale_face_plane_readiness.retained_face_planes -= 1;
    assert_eq!(
        stale_face_plane_readiness.validate(),
        Err(ExactMeshConsumerReadinessError::ReportMismatch {
            field: "retained_face_planes"
        })
    );

    let mut missing_bounds_readiness = package.readiness.clone();
    missing_bounds_readiness.retained_mesh_bounds = false;
    assert_eq!(
        missing_bounds_readiness.validate(),
        Err(ExactMeshConsumerReadinessError::ReportMismatch {
            field: "retained_mesh_bounds"
        })
    );

    let mut invalid_surface_package = package.clone();
    invalid_surface_package
        .surface
        .as_mut()
        .unwrap()
        .nonempty_topology = false;
    assert!(matches!(
        invalid_surface_package.validate_internal(),
        Err(ExactMeshHandoffPackageError::InternalMismatch { field: "surface" })
    ));
    assert_eq!(
        invalid_surface_package.freshness_against_mesh(&solid),
        ExactMeshHandoffPackageFreshness::StalePackage
    );

    let mut invalid_solid_package = package.clone();
    invalid_solid_package
        .solid
        .as_mut()
        .unwrap()
        .retained_face_planes -= 1;
    assert!(matches!(
        invalid_solid_package.validate_internal(),
        Err(ExactMeshHandoffPackageError::InternalMismatch { field: "solid" })
    ));

    let mut invalid_view_package = package.clone();
    invalid_view_package
        .approximate_f64_view
        .as_mut()
        .unwrap()
        .exported_coordinates += 1;
    assert!(matches!(
        invalid_view_package.validate_internal(),
        Err(ExactMeshHandoffPackageError::InternalMismatch {
            field: "approximate_f64_view"
        })
    ));

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

    let mut invalid_summary = summary.clone();
    invalid_summary
        .available_domains
        .push(ExactMeshConsumerDomain::Surface);
    assert!(matches!(
        invalid_summary.validate(),
        Err(hypermesh::ExactMeshDomainSummaryError::SummaryMismatch {
            field: "available_domains"
        })
    ));
    assert_eq!(
        invalid_summary.freshness_against_package(&package),
        ExactMeshDomainSummaryFreshness::StaleSummary
    );

    let mut contradictory_summary = summary.clone();
    contradictory_summary.exact_geometry_domains.clear();
    assert!(matches!(
        contradictory_summary.validate(),
        Err(hypermesh::ExactMeshDomainSummaryError::SummaryMismatch {
            field: "exact_geometry_domains"
        })
    ));

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
        let disjoint_replay = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &separated_right)
            .unwrap();
        assert_eq!(
            disjoint_replay.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint
            }
        );

        let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &right)
            .unwrap();
        let expected_shortcut = match operation {
            ExactBooleanOperation::Union => {
                hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
            }
            ExactBooleanOperation::Intersection => {
                hypermesh::ExactBooleanShortcutKind::ConvexIntersection
            }
            ExactBooleanOperation::Difference => {
                hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
            }
            ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
        };
        assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: expected_shortcut
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
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
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
fn affine_orthogonal_solid_recovers_multi_cell_basis_without_sampling_limits() {
    let left_axis = axis_aligned_l_solid([0, 0, 0]);
    let right_axis = axis_aligned_l_solid([1, 0, 0]);
    let left = skew_affine_mesh_from_axis_aligned(&left_axis, "test skew affine left L solid");
    let right = skew_affine_mesh_from_axis_aligned(&right_axis, "test skew affine right L solid");

    assert!(left.vertices().len() > 8);
    assert!(right.vertices().len() > 8);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
            .preflight(&left, &right)
            .unwrap();
        assert_eq!(
            preflight.support,
            hypermesh::ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        assert!(preflight.blocker.is_none(), "{operation:?}: {preflight:?}");
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &right)
            .unwrap();
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
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
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
        let disjoint_replay = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &separated_right)
            .unwrap();
        assert_eq!(
            disjoint_replay.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint
            }
        );

        let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &right)
            .unwrap();
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
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
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
fn axis_aligned_orthogonal_solid_accepts_face_fan_triangulated_box() {
    let fan_box = face_fan_box();
    let cutter = axis_aligned_box([1, 0, 0], [3, 2, 2]);

    let arrangement = materialize_axis_aligned_orthogonal_solid_intersection(
        &fan_box,
        &cutter,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("face-fan triangulated orthogonal box should certify by exact cells");
    arrangement.validate().unwrap();
    arrangement
        .validate_against_sources(&fan_box, &cutter)
        .unwrap();
    assert_eq!(
        arrangement.freshness_against_sources(&fan_box, &cutter),
        AxisAlignedOrthogonalSolidFreshness::Current
    );
    assert!(arrangement.mesh.facts().mesh.closed_manifold);
}

#[test]
fn axis_aligned_orthogonal_solid_materializes_multiple_cavities() {
    let outer = axis_aligned_box([0, 0, 0], [5, 2, 2]);
    let left_cavity = axis_aligned_box([1, 1, 1], [2, 2, 2]);
    let right_cavity = axis_aligned_box([3, 1, 1], [4, 2, 2]);
    let cavities = combine_exact_meshes(
        &left_cavity,
        &right_cavity,
        "test disjoint orthogonal cavity cutters",
    );

    let arrangement = materialize_axis_aligned_orthogonal_solid_difference(
        &outer,
        &cavities,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("orthogonal cell difference should retain two disjoint cavities");
    arrangement.validate().unwrap();
    arrangement
        .validate_against_sources(&outer, &cavities)
        .unwrap();
    assert_eq!(
        arrangement.freshness_against_sources(&outer, &cavities),
        AxisAlignedOrthogonalSolidFreshness::Current
    );
    assert!(arrangement.mesh.facts().mesh.closed_manifold);
}

#[test]
fn affine_orthogonal_solid_recovers_face_fan_basis_from_cell_edges() {
    let fan_box = skew_affine_mesh_from_axis_aligned(
        &face_fan_box(),
        "test skew affine face-fan orthogonal box",
    );
    let cutter = skew_affine_box([1, 0, 0], [3, 2, 2]);

    let arrangement = materialize_affine_orthogonal_solid_intersection(
        &fan_box,
        &cutter,
        ValidationPolicy::CLOSED,
    )
    .unwrap()
    .expect("affine face-fan box should recover an exact cell-edge basis");
    arrangement.validate().unwrap();
    arrangement
        .validate_against_sources(&fan_box, &cutter)
        .unwrap();
    assert_eq!(
        arrangement.freshness_against_sources(&fan_box, &cutter),
        AffineOrthogonalSolidFreshness::Current
    );
    assert!(arrangement.mesh.facts().mesh.closed_manifold);
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
            hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex,
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
        let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &right)
            .unwrap();
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
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
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
    let preflight = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .preflight(&separated_left, &separated_right)
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
    let separated = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .materialize(&separated_left, &separated_right)
    .unwrap();
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
        stale_separated_output.validate().is_err(),
        "{stale_separated_output:?}"
    );
    assert_eq!(
        stale_separated_output.freshness_against_sources(&separated_left, &separated_right),
        ExactReportFreshness::StaleStatusEvidence,
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
    let separated_evaluation = (ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    ))
    .evaluate(&separated_left, &separated_right)
    .unwrap();
    separated_evaluation.validate().unwrap();
    let mut relabeled_convex_report = separated_evaluation.clone();
    relabeled_convex_report
        .certifications
        .convex_left_in_right
        .relation = ConvexSolidMeshRelation::NotCertifiedConvex;
    relabeled_convex_report
        .certifications
        .convex_left_in_right
        .solid_facts
        .orientation = ClosedMeshOrientation::NotClosed;
    relabeled_convex_report
        .certifications
        .convex_left_in_right
        .solid_facts
        .convexity = hypermesh::ConvexSolidClassification::NotClosed;
    relabeled_convex_report
        .certifications
        .convex_left_in_right
        .solid_facts
        .predicates
        .clear();
    relabeled_convex_report
        .certifications
        .convex_left_in_right
        .vertices
        .clear();
    relabeled_convex_report
        .certifications
        .convex_left_in_right
        .validate()
        .unwrap();
    assert_eq!(
        relabeled_convex_report.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
        "{relabeled_convex_report:?}"
    );
    let dispatched = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .materialize(&separated_left, &separated_right)
    .unwrap();
    assert_eq!(dispatched.kind, separated.kind);

    let contained_on_boundary = tetra_from_corners([1, 1, 0], [2, 1, 0], [1, 2, 0], [1, 1, 1]);
    let container = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let preflight = ExactBooleanRequest::new(
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .preflight(&contained_on_boundary, &container)
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
    let containment =
        ExactBooleanRequest::new(ExactBooleanOperation::Difference, ValidationPolicy::CLOSED)
            .materialize(&contained_on_boundary, &container)
            .unwrap();
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
        stale_containment_output.validate().is_err(),
        "{stale_containment_output:?}"
    );
    assert_eq!(
        stale_containment_output.freshness_against_sources(&contained_on_boundary, &container),
        ExactReportFreshness::StaleRegionFacts,
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

    let axis_overlap =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize(
                &axis_aligned_box([0, 0, 0], [2, 2, 2]),
                &axis_aligned_box([1, 1, 1], [3, 3, 3]),
            )
            .unwrap();
    assert_eq!(
        axis_overlap.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
        }
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
fn full_face_adjacent_union_accepts_interior_subdivided_shared_face() {
    let left = tetra_from_corners([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 2, 1, 0, 1, 2, 0, 0, 0, -6],
        &[
            0, 1, 3, //
            0, 3, 4, //
            1, 4, 3, //
            1, 2, 4, //
            2, 0, 4, //
            0, 5, 1, //
            1, 5, 2, //
            2, 5, 0,
        ],
    )
    .unwrap();

    let union = materialize_full_face_adjacent_union(&left, &right, ValidationPolicy::CLOSED)
        .expect("interior-subdivided shared face should certify as a retained patch");
    assert!(union.shared_faces.is_empty(), "{union:?}");
    assert_eq!(union.shared_patches.len(), 1, "{union:?}");
    assert_eq!(union.shared_patches[0].left_faces, vec![0]);
    assert_eq!(union.shared_patches[0].right_faces, vec![0, 1, 2, 3, 4]);
    assert!(union.mesh.facts().mesh.closed_manifold);
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.freshness_against_sources(&left, &right),
        FullFaceAdjacentUnionFreshness::Current
    );
}

#[test]
fn full_face_adjacent_union_refines_side_faces_for_boundary_subdivided_shared_face() {
    let left = tetra_from_corners([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, -6],
        &[
            0, 3, 2, //
            3, 1, 2, //
            0, 4, 3, //
            3, 4, 1, //
            1, 4, 2, //
            2, 4, 0,
        ],
    )
    .unwrap();

    let union = materialize_full_face_adjacent_union(&left, &right, ValidationPolicy::CLOSED)
        .expect("boundary-subdivided shared face should refine copied side faces");
    assert!(union.shared_faces.is_empty(), "{union:?}");
    assert_eq!(union.shared_patches.len(), 1, "{union:?}");
    assert_eq!(union.shared_patches[0].left_faces, vec![0]);
    assert_eq!(union.shared_patches[0].right_faces, vec![0, 1]);
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.freshness_against_sources(&left, &right),
        FullFaceAdjacentUnionFreshness::Current
    );
    assert!(union.mesh.facts().mesh.closed_manifold);

    let report = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
        .adjacent_union_completion_report(&left, &right)
        .unwrap();
    assert_eq!(
        report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace
    );
    assert_eq!(report.full_face_shared_faces, 0);
    assert_eq!(report.full_face_shared_patches, 1);
    assert!(!report.stronger_kernel_available);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        report.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
}

#[test]
fn full_face_adjacent_union_uses_polygon_patch_for_dual_subdivided_shared_face() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 2, 2, 0, 0, 0, 6],
        &[
            0, 3, 1, //
            1, 3, 2, //
            2, 3, 0, //
            0, 1, 4, //
            1, 2, 4, //
            2, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 1, 1, 0, 0, 0, -6],
        &[
            0, 1, 3, //
            1, 2, 3, //
            2, 0, 3, //
            0, 4, 1, //
            1, 4, 2, //
            2, 4, 0,
        ],
    )
    .unwrap();

    let union = materialize_full_face_adjacent_union(&left, &right, ValidationPolicy::CLOSED)
        .expect("opposite source-owned triangulated disks should certify as one patch");
    assert!(union.shared_faces.is_empty(), "{union:?}");
    assert_eq!(union.shared_patches.len(), 1, "{union:?}");
    assert_eq!(union.shared_patches[0].left_faces, vec![0, 1, 2]);
    assert_eq!(union.shared_patches[0].right_faces, vec![0, 1, 2]);
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.freshness_against_sources(&left, &right),
        FullFaceAdjacentUnionFreshness::Current
    );
    assert!(union.mesh.facts().mesh.closed_manifold);
}

#[test]
fn full_face_adjacent_union_accepts_dual_boundary_subdivided_shared_face() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 3, 0, 0, 0, 0, 6],
        &[
            0, 2, 3, //
            3, 2, 1, //
            0, 3, 4, //
            3, 1, 4, //
            1, 2, 4, //
            2, 0, 4,
        ],
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 6, 0, 0, 0, 6, 0, 0, 3, 0, 0, 0, -6],
        &[
            0, 1, 3, //
            3, 1, 2, //
            0, 4, 1, //
            1, 4, 2, //
            2, 4, 3, //
            3, 4, 0,
        ],
    )
    .unwrap();

    let union = materialize_full_face_adjacent_union(&left, &right, ValidationPolicy::CLOSED)
        .expect("opposite shared disks with different boundary splits should certify");
    assert!(union.shared_faces.is_empty(), "{union:?}");
    assert_eq!(union.shared_patches.len(), 1, "{union:?}");
    assert_eq!(union.shared_patches[0].left_faces, vec![0, 1]);
    assert_eq!(union.shared_patches[0].right_faces, vec![0, 1]);
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        union.freshness_against_sources(&left, &right),
        FullFaceAdjacentUnionFreshness::Current
    );
    assert!(union.mesh.facts().mesh.closed_manifold);

    let report = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
        .adjacent_union_completion_report(&left, &right)
        .unwrap();
    assert_eq!(
        report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedFullFace
    );
    assert_eq!(report.full_face_shared_faces, 0);
    assert_eq!(report.full_face_shared_patches, 1);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();
}

fn tetra_with_subdivided_base() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10, 5, 0, 0],
        &[
            0, 2, 4, //
            4, 2, 1, //
            0, 4, 3, //
            4, 1, 3, //
            1, 2, 3, //
            2, 0, 3,
        ],
    )
    .unwrap()
}

fn square_pyramid_with_base() -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, 5, 5, 10],
        &[
            0, 3, 2, //
            0, 2, 1, //
            0, 1, 4, //
            1, 2, 4, //
            2, 3, 4, //
            3, 0, 4,
        ],
    )
    .unwrap()
}

fn downward_square_pyramid_with_base(min: [i64; 2], max: [i64; 2], z: i64) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], 0, max[0], min[1], 0, max[0], max[1], 0, min[0], max[1], 0, min[0],
            min[1], z,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 4, 1, //
            1, 4, 2, //
            2, 4, 3, //
            3, 4, 0,
        ],
    )
    .unwrap()
}

#[test]
fn adjacent_union_completion_boolean_is_publicly_replayable() {
    let left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let left = combine_exact_meshes(&left_a, &left_b, "test disconnected full-face fixture");
    let right = tetra_from_corners([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, -4]);
    let separated_right = tetra_from_corners([20, 0, 0], [24, 0, 0], [20, 4, 0], [20, 0, 4]);

    let report = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
        .adjacent_union_completion_report(&left, &right)
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

    let (result, _completion_report) =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize_adjacent_union_completion(&left, &right)
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

    let (reported_result, consumed_report) =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize_adjacent_union_completion(&left, &right)
            .unwrap()
            .expect("non-axis full-face adjacent union should retain consumed report");
    assert_eq!(
        consumed_report, report,
        "full-face adjacent completion should return the certified report it consumed"
    );
    consumed_report.validate().unwrap();
    consumed_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(reported_result, result);
    reported_result.validate().unwrap();
    reported_result
        .validate_against_sources(&left, &right)
        .unwrap();

    assert_eq!(
        result.freshness_against_sources(&left, &right),
        ExactReportFreshness::Current
    );
    let mut stale_output = result.clone();
    stale_output.mesh = left.clone();
    assert!(stale_output.validate().is_err(), "{stale_output:?}");
    assert_eq!(
        stale_output.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleRegionFacts,
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
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .materialize_adjacent_union_completion(&left, &right)
        .unwrap()
        .is_none()
    );
    assert!(
        ExactBooleanRequest::new(
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
        )
        .materialize_adjacent_union_completion(&left, &right)
        .unwrap()
        .is_none()
    );
    let intersection_report = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .adjacent_union_completion_report(&left, &right)
    .unwrap();
    assert_eq!(
        intersection_report.status,
        ExactAdjacentUnionCompletionStatus::NotUnion
    );
    assert!(!intersection_report.is_certified());
    intersection_report.validate().unwrap();
    let mut stale_intersection_report = intersection_report.clone();
    stale_intersection_report.retained_face_pairs = 1;
    stale_intersection_report.retained_events = 1;
    stale_intersection_report.blocker.candidate_pairs = 1;
    assert!(stale_intersection_report.validate().is_err());

    let axis_left = axis_aligned_box([0, 0, 0], [1, 1, 1]);
    let axis_right = axis_aligned_box([1, 0, 0], [2, 1, 1]);
    let axis_report =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .adjacent_union_completion_report(&axis_left, &axis_right)
            .unwrap();
    assert_eq!(
        axis_report.status,
        ExactAdjacentUnionCompletionStatus::AxisAlignedBoxPair
    );
    axis_report.validate().unwrap();
    let mut stale_axis_report = axis_report.clone();
    stale_axis_report.retained_face_pairs = 1;
    stale_axis_report.retained_events = 1;
    stale_axis_report.blocker.candidate_pairs = 1;
    assert!(stale_axis_report.validate().is_err());

    assert!(
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED,)
            .materialize_adjacent_union_completion(&axis_left, &axis_right)
            .unwrap()
            .is_none()
    );
    assert!(
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED,)
            .materialize_adjacent_union_completion(&axis_left, &axis_right)
            .unwrap()
            .is_none()
    );
    let axis_replay =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize(&axis_left, &axis_right)
            .unwrap();
    assert_eq!(
        axis_replay.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
        }
    );
    axis_replay
        .validate_operation_against_sources(
            &axis_left,
            &axis_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let crossing_right = tetra_from_corners([1, 1, -1], [5, 1, -1], [1, 5, -1], [1, 1, 3]);
    let crossing_report =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .adjacent_union_completion_report(&left, &crossing_right)
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
    assert!(
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED,)
            .materialize_adjacent_union_completion(&left, &crossing_right)
            .unwrap()
            .is_none()
    );
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
            let closed_attempt = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
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

        let attempt = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
            .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
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
        let evaluation = (ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY))
            .evaluate(&left, &right)
            .unwrap();
        evaluation.validate().unwrap();
        let mut stale_preflight_counts = evaluation.clone();
        stale_preflight_counts.preflight.retained_events += 1;
        stale_preflight_counts.preflight.validate().unwrap();
        assert_eq!(
            stale_preflight_counts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {stale_preflight_counts:?}"
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
        if matches!(operation, ExactBooleanOperation::Intersection) {
            assert!(
                materialize_open_surface_arrangement(
                    &left,
                    &right,
                    operation,
                    ValidationPolicy::CLOSED,
                )
                .unwrap()
                .is_none(),
                "{operation:?} should yield to closed lower-dimensional provenance"
            );
            let replay = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, &right)
                .unwrap();
            assert_eq!(
                replay.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
                }
            );
            replay
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
        stale_materialization.validate().is_err(),
        "{stale_materialization:?}"
    );
    assert_eq!(
        stale_materialization.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence,
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
        let closed_attempt = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
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

        let boundary_attempt =
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
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
    let selection = ExactRegionSelection::KeepAll;
    let validation = ValidationPolicy::ALLOW_BOUNDARY;

    let result = ExactBooleanRequest::new(
        ExactBooleanOperation::SelectedRegions(selection),
        validation,
    )
    .materialize(&left, &right)
    .unwrap();

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
    let evaluation = (ExactBooleanRequest::new(
        ExactBooleanOperation::SelectedRegions(selection),
        validation,
    ))
    .evaluate(&left, &right)
    .unwrap();
    evaluation.validate().unwrap();
    let mut stale_evaluation_region_fact = evaluation.clone();
    let classification = stale_evaluation_region_fact
        .result
        .as_mut()
        .expect("selected-region evaluation should materialize")
        .region_classifications
        .first_mut()
        .expect("selected-region result should retain region facts");
    match classification.relation {
        FaceRegionPlaneRelation::StrictlyAbove => {
            classification.relation = FaceRegionPlaneRelation::StrictlyBelow;
            classification
                .node_sides
                .fill(Some(hyperlimit::PlaneSide::Below));
        }
        _ => {
            classification.relation = FaceRegionPlaneRelation::StrictlyAbove;
            classification
                .node_sides
                .fill(Some(hyperlimit::PlaneSide::Above));
        }
    }
    stale_evaluation_region_fact
        .result
        .as_ref()
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(
        stale_evaluation_region_fact.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
        "{stale_evaluation_region_fact:?}"
    );
    let mut stale_winding_handoff = evaluation.clone();
    stale_winding_handoff
        .certifications
        .winding_readiness
        .retained_events += 1;
    assert!(
        stale_winding_handoff.validate().is_err(),
        "{stale_winding_handoff:?}"
    );

    let mut stale_assembly_source_vertex = result.clone();
    let Some(hypermesh::FaceSplitBoundaryNode::OriginalVertex { vertex, .. }) =
        stale_assembly_source_vertex
            .assembly
            .vertices
            .iter_mut()
            .find_map(|output_vertex| match &mut output_vertex.source {
                source @ hypermesh::FaceSplitBoundaryNode::OriginalVertex { .. } => Some(source),
                hypermesh::FaceSplitBoundaryNode::GraphVertex { .. }
                | hypermesh::FaceSplitBoundaryNode::FaceInterior { .. } => None,
            })
    else {
        panic!("selected-region assembly should retain at least one original source vertex");
    };
    *vertex = usize::MAX;
    stale_assembly_source_vertex.validate().unwrap();
    assert!(
        stale_assembly_source_vertex
            .validate_against_sources(&left, &right)
            .is_err()
    );
    assert_eq!(
        stale_assembly_source_vertex.freshness_against_sources(&left, &right),
        ExactReportFreshness::SourceReplayMismatch
    );

    let mut stale_kind = result.clone();
    stale_kind.kind = ExactBooleanResultKind::SelectedRegions {
        selection: ExactRegionSelection::KeepLeft,
    };
    assert!(stale_kind.validate_against_sources(&left, &right).is_err());

    let keep_left = ExactBooleanRequest::new(
        ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepLeft),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .materialize(&left, &right)
    .unwrap();
    let mut stale_materialization = result.clone();
    stale_materialization.assembly = keep_left.assembly;
    stale_materialization.mesh = keep_left.mesh;
    assert!(
        stale_materialization.validate().is_err(),
        "{stale_materialization:?}"
    );
    assert_eq!(
        stale_materialization.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence,
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
        assert!(
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts
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
        let preflight = hypermesh::ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .preflight(&left, &right)
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

        let readiness = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .winding_readiness(&left, &right)
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

        let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &right)
            .unwrap();
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

        let disjoint_preflight =
            hypermesh::ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .preflight(&left, &disjoint_right)
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
        let disjoint_readiness = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .winding_readiness(&left, &disjoint_right)
            .unwrap();
        assert_eq!(
            disjoint_readiness.status,
            ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
            "{operation:?}: {disjoint_readiness:?}"
        );
        let disjoint_result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&left, &disjoint_right)
            .unwrap();
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
            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(left, right)
                .unwrap();
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
    let lower_result =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize(&lower_left, &lower_right)
            .unwrap();
    assert_eq!(
        lower_result.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
        }
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
            let preflight =
                hypermesh::ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                    .preflight(left, right)
                    .unwrap();
            assert_eq!(
                preflight.support,
                hypermesh::ExactBooleanSupport::CertifiedMixedDimensionalRegularizedSolid,
                "{operation:?}: {preflight:?}"
            );
            let readiness = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .winding_readiness(left, right)
                .unwrap();
            assert_eq!(
                readiness.status,
                ExactWindingReadinessStatus::MixedDimensionalRegularizedSolidAlreadyMaterialized,
                "{operation:?}: {readiness:?}"
            );
            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(left, right)
                .unwrap();
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

            let boundary_preflight =
                ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                    .preflight(left, right)
                    .unwrap();
            assert_eq!(
                boundary_preflight.support,
                hypermesh::ExactBooleanSupport::CertifiedBoundsDisjoint,
                "{operation:?}: {boundary_preflight:?}"
            );
            let boundary_result =
                ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                    .materialize(left, right)
                    .unwrap();
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
            ExactBooleanRequest::with_boundary_policy(
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .materialize_boundary_touching_policy(&left, &right)
            .unwrap()
            .is_none()
        );

        let result = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .materialize_boundary_touching_policy(&left, &right)
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

    let closed_left_a = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let closed_left_b = tetra_from_corners([10, 0, 0], [12, 0, 0], [10, 2, 0], [10, 0, 2]);
    let closed_left = combine_exact_meshes(
        &closed_left_a,
        &closed_left_b,
        "test boundary policy closed replay left",
    );
    let closed_right = tetra_from_corners([0, 0, 0], [-4, 0, 0], [0, -4, 0], [0, 0, -4]);
    for (operation, shortcut) in [
        (
            ExactBooleanOperation::Union,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion,
        ),
        (
            ExactBooleanOperation::Intersection,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingIntersection,
        ),
        (
            ExactBooleanOperation::Difference,
            hypermesh::ExactBooleanShortcutKind::ClosedBoundaryTouchingDifference,
        ),
    ] {
        let direct = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .materialize_boundary_touching_policy(&closed_left, &closed_right)
        .unwrap()
        .expect("closed boundary-touching regularization should materialize directly");
        assert_eq!(
            direct.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut
            },
            "{operation:?}: {direct:?}"
        );
        direct
            .validate_operation_against_sources(
                &closed_left,
                &closed_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
        let replay = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .materialize(&closed_left, &closed_right)
        .unwrap();
        assert_eq!(
            replay.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut
            }
        );
        replay
            .validate_operation_against_sources(
                &closed_left,
                &closed_right,
                operation,
                ValidationPolicy::CLOSED,
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

    let evidence = certify_coplanar_volumetric_cell_evidence(&left, &right).unwrap();
    evidence.validate().unwrap();
    evidence.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        evidence.obstacle,
        hypermesh::CoplanarVolumetricCellObstacle::BoundaryOnlyContact
    );
    assert_eq!(evidence.positive_area_coplanar_overlapping_pairs, 0);

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
        let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
            .preflight(&left, &right)
            .unwrap();
        assert_eq!(preflight.support, support, "{operation:?}: {preflight:?}");
        assert!(
            preflight.retained_face_pairs > 0,
            "closed boundary-touching shortcut should retain graph evidence: {operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        let (result, _consumed_evidence) =
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize_closed_boundary_touching_regularized_with_evidence(&left, &right)
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
        let evaluation = (ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
            .evaluate(&left, &right)
            .unwrap();
        evaluation.validate().unwrap();
        let mut relabeled_boundary_report = evaluation.clone();
        relabeled_boundary_report
            .certifications
            .boundary_touching
            .status = ExactBoundaryTouchingStatus::NotBoundaryOnly;
        assert!(
            relabeled_boundary_report.validate().is_err(),
            "{operation:?}: {relabeled_boundary_report:?}"
        );

        let (evidenced_result, consumed_evidence) =
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize_closed_boundary_touching_regularized_with_evidence(&left, &right)
                .unwrap()
                .expect("closed zero-area boundary contact should retain consumed evidence");
        assert_eq!(
            consumed_evidence, evidence,
            "{operation:?}: consumed evidence should match certified zero-area boundary report"
        );
        consumed_evidence.validate().unwrap();
        consumed_evidence
            .validate_against_sources(&left, &right)
            .unwrap();
        assert_eq!(
            evidenced_result.kind, result.kind,
            "{operation:?}: {evidenced_result:?}"
        );
        assert_eq!(
            evidenced_result.mesh.vertices().len(),
            result.mesh.vertices().len(),
            "{operation:?}: {evidenced_result:?}"
        );
        assert_eq!(
            evidenced_result.mesh.triangles().len(),
            result.mesh.triangles().len(),
            "{operation:?}: {evidenced_result:?}"
        );
        evidenced_result.validate().unwrap();
        evidenced_result
            .validate_against_sources(&left, &right)
            .unwrap();

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
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::StaleStatusEvidence,
                "{stale_output:?}"
            );
        }
        if operation == ExactBooleanOperation::Difference {
            let mut stale_output = result.clone();
            stale_output.mesh = right.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::StaleRegionFacts,
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
        let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
            .preflight(&left, &right)
            .unwrap();
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
            let readiness = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .winding_readiness(&left, &right)
                .unwrap();
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
            let evaluation = (ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
                .evaluate(&left, &right)
                .unwrap();
            evaluation.validate().unwrap();
            let mut cleared_handoff_evidence = evaluation.clone();
            cleared_handoff_evidence
                .certifications
                .winding_readiness
                .coplanar_volumetric_evidence = None;
            assert!(
                cleared_handoff_evidence.validate().is_err(),
                "{operation:?}: {cleared_handoff_evidence:?}"
            );
        }

        assert!(
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED,)
                .materialize_closed_boundary_touching_regularized_with_evidence(&left, &right)
                .unwrap()
                .is_none()
        );
        let (result, _consumed_evidence) =
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED,).materialize_closed_no_volume_overlap_regularized_with_evidence(&left, &right)
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
        let (evidenced_result, consumed_evidence) =
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize_closed_no_volume_overlap_regularized_with_evidence(&left, &right)
                .unwrap()
                .expect("positive-area no-volume materializer should retain consumed evidence");
        assert_eq!(
            consumed_evidence, evidence,
            "{operation:?}: consumed evidence should match certified no-volume report"
        );
        consumed_evidence.validate().unwrap();
        assert_eq!(
            evidenced_result.kind, result.kind,
            "{operation:?}: {evidenced_result:?}"
        );
        assert_eq!(
            evidenced_result.mesh.vertices().len(),
            result.mesh.vertices().len(),
            "{operation:?}: {evidenced_result:?}"
        );
        assert_eq!(
            evidenced_result.mesh.triangles().len(),
            result.mesh.triangles().len(),
            "{operation:?}: {evidenced_result:?}"
        );
        evidenced_result.validate().unwrap();
        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current
        );
        if operation == ExactBooleanOperation::Union {
            let mut stale_output = result.clone();
            stale_output.mesh = left.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, &right),
                ExactReportFreshness::StaleRegionFacts,
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
        let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize_closed_winding_separated(&separated_left, &separated_right)
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
        let separated_evaluation = (ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
            .evaluate(&separated_left, &separated_right)
            .unwrap();
        separated_evaluation.validate().unwrap();
        let mut relabeled_winding_report = separated_evaluation.clone();
        relabeled_winding_report
            .certifications
            .closed_winding_left_in_right
            .relation = hypermesh::ClosedMeshWindingMeshRelation::NotClosed;
        relabeled_winding_report
            .certifications
            .closed_winding_left_in_right
            .target_closed = false;
        relabeled_winding_report
            .certifications
            .closed_winding_left_in_right
            .vertices
            .clear();
        relabeled_winding_report
            .certifications
            .closed_winding_left_in_right
            .validate()
            .unwrap();
        assert_eq!(
            relabeled_winding_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_winding_report:?}"
        );
        assert_eq!(
            result.freshness_against_sources(&separated_left, &intersecting_right),
            ExactReportFreshness::SourceReplayMismatch
        );
        if operation == ExactBooleanOperation::Intersection {
            let mut stale_output = result.clone();
            stale_output.mesh = separated_left.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&separated_left, &separated_right),
                ExactReportFreshness::StaleStatusEvidence,
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
        let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize_closed_winding_containment(&container, &contained)
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
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&container, &contained),
                ExactReportFreshness::StaleRegionFacts,
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
fn closed_winding_materializers_yield_to_earlier_public_convex_replay() {
    let separated_left = tetra_from_corners([0, 0, 0], [2, 0, 0], [0, 2, 0], [0, 0, 2]);
    let separated_right = tetra_from_corners([1, 1, 1], [3, 1, 1], [1, 3, 1], [1, 1, 3]);
    let container = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let contained = tetra_from_corners([1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]);

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        assert!(
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED,)
                .materialize_closed_winding_separated(&separated_left, &separated_right)
                .unwrap()
                .is_none(),
            "{operation:?} should yield to convex-separated public provenance"
        );
        let separated_replay = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&separated_left, &separated_right)
            .unwrap();
        assert_convex_public_replay(&separated_replay, operation);
        separated_replay
            .validate_operation_against_sources(
                &separated_left,
                &separated_right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();

        assert!(
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED,)
                .materialize_closed_winding_containment(&container, &contained)
                .unwrap()
                .is_none(),
            "{operation:?} should yield to convex-containment public provenance"
        );
        let containment_replay = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&container, &contained)
            .unwrap();
        assert_convex_public_replay(&containment_replay, operation);
        containment_replay
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

fn assert_convex_public_replay(result: &ExactBooleanResult, operation: ExactBooleanOperation) {
    let ExactBooleanResultKind::CertifiedShortcut {
        operation: replay_operation,
        shortcut,
    } = &result.kind
    else {
        panic!("{operation:?}: expected certified convex shortcut, got {result:?}");
    };
    assert_eq!(*replay_operation, operation, "{result:?}");
    assert!(
        matches!(
            shortcut,
            hypermesh::ExactBooleanShortcutKind::ConvexUnion
                | hypermesh::ExactBooleanShortcutKind::ConvexIntersection
                | hypermesh::ExactBooleanShortcutKind::ConvexDifference
                | hypermesh::ExactBooleanShortcutKind::ConvexSeparated
                | hypermesh::ExactBooleanShortcutKind::ConvexContainment
        ),
        "{operation:?}: expected convex public replay, got {result:?}"
    );
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

    let preflight = hypermesh::ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .preflight(&left, &right)
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

    let readiness = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .winding_readiness(&left, &right)
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

    let closed_attempt =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
            .unwrap();
    assert_eq!(closed_attempt.output_validation, ValidationPolicy::CLOSED);
    assert_eq!(
        closed_attempt.decline,
        Some(hypermesh::ExactArrangementBooleanDecline::OutputValidation),
        "{closed_attempt:?}"
    );
    assert_eq!(
        closed_attempt.topology_assembly,
        Some(ExactTopologyAssemblyStatus::Complete),
        "{closed_attempt:?}"
    );
    assert_eq!(
        closed_attempt
            .topology_assembly_report
            .as_ref()
            .map(|report| report.status),
        closed_attempt.topology_assembly,
        "{closed_attempt:?}"
    );
    assert_eq!(
        closed_attempt.region_ownership,
        Some(ExactRegionOwnershipStatus::FaceResolved),
        "{closed_attempt:?}"
    );
    assert_eq!(
        closed_attempt
            .region_ownership_report
            .as_ref()
            .map(|report| report.status),
        closed_attempt.region_ownership,
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
    let evaluation = (ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    ))
    .evaluate(&left, &right)
    .unwrap();
    evaluation.validate().unwrap();
    assert_eq!(
        evaluation.preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedArrangementCellComplex,
        "{evaluation:?}"
    );
    let mut unresolved_ownership = evaluation.clone();
    let ownership = unresolved_ownership
        .certifications
        .region_ownership
        .as_mut()
        .expect("named arrangement evaluation should retain ownership evidence");
    ownership.status = ExactRegionOwnershipStatus::RequiresWinding;
    ownership.blockers = vec![ExactArrangementBlocker::UnresolvedRegionClassification];
    ownership.volume_regions = 0;
    ownership.exterior_volume_regions = 0;
    ownership.left_owned_volumes = 0;
    ownership.right_owned_volumes = 0;
    ownership.shared_owned_volumes = 0;
    ownership.unowned_bounded_volumes = 0;
    ownership.volume_adjacencies = 0;
    ownership.volume_adjacency_face_sides = 0;
    ownership.volume_adjacency_separating_faces = 0;
    ownership.validate().unwrap();
    assert_eq!(
        unresolved_ownership.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut incomplete_topology = evaluation.clone();
    let topology = incomplete_topology
        .certifications
        .topology_assembly
        .as_mut()
        .expect("named arrangement evaluation should retain topology assembly evidence");
    topology.status = ExactTopologyAssemblyStatus::MissingRegionPlan;
    topology.region_boundaries = 0;
    topology.region_boundary_nodes = 0;
    topology.validate().unwrap();
    assert_eq!(
        incomplete_topology.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut declined_arrangement_attempt = evaluation.clone();
    let attempt = declined_arrangement_attempt
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt");
    attempt.stage = hypermesh::ExactArrangementBooleanStage::Triangulated;
    attempt.decline = Some(hypermesh::ExactArrangementBooleanDecline::OutputValidation);
    attempt.materialized_shortcut = None;
    attempt.validate().unwrap();
    assert_eq!(
        declined_arrangement_attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_attempt_gate = evaluation.clone();
    let attempt = stale_attempt_gate
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt");
    attempt.region_ownership = Some(ExactRegionOwnershipStatus::RequiresWinding);
    assert_eq!(
        attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    assert_eq!(
        stale_attempt_gate.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_attempt_report = evaluation.clone();
    let attempt = stale_attempt_report
        .certifications
        .arrangement_attempt
        .as_mut()
        .expect("named evaluation should retain arrangement attempt");
    attempt
        .topology_assembly_report
        .as_mut()
        .expect("named attempt should retain topology report")
        .status = ExactTopologyAssemblyStatus::MissingRegionPlan;
    assert_eq!(
        attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    assert_eq!(
        stale_attempt_report.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );
    let mut stale_readiness_counts = evaluation.clone();
    stale_readiness_counts
        .certifications
        .winding_readiness
        .retained_face_pairs += 1;
    stale_readiness_counts
        .certifications
        .winding_readiness
        .retained_events += 1;
    assert!(
        stale_readiness_counts.validate().is_err(),
        "{stale_readiness_counts:?}"
    );
    assert!(!result.region_classifications.is_empty());
    assert!(!result.triangulations.is_empty());
    assert!(!result.volumetric_classifications.is_empty());
    assert!(!result.assembly.triangles.is_empty());
    assert!(!result.mesh.triangles().is_empty());
    let mut relabeled_operation = result.clone();
    relabeled_operation.kind = ExactBooleanResultKind::ArrangementCellComplexMaterialized {
        operation: ExactBooleanOperation::Intersection,
    };
    assert_eq!(
        relabeled_operation.validate(),
        Err(hypermesh::ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation),
        "{relabeled_operation:?}"
    );
    if result.volumetric_classifications.len() > 1 {
        let mut stale_volumetric_order = result.clone();
        stale_volumetric_order.volumetric_classifications.swap(0, 1);
        assert!(
            stale_volumetric_order.validate().is_err(),
            "{stale_volumetric_order:?}"
        );
        assert_eq!(
            stale_volumetric_order.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
            "{stale_volumetric_order:?}"
        );
    }

    let difference = materialize_volumetric_winding_arrangement(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .expect("overlapping closed solids should materialize exact volumetric difference");
    difference.validate().unwrap();
    let Some(reversed_triangle) =
        difference.assembly.triangles.iter().position(|triangle| {
            triangle.orientation == ExactOutputTriangleOrientation::ReverseSource
        })
    else {
        panic!("volumetric difference should retain a reversed source triangle");
    };
    let mut stale_difference_orientation = difference.clone();
    stale_difference_orientation.assembly.triangles[reversed_triangle].orientation =
        ExactOutputTriangleOrientation::PreserveSource;
    assert_eq!(
        stale_difference_orientation.validate(),
        Err(hypermesh::ExactReportValidationError::VolumetricMaterializedAssemblyViolatesOperation),
        "{stale_difference_orientation:?}"
    );
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
        let closure = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
            .volumetric_boundary_closure(&left, &right)
            .unwrap();
        assert_eq!(
            closure.status,
            hypermesh::ExactVolumetricBoundaryClosureStatus::CoplanarClosureAvailable,
            "{operation:?}: {closure:?}"
        );
        closure.validate().unwrap();
        closure.validate_against_sources(&left, &right).unwrap();

        let preflight = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
            .preflight(&left, &right)
            .unwrap();
        assert_eq!(
            preflight.support,
            hypermesh::ExactBooleanSupport::CertifiedArrangementCellComplex,
            "{operation:?}: {preflight:?}"
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();

        let readiness = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
            .winding_readiness(&left, &right)
            .unwrap();
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
            result.topology_assembly_report.is_some(),
            "{operation:?}: {result:?}"
        );
        assert!(
            result.region_ownership_report.is_some(),
            "{operation:?}: {result:?}"
        );
        assert!(
            result.mesh.facts().mesh.closed_manifold || result.mesh.triangles().is_empty(),
            "{operation:?}: {:?}",
            result.mesh.facts().mesh
        );
        result.validate_against_sources(&left, &right).unwrap();

        let (cap_result, cap_report) = materialize_volumetric_coplanar_boundary_closure_boolean(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .unwrap()
        .expect("public coplanar cap materializer should retain closure provenance");
        assert_eq!(cap_report, closure, "{operation:?}: {cap_report:?}");
        cap_report.validate().unwrap();
        assert_eq!(
            cap_result.kind, result.kind,
            "{operation:?}: {cap_result:?}"
        );
        assert_eq!(
            cap_result.mesh.vertices().len(),
            result.mesh.vertices().len(),
            "{operation:?}: {cap_result:?}"
        );
        assert_eq!(
            cap_result.mesh.triangles().len(),
            result.mesh.triangles().len(),
            "{operation:?}: {cap_result:?}"
        );
        cap_result.validate().unwrap();
        assert_eq!(
            cap_result.topology_assembly_report, result.topology_assembly_report,
            "{operation:?}: {cap_result:?}"
        );
        assert_eq!(
            cap_result.region_ownership_report, result.region_ownership_report,
            "{operation:?}: {cap_result:?}"
        );

        assert_eq!(
            result.freshness_against_sources(&left, &right),
            ExactReportFreshness::Current,
            "{operation:?}: {result:?}"
        );
        let mut stale_output = result.clone();
        stale_output.mesh = left.clone();
        assert!(
            stale_output.validate().is_err(),
            "{operation:?}: {stale_output:?}"
        );
        assert_eq!(
            stale_output.freshness_against_sources(&left, &right),
            ExactReportFreshness::StaleRegionFacts,
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
    let replay = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .materialize(&left, &right)
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
    let convex_intersection = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .materialize(&convex_left, &convex_right)
    .unwrap();
    assert_eq!(
        convex_intersection.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Intersection,
            shortcut: hypermesh::ExactBooleanShortcutKind::ConvexIntersection
        }
    );
}

#[test]
fn exact_contained_face_adjacent_union_is_publicly_replayable() {
    let left = tetra_from_corners([0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]);
    let right = tetra_from_corners([1, 1, 0], [1, 3, 0], [3, 1, 0], [1, 1, -2]);
    let separated_right = tetra([20, 0, 0]);
    let subdivided_left = tetra_with_subdivided_base();
    let split_crossing_right = tetra_from_corners([1, 1, 0], [1, 4, 0], [4, 3, 0], [1, 1, -2]);
    let square_base_left = square_pyramid_with_base();
    let same_orientation_square_cap =
        tetra_from_corners([2, 2, 0], [6, 2, 0], [2, 6, 0], [2, 2, -2]);
    let square_cap_right = tetra_from_corners([2, 2, 0], [2, 6, 0], [6, 2, 0], [2, 2, -2]);
    let square_disk_cap_right = downward_square_pyramid_with_base([2, 2], [6, 6], -2);
    let two_caps_right = combine_exact_meshes(
        &tetra_from_corners([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -2]),
        &tetra_from_corners([4, 1, 0], [4, 2, 0], [5, 1, 0], [4, 1, -2]),
        "test two contained caps",
    );

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

    let split_union = materialize_contained_face_adjacent_union(
        &subdivided_left,
        &split_crossing_right,
        ValidationPolicy::CLOSED,
    )
    .expect("contained cap crossing a source subdivision should materialize");
    split_union.validate().unwrap();
    split_union
        .validate_against_sources(&subdivided_left, &split_crossing_right)
        .unwrap();
    assert_eq!(split_union.containing_faces.len(), 2);
    assert_eq!(split_union.contained_faces.len(), 1);
    assert!(split_union.mesh.facts().mesh.closed_manifold);
    let square_union = materialize_contained_face_adjacent_union(
        &square_base_left,
        &square_cap_right,
        ValidationPolicy::CLOSED,
    )
    .expect("contained cap inside a non-triangular source patch should materialize");
    square_union.validate().unwrap();
    square_union
        .validate_against_sources(&square_base_left, &square_cap_right)
        .unwrap();
    assert_eq!(square_union.containing_faces.len(), 2);
    assert_eq!(square_union.contained_faces.len(), 1);
    assert!(square_union.mesh.facts().mesh.closed_manifold);
    assert!(
        materialize_contained_face_adjacent_union(
            &square_base_left,
            &same_orientation_square_cap,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );
    let square_disk_union = materialize_contained_face_adjacent_union(
        &square_base_left,
        &square_disk_cap_right,
        ValidationPolicy::CLOSED,
    )
    .expect("multi-face contained cap inside a non-triangular patch should materialize");
    square_disk_union.validate().unwrap();
    square_disk_union
        .validate_against_sources(&square_base_left, &square_disk_cap_right)
        .unwrap();
    assert_eq!(square_disk_union.containing_faces.len(), 2);
    assert_eq!(square_disk_union.contained_faces.len(), 2);
    assert!(square_disk_union.mesh.facts().mesh.closed_manifold);
    let multi_hole_union =
        materialize_contained_face_adjacent_union(&left, &two_caps_right, ValidationPolicy::CLOSED)
            .expect("two contained caps on one source face should materialize");
    multi_hole_union.validate().unwrap();
    multi_hole_union
        .validate_against_sources(&left, &two_caps_right)
        .unwrap();
    assert_eq!(multi_hole_union.containing_faces.len(), 1);
    assert_eq!(multi_hole_union.contained_faces.len(), 2);
    assert!(multi_hole_union.mesh.facts().mesh.closed_manifold);
    let multi_hole_report =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .adjacent_union_completion_report(&left, &two_caps_right)
            .unwrap();
    assert_eq!(
        multi_hole_report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
    );
    assert_eq!(multi_hole_report.containing_faces, 1);
    assert_eq!(multi_hole_report.contained_faces, 2);
    multi_hole_report.validate().unwrap();
    multi_hole_report
        .validate_against_sources(&left, &two_caps_right)
        .unwrap();

    let mut missing_contained = union.clone();
    missing_contained.contained_faces.clear();
    assert_eq!(
        missing_contained.validate(),
        Err(ContainedFaceAdjacentUnionError::InvalidCertificate)
    );
    assert_eq!(
        missing_contained.freshness_against_sources(&left, &right),
        ContainedFaceAdjacentUnionFreshness::InvalidCertificate
    );

    let mut relabeled_containing = union.clone();
    relabeled_containing.containing_face = usize::MAX;
    assert_eq!(
        relabeled_containing.validate(),
        Err(ContainedFaceAdjacentUnionError::InvalidCertificate)
    );

    let mut duplicate_containing = union.clone();
    duplicate_containing
        .containing_faces
        .push(duplicate_containing.containing_faces[0]);
    assert_eq!(
        duplicate_containing.validate(),
        Err(ContainedFaceAdjacentUnionError::InvalidCertificate)
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
        ContainedFaceAdjacentUnionFreshness::InvalidOutput
    );

    assert_eq!(
        union.freshness_against_sources(&left, &separated_right),
        ContainedFaceAdjacentUnionFreshness::SourceReplayMismatch
    );

    assert!(
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED,)
            .materialize_adjacent_union_completion(&left, &right)
            .unwrap()
            .is_none()
    );

    let disjoint_shell = tetra_from_corners([40, 0, 0], [41, 0, 0], [40, 1, 0], [40, 0, 1]);
    let split_container = combine_exact_meshes(
        &subdivided_left,
        &disjoint_shell,
        "test disconnected subdivided contained-face fixture",
    );
    let split_report =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .adjacent_union_completion_report(&split_container, &split_crossing_right)
            .unwrap();
    assert_eq!(
        split_report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
    );
    assert_eq!(split_report.containing_faces, 2);
    assert_eq!(split_report.contained_faces, 1);
    split_report.validate().unwrap();
    split_report
        .validate_against_sources(&split_container, &split_crossing_right)
        .unwrap();

    let square_disk_container = combine_exact_meshes(
        &square_base_left,
        &disjoint_shell,
        "test disconnected multi-face contained-cap fixture",
    );
    let square_disk_report =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .adjacent_union_completion_report(&square_disk_container, &square_disk_cap_right)
            .unwrap();
    assert_eq!(
        square_disk_report.status,
        ExactAdjacentUnionCompletionStatus::CertifiedContainedFace
    );
    assert_eq!(square_disk_report.containing_faces, 2);
    assert_eq!(square_disk_report.contained_faces, 2);
    square_disk_report.validate().unwrap();
    square_disk_report
        .validate_against_sources(&square_disk_container, &square_disk_cap_right)
        .unwrap();

    let container = combine_exact_meshes(
        &left,
        &disjoint_shell,
        "test disconnected contained-face fixture",
    );
    let completion_report =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .adjacent_union_completion_report(&container, &right)
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
    let (result, _completion_report) =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize_adjacent_union_completion(&container, &right)
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

    let (reported_result, consumed_report) =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize_adjacent_union_completion(&container, &right)
            .unwrap()
            .expect("contained-face adjacent union should retain consumed report");
    assert_eq!(
        consumed_report, completion_report,
        "contained-face adjacent completion should return the certified report it consumed"
    );
    consumed_report.validate().unwrap();
    consumed_report
        .validate_against_sources(&container, &right)
        .unwrap();
    assert_eq!(reported_result, result);
    reported_result.validate().unwrap();
    reported_result
        .validate_against_sources(&container, &right)
        .unwrap();

    assert_eq!(
        result.freshness_against_sources(&container, &right),
        ExactReportFreshness::Current
    );
    let mut stale_output = result.clone();
    stale_output.mesh = container.clone();
    assert!(stale_output.validate().is_err(), "{stale_output:?}");
    assert_eq!(
        stale_output.freshness_against_sources(&container, &right),
        ExactReportFreshness::StaleRegionFacts,
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

    let refinement = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .refinement_report(&left, &overlapping_right)
    .unwrap();
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

    let planar = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .planar_arrangement_report(&left, &overlapping_right)
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

    let same_surface = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .same_surface_report(&left, &left);
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
    let open_disjoint = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .open_surface_disjoint_report(&left, &parallel_right)
    .unwrap();
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

    let report = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .open_surface_disjoint_report(&left, &right)
    .unwrap();

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

    let report = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .planar_arrangement_report(&left, &right)
    .unwrap();

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
    let mut noncanonical_chain_geometry = geometry.clone();
    noncanonical_chain_geometry.faces[0].boundary_chains[0]
        .nodes
        .rotate_left(1);
    let noncanonical_chain_report =
        noncanonical_chain_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        noncanonical_chain_report
            .diagnostics
            .iter()
            .any(|diagnostic| {
                diagnostic.kind == hypermesh::SplitPlanDiagnosticKind::WrongChainStart
            }),
        "{noncanonical_chain_report:?}"
    );
    assert_eq!(
        noncanonical_chain_geometry.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );
    let mut duplicate_chain_geometry = geometry.clone();
    let duplicate_chain = duplicate_chain_geometry.faces[0].boundary_chains[0].clone();
    duplicate_chain_geometry.faces[0]
        .boundary_chains
        .push(duplicate_chain);
    let duplicate_chain_report =
        duplicate_chain_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        duplicate_chain_report.diagnostics.iter().any(|diagnostic| {
            diagnostic.kind == hypermesh::SplitPlanDiagnosticKind::DuplicateFaceSplitEdge
        }),
        "{duplicate_chain_report:?}"
    );
    assert_eq!(
        duplicate_chain_geometry.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );
    let mut stale_original_point_geometry = geometry.clone();
    if let hypermesh::FaceSplitBoundaryNode::OriginalVertex { point, .. } =
        &mut stale_original_point_geometry.faces[0].boundary_chains[0].nodes[0]
    {
        *point = p(2, 0, 0);
    }
    let stale_original_point_report =
        stale_original_point_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        stale_original_point_report
            .diagnostics
            .iter()
            .any(|diagnostic| {
                diagnostic.kind
                    == hypermesh::SplitPlanDiagnosticKind::BoundaryNodeSourcePointMismatch
            }),
        "{stale_original_point_report:?}"
    );
    assert_eq!(
        stale_original_point_geometry.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );
    let mut relabeled_geometry = geometry.clone();
    relabeled_geometry.faces[0].triangle.swap(0, 1);
    let geometry_report = relabeled_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        geometry_report.diagnostics.iter().any(|diagnostic| {
            diagnostic.kind == hypermesh::SplitPlanDiagnosticKind::SourceTriangleMismatch
        }),
        "{geometry_report:?}"
    );
    assert_eq!(
        relabeled_geometry.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );
    let regions = geometry.region_plan(&left, &right);
    assert_eq!(
        regions.freshness_against_sources(&left, &right),
        SplitPlanFreshness::Current
    );
    let mut closed_duplicate_regions = regions.clone();
    let first_region_node = closed_duplicate_regions.regions[0].boundary[0].clone();
    closed_duplicate_regions.regions[0]
        .boundary
        .push(first_region_node);
    let closed_duplicate_report = closed_duplicate_regions.validate(&left, &right);
    assert!(
        closed_duplicate_report
            .diagnostics
            .iter()
            .any(|diagnostic| {
                diagnostic.kind
                    == hypermesh::SplitPlanDiagnosticKind::DuplicateConsecutiveRegionNode
            }),
        "{closed_duplicate_report:?}"
    );
    assert_eq!(
        closed_duplicate_regions.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );
    let mut stale_region_point = regions.clone();
    if let hypermesh::FaceSplitBoundaryNode::OriginalVertex { point, .. } =
        &mut stale_region_point.regions[0].boundary[0]
    {
        *point = p(2, 0, 0);
    }
    let stale_region_point_report = stale_region_point.validate(&left, &right);
    assert!(
        stale_region_point_report
            .diagnostics
            .iter()
            .any(|diagnostic| {
                diagnostic.kind
                    == hypermesh::SplitPlanDiagnosticKind::BoundaryNodeSourcePointMismatch
            }),
        "{stale_region_point_report:?}"
    );
    assert_eq!(
        stale_region_point.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );
    let mut missing_region_vertex = regions.clone();
    if let hypermesh::FaceSplitBoundaryNode::OriginalVertex { vertex, .. } =
        &mut missing_region_vertex.regions[0].boundary[0]
    {
        *vertex = usize::MAX;
    }
    let missing_region_vertex_report = missing_region_vertex.validate(&left, &right);
    assert!(
        missing_region_vertex_report
            .diagnostics
            .iter()
            .any(|diagnostic| {
                diagnostic.kind
                    == hypermesh::SplitPlanDiagnosticKind::BoundaryNodeSourceVertexOutOfRange
            }),
        "{missing_region_vertex_report:?}"
    );
    assert_eq!(
        missing_region_vertex.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );
    let mut relabeled_regions = regions.clone();
    relabeled_regions.regions[0].triangle.swap(0, 1);
    let region_report = relabeled_regions.validate(&left, &right);
    assert!(
        region_report.diagnostics.iter().any(|diagnostic| {
            diagnostic.kind == hypermesh::SplitPlanDiagnosticKind::SourceTriangleMismatch
        }),
        "{region_report:?}"
    );
    assert_eq!(
        relabeled_regions.freshness_against_sources(&left, &right),
        SplitPlanFreshness::InvalidPlan
    );

    let mut truncated = pair.clone();
    truncated.triangle.as_mut().unwrap().right_edge_events.pop();
    assert_eq!(
        truncated.validate(),
        Err(MeshFacePairValidationError::CandidateMissingEdgeEvents)
    );
}

#[test]
fn exact_face_pair_plane_separation_retains_triangle_evidence() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 0, 1, 0, 0, 0, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 1, 0, 1, 0, 1, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let pair = classify_mesh_face_pair(&left, 0, &right, 0).unwrap();
    assert_eq!(pair.relation, MeshFacePairRelation::PlaneSeparated);
    let triangle = pair.triangle.as_ref().unwrap();
    assert!(matches!(
        triangle.relation,
        TriangleTriangleRelation::SeparatedByFirstPlane
            | TriangleTriangleRelation::SeparatedBySecondPlane
    ));
    pair.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        pair.freshness_against_sources(&left, &right),
        MeshFacePairFreshness::Current
    );

    let mut missing_triangle = pair.clone();
    missing_triangle.triangle = None;
    assert_eq!(
        missing_triangle.validate(),
        Err(MeshFacePairValidationError::MissingTriangleClassification)
    );

    let mut invalid_triangle = pair;
    invalid_triangle
        .triangle
        .as_mut()
        .unwrap()
        .right_against_left_plane
        .vertex_sides[0] = None;
    assert_eq!(
        invalid_triangle.validate(),
        Err(MeshFacePairValidationError::InvalidTriangleClassification)
    );
}

#[test]
fn exact_boolean_public_shortcuts_handle_disjoint_operands() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);

    let preflight = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .preflight(&left, &right)
    .unwrap();
    assert!(!preflight.graph_had_unknowns);

    let union = ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
        .materialize(&left, &right)
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

    let intersection = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .materialize(&left, &right)
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
        let empty_result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&empty, &solid)
            .unwrap();
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
        let empty_evaluation = (ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
            .evaluate(&empty, &solid)
            .unwrap();
        empty_evaluation.validate().unwrap();
        let mut relabeled_empty_facts = empty_evaluation.clone();
        relabeled_empty_facts.certifications.trivial.left_empty = false;
        relabeled_empty_facts.certifications.trivial.right_empty = false;
        assert_eq!(
            relabeled_empty_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_empty_facts:?}"
        );
        if operation == ExactBooleanOperation::Union {
            assert_eq!(
                empty_result.freshness_against_sources(&empty, &disjoint_solid),
                ExactReportFreshness::SourceReplayMismatch
            );
        }

        let empty_open_result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&empty, &open_disjoint_left)
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

        let direct_empty_open = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&empty, &open_disjoint_left)
            .unwrap();
        assert_eq!(direct_empty_open.kind, empty_open_result.kind);
        assert!(direct_empty_open.mesh.triangles().is_empty());
        assert_eq!(
            direct_empty_open.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::EmptyOperand
            },
            "{operation:?}: {direct_empty_open:?}"
        );

        let open_empty_result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&open_disjoint_left, &empty)
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

        let disjoint_result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&solid, &disjoint_solid)
            .unwrap();
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
        let disjoint_evaluation = (ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
            .evaluate(&solid, &disjoint_solid)
            .unwrap();
        disjoint_evaluation.validate().unwrap();
        let mut relabeled_disjoint_facts = disjoint_evaluation.clone();
        relabeled_disjoint_facts
            .certifications
            .trivial
            .bounds_disjoint = false;
        assert_eq!(
            relabeled_disjoint_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_disjoint_facts:?}"
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

        let identical_result =
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .materialize(&open_identical_left, &open_identical_right)
                .unwrap();
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
        let identical_evaluation =
            (ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY))
                .evaluate(&open_identical_left, &open_identical_right)
                .unwrap();
        identical_evaluation.validate().unwrap();
        let mut relabeled_identity_report = identical_evaluation.clone();
        relabeled_identity_report.certifications.identical.status =
            hypermesh::ExactIdenticalMeshStatus::TriangleSequenceMismatch;
        relabeled_identity_report
            .certifications
            .identical
            .validate()
            .unwrap();
        assert_eq!(
            relabeled_identity_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_identity_report:?}"
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
        let closed_identical_result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
            .materialize(&open_identical_left, &open_identical_right)
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

        let same_surface_result =
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .materialize(&open_identical_left, &open_same_surface_right)
                .unwrap();
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
        let same_surface_evaluation =
            (ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY))
                .evaluate(&open_identical_left, &open_same_surface_right)
                .unwrap();
        same_surface_evaluation.validate().unwrap();
        let mut relabeled_same_surface_report = same_surface_evaluation.clone();
        relabeled_same_surface_report
            .certifications
            .same_surface
            .status = ExactSameSurfaceStatus::VertexCountMismatch;
        relabeled_same_surface_report
            .certifications
            .same_surface
            .left_to_right
            .clear();
        relabeled_same_surface_report
            .certifications
            .same_surface
            .right_to_left
            .clear();
        relabeled_same_surface_report
            .certifications
            .same_surface
            .left_triangles
            .clear();
        relabeled_same_surface_report
            .certifications
            .same_surface
            .right_triangles
            .clear();
        relabeled_same_surface_report
            .certifications
            .same_surface
            .predicates
            .clear();
        relabeled_same_surface_report
            .certifications
            .same_surface
            .validate()
            .unwrap();
        assert_eq!(
            relabeled_same_surface_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_same_surface_report:?}"
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
        let closed_same_surface_result =
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&open_identical_left, &open_same_surface_right)
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
        let lower_dimensional_evaluation =
            (ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
                .evaluate(&open_identical_left, &open_same_surface_right)
                .unwrap();
        lower_dimensional_evaluation.validate().unwrap();
        let mut relabeled_lower_dimensional_facts = lower_dimensional_evaluation.clone();
        relabeled_lower_dimensional_facts
            .certifications
            .regularized_solid
            .left_open_surface = false;
        assert_eq!(
            relabeled_lower_dimensional_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_lower_dimensional_facts:?}"
        );

        let mixed_dimensional_result =
            ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&solid, &open_disjoint_left)
                .unwrap();
        assert_shortcut(
            &mixed_dimensional_result,
            &solid,
            &open_disjoint_left,
            &open_disjoint_left,
            &open_disjoint_right,
            operation,
            ValidationPolicy::CLOSED,
            hypermesh::ExactBooleanShortcutKind::MixedDimensionalRegularizedSolid,
        );
        let mixed_dimensional_evaluation =
            (ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED))
                .evaluate(&solid, &open_disjoint_left)
                .unwrap();
        mixed_dimensional_evaluation.validate().unwrap();
        let mut relabeled_mixed_dimensional_facts = mixed_dimensional_evaluation.clone();
        relabeled_mixed_dimensional_facts
            .certifications
            .regularized_solid
            .right_open_surface = false;
        assert_eq!(
            relabeled_mixed_dimensional_facts.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_mixed_dimensional_facts:?}"
        );

        let open_disjoint_result =
            ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY)
                .materialize_open_surface_disjoint(&open_disjoint_left, &open_disjoint_right)
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
        let open_disjoint_evaluation =
            (ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY))
                .evaluate(&open_disjoint_left, &open_disjoint_right)
                .unwrap();
        open_disjoint_evaluation.validate().unwrap();
        let mut relabeled_disjoint_report = open_disjoint_evaluation.clone();
        relabeled_disjoint_report
            .certifications
            .open_surface_disjoint
            .status = ExactOpenSurfaceDisjointStatus::NotOpenSurface;
        relabeled_disjoint_report
            .certifications
            .open_surface_disjoint
            .left_open_surface = false;
        relabeled_disjoint_report
            .certifications
            .open_surface_disjoint
            .validate()
            .unwrap();
        assert_eq!(
            relabeled_disjoint_report.validate(),
            Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch),
            "{operation:?}: {relabeled_disjoint_report:?}"
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

    let solid_disjoint =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize(&solid, &disjoint_solid)
            .unwrap();
    assert_eq!(
        solid_disjoint.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint
        }
    );
    let identical_replay = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .materialize(&open_identical_left, &open_identical_right)
    .unwrap();
    assert_eq!(
        identical_replay.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::Identical
        }
    );
    assert!(
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED,)
            .materialize_open_surface_disjoint(&solid, &disjoint_solid)
            .unwrap()
            .is_none()
    );
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
            let result = ExactBooleanRequest::new(operation, ValidationPolicy::CLOSED)
                .materialize(&left, right)
                .unwrap();
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut {
                    operation,
                    shortcut: hypermesh::ExactBooleanShortcutKind::ArrangementCellComplex
                }
            );
            result.validate().unwrap();
            let mut stale_output = result.clone();
            stale_output.mesh = stale_right.clone();
            assert!(stale_output.validate().is_err(), "{stale_output:?}");
            assert_eq!(
                stale_output.freshness_against_sources(&left, right),
                ExactReportFreshness::StaleRegionFacts,
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
                    let replay = ExactBooleanRequest::with_boundary_policy(operation, ValidationPolicy::CLOSED, ExactBoundaryBooleanPolicy::Reject).materialize(&left, right)
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
    let open_same_surface = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .materialize(&open_left, &open_right)
    .unwrap();
    assert_eq!(
        open_same_surface.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::SameSurface
        }
    );
    let stale_replay =
        ExactBooleanRequest::new(ExactBooleanOperation::Union, ValidationPolicy::CLOSED)
            .materialize(&left, &stale_right)
            .unwrap();
    assert_eq!(
        stale_replay.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Union,
            shortcut: hypermesh::ExactBooleanShortcutKind::BoundsDisjoint
        }
    );

    let convex_left = tetra_from_corners([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let convex_same_surface = same_surface_a;
    let convex_same_surface_replay = ExactBooleanRequest::new(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .materialize(&convex_left, &convex_same_surface)
    .unwrap();
    assert_eq!(
        convex_same_surface_replay.kind,
        ExactBooleanResultKind::CertifiedShortcut {
            operation: ExactBooleanOperation::Intersection,
            shortcut: hypermesh::ExactBooleanShortcutKind::ConvexIntersection
        }
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

    let attempt = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .arrangement_attempt(&left, &right, ExactRegularizationPolicy::REGULARIZED_SOLID)
    .unwrap();
    attempt.validate().unwrap();
    assert_eq!(
        attempt.topology_assembly,
        Some(ExactTopologyAssemblyStatus::Complete),
        "{attempt:?}"
    );
    assert_eq!(
        attempt
            .topology_assembly_report
            .as_ref()
            .map(|report| report.status),
        attempt.topology_assembly,
        "{attempt:?}"
    );
    assert_eq!(
        attempt.region_ownership,
        Some(ExactRegionOwnershipStatus::VolumeResolved),
        "{attempt:?}"
    );
    assert_eq!(
        attempt
            .region_ownership_report
            .as_ref()
            .map(|report| report.status),
        attempt.region_ownership,
        "{attempt:?}"
    );
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
    let mut stale_attempt_gate = attempt.clone();
    stale_attempt_gate.topology_assembly = Some(ExactTopologyAssemblyStatus::MissingRegionPlan);
    assert_eq!(
        stale_attempt_gate.freshness_against_sources(&left, &right),
        ExactReportFreshness::StaleStatusEvidence
    );
    let mut stale_attempt_report = attempt.clone();
    stale_attempt_report
        .region_ownership_report
        .as_mut()
        .expect("attempt should retain ownership report")
        .status = ExactRegionOwnershipStatus::RequiresWinding;
    assert_eq!(
        stale_attempt_report.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );

    let mut blocked_attempt = attempt.clone();
    blocked_attempt.stage = hypermesh::ExactArrangementBooleanStage::ArrangementBuilt;
    blocked_attempt.decline =
        Some(hypermesh::ExactArrangementBooleanDecline::ArrangementBlockers(Vec::new()));
    blocked_attempt.materialized_shortcut = None;
    blocked_attempt.arrangement_blockers = 1;
    blocked_attempt.selected_faces = 0;
    blocked_attempt.reversed_selected_faces = 0;
    blocked_attempt.volume_oriented_selected_faces = 0;
    blocked_attempt.label_oriented_selected_faces = 0;
    blocked_attempt.selected_volume_regions = 0;
    blocked_attempt.output_vertices = 0;
    blocked_attempt.output_triangles = 0;
    assert_eq!(
        blocked_attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
    );

    blocked_attempt.decline = Some(
        hypermesh::ExactArrangementBooleanDecline::ArrangementBlockers(vec![
            hypermesh::ExactArrangementBlocker::UnresolvedIntersection,
            hypermesh::ExactArrangementBlocker::UndecidableOrdering,
        ]),
    );
    assert_eq!(
        blocked_attempt.validate(),
        Err(hypermesh::ExactReportValidationError::StatusEvidenceMismatch)
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

    let result = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .materialize(&left, &right)
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
    let report = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .boundary_touching_report(&left, &right)
    .unwrap();
    assert!(report.is_certified(), "{report:?}");
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let preflight = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .preflight(&left, &right)
    .unwrap();
    assert_eq!(
        preflight.support,
        hypermesh::ExactBooleanSupport::CertifiedBoundaryPolicyShortcut,
        "{preflight:?}"
    );
    let rejected_policy_preflight = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .preflight(&left, &right)
    .unwrap();
    assert_eq!(
        rejected_policy_preflight.support,
        hypermesh::ExactBooleanSupport::RequiresBoundaryPolicy,
        "{rejected_policy_preflight:?}"
    );

    let policy_preflight = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .preflight(&left, &right)
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
            .is_ok(),
        "default replay should certify a boundary-policy preflight"
    );

    let rejected_readiness = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .winding_readiness(&left, &right)
    .unwrap();
    assert_eq!(
        rejected_readiness.status,
        ExactWindingReadinessStatus::BoundaryPolicyRequired,
        "{rejected_readiness:?}"
    );

    let policy_readiness = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .winding_readiness(&left, &right)
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
    let default_result = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .materialize(&left, &right)
    .unwrap();
    default_result.validate().unwrap();
    assert!(matches!(
        default_result.kind,
        ExactBooleanResultKind::BoundaryPolicyShortcut {
            operation: ExactBooleanOperation::Union
        }
    ));

    let projected = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .materialize(&left, &right)
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

    let closed_intersection_preflight = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .preflight(&left, &right)
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

    let closed_intersection_readiness = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .winding_readiness(&left, &right)
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

    let closed_intersection = ExactBooleanRequest::with_boundary_policy(
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .materialize(&left, &right)
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
        let closed_policy_preflight = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .preflight(&left, &right)
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
        let closed_policy_readiness = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .winding_readiness(&left, &right)
        .unwrap();
        assert_eq!(
            closed_policy_readiness.status,
            ExactWindingReadinessStatus::LowerDimensionalRegularizedSolidAlreadyMaterialized,
            "{operation:?}: {closed_policy_readiness:?}"
        );
        let materialized = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .materialize_boundary_touching_policy(&left, &right)
        .unwrap()
        .expect("closed lower-dimensional regularization should materialize directly");
        assert_eq!(
            materialized.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                operation,
                shortcut: hypermesh::ExactBooleanShortcutKind::LowerDimensionalRegularizedSolid
            },
            "{operation:?}: {materialized:?}"
        );
        materialized
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            )
            .unwrap();
        let closed_regularized = ExactBooleanRequest::with_boundary_policy(
            operation,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        )
        .materialize(&left, &right)
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

    let report = ExactBooleanRequest::new(
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .boundary_touching_report(&left, &right)
    .unwrap();

    assert_eq!(report.status, ExactBoundaryTouchingStatus::NotBoundaryOnly);
    assert_eq!(report.blocker.kind, ExactBooleanBlockerKind::NeedsWinding);
    assert!(report.blocker.candidate_pairs > 0);
    report.validate().unwrap();
    report.validate_against_sources(&left, &right).unwrap();

    let mut stale = report;
    stale.blocker.kind = ExactBooleanBlockerKind::NeedsBoundaryPolicy;
    assert!(stale.validate().is_err());
}
