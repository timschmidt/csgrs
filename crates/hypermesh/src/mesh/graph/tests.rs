use super::*;
use crate::mesh::ExactMesh;
use crate::mesh::boolean::region::FaceRegionPlaneRelation;
use crate::mesh::validation::ExactMeshValidationPolicy;
use crate::mesh::view::PreparedMeshPairBoolean;

fn q(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / &Real::from(denominator)).expect("nonzero denominator")
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn rational_p3(x: [i64; 2], y: [i64; 2], z: [i64; 2]) -> Point3 {
    Point3::new(q(x[0], x[1]), q(y[0], y[1]), q(z[0], z[1]))
}

fn split_point(point: Point3, parameter: Real, face_pair: [usize; 2]) -> EdgeSplitPoint {
    EdgeSplitPoint {
        face_pair,
        plane_face: face_pair[1],
        parameter: parameter.clone(),
        parameter_ratio: SegmentPlaneParameterRatio {
            numerator: parameter,
            denominator: Real::from(1),
        },
        point,
        endpoint_sides: [Some(PlaneSide::Above), Some(PlaneSide::Below)],
    }
}

#[test]
fn face_region_stage_replays_from_internal_graph() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let stale_right = ExactMesh::from_i64_triangles_with_policy(
        &[8, -1, -1, 8, 3, 1, 8, 3, -1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
    let regions = geometry.region_plan(&left, &right);

    let classifications = regions
        .classify_against_opposite_face_planes(&left, &right)
        .unwrap();
    let triangulations = regions.triangulate_with_earcut(&left, &right).unwrap();

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

#[test]
fn face_cell_cdt_replays_from_internal_graph() {
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
    let separated_right = ExactMesh::from_i64_triangles(
        &[10, 10, 10, 11, 10, 10, 10, 11, 10, 10, 10, 11],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();

    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    assert!(graph.face_pairs.len() > 1);
    let mut reversed_graph = graph.clone();
    reversed_graph.face_pairs.reverse();
    reversed_graph
        .validate_against_sources(&left, &right)
        .unwrap();
    let (cell_regions, cell_triangulations) = graph
        .triangulate_face_cells_with_cdt(&left, &right)
        .unwrap()
        .expect("overlapping closed solids should expose exact CDT face cells");
    assert_eq!(
        cell_regions.regions.len(),
        left.triangles().len() + right.triangles().len()
    );
    assert_eq!(cell_triangulations.len(), cell_regions.regions.len());
    assert!(cell_regions.validate(&left, &right).is_valid());
    graph
        .validate_face_cell_cdt_against_sources(&cell_regions, &cell_triangulations, &left, &right)
        .unwrap();
    assert!(
        graph
            .validate_face_cell_cdt_against_sources(
                &cell_regions,
                &cell_triangulations,
                &left,
                &separated_right
            )
            .is_err()
    );
}

#[test]
fn intersection_graph_skips_face_preparation_for_disjoint_mesh_bounds() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 0, 0, 12, 0, 0, 10, 2, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    assert!(graph.face_pairs.is_empty());
    let graph = build_validated_intersection_graph(&left, &right).unwrap();
    assert!(graph.face_pairs.is_empty());
}

#[test]
fn intersection_graph_retains_coplanar_face_pair_events_internal() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0, 9, 9, 9],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 1, 0, 0, 0, 1, 0, -9, -9, -9],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    graph.validate_against_sources(&left, &right).unwrap();
    let retained_pair = graph
        .face_pairs
        .iter()
        .find(|pair| pair.left_face == 0 && pair.right_face == 0)
        .expect("coplanar overlap should retain a graph face pair");
    assert!(retained_pair.projection.is_some());
    assert!(retained_pair.has_constructive_events());

    let mut overlaps = graph.coplanar_overlap_graph_iter().collect::<Vec<_>>();
    let overlap = overlaps.pop().unwrap();
    assert!(graph.coplanar_overlap_graph_count_hint() >= overlaps.len());
    overlap.validate_against_sources(&left, &right).unwrap();
    let mut invalid_overlap = overlap.clone();
    invalid_overlap.edge_overlaps.clear();
    invalid_overlap.vertex_overlaps.clear();
    assert!(
        invalid_overlap
            .validate_against_sources(&left, &right)
            .is_err()
    );

    let split_plan = graph.coplanar_overlap_split_plan(&left, &right).unwrap();
    split_plan.validate_against_sources(&left, &right).unwrap();
    let mut stale_split_plan = split_plan.clone();
    stale_split_plan.graphs.clear();
    assert!(
        stale_split_plan
            .validate_against_sources(&left, &right)
            .is_err()
    );

    let evidence = graph.coplanar_arrangement_evidence(&left, &right).unwrap();
    evidence.validate_against_sources(&left, &right).unwrap();
    let mut invalid_evidence = evidence.clone();
    invalid_evidence.graph_count += 1;
    assert!(
        invalid_evidence
            .validate_against_sources(&left, &right)
            .is_err()
    );
    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[9, 0, 0, 10, 0, 0, 9, 1, 0, -9, -9, -9],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        evidence
            .validate_against_sources(&left, &separated_right)
            .is_err()
    );
}

#[test]
fn face_pair_candidate_retains_source_plane_split_events_internal() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, -1, 1, 3, 1, 1, 3, -1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();

    let graph = build_unvalidated_intersection_graph(&left, &right).unwrap();
    let prepared_left = left.view().prepare_broad_phase().unwrap();
    let prepared_right = right.view().prepare_broad_phase().unwrap();
    assert_eq!(
        build_unvalidated_intersection_graph_from_prepared_views(&prepared_left, &prepared_right)
            .unwrap(),
        graph
    );
    graph.validate_against_meshes(&left, &right).unwrap();
    let prepared_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
    assert_eq!(prepared_pair.prepare_face_pair_classifications(), 1);
    let mut first_classifications = Vec::new();
    prepared_pair
        .with_current_face_pair_classifications(|classifications| {
            first_classifications.extend_from_slice(classifications);
        })
        .unwrap();
    let mut repeated_classifications = Vec::new();
    prepared_pair
        .with_current_face_pair_classifications(|classifications| {
            repeated_classifications.extend_from_slice(classifications);
        })
        .unwrap();
    assert_eq!(first_classifications, repeated_classifications);
    assert_eq!(
        first_classifications,
        vec![classify_mesh_face_pair_unchecked(&left, 0, &right, 0)]
    );
    assert!(!prepared_pair.has_retained_arrangement_shortcut_facts());
    let shortcut_facts = prepared_pair.arrangement_cell_complex_shortcut_facts();
    assert_eq!(
        shortcut_facts,
        crate::mesh::boolean::evidence::ExactArrangementCellComplexShortcutFacts::from_sources(
            &left, &right
        )
    );
    assert!(prepared_pair.has_retained_arrangement_shortcut_facts());
    assert!(!prepared_pair.has_retained_intersection_graph());
    assert_eq!(
        build_unvalidated_intersection_graph_from_prepared_pair_rc(&prepared_pair)
            .unwrap()
            .as_ref(),
        &graph
    );
    assert!(prepared_pair.has_retained_intersection_graph());
    assert!(!prepared_pair.intersection_graph_is_current());
    assert!(prepared_pair.intersection_graph_is_certificate_blocked());
    assert_eq!(
        prepared_pair
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        build_validated_intersection_graph_from_prepared_pair(&prepared_pair)
            .unwrap()
            .as_ref(),
        &graph
    );
    assert!(prepared_pair.intersection_graph_is_current());
    assert_eq!(
        (
            prepared_pair
                .current_intersection_graph_counts()
                .unwrap()
                .face_pair_count(),
            prepared_pair
                .current_intersection_graph_counts()
                .unwrap()
                .event_count(),
        ),
        (graph.face_pairs.len(), graph.event_count())
    );
    assert_eq!(
        build_validated_intersection_graph_from_prepared_pair(&prepared_pair)
            .unwrap()
            .as_ref(),
        &graph
    );
    let cached_union = prepared_pair.union().unwrap();
    cached_union.validate_retained_state().unwrap();
    assert!(prepared_pair.result_is_current(PreparedMeshPairBoolean::Union));
    prepared_pair
        .with_arrangement_view(|view| {
            view.validate_retained_state().unwrap();
        })
        .unwrap();
    assert!(prepared_pair.arrangement_is_current());
    prepared_pair.retain_intersection_graph(ExactIntersectionGraph::from_face_pairs(Vec::new()));
    assert!(prepared_pair.intersection_graph_is_certificate_blocked());
    assert!(!prepared_pair.has_retained_arrangement());
    assert!(!prepared_pair.has_retained_result(PreparedMeshPairBoolean::Union));
    assert_eq!(
        prepared_pair.retained_result_outcome(PreparedMeshPairBoolean::Union),
        None
    );
    assert_eq!(
        build_validated_intersection_graph_from_prepared_pair(&prepared_pair)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::StaleFactReplay
    );
    assert!(prepared_pair.intersection_graph_is_certificate_blocked());
    let retained_pair = graph
        .face_pairs
        .iter()
        .find(|pair| pair.left_face == 0 && pair.right_face == 0)
        .expect("candidate pair should be retained in the graph");
    assert!(retained_pair.projection.is_none());
    assert!(!retained_pair.events.is_empty());
    assert!(retained_pair.has_constructive_events());
    retained_pair
        .validate_against_meshes(&left, &right)
        .unwrap();
    graph.validate_against_sources(&left, &right).unwrap();
    let mut stale_graph = graph.clone();
    stale_graph.face_pairs[0].events.clear();
    assert!(stale_graph.validate_against_sources(&left, &right).is_err());

    let separated_right = ExactMesh::from_i64_triangles_with_policy(
        &[9, -1, -1, 9, 3, 1, 9, 3, -1],
        &[0, 1, 2],
        ExactMeshValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    assert!(
        graph
            .validate_against_sources(&left, &separated_right)
            .is_err()
    );
    let edge_splits = graph.edge_split_plan();
    assert!(edge_splits.validate().blockers.is_empty());
    assert!(
        edge_splits
            .validate_against_sources(&left, &separated_right)
            .blockers
            .iter()
            .any(|blocker| blocker.kind == SplitPlanBlockerKind::SourceReplayMismatch)
    );
    let mut stale_edge_splits = edge_splits.clone();
    stale_edge_splits.unknown_orderings += 1;
    assert!(!stale_edge_splits.validate().blockers.is_empty());

    let graph_vertices = graph.graph_vertex_plan();
    assert!(graph_vertices.validate().blockers.is_empty());
    let topology = graph.split_topology_plan();
    assert!(topology.validate().blockers.is_empty());
    let face_plan = graph.face_split_plan();
    assert!(
        face_plan
            .validate_against_sources(&left, &right)
            .blockers
            .is_empty()
    );
    let geometry = graph.face_split_geometry_plan(&left, &right).unwrap();
    assert!(
        geometry
            .validate_against_sources(&left, &right)
            .blockers
            .is_empty()
    );
    let mut noncanonical_chain_geometry = geometry.clone();
    noncanonical_chain_geometry.faces[0].boundary_chains[0]
        .nodes
        .rotate_left(1);
    let noncanonical_chain_report =
        noncanonical_chain_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        noncanonical_chain_report
            .blockers
            .iter()
            .any(|blocker| blocker.kind == SplitPlanBlockerKind::WrongChainStart),
        "{noncanonical_chain_report:?}"
    );
    assert!(!noncanonical_chain_report.blockers.is_empty());
    let noncanonical_chain_error =
        split_plan_report_to_mesh_error(noncanonical_chain_report.clone());
    assert!(
        noncanonical_chain_error
            .blockers()
            .iter()
            .any(|blocker| blocker.source_side() == Some(ExactMeshSourceSide::Left)),
        "{noncanonical_chain_error:?}"
    );
    let mut duplicate_chain_geometry = geometry.clone();
    let duplicate_chain = duplicate_chain_geometry.faces[0].boundary_chains[0].clone();
    duplicate_chain_geometry.faces[0]
        .boundary_chains
        .push(duplicate_chain);
    let duplicate_chain_report =
        duplicate_chain_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        duplicate_chain_report
            .blockers
            .iter()
            .any(|blocker| { blocker.kind == SplitPlanBlockerKind::DuplicateFaceSplitEdge }),
        "{duplicate_chain_report:?}"
    );
    assert!(!duplicate_chain_report.blockers.is_empty());
    let mut stale_original_point_geometry = geometry.clone();
    if let FaceSplitBoundaryNode::OriginalVertex { point, .. } =
        &mut stale_original_point_geometry.faces[0].boundary_chains[0].nodes[0]
    {
        *point = p(2, 0, 0);
    }
    let stale_original_point_report =
        stale_original_point_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        stale_original_point_report.blockers.iter().any(|blocker| {
            blocker.kind == SplitPlanBlockerKind::BoundaryNodeSourcePointMismatch
        }),
        "{stale_original_point_report:?}"
    );
    assert!(!stale_original_point_report.blockers.is_empty());
    let mut relabeled_geometry = geometry.clone();
    relabeled_geometry.faces[0].triangle.swap(0, 1);
    let geometry_report = relabeled_geometry.validate_boundary_incidence(&left, &right);
    assert!(
        geometry_report
            .blockers
            .iter()
            .any(|blocker| { blocker.kind == SplitPlanBlockerKind::SourceTriangleMismatch }),
        "{geometry_report:?}"
    );
    assert!(!geometry_report.blockers.is_empty());
    let regions = geometry.region_plan(&left, &right);
    assert!(
        regions
            .validate_against_sources(&left, &right)
            .blockers
            .is_empty()
    );
    let mut closed_duplicate_regions = regions.clone();
    let first_region_node = closed_duplicate_regions.regions[0].boundary[0].clone();
    closed_duplicate_regions.regions[0]
        .boundary
        .push(first_region_node);
    let closed_duplicate_report = closed_duplicate_regions.validate(&left, &right);
    assert!(
        closed_duplicate_report.blockers.iter().any(|blocker| {
            blocker.kind == SplitPlanBlockerKind::DuplicateConsecutiveRegionNode
        }),
        "{closed_duplicate_report:?}"
    );
    assert!(!closed_duplicate_report.blockers.is_empty());
    let mut stale_region_point = regions.clone();
    if let FaceSplitBoundaryNode::OriginalVertex { point, .. } =
        &mut stale_region_point.regions[0].boundary[0]
    {
        *point = p(2, 0, 0);
    }
    let stale_region_point_report = stale_region_point.validate(&left, &right);
    assert!(
        stale_region_point_report.blockers.iter().any(|blocker| {
            blocker.kind == SplitPlanBlockerKind::BoundaryNodeSourcePointMismatch
        }),
        "{stale_region_point_report:?}"
    );
    assert!(!stale_region_point_report.blockers.is_empty());
    let mut missing_region_vertex = regions.clone();
    if let FaceSplitBoundaryNode::OriginalVertex { vertex, .. } =
        &mut missing_region_vertex.regions[0].boundary[0]
    {
        *vertex = usize::MAX;
    }
    let missing_region_vertex_report = missing_region_vertex.validate(&left, &right);
    assert!(
        missing_region_vertex_report.blockers.iter().any(|blocker| {
            blocker.kind == SplitPlanBlockerKind::BoundaryNodeSourceVertexOutOfRange
        }),
        "{missing_region_vertex_report:?}"
    );
    assert!(!missing_region_vertex_report.blockers.is_empty());
    let mut relabeled_regions = regions.clone();
    relabeled_regions.regions[0].triangle.swap(0, 1);
    let region_report = relabeled_regions.validate(&left, &right);
    assert!(
        region_report
            .blockers
            .iter()
            .any(|blocker| { blocker.kind == SplitPlanBlockerKind::SourceTriangleMismatch }),
        "{region_report:?}"
    );
    assert!(!region_report.blockers.is_empty());
}

#[test]
fn graph_vertex_plan_buckets_exact_rational_points() {
    let point = rational_p3([1, 2], [-3, 4], [5, 6]);
    let split_plan = ExactEdgeSplitPlan {
        splits: vec![
            EdgeSplit {
                side: MeshSide::Left,
                edge: [0, 1],
                points: vec![split_point(point.clone(), q(1, 3), [0, 0])],
            },
            EdgeSplit {
                side: MeshSide::Right,
                edge: [2, 3],
                points: vec![split_point(point, q(2, 3), [1, 1])],
            },
            EdgeSplit {
                side: MeshSide::Left,
                edge: [3, 4],
                points: vec![split_point(
                    rational_p3([2, 3], [-3, 4], [5, 6]),
                    q(1, 2),
                    [2, 2],
                )],
            },
        ],
        unknown_orderings: 0,
    };

    let graph_vertices = graph_vertex_plan(&split_plan);

    assert_eq!(graph_vertices.unresolved_equalities, 0);
    assert_eq!(graph_vertices.vertices.len(), 2);
    assert_eq!(graph_vertices.vertices[0].uses.len(), 2);
    assert_eq!(graph_vertices.vertices[1].uses.len(), 1);
}

#[test]
fn graph_unknowns_include_unknown_segment_plane_events() {
    let graph = ExactIntersectionGraph::from_face_pairs(vec![FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::Unknown,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides: [None, Some(PlaneSide::Above)],
        }],
    }]);

    assert!(graph.validate().is_ok());
    assert!(graph.has_unknowns());
}

#[test]
fn face_pair_validation_rejects_relation_event_family_mismatch() {
    let candidate_with_coplanar_event = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::Candidate,
        projection: None,
        events: vec![IntersectionEvent::CoplanarVertex {
            vertex_side: MeshSide::Left,
            vertex: 0,
            triangle_side: MeshSide::Right,
            triangle_face: 0,
            location: TriangleLocation::Inside,
        }],
    };
    assert_eq!(
        candidate_with_coplanar_event.validate(),
        Err(IntersectionGraphValidationError::NonCoplanarPairHasCoplanarEvent)
    );

    let coplanar_with_segment_plane_event = FacePairEvents {
        left_face: 0,
        right_face: 0,
        relation: MeshFacePairRelation::CoplanarTouching,
        projection: Some(CoplanarProjection::Xy),
        events: vec![IntersectionEvent::SegmentPlane {
            segment_side: MeshSide::Left,
            edge: [0, 1],
            plane_side: MeshSide::Right,
            plane_face: 0,
            relation: SegmentPlaneRelation::Unknown,
            point: None,
            parameter: None,
            parameter_ratio: None,
            construction_failure: None,
            endpoint_sides: [None, Some(PlaneSide::Above)],
        }],
    };
    assert_eq!(
        coplanar_with_segment_plane_event.validate(),
        Err(IntersectionGraphValidationError::CoplanarPairHasSegmentPlaneEvent)
    );
}

#[test]
fn coplanar_split_validation_rejects_invalid_vertex_overlap_facts() {
    let same_side_vertex = CoplanarOverlapSplitGraph {
        left_face: 0,
        right_face: 0,
        projection: CoplanarProjection::Xy,
        edge_splits: Vec::new(),
        vertex_overlaps: vec![CoplanarVertexOverlap {
            vertex_side: MeshSide::Left,
            vertex: 0,
            triangle_side: MeshSide::Left,
            triangle_face: 0,
            location: TriangleLocation::Inside,
        }],
    };
    assert_eq!(
        same_side_vertex.validate(),
        Err(CoplanarOverlapSplitValidationError::SameSideVertexOverlap)
    );

    let nonconstructive_vertex = CoplanarOverlapSplitGraph {
        left_face: 0,
        right_face: 0,
        projection: CoplanarProjection::Xy,
        edge_splits: Vec::new(),
        vertex_overlaps: vec![CoplanarVertexOverlap {
            vertex_side: MeshSide::Left,
            vertex: 0,
            triangle_side: MeshSide::Right,
            triangle_face: 0,
            location: TriangleLocation::Outside,
        }],
    };
    assert_eq!(
        nonconstructive_vertex.validate(),
        Err(CoplanarOverlapSplitValidationError::NonConstructiveVertexOverlap)
    );
}

#[test]
fn coplanar_arrangement_evidence_rejects_overflowing_counts() {
    let graph_overflow = CoplanarArrangementEvidence {
        status: CoplanarArrangementEvidenceStatus::NeedsPlanarCells,
        graph_count: usize::MAX,
        overlapping_graphs: usize::MAX,
        touching_graphs: 1,
        edge_overlap_count: 1,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: 0,
        interval_endpoint_count: 0,
    };
    assert_eq!(
        graph_overflow.validate(),
        Err(CoplanarArrangementEvidenceError::GraphCountMismatch)
    );

    let split_overflow = CoplanarArrangementEvidence {
        status: CoplanarArrangementEvidenceStatus::NeedsPlanarCells,
        graph_count: 1,
        overlapping_graphs: 1,
        touching_graphs: 0,
        edge_overlap_count: usize::MAX,
        vertex_overlap_count: 0,
        point_split_count: usize::MAX,
        interval_overlap_count: 1,
        interval_endpoint_count: 2,
    };
    assert_eq!(
        split_overflow.validate(),
        Err(CoplanarArrangementEvidenceError::SplitCountExceedsEdgeEvidence)
    );

    let interval_overflow = CoplanarArrangementEvidence {
        status: CoplanarArrangementEvidenceStatus::NeedsPlanarCells,
        graph_count: 1,
        overlapping_graphs: 1,
        touching_graphs: 0,
        edge_overlap_count: usize::MAX,
        vertex_overlap_count: 0,
        point_split_count: 0,
        interval_overlap_count: usize::MAX,
        interval_endpoint_count: usize::MAX,
    };
    assert_eq!(
        interval_overflow.validate(),
        Err(CoplanarArrangementEvidenceError::IntervalEndpointCountMismatch)
    );
}
