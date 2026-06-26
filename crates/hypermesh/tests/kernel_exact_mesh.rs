use hyperlimit::{ApproximationPolicy, MeshSource, Point3, SourceProvenance};
use hypermesh::ExactMesh;
use hypermesh::kernel::{
    ArrangementView, EdgeRef, ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError, ExactMeshRef,
    FaceRef, MeshView, PreparedMeshPair, PreparedMeshPairArrangementCounts,
    PreparedMeshPairBoolean, PreparedMeshPairBroadPhaseSummary,
    PreparedMeshPairBroadPhaseTraversalSummary, PreparedMeshPairCacheStatus,
    PreparedMeshPairClassificationCounts, PreparedMeshPairFactState,
    PreparedMeshPairIntersectionGraphCounts, PreparedMeshPairPlanKind,
    PreparedMeshPairResultOutcome, PreparedMeshPairSweepActiveSet, PreparedMeshPairSweepAxis,
    PreparedMeshPairSweepDirection, PreparedMeshView, TriangleRef, VertexRef,
};
use hyperreal::Real;

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
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
        vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
        SourceProvenance::exact("test tetra"),
    )
    .unwrap()
}

#[test]
fn exact_mesh_named_boolean_methods_materialize_meshes() {
    let empty = ExactMesh::new(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("empty test mesh"),
    )
    .unwrap();
    let solid = tetra([0, 0, 0]);

    let union = empty.union(&solid).unwrap();
    union.validate_retained_state().unwrap();
    assert_eq!(union.triangle_count(), solid.triangle_count());

    let intersection = empty.intersection(&solid).unwrap();
    intersection.validate_retained_state().unwrap();
    assert_eq!(intersection.triangle_count(), 0);

    let difference = solid.difference(&empty).unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(difference.triangle_count(), solid.triangle_count());

    let xor = empty.xor(&solid).unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(xor.triangle_count(), solid.triangle_count());
}

#[test]
fn exact_mesh_borrowed_view_materializes_named_operations() {
    let empty = ExactMesh::new(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("empty test mesh"),
    )
    .unwrap();
    let solid = tetra([0, 0, 0]);

    let union = empty.view().union(solid.view()).unwrap();
    union.validate_retained_state().unwrap();
    assert_eq!(union.triangle_count(), solid.triangle_count());

    let intersection = empty.view().intersection(solid.view()).unwrap();
    intersection.validate_retained_state().unwrap();
    assert_eq!(intersection.triangle_count(), 0);

    let difference = solid.view().difference(empty.view()).unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(difference.triangle_count(), solid.triangle_count());

    let xor = empty.view().xor(solid.view()).unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(xor.triangle_count(), solid.triangle_count());
}

#[test]
fn prepared_mesh_pair_materializes_named_operations() {
    let empty = ExactMesh::new(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("empty test mesh"),
    )
    .unwrap();
    let solid = tetra([0, 0, 0]);
    let pair = empty.view().prepare_broad_phase_pair(solid.view()).unwrap();
    pair.cache_status()
        .source_pair()
        .require_current("source-pair stamp")
        .unwrap();
    let initial_status: PreparedMeshPairCacheStatus = pair.cache_status();
    let _source_state: PreparedMeshPairFactState = initial_status.source_pair();
    assert!(initial_status.source_pair().is_current());
    assert!(initial_status.broad_phase_traversal().is_missing());
    assert!(initial_status.candidate_face_pairs().is_missing());
    assert!(initial_status.face_pair_classifications().is_missing());
    assert!(initial_status.intersection_graph().is_missing());
    assert!(
        initial_status
            .result(PreparedMeshPairBoolean::Union)
            .is_missing()
    );
    assert!(
        initial_status
            .result(PreparedMeshPairBoolean::Union)
            .blocker("union result")
            .unwrap()
            .kind()
            == ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert!(pair.broad_phase_summary().plan().is_empty());
    assert_eq!(pair.broad_phase_summary().candidate_pair_capacity_hint(), 0);
    let initial_broad_phase: PreparedMeshPairBroadPhaseSummary = pair.broad_phase_summary();
    assert!(initial_broad_phase.plan().is_empty());
    assert_eq!(initial_broad_phase.left_face_count(), 0);
    assert_eq!(
        initial_broad_phase.right_face_count(),
        solid.triangle_count()
    );
    assert_eq!(initial_broad_phase.face_pair_product(), 0);
    assert_eq!(initial_broad_phase.candidate_pair_upper_bound(), 0);
    assert_eq!(initial_broad_phase.candidate_pair_capacity_hint(), 0);
    assert_eq!(initial_broad_phase.active_face_capacity_hint(), None);
    assert_eq!(pair.broad_phase_summary(), initial_broad_phase);
    assert!(pair.cache_status().face_pair_classifications().is_missing());
    assert!(
        pair.cache_status()
            .face_pair_classification_counts()
            .is_missing()
    );
    assert!(pair.cache_status().intersection_graph().is_missing());
    assert!(pair.cache_status().arrangement().is_missing());
    assert!(
        pair.cache_status()
            .arrangement_shortcut_facts()
            .is_missing()
    );
    assert_eq!(
        pair.cache_status()
            .arrangement_shortcut_facts()
            .require_current("arrangement shortcut facts")
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Union)
            .is_missing()
    );
    assert_eq!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Union)
            .require_current("union result")
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Union)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Union)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Union)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Intersection)
            .is_missing()
    );
    assert_eq!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Intersection)
            .require_current("intersection result")
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Intersection)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Intersection)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Intersection)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Difference)
            .is_missing()
    );
    assert_eq!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Difference)
            .require_current("difference result")
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Difference)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Difference)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Difference)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Xor)
            .is_missing()
    );
    assert_eq!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Xor)
            .require_current("xor result")
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Xor)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Xor)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Xor)
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .intersection_graph()
            .require_current("intersection graph")
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .face_pair_classifications()
            .require_current("face-pair classification")
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_face_pair_classification_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_arrangement_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );

    let count_only_pair = empty.view().prepare_broad_phase_pair(solid.view()).unwrap();
    let count_only_classification_counts =
        count_only_pair.prepare_face_pair_classification_counts();
    assert_eq!(count_only_classification_counts.face_pair_count(), 0);
    assert_eq!(count_only_classification_counts.graph_required_count(), 0);
    let count_only_graph_counts = count_only_pair.prepare_intersection_graph().unwrap();
    assert_eq!(count_only_graph_counts.face_pair_count(), 0);
    assert_eq!(count_only_graph_counts.event_count(), 0);
    assert!(
        count_only_pair
            .cache_status()
            .face_pair_classifications()
            .is_missing()
    );
    assert!(
        count_only_pair
            .cache_status()
            .face_pair_classification_counts()
            .is_current()
    );
    assert!(
        count_only_pair
            .cache_status()
            .intersection_graph()
            .is_certificate_blocked()
    );

    assert_eq!(pair.prepare_face_pair_classifications(), 0);
    pair.cache_status()
        .face_pair_classifications()
        .require_current("face-pair classification")
        .unwrap();
    let classification_status = pair.cache_status();
    assert!(
        classification_status
            .face_pair_classifications()
            .is_current()
    );
    assert!(
        classification_status
            .face_pair_classification_counts()
            .is_current()
    );
    assert!(classification_status.intersection_graph().is_missing());
    let empty_classification_counts = pair
        .cache_status()
        .current_face_pair_classification_counts()
        .unwrap();
    assert_eq!(empty_classification_counts.face_pair_count(), 0);
    assert_eq!(empty_classification_counts.graph_required_count(), 0);
    assert_eq!(
        pair.cache_status()
            .current_face_pair_classification_counts()
            .unwrap(),
        empty_classification_counts
    );
    assert!(pair.cache_status().intersection_graph().is_missing());
    assert!(
        pair.cache_status()
            .arrangement_shortcut_facts()
            .is_missing()
    );

    let prepared_union_outcome = pair.prepare_result(PreparedMeshPairBoolean::Union).unwrap();
    assert!(prepared_union_outcome.is_mesh());
    let union = pair.current_result(PreparedMeshPairBoolean::Union).unwrap();
    union.validate_retained_state().unwrap();
    assert_eq!(union.triangle_count(), solid.triangle_count());
    assert!(pair.cache_status().face_pair_classifications().is_current());
    assert_eq!(
        pair.cache_status()
            .current_face_pair_classification_counts()
            .unwrap(),
        empty_classification_counts
    );
    assert!(pair.cache_status().intersection_graph().is_missing());
    assert_eq!(
        pair.cache_status()
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.cache_status()
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert!(
        pair.cache_status()
            .arrangement_shortcut_facts()
            .is_missing()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Union)
            .is_current()
    );
    let union_status = pair.cache_status();
    assert!(union_status.arrangement_shortcut_facts().is_missing());
    assert!(
        union_status
            .result(PreparedMeshPairBoolean::Union)
            .is_current()
    );
    assert!(
        union_status
            .result(PreparedMeshPairBoolean::Intersection)
            .is_missing()
    );
    assert!(
        union_status
            .result(PreparedMeshPairBoolean::Difference)
            .is_missing()
    );
    assert!(
        union_status
            .result(PreparedMeshPairBoolean::Xor)
            .is_missing()
    );
    let union_outcome: PreparedMeshPairResultOutcome = pair
        .cache_status()
        .current_result_outcome(PreparedMeshPairBoolean::Union)
        .unwrap();
    assert!(union_outcome.is_mesh());
    assert_eq!(union_outcome.vertex_count(), Some(union.vertices().len()));
    assert_eq!(union_outcome.triangle_count(), Some(union.triangle_count()));
    assert_eq!(union_outcome.blocker_count(), None);
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Union)
            .unwrap(),
        union_outcome
    );
    assert_eq!(prepared_union_outcome, union_outcome);
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Union)
            .unwrap(),
        union_outcome
    );
    pair.cache_status()
        .result(PreparedMeshPairBoolean::Union)
        .require_current("union result")
        .unwrap();
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Union)
            .unwrap()
            .triangle_count(),
        union.triangle_count()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Intersection)
            .is_missing()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Difference)
            .is_missing()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Xor)
            .is_missing()
    );

    let repeated_union = pair.union().unwrap();
    repeated_union.validate_retained_state().unwrap();
    assert_eq!(repeated_union.triangle_count(), union.triangle_count());

    let prepared_intersection_outcome = pair
        .prepare_result(PreparedMeshPairBoolean::Intersection)
        .unwrap();
    assert!(prepared_intersection_outcome.is_mesh());
    let intersection = pair
        .current_result(PreparedMeshPairBoolean::Intersection)
        .unwrap();
    intersection.validate_retained_state().unwrap();
    assert_eq!(intersection.triangle_count(), 0);
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Union)
            .is_current()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Intersection)
            .is_current()
    );
    let intersection_outcome = pair
        .cache_status()
        .current_result_outcome(PreparedMeshPairBoolean::Intersection)
        .unwrap();
    assert!(intersection_outcome.is_mesh());
    assert_eq!(prepared_intersection_outcome, intersection_outcome);
    assert_eq!(
        intersection_outcome.triangle_count(),
        Some(intersection.triangle_count())
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Intersection)
            .unwrap(),
        intersection_outcome
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Intersection)
            .unwrap(),
        intersection_outcome
    );
    pair.cache_status()
        .result(PreparedMeshPairBoolean::Intersection)
        .require_current("intersection result")
        .unwrap();
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Intersection)
            .unwrap()
            .triangle_count(),
        intersection.triangle_count()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Difference)
            .is_missing()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Xor)
            .is_missing()
    );

    let prepared_difference_outcome = pair
        .prepare_result(PreparedMeshPairBoolean::Difference)
        .unwrap();
    assert!(prepared_difference_outcome.is_mesh());
    let difference = pair
        .current_result(PreparedMeshPairBoolean::Difference)
        .unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(difference.triangle_count(), 0);
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Difference)
            .is_current()
    );
    let difference_outcome = pair
        .cache_status()
        .current_result_outcome(PreparedMeshPairBoolean::Difference)
        .unwrap();
    assert!(difference_outcome.is_mesh());
    assert_eq!(prepared_difference_outcome, difference_outcome);
    assert_eq!(
        difference_outcome.triangle_count(),
        Some(difference.triangle_count())
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Difference)
            .unwrap(),
        difference_outcome
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Difference)
            .unwrap(),
        difference_outcome
    );
    pair.cache_status()
        .result(PreparedMeshPairBoolean::Difference)
        .require_current("difference result")
        .unwrap();
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Difference)
            .unwrap()
            .triangle_count(),
        difference.triangle_count()
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Xor)
            .is_missing()
    );

    let prepared_xor_outcome = pair.prepare_result(PreparedMeshPairBoolean::Xor).unwrap();
    assert!(prepared_xor_outcome.is_mesh());
    let xor = pair.current_result(PreparedMeshPairBoolean::Xor).unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(xor.triangle_count(), solid.triangle_count());
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Xor)
            .is_current()
    );
    let xor_outcome = pair
        .cache_status()
        .current_result_outcome(PreparedMeshPairBoolean::Xor)
        .unwrap();
    assert!(xor_outcome.is_mesh());
    assert_eq!(prepared_xor_outcome, xor_outcome);
    assert_eq!(xor_outcome.triangle_count(), Some(xor.triangle_count()));
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Xor)
            .unwrap(),
        xor_outcome
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Xor)
            .unwrap(),
        xor_outcome
    );
    pair.cache_status()
        .result(PreparedMeshPairBoolean::Xor)
        .require_current("xor result")
        .unwrap();
    assert_eq!(
        pair.current_result(PreparedMeshPairBoolean::Xor)
            .unwrap()
            .triangle_count(),
        xor.triangle_count()
    );

    let repeated_xor = pair.xor().unwrap();
    repeated_xor.validate_retained_state().unwrap();
    assert_eq!(repeated_xor.triangle_count(), xor.triangle_count());
}

#[test]
fn exact_mesh_borrowed_view_exposes_retained_facts() {
    let mesh = tetra([0, 0, 0]);
    let view: ExactMeshRef<'_> = mesh.view();
    let mesh_view: MeshView<'_> = view;

    view.validate_retained_state().unwrap();
    assert_eq!(mesh_view.vertices().len(), 4);
    assert_eq!(view.triangle_indices().len(), 4);
    assert_eq!(view.face_count(), 4);
    assert_eq!(view.edge_count(), 6);
    let source_stamp = view.source_stamp();
    assert_eq!(source_stamp.source(), MeshSource::Exact);
    assert_eq!(source_stamp.approximation(), ApproximationPolicy::ExactOnly);
    assert_ne!(source_stamp.source_identity(), 0);
    assert_eq!(source_stamp.construction_version(), 1);
    assert_eq!(source_stamp.vertex_count(), view.vertex_count());
    assert_eq!(source_stamp.edge_count(), view.edge_count());
    assert_eq!(source_stamp.face_count(), view.face_count());
    assert_eq!(view.mesh_bounds(), Some((&p(0, 0, 0), &p(1, 1, 1))));
    assert!(view.is_closed_manifold());
    assert_eq!(view.faces().count(), 4);
    assert_eq!(view.vertex_refs().count(), 4);
    assert_eq!(view.triangle_refs().count(), 4);
    assert_eq!(view.edges().count(), view.edge_count());

    assert_eq!(view.require_vertex(0).unwrap().index(), 0);
    let missing_vertex = view.require_vertex(view.vertex_count()).unwrap_err();
    assert_eq!(
        missing_vertex.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(
        missing_vertex.blockers()[0].vertex(),
        Some(view.vertex_count())
    );

    let vertex: VertexRef<'_> = view.vertex(0).unwrap();
    assert_eq!(vertex.index(), 0);
    assert_eq!(vertex.point(), &p(0, 0, 0));
    assert!(vertex.has_exact_rational_coordinates().unwrap());
    assert!(vertex.has_sparse_coordinate_support().unwrap());
    assert_eq!(vertex.incident_face_count().unwrap(), 3);
    assert_eq!(vertex.incident_edge_count().unwrap(), 3);
    assert_eq!(vertex.incident_face_indices().unwrap(), &[0, 1, 3]);
    assert_eq!(vertex.incident_edge_indices().unwrap(), &[0, 1, 2]);
    assert_eq!(
        vertex
            .incident_faces()
            .unwrap()
            .map(FaceRef::index)
            .collect::<Vec<_>>(),
        vec![0, 1, 3]
    );
    assert_eq!(
        vertex
            .incident_edges()
            .unwrap()
            .map(EdgeRef::index)
            .collect::<Vec<_>>(),
        vec![0, 1, 2]
    );
    assert!(vertex.has_circle_link().unwrap());
    assert!(!vertex.has_disk_link().unwrap());
    assert!(!vertex.has_isolated_link().unwrap());
    assert!(!vertex.has_non_manifold_link().unwrap());

    assert_eq!(view.require_face(0).unwrap().index(), 0);
    let missing_face = view.require_face(view.face_count()).unwrap_err();
    assert_eq!(
        missing_face.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(missing_face.blockers()[0].face(), Some(view.face_count()));

    let face: FaceRef<'_> = view.face(0).unwrap();
    assert_eq!(face.index(), 0);
    assert_eq!(face.vertex_indices(), [0, 2, 1]);
    assert_eq!(view.face_bounds(0), Some((&p(0, 0, 0), &p(1, 1, 0))));
    assert_eq!(
        view.require_face_bounds(0).unwrap(),
        (&p(0, 0, 0), &p(1, 1, 0))
    );
    assert_eq!(
        view.require_face_bounds(view.face_count())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(face.bounds().unwrap(), (&p(0, 0, 0), &p(1, 1, 0)));
    assert_eq!(
        face.vertex_refs().unwrap().map(VertexRef::index),
        face.vertex_indices()
    );
    assert_eq!(face.directed_edges().unwrap(), [[0, 2], [2, 1], [1, 0]]);
    assert!(face.is_non_degenerate().unwrap());
    assert!(!face.degeneracy_predicates().unwrap().is_empty());
    assert_eq!(
        face.plane_coefficients().unwrap(),
        (face.plane_normal().unwrap(), face.plane_offset().unwrap())
    );
    assert_eq!(face.vertices().unwrap().len(), 3);

    assert_eq!(view.require_triangle(1).unwrap().index(), 1);
    let missing_triangle = view.require_triangle(view.face_count()).unwrap_err();
    assert_eq!(
        missing_triangle.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(
        missing_triangle.blockers()[0].face(),
        Some(view.face_count())
    );

    let triangle: TriangleRef<'_> = view.triangle(1).unwrap();
    assert_eq!(triangle.index(), 1);
    assert_eq!(triangle.vertex_indices(), [0, 1, 3]);
    assert_eq!(triangle.bounds().unwrap(), (&p(0, 0, 0), &p(1, 0, 1)));
    assert_eq!(
        triangle.vertex_refs().unwrap().map(VertexRef::index),
        triangle.vertex_indices()
    );
    assert_eq!(triangle.directed_edges().unwrap(), [[0, 1], [1, 3], [3, 0]]);
    assert!(triangle.is_non_degenerate().unwrap());
    assert!(!triangle.degeneracy_predicates().unwrap().is_empty());
    assert_eq!(
        triangle.plane_coefficients().unwrap(),
        (
            triangle.plane_normal().unwrap(),
            triangle.plane_offset().unwrap()
        )
    );

    assert_eq!(view.require_edge(0).unwrap().index(), 0);
    let missing_edge = view.require_edge(view.edge_count()).unwrap_err();
    assert_eq!(
        missing_edge.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(missing_edge.blockers()[0].edge(), None);

    let edge: EdgeRef<'_> = view.edge(0).unwrap();
    assert_eq!(edge.index(), 0);
    assert_eq!(view.edge_bounds(0), Some((&p(0, 0, 0), &p(1, 0, 0))));
    assert_eq!(
        view.require_edge_bounds(0).unwrap(),
        (&p(0, 0, 0), &p(1, 0, 0))
    );
    let missing_edge_bounds = view.require_edge_bounds(view.edge_count()).unwrap_err();
    assert_eq!(
        missing_edge_bounds.blockers()[0].kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(missing_edge_bounds.blockers()[0].edge(), None);
    assert_eq!(edge.incident_face_count(), 2);
    assert_eq!(edge.directed_use_counts(), [1, 1]);
    assert!(edge.is_closed_manifold_edge());
    assert_eq!(
        edge.vertex_refs().unwrap().map(VertexRef::index),
        edge.vertex_indices()
    );
    assert_eq!(edge.bounds().unwrap(), (&p(0, 0, 0), &p(1, 0, 0)));
    assert_eq!(edge.vertices().unwrap().len(), 2);

    let prepared = view.prepare_broad_phase().unwrap();
    assert_eq!(prepared.mesh_bounds(), view.mesh_bounds());
}

#[test]
fn exact_mesh_source_stamp_distinguishes_source_provenance() {
    let vertices = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(0, 0, 1)];
    let triangles = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
    let left = ExactMesh::new(
        vertices.clone(),
        triangles.clone(),
        SourceProvenance::exact("source identity left"),
    )
    .unwrap();
    let right = ExactMesh::new(
        vertices,
        triangles,
        SourceProvenance::exact("source identity right"),
    )
    .unwrap();

    let left_stamp = left.view().source_stamp();
    let right_stamp = right.view().source_stamp();
    assert_eq!(left_stamp.source(), right_stamp.source());
    assert_eq!(left_stamp.approximation(), right_stamp.approximation());
    assert_eq!(
        left_stamp.construction_version(),
        right_stamp.construction_version()
    );
    assert_eq!(left_stamp.vertex_count(), right_stamp.vertex_count());
    assert_eq!(left_stamp.edge_count(), right_stamp.edge_count());
    assert_eq!(left_stamp.face_count(), right_stamp.face_count());
    assert_ne!(left_stamp.source_identity(), right_stamp.source_identity());
}

#[test]
fn exact_mesh_source_stamp_distinguishes_same_label_geometry() {
    let triangles = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
    let left = ExactMesh::new(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(0, 0, 1)],
        triangles.clone(),
        SourceProvenance::exact("source identity shared"),
    )
    .unwrap();
    let right = ExactMesh::new(
        vec![p(0, 0, 0), p(2, 0, 0), p(0, 1, 0), p(0, 0, 1)],
        triangles,
        SourceProvenance::exact("source identity shared"),
    )
    .unwrap();

    let left_stamp = left.view().source_stamp();
    let right_stamp = right.view().source_stamp();
    assert_eq!(left_stamp.source(), right_stamp.source());
    assert_eq!(left_stamp.approximation(), right_stamp.approximation());
    assert_eq!(
        left_stamp.construction_version(),
        right_stamp.construction_version()
    );
    assert_eq!(left_stamp.vertex_count(), right_stamp.vertex_count());
    assert_eq!(left_stamp.edge_count(), right_stamp.edge_count());
    assert_eq!(left_stamp.face_count(), right_stamp.face_count());
    assert_ne!(left_stamp.source_identity(), right_stamp.source_identity());
}

#[test]
fn exact_mesh_borrowed_view_certifies_bounds_before_candidate_pairs() {
    let left = tetra([0, 0, 0]);
    let overlapping = tetra([0, 0, 0]);
    let disjoint = tetra([5, 0, 0]);

    left.view().validate_retained_bounds().unwrap();
    left.view().validate_retained_bounds_certificate().unwrap();
    let prepared_left: PreparedMeshView<'_> = left.view().prepare_broad_phase().unwrap();
    let prepared_overlapping: PreparedMeshView<'_> =
        overlapping.view().prepare_broad_phase().unwrap();
    let prepared_pair: PreparedMeshPair<'_, '_> = left
        .view()
        .prepare_broad_phase_pair(overlapping.view())
        .unwrap();
    assert!(prepared_pair.broad_phase_summary().plan().is_sweep());
    assert_eq!(
        prepared_pair.left().view().face_count(),
        left.triangle_count()
    );
    assert_eq!(
        prepared_pair.right().view().face_count(),
        overlapping.triangle_count()
    );
    assert!(
        prepared_pair
            .broad_phase_summary()
            .candidate_pair_capacity_hint()
            > 0
    );
    let broad_phase_summary: PreparedMeshPairBroadPhaseSummary =
        prepared_pair.broad_phase_summary();
    let _plan_kind: PreparedMeshPairPlanKind = broad_phase_summary.plan();
    let _sweep_axis: Option<PreparedMeshPairSweepAxis> = broad_phase_summary.sweep_axis();
    let _sweep_direction: Option<PreparedMeshPairSweepDirection> =
        broad_phase_summary.sweep_direction();
    let _sweep_active_set: Option<PreparedMeshPairSweepActiveSet> =
        broad_phase_summary.sweep_active_set();
    assert!(prepared_pair.cache_status().source_pair().is_current());
    assert_eq!(
        broad_phase_summary.left_source(),
        left.view().source_stamp()
    );
    assert_eq!(
        broad_phase_summary.right_source(),
        overlapping.view().source_stamp()
    );
    assert_eq!(broad_phase_summary.left_face_count(), left.triangle_count());
    assert_eq!(
        broad_phase_summary.right_face_count(),
        overlapping.triangle_count()
    );
    assert_eq!(
        broad_phase_summary.face_pair_product(),
        left.triangle_count() * overlapping.triangle_count()
    );
    assert_eq!(prepared_pair.broad_phase_summary(), broad_phase_summary);
    assert!(broad_phase_summary.candidate_pair_upper_bound() > 0);
    assert!(
        broad_phase_summary.candidate_pair_upper_bound() <= broad_phase_summary.face_pair_product()
    );
    if broad_phase_summary.plan().is_sweep() {
        assert!(broad_phase_summary.active_face_capacity_hint().is_some());
        assert!(broad_phase_summary.sweep_axis().is_some());
        assert!(broad_phase_summary.sweep_direction().is_some());
        assert!(broad_phase_summary.sweep_active_set().is_some());
    }
    assert!(
        prepared_pair
            .cache_status()
            .candidate_face_pairs()
            .is_missing()
    );
    assert!(
        prepared_pair
            .cache_status()
            .broad_phase_traversal()
            .is_missing()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_broad_phase_traversal_summary()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        prepared_pair
            .with_current_candidate_face_pairs(|pairs| pairs.len())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let count_only_summary: PreparedMeshPairBroadPhaseTraversalSummary =
        prepared_pair.prepare_broad_phase_traversal_summary();
    assert!(
        prepared_pair
            .cache_status()
            .broad_phase_traversal()
            .is_current()
    );
    assert!(
        prepared_pair
            .cache_status()
            .candidate_face_pairs()
            .is_missing()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_broad_phase_traversal_summary()
            .unwrap(),
        count_only_summary
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_broad_phase_traversal_summary()
            .unwrap()
            .candidate_pair_count(),
        count_only_summary.candidate_pair_count()
    );
    assert_eq!(
        prepared_pair
            .with_current_candidate_face_pairs(|pairs| pairs.len())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let retained_candidate_count = prepared_pair.prepare_candidate_face_pairs();
    assert_eq!(
        retained_candidate_count,
        count_only_summary.candidate_pair_count()
    );
    assert!(retained_candidate_count > 0);
    assert!(retained_candidate_count <= broad_phase_summary.candidate_pair_upper_bound());
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_broad_phase_traversal_summary()
            .unwrap()
            .candidate_pair_count(),
        retained_candidate_count
    );
    assert_eq!(
        prepared_pair
            .with_current_candidate_face_pairs(|pairs| {
                assert!(pairs.iter().all(|[left_face, right_face]| {
                    *left_face < left.triangle_count() && *right_face < overlapping.triangle_count()
                }));
                pairs.len()
            })
            .unwrap(),
        retained_candidate_count
    );
    assert!(
        prepared_pair
            .cache_status()
            .candidate_face_pairs()
            .is_current()
    );
    let traversal_summary: PreparedMeshPairBroadPhaseTraversalSummary = prepared_pair
        .cache_status()
        .current_broad_phase_traversal_summary()
        .unwrap();
    assert_eq!(traversal_summary.broad_phase_summary(), broad_phase_summary);
    assert_eq!(
        traversal_summary.candidate_pair_count(),
        retained_candidate_count
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_broad_phase_traversal_summary()
            .unwrap(),
        traversal_summary
    );
    assert_eq!(
        traversal_summary.broad_phase_rejection_count(),
        broad_phase_summary.face_pair_product() - traversal_summary.candidate_pair_count()
    );
    assert_eq!(
        traversal_summary.candidate_upper_bound_slack(),
        broad_phase_summary.candidate_pair_upper_bound() - traversal_summary.candidate_pair_count()
    );
    assert_eq!(
        traversal_summary.candidate_upper_bound_saturated(),
        traversal_summary.candidate_upper_bound_slack() == 0
    );
    prepared_pair
        .cache_status()
        .candidate_face_pairs()
        .require_current("broad-phase candidate face pairs")
        .unwrap();
    assert!(
        prepared_pair
            .cache_status()
            .face_pair_classifications()
            .is_missing()
    );
    assert!(
        prepared_pair
            .cache_status()
            .face_pair_classification_counts()
            .is_missing()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_face_pair_classification_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let streamed_graph_pair = left
        .view()
        .prepare_broad_phase_pair(overlapping.view())
        .unwrap();
    let streamed_graph_counts = streamed_graph_pair.prepare_intersection_graph().unwrap();
    assert!(
        streamed_graph_pair
            .cache_status()
            .broad_phase_traversal()
            .is_current()
    );
    assert!(
        streamed_graph_pair
            .cache_status()
            .candidate_face_pairs()
            .is_missing()
    );
    assert!(
        streamed_graph_pair
            .cache_status()
            .face_pair_classifications()
            .is_missing()
    );
    assert!(
        streamed_graph_pair
            .cache_status()
            .face_pair_classification_counts()
            .is_current()
    );
    let streamed_graph_classification_counts = streamed_graph_pair
        .cache_status()
        .current_face_pair_classification_counts()
        .unwrap();
    assert_eq!(
        streamed_graph_classification_counts.graph_required_count(),
        streamed_graph_counts.face_pair_count()
    );
    assert!(
        streamed_graph_pair
            .cache_status()
            .intersection_graph()
            .is_certificate_blocked()
    );
    let streamed_classification_pair = left
        .view()
        .prepare_broad_phase_pair(overlapping.view())
        .unwrap();
    let streamed_classification_counts =
        streamed_classification_pair.prepare_face_pair_classification_counts();
    assert!(
        streamed_classification_pair
            .cache_status()
            .broad_phase_traversal()
            .is_current()
    );
    assert!(
        streamed_classification_pair
            .cache_status()
            .candidate_face_pairs()
            .is_missing()
    );
    assert!(
        streamed_classification_pair
            .cache_status()
            .face_pair_classifications()
            .is_missing()
    );
    assert!(
        streamed_classification_pair
            .cache_status()
            .face_pair_classification_counts()
            .is_current()
    );
    assert_eq!(
        streamed_classification_pair
            .cache_status()
            .current_face_pair_classification_counts()
            .unwrap(),
        streamed_classification_counts
    );
    let classification_records_first_pair = left
        .view()
        .prepare_broad_phase_pair(overlapping.view())
        .unwrap();
    let classification_record_count =
        classification_records_first_pair.prepare_face_pair_classifications();
    assert!(
        classification_records_first_pair
            .cache_status()
            .face_pair_classifications()
            .is_current()
    );
    assert!(
        classification_records_first_pair
            .cache_status()
            .candidate_face_pairs()
            .is_missing()
    );
    assert_eq!(
        classification_records_first_pair.prepare_candidate_face_pairs(),
        classification_record_count
    );
    assert!(
        classification_records_first_pair
            .cache_status()
            .candidate_face_pairs()
            .is_current()
    );
    let classification_counts: PreparedMeshPairClassificationCounts =
        prepared_pair.prepare_face_pair_classification_counts();
    assert_eq!(
        classification_counts.face_pair_count(),
        retained_candidate_count
    );
    assert!(
        prepared_pair
            .cache_status()
            .face_pair_classifications()
            .is_missing()
    );
    assert!(
        prepared_pair
            .cache_status()
            .face_pair_classification_counts()
            .is_current()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_face_pair_classification_counts()
            .unwrap(),
        classification_counts
    );
    assert_eq!(
        prepared_pair.prepare_face_pair_classifications(),
        classification_counts.face_pair_count()
    );
    assert!(classification_counts.face_pair_count() > 0);
    assert!(
        classification_counts.face_pair_count() <= broad_phase_summary.candidate_pair_upper_bound()
    );
    assert!(classification_counts.graph_required_count() > 0);
    assert_eq!(
        classification_counts.graph_required_count(),
        classification_counts.coplanar_touching_count()
            + classification_counts.coplanar_overlapping_count()
            + classification_counts.candidate_count()
            + classification_counts.unknown_count()
    );
    assert_eq!(
        classification_counts.face_pair_count(),
        classification_counts.plane_separated_count()
            + classification_counts.graph_required_count()
    );
    assert!(
        prepared_pair
            .cache_status()
            .face_pair_classifications()
            .is_current()
    );
    assert!(
        prepared_pair
            .cache_status()
            .face_pair_classification_counts()
            .is_current()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_face_pair_classification_counts()
            .unwrap(),
        classification_counts
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_face_pair_classification_counts()
            .unwrap(),
        classification_counts
    );
    let retained_graph_counts: PreparedMeshPairIntersectionGraphCounts =
        prepared_pair.prepare_intersection_graph().unwrap();
    assert_eq!(
        retained_graph_counts.face_pair_count(),
        classification_counts.graph_required_count()
    );
    assert!(retained_graph_counts.event_count() > 0);
    assert!(!retained_graph_counts.has_unknowns());
    assert!(
        prepared_pair
            .cache_status()
            .intersection_graph()
            .is_certificate_blocked()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        prepared_pair.prepare_current_intersection_graph().unwrap(),
        retained_graph_counts
    );
    assert!(
        prepared_pair
            .cache_status()
            .intersection_graph()
            .is_current()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_intersection_graph_counts()
            .unwrap(),
        retained_graph_counts
    );

    assert_eq!(prepared_left.view().face_count(), left.triangle_count());
    assert_eq!(
        prepared_overlapping.view().face_count(),
        overlapping.triangle_count()
    );
    let mut candidates = Vec::new();
    prepared_left
        .try_visit_candidate_face_pairs(&prepared_overlapping, &mut |pair| {
            candidates.push(pair);
            Ok::<(), ()>(())
        })
        .unwrap();
    candidates.sort_unstable();
    assert!(!candidates.is_empty());
    assert!(candidates.iter().all(|[left_face, right_face]| {
        *left_face < left.triangle_count() && *right_face < overlapping.triangle_count()
    }));
    let mut classification_first_candidates = classification_records_first_pair
        .with_current_candidate_face_pairs(|pairs| pairs.to_vec())
        .unwrap();
    classification_first_candidates.sort_unstable();
    assert_eq!(classification_first_candidates, candidates);

    let prepared_disjoint = disjoint.view().prepare_broad_phase().unwrap();
    let disjoint_pair = left
        .view()
        .prepare_broad_phase_pair(disjoint.view())
        .unwrap();
    assert!(disjoint_pair.broad_phase_summary().plan().is_empty());
    assert_eq!(disjoint_pair.broad_phase_summary().sweep_axis(), None);
    assert_eq!(disjoint_pair.broad_phase_summary().sweep_direction(), None);
    assert_eq!(disjoint_pair.broad_phase_summary().sweep_active_set(), None);
    assert_eq!(
        disjoint_pair
            .broad_phase_summary()
            .candidate_pair_capacity_hint(),
        0
    );
    let mut disjoint_candidates = Vec::new();
    prepared_left
        .try_visit_candidate_face_pairs(&prepared_disjoint, &mut |pair| {
            disjoint_candidates.push(pair);
            Ok::<(), ()>(())
        })
        .unwrap();
    assert!(disjoint_candidates.is_empty());

    let mut direct_pair_candidates = Vec::new();
    left.view()
        .prepare_broad_phase_pair(overlapping.view())
        .unwrap()
        .try_visit_candidate_face_pairs(&mut |pair| {
            direct_pair_candidates.push(pair);
            Ok::<(), ()>(())
        })
        .unwrap();
    direct_pair_candidates.sort_unstable();
    assert_eq!(direct_pair_candidates, candidates);

    let mut prepared_pair_candidates = Vec::new();
    prepared_left
        .try_visit_candidate_face_pairs(&prepared_overlapping, &mut |pair| {
            prepared_pair_candidates.push(pair);
            Ok::<(), ()>(())
        })
        .unwrap();
    prepared_pair_candidates.sort_unstable();
    assert_eq!(prepared_pair_candidates, candidates);
    assert_eq!(direct_pair_candidates.len(), candidates.len());

    let mut owned_pair_candidates = Vec::new();
    prepared_pair
        .try_visit_candidate_face_pairs(&mut |pair| {
            owned_pair_candidates.push(pair);
            Ok::<(), ()>(())
        })
        .unwrap();
    owned_pair_candidates.sort_unstable();
    assert_eq!(owned_pair_candidates, candidates);

    let mut repeated_owned_pair_candidates = Vec::new();
    prepared_pair
        .try_visit_candidate_face_pairs(&mut |pair| {
            repeated_owned_pair_candidates.push(pair);
            Ok::<(), ()>(())
        })
        .unwrap();
    repeated_owned_pair_candidates.sort_unstable();
    assert_eq!(repeated_owned_pair_candidates, candidates);

    let mut outer_visits = 0;
    let mut inner_visits = 0;
    prepared_pair
        .try_visit_candidate_face_pairs(&mut |_| {
            outer_visits += 1;
            prepared_pair.try_visit_candidate_face_pairs(&mut |_| {
                inner_visits += 1;
                Err("inner stop")
            })?;
            Err("outer stop")
        })
        .unwrap_err();
    assert_eq!(outer_visits, 1);
    assert_eq!(inner_visits, 1);

    let mut reentrant_outer_visits = 0;
    let mut reentrant_inner_visits = 0;
    prepared_pair
        .try_visit_candidate_face_pairs(&mut |_| {
            reentrant_outer_visits += 1;
            prepared_pair
                .try_visit_candidate_face_pairs(&mut |_| {
                    reentrant_inner_visits += 1;
                    Err("inner stop")
                })
                .unwrap_err();
            Ok::<(), ()>(())
        })
        .unwrap();
    assert_eq!(reentrant_outer_visits, candidates.len());
    assert_eq!(reentrant_inner_visits, candidates.len());
}

#[test]
fn prepared_broad_phase_candidate_visitor_can_stop_early() {
    let left = tetra([0, 0, 0]);
    let right = tetra([0, 0, 0]);
    let prepared_left = left.view().prepare_broad_phase().unwrap();
    let prepared_right = right.view().prepare_broad_phase().unwrap();
    let prepared_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

    let mut visited = 0;
    let result = prepared_pair.try_visit_candidate_face_pairs(&mut |_| {
        visited += 1;
        Err("stop")
    });

    assert_eq!(result, Err("stop"));
    assert_eq!(visited, 1);

    visited = 0;
    let result = prepared_left.try_visit_candidate_face_pairs(&prepared_right, &mut |_| {
        visited += 1;
        Err("stop")
    });

    assert_eq!(result, Err("stop"));
    assert_eq!(visited, 1);
}

#[test]
fn prepared_pair_candidate_visitor_streams_without_storing_records() {
    let left = tetra([0, 0, 0]);
    let right = tetra([0, 0, 0]);
    let prepared_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

    let mut visited = 0usize;
    prepared_pair
        .try_visit_candidate_face_pairs(&mut |_| {
            visited += 1;
            Ok::<(), ()>(())
        })
        .unwrap();

    assert!(
        prepared_pair
            .cache_status()
            .broad_phase_traversal()
            .is_current()
    );
    assert!(
        prepared_pair
            .cache_status()
            .candidate_face_pairs()
            .is_missing()
    );
    assert_eq!(
        prepared_pair
            .cache_status()
            .current_broad_phase_traversal_summary()
            .unwrap()
            .candidate_pair_count(),
        visited
    );
    assert_eq!(
        prepared_pair
            .with_current_candidate_face_pairs(|pairs| pairs.len())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
}

#[test]
fn exact_arrangement_borrowed_view_exposes_retained_topology_counts() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);
    let direct_counts = left
        .with_arrangement_view(&right, |view: ArrangementView<'_>| {
            view.validate_retained_state().unwrap();
            assert!(view.is_complete());
            assert_eq!(view.vertices().count(), view.vertex_count());
            assert_eq!(view.edges().count(), view.edge_count());
            assert_eq!(view.face_cells().count(), view.face_cell_count());
            assert_eq!(view.blocker_count(), 0);
            if view.vertex_count() > 0 {
                assert_eq!(view.require_vertex(0).unwrap().index(), 0);
            }
            if view.edge_count() > 0 {
                assert_eq!(view.require_edge(0).unwrap().index(), 0);
            }
            if view.face_cell_count() > 0 {
                assert_eq!(view.require_face_cell(0).unwrap().index(), 0);
            }

            let missing_vertex = view.require_vertex(view.vertex_count()).unwrap_err();
            assert_eq!(
                missing_vertex.blockers()[0].kind(),
                ExactMeshBlockerKind::IndexOutOfBounds
            );
            assert_eq!(
                missing_vertex.blockers()[0].vertex(),
                Some(view.vertex_count())
            );
            let missing_edge = view.require_edge(view.edge_count()).unwrap_err();
            assert_eq!(
                missing_edge.blockers()[0].kind(),
                ExactMeshBlockerKind::IndexOutOfBounds
            );
            let missing_face_cell = view.require_face_cell(view.face_cell_count()).unwrap_err();
            assert_eq!(
                missing_face_cell.blockers()[0].kind(),
                ExactMeshBlockerKind::IndexOutOfBounds
            );

            if let Some(vertex) = view.vertex(0) {
                assert_eq!(vertex.index(), 0);
                assert!(vertex.provenance_count().unwrap() > 0);
                let _ = vertex.point().unwrap();
            }
            if let Some(edge) = view.edge(0) {
                assert_eq!(edge.index(), 0);
                assert_eq!(edge.vertices().unwrap().len(), 2);
            }
            if let Some(face_cell) = view.face_cell(0) {
                assert_eq!(face_cell.index(), 0);
                assert_eq!(
                    face_cell.boundary_node_count().unwrap(),
                    face_cell.boundary_point_count().unwrap()
                );
                assert_eq!(
                    face_cell.boundary_points().unwrap().count(),
                    face_cell.boundary_point_count().unwrap()
                );
            }

            (
                view.vertex_count(),
                view.edge_count(),
                view.face_cell_count(),
                view.region_count(),
                view.volume_region_count(),
                view.volume_adjacency_count(),
                view.lower_dimensional_artifact_count(),
                view.blocker_count(),
            )
        })
        .unwrap();

    let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
    assert!(pair.cache_status().intersection_graph().is_missing());
    assert!(pair.cache_status().arrangement().is_missing());
    assert_eq!(
        pair.cache_status()
            .current_arrangement_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.with_current_arrangement_view(|view: ArrangementView<'_>| view.vertex_count())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let prepared_arrangement_counts: PreparedMeshPairArrangementCounts =
        pair.prepare_arrangement().unwrap();
    assert!(prepared_arrangement_counts.is_complete());
    assert!(pair.cache_status().arrangement().is_current());
    assert_eq!(
        pair.cache_status().current_arrangement_counts().unwrap(),
        prepared_arrangement_counts
    );
    let prepared_counts = pair
        .with_arrangement_view(|view: ArrangementView<'_>| {
            view.validate_retained_state().unwrap();
            assert!(view.is_complete());
            (
                view.vertex_count(),
                view.edge_count(),
                view.face_cell_count(),
                view.region_count(),
                view.volume_region_count(),
                view.volume_adjacency_count(),
                view.lower_dimensional_artifact_count(),
                view.blocker_count(),
            )
        })
        .unwrap();
    assert_eq!(prepared_counts, direct_counts);
    let current_prepared_counts = pair
        .with_current_arrangement_view(|view: ArrangementView<'_>| {
            (
                view.vertex_count(),
                view.edge_count(),
                view.face_cell_count(),
                view.region_count(),
                view.volume_region_count(),
                view.volume_adjacency_count(),
                view.lower_dimensional_artifact_count(),
                view.blocker_count(),
            )
        })
        .unwrap();
    assert_eq!(current_prepared_counts, direct_counts);
    assert!(pair.cache_status().arrangement().is_current());
    let retained_arrangement_counts = pair.cache_status().current_arrangement_counts().unwrap();
    assert_eq!(retained_arrangement_counts, prepared_arrangement_counts);
    assert_eq!(
        (
            retained_arrangement_counts.vertex_count(),
            retained_arrangement_counts.edge_count(),
            retained_arrangement_counts.face_cell_count(),
            retained_arrangement_counts.region_count(),
            retained_arrangement_counts.volume_region_count(),
            retained_arrangement_counts.volume_adjacency_count(),
            retained_arrangement_counts.lower_dimensional_artifact_count(),
            retained_arrangement_counts.blocker_count(),
        ),
        direct_counts
    );
    assert!(retained_arrangement_counts.is_complete());
    let repeated_counts = pair
        .with_arrangement_view(|view: ArrangementView<'_>| {
            (
                view.vertex_count(),
                view.edge_count(),
                view.face_cell_count(),
                view.region_count(),
                view.volume_region_count(),
                view.volume_adjacency_count(),
                view.lower_dimensional_artifact_count(),
                view.blocker_count(),
            )
        })
        .unwrap();
    assert_eq!(repeated_counts, direct_counts);
    assert_eq!(
        pair.cache_status().current_arrangement_counts().unwrap(),
        retained_arrangement_counts
    );
    assert!(pair.cache_status().intersection_graph().is_current());
    let arrangement_graph_counts = pair
        .cache_status()
        .current_intersection_graph_counts()
        .unwrap();
    assert_eq!(arrangement_graph_counts.face_pair_count(), 0);
    assert_eq!(arrangement_graph_counts.event_count(), 0);
    assert!(!arrangement_graph_counts.has_unknowns());
}

#[test]
fn prepared_pair_named_boolean_preserves_retained_arrangement() {
    let left = tetra([0, 0, 0]);
    let right = tetra([1, 0, 0]);
    let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

    let arrangement_counts = pair.prepare_arrangement().unwrap();
    assert!(pair.cache_status().arrangement().is_current());

    let intersection_outcome = pair
        .prepare_result(PreparedMeshPairBoolean::Intersection)
        .unwrap();
    assert!(pair.cache_status().arrangement().is_current());
    assert_eq!(
        pair.cache_status().current_arrangement_counts().unwrap(),
        arrangement_counts
    );
    assert!(
        pair.cache_status()
            .result(PreparedMeshPairBoolean::Intersection)
            .is_current()
    );
    assert_eq!(
        pair.cache_status()
            .current_result_outcome(PreparedMeshPairBoolean::Intersection)
            .unwrap(),
        intersection_outcome
    );
}

#[test]
fn exact_mesh_transform_and_inverse_replay_retained_state() {
    let mesh = tetra([0, 0, 0]);
    let translated = mesh
        .transform([
            [Real::from(1), Real::from(0), Real::from(0), Real::from(2)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(-3)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(5)],
            [Real::from(0), Real::from(0), Real::from(0), Real::from(1)],
        ])
        .unwrap();

    translated.validate_retained_state().unwrap();
    assert_eq!(
        translated.view().source_stamp().construction_version(),
        mesh.view().source_stamp().construction_version() + 1
    );
    assert_eq!(translated.vertices()[0], p(2, -3, 5));
    assert_eq!(
        translated.triangle_indices().collect::<Vec<_>>(),
        mesh.triangle_indices().collect::<Vec<_>>()
    );

    let reflected = mesh
        .transform([
            [Real::from(-1), Real::from(0), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(0), Real::from(1)],
        ])
        .unwrap();

    reflected.validate_retained_state().unwrap();
    assert_eq!(
        reflected.view().source_stamp().construction_version(),
        mesh.view().source_stamp().construction_version() + 1
    );
    assert_eq!(reflected.triangle_indices().next(), Some([0, 1, 2]));

    let inverted = mesh.inverse().unwrap();
    inverted.validate_retained_state().unwrap();
    assert_eq!(
        inverted.view().source_stamp().construction_version(),
        mesh.view().source_stamp().construction_version() + 1
    );
    assert_eq!(inverted.vertices(), mesh.vertices());
    assert_eq!(inverted.triangle_indices().next(), Some([0, 1, 2]));
}

#[test]
fn exact_mesh_borrowed_view_transform_and_inverse_replay_retained_state() {
    let mesh = tetra([0, 0, 0]);

    let translated = mesh
        .view()
        .transform([
            [Real::from(1), Real::from(0), Real::from(0), Real::from(2)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(3)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(5)],
            [Real::from(0), Real::from(0), Real::from(0), Real::from(1)],
        ])
        .unwrap();
    translated.validate_retained_state().unwrap();
    assert_eq!(translated.vertices()[0], p(2, 3, 5));

    let shifted = mesh
        .view()
        .transform([
            [Real::from(1), Real::from(0), Real::from(0), Real::from(4)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(0), Real::from(1)],
        ])
        .unwrap();
    shifted.validate_retained_state().unwrap();
    assert_eq!(shifted.vertices()[0], p(4, 0, 0));

    let inverse = mesh.view().inverse().unwrap();
    inverse.validate_retained_state().unwrap();
    assert_eq!(inverse.triangle_indices().next(), Some([0, 1, 2]));
}

#[test]
fn exact_mesh_transform_accepts_homogeneous_affine_rows() {
    let mesh = tetra([0, 0, 0]);
    let transformed = mesh
        .transform([
            [Real::from(1), Real::from(0), Real::from(0), Real::from(4)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(5)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(6)],
            [Real::from(0), Real::from(0), Real::from(0), Real::from(1)],
        ])
        .unwrap();

    transformed.validate_retained_state().unwrap();
    assert_eq!(transformed.vertices()[0], p(4, 5, 6));
}

#[test]
fn exact_mesh_transform_rejects_non_affine_homogeneous_rows() {
    let mesh = tetra([0, 0, 0]);
    let error = mesh
        .transform([
            [Real::from(1), Real::from(0), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(1)],
        ])
        .unwrap_err();

    assert_eq!(
        error.blockers()[0].kind(),
        ExactMeshBlockerKind::UnsupportedExactOperation
    );
}

#[test]
fn exact_mesh_error_names_cover_kernel_blockers() {
    let error: ExactMeshError = ExactMesh::new(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)],
        vec![[0, 1, 3]],
        SourceProvenance::exact("invalid test mesh"),
    )
    .unwrap_err();
    let blocker: ExactMeshBlocker = error.blockers()[0].clone();

    assert_eq!(blocker.face(), Some(0));
    assert_eq!(blocker.vertex(), Some(3));
}
