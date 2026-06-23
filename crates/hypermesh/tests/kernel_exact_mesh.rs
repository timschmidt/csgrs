use hyperlimit::{Point3, SourceProvenance};
use hypermesh::{
    ArrangementView, EdgeRef, ExactMesh, ExactMeshBlocker, ExactMeshError, ExactMeshRef, FaceRef,
    PreparedMeshPair, PreparedMeshPairCacheStatus, PreparedMeshPairFactState,
    PreparedMeshPairPlanKind, PreparedMeshPairSweepAxis, PreparedMeshPairSweepDirection,
    PreparedMeshPairView, PreparedMeshView, TriangleRef, VertexRef,
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
    let initial_status: PreparedMeshPairCacheStatus = pair.cache_status();
    assert_eq!(
        initial_status.candidate_pair_plan(),
        PreparedMeshPairPlanKind::Empty
    );
    assert_eq!(initial_status.candidate_pair_capacity_hint(), 0);
    let initial_broad_phase = initial_status.broad_phase_summary();
    assert_eq!(initial_broad_phase.plan(), PreparedMeshPairPlanKind::Empty);
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
    assert_eq!(
        initial_status.face_pair_classifications(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        initial_status.retained_face_pair_classification_count(),
        None
    );
    assert_eq!(
        initial_status.retained_face_pair_classification_counts(),
        None
    );
    assert_eq!(
        initial_status.intersection_graph(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        initial_status.retained_intersection_graph_face_pair_count(),
        None
    );
    assert_eq!(
        initial_status.retained_intersection_graph_event_count(),
        None
    );
    assert_eq!(initial_status.retained_intersection_graph_counts(), None);
    assert_eq!(
        initial_status.arrangement(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(initial_status.retained_arrangement_counts(), None);
    assert_eq!(
        initial_status.arrangement_shortcut_facts(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        initial_status
            .require_current_arrangement_shortcut_facts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status.union_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(initial_status.union_result_outcome(), None);
    assert_eq!(
        initial_status
            .require_current_union_result()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .current_union_result_outcome()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_union_result().unwrap_err().blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_union_result_outcome().unwrap_err().blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status.intersection_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(initial_status.intersection_result_outcome(), None);
    assert_eq!(
        initial_status
            .require_current_intersection_result()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .current_intersection_result_outcome()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_intersection_result().unwrap_err().blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_intersection_result_outcome()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status.difference_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(initial_status.difference_result_outcome(), None);
    assert_eq!(
        initial_status
            .require_current_difference_result()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .current_difference_result_outcome()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_difference_result().unwrap_err().blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_difference_result_outcome()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status.xor_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(initial_status.xor_result_outcome(), None);
    assert_eq!(
        initial_status
            .require_current_xor_result()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .current_xor_result_outcome()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_xor_result().unwrap_err().blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_xor_result_outcome().unwrap_err().blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .require_current_intersection_graph()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .require_current_face_pair_classifications()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        initial_status
            .current_face_pair_classification_count()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_face_pair_classification_count()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.current_arrangement_counts().unwrap_err().blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );

    assert_eq!(pair.prepare_face_pair_classifications(), 0);
    let classified_status = pair.cache_status();
    classified_status
        .require_current_face_pair_classifications()
        .unwrap();
    let empty_classification_counts = classified_status
        .current_face_pair_classification_counts()
        .unwrap();
    assert_eq!(
        classified_status
            .current_face_pair_classification_count()
            .unwrap(),
        0
    );
    assert_eq!(empty_classification_counts.face_pair_count(), 0);
    assert_eq!(empty_classification_counts.graph_required_count(), 0);
    assert_eq!(
        classified_status.retained_face_pair_classification_counts(),
        Some(empty_classification_counts)
    );
    assert_eq!(pair.current_face_pair_classification_count().unwrap(), 0);
    assert_eq!(
        classified_status.intersection_graph(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        classified_status.arrangement_shortcut_facts(),
        PreparedMeshPairFactState::Missing
    );

    pair.prepare_arrangement_shortcut_facts().unwrap();
    let shortcut_status = pair.cache_status();
    assert_eq!(
        shortcut_status.arrangement_shortcut_facts(),
        PreparedMeshPairFactState::Current
    );
    shortcut_status
        .require_current_arrangement_shortcut_facts()
        .unwrap();
    assert_eq!(
        shortcut_status.intersection_graph(),
        PreparedMeshPairFactState::Missing
    );

    let prepared_union_outcome = pair.prepare_union_result().unwrap();
    assert!(prepared_union_outcome.is_mesh());
    let union = pair.current_union_result().unwrap();
    union.validate_retained_state().unwrap();
    assert_eq!(union.triangle_count(), solid.triangle_count());
    let retained_status = pair.cache_status();
    assert_eq!(
        retained_status.face_pair_classifications(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        retained_status.retained_face_pair_classification_count(),
        Some(0)
    );
    assert_eq!(
        retained_status.retained_face_pair_classification_counts(),
        Some(empty_classification_counts)
    );
    assert_eq!(
        retained_status.intersection_graph(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        retained_status.retained_intersection_graph_face_pair_count(),
        Some(0)
    );
    assert_eq!(
        retained_status.retained_intersection_graph_event_count(),
        Some(0)
    );
    let empty_graph_counts = retained_status
        .retained_intersection_graph_counts()
        .unwrap();
    assert_eq!(empty_graph_counts.face_pair_count(), 0);
    assert_eq!(empty_graph_counts.event_count(), 0);
    assert_eq!(empty_graph_counts.coplanar_overlap_graph_count(), 0);
    assert!(!empty_graph_counts.has_unknowns());
    retained_status
        .require_current_intersection_graph()
        .unwrap();
    assert_eq!(
        retained_status.current_intersection_graph_counts().unwrap(),
        empty_graph_counts
    );
    assert_eq!(
        pair.current_intersection_graph_counts().unwrap(),
        empty_graph_counts
    );
    assert_eq!(
        retained_status.arrangement_shortcut_facts(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        retained_status.union_result(),
        PreparedMeshPairFactState::Current
    );
    let union_outcome = retained_status.union_result_outcome().unwrap();
    assert!(union_outcome.is_mesh());
    assert_eq!(union_outcome.vertex_count(), Some(union.vertices().len()));
    assert_eq!(union_outcome.triangle_count(), Some(union.triangle_count()));
    assert_eq!(union_outcome.blocker_count(), None);
    assert_eq!(
        retained_status.current_union_result_outcome().unwrap(),
        union_outcome
    );
    assert_eq!(prepared_union_outcome, union_outcome);
    assert_eq!(pair.current_union_result_outcome().unwrap(), union_outcome);
    retained_status.require_current_union_result().unwrap();
    assert_eq!(
        pair.current_union_result().unwrap().triangle_count(),
        union.triangle_count()
    );
    assert_eq!(
        retained_status.intersection_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(retained_status.intersection_result_outcome(), None);
    assert_eq!(
        retained_status.difference_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(retained_status.difference_result_outcome(), None);
    assert_eq!(
        retained_status.xor_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(retained_status.xor_result_outcome(), None);

    let repeated_union = pair.union().unwrap();
    repeated_union.validate_retained_state().unwrap();
    assert_eq!(repeated_union.triangle_count(), union.triangle_count());

    let prepared_intersection_outcome = pair.prepare_intersection_result().unwrap();
    assert!(prepared_intersection_outcome.is_mesh());
    let intersection = pair.current_intersection_result().unwrap();
    intersection.validate_retained_state().unwrap();
    assert_eq!(intersection.triangle_count(), 0);
    let intersection_status = pair.cache_status();
    assert_eq!(
        intersection_status.union_result(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        intersection_status.intersection_result(),
        PreparedMeshPairFactState::Current
    );
    let intersection_outcome = intersection_status.intersection_result_outcome().unwrap();
    assert!(intersection_outcome.is_mesh());
    assert_eq!(prepared_intersection_outcome, intersection_outcome);
    assert_eq!(
        intersection_outcome.triangle_count(),
        Some(intersection.triangle_count())
    );
    assert_eq!(
        intersection_status
            .current_intersection_result_outcome()
            .unwrap(),
        intersection_outcome
    );
    assert_eq!(
        pair.current_intersection_result_outcome().unwrap(),
        intersection_outcome
    );
    intersection_status
        .require_current_intersection_result()
        .unwrap();
    assert_eq!(
        pair.current_intersection_result().unwrap().triangle_count(),
        intersection.triangle_count()
    );
    assert_eq!(
        intersection_status.difference_result(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        intersection_status.xor_result(),
        PreparedMeshPairFactState::Missing
    );

    let prepared_difference_outcome = pair.prepare_difference_result().unwrap();
    assert!(prepared_difference_outcome.is_mesh());
    let difference = pair.current_difference_result().unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(difference.triangle_count(), 0);
    let difference_status = pair.cache_status();
    assert_eq!(
        difference_status.difference_result(),
        PreparedMeshPairFactState::Current
    );
    let difference_outcome = difference_status.difference_result_outcome().unwrap();
    assert!(difference_outcome.is_mesh());
    assert_eq!(prepared_difference_outcome, difference_outcome);
    assert_eq!(
        difference_outcome.triangle_count(),
        Some(difference.triangle_count())
    );
    assert_eq!(
        difference_status
            .current_difference_result_outcome()
            .unwrap(),
        difference_outcome
    );
    assert_eq!(
        pair.current_difference_result_outcome().unwrap(),
        difference_outcome
    );
    difference_status
        .require_current_difference_result()
        .unwrap();
    assert_eq!(
        pair.current_difference_result().unwrap().triangle_count(),
        difference.triangle_count()
    );
    assert_eq!(
        difference_status.xor_result(),
        PreparedMeshPairFactState::Missing
    );

    let prepared_xor_outcome = pair.prepare_xor_result().unwrap();
    assert!(prepared_xor_outcome.is_mesh());
    let xor = pair.current_xor_result().unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(xor.triangle_count(), solid.triangle_count());
    assert_eq!(
        pair.cache_status().xor_result(),
        PreparedMeshPairFactState::Current
    );
    let xor_outcome = pair.cache_status().xor_result_outcome().unwrap();
    assert!(xor_outcome.is_mesh());
    assert_eq!(prepared_xor_outcome, xor_outcome);
    assert_eq!(xor_outcome.triangle_count(), Some(xor.triangle_count()));
    assert_eq!(
        pair.cache_status().current_xor_result_outcome().unwrap(),
        xor_outcome
    );
    assert_eq!(pair.current_xor_result_outcome().unwrap(), xor_outcome);
    pair.cache_status().require_current_xor_result().unwrap();
    assert_eq!(
        pair.current_xor_result().unwrap().triangle_count(),
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

    view.validate_retained_state().unwrap();
    assert_eq!(view.vertices().len(), 4);
    assert_eq!(view.triangle_indices().len(), 4);
    assert_eq!(view.face_count(), 4);
    assert_eq!(view.edge_count(), 6);
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
        hypermesh::ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(
        missing_vertex.blockers()[0].vertex(),
        Some(view.vertex_count())
    );

    let vertex: VertexRef<'_> = view.vertex(0).unwrap();
    assert_eq!(vertex.index(), 0);
    assert_eq!(vertex.point(), &p(0, 0, 0));
    assert!(vertex.has_exact_rational_coordinates());
    assert!(vertex.has_sparse_coordinate_support());
    assert_eq!(vertex.incident_face_count(), 3);
    assert_eq!(vertex.incident_edge_count(), 3);
    assert!(vertex.has_circle_link());
    assert!(!vertex.has_disk_link());
    assert!(!vertex.has_isolated_link());
    assert!(!vertex.has_non_manifold_link());

    assert_eq!(view.require_face(0).unwrap().index(), 0);
    let missing_face = view.require_face(view.face_count()).unwrap_err();
    assert_eq!(
        missing_face.blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(missing_face.blockers()[0].face(), Some(view.face_count()));

    let face: FaceRef<'_> = view.face(0).unwrap();
    assert_eq!(face.index(), 0);
    assert_eq!(face.vertex_indices(), [0, 2, 1]);
    assert_eq!(view.face_bounds(0), Some((&p(0, 0, 0), &p(1, 1, 0))));
    assert_eq!(face.bounds(), (&p(0, 0, 0), &p(1, 1, 0)));
    assert_eq!(
        face.vertex_refs().map(VertexRef::index),
        face.vertex_indices()
    );
    assert_eq!(face.directed_edges(), [[0, 2], [2, 1], [1, 0]]);
    assert!(face.is_non_degenerate());
    assert!(!face.degeneracy_predicates().is_empty());
    assert_eq!(
        face.plane_coefficients(),
        (face.plane_normal(), face.plane_offset())
    );
    assert_eq!(face.vertices().len(), 3);

    assert_eq!(view.require_triangle(1).unwrap().index(), 1);
    let missing_triangle = view.require_triangle(view.face_count()).unwrap_err();
    assert_eq!(
        missing_triangle.blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(
        missing_triangle.blockers()[0].face(),
        Some(view.face_count())
    );

    let triangle: TriangleRef<'_> = view.triangle(1).unwrap();
    assert_eq!(triangle.index(), 1);
    assert_eq!(triangle.vertex_indices(), [0, 1, 3]);
    assert_eq!(triangle.bounds(), (&p(0, 0, 0), &p(1, 0, 1)));
    assert_eq!(
        triangle.vertex_refs().map(VertexRef::index),
        triangle.vertex_indices()
    );
    assert_eq!(triangle.directed_edges(), [[0, 1], [1, 3], [3, 0]]);
    assert!(triangle.is_non_degenerate());
    assert!(!triangle.degeneracy_predicates().is_empty());
    assert_eq!(
        triangle.plane_coefficients(),
        (triangle.plane_normal(), triangle.plane_offset())
    );

    assert_eq!(view.require_edge(0).unwrap().index(), 0);
    let missing_edge = view.require_edge(view.edge_count()).unwrap_err();
    assert_eq!(
        missing_edge.blockers()[0].kind(),
        hypermesh::ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(missing_edge.blockers()[0].edge(), None);

    let edge: EdgeRef<'_> = view.edge(0).unwrap();
    assert_eq!(edge.index(), 0);
    assert_eq!(edge.incident_face_count(), 2);
    assert_eq!(edge.directed_use_counts(), [1, 1]);
    assert!(edge.is_closed_manifold_edge());
    assert_eq!(
        edge.vertex_refs().map(VertexRef::index),
        edge.vertex_indices()
    );
    assert_eq!(edge.bounds(), (p(0, 0, 0), p(1, 0, 0)));
    assert_eq!(edge.vertices().len(), 2);

    let prepared = view.prepare_broad_phase().unwrap();
    assert_eq!(prepared.mesh_bounds(), view.mesh_bounds());
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
    assert_eq!(
        prepared_pair.cache_status().candidate_pair_plan(),
        PreparedMeshPairPlanKind::Sweep
    );
    assert_eq!(
        prepared_pair.left().view().face_count(),
        left.triangle_count()
    );
    assert_eq!(
        prepared_pair.right().view().face_count(),
        overlapping.triangle_count()
    );
    assert!(prepared_pair.candidate_face_pair_capacity_hint() > 0);
    let unprepared_status = prepared_pair.cache_status();
    let broad_phase_summary = unprepared_status.broad_phase_summary();
    assert_eq!(
        broad_phase_summary.plan(),
        unprepared_status.candidate_pair_plan()
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
    assert_eq!(
        broad_phase_summary.candidate_pair_capacity_hint(),
        prepared_pair.candidate_face_pair_capacity_hint()
    );
    assert_eq!(prepared_pair.broad_phase_summary(), broad_phase_summary);
    assert!(broad_phase_summary.candidate_pair_upper_bound() > 0);
    assert!(
        broad_phase_summary.candidate_pair_upper_bound() <= broad_phase_summary.face_pair_product()
    );
    if broad_phase_summary.plan() == PreparedMeshPairPlanKind::Sweep {
        assert!(broad_phase_summary.active_face_capacity_hint().is_some());
        assert!(matches!(
            broad_phase_summary.sweep_axis(),
            Some(
                PreparedMeshPairSweepAxis::X
                    | PreparedMeshPairSweepAxis::Y
                    | PreparedMeshPairSweepAxis::Z
            )
        ));
        assert!(matches!(
            broad_phase_summary.sweep_direction(),
            Some(
                PreparedMeshPairSweepDirection::LeftDriven
                    | PreparedMeshPairSweepDirection::RightDriven
            )
        ));
    }
    assert_eq!(
        unprepared_status.candidate_face_pairs(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(unprepared_status.retained_candidate_face_pair_count(), None);
    assert_eq!(
        unprepared_status
            .current_candidate_face_pair_count()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let retained_candidate_count = prepared_pair.prepare_candidate_face_pairs();
    assert!(retained_candidate_count > 0);
    assert!(retained_candidate_count <= broad_phase_summary.candidate_pair_upper_bound());
    assert_eq!(
        prepared_pair.current_candidate_face_pair_count().unwrap(),
        retained_candidate_count
    );
    let candidate_status = prepared_pair.cache_status();
    assert_eq!(
        candidate_status.candidate_face_pairs(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        candidate_status.retained_candidate_face_pair_count(),
        Some(retained_candidate_count)
    );
    assert_eq!(
        candidate_status.retained_broad_phase_rejection_count(),
        Some(broad_phase_summary.face_pair_product() - retained_candidate_count)
    );
    assert_eq!(
        candidate_status.retained_candidate_upper_bound_slack(),
        Some(broad_phase_summary.candidate_pair_upper_bound() - retained_candidate_count)
    );
    assert_eq!(
        candidate_status.retained_candidate_upper_bound_saturated(),
        Some(broad_phase_summary.candidate_pair_upper_bound() == retained_candidate_count)
    );
    candidate_status
        .require_current_candidate_face_pairs()
        .unwrap();
    assert_eq!(
        unprepared_status.face_pair_classifications(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        unprepared_status.retained_face_pair_classification_counts(),
        None
    );
    assert_eq!(
        unprepared_status
            .current_face_pair_classification_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let classification_counts = prepared_pair.prepare_face_pair_classification_counts();
    assert_eq!(
        classification_counts.face_pair_count(),
        retained_candidate_count
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
    let classified_status = prepared_pair.cache_status();
    assert_eq!(
        classified_status.face_pair_classifications(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        classified_status.retained_face_pair_classification_count(),
        Some(classification_counts.face_pair_count())
    );
    assert_eq!(
        classified_status.retained_face_pair_classification_counts(),
        Some(classification_counts)
    );
    assert_eq!(
        prepared_pair
            .current_face_pair_classification_counts()
            .unwrap(),
        classification_counts
    );
    let retained_graph_counts = prepared_pair.prepare_intersection_graph().unwrap();
    assert_eq!(
        retained_graph_counts.face_pair_count(),
        classification_counts.graph_required_count()
    );
    assert!(retained_graph_counts.event_count() > 0);
    assert!(!retained_graph_counts.has_unknowns());
    let graph_retained_status = prepared_pair.cache_status();
    assert_eq!(
        graph_retained_status.intersection_graph(),
        PreparedMeshPairFactState::CertificateBlocked
    );
    assert_eq!(
        graph_retained_status.retained_intersection_graph_face_pair_count(),
        Some(retained_graph_counts.face_pair_count())
    );
    assert_eq!(
        graph_retained_status.retained_intersection_graph_event_count(),
        Some(retained_graph_counts.event_count())
    );
    assert_eq!(
        graph_retained_status.retained_intersection_graph_counts(),
        Some(retained_graph_counts)
    );
    assert_eq!(
        prepared_pair
            .current_intersection_graph_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        prepared_pair.prepare_current_intersection_graph().unwrap(),
        retained_graph_counts
    );
    assert_eq!(
        prepared_pair.cache_status().intersection_graph(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        prepared_pair.current_intersection_graph_counts().unwrap(),
        retained_graph_counts
    );

    let pair_view: PreparedMeshPairView<'_, '_, '_> =
        prepared_left.pair_with(&prepared_overlapping);
    assert_eq!(pair_view.left().view().face_count(), left.triangle_count());
    assert_eq!(
        pair_view.right().view().face_count(),
        overlapping.triangle_count()
    );
    assert_eq!(pair_view.broad_phase_summary(), broad_phase_summary);
    assert_eq!(
        pair_view
            .broad_phase_summary()
            .candidate_pair_capacity_hint(),
        pair_view.candidate_face_pair_capacity_hint()
    );

    let mut candidates = Vec::new();
    pair_view.visit_candidate_face_pairs(&mut |pair| {
        candidates.push(pair);
    });
    candidates.sort_unstable();
    assert!(!candidates.is_empty());
    assert!(candidates.iter().all(|[left_face, right_face]| {
        *left_face < left.triangle_count() && *right_face < overlapping.triangle_count()
    }));

    let prepared_disjoint = disjoint.view().prepare_broad_phase().unwrap();
    let disjoint_pair = left
        .view()
        .prepare_broad_phase_pair(disjoint.view())
        .unwrap();
    assert_eq!(
        disjoint_pair.cache_status().candidate_pair_plan(),
        PreparedMeshPairPlanKind::Empty
    );
    assert_eq!(
        disjoint_pair
            .cache_status()
            .broad_phase_summary()
            .sweep_axis(),
        None
    );
    assert_eq!(
        disjoint_pair
            .cache_status()
            .broad_phase_summary()
            .sweep_direction(),
        None
    );
    assert_eq!(disjoint_pair.candidate_face_pair_capacity_hint(), 0);
    let mut disjoint_candidates = Vec::new();
    prepared_left.visit_candidate_face_pairs(&prepared_disjoint, &mut |pair| {
        disjoint_candidates.push(pair);
    });
    assert!(disjoint_candidates.is_empty());

    let mut direct_pair_candidates = Vec::new();
    left.view()
        .visit_candidate_face_pairs(overlapping.view(), &mut |pair| {
            direct_pair_candidates.push(pair);
        })
        .unwrap();
    direct_pair_candidates.sort_unstable();
    assert_eq!(direct_pair_candidates, candidates);

    let mut prepared_pair_candidates = Vec::new();
    prepared_left.visit_candidate_face_pairs(&prepared_overlapping, &mut |pair| {
        prepared_pair_candidates.push(pair);
    });
    prepared_pair_candidates.sort_unstable();
    assert_eq!(prepared_pair_candidates, candidates);
    assert_eq!(direct_pair_candidates.len(), candidates.len());

    let mut owned_pair_candidates = Vec::new();
    prepared_pair.visit_candidate_face_pairs(&mut |pair| {
        owned_pair_candidates.push(pair);
    });
    owned_pair_candidates.sort_unstable();
    assert_eq!(owned_pair_candidates, candidates);

    let mut repeated_owned_pair_candidates = Vec::new();
    prepared_pair.visit_candidate_face_pairs(&mut |pair| {
        repeated_owned_pair_candidates.push(pair);
    });
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
    prepared_pair.visit_candidate_face_pairs(&mut |_| {
        reentrant_outer_visits += 1;
        prepared_pair
            .try_visit_candidate_face_pairs(&mut |_| {
                reentrant_inner_visits += 1;
                Err("inner stop")
            })
            .unwrap_err();
    });
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
    let pair_view = prepared_left.pair_with(&prepared_right);
    let result = pair_view.try_visit_candidate_face_pairs(&mut |_| {
        visited += 1;
        Err("stop")
    });

    assert_eq!(result, Err("stop"));
    assert_eq!(visited, 1);
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
                hypermesh::ExactMeshBlockerKind::IndexOutOfBounds
            );
            assert_eq!(
                missing_vertex.blockers()[0].vertex(),
                Some(view.vertex_count())
            );
            let missing_edge = view.require_edge(view.edge_count()).unwrap_err();
            assert_eq!(
                missing_edge.blockers()[0].kind(),
                hypermesh::ExactMeshBlockerKind::IndexOutOfBounds
            );
            let missing_face_cell = view.require_face_cell(view.face_cell_count()).unwrap_err();
            assert_eq!(
                missing_face_cell.blockers()[0].kind(),
                hypermesh::ExactMeshBlockerKind::IndexOutOfBounds
            );

            if let Some(vertex) = view.vertex(0) {
                assert_eq!(vertex.index(), 0);
                assert!(vertex.provenance_count() > 0);
                let _ = vertex.point();
            }
            if let Some(edge) = view.edge(0) {
                assert_eq!(edge.index(), 0);
                assert_eq!(edge.vertices().len(), 2);
            }
            if let Some(face_cell) = view.face_cell(0) {
                assert_eq!(face_cell.index(), 0);
                assert_eq!(
                    face_cell.boundary_node_count(),
                    face_cell.boundary_point_count()
                );
                assert_eq!(
                    face_cell.boundary_points().count(),
                    face_cell.boundary_point_count()
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
    assert_eq!(
        pair.cache_status().intersection_graph(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        pair.cache_status().arrangement(),
        PreparedMeshPairFactState::Missing
    );
    assert_eq!(
        pair.cache_status()
            .current_arrangement_counts()
            .unwrap_err()
            .blockers()[0]
            .kind(),
        hypermesh::ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let prepared_arrangement_counts = pair.prepare_arrangement().unwrap();
    assert!(prepared_arrangement_counts.is_complete());
    assert_eq!(
        pair.cache_status().arrangement(),
        PreparedMeshPairFactState::Current
    );
    assert_eq!(
        pair.current_arrangement_counts().unwrap(),
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
    let prepared_status = pair.cache_status();
    assert_eq!(
        prepared_status.arrangement(),
        PreparedMeshPairFactState::Current
    );
    let retained_arrangement_counts = prepared_status.current_arrangement_counts().unwrap();
    assert_eq!(retained_arrangement_counts, prepared_arrangement_counts);
    assert_eq!(
        prepared_status.retained_arrangement_counts(),
        Some(retained_arrangement_counts)
    );
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
        pair.current_arrangement_counts().unwrap(),
        retained_arrangement_counts
    );
    assert_eq!(
        pair.cache_status().intersection_graph(),
        PreparedMeshPairFactState::Current
    );
    let arrangement_graph_counts = pair.current_intersection_graph_counts().unwrap();
    assert_eq!(arrangement_graph_counts.face_pair_count(), 0);
    assert_eq!(arrangement_graph_counts.event_count(), 0);
    assert!(!arrangement_graph_counts.has_unknowns());
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
    assert_eq!(reflected.triangle_indices().next(), Some([0, 1, 2]));

    let inverted = mesh.inverse().unwrap();
    inverted.validate_retained_state().unwrap();
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
        hypermesh::ExactMeshBlockerKind::UnsupportedExactOperation
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
