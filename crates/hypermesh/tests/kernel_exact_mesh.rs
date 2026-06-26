use hyperlimit::{Point3, SourceProvenance};
use hypermesh::ExactMesh;
use hypermesh::kernel::{
    ArrangementView, EdgeRef, ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError, FaceRef,
    MeshView, PreparedMeshPair, TriangleRef, VertexRef,
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

fn vertices(mesh: &ExactMesh) -> &[Point3] {
    mesh.view().vertices()
}

fn triangle_count(mesh: &ExactMesh) -> usize {
    mesh.view().face_count()
}

fn triangle_indices(mesh: &ExactMesh) -> impl ExactSizeIterator<Item = [usize; 3]> + '_ {
    mesh.view().triangle_indices()
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
    assert_eq!(triangle_count(&union), triangle_count(&solid));

    let intersection = empty.intersection(&solid).unwrap();
    intersection.validate_retained_state().unwrap();
    assert_eq!(triangle_count(&intersection), 0);

    let difference = solid.difference(&empty).unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(triangle_count(&difference), triangle_count(&solid));

    let xor = empty.xor(&solid).unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(triangle_count(&xor), triangle_count(&solid));
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
    assert_eq!(triangle_count(&union), triangle_count(&solid));

    let intersection = empty.view().intersection(solid.view()).unwrap();
    intersection.validate_retained_state().unwrap();
    assert_eq!(triangle_count(&intersection), 0);

    let difference = solid.view().difference(empty.view()).unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(triangle_count(&difference), triangle_count(&solid));

    let xor = empty.view().xor(solid.view()).unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(triangle_count(&xor), triangle_count(&solid));
}

#[test]
fn prepared_mesh_pair_exposes_retained_pair_facts_without_named_policy() {
    let empty = ExactMesh::new(
        Vec::new(),
        Vec::new(),
        SourceProvenance::exact("empty test mesh"),
    )
    .unwrap();
    let solid = tetra([0, 0, 0]);
    let pair = empty.view().prepare_broad_phase_pair(solid.view()).unwrap();
    assert_eq!(
        pair.with_current_arrangement_view(|view| view.vertex_count())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    assert_eq!(
        pair.with_candidate_face_pairs(|pairs| pairs.len()).unwrap(),
        0
    );
    assert_eq!(
        pair.with_current_candidate_face_pairs(|pairs| pairs.len())
            .unwrap(),
        0
    );
}

#[test]
fn exact_mesh_borrowed_view_exposes_retained_facts() {
    let mesh = tetra([0, 0, 0]);
    let view: MeshView<'_> = mesh.view();
    let mesh_view: MeshView<'_> = view;

    view.validate_retained_state().unwrap();
    assert_eq!(mesh_view.vertices().len(), 4);
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

    view.validate_retained_bounds_certificate().unwrap();
    assert_eq!(view.mesh_bounds(), Some((&p(0, 0, 0), &p(1, 1, 1))));
}

#[test]
fn exact_mesh_borrowed_view_certifies_bounds_before_candidate_pairs() {
    let left = tetra([0, 0, 0]);
    let overlapping = tetra([0, 0, 0]);
    let disjoint = tetra([5, 0, 0]);

    left.view().validate_retained_bounds().unwrap();
    left.view().validate_retained_bounds_certificate().unwrap();
    let prepared_pair: PreparedMeshPair<'_, '_> = left
        .view()
        .prepare_broad_phase_pair(overlapping.view())
        .unwrap();
    assert_eq!(
        prepared_pair
            .with_current_candidate_face_pairs(|pairs| pairs.len())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
    );
    let retained_candidate_count = prepared_pair
        .with_candidate_face_pairs(|pairs| pairs.len())
        .unwrap();
    assert!(retained_candidate_count > 0);
    assert_eq!(
        prepared_pair
            .with_current_candidate_face_pairs(|pairs| {
                assert!(pairs.iter().all(|[left_face, right_face]| {
                    *left_face < triangle_count(&left) && *right_face < triangle_count(&overlapping)
                }));
                pairs.len()
            })
            .unwrap(),
        retained_candidate_count
    );
    let classification_records_first_pair = left
        .view()
        .prepare_broad_phase_pair(overlapping.view())
        .unwrap();
    let mut classification_first_candidates = classification_records_first_pair
        .with_candidate_face_pairs(|pairs| pairs.to_vec())
        .unwrap();
    classification_first_candidates.sort_unstable();
    assert!(retained_candidate_count > 0);
    prepared_pair
        .with_arrangement_view(|view| {
            view.validate_retained_state().unwrap();
        })
        .unwrap();

    assert_eq!(left.view().face_count(), triangle_count(&left));
    assert_eq!(
        overlapping.view().face_count(),
        triangle_count(&overlapping)
    );
    let candidates = classification_first_candidates.clone();
    assert!(!candidates.is_empty());
    assert!(candidates.iter().all(|[left_face, right_face]| {
        *left_face < triangle_count(&left) && *right_face < triangle_count(&overlapping)
    }));
    assert_eq!(classification_first_candidates, candidates);

    let mut disjoint_candidates = Vec::new();
    left.view()
        .prepare_broad_phase_pair(disjoint.view())
        .unwrap()
        .try_visit_candidate_face_pairs(&mut |pair| {
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
    let prepared_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

    let mut visited = 0;
    let result = prepared_pair.try_visit_candidate_face_pairs(&mut |_| {
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

    assert!(visited > 0);
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
    let direct_pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();
    let direct_counts = direct_pair
        .with_arrangement_view(|view: ArrangementView<'_>| {
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
    assert_eq!(
        pair.with_current_arrangement_view(|view: ArrangementView<'_>| view.vertex_count())
            .unwrap_err()
            .blockers()[0]
            .kind(),
        ExactMeshBlockerKind::MissingRequiredEvidence
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
}

#[test]
fn borrowed_named_boolean_leaves_prepared_pair_arrangement_current() {
    let left = tetra([0, 0, 0]);
    let right = tetra([1, 0, 0]);
    let pair = left.view().prepare_broad_phase_pair(right.view()).unwrap();

    pair.with_arrangement_view(|view| {
        view.validate_retained_state().unwrap();
    })
    .unwrap();

    let intersection = left.view().intersection(right.view()).unwrap();
    pair.with_current_arrangement_view(|view| {
        view.validate_retained_state().unwrap();
    })
    .unwrap();
    intersection.validate_retained_state().unwrap();
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
    assert_eq!(vertices(&translated)[0], p(2, -3, 5));
    assert_eq!(
        triangle_indices(&translated).collect::<Vec<_>>(),
        triangle_indices(&mesh).collect::<Vec<_>>()
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
    assert_eq!(triangle_indices(&reflected).next(), Some([0, 1, 2]));

    let inverted = mesh.inverse().unwrap();
    inverted.validate_retained_state().unwrap();
    assert_eq!(vertices(&inverted), vertices(&mesh));
    assert_eq!(triangle_indices(&inverted).next(), Some([0, 1, 2]));
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
    assert_eq!(vertices(&translated)[0], p(2, 3, 5));

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
    assert_eq!(vertices(&shifted)[0], p(4, 0, 0));

    let inverse = mesh.view().inverse().unwrap();
    inverse.validate_retained_state().unwrap();
    assert_eq!(triangle_indices(&inverse).next(), Some([0, 1, 2]));
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
    assert_eq!(vertices(&transformed)[0], p(4, 5, 6));
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
