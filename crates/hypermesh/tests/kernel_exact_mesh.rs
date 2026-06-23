use hyperlimit::{Point3, SourceProvenance};
use hypermesh::{ExactMesh, ExactMeshBlocker, ExactMeshError};
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
fn exact_mesh_borrowed_view_exposes_retained_facts() {
    let mesh = tetra([0, 0, 0]);
    let view = mesh.view();

    view.validate_retained_state().unwrap();
    assert_eq!(view.vertices().len(), 4);
    assert_eq!(view.triangle_indices().len(), 4);
    assert_eq!(view.face_count(), 4);
    assert_eq!(view.edge_count(), 6);
    assert!(view.is_closed_manifold());
    assert_eq!(view.faces().count(), 4);
    assert_eq!(view.triangle_refs().count(), 4);
    assert_eq!(view.edges().count(), view.edge_count());

    let face = view.face(0).unwrap();
    assert_eq!(face.index(), 0);
    assert_eq!(face.vertex_indices(), [0, 2, 1]);
    assert_eq!(
        face.plane_coefficients(),
        (face.plane_normal(), face.plane_offset())
    );
    assert_eq!(face.vertices().len(), 3);

    let triangle = view.triangle(1).unwrap();
    assert_eq!(triangle.index(), 1);
    assert_eq!(triangle.vertex_indices(), [0, 1, 3]);
    assert_eq!(
        triangle.plane_coefficients(),
        (triangle.plane_normal(), triangle.plane_offset())
    );

    let edge = view.edge(0).unwrap();
    assert_eq!(edge.index(), 0);
    assert_eq!(edge.incident_face_count(), 2);
    assert_eq!(edge.directed_use_counts(), [1, 1]);
    assert!(edge.is_closed_manifold_edge());
    assert_eq!(edge.vertices().len(), 2);
}

#[test]
fn exact_mesh_borrowed_view_replays_bounds_before_candidate_pairs() {
    let left = tetra([0, 0, 0]);
    let overlapping = tetra([0, 0, 0]);
    let disjoint = tetra([5, 0, 0]);

    left.view().validate_retained_bounds().unwrap();
    let prepared_left = left.view().prepare_broad_phase().unwrap();
    let prepared_overlapping = overlapping.view().prepare_broad_phase().unwrap();
    let pair_view = prepared_left.pair_with(&prepared_overlapping);
    assert_eq!(pair_view.left().view().face_count(), left.triangle_count());
    assert_eq!(
        pair_view.right().view().face_count(),
        overlapping.triangle_count()
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
}

#[test]
fn prepared_broad_phase_candidate_visitor_can_stop_early() {
    let left = tetra([0, 0, 0]);
    let right = tetra([0, 0, 0]);
    let prepared_left = left.view().prepare_broad_phase().unwrap();
    let prepared_right = right.view().prepare_broad_phase().unwrap();

    let mut visited = 0;
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
    left.with_arrangement_view(&right, |view| {
        view.validate_retained_state().unwrap();
        assert_eq!(view.vertices().count(), view.vertex_count());
        assert_eq!(view.edges().count(), view.edge_count());
        assert_eq!(view.face_cells().count(), view.face_cell_count());
        assert_eq!(view.blocker_count(), 0);

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
    })
    .unwrap();
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
