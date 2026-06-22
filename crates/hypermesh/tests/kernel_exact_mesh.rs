use hyperlimit::{Point3, SourceProvenance};
use hypermesh::{ExactAffineTransform3, ExactMesh, ExactMeshBlocker, ExactMeshError, Triangle};
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
        vec![
            Triangle([0, 2, 1]),
            Triangle([0, 1, 3]),
            Triangle([1, 2, 3]),
            Triangle([2, 0, 3]),
        ],
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
    assert_eq!(union.triangles().len(), solid.triangles().len());

    let intersection = empty.intersection(&solid).unwrap();
    intersection.validate_retained_state().unwrap();
    assert!(intersection.triangles().is_empty());

    let difference = solid.difference(&empty).unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(difference.triangles().len(), solid.triangles().len());

    let xor = empty.xor(&solid).unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(xor.triangles().len(), solid.triangles().len());
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
    assert_eq!(union.triangles().len(), solid.triangles().len());

    let intersection = empty.view().intersection(solid.view()).unwrap();
    intersection.validate_retained_state().unwrap();
    assert!(intersection.triangles().is_empty());

    let difference = solid.view().difference(empty.view()).unwrap();
    difference.validate_retained_state().unwrap();
    assert_eq!(difference.triangles().len(), solid.triangles().len());

    let xor = empty.view().xor(solid.view()).unwrap();
    xor.validate_retained_state().unwrap();
    assert_eq!(xor.triangles().len(), solid.triangles().len());
}

#[test]
fn exact_mesh_borrowed_view_exposes_retained_facts() {
    let mesh = tetra([0, 0, 0]);
    let view = mesh.view();

    view.validate_retained_state().unwrap();
    assert_eq!(view.vertices().len(), 4);
    assert_eq!(view.triangles().len(), 4);
    assert_eq!(view.facts().mesh.face_count, 4);
    assert_eq!(view.faces().count(), 4);
    assert_eq!(view.triangle_refs().count(), 4);
    assert_eq!(view.edges().count(), mesh.facts().edges.len());

    let face = view.face(0).unwrap();
    assert_eq!(face.index(), 0);
    assert_eq!(face.triangle().0, [0, 2, 1]);
    assert_eq!(face.facts().triangle.face, 0);
    assert_eq!(face.plane(), &face.facts().plane);
    assert_eq!(face.vertices().len(), 3);

    let triangle = view.triangle(1).unwrap();
    assert_eq!(triangle.index(), 1);
    assert_eq!(triangle.facts().triangle.vertices, triangle.triangle().0);
    assert_eq!(triangle.plane(), &triangle.facts().plane);

    let edge = view.edge(0).unwrap();
    assert_eq!(edge.index(), 0);
    assert_eq!(edge.vertices().len(), 2);
}

#[test]
fn exact_mesh_borrowed_view_replays_bounds_before_candidate_pairs() {
    let left = tetra([0, 0, 0]);
    let overlapping = tetra([0, 0, 0]);
    let disjoint = tetra([5, 0, 0]);

    left.view().validate_retained_bounds().unwrap();
    let candidates = left
        .view()
        .candidate_face_pairs(overlapping.view())
        .unwrap();
    let mut visited_candidates = Vec::new();
    left.view()
        .visit_candidate_face_pairs(overlapping.view(), |pair| visited_candidates.push(pair))
        .unwrap();
    visited_candidates.sort_unstable();
    assert_eq!(visited_candidates, candidates);
    assert!(!candidates.is_empty());
    assert!(candidates.iter().all(|[left_face, right_face]| {
        *left_face < left.triangles().len() && *right_face < overlapping.triangles().len()
    }));

    let disjoint_candidates = left.view().candidate_face_pairs(disjoint.view()).unwrap();
    assert!(disjoint_candidates.is_empty());

    let prepared_left = left.view().prepare_broad_phase().unwrap();
    let prepared_overlapping = overlapping.view().prepare_broad_phase().unwrap();
    let prepared_pair = left
        .view()
        .prepare_pair_broad_phase(overlapping.view())
        .unwrap();
    assert!(
        prepared_left.candidate_face_pair_capacity_hint(&prepared_overlapping) >= candidates.len()
    );
    assert!(prepared_pair.candidate_face_pair_capacity_hint() >= candidates.len());
    assert_eq!(
        prepared_left.candidate_face_pairs(&prepared_overlapping),
        candidates
    );
    assert_eq!(prepared_pair.candidate_face_pairs(), candidates);
    let mut prepared_visited = Vec::new();
    prepared_pair.visit_candidate_face_pairs(|pair| prepared_visited.push(pair));
    prepared_visited.sort_unstable();
    assert_eq!(prepared_visited, candidates);

    let mut first_prepared = None;
    let stopped = prepared_pair.try_visit_candidate_face_pairs(|pair| {
        first_prepared = Some(pair);
        Err::<(), _>(())
    });
    assert_eq!(stopped, Err(()));
    assert!(first_prepared.is_some());
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
        .transform(&ExactAffineTransform3::translation(p(2, -3, 5)))
        .unwrap();

    translated.validate_retained_state().unwrap();
    assert_eq!(translated.vertices()[0], p(2, -3, 5));
    assert_eq!(translated.triangles(), mesh.triangles());

    let reflected = mesh
        .transform(&ExactAffineTransform3::from_rows(
            [
                [Real::from(-1), Real::from(0), Real::from(0)],
                [Real::from(0), Real::from(1), Real::from(0)],
                [Real::from(0), Real::from(0), Real::from(1)],
            ],
            [Real::from(0), Real::from(0), Real::from(0)],
        ))
        .unwrap();

    reflected.validate_retained_state().unwrap();
    assert_eq!(reflected.triangles()[0], Triangle([0, 1, 2]));

    let inverted = mesh.inverse().unwrap();
    inverted.validate_retained_state().unwrap();
    assert_eq!(inverted.vertices(), mesh.vertices());
    assert_eq!(inverted.triangles()[0], Triangle([0, 1, 2]));
}

#[test]
fn exact_mesh_borrowed_view_transform_and_inverse_replay_retained_state() {
    let mesh = tetra([0, 0, 0]);
    let transform = ExactAffineTransform3::translation(p(2, 3, 5));

    let translated = mesh.view().transform(&transform).unwrap();
    translated.validate_retained_state().unwrap();
    assert_eq!(translated.vertices()[0], p(2, 3, 5));

    let shifted = mesh
        .view()
        .transform_by([
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
    assert_eq!(inverse.triangles()[0].0, [0, 1, 2]);
}

#[test]
fn exact_mesh_transform_by_accepts_homogeneous_affine_rows() {
    let mesh = tetra([0, 0, 0]);
    let transformed = mesh
        .transform_by([
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
fn exact_mesh_transform_by_rejects_non_affine_homogeneous_rows() {
    let mesh = tetra([0, 0, 0]);
    let error = mesh
        .transform_by([
            [Real::from(1), Real::from(0), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(1), Real::from(0), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(0)],
            [Real::from(0), Real::from(0), Real::from(1), Real::from(1)],
        ])
        .unwrap_err();

    assert_eq!(
        error.blockers[0].kind,
        hypermesh::ExactMeshBlockerKind::UnsupportedExactOperation
    );
}

#[test]
fn exact_mesh_error_names_cover_kernel_blockers() {
    let error: ExactMeshError = ExactMesh::new(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)],
        vec![Triangle([0, 1, 3])],
        SourceProvenance::exact("invalid test mesh"),
    )
    .unwrap_err();
    let blocker: ExactMeshBlocker = error.blockers[0].clone();

    assert_eq!(blocker.face, Some(0));
    assert_eq!(blocker.vertex, Some(3));
}
