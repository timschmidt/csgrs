use hyperlimit::{Point3, SourceProvenance};
use hypermesh::ExactMesh;
use hypermesh::kernel::{
    EdgeRef, ExactMeshBlocker, ExactMeshBlockerKind, ExactMeshError, FaceRef, MeshView,
    TriangleRef, VertexRef,
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
    mesh.view().triangles().map(TriangleRef::vertex_indices)
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
    union.view().validate_retained_state().unwrap();
    assert_eq!(triangle_count(&union), triangle_count(&solid));

    let intersection = empty.intersection(&solid).unwrap();
    intersection.view().validate_retained_state().unwrap();
    assert_eq!(triangle_count(&intersection), 0);

    let difference = solid.difference(&empty).unwrap();
    difference.view().validate_retained_state().unwrap();
    assert_eq!(triangle_count(&difference), triangle_count(&solid));

    let xor = empty.xor(&solid).unwrap();
    xor.view().validate_retained_state().unwrap();
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
    union.view().validate_retained_state().unwrap();
    assert_eq!(triangle_count(&union), triangle_count(&solid));

    let intersection = empty.view().intersection(solid.view()).unwrap();
    intersection.view().validate_retained_state().unwrap();
    assert_eq!(triangle_count(&intersection), 0);

    let difference = solid.view().difference(empty.view()).unwrap();
    difference.view().validate_retained_state().unwrap();
    assert_eq!(triangle_count(&difference), triangle_count(&solid));

    let xor = empty.view().xor(solid.view()).unwrap();
    xor.view().validate_retained_state().unwrap();
    assert_eq!(triangle_count(&xor), triangle_count(&solid));
}

#[test]
fn exact_mesh_borrowed_view_exposes_retained_facts() {
    let mesh = tetra([0, 0, 0]);
    let view: MeshView<'_> = mesh.view();
    let mesh_view: MeshView<'_> = view;

    view.validate_retained_state().unwrap();
    assert_eq!(mesh_view.vertices().len(), 4);
    assert_eq!(view.triangles().len(), 4);
    assert_eq!(view.face_count(), 4);
    assert_eq!(view.edge_count(), 6);
    assert_eq!(
        view.mesh_bounds().unwrap(),
        Some((&p(0, 0, 0), &p(1, 1, 1)))
    );
    assert!(view.is_closed_manifold());
    assert_eq!(view.faces().len(), 4);
    assert_eq!(view.vertex_refs().len(), 4);
    assert_eq!(view.edges().len(), view.edge_count());
    assert_eq!(
        view.triangles()
            .map(TriangleRef::vertex_indices)
            .collect::<Vec<_>>(),
        view.faces()
            .map(FaceRef::vertex_indices)
            .collect::<Vec<_>>()
    );
    assert_eq!(view.triangle(0).unwrap().index(), 0);
    assert_eq!(view.triangle(0).unwrap().vertex_indices(), [0, 2, 1]);
    assert_eq!(
        view.triangle(0)
            .unwrap()
            .vertex_refs()
            .unwrap()
            .map(VertexRef::index),
        [0, 2, 1]
    );
    assert_eq!(view.triangle(0).unwrap().vertices().unwrap().len(), 3);

    assert_eq!(view.vertex(0).unwrap().index(), 0);
    let missing_vertex = view.vertex(view.vertex_count()).unwrap_err();
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
    assert!(vertex.has_exact_rational_coordinates());
    assert!(vertex.has_sparse_coordinate_support());
    assert_eq!(vertex.incident_face_count(), 3);
    assert_eq!(vertex.incident_edge_count(), 3);
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
    assert!(vertex.has_circle_link());
    assert!(!vertex.has_disk_link());
    assert!(!vertex.has_isolated_link());
    assert!(!vertex.has_non_manifold_link());

    assert_eq!(view.face(0).unwrap().index(), 0);
    let missing_face = view.face(view.face_count()).unwrap_err();
    assert_eq!(
        missing_face.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(missing_face.blockers()[0].face(), Some(view.face_count()));

    let face: FaceRef<'_> = view.face(0).unwrap();
    assert_eq!(face.index(), 0);
    let triangle: TriangleRef<'_> = face.triangle();
    assert_eq!(triangle.index(), face.index());
    assert_eq!(triangle.vertex_indices(), face.vertex_indices());
    assert_eq!(face.vertex_indices(), [0, 2, 1]);
    assert_eq!(view.face_bounds(0).unwrap(), (&p(0, 0, 0), &p(1, 1, 0)));
    assert_eq!(
        view.face_bounds(view.face_count()).unwrap_err().blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(face.bounds().unwrap(), (&p(0, 0, 0), &p(1, 1, 0)));
    assert_eq!(
        face.vertex_refs().unwrap().map(VertexRef::index),
        face.vertex_indices()
    );
    assert_eq!(face.directed_edges(), [[0, 2], [2, 1], [1, 0]]);
    assert!(face.is_non_degenerate());
    assert!(!face.degeneracy_predicates().is_empty());
    assert_eq!(
        face.plane_coefficients(),
        (face.plane_normal(), face.plane_offset())
    );
    assert_eq!(face.vertices().unwrap().len(), 3);

    assert_eq!(view.face(1).unwrap().index(), 1);
    let missing_face_row = view.face(view.face_count()).unwrap_err();
    assert_eq!(
        missing_face_row.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(
        missing_face_row.blockers()[0].face(),
        Some(view.face_count())
    );

    let second_face: FaceRef<'_> = view.face(1).unwrap();
    assert_eq!(second_face.index(), 1);
    assert_eq!(second_face.vertex_indices(), [0, 1, 3]);
    assert_eq!(second_face.bounds().unwrap(), (&p(0, 0, 0), &p(1, 0, 1)));
    assert_eq!(
        second_face.vertex_refs().unwrap().map(VertexRef::index),
        second_face.vertex_indices()
    );
    assert_eq!(second_face.directed_edges(), [[0, 1], [1, 3], [3, 0]]);
    assert!(second_face.is_non_degenerate());
    assert!(!second_face.degeneracy_predicates().is_empty());
    assert_eq!(
        second_face.plane_coefficients(),
        (second_face.plane_normal(), second_face.plane_offset())
    );
    assert_eq!(
        second_face.vertices().unwrap(),
        [&p(0, 0, 0), &p(1, 0, 0), &p(0, 0, 1)]
    );

    assert_eq!(view.edge(0).unwrap().index(), 0);
    let missing_edge = view.edge(view.edge_count()).unwrap_err();
    assert_eq!(
        missing_edge.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
    );
    assert_eq!(missing_edge.blockers()[0].edge(), None);

    let edge: EdgeRef<'_> = view.edge(0).unwrap();
    assert_eq!(edge.index(), 0);
    assert_eq!(view.edge_bounds(0).unwrap(), (&p(0, 0, 0), &p(1, 0, 0)));
    let missing_edge_bounds = view.edge_bounds(view.edge_count()).unwrap_err();
    assert_eq!(
        missing_edge_bounds.blockers()[0].kind(),
        ExactMeshBlockerKind::IndexOutOfBounds
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
    assert_eq!(
        view.mesh_bounds().unwrap(),
        Some((&p(0, 0, 0), &p(1, 1, 1)))
    );
}

#[test]
fn exact_arrangement_borrowed_view_exposes_retained_topology_counts() {
    let left = tetra([0, 0, 0]);
    let right = tetra([3, 0, 0]);
    let direct_counts = left
        .view()
        .with_arrangement_view(right.view(), |view| {
            view.validate_retained_state().unwrap();
            assert!(view.is_complete());
            assert_eq!(view.vertices().count(), view.vertex_count());
            assert_eq!(view.edges().count(), view.edge_count());
            assert_eq!(view.face_cells().count(), view.face_cell_count());
            assert_eq!(view.vertices().len(), view.vertex_count());
            assert_eq!(view.edges().len(), view.edge_count());
            assert_eq!(view.face_cells().len(), view.face_cell_count());
            assert_eq!(view.blocker_count(), 0);
            if view.vertex_count() > 0 {
                assert_eq!(view.vertex(0).unwrap().index(), 0);
            }
            if view.edge_count() > 0 {
                assert_eq!(view.edge(0).unwrap().index(), 0);
            }
            if view.face_cell_count() > 0 {
                assert_eq!(view.face_cell(0).unwrap().index(), 0);
            }

            let missing_vertex = view.vertex(view.vertex_count()).unwrap_err();
            assert_eq!(
                missing_vertex.blockers()[0].kind(),
                ExactMeshBlockerKind::IndexOutOfBounds
            );
            assert_eq!(
                missing_vertex.blockers()[0].vertex(),
                Some(view.vertex_count())
            );
            let missing_edge = view.edge(view.edge_count()).unwrap_err();
            assert_eq!(
                missing_edge.blockers()[0].kind(),
                ExactMeshBlockerKind::IndexOutOfBounds
            );
            let missing_face_cell = view.face_cell(view.face_cell_count()).unwrap_err();
            assert_eq!(
                missing_face_cell.blockers()[0].kind(),
                ExactMeshBlockerKind::IndexOutOfBounds
            );

            if let Ok(vertex) = view.vertex(0) {
                assert_eq!(vertex.index(), 0);
                assert!(vertex.provenance_count() > 0);
                let _ = vertex.point();
            }
            if let Ok(edge) = view.edge(0) {
                assert_eq!(edge.index(), 0);
                assert_eq!(edge.vertices().len(), 2);
            }
            if let Ok(face_cell) = view.face_cell(0) {
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

    let repeated_counts = left
        .view()
        .with_arrangement_view(right.view(), |view| {
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

    translated.view().validate_retained_state().unwrap();
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

    reflected.view().validate_retained_state().unwrap();
    assert_eq!(triangle_indices(&reflected).next(), Some([0, 1, 2]));

    let inverted = mesh.inverse().unwrap();
    inverted.view().validate_retained_state().unwrap();
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
    translated.view().validate_retained_state().unwrap();
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
    shifted.view().validate_retained_state().unwrap();
    assert_eq!(vertices(&shifted)[0], p(4, 0, 0));

    let inverse = mesh.view().inverse().unwrap();
    inverse.view().validate_retained_state().unwrap();
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

    transformed.view().validate_retained_state().unwrap();
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
