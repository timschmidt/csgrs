use hyperlimit::{Point3, SourceProvenance};
use hypermesh::{ExactMesh, ExactMeshBlocker, ExactMeshError, Triangle};
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
}

#[test]
fn exact_mesh_borrowed_view_exposes_retained_facts() {
    let mesh = tetra([0, 0, 0]);
    let view = mesh.view();

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
    assert_eq!(face.vertices().len(), 3);

    let triangle = view.triangle(1).unwrap();
    assert_eq!(triangle.index(), 1);
    assert_eq!(triangle.facts().triangle.vertices, triangle.triangle().0);

    let edge = view.edge(0).unwrap();
    assert_eq!(edge.index(), 0);
    assert_eq!(edge.vertices().len(), 2);
}

#[test]
fn exact_mesh_error_names_cover_kernel_diagnostics() {
    let error: ExactMeshError = ExactMesh::new(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)],
        vec![Triangle([0, 1, 3])],
        SourceProvenance::exact("invalid test mesh"),
    )
    .unwrap_err();
    let blocker: ExactMeshBlocker = error.diagnostics[0].clone();

    assert_eq!(blocker.face, Some(0));
    assert_eq!(blocker.vertex, Some(3));
}
