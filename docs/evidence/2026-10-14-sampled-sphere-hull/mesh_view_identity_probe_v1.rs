//! Borrowed public slices must describe the geometry actually queried.
use csgrs::{Real, TriangleMesh, solid};
use hypermesh::{HypermeshError, MeshCertainty, MeshContext, PredicatePolicy, polygon_soup};

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

fn cube() -> TriangleMesh {
    let mesh = solid::cube(Real::one());
    TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec())
}

fn main() {
    let original = cube();
    original.axis_aligned_box_bounds(&STRICT).unwrap();
    let mut view = original.as_ref();
    // Same starting address, different slice length.
    view.triangles = &original.triangles[..original.triangles.len() - 1];
    let fresh = TriangleMesh::new(view.positions.to_vec(), view.triangles.to_vec());
    assert!(matches!(
        polygon_soup(&STRICT, &[fresh.as_ref()]),
        Err(HypermeshError::OpenInput { .. })
    ));
    println!(
        "control=borrowed_view_shortened_triangles correct={}",
        matches!(
            polygon_soup(&STRICT, &[view]),
            Err(HypermeshError::OpenInput { .. })
        )
    );

    for reorder in [false, true] {
        let original = cube();
        polygon_soup(&STRICT, &[original.as_ref()]).unwrap();
        let mut positions = original.positions.to_vec();
        let mut triangles = original.triangles.to_vec();
        let mut view = original.as_ref();
        if reorder {
            triangles.reverse();
            view.triangles = &triangles;
        } else {
            for point in &mut positions {
                point.x = &point.x + Real::from(10);
            }
            view.positions = &positions;
        }
        let fresh = TriangleMesh::new(view.positions.to_vec(), view.triangles.to_vec());
        let expected = polygon_soup(&STRICT, &[fresh.as_ref()]).unwrap();
        let actual = polygon_soup(&STRICT, &[view]).unwrap();
        assert_eq!(expected.certainty, MeshCertainty::Certified);
        println!(
            "control=borrowed_view_{} correct={}",
            if reorder {
                "reindexed_triangles"
            } else {
                "replaced_positions"
            },
            actual == expected
        );
    }

    let original = cube();
    let mut triangles = original.triangles.to_vec();
    triangles.reverse();
    let mut view = original.as_ref();
    view.triangles = &triangles;
    polygon_soup(&STRICT, &[view]).unwrap();
    let fresh = TriangleMesh::new(original.positions.to_vec(), original.triangles.to_vec());
    let expected = polygon_soup(&STRICT, &[fresh.as_ref()]).unwrap();
    assert_eq!(expected.certainty, MeshCertainty::Certified);
    println!(
        "control=edited_borrowed_view_cannot_publish_original_planes correct={}",
        polygon_soup(&STRICT, &[original.as_ref()]).unwrap() == expected
    );
}
