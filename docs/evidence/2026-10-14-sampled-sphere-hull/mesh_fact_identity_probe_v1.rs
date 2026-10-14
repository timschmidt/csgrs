//! Observe public mesh edits against fresh exact geometry, without changing production code.
use csgrs::{Real, TriangleMesh, solid};
use hypermesh::{HypermeshError, MeshCertainty, MeshContext, PredicatePolicy, polygon_soup};
use std::sync::Arc;

const STRICT: MeshContext = MeshContext::new(PredicatePolicy::STRICT);

fn cube() -> TriangleMesh {
    let original = solid::cube(Real::one());
    // Independent storage and facts for each control, including Arc::make_mut.
    TriangleMesh::new(original.positions.to_vec(), original.triangles.to_vec())
}

fn shift_positions(mesh: &mut TriangleMesh, in_place: bool) {
    if in_place {
        for point in Arc::make_mut(&mut mesh.positions) {
            point.x = &point.x + Real::from(10);
        }
    } else {
        mesh.positions = mesh
            .positions
            .iter()
            .cloned()
            .map(|mut point| {
                point.x = &point.x + Real::from(10);
                point
            })
            .collect::<Vec<_>>()
            .into();
    }
}

fn expected_bounds(mesh: &TriangleMesh, low: i32, high: i32) -> bool {
    let outcome = mesh.exact_bounds(&STRICT).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    let bounds = outcome.value.unwrap();
    bounds.mins.x == Real::from(low)
        && bounds.maxs.x == Real::from(high)
        && bounds.mins.y == Real::zero()
        && bounds.maxs.y == Real::one()
        && bounds.mins.z == Real::zero()
        && bounds.maxs.z == Real::one()
}

fn warm_validity(mesh: &TriangleMesh) {
    let outcome = mesh.axis_aligned_box_bounds(&STRICT).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);
    assert!(outcome.value.is_some());
}

fn open_is_rejected(mesh: &TriangleMesh) -> bool {
    matches!(
        polygon_soup(&STRICT, &[mesh.as_ref()]),
        Err(HypermeshError::OpenInput { .. })
    )
}

fn main() {
    for in_place in [false, true] {
        let mut mesh = cube();
        assert!(expected_bounds(&mesh, 0, 1));
        let pointer = Arc::as_ptr(&mesh.positions);
        shift_positions(&mut mesh, in_place);
        let fresh = TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec());
        assert!(expected_bounds(&fresh, 10, 11));
        println!(
            "control=bounds_after_{} correct={} source_pointer_changed={}",
            if in_place { "make_mut" } else { "replacement" },
            expected_bounds(&mesh, 10, 11),
            !std::ptr::eq(pointer, Arc::as_ptr(&mesh.positions))
        );
    }

    let original = cube();
    let mut edited = original.clone();
    shift_positions(&mut edited, false);
    assert!(expected_bounds(&edited, 10, 11));
    println!(
        "control=edited_clone_cannot_publish_original_bounds correct={}",
        expected_bounds(&original, 0, 1)
    );

    for in_place in [false, true] {
        let mut mesh = cube();
        warm_validity(&mesh);
        if in_place {
            // Reverse one face without changing the number of triangles.
            let face = &mut Arc::make_mut(&mut mesh.triangles)[0];
            std::mem::swap(&mut face.v1, &mut face.v2);
            let fresh = TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec());
            assert!(matches!(
                polygon_soup(&STRICT, &[fresh.as_ref()]),
                Err(HypermeshError::NonPwnInput { .. })
            ));
            println!(
                "control=validity_after_make_mut correct={}",
                matches!(
                    polygon_soup(&STRICT, &[mesh.as_ref()]),
                    Err(HypermeshError::NonPwnInput { .. })
                )
            );
        } else {
            mesh.triangles = mesh.triangles[1..].to_vec().into();
            let fresh = TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec());
            assert!(open_is_rejected(&fresh));
            println!(
                "control=validity_after_replacement correct={}",
                open_is_rejected(&mesh)
            );
            println!(
                "control=reversed_winding_cannot_transfer_stale_validity correct={}",
                open_is_rejected(&mesh.reversed_winding())
            );
            let subdivided = mesh
                .subdivide_triangles(std::num::NonZeroU32::new(1).unwrap())
                .unwrap();
            println!(
                "control=subdivision_cannot_transfer_stale_validity correct={}",
                open_is_rejected(&subdivided)
            );
        }
    }

    for reorder in [false, true] {
        let mut mesh = cube();
        polygon_soup(&STRICT, &[mesh.as_ref()]).unwrap();
        if reorder {
            Arc::make_mut(&mut mesh.triangles).reverse();
        } else {
            shift_positions(&mut mesh, false);
        }
        let fresh = TriangleMesh::new(mesh.positions.to_vec(), mesh.triangles.to_vec());
        let expected = polygon_soup(&STRICT, &[fresh.as_ref()]).unwrap();
        assert_eq!(expected.certainty, MeshCertainty::Certified);
        let actual = polygon_soup(&STRICT, &[mesh.as_ref()]).unwrap();
        assert_eq!(actual.certainty, MeshCertainty::Certified);
        println!(
            "control=source_planes_after_{} correct={}",
            if reorder { "reindex" } else { "translation" },
            actual.value.polygons == expected.value.polygons
        );
    }
}
