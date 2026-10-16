//! The original 26-box workload through the public modeling layer.

use csgrs::{
    GeometryCertainty, GeometryContext, Real, TriangleMesh,
    solid::{self, SolidExt},
};
use hypermesh::{MeshCertainty, Point3};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn boxes() -> Vec<TriangleMesh> {
    let mut boxes = Vec::new();
    for (x, a) in [(0, 2), (1, 3), (3, 2)] {
        for (y, b) in [(0, 2), (1, 3), (3, 2)] {
            for (z, c) in [(0, 2), (1, 3), (3, 2)] {
                if (x, y, z) != (1, 1, 1) {
                    boxes.push(
                        solid::cuboid(r(a), r(b), r(c))
                            .unwrap()
                            .translated(r(x), r(y), r(z))
                            .unwrap(),
                    );
                }
            }
        }
    }
    boxes
}

fn assert_cavity(mesh: &TriangleMesh, offset: [i32; 3]) {
    let context = GeometryContext::STRICT.mesh_context();
    assert_eq!(
        mesh.validate_closed_pwn(&context).unwrap().certainty,
        MeshCertainty::Certified
    );
    let six_volume = mesh.triangles.iter().fold(r(0), |sum, triangle| {
        let [a, b, c] = triangle
            .indices()
            .map(|index| mesh.positions[index].to_vector());
        sum + a.dot(&b.cross(&c))
    });
    // [0,5]^3 minus (2,3)^3 has exact volume 124, invariant under translation.
    assert_eq!(six_volume, r(744));
    for (twice, expected) in [([1, 1, 1], true), ([5, 5, 5], false), ([12, 1, 1], false)] {
        let point = std::array::from_fn::<_, 3, _>(|axis| {
            (r(twice[axis]) / r(2)).unwrap() + r(offset[axis])
        });
        let [x, y, z] = point;
        let outcome = mesh.contains_point(&context, &Point3::new(x, y, z)).unwrap();
        assert_eq!(outcome.certainty, MeshCertainty::Certified);
        assert_eq!(outcome.value, expected);
    }
}

#[test]
fn original_batch_shells_and_incremental_union_remain_consumable() {
    let context = GeometryContext::STRICT;
    let cells = boxes();
    let merged = solid::merge(&cells).unwrap();
    // Empty identity composition must validate and regularize all authored
    // shells, rather than concealing the formerly inconsistent arrangement.
    let batch = solid::empty()
        .try_union_with_context(&merged, &context)
        .unwrap();
    assert_eq!(batch.certainty, GeometryCertainty::Certified);
    assert_cavity(&batch.value, [0, 0, 0]);

    let mut accumulated = solid::empty();
    for cell in &cells {
        let outcome = accumulated.try_union_with_context(cell, &context).unwrap();
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        accumulated = outcome.value;
    }
    assert_cavity(&accumulated, [0, 0, 0]);

    let offset = [7, -2, 3];
    let translate = |mesh: &TriangleMesh| {
        mesh.translated(r(offset[0]), r(offset[1]), r(offset[2]))
            .unwrap()
    };
    assert_cavity(&translate(&batch.value), offset);
    let moved = cells.iter().map(translate).collect::<Vec<_>>();
    let moved = solid::merge(&moved).unwrap();
    let moved = solid::empty()
        .try_union_with_context(&moved, &context)
        .unwrap();
    assert_eq!(moved.certainty, GeometryCertainty::Certified);
    assert_cavity(&moved.value, offset);

    let removed = batch
        .value
        .try_difference_with_context(&batch.value, &context)
        .unwrap();
    assert_eq!(removed.certainty, GeometryCertainty::Certified);
    assert!(removed.value.positions.is_empty() && removed.value.triangles.is_empty());
    let restored = removed
        .value
        .try_union_with_context(&batch.value, &context)
        .unwrap();
    assert_eq!(restored.certainty, GeometryCertainty::Certified);
    assert_cavity(&restored.value, [0, 0, 0]);
}
