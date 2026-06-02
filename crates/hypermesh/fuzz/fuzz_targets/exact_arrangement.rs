#![no_main]

use hyperlimit::Point2;
use hypermesh::exact::{
    ExactArrangement, ExactArrangement2dRegion, ExactArrangement2dRegionRing,
    ExactArrangement2dSetOperation, ExactBooleanOperation, ExactMesh,
    ExactRegularizationPolicy, ValidationPolicy, boolean_exact, build_exact_arrangement2d_overlay,
};
use hyperreal::Real;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if data.len() < 12 {
        return;
    }

    let mut values = Vec::new();
    for chunk in data.chunks_exact(2).take(36) {
        let raw = i16::from_le_bytes(chunk.try_into().unwrap()) as i64;
        values.push(raw.rem_euclid(17) - 8);
    }

    exercise_planar_overlay(&values);
    exercise_mesh_arrangement(&values);
});

fn exercise_planar_overlay(values: &[i64]) {
    if values.len() < 8 {
        return;
    }
    let left = square_ring(
        ExactArrangement2dRegion::Left,
        values[0],
        values[1],
        1 + values[2].abs(),
    );
    let right = square_ring(
        ExactArrangement2dRegion::Right,
        values[3],
        values[4],
        1 + values[5].abs(),
    );
    for operation in [
        ExactArrangement2dSetOperation::Union,
        ExactArrangement2dSetOperation::Intersection,
        ExactArrangement2dSetOperation::Difference,
    ] {
        let overlay = build_exact_arrangement2d_overlay(&[left.clone(), right.clone()], operation);
        let _ = overlay.is_complete();
    }
}

fn exercise_mesh_arrangement(values: &[i64]) {
    if values.len() < 24 {
        return;
    }
    let Some(left) = tetrahedron_from_values(&values[0..12]) else {
        return;
    };
    let Some(right) = tetrahedron_from_values(&values[12..24]) else {
        return;
    };
    let Ok(arrangement) = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    ) else {
        return;
    };
    let _ = arrangement.validate_against_sources_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    );
    exercise_volume_graph_invariants(&arrangement);
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        if let Ok(selected) =
            arrangement.select_with_policy(operation, ExactRegularizationPolicy::RETAIN_ARTIFACTS)
        {
            if let Ok(simplified) =
                selected.simplify_exact_with_policy(ExactRegularizationPolicy::RETAIN_ARTIFACTS)
            {
                if simplified.blockers.is_empty() {
                    let _ = simplified.triangulate();
                }
            }
        }
        if let Ok(result) = boolean_exact(&left, &right, operation, ValidationPolicy::ALLOW_BOUNDARY)
        {
            let _ = result.validate();
        }
    }
}

fn exercise_volume_graph_invariants(arrangement: &ExactArrangement) {
    let Some(shells) = arrangement.shells_or_regions.as_ref() else {
        return;
    };
    if shells.is_empty() || shells.iter().any(|shell| !shell.closed || !shell.manifold) {
        assert!(arrangement.volume_regions.is_none());
        assert!(arrangement.volume_adjacencies.is_none());
        return;
    }
    let volume_regions = arrangement
        .volume_regions
        .as_ref()
        .expect("closed manifold shells must expose volume regions");
    let volume_adjacencies = arrangement
        .volume_adjacencies
        .as_ref()
        .expect("closed manifold shells must expose volume adjacencies");
    assert_eq!(volume_regions.len(), shells.len() + 1);
    assert_eq!(volume_adjacencies.len(), shells.len());
    assert!(volume_regions[0].exterior);
    for adjacency in volume_adjacencies {
        assert_eq!(adjacency.exterior_volume, 0);
        assert!(adjacency.interior_volume < volume_regions.len());
        assert_eq!(
            adjacency.separating_face_cells,
            shells[adjacency.shell_region].face_cells
        );
    }
}

fn square_ring(
    region: ExactArrangement2dRegion,
    x: i64,
    y: i64,
    size: i64,
) -> ExactArrangement2dRegionRing {
    ExactArrangement2dRegionRing::new(
        region,
        vec![
            point2(x, y),
            point2(x + size, y),
            point2(x + size, y + size),
            point2(x, y + size),
        ],
    )
}

fn point2(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn tetrahedron_from_values(values: &[i64]) -> Option<ExactMesh> {
    ExactMesh::from_i64_triangles_with_policy(
        values,
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}
