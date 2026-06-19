#![no_main]

use std::cmp::Ordering;
use std::collections::BTreeSet;

use hyperlimit::{Point2, compare_reals};
use hypermesh::{
    ExactArrangement, ExactArrangement2dRegion, ExactArrangement2dRegionRing,
    ExactArrangement2dSetOperation, ExactBooleanOperation, ExactBooleanRequest,
    ExactBooleanWorkspace, ExactMesh, ExactRegularizationPolicy, ValidationPolicy,
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
    exercise_nested_shell_cavity();
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
        let overlay = ExactArrangement2dRegionRing::overlay(&[left.clone(), right.clone()], operation);
        if overlay.is_complete() {
            let mut assigned_holes = BTreeSet::new();
            for component in &overlay.output_components {
                assert!(component.outer_loop < overlay.output_loops.len());
                assert_eq!(
                    compare_reals(
                        &overlay.output_loops[component.outer_loop].signed_area_twice,
                        &Real::from(0),
                    )
                    .value(),
                    Some(Ordering::Greater)
                );
                for &hole_loop in &component.hole_loops {
                    assert!(hole_loop < overlay.output_loops.len());
                    assert!(assigned_holes.insert(hole_loop));
                    assert_eq!(
                        compare_reals(
                            &overlay.output_loops[hole_loop].signed_area_twice,
                            &Real::from(0),
                        )
                        .value(),
                        Some(Ordering::Less)
                    );
                }
            }
            let negative_loops = overlay
                .output_loops
                .iter()
                .filter(|loop_| {
                    compare_reals(&loop_.signed_area_twice, &Real::from(0)).value()
                        == Some(Ordering::Less)
                })
                .count();
            assert_eq!(assigned_holes.len(), negative_loops);
        }
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
                assert_eq!(
                    simplified.lower_dimensional_artifacts,
                    arrangement.lower_dimensional_artifacts
                );
                if simplified.blockers.is_empty() {
                    let _ = simplified.triangulate();
                }
            }
        }
        let request = ExactBooleanRequest::new(operation, ValidationPolicy::ALLOW_BOUNDARY);
        let mut workspace = ExactBooleanWorkspace::new(&left, &right);
        if let Ok(result) = workspace.materialize_ref(request) {
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
    assert_eq!(
        volume_regions
            .iter()
            .filter(|region| region.exterior)
            .count(),
        1
    );
    let mut volume_indices = BTreeSet::new();
    for (expected, region) in volume_regions.iter().enumerate() {
        assert_eq!(region.index, expected);
        assert!(volume_indices.insert(region.index));
        for &shell in &region.boundary_shells {
            assert!(shell < shells.len());
        }
    }
    for adjacency in volume_adjacencies {
        assert!(adjacency.shell_region < shells.len());
        assert!(adjacency.exterior_volume < volume_regions.len());
        assert!(adjacency.interior_volume < volume_regions.len());
        assert_ne!(adjacency.exterior_volume, adjacency.interior_volume);
        assert_eq!(
            adjacency.separating_face_cells,
            shells[adjacency.shell_region].face_cells
        );
    }
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        if let Ok(selected) = arrangement
            .clone()
            .select_with_policy(operation, ExactRegularizationPolicy::RETAIN_ARTIFACTS)
        {
            for selected_volume in selected.selected_volume_regions {
                assert!(selected_volume < volume_regions.len());
            }
        }
    }
}

fn exercise_nested_shell_cavity() {
    let Some(left) = tetrahedron_with_reversed_inner() else {
        return;
    };
    let Some(right) = tetrahedron_mesh(
        &[
            [30, 0, 0],
            [31, 0, 0],
            [30, 1, 0],
            [30, 0, 1],
        ],
        ValidationPolicy::CLOSED,
    ) else {
        return;
    };
    let Ok(arrangement) = ExactArrangement::from_meshes_with_policy(
        &left,
        &right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    ) else {
        return;
    };
    exercise_volume_graph_invariants(&arrangement);
    let Ok(labeled) = arrangement.label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
    else {
        return;
    };
    let empty_cavity = labeled
        .volume_regions
        .iter()
        .find(|region| !region.exterior && !region.in_left && !region.in_right)
        .map(|region| region.index);
    let Some(empty_cavity) = empty_cavity else {
        return;
    };
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Difference,
    ] {
        if let Ok(selected) = labeled.clone().select(operation) {
            assert!(!selected.selected_volume_regions.contains(&empty_cavity));
        }
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

fn tetrahedron_mesh(points: &[[i64; 3]; 4], policy: ValidationPolicy) -> Option<ExactMesh> {
    ExactMesh::from_i64_triangles_with_policy(
        &[
            points[0][0],
            points[0][1],
            points[0][2],
            points[1][0],
            points[1][1],
            points[1][2],
            points[2][0],
            points[2][1],
            points[2][2],
            points[3][0],
            points[3][1],
            points[3][2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
        policy,
    )
    .ok()
}

fn tetrahedron_with_reversed_inner() -> Option<ExactMesh> {
    let outer = [[0, 0, 0], [20, 0, 0], [0, 20, 0], [0, 0, 20]];
    let inner = [[1, 1, 1], [2, 1, 1], [1, 2, 1], [1, 1, 2]];
    let mut vertices = Vec::new();
    for point in outer.iter().chain(inner.iter()) {
        vertices.extend(point);
    }
    let shell_triangles = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
    let mut triangles = Vec::new();
    for tri in shell_triangles {
        triangles.extend([tri[0], tri[1], tri[2]]);
    }
    for tri in shell_triangles {
        triangles.extend([4 + tri[0], 4 + tri[2], 4 + tri[1]]);
    }
    ExactMesh::from_i64_triangles_with_policy(&vertices, &triangles, ValidationPolicy::CLOSED).ok()
}
