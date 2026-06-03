#![no_main]

use std::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};
use hypermesh::exact::surface::{
    arrange_coplanar_convex_surface_component_holed_difference,
    arrange_coplanar_convex_surface_difference, arrange_coplanar_convex_surface_holed_difference,
    arrange_coplanar_convex_surface_multi_difference,
    arrange_coplanar_convex_surface_multi_holed_difference,
    arrange_coplanar_surface_component_difference,
    arrange_coplanar_surface_component_holed_difference,
    arrange_coplanar_surface_cutter_hole_contact_difference,
    arrange_coplanar_surface_multi_difference, arrange_coplanar_surface_point_touch_difference,
    arrange_coplanar_surface_side_cutter_difference, arrange_single_triangle_coplanar_difference,
    arrange_single_triangle_coplanar_holed_difference, difference_single_triangle_coplanar_surfaces,
};
use hypermesh::exact::{
    ExactArrangement, ExactBooleanOperation, ExactBooleanResultKind, ExactBooleanShortcutKind,
    ExactBoundaryBooleanPolicy, ExactLabeledCellComplex, ExactMesh, ExactRegularizationPolicy,
    ValidationPolicy, boolean_exact,
    exact_arrangement_boolean_attempt_report, preflight_boolean_exact,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let b = |index: usize| byte(data, index + 1);
    let span = 12 + i64::from(b(0) % 12);
    let left = rect_mesh(0, 0, span, span);
    let strict_hole = rect_mesh(
        2 + i64::from(b(1) % 3),
        2 + i64::from(b(2) % 3),
        5 + i64::from(b(3) % 5),
        5 + i64::from(b(4) % 5),
    );
    let retained_holes = multi_rect_mesh(&[
        (
            2,
            2,
            4 + i64::from(b(5) % 3),
            4 + i64::from(b(6) % 3),
        ),
        (
            span - 5,
            span - 5,
            span - 2,
            span - 2 + i64::from(b(7) % 2),
        ),
    ]);
    let side_cutter = rect_mesh(-2, 3, 4 + i64::from(b(8) % 5), span - 3);
    let crossing_cutter = triangle_mesh([
        [0, 4 + i64::from(b(9) % 4), 0],
        [span / 2, span / 2, 0],
        [0, span - 2, 0],
    ]);
    let point_touch = triangle_mesh([[span, span, 0], [span + 2, span, 0], [span, span + 2, 0]]);
    let single_left = triangle_mesh([[0, 0, 0], [span, 0, 0], [0, span, 0]]);
    let single_right = triangle_mesh([[1, 1, 0], [span / 2, 1, 0], [1, span / 2, 0]]);

    match byte(data, 0) % 10 {
        0 => exercise_pair(&left, &strict_hole),
        1 => exercise_pair(&left, &retained_holes),
        2 => exercise_pair(&left, &side_cutter),
        3 => exercise_pair(&left, &crossing_cutter),
        4 => exercise_pair(&left, &point_touch),
        5 => exercise_pair(&single_left, &single_right),
        6 => {
            if let Some(holed_left) =
                arrange_coplanar_convex_surface_holed_difference(&left, &strict_hole)
            {
                exercise_pair(&holed_left.mesh, &side_cutter);
            }
        }
        7 => {
            if let Some(holed_left) =
                arrange_coplanar_convex_surface_holed_difference(&left, &strict_hole)
            {
                exercise_pair(&holed_left.mesh, &crossing_cutter);
            }
        }
        8 => {
            if let Some(multi_holed_left) =
                arrange_coplanar_convex_surface_multi_holed_difference(&left, &retained_holes)
            {
                exercise_pair(&multi_holed_left.mesh, &side_cutter);
            }
        }
        _ => {
            if let Some(multi_holed_left) =
                arrange_coplanar_convex_surface_multi_holed_difference(&left, &retained_holes)
            {
                exercise_pair(&multi_holed_left.mesh, &crossing_cutter);
            }
        }
    }
});

fn byte(data: &[u8], index: usize) -> u8 {
    data.get(index).copied().unwrap_or(0)
}

fn exercise_pair(left: &ExactMesh, right: &ExactMesh) {
    exercise_arrangement_pipeline(left, right);
    check_legacy(
        left,
        right,
        arrange_coplanar_convex_surface_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_convex_surface_multi_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_surface_component_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_surface_multi_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_surface_side_cutter_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_surface_cutter_hole_contact_difference(left, right)
            .map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_convex_surface_holed_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_convex_surface_multi_holed_difference(left, right)
            .map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_convex_surface_component_holed_difference(left, right)
            .map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_surface_component_holed_difference(left, right)
            .map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_coplanar_surface_point_touch_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        difference_single_triangle_coplanar_surfaces(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_single_triangle_coplanar_difference(left, right).map(|artifact| artifact.mesh),
    );
    check_legacy(
        left,
        right,
        arrange_single_triangle_coplanar_holed_difference(left, right).map(|artifact| artifact.mesh),
    );
}

fn exercise_arrangement_pipeline(left: &ExactMesh, right: &ExactMesh) {
    let Ok(arrangement) = ExactArrangement::from_meshes_with_policy(
        left,
        right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    ) else {
        return;
    };
    let _ = arrangement.validate_against_sources_with_policy(
        left,
        right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    );
    let Ok(labeled) = arrangement
        .clone()
        .label_regions(ExactRegularizationPolicy::RETAIN_ARTIFACTS)
    else {
        return;
    };
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        exercise_arrangement_operation(left, right, &labeled, operation);
        check_arrangement_attempt_shortcut_consistency(left, right, operation);
    }
}

fn exercise_arrangement_operation(
    left: &ExactMesh,
    right: &ExactMesh,
    labeled: &ExactLabeledCellComplex,
    operation: ExactBooleanOperation,
) {
    let Ok(selected) =
        labeled
            .clone()
            .select_with_policy(operation, ExactRegularizationPolicy::RETAIN_ARTIFACTS)
    else {
        return;
    };
    let _ = selected.validate_against_sources(
        left,
        right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    );
    if !selected.volume_adjacencies.is_empty() {
        assert!(
            selected
                .selected_face_orientations
                .iter()
                .all(|orientation| orientation.from_volume_adjacency)
        );
    }
    let Ok(simplified) =
        selected.simplify_exact_with_policy(ExactRegularizationPolicy::RETAIN_ARTIFACTS)
    else {
        return;
    };
    let _ = simplified.validate_against_sources(
        left,
        right,
        ExactRegularizationPolicy::RETAIN_ARTIFACTS,
    );
    if simplified.blockers.is_empty() {
        let Ok(mesh) = simplified.triangulate() else {
            return;
        };
        mesh.validate_retained_state().unwrap();
        if let Ok(result) = boolean_exact(left, right, operation, ValidationPolicy::ALLOW_BOUNDARY)
            && result.kind
                == (ExactBooleanResultKind::CertifiedShortcut {
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                })
        {
            assert_same_mesh_shape(&result.mesh, &mesh);
        }
    }
}

fn check_arrangement_attempt_shortcut_consistency(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) {
    let Ok(attempt) = exact_arrangement_boolean_attempt_report(
        left,
        right,
        operation,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    ) else {
        return;
    };
    let Some(shortcut) = attempt.materialized_shortcut else {
        return;
    };
    let Ok(result) = boolean_exact(left, right, operation, ValidationPolicy::ALLOW_BOUNDARY) else {
        return;
    };
    result.validate().unwrap();
    match shortcut {
        ExactBooleanShortcutKind::BoolMeshSplit => {
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut { shortcut }
            );
            assert_eq!(result.mesh.vertices().len(), attempt.output_vertices);
            assert_eq!(result.mesh.triangles().len(), attempt.output_triangles);
        }
        ExactBooleanShortcutKind::ArrangementCellComplex => {
            if result.kind
                == (ExactBooleanResultKind::CertifiedShortcut {
                    shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
                })
            {
                assert_eq!(result.mesh.vertices().len(), attempt.output_vertices);
                assert_eq!(result.mesh.triangles().len(), attempt.output_triangles);
            }
        }
        _ => {}
    }
}

fn check_legacy(left: &ExactMesh, right: &ExactMesh, legacy: Option<ExactMesh>) {
    let Some(legacy) = legacy else {
        return;
    };
    legacy.validate_retained_state().unwrap();

    let preflight = preflight_boolean_exact(left, right, ExactBooleanOperation::Difference)
        .expect("legacy-certified difference should preflight");
    preflight.validate().unwrap();
    preflight.validate_against_sources(left, right).unwrap();

    let result = boolean_exact(
        left,
        right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("legacy-certified difference should materialize");
    result.validate().unwrap();
    result
        .validate_operation_against_sources(
            left,
            right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    exact_arrangement_boolean_attempt_report(
        left,
        right,
        ExactBooleanOperation::Difference,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .expect("arrangement attempt report should build for exact surface pair");
    assert_meshes_have_same_shape(&result.mesh, &legacy);
}

fn rect_mesh(x0: i64, y0: i64, x1: i64, y1: i64) -> ExactMesh {
    multi_rect_mesh(&[(x0, y0, x1, y1)])
}

fn multi_rect_mesh(rects: &[(i64, i64, i64, i64)]) -> ExactMesh {
    let mut points = Vec::with_capacity(rects.len() * 4 * 3);
    let mut triangles = Vec::with_capacity(rects.len() * 6);
    for &(x0, y0, x1, y1) in rects {
        if x0 >= x1 || y0 >= y1 {
            continue;
        }
        let base = points.len() / 3;
        points.extend([x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y1, 0]);
        triangles.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &points,
        &triangles,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("fuzz rectangle fixture should import")
}

fn triangle_mesh(points: [[i64; 3]; 3]) -> ExactMesh {
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
        ],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("fuzz triangle fixture should import")
}

fn assert_same_mesh_shape(left: &ExactMesh, right: &ExactMesh) {
    assert_meshes_have_same_shape(left, right);
}

fn assert_meshes_have_same_shape(left: &ExactMesh, right: &ExactMesh) {
    assert!(
        exact_mesh_vertex_sets_match(left, right) && left.triangles().len() == right.triangles().len()
            || exact_mesh_boundary_edges_match(left, right),
        "meshes do not have the same exact vertex set or boundary"
    );
}

fn exact_mesh_vertex_sets_match(left: &ExactMesh, right: &ExactMesh) -> bool {
    left.vertices().len() == right.vertices().len()
        && left.vertices().iter().all(|left_point| {
            right
                .vertices()
                .iter()
                .any(|right_point| point3_exact_equal(left_point, right_point))
        })
        && right.vertices().iter().all(|right_point| {
            left.vertices()
                .iter()
                .any(|left_point| point3_exact_equal(left_point, right_point))
        })
}

#[derive(Clone)]
struct ExactBoundaryEdge {
    endpoints: [Point3; 2],
    count: usize,
}

fn exact_mesh_boundary_edges_match(left: &ExactMesh, right: &ExactMesh) -> bool {
    let Some(left_edges) = exact_mesh_boundary_edges(left) else {
        return false;
    };
    let Some(right_edges) = exact_mesh_boundary_edges(right) else {
        return false;
    };
    !left_edges.is_empty()
        && left_edges.len() == right_edges.len()
        && left_edges.iter().all(|left_edge| {
            right_edges.iter().any(|right_edge| {
                left_edge.count == right_edge.count
                    && point3_edge_exact_equal(&left_edge.endpoints, &right_edge.endpoints)
            })
        })
        && right_edges.iter().all(|right_edge| {
            left_edges.iter().any(|left_edge| {
                left_edge.count == right_edge.count
                    && point3_edge_exact_equal(&right_edge.endpoints, &left_edge.endpoints)
            })
        })
}

fn exact_mesh_boundary_edges(mesh: &ExactMesh) -> Option<Vec<ExactBoundaryEdge>> {
    let mut edges = Vec::<ExactBoundaryEdge>::new();
    for triangle in mesh.triangles() {
        for [start, end] in triangle_edges(triangle.0) {
            let edge = [
                mesh.vertices().get(start)?.clone(),
                mesh.vertices().get(end)?.clone(),
            ];
            if let Some(existing) = edges
                .iter_mut()
                .find(|existing| point3_edge_exact_equal(&existing.endpoints, &edge))
            {
                existing.count += 1;
            } else {
                edges.push(ExactBoundaryEdge {
                    endpoints: edge,
                    count: 1,
                });
            }
        }
    }
    if edges.iter().any(|edge| edge.count > 2) {
        return None;
    }
    Some(edges.into_iter().filter(|edge| edge.count == 1).collect())
}

fn triangle_edges(triangle: [usize; 3]) -> [[usize; 2]; 3] {
    [
        [triangle[0], triangle[1]],
        [triangle[1], triangle[2]],
        [triangle[2], triangle[0]],
    ]
}

fn point3_edge_exact_equal(left: &[Point3; 2], right: &[Point3; 2]) -> bool {
    point3_exact_equal(&left[0], &right[0]) && point3_exact_equal(&left[1], &right[1])
        || point3_exact_equal(&left[0], &right[1]) && point3_exact_equal(&left[1], &right[0])
}

fn point3_exact_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}
