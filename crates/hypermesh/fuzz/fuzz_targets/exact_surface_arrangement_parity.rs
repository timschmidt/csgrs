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
    ExactBooleanOperation, ExactBooleanShortcutKind, ExactBoundaryBooleanPolicy, ExactMesh,
    ExactRegularizationPolicy, ValidationPolicy, boolean_exact,
    exact_arrangement_boolean_attempt_report, preflight_boolean_exact,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let span = 12 + i64::from(byte(data, 0) % 12);
    let left = rect_mesh(0, 0, span, span);
    let strict_hole = rect_mesh(
        2 + i64::from(byte(data, 1) % 3),
        2 + i64::from(byte(data, 2) % 3),
        5 + i64::from(byte(data, 3) % 5),
        5 + i64::from(byte(data, 4) % 5),
    );
    let retained_holes = multi_rect_mesh(&[
        (
            2,
            2,
            4 + i64::from(byte(data, 5) % 3),
            4 + i64::from(byte(data, 6) % 3),
        ),
        (
            span - 5,
            span - 5,
            span - 2,
            span - 2 + i64::from(byte(data, 7) % 2),
        ),
    ]);
    let side_cutter = rect_mesh(-2, 3, 4 + i64::from(byte(data, 8) % 5), span - 3);
    let crossing_cutter = triangle_mesh([
        [0, 4 + i64::from(byte(data, 9) % 4), 0],
        [span / 2, span / 2, 0],
        [0, span - 2, 0],
    ]);
    let point_touch = triangle_mesh([[span, span, 0], [span + 2, span, 0], [span, span + 2, 0]]);
    let single_left = triangle_mesh([[0, 0, 0], [span, 0, 0], [0, span, 0]]);
    let single_right = triangle_mesh([[1, 1, 0], [span / 2, 1, 0], [1, span / 2, 0]]);

    exercise_pair(&left, &strict_hole);
    exercise_pair(&left, &retained_holes);
    exercise_pair(&left, &side_cutter);
    exercise_pair(&left, &crossing_cutter);
    exercise_pair(&left, &point_touch);
    exercise_pair(&single_left, &single_right);

    if let Some(holed_left) = arrange_coplanar_convex_surface_holed_difference(&left, &strict_hole)
    {
        exercise_pair(&holed_left.mesh, &side_cutter);
        exercise_pair(&holed_left.mesh, &crossing_cutter);
    }
    if let Some(multi_holed_left) =
        arrange_coplanar_convex_surface_multi_holed_difference(&left, &retained_holes)
    {
        exercise_pair(&multi_holed_left.mesh, &side_cutter);
        exercise_pair(&multi_holed_left.mesh, &crossing_cutter);
    }
});

fn byte(data: &[u8], index: usize) -> u8 {
    data.get(index).copied().unwrap_or(0)
}

fn exercise_pair(left: &ExactMesh, right: &ExactMesh) {
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

    let attempt = exact_arrangement_boolean_attempt_report(
        left,
        right,
        ExactBooleanOperation::Difference,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    )
    .expect("arrangement attempt report should build for exact surface pair");
    if attempt.materialized_shortcut != Some(ExactBooleanShortcutKind::ArrangementCellComplex) {
        assert_same_mesh_shape(&result.mesh, &legacy);
    }
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
    assert_eq!(left.triangles().len(), right.triangles().len());
    assert_eq!(left.vertices().len(), right.vertices().len());
    assert!(left.vertices().iter().all(|left_point| {
        right
            .vertices()
            .iter()
            .any(|right_point| point3_exact_equal(left_point, right_point))
    }));
    assert!(right.vertices().iter().all(|right_point| {
        left.vertices()
            .iter()
            .any(|left_point| point3_exact_equal(left_point, right_point))
    }));
}

fn point3_exact_equal(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}
