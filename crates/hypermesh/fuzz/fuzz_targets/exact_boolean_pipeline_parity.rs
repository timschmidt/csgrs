#![no_main]

use std::cmp::Ordering;

use hyperlimit::{Point3, compare_reals};
use hypermesh::exact::surface::{
    arrange_coplanar_convex_surface_component_union, arrange_coplanar_convex_surface_holed_difference,
    arrange_coplanar_convex_surface_intersection, arrange_coplanar_convex_surface_multi_intersection,
    arrange_coplanar_convex_surface_multi_union, arrange_coplanar_convex_surface_union,
    arrange_coplanar_surface_component_holed_intersection,
    arrange_coplanar_surface_component_holed_union, arrange_coplanar_surface_component_intersection,
    arrange_coplanar_surface_component_union, arrange_coplanar_surface_multi_component_intersection,
    arrange_coplanar_surface_multi_component_union, arrange_coplanar_surface_point_touch_union,
    arrange_single_triangle_coplanar_difference, arrange_single_triangle_coplanar_holed_difference,
    arrange_single_triangle_coplanar_union, difference_single_triangle_coplanar_surfaces,
    intersect_single_triangle_coplanar_surfaces, union_single_triangle_coplanar_surfaces,
};
use hypermesh::exact::{
    ExactArrangement, ExactBooleanOperation, ExactBooleanResultKind, ExactBooleanShortcutKind,
    ExactBooleanSupport, ExactMesh, ExactRegularizationPolicy, ValidationPolicy, boolean_exact,
    exact_arrangement_boolean_attempt_report, preflight_boolean_exact,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let fixture = CoplanarFixture::from_data(data);
    let pairs = fixture.pairs();
    if pairs.is_empty() {
        return;
    }
    let pair_index = usize::from(byte(data, 5)) % pairs.len();
    let operation = operation_from_byte(byte(data, 6));
    let (left, right) = &pairs[pair_index];
    exercise_case(left, right, operation);
});

fn byte(data: &[u8], index: usize) -> u8 {
    data.get(index).copied().unwrap_or(0)
}

#[derive(Clone)]
struct CoplanarFixture {
    span: i64,
    inset: i64,
    offset: i64,
    cutter_width: i64,
    branch: i64,
}

impl CoplanarFixture {
    fn from_data(data: &[u8]) -> Self {
        let byte = |index: usize| data.get(index).copied().unwrap_or(0);
        let span = 8 + i64::from(byte(0) % 12);
        let inset = 1 + i64::from(byte(1) % 3);
        let offset = i64::from(byte(2) % 5) - 2;
        let cutter_width = 2 + i64::from(byte(3) % 5);
        let branch = 1 + i64::from(byte(4) % 4);
        Self {
            span,
            inset,
            offset,
            cutter_width,
            branch,
        }
    }

    fn pairs(&self) -> Vec<(ExactMesh, ExactMesh)> {
        let left = rect_mesh(0, 0, self.span, self.span);
        let overlapping = rect_mesh(
            self.inset,
            self.inset + self.offset.max(0),
            self.span - self.inset,
            self.span - self.inset + self.offset.min(0),
        );
        let side_cutter = rect_mesh(
            -self.inset,
            self.inset,
            self.cutter_width,
            self.span - self.inset,
        );
        let point_touch = triangle_mesh([
            [self.span, self.span, 0],
            [self.span + self.branch, self.span, 0],
            [self.span, self.span + self.branch, 0],
        ]);
        let single_left = triangle_mesh([[0, 0, 0], [self.span, 0, 0], [0, self.span, 0]]);
        let single_right = triangle_mesh([
            [self.inset, self.inset, 0],
            [self.span - self.inset, self.inset, 0],
            [self.inset, self.span - self.inset, 0],
        ]);
        let mut pairs = vec![
            (left.clone(), overlapping.clone()),
            (left.clone(), side_cutter.clone()),
            (left.clone(), point_touch),
            (single_left, single_right),
        ];

        let hole = rect_mesh(
            self.inset + 1,
            self.inset + 1,
            self.span - self.inset - 1,
            self.span - self.inset - 1,
        );
        if let Some(holed_left) = arrange_coplanar_convex_surface_holed_difference(&left, &hole) {
            pairs.push((holed_left.mesh.clone(), side_cutter));
            pairs.push((holed_left.mesh, overlapping));
        }
        pairs
    }
}

fn operation_from_byte(byte: u8) -> ExactBooleanOperation {
    match byte % 3 {
        0 => ExactBooleanOperation::Union,
        1 => ExactBooleanOperation::Intersection,
        _ => ExactBooleanOperation::Difference,
    }
}

fn exercise_case(left: &ExactMesh, right: &ExactMesh, operation: ExactBooleanOperation) {
    exercise_boolmesh_workspace(left, right, operation);
    exercise_preflight_public_consistency(left, right, operation);
    exercise_arrangement_public_consistency(left, right, operation);
    exercise_surface_fallback_parity(left, right, operation);
}

fn exercise_boolmesh_workspace(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) {
    let workspace = hypermesh::exact::boolmesh::exact_boolmesh_workspace(left, right, operation);
    let _ = workspace.validate_against_sources(left, right);
}

fn exercise_preflight_public_consistency(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) {
    let Ok(preflight) = preflight_boolean_exact(left, right, operation) else {
        return;
    };
    if preflight.validate().is_err() {
        return;
    }
    let _ = preflight.validate_against_sources(left, right);
    let Ok(result) = boolean_exact(left, right, operation, ValidationPolicy::ALLOW_BOUNDARY) else {
        return;
    };
    result.validate().unwrap();
    match preflight.support {
        ExactBooleanSupport::CertifiedArrangementCellComplex => assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            }
        ),
        ExactBooleanSupport::CertifiedBoolMeshSplit => assert_eq!(
            result.kind,
            ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::BoolMeshSplit,
            }
        ),
        _ => {}
    }
}

fn exercise_arrangement_public_consistency(
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
        ExactBooleanShortcutKind::ArrangementCellComplex => {
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut { shortcut }
            );
            assert_eq!(result.mesh.vertices().len(), attempt.output_vertices);
            assert_eq!(result.mesh.triangles().len(), attempt.output_triangles);
            check_direct_arrangement_triangulation(left, right, operation, &result.mesh);
        }
        ExactBooleanShortcutKind::BoolMeshSplit => {
            assert_eq!(
                result.kind,
                ExactBooleanResultKind::CertifiedShortcut { shortcut }
            );
            assert_eq!(result.mesh.vertices().len(), attempt.output_vertices);
            assert_eq!(result.mesh.triangles().len(), attempt.output_triangles);
        }
        _ => {}
    }
}

fn check_direct_arrangement_triangulation(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
    public_mesh: &ExactMesh,
) {
    let Ok(arrangement) = ExactArrangement::from_meshes_with_policy(
        left,
        right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    ) else {
        return;
    };
    let _ = arrangement.validate_against_sources_with_policy(
        left,
        right,
        ExactRegularizationPolicy::REGULARIZED_SOLID,
    );
    let Ok(labeled) = arrangement.label_regions(ExactRegularizationPolicy::REGULARIZED_SOLID)
    else {
        return;
    };
    let Ok(selected) =
        labeled.select_with_policy(operation, ExactRegularizationPolicy::REGULARIZED_SOLID)
    else {
        return;
    };
    let Ok(simplified) =
        selected.simplify_exact_with_policy(ExactRegularizationPolicy::REGULARIZED_SOLID)
    else {
        return;
    };
    if !simplified.blockers.is_empty() {
        return;
    }
    let Ok(mesh) = simplified.triangulate() else {
        return;
    };
    assert_same_mesh_shape(public_mesh, &mesh);
}

fn exercise_surface_fallback_parity(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) {
    for legacy in legacy_outputs(left, right, operation) {
        legacy.validate_retained_state().unwrap();
        let Ok(result) = boolean_exact(left, right, operation, ValidationPolicy::ALLOW_BOUNDARY)
        else {
            continue;
        };
        result.validate().unwrap();
        if result.kind
            != (ExactBooleanResultKind::CertifiedShortcut {
                shortcut: ExactBooleanShortcutKind::ArrangementCellComplex,
            })
        {
            assert_same_mesh_shape(&result.mesh, &legacy);
        }
    }
}

fn legacy_outputs(
    left: &ExactMesh,
    right: &ExactMesh,
    operation: ExactBooleanOperation,
) -> Vec<ExactMesh> {
    match operation {
        ExactBooleanOperation::Union => [
            arrange_coplanar_convex_surface_union(left, right).map(|artifact| artifact.mesh),
            arrange_coplanar_convex_surface_component_union(left, right)
                .map(|artifact| artifact.mesh),
            arrange_coplanar_convex_surface_multi_union(left, right).map(|artifact| artifact.mesh),
            arrange_coplanar_surface_component_union(left, right).map(|artifact| artifact.mesh),
            arrange_coplanar_surface_component_holed_union(left, right)
                .map(|artifact| artifact.mesh),
            arrange_coplanar_surface_multi_component_union(left, right)
                .map(|artifact| artifact.mesh),
            arrange_coplanar_surface_point_touch_union(left, right).map(|artifact| artifact.mesh),
            union_single_triangle_coplanar_surfaces(left, right).map(|artifact| artifact.mesh),
            arrange_single_triangle_coplanar_union(left, right).map(|artifact| artifact.mesh),
        ]
        .into_iter()
        .flatten()
        .collect(),
        ExactBooleanOperation::Intersection => [
            arrange_coplanar_convex_surface_intersection(left, right).map(|artifact| artifact.mesh),
            arrange_coplanar_convex_surface_multi_intersection(left, right)
                .map(|artifact| artifact.mesh),
            arrange_coplanar_surface_component_intersection(left, right)
                .map(|artifact| artifact.mesh),
            arrange_coplanar_surface_multi_component_intersection(left, right)
                .map(|artifact| artifact.mesh),
            arrange_coplanar_surface_component_holed_intersection(left, right)
                .map(|artifact| artifact.mesh),
            intersect_single_triangle_coplanar_surfaces(left, right).map(|artifact| artifact.mesh),
        ]
        .into_iter()
        .flatten()
        .collect(),
        ExactBooleanOperation::Difference => [
            difference_single_triangle_coplanar_surfaces(left, right).map(|artifact| artifact.mesh),
            arrange_single_triangle_coplanar_difference(left, right).map(|artifact| artifact.mesh),
            arrange_single_triangle_coplanar_holed_difference(left, right)
                .map(|artifact| artifact.mesh),
        ]
        .into_iter()
        .flatten()
        .collect(),
        ExactBooleanOperation::SelectedRegions(_) => Vec::new(),
    }
}

fn rect_mesh(x0: i64, y0: i64, x1: i64, y1: i64) -> ExactMesh {
    let (x0, x1) = ordered_span(x0, x1);
    let (y0, y1) = ordered_span(y0, y1);
    ExactMesh::from_i64_triangles_with_policy(
        &[x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y1, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("fuzz rectangle should import")
}

fn ordered_span(a: i64, b: i64) -> (i64, i64) {
    if a == b {
        (a, a + 1)
    } else if a < b {
        (a, b)
    } else {
        (b, a)
    }
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
    .expect("fuzz triangle should import")
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
