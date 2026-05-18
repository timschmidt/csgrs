#![no_main]

use hypermesh::exact::{
    ExactBooleanOperation, ExactBooleanPolicy, ExactBoundaryBooleanPolicy, ExactMesh,
    ValidationPolicy, arrange_single_triangle_coplanar_difference,
    arrange_single_triangle_coplanar_holed_difference, arrange_single_triangle_coplanar_union,
    arrange_coplanar_convex_surface_difference, arrange_coplanar_convex_surface_holed_difference,
    arrange_coplanar_convex_surface_intersection, arrange_coplanar_convex_surface_union,
    boolean_exact_with_boundary_policy, boolean_selected_regions, certify_boundary_touching_report,
    certify_convex_solid, certify_open_surface_disjoint_report, certify_planar_arrangement_report,
    certify_refinement_report, certify_coplanar_convex_surface_containment,
    certify_coplanar_convex_surface_equivalence, certify_coplanar_convex_surface_report,
    certify_same_surface_report,
    certify_single_triangle_coplanar_containment, certify_single_triangle_coplanar_containment_report,
    certify_winding_readiness_report, classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, difference_single_triangle_coplanar_surfaces,
    intersect_single_triangle_coplanar_surfaces, preflight_boolean_exact,
    union_single_triangle_coplanar_surfaces,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let mut values = Vec::new();
    let mut indices = Vec::new();

    for chunk in data.chunks_exact(2).take(72) {
        values.push(i16::from_le_bytes(chunk.try_into().unwrap()) as i64);
    }
    for chunk in data.chunks_exact(2).skip(72).take(72) {
        indices.push((u16::from_le_bytes(chunk.try_into().unwrap()) % 12) as usize);
    }

    if values.len() < 18 || indices.len() < 6 {
        return;
    }

    let left_value_end = values.len() / 2 / 3 * 3;
    let right_value_start = left_value_end;
    let right_value_len = (values.len() - right_value_start) / 3 * 3;
    let left_index_end = indices.len() / 2 / 3 * 3;
    let right_index_start = left_index_end;
    let right_index_len = (indices.len() - right_index_start) / 3 * 3;

    let left_values = &values[..left_value_end];
    let right_values = &values[right_value_start..right_value_start + right_value_len];
    let left_indices = &indices[..left_index_end];
    let right_indices = &indices[right_index_start..right_index_start + right_index_len];

    let Ok(left) = ExactMesh::from_i64_triangles_with_policy(
        left_values,
        left_indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    ) else {
        return;
    };
    let Ok(right) = ExactMesh::from_i64_triangles_with_policy(
        right_values,
        right_indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    ) else {
        return;
    };

    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .map(|report| report.validate());
    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| report.validate());
    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate());
    let _ = certify_same_surface_report(&left, &right).validate();
    let _ = certify_same_surface_report(&right, &left).validate();
    let _ = certify_coplanar_convex_surface_equivalence(&left, &right)
        .map(|report| report.validate());
    let _ = certify_coplanar_convex_surface_equivalence(&right, &left)
        .map(|report| report.validate());
    let _ = certify_coplanar_convex_surface_report(&left, &right).validate();
    let _ = certify_coplanar_convex_surface_report(&right, &left).validate();
    let _ =
        certify_coplanar_convex_surface_containment(&left, &right).map(|report| report.validate());
    let _ =
        certify_coplanar_convex_surface_containment(&right, &left).map(|report| report.validate());
    let _ = certify_open_surface_disjoint_report(&left, &right).map(|report| report.validate());
    let _ = certify_open_surface_disjoint_report(&right, &left).map(|report| report.validate());
    let _ = certify_boundary_touching_report(&left, &right).map(|report| report.validate());
    let _ = certify_boundary_touching_report(&right, &left).map(|report| report.validate());
    let _ = certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Union)
        .map(|report| report.validate());
    let _ = certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| report.validate());
    let _ = certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate());
    let _ = certify_planar_arrangement_report(&right, &left, ExactBooleanOperation::Union)
        .map(|report| report.validate());
    let _ = certify_planar_arrangement_report(&right, &left, ExactBooleanOperation::Intersection)
        .map(|report| report.validate());
    let _ = certify_planar_arrangement_report(&right, &left, ExactBooleanOperation::Difference)
        .map(|report| report.validate());
    let _ = certify_refinement_report(&left, &right, ExactBooleanOperation::Union)
        .map(|report| report.validate());
    let _ = certify_refinement_report(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| report.validate());
    let _ = certify_refinement_report(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate());
    let _ = certify_refinement_report(&right, &left, ExactBooleanOperation::Union)
        .map(|report| report.validate());
    let _ = certify_refinement_report(&right, &left, ExactBooleanOperation::Intersection)
        .map(|report| report.validate());
    let _ = certify_refinement_report(&right, &left, ExactBooleanOperation::Difference)
        .map(|report| report.validate());
    let _ = certify_winding_readiness_report(&left, &right, ExactBooleanOperation::Union)
        .map(|report| report.validate());
    let _ = certify_winding_readiness_report(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| report.validate());
    let _ = certify_winding_readiness_report(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate());
    let _ = certify_winding_readiness_report(&right, &left, ExactBooleanOperation::Union)
        .map(|report| report.validate());
    let _ = certify_winding_readiness_report(&right, &left, ExactBooleanOperation::Intersection)
        .map(|report| report.validate());
    let _ = certify_winding_readiness_report(&right, &left, ExactBooleanOperation::Difference)
        .map(|report| report.validate());
    let _ = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .map(|result| result.validate());
    let _ = certify_convex_solid(&left).validate();
    let _ = certify_convex_solid(&right).validate();
    let _ = certify_single_triangle_coplanar_containment(&left, &right);
    let _ = certify_single_triangle_coplanar_containment(&right, &left);
    let _ = certify_single_triangle_coplanar_containment_report(&left, &right).validate();
    let _ = certify_single_triangle_coplanar_containment_report(&right, &left).validate();
    if let Some(output) = intersect_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = intersect_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = union_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = union_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_union(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_union(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = difference_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = difference_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_difference(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_difference(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_holed_difference(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_holed_difference(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_holed_difference(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_holed_difference(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_intersection(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_intersection(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_union(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_union(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_difference(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_coplanar_convex_surface_difference(&right, &left) {
        let _ = output.validate();
    }
    let _ = classify_mesh_vertices_against_convex_solid(&left, &right);
    let _ = classify_mesh_vertices_against_convex_solid(&right, &left);
    let _ = classify_mesh_vertices_against_convex_solid_report(&left, &right).validate();
    let _ = classify_mesh_vertices_against_convex_solid_report(&right, &left).validate();

    if left.triangles().len() <= 4 && right.triangles().len() <= 4 {
        let _ = boolean_selected_regions(&left, &right, ExactBooleanPolicy::KEEP_ALL_BOUNDARY)
            .map(|result| result.validate());
    }
});
