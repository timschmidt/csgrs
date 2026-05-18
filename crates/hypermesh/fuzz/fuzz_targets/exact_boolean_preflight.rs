#![no_main]

use hypermesh::exact::{
    ExactBooleanOperation, ExactBooleanPolicy, ExactBoundaryBooleanPolicy, ExactMesh,
    ValidationPolicy, boolean_exact_with_boundary_policy, boolean_selected_regions,
    certify_convex_solid, certify_single_triangle_coplanar_containment,
    certify_same_surface_report, certify_single_triangle_coplanar_containment_report,
    classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report,
    difference_single_triangle_coplanar_surfaces, intersect_single_triangle_coplanar_surfaces,
    preflight_boolean_exact, union_single_triangle_coplanar_surfaces,
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

    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union);
    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection);
    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference);
    let _ = certify_same_surface_report(&left, &right);
    let _ = certify_same_surface_report(&right, &left);
    let _ = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    );
    let _ = certify_convex_solid(&left);
    let _ = certify_convex_solid(&right);
    let _ = certify_single_triangle_coplanar_containment(&left, &right);
    let _ = certify_single_triangle_coplanar_containment(&right, &left);
    let _ = certify_single_triangle_coplanar_containment_report(&left, &right);
    let _ = certify_single_triangle_coplanar_containment_report(&right, &left);
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
    if let Some(output) = difference_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = difference_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate();
    }
    let _ = classify_mesh_vertices_against_convex_solid(&left, &right);
    let _ = classify_mesh_vertices_against_convex_solid(&right, &left);
    let _ = classify_mesh_vertices_against_convex_solid_report(&left, &right);
    let _ = classify_mesh_vertices_against_convex_solid_report(&right, &left);

    if left.triangles().len() <= 4 && right.triangles().len() <= 4 {
        let _ = boolean_selected_regions(&left, &right, ExactBooleanPolicy::KEEP_ALL_BOUNDARY);
    }
});
