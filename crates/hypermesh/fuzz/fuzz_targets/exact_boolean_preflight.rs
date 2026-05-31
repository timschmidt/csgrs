#![no_main]

#[cfg(feature = "exact-triangulation")]
use std::cmp::Ordering;

#[cfg(feature = "exact-triangulation")]
use hyperlimit::{PlaneSide, Point3, compare_reals};
use hypermesh::exact::{
    CoplanarAffineSurfaceArrangement, CoplanarAffineSurfaceBasis, CoplanarArrangementOperation,
    CoplanarOrthogonalSurfaceArrangement, CoplanarOrthogonalSurfaceComponent,
    CoplanarOrthogonalSurfaceOperation, CoplanarSurfaceContainment,
    CoplanarSurfaceContainmentStatus, CoplanarTriangleRelation, ExactBooleanOperation,
    ExactBooleanPolicy, ExactBooleanSupport, ExactBoundaryBooleanPolicy, ExactMesh, ExactReal,
    ExactRegionSelection, FaceRegionPlaneRelation, FaceSplitBoundaryNode, SourceProvenance,
    SegmentPlaneRelation, Triangle, ValidationPolicy,
    arrange_coplanar_affine_surface_difference, arrange_coplanar_affine_surface_intersection,
    arrange_coplanar_affine_surface_union,
    arrange_coplanar_convex_surface_component_holed_difference,
    arrange_coplanar_convex_surface_component_union, arrange_coplanar_convex_surface_difference,
    arrange_coplanar_convex_surface_holed_difference, arrange_coplanar_convex_surface_intersection,
    arrange_coplanar_convex_surface_multi_difference,
    arrange_coplanar_convex_surface_multi_holed_difference,
    arrange_coplanar_convex_surface_multi_intersection,
    arrange_coplanar_convex_surface_multi_union, arrange_coplanar_convex_surface_union,
    arrange_coplanar_orthogonal_surface_difference,
    arrange_coplanar_orthogonal_surface_intersection, arrange_coplanar_orthogonal_surface_union,
    arrange_coplanar_surface_component_difference,
    arrange_coplanar_surface_component_holed_difference,
    arrange_coplanar_surface_component_holed_intersection,
    arrange_coplanar_surface_component_holed_union, arrange_coplanar_surface_component_intersection,
    arrange_coplanar_surface_component_union,
    arrange_coplanar_surface_cutter_hole_contact_difference,
    arrange_coplanar_surface_multi_component_intersection,
    arrange_coplanar_surface_multi_component_union, arrange_coplanar_surface_multi_difference,
    arrange_coplanar_surface_point_touch_difference, arrange_coplanar_surface_point_touch_union,
    arrange_coplanar_surface_side_cutter_difference,
    arrange_single_triangle_coplanar_difference, arrange_single_triangle_coplanar_holed_difference,
    arrange_single_triangle_coplanar_union, boolean_exact_with_boundary_policy,
    boolean_selected_regions, build_intersection_graph, build_selected_region_mesh,
    certify_boundary_touching_report, certify_convex_solid,
    certify_coplanar_convex_surface_containment, certify_coplanar_convex_surface_equivalence,
    certify_coplanar_convex_surface_report, certify_coplanar_surface_boundary_touch,
    certify_coplanar_surface_mesh_containment, certify_coplanar_volumetric_cell_evidence,
    certify_exact_mesh_proposal, certify_open_surface_disjoint_report,
    certify_planar_arrangement_evidence, certify_planar_arrangement_report,
    certify_refinement_report, certify_same_surface_report,
    certify_single_triangle_coplanar_containment,
    certify_single_triangle_coplanar_containment_report, certify_winding_readiness_report,
    classify_coplanar_triangles, classify_mesh_face_pair, classify_mesh_face_pairs,
    classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, classify_point_against_convex_solid_report,
    classify_triangle_against_face_plane, classify_triangle_triangle,
    difference_single_triangle_coplanar_surfaces, intersect_closed_convex_solids,
    intersect_single_triangle_coplanar_surfaces, preflight_boolean_exact,
    subtract_closed_convex_solids_single_cap, union_single_triangle_coplanar_surfaces,
};
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::{CoplanarProjection, ExactBooleanAssemblyPlan, FaceRegionTriangulation};
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_kernel11_shadow_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_kernel02_shadow_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_kernel12_shadow_accumulator_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_kernel_frame_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_kernel12_accumulator_replay_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_kernel12_intersect_loop_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_kernel03_winding_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_boolean45_triangulation_probe_for_internal_fuzz;
#[cfg(feature = "exact-triangulation")]
use hypermesh::exact::boolmesh::exact_boolmesh_cleanup_probe_for_internal_fuzz;
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let deterministic_data = data.strip_suffix(b"\n").unwrap_or(data);
    if let [b'C', b'A', b'S', b'E', selector] = deterministic_data {
        exercise_deterministic_case(*selector);
        return;
    }

    let mut values = Vec::new();
    let mut indices = Vec::new();

    const FUZZ_VALUE_WORDS: usize = 36;
    const FUZZ_INDEX_WORDS: usize = 36;

    for chunk in data.chunks_exact(2).take(FUZZ_VALUE_WORDS) {
        let raw = i16::from_le_bytes(chunk.try_into().unwrap()) as i32;
        // Yap's exact-computation model keeps decisions exact but still
        // requires an explicit resource budget for adversarial inputs; see
        // Chee K. Yap, "Towards Exact Geometric Computation,"
        // Computational Geometry 7.1-2 (1997). The fuzz target stresses
        // degeneracy and topology on a bounded integer grid so coefficient
        // swell does not mask boolean regressions as allocator OOMs.
        values.push((raw.rem_euclid(257) - 128) as i64);
    }
    for chunk in data
        .chunks_exact(2)
        .skip(FUZZ_VALUE_WORDS)
        .take(FUZZ_INDEX_WORDS)
    {
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

    let left_points = left
        .vertices()
        .iter()
        .map(|vertex| vertex.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let left_triangles = left
        .triangles()
        .iter()
        .map(|triangle| triangle.0)
        .collect::<Vec<_>>();
    let _ = left
        .bounds()
        .validate_against_sources(&left_points, &left_triangles);
    if let Some(bounds) = left.bounds().mesh.as_ref() {
        let _ = bounds.validate_against_points(&left_points);
    }
    for (bounds, triangle) in left.bounds().faces.iter().zip(left_triangles.iter()) {
        let _ = bounds.validate_against_triangle([
            &left_points[triangle[0]],
            &left_points[triangle[1]],
            &left_points[triangle[2]],
        ]);
    }
    let _ = left
        .facts()
        .validate_against_sources(&left_points, &left_triangles);
    let _ = left.provenance().source.validate();
    for predicate in &left.provenance().predicates {
        let _ = predicate.validate();
    }
    let _ = left.provenance().validate();
    let right_points = right
        .vertices()
        .iter()
        .map(|vertex| vertex.to_hyperlimit_point())
        .collect::<Vec<_>>();
    let right_triangles = right
        .triangles()
        .iter()
        .map(|triangle| triangle.0)
        .collect::<Vec<_>>();
    let _ = right
        .bounds()
        .validate_against_sources(&right_points, &right_triangles);
    if let Some(bounds) = right.bounds().mesh.as_ref() {
        let _ = bounds.validate_against_points(&right_points);
    }
    for (bounds, triangle) in right.bounds().faces.iter().zip(right_triangles.iter()) {
        let _ = bounds.validate_against_triangle([
            &right_points[triangle[0]],
            &right_points[triangle[1]],
            &right_points[triangle[2]],
        ]);
    }
    let _ = right
        .facts()
        .validate_against_sources(&right_points, &right_triangles);
    let _ = right.provenance().source.validate();
    for predicate in &right.provenance().predicates {
        let _ = predicate.validate();
    }
    let _ = right.provenance().validate();

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let workspace = hypermesh::exact::exact_boolmesh_workspace(&left, &right, operation);
        let _ = workspace.validate_against_sources(&left, &right);
        let mut corrupted = workspace.clone();
        corrupted.boolean03.w30.push(0);
        assert!(corrupted.validate_against_sources(&left, &right).is_err());
        if !workspace.kernel12_events.is_empty() {
            let mut corrupted_event = workspace.clone();
            corrupted_event.kernel12_events[0].edge_face.edge[0] = left.vertices().len();
            assert!(
                corrupted_event
                    .validate_against_sources(&left, &right)
                    .is_err()
            );
        }
        if !workspace.boolean03.p1q2.is_empty() || !workspace.boolean03.p2q1.is_empty() {
            let mut corrupted_lowering = workspace.clone();
            corrupted_lowering.boolean03.x12.push(1);
            assert!(
                corrupted_lowering
                    .validate_against_sources(&left, &right)
                    .is_err()
            );
            let mut corrupted_pairing = workspace.clone();
            corrupted_pairing.pair_up.source_edge_runs[0].unpaired_events += 1;
            assert!(
                corrupted_pairing
                    .validate_against_sources(&left, &right)
                    .is_err()
            );
        }
        if let Some(boolean45) = workspace.boolean45.as_ref() {
            assert_eq!(
                boolean45.output_triangles.invalid_local_triangles,
                0,
                "valid boolmesh loop triangulations should materialize clean output triangles"
            );
            assert_eq!(
                boolean45.output_triangles.triangles.len(),
                boolean45
                    .loop_triangulation
                    .triangulations
                    .iter()
                    .map(|triangulation| triangulation.triangles.len() / 3)
                    .sum::<usize>()
            );
            assert_eq!(
                boolean45.mesh_export.vertex_count,
                boolean45.vertex_allocation.output_vertex_origins.len()
            );
            assert_eq!(
                boolean45.mesh_export.triangles.len(),
                boolean45.output_triangles.triangles.len()
            );
            assert_eq!(
                boolean45.mesh_export.blocked_output_triangles,
                boolean45.output_triangles.missing_loop_triangulations
                    + boolean45.output_triangles.invalid_local_triangles
            );
            assert!(
                boolean45.mesh_export.orientation_failures
                    <= boolean45.output_triangles.triangles.len()
            );
            if !boolean45.output_triangles.triangles.is_empty() {
                let mut corrupted_output = workspace.clone();
                corrupted_output
                    .boolean45
                    .as_mut()
                    .unwrap()
                    .output_triangles
                    .triangles[0]
                    .vertices[0] = usize::MAX;
                assert!(
                    corrupted_output
                        .validate_against_sources(&left, &right)
                        .is_err()
                );
            }
            if !boolean45.loop_triangulation.triangulations.is_empty() {
                let mut corrupted_component = workspace.clone();
                corrupted_component
                    .boolean45
                    .as_mut()
                    .unwrap()
                    .loop_triangulation
                    .triangulations[0]
                    .component_loop_indices
                    .push(usize::MAX);
                assert!(
                    corrupted_component
                        .validate_against_sources(&left, &right)
                        .is_err()
                );
                let mut corrupted_constraint = workspace.clone();
                corrupted_constraint
                    .boolean45
                    .as_mut()
                    .unwrap()
                    .loop_triangulation
                    .triangulations[0]
                    .constraint_edges
                    .push([0, usize::MAX]);
                assert!(
                    corrupted_constraint
                        .validate_against_sources(&left, &right)
                        .is_err()
                );
            }
            if !boolean45.mesh_export.triangles.is_empty() {
                let mut corrupted_export = workspace.clone();
                corrupted_export
                    .boolean45
                    .as_mut()
                    .unwrap()
                    .mesh_export
                    .triangles[0]
                    .0[0] = usize::MAX;
                assert!(
                    corrupted_export
                        .validate_against_sources(&left, &right)
                        .is_err()
                );
            }
        }
        if workspace.is_certified_bounds_disjoint() {
            let execution = hypermesh::exact::execute_exact_boolmesh_bounds_disjoint(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
            )
            .expect("certified disjoint boolmesh slice should execute");
            execution.validate_against_sources(&left, &right).unwrap();
        }
    }

    if left.facts().mesh.closed_manifold && !right.vertices().is_empty() {
        let point = right.vertices()[0].to_hyperlimit_point();
        let point_winding =
            hypermesh::exact::classify_point_against_closed_mesh_winding_report(&point, &left);
        let _ = point_winding.validate_against_sources(&point, &left);
        let mesh_winding =
            hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(
                &right, &left,
            );
        let _ = mesh_winding.validate_against_sources(&right, &left);
        let mut corrupted = mesh_winding;
        if let Some(vertex) = corrupted.vertices.first_mut() {
            vertex.tested_axes = usize::MAX;
            assert!(corrupted.validate().is_err());
        }
    }
    if right.facts().mesh.closed_manifold && !left.vertices().is_empty() {
        let point = left.vertices()[0].to_hyperlimit_point();
        let point_winding =
            hypermesh::exact::classify_point_against_closed_mesh_winding_report(&point, &right);
        let _ = point_winding.validate_against_sources(&point, &right);
        let mesh_winding =
            hypermesh::exact::classify_mesh_vertices_against_closed_mesh_winding_report(
                &left, &right,
            );
        let _ = mesh_winding.validate_against_sources(&left, &right);
        let mut corrupted = mesh_winding;
        if let Some(vertex) = corrupted.vertices.first_mut() {
            vertex.axis = None;
            if vertex.is_decided() {
                assert!(corrupted.validate().is_err());
            }
        }
    }

    if let (Some(left_tri), Some(right_tri)) = (left.triangles().first(), right.triangles().first())
    {
        let _ = classify_mesh_face_pair(&left, 0, &right, 0)
            .map(|classification| classification.validate_against_sources(&left, &right));
        let mut points = left
            .vertices()
            .iter()
            .map(|vertex| vertex.to_hyperlimit_point())
            .collect::<Vec<_>>();
        let right_offset = points.len();
        points.extend(
            right
                .vertices()
                .iter()
                .map(|vertex| vertex.to_hyperlimit_point()),
        );
        let left_face = left_tri.0;
        let right_face = [
            right_tri.0[0] + right_offset,
            right_tri.0[1] + right_offset,
            right_tri.0[2] + right_offset,
        ];
        let plane = classify_triangle_against_face_plane(&points, left_face, right_face);
        let _ = plane.validate_against_sources(&points, left_face, right_face);
        let coplanar = classify_coplanar_triangles(&points, left_face, right_face);
        let _ = coplanar.validate_against_sources(&points, left_face, right_face);
        let narrow = classify_triangle_triangle(&points, left_face, right_face);
        let _ = narrow.validate_against_sources(&points, left_face, right_face);
        for event in narrow
            .right_edge_events
            .iter()
            .chain(narrow.left_edge_events.iter())
        {
            let _ = event.validate();
        }
        if let Some(event) = narrow.right_edge_events.first() {
            let edge = [right_face[0], right_face[1]];
            let _ = event.validate_against_sources(
                &points[left_face[0]],
                &points[left_face[1]],
                &points[left_face[2]],
                &points[edge[0]],
                &points[edge[1]],
            );
        }
    }
    let _ = classify_mesh_face_pairs(&left, &right).map(|classifications| {
        classifications
            .iter()
            .map(|classification| classification.validate_against_sources(&left, &right))
            .collect::<Vec<_>>()
    });

    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .map(|report| report.validate_against_sources(&left, &right));
    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| report.validate_against_sources(&left, &right));
    let _ = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate_against_sources(&left, &right));
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        if let Ok(result) = boolean_exact_with_boundary_policy(
            &left,
            &right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::PreserveSeparateShells,
        ) {
            let _ = result.validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::ALLOW_BOUNDARY,
                ExactBoundaryBooleanPolicy::PreserveSeparateShells,
            );
            if matches!(
                result.kind,
                hypermesh::exact::ExactBooleanResultKind::WindingMaterialized { .. }
            ) {
                let mut missing_volumetric = result.clone();
                missing_volumetric.volumetric_classifications.clear();
                assert!(missing_volumetric.validate().is_err());
            }
            let mut stale_result = result;
            stale_result.graph_had_unknowns = true;
            assert!(
                stale_result
                    .validate_operation_against_sources(
                        &left,
                        &right,
                        operation,
                        ValidationPolicy::ALLOW_BOUNDARY,
                        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
                    )
                    .is_err()
            );
        }
    }
    let _ = preflight_boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
    )
    .map(|report| {
        let _ = report.validate_against_sources(&left, &right);
        if let Some(blocker) = &report.blocker {
            let _ = blocker.validate_against_sources(&left, &right);
        }
        let mut blocked = report.clone();
        blocked.blocker = Some(hypermesh::exact::ExactBooleanBlocker {
            kind: hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding,
            candidate_pairs: 1,
            coplanar_overlapping_pairs: 0,
            coplanar_touching_pairs: 0,
            unknown_pairs: 0,
            construction_failed_events: 0,
        });
        assert!(blocked.validate().is_err());
        let mut orphan_event = report.clone();
        orphan_event.retained_face_pairs = 0;
        orphan_event.retained_events = 1;
        orphan_event.region_count = 0;
        orphan_event.region_classifications.clear();
        assert!(orphan_event.validate().is_err());
        let mut empty_pair = report.clone();
        empty_pair.retained_face_pairs = 1;
        empty_pair.retained_events = 0;
        empty_pair.region_count = 0;
        empty_pair.region_classifications.clear();
        assert!(empty_pair.validate().is_err());
        let mut mismatched_region_count = report.clone();
        if !mismatched_region_count.region_classifications.is_empty() {
            mismatched_region_count.region_count += 1;
            assert!(mismatched_region_count.validate().is_err());
        }
        let mut duplicated_region_classification = report.clone();
        if let Some(classification) = duplicated_region_classification
            .region_classifications
            .first()
            .cloned()
        {
            duplicated_region_classification
                .region_classifications
                .push(classification);
            assert!(duplicated_region_classification.validate().is_err());
        }
        let mut undecided_region = report;
        if let Some(classification) = undecided_region.region_classifications.first_mut() {
            classification.relation = FaceRegionPlaneRelation::Unknown;
            classification.node_sides.fill(None);
            assert!(undecided_region.validate().is_err());
        }
    });
    for mut report in [
        certify_same_surface_report(&left, &right),
        certify_same_surface_report(&right, &left),
    ] {
        let _ = report.validate();
        let _ = report.validate_against_sources(&left, &right);
        if report.is_certified() {
            if !report.right_to_left.is_empty() {
                report.right_to_left[0] = usize::MAX;
                assert!(report.validate().is_err());
            }
        } else {
            report.right_to_left.push(0);
            assert!(report.validate().is_err());
        }
    }
    if let Some(certificate) = certify_coplanar_convex_surface_equivalence(&left, &right) {
        let _ = certificate.validate_against_sources(&left, &right);
        let mut reversed_hull = certificate.clone();
        reversed_hull.polygon.reverse();
        assert!(reversed_hull.validate().is_err());
        let mut repeated_hull_point = certificate;
        if repeated_hull_point.polygon.len() > 1 {
            repeated_hull_point.polygon[1] = repeated_hull_point.polygon[0].clone();
            assert!(repeated_hull_point.validate().is_err());
        }
    }
    if let Some(certificate) = certify_coplanar_convex_surface_equivalence(&right, &left) {
        let _ = certificate.validate_against_sources(&right, &left);
        let mut reversed_hull = certificate.clone();
        reversed_hull.polygon.reverse();
        assert!(reversed_hull.validate().is_err());
        let mut repeated_hull_point = certificate;
        if repeated_hull_point.polygon.len() > 1 {
            repeated_hull_point.polygon[1] = repeated_hull_point.polygon[0].clone();
            assert!(repeated_hull_point.validate().is_err());
        }
    }
    for report in [
        certify_coplanar_convex_surface_report(&left, &right),
        certify_coplanar_convex_surface_report(&right, &left),
    ] {
        let _ = report.validate();
        let _ = report.validate_against_sources(&left, &right);
        let mut stale_report = report.clone();
        if let Some(equivalence) = stale_report.equivalence.as_mut() {
            equivalence.polygon.reverse();
            assert!(
                stale_report
                    .validate_against_sources(&left, &right)
                    .is_err()
            );
        }
        let mut stale_report = report.clone();
        if let Some(containment) = stale_report.containment.as_mut() {
            containment.left_hull.reverse();
            assert!(
                stale_report
                    .validate_against_sources(&left, &right)
                    .is_err()
            );
        }
    }
    if let Some(certificate) = certify_coplanar_convex_surface_containment(&left, &right) {
        let _ = certificate.validate_against_sources(&left, &right);
        let mut reversed_left_hull = certificate.clone();
        reversed_left_hull.left_hull.reverse();
        assert!(reversed_left_hull.validate().is_err());
        let mut repeated_right_hull_point = certificate;
        if repeated_right_hull_point.right_hull.len() > 1 {
            repeated_right_hull_point.right_hull[1] =
                repeated_right_hull_point.right_hull[0].clone();
            assert!(repeated_right_hull_point.validate().is_err());
        }
    }
    if let Some(certificate) = certify_coplanar_convex_surface_containment(&right, &left) {
        let _ = certificate.validate_against_sources(&right, &left);
        let mut reversed_left_hull = certificate.clone();
        reversed_left_hull.left_hull.reverse();
        assert!(reversed_left_hull.validate().is_err());
        let mut repeated_right_hull_point = certificate;
        if repeated_right_hull_point.right_hull.len() > 1 {
            repeated_right_hull_point.right_hull[1] =
                repeated_right_hull_point.right_hull[0].clone();
            assert!(repeated_right_hull_point.validate().is_err());
        }
    }
    let _ = certify_open_surface_disjoint_report(&left, &right).map(|mut report| {
        let _ = report.validate();
        let _ = report.validate_against_sources(&left, &right);
        if matches!(
            report.status,
            hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind = hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding;
            assert!(wrong_kind.validate().is_err());
        } else {
            let mut unresolved = report.clone();
            unresolved.blocker.construction_failed_events += 1;
            assert!(unresolved.validate().is_err());
        }
        if report.retained_face_pairs > 0 {
            report.blocker.candidate_pairs = 0;
            report.blocker.coplanar_overlapping_pairs = 0;
            report.blocker.coplanar_touching_pairs = 0;
            report.blocker.unknown_pairs = 0;
            assert!(report.validate().is_err());
        }
    });
    let _ = certify_open_surface_disjoint_report(&right, &left).map(|mut report| {
        let _ = report.validate();
        let _ = report.validate_against_sources(&right, &left);
        if matches!(
            report.status,
            hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind = hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding;
            assert!(wrong_kind.validate().is_err());
        } else {
            let mut unresolved = report.clone();
            unresolved.blocker.construction_failed_events += 1;
            assert!(unresolved.validate().is_err());
        }
        if report.retained_face_pairs > 0 {
            report.blocker.candidate_pairs = 0;
            report.blocker.coplanar_overlapping_pairs = 0;
            report.blocker.coplanar_touching_pairs = 0;
            report.blocker.unknown_pairs = 0;
            assert!(report.validate().is_err());
        }
    });
    let _ = certify_boundary_touching_report(&left, &right).map(|mut report| {
        let _ = report.validate();
        let _ = report.validate_against_sources(&left, &right);
        if matches!(
            report.status,
            hypermesh::exact::ExactBoundaryTouchingStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind =
                hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy;
            assert!(wrong_kind.validate().is_err());
        } else {
            let mut unresolved = report.clone();
            unresolved.blocker.construction_failed_events += 1;
            assert!(unresolved.validate().is_err());
        }
        if report.retained_face_pairs > 0 {
            report.blocker.candidate_pairs = 0;
            report.blocker.coplanar_overlapping_pairs = 0;
            report.blocker.coplanar_touching_pairs = 0;
            report.blocker.unknown_pairs = 0;
            assert!(report.validate().is_err());
        }
    });
    let _ = certify_boundary_touching_report(&right, &left).map(|mut report| {
        let _ = report.validate();
        let _ = report.validate_against_sources(&right, &left);
        if matches!(
            report.status,
            hypermesh::exact::ExactBoundaryTouchingStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind =
                hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy;
            assert!(wrong_kind.validate().is_err());
        } else {
            let mut unresolved = report.clone();
            unresolved.blocker.construction_failed_events += 1;
            assert!(unresolved.validate().is_err());
        }
        if report.retained_face_pairs > 0 {
            report.blocker.candidate_pairs = 0;
            report.blocker.coplanar_overlapping_pairs = 0;
            report.blocker.coplanar_touching_pairs = 0;
            report.blocker.unknown_pairs = 0;
            assert!(report.validate().is_err());
        }
    });
    let _ = certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Union).map(
        |report| {
            let _ = report.validate();
            let _ = report.validate_against_sources(&left, &right);
            let _ = report.freshness_against_sources(&left, &right);
            let _ = report.blocker.validate_against_sources(&left, &right);
            if matches!(
                report.status,
                hypermesh::exact::ExactPlanarArrangementStatus::GraphUnknowns
            ) {
                let mut wrong_kind = report.clone();
                wrong_kind.blocker.kind =
                    hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement;
                assert!(wrong_kind.validate().is_err());
                assert_ne!(
                    wrong_kind.freshness_against_sources(&left, &right),
                    hypermesh::exact::ExactReportFreshness::Current
                );
            } else {
                let mut unresolved = report.clone();
                unresolved.blocker.construction_failed_events += 1;
                assert!(unresolved.validate().is_err());
                assert_ne!(
                    unresolved.freshness_against_sources(&left, &right),
                    hypermesh::exact::ExactReportFreshness::Current
                );
            }
            if report.arrangement_readiness.is_some() {
                let mut mismatched_readiness = report.clone();
                if let Some(readiness) = mismatched_readiness.arrangement_readiness.as_mut() {
                    readiness.graph_count += 1;
                    readiness.touching_graphs += 1;
                }
                assert!(mismatched_readiness.validate().is_err());
                assert_ne!(
                    mismatched_readiness.freshness_against_sources(&left, &right),
                    hypermesh::exact::ExactReportFreshness::Current
                );
            }
        },
    );
    let _ = certify_planar_arrangement_evidence(&left, &right).map(|report| {
        let _ = report.validate();
        let _ = report.validate_against_sources(&left, &right);
        let _ = report.freshness_against_sources(&left, &right);
        let mut stale_obstacle = report.clone();
        stale_obstacle.obstacle = hypermesh::exact::PlanarArrangementObstacle::NoCoplanarOverlap;
        if stale_obstacle != report {
            assert!(stale_obstacle.validate().is_err());
            assert_ne!(
                stale_obstacle.freshness_against_sources(&left, &right),
                hypermesh::exact::ExactPlanarArrangementEvidenceFreshness::Current
            );
        }
        let mut stale_branch_side = report.clone();
        stale_branch_side.left_branch_point_count =
            stale_branch_side.branch_point_count.saturating_add(1);
        if stale_branch_side != report {
            assert!(stale_branch_side.validate().is_err());
            assert_eq!(
                stale_branch_side.freshness_against_sources(&left, &right),
                hypermesh::exact::ExactPlanarArrangementEvidenceFreshness::StaleBranchPoints
            );
        }
    });
    let _ = certify_coplanar_volumetric_cell_evidence(&left, &right).map(|report| {
        let _ = report.validate();
        let _ = report.validate_against_sources(&left, &right);
        let _ = report.freshness_against_sources(&left, &right);
        let mut stale_counts = report.clone();
        stale_counts.retained_face_pair_count += 1;
        assert!(stale_counts.validate().is_err());
        assert_ne!(
            stale_counts.freshness_against_sources(&left, &right),
            hypermesh::exact::CoplanarVolumetricCellEvidenceFreshness::Current
        );
        let mut stale_side_counts = report.clone();
        stale_side_counts.same_side_coplanar_overlapping_pairs =
            stale_side_counts.same_side_coplanar_overlapping_pairs.saturating_add(1);
        if stale_side_counts != report {
            assert!(stale_side_counts.validate().is_err());
            assert_eq!(
                stale_side_counts.freshness_against_sources(&left, &right),
                hypermesh::exact::CoplanarVolumetricCellEvidenceFreshness::StaleCoplanarEvidence
            );
        }
    });
    let _ = certify_exact_mesh_proposal(&left).map(|report| {
        let _ = report.validate();
        let _ = report.validate_against_mesh(&left);
        let mut missing_replay = report.clone();
        missing_replay.exact_replay_performed = false;
        assert!(missing_replay.validate().is_err());
        let mut unaccepted = report;
        unaccepted.accepted_topology = false;
        assert!(unaccepted.validate().is_err());
    });
    let _ = certify_exact_mesh_proposal(&right).map(|report| {
        let _ = report.validate();
        let _ = report.validate_against_mesh(&right);
    });
    let _ = certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| {
            let _ = report.validate();
            let _ = report.validate_against_sources(&left, &right);
            let _ = report.blocker.validate_against_sources(&left, &right);
            if report.arrangement_readiness.is_some() {
                let mut mismatched_readiness = report;
                if let Some(readiness) = mismatched_readiness.arrangement_readiness.as_mut() {
                    readiness.graph_count += 1;
                    readiness.touching_graphs += 1;
                }
                assert!(mismatched_readiness.validate().is_err());
            }
        });
    let _ = certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate_against_sources(&left, &right));
    let _ = certify_planar_arrangement_report(&right, &left, ExactBooleanOperation::Union)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_planar_arrangement_report(&right, &left, ExactBooleanOperation::Intersection)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_planar_arrangement_report(&right, &left, ExactBooleanOperation::Difference)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_refinement_report(&left, &right, ExactBooleanOperation::Union)
        .map(|report| report.validate_against_sources(&left, &right));
    let _ = certify_refinement_report(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| report.validate_against_sources(&left, &right));
    let _ = certify_refinement_report(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate_against_sources(&left, &right));
    let _ = certify_refinement_report(&right, &left, ExactBooleanOperation::Union)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_refinement_report(&right, &left, ExactBooleanOperation::Intersection)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_refinement_report(&right, &left, ExactBooleanOperation::Difference)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_winding_readiness_report(&left, &right, ExactBooleanOperation::Union).map(
        |report| {
            let _ = report.validate();
            let _ = report.validate_against_sources(&left, &right);
            let _ = report.freshness_against_sources(&left, &right);
            if !matches!(
                report.status,
                hypermesh::exact::ExactWindingReadinessStatus::GraphUnknowns
            ) {
                let mut unresolved = report.clone();
                unresolved.blocker.construction_failed_events += 1;
                assert!(unresolved.validate().is_err());
                assert_ne!(
                    unresolved.freshness_against_sources(&left, &right),
                    hypermesh::exact::ExactReportFreshness::Current
                );
            }
            if report.arrangement_readiness.is_some() {
                let mut mismatched_readiness = report.clone();
                if let Some(readiness) = mismatched_readiness.arrangement_readiness.as_mut() {
                    readiness.graph_count += 1;
                    readiness.touching_graphs += 1;
                }
                assert!(mismatched_readiness.validate().is_err());
                assert_ne!(
                    mismatched_readiness.freshness_against_sources(&left, &right),
                    hypermesh::exact::ExactReportFreshness::Current
                );
            }
            let mut undecided_region = report;
            if let Some(classification) = undecided_region.region_classifications.first_mut() {
                classification.relation = FaceRegionPlaneRelation::Unknown;
                classification.node_sides.fill(None);
                assert!(undecided_region.validate().is_err());
                assert_ne!(
                    undecided_region.freshness_against_sources(&left, &right),
                    hypermesh::exact::ExactReportFreshness::Current
                );
            }
        },
    );
    let _ = certify_winding_readiness_report(&left, &right, ExactBooleanOperation::Intersection)
        .map(|report| {
            let _ = report.validate();
            let _ = report.validate_against_sources(&left, &right);
        });
    let _ = certify_winding_readiness_report(&left, &right, ExactBooleanOperation::Difference)
        .map(|report| report.validate_against_sources(&left, &right));
    let _ = certify_winding_readiness_report(&right, &left, ExactBooleanOperation::Union)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_winding_readiness_report(&right, &left, ExactBooleanOperation::Intersection)
        .map(|report| report.validate_against_sources(&right, &left));
    let _ = certify_winding_readiness_report(&right, &left, ExactBooleanOperation::Difference)
        .map(|report| report.validate_against_sources(&right, &left));
    if let Ok(graph) = build_intersection_graph(&left, &right) {
        let _ = graph.validate();
        let _ = graph.validate_against_meshes(&left, &right);
        let mut relabeled_graph = graph.clone();
        if let Some(pair) = relabeled_graph.face_pairs.first_mut() {
            pair.left_face = usize::MAX;
            assert!(
                relabeled_graph
                    .validate_against_meshes(&left, &right)
                    .is_err()
            );
            assert!(
                relabeled_graph
                    .coplanar_arrangement_readiness_report(&left, &right)
                    .is_err()
            );
        }
        let _ = graph
            .coplanar_overlap_split_plan(&left, &right)
            .map(|plan| plan.validate_against_meshes(&left, &right));
        let _ = graph
            .coplanar_arrangement_readiness_report(&left, &right)
            .map(|report| {
                let local = report.validate();
                let source = report.validate_against_sources(&left, &right);
                (local, source)
            });
    }
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
    for mut report in [
        certify_single_triangle_coplanar_containment_report(&left, &right),
        certify_single_triangle_coplanar_containment_report(&right, &left),
    ] {
        let _ = report.validate();
        let _ = report.validate_against_sources(&left, &right);
        let mut stale_report = report.clone();
        if stale_report.triangle.is_some() {
            stale_report.triangle = None;
            assert!(
                stale_report
                    .validate_against_sources(&left, &right)
                    .is_err()
            );
        }
        if let Some(coplanar) = &report.coplanar {
            report.status = match coplanar.relation {
                CoplanarTriangleRelation::Disjoint | CoplanarTriangleRelation::Unknown => {
                    CoplanarSurfaceContainmentStatus::AmbiguousOrIdentical
                }
                CoplanarTriangleRelation::Touching => CoplanarSurfaceContainmentStatus::Certified(
                    CoplanarSurfaceContainment::LeftInsideRight,
                ),
                CoplanarTriangleRelation::Overlapping => {
                    CoplanarSurfaceContainmentStatus::DisjointOrUnknown
                }
            };
            assert!(report.validate().is_err());
        }
    }
    if let Some(output) = intersect_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate_against_sources(&left, &right);
    }
    if let Some(output) = intersect_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate_against_sources(&right, &left);
    }
    if let Some(output) = union_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate_against_sources(&left, &right);
        let mut nonconvex = output.clone();
        if nonconvex.polygon.len() >= 4 {
            nonconvex.polygon.swap(2, 3);
            if let Some(mesh) = fan_surface_mesh_with_swapped_tail(&output.mesh) {
                nonconvex.mesh = mesh;
                assert!(nonconvex.validate().is_err());
            }
        }
    }
    if let Some(output) = union_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate_against_sources(&right, &left);
        let mut nonconvex = output.clone();
        if nonconvex.polygon.len() >= 4 {
            nonconvex.polygon.swap(2, 3);
            if let Some(mesh) = fan_surface_mesh_with_swapped_tail(&output.mesh) {
                nonconvex.mesh = mesh;
                assert!(nonconvex.validate().is_err());
            }
        }
    }
    if let Some(output) = arrange_single_triangle_coplanar_union(&left, &right) {
        let _ = output.validate_against_sources(&left, &right, CoplanarArrangementOperation::Union);
        let mut reversed_loop = output.clone();
        reversed_loop.polygon.reverse();
        if let Some(mesh) = reversed_vertex_fan_surface_mesh(&output.mesh) {
            reversed_loop.mesh = mesh;
            assert!(reversed_loop.validate().is_err());
        }
    }
    if let Some(output) = arrange_single_triangle_coplanar_union(&right, &left) {
        let _ = output.validate_against_sources(&right, &left, CoplanarArrangementOperation::Union);
        let mut reversed_loop = output.clone();
        reversed_loop.polygon.reverse();
        if let Some(mesh) = reversed_vertex_fan_surface_mesh(&output.mesh) {
            reversed_loop.mesh = mesh;
            assert!(reversed_loop.validate().is_err());
        }
    }
    if let Some(output) = difference_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate_against_sources(&left, &right);
        let mut nonconvex = output.clone();
        if nonconvex.polygon.len() >= 4 {
            nonconvex.polygon.swap(2, 3);
            if let Some(mesh) = fan_surface_mesh_with_swapped_tail(&output.mesh) {
                nonconvex.mesh = mesh;
                assert!(nonconvex.validate().is_err());
            }
        }
    }
    if let Some(output) = difference_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate_against_sources(&right, &left);
        let mut nonconvex = output.clone();
        if nonconvex.polygon.len() >= 4 {
            nonconvex.polygon.swap(2, 3);
            if let Some(mesh) = fan_surface_mesh_with_swapped_tail(&output.mesh) {
                nonconvex.mesh = mesh;
                assert!(nonconvex.validate().is_err());
            }
        }
    }
    if let Some(output) = arrange_single_triangle_coplanar_difference(&left, &right) {
        let _ = output.validate_against_sources(
            &left,
            &right,
            CoplanarArrangementOperation::Difference,
        );
    }
    if let Some(output) = arrange_single_triangle_coplanar_difference(&right, &left) {
        let _ = output.validate_against_sources(
            &right,
            &left,
            CoplanarArrangementOperation::Difference,
        );
    }
    if let Some(output) = arrange_single_triangle_coplanar_holed_difference(&left, &right) {
        let _ = output.validate_against_sources(&left, &right);
        let mut reversed_outer = output.clone();
        reversed_outer.outer.reverse();
        assert!(reversed_outer.validate().is_err());
        let mut reversed_hole = output.clone();
        reversed_hole.hole.reverse();
        assert!(reversed_hole.validate().is_err());
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) = filled_hole_surface_mesh(&output.mesh, output.outer.len()) {
            let mut filled_hole = output.clone();
            filled_hole.mesh = mesh;
            assert!(filled_hole.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
        if let Some(mesh) = retained_ring_crossing_surface_mesh(&output.mesh) {
            let mut crossing_ring = output.clone();
            crossing_ring.mesh = mesh;
            assert!(crossing_ring.validate().is_err());
        }
    }
    if let Some(output) = arrange_single_triangle_coplanar_holed_difference(&right, &left) {
        let _ = output.validate_against_sources(&right, &left);
        let mut reversed_outer = output.clone();
        reversed_outer.outer.reverse();
        assert!(reversed_outer.validate().is_err());
        let mut reversed_hole = output.clone();
        reversed_hole.hole.reverse();
        assert!(reversed_hole.validate().is_err());
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) = filled_hole_surface_mesh(&output.mesh, output.outer.len()) {
            let mut filled_hole = output.clone();
            filled_hole.mesh = mesh;
            assert!(filled_hole.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
        if let Some(mesh) = retained_ring_crossing_surface_mesh(&output.mesh) {
            let mut crossing_ring = output.clone();
            crossing_ring.mesh = mesh;
            assert!(crossing_ring.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_holed_difference(&left, &right) {
        let _ = output.validate_against_sources(&left, &right);
        let mut reversed_outer = output.clone();
        reversed_outer.outer.reverse();
        assert!(reversed_outer.validate().is_err());
        let mut reversed_hole = output.clone();
        reversed_hole.hole.reverse();
        assert!(reversed_hole.validate().is_err());
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) = filled_hole_surface_mesh(&output.mesh, output.outer.len()) {
            let mut filled_hole = output.clone();
            filled_hole.mesh = mesh;
            assert!(filled_hole.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
        if let Some(mesh) = retained_ring_crossing_surface_mesh(&output.mesh) {
            let mut crossing_ring = output.clone();
            crossing_ring.mesh = mesh;
            assert!(crossing_ring.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_holed_difference(&right, &left) {
        let _ = output.validate_against_sources(&right, &left);
        let mut reversed_outer = output.clone();
        reversed_outer.outer.reverse();
        assert!(reversed_outer.validate().is_err());
        let mut reversed_hole = output.clone();
        reversed_hole.hole.reverse();
        assert!(reversed_hole.validate().is_err());
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) = filled_hole_surface_mesh(&output.mesh, output.outer.len()) {
            let mut filled_hole = output.clone();
            filled_hole.mesh = mesh;
            assert!(filled_hole.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
        if let Some(mesh) = retained_ring_crossing_surface_mesh(&output.mesh) {
            let mut crossing_ring = output.clone();
            crossing_ring.mesh = mesh;
            assert!(crossing_ring.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_multi_holed_difference(&left, &right) {
        let _ = output.validate_against_sources(&left, &right);
        let mut reversed_outer = output.clone();
        reversed_outer.outer.reverse();
        assert!(reversed_outer.validate().is_err());
        let mut reversed_hole = output.clone();
        if let Some(hole) = reversed_hole.holes.first_mut() {
            hole.reverse();
            assert!(reversed_hole.validate().is_err());
        }
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_multi_holed_difference(&right, &left) {
        let _ = output.validate_against_sources(&right, &left);
        let mut reversed_outer = output.clone();
        reversed_outer.outer.reverse();
        assert!(reversed_outer.validate().is_err());
        let mut reversed_hole = output.clone();
        if let Some(hole) = reversed_hole.holes.first_mut() {
            hole.reverse();
            assert!(reversed_hole.validate().is_err());
        }
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_intersection(&left, &right) {
        let _ = output.validate_against_sources(
            &left,
            &right,
            CoplanarArrangementOperation::Intersection,
        );
    }
    if let Some(output) = arrange_coplanar_convex_surface_intersection(&right, &left) {
        let _ = output.validate_against_sources(
            &right,
            &left,
            CoplanarArrangementOperation::Intersection,
        );
    }
    if let Some(output) = arrange_coplanar_convex_surface_multi_intersection(&left, &right) {
        let _ = output.validate_intersection_against_sources(&left, &right);
        let mut reversed_component = output.clone();
        if let Some(component) = reversed_component.polygons.first_mut() {
            component.reverse();
            assert!(reversed_component.validate().is_err());
        }
        if let Some(mesh) =
            cross_component_surface_mesh(&output.mesh, output.polygons.first().map(Vec::len))
        {
            let mut cross_component = output.clone();
            cross_component.mesh = mesh;
            assert!(cross_component.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_multi_intersection(&right, &left) {
        let _ = output.validate_intersection_against_sources(&right, &left);
        let mut reversed_component = output.clone();
        if let Some(component) = reversed_component.polygons.first_mut() {
            component.reverse();
            assert!(reversed_component.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_union(&left, &right) {
        let _ = output.validate_against_sources(&left, &right, CoplanarArrangementOperation::Union);
    }
    if let Some(output) = arrange_coplanar_convex_surface_union(&right, &left) {
        let _ = output.validate_against_sources(&right, &left, CoplanarArrangementOperation::Union);
    }
    if let Some(output) = arrange_coplanar_convex_surface_difference(&left, &right) {
        let _ = output.validate_against_sources(
            &left,
            &right,
            CoplanarArrangementOperation::Difference,
        );
    }
    if let Some(output) = arrange_coplanar_convex_surface_difference(&right, &left) {
        let _ = output.validate_against_sources(
            &right,
            &left,
            CoplanarArrangementOperation::Difference,
        );
    }
    if let Some(output) = arrange_coplanar_convex_surface_multi_difference(&left, &right) {
        let _ = output.validate_against_sources(&left, &right);
        let mut reversed_component = output.clone();
        if let Some(component) = reversed_component.polygons.first_mut() {
            component.reverse();
            assert!(reversed_component.validate().is_err());
        }
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) =
            cross_component_surface_mesh(&output.mesh, output.polygons.first().map(Vec::len))
        {
            let mut cross_component = output.clone();
            cross_component.mesh = mesh;
            assert!(cross_component.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
    }
    if let Some(output) = arrange_coplanar_convex_surface_multi_difference(&right, &left) {
        let _ = output.validate_against_sources(&right, &left);
        let mut reversed_component = output.clone();
        if let Some(component) = reversed_component.polygons.first_mut() {
            component.reverse();
            assert!(reversed_component.validate().is_err());
        }
        if let Some(mesh) = reversed_surface_mesh(&output.mesh) {
            let mut reversed_mesh = output.clone();
            reversed_mesh.mesh = mesh;
            assert!(reversed_mesh.validate().is_err());
        }
        if let Some(mesh) =
            cross_component_surface_mesh(&output.mesh, output.polygons.first().map(Vec::len))
        {
            let mut cross_component = output.clone();
            cross_component.mesh = mesh;
            assert!(cross_component.validate().is_err());
        }
        if let Some(mesh) = first_triangle_only_surface_mesh(&output.mesh) {
            let mut isolated_vertex = output.clone();
            isolated_vertex.mesh = mesh;
            assert!(isolated_vertex.validate().is_err());
        }
        if let Some(mesh) = boundary_mismatched_surface_mesh(&output.mesh) {
            let mut bad_boundary = output.clone();
            bad_boundary.mesh = mesh;
            assert!(bad_boundary.validate().is_err());
        }
    }
    let _ = classify_mesh_vertices_against_convex_solid(&left, &right);
    let _ = classify_mesh_vertices_against_convex_solid(&right, &left);
    let _ = certify_convex_solid(&left)
        .validate()
        .and_then(|_| certify_convex_solid(&left).validate_against_source(&left));
    let _ = certify_convex_solid(&right)
        .validate()
        .and_then(|_| certify_convex_solid(&right).validate_against_source(&right));
    if let Some(intersection) = intersect_closed_convex_solids(&left, &right) {
        let _ = intersection.validate();
        let _ = intersection.validate_against_sources(&left, &right);
    }
    if let Some(intersection) = intersect_closed_convex_solids(&right, &left) {
        let _ = intersection.validate();
        let _ = intersection.validate_against_sources(&right, &left);
    }
    if let Some(difference) = subtract_closed_convex_solids_single_cap(&left, &right) {
        let _ = difference.validate();
        let _ = difference.validate_against_sources(&left, &right);
    }
    if let Some(difference) = subtract_closed_convex_solids_single_cap(&right, &left) {
        let _ = difference.validate();
        let _ = difference.validate_against_sources(&right, &left);
    }
    if let Some(point) = left.vertices().first() {
        let point = point.to_hyperlimit_point();
        let _ = classify_point_against_convex_solid_report(&point, &right)
            .validate_against_sources(&point, &right);
    }
    let _ = classify_mesh_vertices_against_convex_solid_report(&left, &right)
        .validate()
        .and_then(|_| {
            classify_mesh_vertices_against_convex_solid_report(&left, &right)
                .validate_against_sources(&left, &right)
        });
    let _ = classify_mesh_vertices_against_convex_solid_report(&right, &left)
        .validate()
        .and_then(|_| {
            classify_mesh_vertices_against_convex_solid_report(&right, &left)
                .validate_against_sources(&right, &left)
        });

    if left.triangles().len() <= 4 && right.triangles().len() <= 4 {
        let _ = build_selected_region_mesh(
            &left,
            &right,
            ExactRegionSelection::KeepAll,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .map(|mesh| mesh.validate_retained_state());
        let _ = boolean_selected_regions(&left, &right, ExactBooleanPolicy::KEEP_ALL_BOUNDARY).map(
            |result| {
                let _ = result.validate();
                let _ = result.validate_against_sources(&left, &right);
                let _ = result.assembly.validate_against_sources(
                    &left,
                    &right,
                    ExactRegionSelection::KeepAll,
                );
                for classification in &result.region_classifications {
                    let _ = classification.validate_against_sources(&left, &right);
                }
                for triangulation in &result.triangulations {
                    let _ = triangulation.validate_against_sources(&left, &right);
                }
                let mut unknown_graph = result;
                unknown_graph.graph_had_unknowns = true;
                assert!(unknown_graph.validate().is_err());
                let mut missing_classifications = unknown_graph.clone();
                missing_classifications.graph_had_unknowns = false;
                missing_classifications.region_classifications.clear();
                assert!(missing_classifications.validate().is_err());
                let mut missing_triangulations = missing_classifications;
                missing_triangulations.region_classifications =
                    unknown_graph.region_classifications.clone();
                missing_triangulations.triangulations.clear();
                assert!(missing_triangulations.validate().is_err());
                let mut duplicated_triangulation = unknown_graph.clone();
                if let Some(triangulation) =
                    duplicated_triangulation.triangulations.first().cloned()
                {
                    duplicated_triangulation.triangulations.push(triangulation);
                    assert!(duplicated_triangulation.validate().is_err());
                }
                let mut unclassified_triangulation = unknown_graph;
                let mut same_side_classification = unclassified_triangulation.clone();
                if let Some(classification) =
                    same_side_classification.region_classifications.first_mut()
                {
                    classification.plane_side = classification.region_side;
                    assert!(same_side_classification.validate().is_err());
                }
                let mut undecided_classification = unclassified_triangulation.clone();
                if let Some(classification) =
                    undecided_classification.region_classifications.first_mut()
                {
                    classification.relation = FaceRegionPlaneRelation::Unknown;
                    classification.node_sides.fill(None);
                    assert!(undecided_classification.validate().is_err());
                }
                let mut duplicated_classification = unclassified_triangulation.clone();
                if let Some(classification) = duplicated_classification
                    .region_classifications
                    .first()
                    .cloned()
                {
                    duplicated_classification
                        .region_classifications
                        .push(classification);
                    assert!(duplicated_classification.validate().is_err());
                }
                let mut orphaned_classification = unclassified_triangulation.clone();
                if let Some(mut classification) = orphaned_classification
                    .region_classifications
                    .first()
                    .cloned()
                {
                    classification.region_face = usize::MAX;
                    orphaned_classification
                        .region_classifications
                        .push(classification);
                    assert!(orphaned_classification.validate().is_err());
                }
                let mut untriangulated_assembly = unclassified_triangulation.clone();
                if let Some(triangle) = untriangulated_assembly.assembly.triangles.first_mut() {
                    triangle.source_face = usize::MAX;
                    assert!(untriangulated_assembly.validate().is_err());
                }
                let mut outside_triangulation = unclassified_triangulation.clone();
                if let Some(&vertex) = outside_triangulation
                    .assembly
                    .triangles
                    .first()
                    .and_then(|triangle| triangle.vertices.first())
                {
                    let point = outside_triangulation.assembly.vertices[vertex]
                        .point
                        .clone();
                    outside_triangulation.assembly.vertices[vertex].source =
                        FaceSplitBoundaryNode::OriginalVertex {
                            vertex: usize::MAX,
                            point,
                        };
                    assert!(outside_triangulation.validate().is_err());
                }
                let mut reversed_orientation = unclassified_triangulation.clone();
                if let Some(triangle) = reversed_orientation.assembly.triangles.first_mut() {
                    triangle.vertices.swap(1, 2);
                    assert!(reversed_orientation.assembly.validate().is_ok());
                    assert!(
                        reversed_orientation
                            .assembly
                            .validate_source_face_incidence(&left, &right)
                            .is_err()
                    );
                    assert!(
                        reversed_orientation
                            .validate_against_sources(&left, &right)
                            .is_err()
                    );
                    assert!(
                        reversed_orientation
                            .assembly
                            .checked_to_exact_mesh_with_sources(
                                &left,
                                &right,
                                ValidationPolicy::ALLOW_BOUNDARY,
                            )
                            .is_err()
                    );
                }
                let mut unreferenced_vertex = unclassified_triangulation.clone();
                if let Some(vertex) = unreferenced_vertex.assembly.vertices.first().cloned() {
                    unreferenced_vertex.assembly.vertices.push(vertex);
                    assert!(unreferenced_vertex.assembly.validate().is_err());
                    assert!(
                        unreferenced_vertex
                            .assembly
                            .to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
                            .is_err()
                    );
                    assert!(unreferenced_vertex.validate().is_err());
                }
                let mut mismatched_mesh = unclassified_triangulation.clone();
                let mut mesh_vertices = mismatched_mesh.mesh.vertices().to_vec();
                if let Some(vertex) = mesh_vertices.first_mut() {
                    *vertex = hypermesh::exact::ExactPoint3::new(
                        hypermesh::exact::ExactReal::from(99),
                        hypermesh::exact::ExactReal::from(0),
                        hypermesh::exact::ExactReal::from(0),
                    );
                    if let Ok(mesh) = ExactMesh::new_with_policy(
                        mesh_vertices,
                        mismatched_mesh.mesh.triangles().to_vec(),
                        SourceProvenance::exact("fuzz selected-region mesh vertex payload"),
                        ValidationPolicy::ALLOW_BOUNDARY,
                    ) {
                        mismatched_mesh.mesh = mesh;
                        assert!(mismatched_mesh.validate().is_err());
                    }
                }
                let mut mismatched_mesh = unclassified_triangulation.clone();
                let mut mesh_triangles = mismatched_mesh.mesh.triangles().to_vec();
                if let Some(triangle) = mesh_triangles.first_mut() {
                    triangle.0.swap(1, 2);
                    if let Ok(mesh) = ExactMesh::new_with_policy(
                        mismatched_mesh.mesh.vertices().to_vec(),
                        mesh_triangles,
                        SourceProvenance::exact("fuzz selected-region mesh payload"),
                        ValidationPolicy::ALLOW_BOUNDARY,
                    ) {
                        mismatched_mesh.mesh = mesh;
                        assert!(mismatched_mesh.validate().is_err());
                    }
                }
                if let Some(triangulation) = unclassified_triangulation.triangulations.first_mut() {
                    triangulation.face = usize::MAX;
                    assert!(unclassified_triangulation.validate().is_err());
                }
            },
        );
    }
});

#[cfg(feature = "exact-triangulation")]
fn exercise_deterministic_case(selector: u8) {
    const DETERMINISTIC_CASES: u8 = 69;
    match selector % DETERMINISTIC_CASES {
        0 => exercise_partial_convex_union_boundary(),
        1 => exercise_face_interior_steiner_boundary(),
        2 => exercise_multi_component_coplanar_union(),
        3 => exercise_component_coplanar_intersection(),
        4 => exercise_component_coplanar_difference(),
        5 => exercise_boundary_centroid_volumetric_representative(),
        6 => exercise_exhausted_boundary_volumetric_representatives(),
        7 => exercise_closed_coplanar_overlap_boundary_policy(),
        8 => exercise_closed_vertex_touch_boundary_policy(),
        9 => exercise_axis_aligned_coplanar_volumetric_boxes(),
        10 => exercise_axis_aligned_orthogonal_solid_cell_complexes(),
        11 => exercise_affine_coplanar_volumetric_boxes(),
        12 => exercise_affine_orthogonal_solid_cell_complexes(),
        13 => exercise_affine_orthogonal_solid_cell_complex_frame_discovery(),
        14 => exercise_mixed_coplanar_volumetric_materialization(),
        15 => exercise_non_rectilinear_coplanar_volumetric_materialization(),
        16 => exercise_full_face_adjacent_union(),
        17 => exercise_contained_face_adjacent_union(),
        18 => exercise_nonconvex_component_union_loop(),
        19 => exercise_nonconvex_multi_component_union_loop(),
        20 => exercise_contact_opening_with_retained_hole(),
        21 => exercise_independent_contact_openings(),
        22 => exercise_connected_multi_cutter_opening_with_retained_hole(),
        23 => exercise_multiple_side_cutter_openings_with_retained_hole(),
        24 => exercise_consumed_hole_side_cutter_openings(),
        25 => exercise_side_cutter_opening_without_holes(),
        26 => exercise_mixed_consumed_hole_and_side_openings_without_retained_holes(),
        27 => exercise_nonagon_full_face_adjacent_union(),
        28 => exercise_decagon_full_face_adjacent_union(),
        29 => exercise_component_holed_coplanar_union(),
        30 => exercise_disconnected_component_holed_coplanar_union(),
        31 => exercise_two_disk_component_holed_coplanar_union(),
        32 => exercise_overlapping_component_holed_coplanar_union(),
        33 => exercise_nonconvex_overlap_component_holed_coplanar_union(),
        34 => exercise_point_branch_component_holed_coplanar_union(),
        35 => exercise_nonrectangular_component_union_hull_coverage(),
        36 => exercise_holed_coplanar_mesh_containment(),
        37 => exercise_component_holed_coplanar_intersection(),
        38 => exercise_same_outer_component_holed_coplanar_intersection(),
        39 => exercise_same_outer_component_holed_coplanar_intersection_with_island(),
        40 => exercise_same_outer_component_holed_coplanar_difference(),
        41 => exercise_same_outer_holed_coplanar_multi_difference(),
        42 => exercise_same_outer_holed_coplanar_component_difference(),
        43 => exercise_same_outer_holed_coplanar_filled_union(),
        44 => exercise_same_outer_holed_coplanar_retained_union(),
        45 => exercise_exact_boolmesh_bounds_disjoint_port(),
        46 => exercise_exact_boolmesh_kernel12_port(),
        47 => exercise_exact_boolmesh_open_crossing_adjacency_port(),
        48 => exercise_exact_boolmesh_kernel03_no_intersection_port(),
        49 => exercise_exact_boolmesh_kernel12_endpoint_shadow_port(),
        50 => exercise_exact_boolmesh_kernel12_boundary_endpoint_shadow_port(),
        51 => exercise_exact_boolmesh_kernel11_shadow_port(),
        52 => exercise_exact_boolmesh_kernel02_shadow_port(),
        53 => exercise_exact_boolmesh_kernel12_shadow_accumulator_port(),
        54 => exercise_exact_boolmesh_kernel_frame_port(),
        55 => exercise_exact_boolmesh_kernel12_accumulator_replay_port(),
        56 => exercise_exact_boolmesh_kernel12_intersect_loop_port(),
        57 => exercise_exact_boolmesh_kernel12_intersect_halfedge_row_port(),
        58 => exercise_exact_boolmesh_boolean45_halfedge_row_port(),
        59 => exercise_exact_boolmesh_kernel12_intersect_boundary_endpoint_port(),
        60 => exercise_exact_boolmesh_kernel12_coplanar_interval_port(),
        61 => exercise_exact_boolmesh_kernel03_winding_port(),
        62 => exercise_exact_boolmesh_cleanup_materialization_port(),
        63 => exercise_exact_boolmesh_holed_triangulation_port(),
        64 => exercise_exact_boolmesh_positive_area_coplanar_kernel12_port(),
        65 => exercise_exact_boolmesh_boolean45_triangulation_port(),
        66 => exercise_exact_boolmesh_source_edge_cleanup_port(),
        67 => exercise_exact_boolmesh_boundary_closure_cleanup_port(),
        68 => exercise_same_outer_source_island_point_touch_replay(),
        _ => exercise_nonconvex_coplanar_volumetric_difference_fan_split(),
    }
}

#[cfg(not(feature = "exact-triangulation"))]
fn exercise_deterministic_case(_: u8) {
    exercise_partial_convex_union_boundary();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_bounds_disjoint_port() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh left fixture must import");
    let right = ExactMesh::from_i64_triangles(
        &[10, 0, 0, 12, 0, 0, 10, 2, 0, 10, 0, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh right fixture must import");

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let workspace = hypermesh::exact::exact_boolmesh_workspace(&left, &right, operation);
        workspace.validate_against_sources(&left, &right).unwrap();
        assert!(workspace.is_certified_bounds_disjoint());
        let size_output = workspace
            .boolean45
            .as_ref()
            .expect("bounds-disjoint boolmesh workspace must size output");
        assert_eq!(size_output.inserted_intersection_vertices, 0);
        assert_eq!(size_output.source_edge_incident_gaps, 0);
        assert!(size_output.face_halfedge_offsets.windows(2).all(|window| window[0] <= window[1]));
        assert_eq!(
            size_output.vertex_allocation.output_vertex_origins.len(),
            size_output.vertices_from_left
                + size_output.vertices_from_right
                + size_output.inserted_intersection_vertices
        );
        assert!(size_output.new_edge_vertices.source_edge_runs.is_empty());
        assert!(size_output.new_edge_vertices.face_pair_runs.is_empty());
        assert!(size_output.partial_source_edges.source_edge_runs.is_empty());
        assert!(size_output.new_face_pair_edges.face_pair_runs.is_empty());
        match operation {
            ExactBooleanOperation::Union => {
                assert_eq!(size_output.whole_source_edges.source_edge_runs.len(), 12);
                assert_eq!(
                    size_output
                        .whole_source_edges
                        .source_edge_runs
                        .iter()
                        .map(|run| run.fragments.len())
                        .sum::<usize>(),
                    12
                );
                assert!(size_output.whole_source_edges.source_edge_runs.iter().all(
                    |run| run.incident_faces.len() == 2
                        && run.incident_edges.len() == 2
                        && run.source_halfedge / 3 == run.incident_faces[0]
                        && run.incident_edges[0] == run.edge
                ));
                assert_eq!(size_output.halfedge_assembly.emitted_pairs, 12);
                assert_eq!(size_output.halfedge_assembly.unfilled_halfedges, 0);
                assert_eq!(size_output.face_loop_assembly.loops.len(), 8);
                assert_eq!(size_output.face_loop_assembly.incomplete_faces, 0);
                assert_eq!(size_output.loop_triangulation.triangulations.len(), 8);
                assert_eq!(size_output.loop_triangulation.triangulation_failures, 0);
            }
            ExactBooleanOperation::Intersection => {
                assert!(size_output.whole_source_edges.source_edge_runs.is_empty());
                assert_eq!(size_output.halfedge_assembly.emitted_pairs, 0);
                assert_eq!(size_output.halfedge_assembly.unfilled_halfedges, 0);
                assert!(size_output.face_loop_assembly.loops.is_empty());
                assert!(size_output.loop_triangulation.triangulations.is_empty());
            }
            ExactBooleanOperation::Difference => {
                assert_eq!(size_output.whole_source_edges.source_edge_runs.len(), 6);
                assert!(size_output.whole_source_edges.source_edge_runs.iter().all(
                    |run| run.side == hypermesh::exact::ExactBoolMeshSide::Left
                        && run.source_halfedge / 3 == run.incident_faces[0]
                        && run.incident_edges[0] == run.edge
                        && run.fragments.len() == 1
                ));
                assert_eq!(size_output.halfedge_assembly.emitted_pairs, 6);
                assert_eq!(size_output.halfedge_assembly.unfilled_halfedges, 0);
                assert_eq!(size_output.face_loop_assembly.loops.len(), 4);
                assert_eq!(size_output.face_loop_assembly.incomplete_faces, 0);
                assert_eq!(size_output.loop_triangulation.triangulations.len(), 4);
            }
            ExactBooleanOperation::SelectedRegions(_) => unreachable!(),
        }
        assert_eq!(
            size_output.halfedge_assembly.output_halfedges.len(),
            size_output.face_halfedge_offsets.last().copied().unwrap()
        );
        assert_eq!(size_output.halfedge_assembly.face_overflows, 0);
        assert_eq!(size_output.halfedge_assembly.missing_source_face_maps, 0);
        assert_eq!(
            size_output
                .halfedge_assembly
                .emitted_boundary_halfedges,
            0
        );
        assert_eq!(size_output.face_loop_assembly.repeated_halfedges, 0);
        let mut malformed_boundary_count = workspace.clone();
        malformed_boundary_count
            .boolean45
            .as_mut()
            .unwrap()
            .halfedge_assembly
            .emitted_boundary_halfedges += 1;
        assert!(malformed_boundary_count
            .validate_against_sources(&left, &right)
            .is_err());
        if operation == ExactBooleanOperation::Union {
            let mut malformed = workspace.clone();
            malformed
                .boolean45
                .as_mut()
                .unwrap()
                .whole_source_edges
                .source_edge_runs[0]
                .fragments[0]
                .output_head = usize::MAX;
            assert!(malformed.validate_against_sources(&left, &right).is_err());
            let mut malformed_edge_use = workspace.clone();
            malformed_edge_use
                .boolean45
                .as_mut()
                .unwrap()
                .whole_source_edges
                .source_edge_runs[0]
                .incident_edges[0] = [usize::MAX, 0];
            assert!(malformed_edge_use
                .validate_against_sources(&left, &right)
                .is_err());
            let mut malformed_halfedges = workspace.clone();
            malformed_halfedges
                .boolean45
                .as_mut()
                .unwrap()
                .halfedge_assembly
                .output_halfedges[0]
                .as_mut()
                .unwrap()
                .pair = usize::MAX;
            assert!(malformed_halfedges
                .validate_against_sources(&left, &right)
                .is_err());
            let mut malformed_loops = workspace.clone();
            malformed_loops
                .boolean45
                .as_mut()
                .unwrap()
                .face_loop_assembly
                .loops[0]
                .vertices
                .clear();
            assert!(malformed_loops
                .validate_against_sources(&left, &right)
                .is_err());
            let mut malformed_triangulation = workspace.clone();
            malformed_triangulation
                .boolean45
                .as_mut()
                .unwrap()
                .loop_triangulation
                .triangulations[0]
                .triangles[0] = usize::MAX;
            assert!(malformed_triangulation
                .validate_against_sources(&left, &right)
                .is_err());
        }
        let execution = hypermesh::exact::execute_exact_boolmesh_bounds_disjoint(
            &left,
            &right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .expect("deterministic bounds-disjoint boolmesh slice should execute");
        execution.validate_against_sources(&left, &right).unwrap();
    }
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_port() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh kernel12 left fixture must import");
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, -1, 3, 1, 3, 1, 3, 3, 3, 3, 1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh kernel12 right fixture must import");

    let workspace = hypermesh::exact::exact_boolmesh_workspace(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
    );
    workspace.validate_against_sources(&left, &right).unwrap();
    assert!(workspace.blocker.is_none());
    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("deterministic boolmesh kernel12 fixture must materialize after exact cleanup");
    execution.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        execution.shortcut,
        hypermesh::exact::ExactBooleanShortcutKind::BoolMeshSplit
    );
    assert_eq!(execution.mesh.vertices().len(), 5);
    assert_eq!(execution.mesh.triangles().len(), 6);
    assert!(execution.mesh.facts().mesh.closed_manifold);
    assert!(workspace.kernel12_events.iter().any(|event| matches!(
        event.relation,
        SegmentPlaneRelation::ProperCrossing | SegmentPlaneRelation::EndpointOnPlane
    )));
    assert!(
        !workspace.boolean03.p1q2.is_empty() || !workspace.boolean03.p2q1.is_empty(),
        "deterministic kernel12 fixture must lower proper crossings"
    );
    let mut stale_kernel03 = workspace.clone();
    stale_kernel03.boolean03.w03[0] += 1;
    assert!(stale_kernel03
        .validate_against_sources(&left, &right)
        .is_err());
    assert!(workspace
        .boolean03
        .p1q2
        .iter()
        .chain(workspace.boolean03.p2q1.iter())
        .all(|pair| pair.edge[0] < pair.edge[1]));
    assert_eq!(workspace.boolean03.p1q2.len(), 2);
    assert_eq!(workspace.boolean03.p2q1.len(), 4);
    let size_output = workspace
        .boolean45
        .as_ref()
        .expect("kernel12 boolmesh workspace must size output");
    assert_eq!(
        size_output.inserted_intersection_vertices,
        workspace
            .boolean03
            .x12
            .iter()
            .chain(workspace.boolean03.x21.iter())
            .map(|sign| sign.unsigned_abs() as usize)
            .sum::<usize>()
    );
    assert_eq!(size_output.source_edge_incident_gaps, 0);
    assert!(size_output.face_halfedge_offsets.windows(2).all(|window| window[0] <= window[1]));
    assert_eq!(
        size_output.vertex_allocation.output_vertex_origins.len(),
        size_output.vertices_from_left
            + size_output.vertices_from_right
            + size_output.inserted_intersection_vertices
    );
    assert_eq!(
        size_output
            .new_edge_vertices
            .source_edge_runs
            .iter()
            .map(|run| run.points.len())
            .sum::<usize>(),
        size_output.inserted_intersection_vertices
    );
    assert_eq!(
        size_output
            .new_edge_vertices
            .face_pair_runs
            .iter()
            .map(|run| run.points.len())
            .sum::<usize>(),
        size_output.inserted_intersection_vertices * 2
    );
    assert_eq!(
        size_output
            .partial_source_edges
            .source_edge_runs
            .iter()
            .flat_map(|run| run.points.iter())
            .filter(|point| matches!(
                point.origin,
                hypermesh::exact::ExactBoolMeshPartialEdgePointOrigin::RoutedIntersection(_)
            ))
            .count(),
        size_output.inserted_intersection_vertices
    );
    assert_eq!(
        size_output
            .new_face_pair_edges
            .face_pair_runs
            .iter()
            .map(|run| run.points.len())
            .sum::<usize>(),
        size_output.inserted_intersection_vertices * 2
    );
    assert!(size_output
        .new_face_pair_edges
        .face_pair_runs
        .iter()
        .all(|run| run.points.windows(2).all(|window| {
            (
                window[0].order_index,
                window[0].collision,
                window[0].output_vertex,
            ) <= (
                window[1].order_index,
                window[1].collision,
                window[1].output_vertex,
            )
        })));
    assert_eq!(size_output.new_face_pair_edges.unpaired_runs, 0);
    assert!(size_output.whole_source_edges.source_edge_runs.is_empty());
    assert_eq!(
        size_output.halfedge_assembly.output_halfedges.len(),
        size_output.face_halfedge_offsets.last().copied().unwrap()
    );
    assert_eq!(size_output.halfedge_assembly.face_overflows, 0);
    assert_eq!(size_output.halfedge_assembly.missing_source_face_maps, 0);
    assert_eq!(
        size_output.halfedge_assembly.emitted_pairs * 2
            + size_output.halfedge_assembly.emitted_boundary_halfedges
            + size_output.halfedge_assembly.unfilled_halfedges,
        size_output.halfedge_assembly.output_halfedges.len()
    );
    assert_eq!(size_output.face_loop_assembly.repeated_halfedges, 0);
    assert_eq!(size_output.face_loop_assembly.non_loop_halfedges, 0);
    assert!(size_output
        .face_loop_assembly
        .loops
        .iter()
        .all(|face_loop| face_loop.halfedges.len() >= 3
            && face_loop.halfedges.len() == face_loop.vertices.len()));
    assert_eq!(size_output.loop_triangulation.short_loops, 0);
    assert_eq!(size_output.loop_triangulation.missing_source_faces, 0);
    assert_eq!(
        size_output.loop_triangulation.missing_vertex_coordinates,
        0
    );
    assert_eq!(size_output.loop_triangulation.triangulation_failures, 0);
    assert!(size_output
        .loop_triangulation
        .triangulations
        .iter()
        .all(|triangulation| triangulation.triangles.len().is_multiple_of(3)
            && triangulation
                .triangles
                .iter()
                .all(|index| *index < triangulation.vertices.len())));
    assert_eq!(
        workspace
            .pair_up
            .source_edge_runs
            .iter()
            .map(|run| run.events.len())
            .sum::<usize>(),
        workspace.boolean03.p1q2.len() + workspace.boolean03.p2q1.len()
    );
    let mut malformed = workspace.clone();
    malformed
        .boolean45
        .as_mut()
        .unwrap()
        .source_face_to_output_face
        .push(Some(0));
    assert!(malformed.validate_against_sources(&left, &right).is_err());
    let mut malformed_allocation = workspace.clone();
    malformed_allocation
        .boolean45
        .as_mut()
        .unwrap()
        .vertex_allocation
        .output_vertex_origins
        .clear();
    assert!(malformed_allocation
        .validate_against_sources(&left, &right)
        .is_err());
    let mut malformed_edge_points = workspace.clone();
    malformed_edge_points
        .boolean45
        .as_mut()
        .unwrap()
        .new_edge_vertices
        .source_edge_runs[0]
        .points[0]
        .collision = usize::MAX;
    assert!(malformed_edge_points
        .validate_against_sources(&left, &right)
        .is_err());
    let mut malformed_partial_edges = workspace.clone();
    malformed_partial_edges
        .boolean45
        .as_mut()
        .unwrap()
        .partial_source_edges
        .source_edge_runs[0]
        .points[0]
        .output_vertex = usize::MAX;
    assert!(malformed_partial_edges
        .validate_against_sources(&left, &right)
        .is_err());
    let mut malformed_new_edges = workspace.clone();
    malformed_new_edges
        .boolean45
        .as_mut()
        .unwrap()
        .new_face_pair_edges
        .face_pair_runs[0]
        .points[0]
        .collision = usize::MAX;
    assert!(malformed_new_edges
        .validate_against_sources(&left, &right)
        .is_err());
    let mut stale_new_edge_order = workspace.clone();
    stale_new_edge_order
        .boolean45
        .as_mut()
        .unwrap()
        .new_face_pair_edges
        .face_pair_runs[0]
        .points[0]
        .order_index = usize::MAX;
    assert!(stale_new_edge_order
        .validate_against_sources(&left, &right)
        .is_err());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_cleanup_materialization_port() {
    assert!(exact_boolmesh_cleanup_probe_for_internal_fuzz(0));
    assert!(exact_boolmesh_cleanup_probe_for_internal_fuzz(1));

    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic cleanup left fixture must import");
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, -1, 3, 1, 3, 1, 3, 3, 3, 3, 1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic cleanup right fixture must import");

    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("exact cleanup must certify the skew boolmesh split fixture");
    execution.validate_against_sources(&left, &right).unwrap();
    assert_eq!(execution.mesh.vertices().len(), 5);
    assert_eq!(execution.mesh.triangles().len(), 6);
    assert_eq!(execution.mesh.facts().mesh.boundary_edges, 0);
    assert_eq!(execution.mesh.facts().mesh.duplicate_directed_edges, 0);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_holed_triangulation_port() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic holed-triangulation left fixture must import");
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, 0, 2, 1, 0, 1, 2, 0, 1, 1, -1],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic holed-triangulation right fixture must import");

    let workspace = hypermesh::exact::exact_boolmesh_workspace(
        &left,
        &right,
        ExactBooleanOperation::Union,
    );
    workspace.validate_against_sources(&left, &right).unwrap();
    assert!(workspace.blocker.is_none());
    let stage = workspace
        .boolean45
        .as_ref()
        .expect("strict coplanar fixture must reach boolean45");
    assert_eq!(stage.loop_triangulation.multi_loop_faces, 0);
    assert!(stage
        .loop_triangulation
        .triangulations
        .iter()
        .any(|triangulation| triangulation.vertices.len() > 3));

    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("strict coplanar fixture should materialize through holed triangulation");
    execution.validate_against_sources(&left, &right).unwrap();
    assert_eq!(execution.mesh.vertices().len(), 8);
    assert_eq!(execution.mesh.triangles().len(), 12);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_endpoint_shadow_port() {
    let left = tetrahedron_i64([1, 1, 0], [1, 1, 2], [2, 1, 1], [1, 2, 1]);
    let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);

    let workspace =
        hypermesh::exact::exact_boolmesh_workspace(&left, &right, ExactBooleanOperation::Intersection);
    workspace.validate_against_sources(&left, &right).unwrap();
    assert_eq!(workspace.kernel12_unknown_events, 0);
    assert_eq!(workspace.kernel12_construction_failures, 0);
    assert_eq!(workspace.kernel12_coplanar_events, 0);
    assert!(workspace.kernel12_events.iter().any(|event| {
        event.relation == SegmentPlaneRelation::EndpointOnPlane
            && event.endpoint_sides.contains(&Some(PlaneSide::On))
    }));
    assert!(workspace
        .pair_up
        .source_edge_runs
        .iter()
        .flat_map(|run| run.events.iter())
        .any(|event| {
            compare_reals(&event.parameter, &ExactReal::from(0)).value() == Some(Ordering::Equal)
                || compare_reals(&event.parameter, &ExactReal::from(1)).value()
                    == Some(Ordering::Equal)
        }));
    assert!(!workspace.boolean03.p1q2.is_empty() || !workspace.boolean03.p2q1.is_empty());
    assert_eq!(workspace.blocker, None);
    let stage = workspace.boolean45.as_ref().unwrap();
    assert_eq!(stage.loop_triangulation.dropped_degenerate_faces.len(), 4);
    assert!(stage.output_triangles.triangles.is_empty());
    assert!(stage.mesh_export.triangles.is_empty());

    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("endpoint-only intersection should materialize as an empty boolmesh split");
    execution.validate_against_sources(&left, &right).unwrap();
    assert!(execution.mesh.triangles().is_empty());
    assert!(execution.mesh.vertices().is_empty());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_boundary_endpoint_shadow_port() {
    let left = tetrahedron_i64([2, 0, 0], [2, 0, 2], [3, 1, 1], [1, 1, 1]);
    let right = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);

    let workspace =
        hypermesh::exact::exact_boolmesh_workspace(&left, &right, ExactBooleanOperation::Intersection);
    workspace.validate_against_sources(&left, &right).unwrap();
    assert_eq!(workspace.kernel12_unknown_events, 0);
    assert_eq!(workspace.kernel12_construction_failures, 0);
    assert_eq!(workspace.kernel12_coplanar_events, 0);
    assert!(workspace.kernel12_events.iter().any(|event| {
        event.relation == SegmentPlaneRelation::EndpointOnPlane
            && event.endpoint_sides.contains(&Some(PlaneSide::On))
            && event
                .point
                .as_ref()
                .is_some_and(|point| compare_reals(&point.y, &ExactReal::from(0)).value() == Some(Ordering::Equal))
    }));
    assert_eq!(workspace.blocker, None);
    let stage = workspace.boolean45.as_ref().unwrap();
    assert_eq!(stage.loop_triangulation.dropped_degenerate_faces.len(), 5);
    assert!(stage.output_triangles.triangles.is_empty());
    assert!(stage.mesh_export.triangles.is_empty());

    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("boundary endpoint-only intersection should materialize as an empty boolmesh split");
    execution.validate_against_sources(&left, &right).unwrap();
    assert!(execution.mesh.triangles().is_empty());
    assert!(execution.mesh.vertices().is_empty());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel11_shadow_port() {
    assert!(exact_boolmesh_kernel11_shadow_probe_for_internal_fuzz(51));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel02_shadow_port() {
    assert!(exact_boolmesh_kernel02_shadow_probe_for_internal_fuzz(52));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_shadow_accumulator_port() {
    assert!(exact_boolmesh_kernel12_shadow_accumulator_probe_for_internal_fuzz(53));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel_frame_port() {
    assert!(exact_boolmesh_kernel_frame_probe_for_internal_fuzz(54));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_accumulator_replay_port() {
    assert!(exact_boolmesh_kernel12_accumulator_replay_probe_for_internal_fuzz(55));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_intersect_loop_port() {
    assert!(exact_boolmesh_kernel12_intersect_loop_probe_for_internal_fuzz(56));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_intersect_halfedge_row_port() {
    assert!(exact_boolmesh_kernel12_intersect_loop_probe_for_internal_fuzz(58));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_intersect_boundary_endpoint_port() {
    assert!(exact_boolmesh_kernel12_intersect_loop_probe_for_internal_fuzz(60));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel12_coplanar_interval_port() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic boolmesh coplanar interval left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 2, 0, 0, 1, -2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic boolmesh coplanar interval right fixture must import");
    let workspace = hypermesh::exact::exact_boolmesh_workspace(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
    );
    workspace.validate_against_sources(&left, &right).unwrap();
    assert_eq!(workspace.kernel12_coplanar_events, 0);
    assert_eq!(workspace.blocker, None);
    assert!(workspace.boolean03.p1q2.len() >= 2);
    assert!(workspace.boolean03.p2q1.len() >= 2);
    let stage = workspace.boolean45.as_ref().unwrap();
    assert_eq!(stage.partial_source_edges.unpaired_runs, 0);
    assert_eq!(stage.new_face_pair_edges.unpaired_runs, 0);
    assert_eq!(stage.halfedge_assembly.unfilled_halfedges, 0);
    assert_eq!(stage.face_loop_assembly.dropped_open_chain_halfedges, 2);
    assert!(stage.output_triangles.triangles.is_empty());
    assert!(workspace
        .pair_up
        .source_edge_runs
        .iter()
        .flat_map(|run| run.events.iter())
        .any(|event| matches!(
            event.point,
            hypermesh::exact::ExactBoolMeshPointConstruction::EdgeParameter { .. }
        )));
    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("boundary-only interval should materialize as an empty boolmesh split");
    execution.validate_against_sources(&left, &right).unwrap();
    assert!(execution.mesh.triangles().is_empty());
    assert!(execution.mesh.vertices().is_empty());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_positive_area_coplanar_kernel12_port() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = tetrahedron_i64([2, -1, 0], [5, -1, 0], [2, 2, 0], [2, -1, -3]);
    let workspace =
        hypermesh::exact::exact_boolmesh_workspace(&left, &right, ExactBooleanOperation::Union);
    workspace.validate_against_sources(&left, &right).unwrap();
    assert_eq!(workspace.kernel12_coplanar_events, 0);
    assert!(workspace.blocker.is_none());
    let stage = workspace
        .boolean45
        .as_ref()
        .expect("positive-area coplanar rows must reach boolean45");
    assert_eq!(stage.halfedge_assembly.unfilled_halfedges, 0);
    assert_eq!(stage.face_loop_assembly.incomplete_faces, 0);
    assert_eq!(stage.face_loop_assembly.non_loop_halfedges, 0);
    assert_eq!(stage.loop_triangulation.short_loops, 0);
    assert_eq!(stage.loop_triangulation.triangulation_failures, 0);
    assert!(
        stage
            .loop_triangulation
            .triangulations
            .iter()
            .any(|triangulation| triangulation.output_face == 0
                && triangulation.clipped_loop_indices.contains(&2))
    );
    assert_eq!(stage.output_triangles.missing_loop_triangulations, 0);
    assert_eq!(stage.mesh_export.blocked_output_triangles, 0);
    assert!(workspace.boolean03.x12.iter().any(|sign| *sign < 0));
    assert!(workspace.boolean03.x21.iter().any(|sign| *sign < 0));
    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("positive-area coplanar boolmesh fixture should materialize");
    execution.validate_against_sources(&left, &right).unwrap();
    assert_eq!(execution.mesh.vertices().len(), 9);
    assert_eq!(execution.mesh.triangles().len(), 14);

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("completed positive-area boolmesh split should preflight");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(preflight.support, ExactBooleanSupport::CertifiedBoolMeshSplit);
    assert!(preflight.blocker.is_none());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_source_edge_cleanup_port() {
    let left = upward_l_prism_i64([[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]], 5);
    let right = tetrahedron_i64([1, 1, 0], [7, 1, 0], [1, 7, 0], [1, 1, 5]);

    let workspace =
        hypermesh::exact::exact_boolmesh_workspace(&left, &right, ExactBooleanOperation::Union);
    workspace.validate_against_sources(&left, &right).unwrap();
    assert!(workspace.blocker.is_none());
    let stage = workspace
        .boolean45
        .as_ref()
        .expect("source-edge cleanup fixture must reach boolean45");
    assert_eq!(stage.face_loop_assembly.dropped_open_chain_halfedges, 23);
    assert_eq!(stage.partial_source_edges.unpaired_runs, 0);
    assert_eq!(stage.new_face_pair_edges.unpaired_runs, 0);
    assert_eq!(stage.halfedge_assembly.unfilled_halfedges, 0);
    assert_eq!(stage.face_loop_assembly.non_loop_halfedges, 0);
    assert_eq!(stage.loop_triangulation.triangulation_failures, 0);
    assert_eq!(stage.mesh_export.triangles.len(), 29);

    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("source-edge cleanup fixture should materialize through boolmesh");
    execution.validate_against_sources(&left, &right).unwrap();
    assert_eq!(execution.mesh.vertices().len(), 19);
    assert_eq!(execution.mesh.triangles().len(), 34);
    assert_eq!(execution.mesh.facts().mesh.boundary_edges, 0);
    assert_eq!(execution.mesh.facts().mesh.non_manifold_vertices, 0);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_boundary_closure_cleanup_port() {
    let left = upward_l_prism_i64([[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]], 5);
    let right = tetrahedron_i64([2, -1, 0], [5, -1, 0], [2, 2, 0], [2, -1, -3]);

    let workspace =
        hypermesh::exact::exact_boolmesh_workspace(&left, &right, ExactBooleanOperation::Union);
    workspace.validate_against_sources(&left, &right).unwrap();
    assert!(workspace.blocker.is_none());
    let stage = workspace
        .boolean45
        .as_ref()
        .expect("boundary closure fixture must reach boolean45");
    assert_eq!(stage.face_loop_assembly.dropped_open_chain_halfedges, 32);
    assert_eq!(stage.face_loop_assembly.non_loop_halfedges, 0);
    assert_eq!(stage.mesh_export.triangles.len(), 18);
    assert_eq!(stage.mesh_export.boundary_edges.len(), 8);
    assert_eq!(stage.mesh_export.boundary_closure_records.len(), 8);
    assert!(
        stage
            .face_loop_assembly
            .dropped_open_chains
            .iter()
            .any(|chain| chain.source_kind
                == hypermesh::exact::ExactBoolMeshDroppedOpenChainSourceKind::Mixed)
    );

    let execution = hypermesh::exact::execute_exact_boolmesh_port(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("boundary closure fixture should materialize through boolmesh");
    execution.validate_against_sources(&left, &right).unwrap();
    assert_eq!(execution.mesh.facts().mesh.boundary_edges, 0);
    assert_eq!(execution.mesh.facts().mesh.duplicate_directed_edges, 0);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel03_winding_port() {
    assert!(exact_boolmesh_kernel03_winding_probe_for_internal_fuzz(61));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_boolean45_triangulation_port() {
    assert!(exact_boolmesh_boolean45_triangulation_probe_for_internal_fuzz(65));
    assert!(exact_boolmesh_boolean45_triangulation_probe_for_internal_fuzz(66));
    assert!(exact_boolmesh_boolean45_triangulation_probe_for_internal_fuzz(67));
    assert!(exact_boolmesh_boolean45_triangulation_probe_for_internal_fuzz(68));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_boolean45_halfedge_row_port() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 1, 1, 5, 2, 1, 5],
        &[2, 0, 1],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic boolean45 halfedge-row source fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 4, 4, 0, 4, 0, 4, 4],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic boolean45 halfedge-row opposite fixture must import");
    let workspace = hypermesh::exact::exact_boolmesh_workspace(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
    );
    workspace.validate_against_sources(&left, &right).unwrap();
    assert!(workspace
        .pair_up
        .source_edge_runs
        .iter()
        .any(|run| run.source_halfedge == 1 && run.tail == 0 && run.head == 1));
    let size_output = workspace
        .boolean45
        .as_ref()
        .expect("halfedge-row crossing must reach boolean45");
    assert!(size_output
        .new_edge_vertices
        .source_edge_runs
        .iter()
        .any(|run| run.source_halfedge == 1 && run.tail == 0 && run.head == 1));
    assert!(size_output
        .partial_source_edges
        .source_edge_runs
        .iter()
        .any(|run| run.source_halfedge == 1
            && run.incident_faces == vec![0]
            && run.incident_edges == vec![[0, 1]]));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_kernel03_no_intersection_port() {
    let inner = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 2, 1, 1, 1, 2, 1, 1, 1, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh kernel03 inner fixture must import");
    let outer = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh kernel03 outer fixture must import");

    for (operation, left_vertices, right_vertices, output_faces) in [
        (
            ExactBooleanOperation::Union,
            0,
            outer.vertices().len(),
            outer.triangles().len(),
        ),
        (
            ExactBooleanOperation::Intersection,
            inner.vertices().len(),
            0,
            inner.triangles().len(),
        ),
        (ExactBooleanOperation::Difference, 0, 0, 0),
    ] {
        let workspace = hypermesh::exact::exact_boolmesh_workspace(&inner, &outer, operation);
        workspace.validate_against_sources(&inner, &outer).unwrap();
        assert!(workspace.blocker.is_none());
        assert!(!workspace.is_certified_bounds_disjoint());
        assert!(workspace.is_certified_no_intersection_kernel03());
        assert_eq!(workspace.boolean03.w03, vec![1; inner.vertices().len()]);
        assert_eq!(workspace.boolean03.w30, vec![0; outer.vertices().len()]);
        assert!(workspace.boolean03.p1q2.is_empty());
        assert!(workspace.boolean03.p2q1.is_empty());
        let size_output = workspace
            .boolean45
            .as_ref()
            .expect("kernel03 no-intersection workspace must size output");
        assert_eq!(size_output.vertices_from_left, left_vertices);
        assert_eq!(size_output.vertices_from_right, right_vertices);
        assert_eq!(size_output.inserted_intersection_vertices, 0);
        assert_eq!(size_output.source_edge_incident_gaps, 0);
        assert_eq!(size_output.mesh_export.triangles.len(), output_faces);
        assert_eq!(size_output.halfedge_assembly.emitted_boundary_halfedges, 0);
        let execution = hypermesh::exact::execute_exact_boolmesh_port(
            &inner,
            &outer,
            operation,
            ValidationPolicy::CLOSED,
        )
        .expect("kernel03 no-intersection boolmesh port should execute");
        execution.validate_against_sources(&inner, &outer).unwrap();
        assert_eq!(
            execution.shortcut,
            hypermesh::exact::ExactBooleanShortcutKind::WindingContainment
        );
        assert_eq!(execution.mesh.triangles().len(), output_faces);

        let mut stale_winding = workspace.clone();
        stale_winding.boolean03.w03[0] = 0;
        assert!(stale_winding
            .validate_against_sources(&inner, &outer)
            .is_err());
    }

    let separated_left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh separated-left fixture must import");
    let separated_right = ExactMesh::from_i64_triangles(
        &[4, 4, 4, 4, 0, 4, 0, 4, 4, 4, 4, 0],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic boolmesh separated-right fixture must import");

    for (operation, output_faces) in [
        (
            ExactBooleanOperation::Union,
            separated_left.triangles().len() + separated_right.triangles().len(),
        ),
        (ExactBooleanOperation::Intersection, 0),
        (
            ExactBooleanOperation::Difference,
            separated_left.triangles().len(),
        ),
    ] {
        let workspace =
            hypermesh::exact::exact_boolmesh_workspace(&separated_left, &separated_right, operation);
        workspace
            .validate_against_sources(&separated_left, &separated_right)
            .unwrap();
        assert!(workspace.blocker.is_none());
        assert!(!workspace.is_certified_bounds_disjoint());
        assert!(workspace.is_certified_no_intersection_kernel03());
        assert_eq!(
            workspace.boolean03.w03,
            vec![0; separated_left.vertices().len()]
        );
        assert_eq!(
            workspace.boolean03.w30,
            vec![0; separated_right.vertices().len()]
        );
        let execution = hypermesh::exact::execute_exact_boolmesh_port(
            &separated_left,
            &separated_right,
            operation,
            ValidationPolicy::CLOSED,
        )
        .expect("separated overlapping-AABB boolmesh port should execute");
        execution
            .validate_against_sources(&separated_left, &separated_right)
            .unwrap();
        assert_eq!(
            execution.shortcut,
            hypermesh::exact::ExactBooleanShortcutKind::WindingSeparated
        );
        assert_eq!(execution.mesh.triangles().len(), output_faces);
    }
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exact_boolmesh_open_crossing_adjacency_port() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic open boolmesh left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, -1, -1, 1, 3, -1, 1, -1, 1, 3, -1, -1, 3, 3, -1, 3, -1, 1,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("deterministic open boolmesh right fixture must import");

    let workspace = hypermesh::exact::exact_boolmesh_workspace(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
    );
    workspace.validate_against_sources(&left, &right).unwrap();
    let size_output = workspace
        .boolean45
        .as_ref()
        .expect("open crossing boolmesh workspace must size output");
    assert!(size_output
        .partial_source_edges
        .source_edge_runs
        .iter()
        .any(|run| run.incident_faces.len() == 1 && !run.points.is_empty()));
    assert_eq!(size_output.source_edge_incident_gaps, 0);
    assert_eq!(size_output.halfedge_assembly.source_edge_incident_gaps, 0);
    assert_eq!(
        size_output
            .new_edge_vertices
            .face_pair_runs
            .iter()
            .map(|run| run.points.len())
            .sum::<usize>(),
        size_output
            .partial_source_edges
            .source_edge_runs
            .iter()
            .map(|run| {
                run.points
                    .iter()
                    .filter(|point| {
                        matches!(
                            point.origin,
                            hypermesh::exact::ExactBoolMeshPartialEdgePointOrigin::RoutedIntersection(_)
                        )
                    })
                    .count()
                    * run.incident_faces.len()
            })
            .sum::<usize>()
    );
}

fn exercise_partial_convex_union_boundary() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0, 0, 0, 4],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic convex left fixture must import");
    let right = ExactMesh::from_i64_triangles(
        &[1, 1, 1, 5, 1, 1, 1, 5, 1, 1, 1, 5],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("deterministic convex right fixture must import");
    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("partial convex union preflight should classify exact face cells");
    preflight
        .validate()
        .expect("preflight report must validate");
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedWindingMaterialized
    );
    let graph = build_intersection_graph(&left, &right).expect("fixture graph should build");
    let (_regions, cells) =
        hypermesh::exact::triangulate_all_face_cells_with_cdt(&graph, &left, &right)
            .expect("fixture cell triangulation should not error")
            .expect("fixture should produce exact planar cells");
    assert!(cells.iter().any(|cell| {
        cell.side == hypermesh::exact::MeshSide::Left
            && cell.face == 2
            && cell.triangles.len() / 3 == 7
    }));
    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("partial convex union should materialize from exact winding cells");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .expect("winding-materialized union should replay");
    assert!(result.mesh.facts().mesh.closed_manifold);
    let mut missing_volumetric = result;
    missing_volumetric.volumetric_classifications.clear();
    assert!(matches!(
        missing_volumetric.validate(),
        Err(hypermesh::exact::ExactReportValidationError::MissingVolumetricClassifications)
    ));
}

#[cfg(feature = "exact-triangulation")]
fn exercise_multi_component_coplanar_union() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
            10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component coplanar union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 0, 0, 3, 0, 0, 3, 2, 0, 1, 2, 0, //
            11, 0, 0, 13, 0, 0, 13, 2, 0, 11, 2, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component coplanar union right fixture must import");

    let union = arrange_coplanar_convex_surface_multi_union(&left, &right)
        .expect("fixture should materialize as two retained union components");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    let mut invalid = union.clone();
    invalid.polygons[0].reverse();
    assert!(invalid.validate().is_err());

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("multi-component coplanar union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component coplanar union should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let edge_touch_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("full-edge convex surface left fixture must import");
    let edge_touch_right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("full-edge convex surface right fixture must import");
    let edge_touch_union =
        arrange_coplanar_convex_surface_union(&edge_touch_left, &edge_touch_right)
            .expect("full-edge convex surface union should materialize one loop");
    edge_touch_union.validate().unwrap();
    edge_touch_union
        .validate_against_sources(
            &edge_touch_left,
            &edge_touch_right,
            CoplanarArrangementOperation::Union,
        )
        .unwrap();
    let edge_touch_result = hypermesh::exact::boolean_exact(
        &edge_touch_left,
        &edge_touch_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("full-edge convex surface union should materialize");
    edge_touch_result
        .validate_operation_against_sources(
            &edge_touch_left,
            &edge_touch_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_touch_right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-touching convex surface fixture must import");
    assert!(arrange_coplanar_convex_surface_union(&edge_touch_left, &point_touch_right).is_none());
    let point_touch_union =
        arrange_coplanar_surface_point_touch_union(&edge_touch_left, &point_touch_right)
            .expect("exact vertex-vertex point-touch surface union should materialize");
    point_touch_union.validate().unwrap();
    point_touch_union
        .validate_union_against_sources(&edge_touch_left, &point_touch_right)
        .unwrap();
    let point_touch_preflight = preflight_boolean_exact(
        &edge_touch_left,
        &point_touch_right,
        ExactBooleanOperation::Union,
    )
    .expect("point-touching convex surface preflight should classify certified point-touch union");
    point_touch_preflight.validate().unwrap();
    assert_eq!(
        point_touch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchUnion
    );
    let point_touch_result = hypermesh::exact::boolean_exact(
        &edge_touch_left,
        &point_touch_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-touching convex surface union should materialize");
    point_touch_result
        .validate_operation_against_sources(
            &edge_touch_left,
            &point_touch_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    let point_touch_intersection =
        preflight_boolean_exact(&edge_touch_left, &point_touch_right, ExactBooleanOperation::Intersection)
            .expect("point-touching convex surface intersection should classify empty shortcut");
    point_touch_intersection.validate().unwrap();
    assert_eq!(
        point_touch_intersection.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchIntersection
    );
    let point_touch_difference =
        preflight_boolean_exact(&edge_touch_left, &point_touch_right, ExactBooleanOperation::Difference)
            .expect("point-touching convex surface difference should classify left-preserving shortcut");
    point_touch_difference.validate().unwrap();
    assert_eq!(
        point_touch_difference.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    hypermesh::exact::boolean_exact(
        &edge_touch_left,
        &point_touch_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-touching convex surface intersection should materialize empty");
    hypermesh::exact::boolean_exact(
        &edge_touch_left,
        &point_touch_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-touching convex surface difference should keep left");

    let mixed_contact_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
            8, 4, 0, 10, 4, 0, 10, 6, 0, 8, 6, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed edge/point contact left fixture must import");
    let mixed_contact_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 8, 0, 0, 8, 4, 0, 4, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed edge/point contact right fixture must import");
    assert!(
        arrange_coplanar_surface_component_union(&mixed_contact_left, &mixed_contact_right)
            .is_none()
    );
    let mixed_contact_union =
        arrange_coplanar_surface_point_touch_union(&mixed_contact_left, &mixed_contact_right)
            .expect("mixed edge-connected and point-touching union should materialize");
    mixed_contact_union.validate().unwrap();
    mixed_contact_union
        .validate_union_against_sources(&mixed_contact_left, &mixed_contact_right)
        .unwrap();
    assert_eq!(mixed_contact_union.polygons.len(), 2);
    assert_eq!(
        preflight_boolean_exact(
            &mixed_contact_left,
            &mixed_contact_right,
            ExactBooleanOperation::Union,
        )
        .expect("mixed edge/point union preflight should classify shortcut")
        .support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchUnion
    );
    hypermesh::exact::boolean_exact(
        &mixed_contact_left,
        &mixed_contact_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed edge/point union should materialize")
    .validate()
    .unwrap();

    let mixed_overlap_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 6, 0, 0, 6, 4, 0, 0, 4, 0, //
            8, 4, 0, 10, 4, 0, 10, 6, 0, 8, 6, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed overlap/point contact left fixture must import");
    let mixed_overlap_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 8, 0, 0, 8, 4, 0, 4, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed overlap/point contact right fixture must import");
    assert!(
        arrange_coplanar_surface_component_union(&mixed_overlap_left, &mixed_overlap_right)
            .is_none()
    );
    let mixed_overlap_union =
        arrange_coplanar_surface_point_touch_union(&mixed_overlap_left, &mixed_overlap_right)
            .expect("mixed overlapping and point-touching union should materialize");
    mixed_overlap_union.validate().unwrap();
    mixed_overlap_union
        .validate_union_against_sources(&mixed_overlap_left, &mixed_overlap_right)
        .unwrap();
    assert_eq!(
        preflight_boolean_exact(
            &mixed_overlap_left,
            &mixed_overlap_right,
            ExactBooleanOperation::Union,
        )
        .expect("mixed overlap/point union preflight should classify shortcut")
        .support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchUnion
    );
    hypermesh::exact::boolean_exact(
        &mixed_overlap_left,
        &mixed_overlap_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed overlap/point union should materialize")
    .validate_operation_against_sources(
        &mixed_overlap_left,
        &mixed_overlap_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let vertex_edge_right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 2, 0, 3, 3, 0, 3, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("vertex-edge point contact fixture must import");
    let vertex_edge_union =
        arrange_coplanar_surface_point_touch_union(&edge_touch_left, &vertex_edge_right)
            .expect("vertex-edge point contact should split the touched edge exactly");
    vertex_edge_union.validate().unwrap();
    vertex_edge_union
        .validate_union_against_sources(&edge_touch_left, &vertex_edge_right)
        .unwrap();

    let nonconvex_point_touch_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 10, 0, 0, 10, 4, 0, 7, 4, 0, 6, 6, 0, 10, 8, 0, 10, 12, 0, 0, 12, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 3, 4, //
            0, 4, 7, //
            7, 4, 5, //
            7, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex point-touch left fixture must import");
    let nonconvex_point_touch_right = ExactMesh::from_i64_triangles_with_policy(
        &[10, 12, 0, 12, 12, 0, 12, 14, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex vertex-vertex point-touch fixture must import");
    let nonconvex_point_touch_union = arrange_coplanar_surface_point_touch_union(
        &nonconvex_point_touch_left,
        &nonconvex_point_touch_right,
    )
    .expect("nonconvex branch-only point touch should materialize");
    nonconvex_point_touch_union.validate().unwrap();
    nonconvex_point_touch_union
        .validate_union_against_sources(&nonconvex_point_touch_left, &nonconvex_point_touch_right)
        .unwrap();
    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        preflight_boolean_exact(
            &nonconvex_point_touch_left,
            &nonconvex_point_touch_right,
            operation,
        )
        .expect("nonconvex point-touch preflight should classify")
        .validate()
        .unwrap();
        hypermesh::exact::boolean_exact(
            &nonconvex_point_touch_left,
            &nonconvex_point_touch_right,
            operation,
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("nonconvex point-touch boolean shortcut should materialize")
        .validate()
        .unwrap();
    }
    let nonconvex_vertex_edge_touch_right = ExactMesh::from_i64_triangles_with_policy(
        &[5, 12, 0, 6, 14, 0, 4, 14, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex vertex-edge point-touch fixture must import");
    let nonconvex_vertex_edge_union = arrange_coplanar_surface_point_touch_union(
        &nonconvex_point_touch_left,
        &nonconvex_vertex_edge_touch_right,
    )
    .expect("nonconvex vertex-edge branch contact should materialize");
    nonconvex_vertex_edge_union.validate().unwrap();
    nonconvex_vertex_edge_union
        .validate_union_against_sources(
            &nonconvex_point_touch_left,
            &nonconvex_vertex_edge_touch_right,
        )
        .unwrap();
    let nonconvex_edge_touch_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 12, 0, 6, 12, 0, 6, 14, 0, 4, 14, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex edge-contact fixture must import");
    assert!(arrange_coplanar_surface_point_touch_union(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
    )
    .is_none());
    let nonconvex_edge_union = arrange_coplanar_surface_component_union(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
    )
    .expect("nonconvex source positive edge contact should materialize as component union");
    nonconvex_edge_union.validate().unwrap();
    nonconvex_edge_union
        .validate_component_union_against_sources(
            &nonconvex_point_touch_left,
            &nonconvex_edge_touch_right,
        )
        .unwrap();
    assert!(
        certify_coplanar_surface_boundary_touch(
            &nonconvex_point_touch_left,
            &nonconvex_edge_touch_right,
        )
        .is_some()
    );
    let nonconvex_edge_preflight = preflight_boolean_exact(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
        ExactBooleanOperation::Union,
    )
    .expect("nonconvex source edge-contact union preflight should classify shortcut");
    nonconvex_edge_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_edge_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source edge-contact union should materialize")
    .validate()
    .unwrap();
    let nonconvex_edge_intersection = preflight_boolean_exact(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("nonconvex source edge-contact intersection preflight should classify shortcut");
    nonconvex_edge_intersection.validate().unwrap();
    assert_eq!(
        nonconvex_edge_intersection.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceBoundaryTouchIntersection
    );
    let nonconvex_edge_intersection_result = hypermesh::exact::boolean_exact(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source edge-contact intersection should materialize");
    nonconvex_edge_intersection_result.validate().unwrap();
    assert!(nonconvex_edge_intersection_result.mesh.triangles().is_empty());
    let nonconvex_edge_difference = preflight_boolean_exact(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex source edge-contact difference preflight should classify shortcut");
    nonconvex_edge_difference.validate().unwrap();
    assert_eq!(
        nonconvex_edge_difference.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceBoundaryTouchDifference
    );
    let nonconvex_edge_difference_result = hypermesh::exact::boolean_exact(
        &nonconvex_point_touch_left,
        &nonconvex_edge_touch_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source edge-contact difference should preserve left");
    nonconvex_edge_difference_result.validate().unwrap();
    assert_eq!(
        nonconvex_edge_difference_result.mesh.vertices(),
        nonconvex_point_touch_left.vertices()
    );
    assert_eq!(
        nonconvex_edge_difference_result.mesh.triangles(),
        nonconvex_point_touch_left.triangles()
    );
    let nonconvex_positive_overlap = ExactMesh::from_i64_triangles_with_policy(
        &[4, 10, 0, 8, 10, 0, 8, 14, 0, 4, 14, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex positive-overlap fixture must import");
    assert!(
        certify_coplanar_surface_boundary_touch(
            &nonconvex_point_touch_left,
            &nonconvex_positive_overlap,
        )
        .is_none()
    );
    let nonconvex_overlap_intersection = preflight_boolean_exact(
        &nonconvex_point_touch_left,
        &nonconvex_positive_overlap,
        ExactBooleanOperation::Intersection,
    )
    .expect("positive-overlap preflight should not fail");
    assert_ne!(
        nonconvex_overlap_intersection.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceBoundaryTouchIntersection
    );

    let bridge_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
            4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0, //
            10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("bridged multi-component left fixture must import");
    let bridge_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 0, 0, 5, 0, 0, 5, 2, 0, 1, 2, 0, //
            11, 0, 0, 13, 0, 0, 13, 2, 0, 11, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("bridged multi-component right fixture must import");
    let bridge_union = arrange_coplanar_convex_surface_multi_union(&bridge_left, &bridge_right)
        .expect("bridge strip cluster should materialize with a far output component");
    bridge_union.validate().unwrap();
    bridge_union
        .validate_union_against_sources(&bridge_left, &bridge_right)
        .unwrap();
    let bridge_preflight =
        preflight_boolean_exact(&bridge_left, &bridge_right, ExactBooleanOperation::Union)
            .expect("bridged multi-component coplanar union preflight should classify shortcut");
    bridge_preflight.validate().unwrap();
    assert_eq!(
        bridge_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiUnion
    );
    let bridge_result = hypermesh::exact::boolean_exact(
        &bridge_left,
        &bridge_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("bridged multi-component coplanar union should materialize");
    bridge_result
        .validate_operation_against_sources(
            &bridge_left,
            &bridge_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let single_bridge_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
            4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single bridged strip left fixture must import");
    let single_bridge_right = ExactMesh::from_i64_triangles_with_policy(
        &[1, 0, 0, 5, 0, 0, 5, 2, 0, 1, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single bridged strip right fixture must import");
    let single_bridge_union =
        arrange_coplanar_convex_surface_component_union(&single_bridge_left, &single_bridge_right)
            .expect("single bridge strip cluster should materialize one output loop");
    single_bridge_union.validate().unwrap();
    single_bridge_union
        .validate_against_sources(
            &single_bridge_left,
            &single_bridge_right,
            CoplanarArrangementOperation::Union,
        )
        .unwrap();
    let single_bridge_result = hypermesh::exact::boolean_exact(
        &single_bridge_left,
        &single_bridge_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single bridge strip union should materialize");
    single_bridge_result
        .validate_operation_against_sources(
            &single_bridge_left,
            &single_bridge_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let edge_bridge_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
            4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("edge-bridged strip left fixture must import");
    let edge_bridge_right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("edge-bridged strip right fixture must import");
    let edge_bridge_union =
        arrange_coplanar_convex_surface_component_union(&edge_bridge_left, &edge_bridge_right)
            .expect("full-edge bridge strip cluster should materialize one output loop");
    edge_bridge_union.validate().unwrap();
    edge_bridge_union
        .validate_against_sources(
            &edge_bridge_left,
            &edge_bridge_right,
            CoplanarArrangementOperation::Union,
        )
        .unwrap();
    let edge_bridge_result = hypermesh::exact::boolean_exact(
        &edge_bridge_left,
        &edge_bridge_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("full-edge bridge strip union should materialize");
    edge_bridge_result
        .validate_operation_against_sources(
            &edge_bridge_left,
            &edge_bridge_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_touching_bridge = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-touching bridge rejection fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_union(&edge_bridge_left, &point_touching_bridge)
            .is_none()
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_nonrectangular_component_union_hull_coverage() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 1, 2, 0, //
            4, 0, 0, 6, 0, 0, 5, 2, 0, 4, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("non-rectangular component union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("non-rectangular component union right fixture must import");

    let union = arrange_coplanar_convex_surface_component_union(&left, &right)
        .expect("non-rectangular component tiling should materialize one convex hull");
    union.validate().unwrap();
    union
        .validate_against_sources(&left, &right, CoplanarArrangementOperation::Union)
        .unwrap();
    let mut stale = union.clone();
    stale.polygon.reverse();
    assert!(stale.validate().is_err());

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("non-rectangular component union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("non-rectangular component union should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_holed_coplanar_mesh_containment() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("holed containment outer fixture must import");
    let hole = ExactMesh::from_i64_triangles_with_policy(
        &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("holed containment hole fixture must import");
    let annulus = arrange_coplanar_convex_surface_holed_difference(&outer, &hole)
        .expect("holed containment fixture should materialize an annulus")
        .mesh;
    let cover = ExactMesh::from_i64_triangles_with_policy(
        &[-1, -1, 0, 12, -1, 0, 11, 11, 0, -1, 12, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("holed containment cover fixture must import");

    assert_eq!(
        certify_coplanar_surface_mesh_containment(&annulus, &cover),
        Some(CoplanarSurfaceContainment::LeftInsideRight)
    );
    assert_eq!(
        certify_coplanar_surface_mesh_containment(&cover, &annulus),
        Some(CoplanarSurfaceContainment::RightInsideLeft)
    );
    assert!(
        certify_coplanar_surface_mesh_containment(&hole, &annulus).is_none(),
        "annulus coverage must not fill its retained hole"
    );

    let preflight = preflight_boolean_exact(&annulus, &cover, ExactBooleanOperation::Intersection)
        .expect("holed containment intersection preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&annulus, &cover)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceContainment
    );
    hypermesh::exact::boolean_exact(
        &annulus,
        &cover,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("holed containment intersection should copy the inner source")
    .validate_operation_against_sources(
        &annulus,
        &cover,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_component_holed_coplanar_intersection() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed intersection outer fixture must import");
    let hole = ExactMesh::from_i64_triangles_with_policy(
        &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed intersection hole fixture must import");
    let annulus = arrange_coplanar_convex_surface_holed_difference(&outer, &hole)
        .expect("component-holed intersection fixture should materialize an annulus")
        .mesh;
    let clipper = ExactMesh::from_i64_triangles_with_policy(
        &[2, 1, 0, 9, 2, 0, 8, 9, 0, 1, 8, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed intersection clipper fixture must import");

    let intersection =
        arrange_coplanar_surface_component_holed_intersection(&annulus, &clipper)
            .expect("source-owned clipper should expose the annulus hole");
    intersection.validate().unwrap();
    intersection
        .validate_intersection_against_sources(&annulus, &clipper)
        .unwrap();
    assert_eq!(intersection.components.len(), 1);
    assert_eq!(intersection.components[0].holes.len(), 1);
    assert_eq!(
        arrange_coplanar_surface_component_holed_intersection(&clipper, &annulus),
        Some(intersection)
    );

    let crossing_hole = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 5, 3, 0, 5, 7, 0, 3, 7, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed crossing-hole fixture must import");
    assert!(
        arrange_coplanar_surface_component_holed_intersection(&annulus, &crossing_hole).is_none()
    );

    let no_retained_hole = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed no-hole fixture must import");
    assert!(
        arrange_coplanar_surface_component_holed_intersection(&annulus, &no_retained_hole)
            .is_none()
    );

    let preflight = preflight_boolean_exact(&annulus, &clipper, ExactBooleanOperation::Intersection)
        .expect("component-holed intersection preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&annulus, &clipper)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    hypermesh::exact::boolean_exact(
        &annulus,
        &clipper,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed intersection should materialize")
    .validate_operation_against_sources(
        &annulus,
        &clipper,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_component_holed_coplanar_intersection() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed intersection outer fixture must import");
    let left_hole = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed intersection left hole fixture must import");
    let right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed intersection right hole fixture must import");
    let left = arrange_coplanar_convex_surface_holed_difference(&outer, &left_hole)
        .expect("same-outer left annulus should materialize")
        .mesh;
    let right = arrange_coplanar_convex_surface_holed_difference(&outer, &right_hole)
        .expect("same-outer right annulus should materialize")
        .mesh;

    let intersection = arrange_coplanar_surface_component_holed_intersection(&left, &right)
        .expect("same outer annuli with disjoint holes should materialize");
    intersection.validate().unwrap();
    intersection
        .validate_intersection_against_sources(&left, &right)
        .unwrap();
    assert_eq!(intersection.components.len(), 1);
    assert_eq!(intersection.components[0].holes.len(), 2);
    assert_eq!(
        arrange_coplanar_surface_component_holed_intersection(&right, &left),
        Some(intersection)
    );

    let overlapping_hole = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 5, 3, 0, 5, 6, 0, 3, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer overlapping hole fixture must import");
    let overlapping = arrange_coplanar_convex_surface_holed_difference(&outer, &overlapping_hole)
        .expect("same-outer overlapping annulus should materialize")
        .mesh;
    let overlap_intersection =
        arrange_coplanar_surface_component_holed_intersection(&left, &overlapping)
            .expect("same-outer rectangular overlap should retain one merged hole");
    overlap_intersection.validate().unwrap();
    overlap_intersection
        .validate_intersection_against_sources(&left, &overlapping)
        .unwrap();
    assert_eq!(overlap_intersection.components.len(), 1);
    assert_eq!(overlap_intersection.components[0].holes.len(), 1);
    assert_eq!(overlap_intersection.components[0].holes[0].len(), 8);
    let overlap_preflight =
        preflight_boolean_exact(&left, &overlapping, ExactBooleanOperation::Intersection)
            .expect("same-outer rectangular overlap preflight should classify shortcut");
    overlap_preflight.validate().unwrap();
    overlap_preflight
        .validate_against_sources(&left, &overlapping)
        .unwrap();
    hypermesh::exact::boolean_exact(
        &left,
        &overlapping,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer rectangular overlap boolean should materialize")
    .validate_operation_against_sources(
        &left,
        &overlapping,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let bridge_left_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
            6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer bridge left holes fixture must import");
    let bridge_right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer bridge right hole fixture must import");
    let bridge_left =
        arrange_coplanar_convex_surface_multi_holed_difference(&outer, &bridge_left_holes)
            .expect("same-outer bridge left fixture should materialize")
            .mesh;
    let bridge_right =
        arrange_coplanar_convex_surface_holed_difference(&outer, &bridge_right_hole)
            .expect("same-outer bridge right fixture should materialize")
            .mesh;
    let bridge_intersection =
        arrange_coplanar_surface_component_holed_intersection(&bridge_left, &bridge_right)
            .expect("same-outer rectangular bridge should retain one connected merged hole");
    bridge_intersection.validate().unwrap();
    bridge_intersection
        .validate_intersection_against_sources(&bridge_left, &bridge_right)
        .unwrap();
    assert_eq!(bridge_intersection.components.len(), 1);
    assert_eq!(bridge_intersection.components[0].holes.len(), 1);
    assert!(
        bridge_intersection.components[0].holes[0].len() >= 8,
        "bridge fixture should exercise non-rectangular orthogonal retained-hole output"
    );
    assert_eq!(
        arrange_coplanar_surface_component_holed_intersection(&bridge_right, &bridge_left),
        Some(bridge_intersection)
    );
    let bridge_preflight =
        preflight_boolean_exact(&bridge_left, &bridge_right, ExactBooleanOperation::Intersection)
            .expect("same-outer bridge preflight should classify shortcut");
    bridge_preflight.validate().unwrap();
    bridge_preflight
        .validate_against_sources(&bridge_left, &bridge_right)
        .unwrap();
    hypermesh::exact::boolean_exact(
        &bridge_left,
        &bridge_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer bridge boolean should materialize")
    .validate_operation_against_sources(
        &bridge_left,
        &bridge_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let nonrect_bridge_hole = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 7, 3, 0, 8, 8, 0, 3, 7, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nonrectangular bridge hole fixture must import");
    let nonrect_bridge =
        arrange_coplanar_convex_surface_holed_difference(&outer, &nonrect_bridge_hole)
            .expect("same-outer nonrectangular bridge fixture should materialize")
            .mesh;
    let nonrect_bridge_intersection =
        arrange_coplanar_surface_component_holed_intersection(&bridge_left, &nonrect_bridge)
            .expect("same-outer convex nonrectangular bridge should retain one merged hole");
    nonrect_bridge_intersection.validate().unwrap();
    nonrect_bridge_intersection
        .validate_intersection_against_sources(&bridge_left, &nonrect_bridge)
        .unwrap();
    assert_eq!(nonrect_bridge_intersection.components.len(), 1);
    assert_eq!(nonrect_bridge_intersection.components[0].holes.len(), 1);
    assert_eq!(
        arrange_coplanar_surface_component_holed_intersection(&nonrect_bridge, &bridge_left),
        Some(nonrect_bridge_intersection)
    );
    let nonrect_bridge_preflight =
        preflight_boolean_exact(&bridge_left, &nonrect_bridge, ExactBooleanOperation::Intersection)
            .expect("same-outer nonrectangular bridge preflight should classify shortcut");
    nonrect_bridge_preflight.validate().unwrap();
    nonrect_bridge_preflight
        .validate_against_sources(&bridge_left, &nonrect_bridge)
        .unwrap();
    hypermesh::exact::boolean_exact(
        &bridge_left,
        &nonrect_bridge,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nonrectangular bridge boolean should materialize")
    .validate_operation_against_sources(
        &bridge_left,
        &nonrect_bridge,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let orthogonal_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
    let orthogonal_left_hole = rect_surface_i64(&[(4, 4, 12, 8), (4, 8, 8, 16)]);
    let orthogonal_right_hole = rect_surface_i64(&[(8, 6, 16, 10)]);
    let orthogonal_left = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_left_hole,
    )
    .expect("same-outer orthogonal retained-hole left should materialize")
    .mesh;
    let orthogonal_right = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_right_hole,
    )
    .expect("same-outer orthogonal retained-hole right should materialize")
    .mesh;
    let orthogonal_intersection = arrange_coplanar_surface_component_holed_intersection(
        &orthogonal_left,
        &orthogonal_right,
    )
    .expect("same-outer orthogonal retained-hole union should materialize");
    orthogonal_intersection.validate().unwrap();
    orthogonal_intersection
        .validate_intersection_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(orthogonal_intersection.components.len(), 1);
    assert_eq!(orthogonal_intersection.components[0].holes.len(), 1);
    assert!(orthogonal_intersection.components[0].holes[0].len() > 6);
    let orthogonal_reverse =
        arrange_coplanar_surface_component_holed_intersection(&orthogonal_right, &orthogonal_left)
            .expect("same-outer orthogonal retained-hole union should be symmetric");
    orthogonal_reverse.validate().unwrap();
    orthogonal_reverse
        .validate_intersection_against_sources(&orthogonal_right, &orthogonal_left)
        .unwrap();
    assert_eq!(orthogonal_reverse.components.len(), 1);
    assert_eq!(orthogonal_reverse.components[0].holes.len(), 1);
    assert_eq!(
        orthogonal_reverse.components[0].holes[0].len(),
        orthogonal_intersection.components[0].holes[0].len()
    );
    let orthogonal_preflight = preflight_boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("same-outer orthogonal retained-hole preflight should classify shortcut");
    orthogonal_preflight.validate().unwrap();
    orthogonal_preflight
        .validate_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(
        orthogonal_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    hypermesh::exact::boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer orthogonal retained-hole boolean should materialize")
    .validate_operation_against_sources(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let affine_origin = (0, 0, 0);
    let affine_basis_u = (2, 1, 0);
    let affine_basis_v = (-1, 2, 0);
    let affine_outer = affine_rect_surface_i64(
        &[(0, 0, 20, 20)],
        affine_origin,
        affine_basis_u,
        affine_basis_v,
    );
    let affine_left_hole = affine_rect_surface_i64(
        &[(4, 4, 12, 8), (4, 8, 8, 16)],
        affine_origin,
        affine_basis_u,
        affine_basis_v,
    );
    let affine_right_hole = affine_rect_surface_i64(
        &[(6, 6, 16, 10)],
        affine_origin,
        affine_basis_u,
        affine_basis_v,
    );
    let affine_left =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_left_hole)
            .expect("same-outer affine nonconvex retained-hole left should materialize")
            .mesh;
    let affine_right =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_right_hole)
            .expect("same-outer affine retained-hole right should materialize")
            .mesh;
    assert!(
        arrange_coplanar_orthogonal_surface_intersection(&affine_left, &affine_right).is_none()
    );
    let affine_intersection =
        arrange_coplanar_surface_component_holed_intersection(&affine_left, &affine_right)
            .expect("same-outer affine simple retained-hole union should materialize");
    affine_intersection.validate().unwrap();
    affine_intersection
        .validate_intersection_against_sources(&affine_left, &affine_right)
        .unwrap();
    assert_eq!(affine_intersection.components.len(), 1);
    assert_eq!(affine_intersection.components[0].holes.len(), 1);
    assert!(
        affine_intersection.components[0].holes[0].len() > 6,
        "affine simple retained-hole union should preserve exact split vertices"
    );
    let affine_reverse =
        arrange_coplanar_surface_component_holed_intersection(&affine_right, &affine_left)
            .expect("same-outer affine simple retained-hole union should be symmetric");
    affine_reverse.validate().unwrap();
    affine_reverse
        .validate_intersection_against_sources(&affine_right, &affine_left)
        .unwrap();
    assert_eq!(affine_reverse.components.len(), 1);
    assert_eq!(affine_reverse.components[0].holes.len(), 1);
    let affine_preflight =
        preflight_boolean_exact(&affine_left, &affine_right, ExactBooleanOperation::Intersection)
            .expect("same-outer affine retained-hole preflight should classify shortcut");
    affine_preflight.validate().unwrap();
    affine_preflight
        .validate_against_sources(&affine_left, &affine_right)
        .unwrap();
    assert_eq!(
        affine_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    hypermesh::exact::boolean_exact(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer affine retained-hole boolean should materialize")
    .validate_operation_against_sources(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let affine_touching_hole = affine_rect_surface_i64(
        &[(12, 6, 16, 10)],
        affine_origin,
        affine_basis_u,
        affine_basis_v,
    );
    let affine_touching =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_touching_hole)
            .expect("same-outer affine touching retained-hole source should materialize")
            .mesh;
    assert!(
        arrange_coplanar_surface_component_holed_intersection(&affine_left, &affine_touching)
            .is_none()
    );

    let disconnected_outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer disconnected retained-hole outer fixture must import");
    let disconnected_left_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 5, 2, 0, 5, 5, 0, 2, 5, 0, //
            12, 12, 0, 15, 12, 0, 15, 15, 0, 12, 15, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer disconnected left holes fixture must import");
    let disconnected_right_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 2, 0, 7, 2, 0, 7, 5, 0, 4, 5, 0, //
            14, 12, 0, 17, 12, 0, 17, 15, 0, 14, 15, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer disconnected right holes fixture must import");
    let disconnected_left = arrange_coplanar_convex_surface_multi_holed_difference(
        &disconnected_outer,
        &disconnected_left_holes,
    )
    .expect("same-outer disconnected left fixture should materialize")
    .mesh;
    let disconnected_right = arrange_coplanar_convex_surface_multi_holed_difference(
        &disconnected_outer,
        &disconnected_right_holes,
    )
    .expect("same-outer disconnected right fixture should materialize")
    .mesh;
    let disconnected_intersection = arrange_coplanar_surface_component_holed_intersection(
        &disconnected_left,
        &disconnected_right,
    )
    .expect("same-outer disconnected rectangle-strip clusters should materialize");
    disconnected_intersection.validate().unwrap();
    disconnected_intersection
        .validate_intersection_against_sources(&disconnected_left, &disconnected_right)
        .unwrap();
    assert_eq!(disconnected_intersection.components.len(), 1);
    assert_eq!(disconnected_intersection.components[0].holes.len(), 2);
    assert!(
        disconnected_intersection.components[0]
            .holes
            .iter()
            .all(|hole| hole.len() == 4),
        "disconnected positive-area rectangle-strip clusters should replay as two exact rectangles"
    );
    assert_eq!(
        arrange_coplanar_surface_component_holed_intersection(
            &disconnected_right,
            &disconnected_left,
        ),
        Some(disconnected_intersection)
    );
    let disconnected_preflight = preflight_boolean_exact(
        &disconnected_left,
        &disconnected_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("same-outer disconnected retained-hole preflight should classify shortcut");
    disconnected_preflight.validate().unwrap();
    disconnected_preflight
        .validate_against_sources(&disconnected_left, &disconnected_right)
        .unwrap();
    assert_eq!(
        disconnected_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    hypermesh::exact::boolean_exact(
        &disconnected_left,
        &disconnected_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer disconnected retained-hole boolean should materialize")
    .validate_operation_against_sources(
        &disconnected_left,
        &disconnected_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let small_hole = ExactMesh::from_i64_triangles_with_policy(
        &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nested small hole fixture must import");
    let large_hole = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nested large hole fixture must import");
    let small = arrange_coplanar_convex_surface_holed_difference(&outer, &small_hole)
        .expect("same-outer small-hole annulus should materialize")
        .mesh;
    let large = arrange_coplanar_convex_surface_holed_difference(&outer, &large_hole)
        .expect("same-outer large-hole annulus should materialize")
        .mesh;
    let nested = arrange_coplanar_surface_component_holed_intersection(&small, &large)
        .expect("same-outer nested-hole intersection should retain the larger hole");
    nested.validate().unwrap();
    nested
        .validate_intersection_against_sources(&small, &large)
        .unwrap();
    let nested_reverse = arrange_coplanar_surface_component_holed_intersection(&large, &small)
        .expect("same-outer nested-hole intersection should be symmetric");
    nested_reverse.validate().unwrap();
    nested_reverse
        .validate_intersection_against_sources(&large, &small)
        .unwrap();

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection)
        .expect("same-outer holed intersection preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed intersection should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_component_holed_coplanar_intersection_with_island() {
    let outer = rect_surface_i64(&[(0, 0, 20, 20)]);
    let left_holes = rect_surface_i64(&[(4, 4, 13, 8), (4, 8, 8, 17)]);
    let right_holes = rect_surface_i64(&[(12, 4, 16, 13), (7, 12, 16, 16)]);
    let left = arrange_coplanar_orthogonal_surface_difference(&outer, &left_holes)
        .expect("same-outer island left fixture should materialize")
        .mesh;
    let right = arrange_coplanar_orthogonal_surface_difference(&outer, &right_holes)
        .expect("same-outer island right fixture should materialize")
        .mesh;

    let intersection = arrange_coplanar_surface_component_holed_intersection(&left, &right)
        .expect("same-outer retained-hole frame should preserve its central island");
    intersection.validate().unwrap();
    intersection
        .validate_intersection_against_sources(&left, &right)
        .unwrap();
    assert_eq!(intersection.components.len(), 2);
    assert_eq!(
        intersection
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    assert!(
        intersection
            .components
            .iter()
            .any(|component| component.holes.is_empty()),
        "the complement of the removed orthogonal frame must be retained as an island"
    );
    let reverse = arrange_coplanar_surface_component_holed_intersection(&right, &left)
        .expect("same-outer retained-hole island intersection should be symmetric");
    reverse.validate().unwrap();
    reverse
        .validate_intersection_against_sources(&right, &left)
        .unwrap();

    let mut stale = intersection.clone();
    let island = stale
        .components
        .iter()
        .position(|component| component.holes.is_empty())
        .expect("fixture should expose an island component");
    stale.components.remove(island);
    assert!(
        stale
            .validate_intersection_against_sources(&left, &right)
            .is_err()
    );

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection)
        .expect("same-outer retained-hole island preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer retained-hole island boolean should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let point_touch_right_hole = rect_surface_i64(&[(8, 8, 12, 12)]);
    let point_touch_right =
        arrange_coplanar_convex_surface_holed_difference(&outer, &point_touch_right_hole)
            .expect("same-outer point-touch source should materialize")
            .mesh;
    assert!(
        arrange_coplanar_surface_component_holed_intersection(&left, &point_touch_right).is_none()
    );

    let branch_outer = rect_surface_i64(&[(0, 0, 5, 5)]);
    let branch_holes = rect_surface_i64(&[
        (1, 2, 2, 3),
        (1, 3, 2, 4),
        (2, 1, 3, 2),
        (2, 3, 3, 4),
        (3, 1, 4, 2),
        (3, 2, 4, 3),
        (3, 3, 4, 4),
    ]);
    let branch_source =
        arrange_coplanar_orthogonal_surface_difference(&branch_outer, &branch_holes)
            .expect("point-branched retained-hole source should materialize")
            .mesh;
    let branch_intersection =
        arrange_coplanar_surface_component_holed_intersection(&branch_source, &branch_outer)
            .expect("point-branched retained-hole island should survive equal-outer clipping");
    branch_intersection.validate().unwrap();
    branch_intersection
        .validate_intersection_against_sources(&branch_source, &branch_outer)
        .unwrap();
    assert_eq!(branch_intersection.components.len(), 2);
    assert!(
        branch_intersection
            .components
            .iter()
            .any(|component| component.holes.is_empty())
    );
    let branch_reverse =
        arrange_coplanar_surface_component_holed_intersection(&branch_outer, &branch_source)
            .expect("point-branched retained-hole island clipping should be symmetric");
    branch_reverse.validate().unwrap();
    branch_reverse
        .validate_intersection_against_sources(&branch_outer, &branch_source)
        .unwrap();
    let branch_preflight =
        preflight_boolean_exact(&branch_source, &branch_outer, ExactBooleanOperation::Intersection)
            .expect("point-branched retained-hole island preflight should classify shortcut");
    branch_preflight.validate().unwrap();
    branch_preflight
        .validate_against_sources(&branch_source, &branch_outer)
        .unwrap();

    let branch_island_killer_hole = rect_surface_i64(&[(2, 2, 3, 3)]);
    let branch_island_killer =
        arrange_coplanar_convex_surface_holed_difference(&branch_outer, &branch_island_killer_hole)
            .expect("point-branched island-consuming source should materialize")
            .mesh;
    let branch_consumed =
        arrange_coplanar_surface_component_holed_intersection(&branch_source, &branch_island_killer)
            .expect("opposite retained hole should consume the source-owned island");
    branch_consumed.validate().unwrap();
    branch_consumed
        .validate_intersection_against_sources(&branch_source, &branch_island_killer)
        .unwrap();
    assert_eq!(branch_consumed.components.len(), 1);
    assert!(
        branch_consumed
            .components
            .iter()
            .all(|component| !component.holes.is_empty())
    );
    let branch_consumed_reverse =
        arrange_coplanar_surface_component_holed_intersection(&branch_island_killer, &branch_source)
            .expect("source-owned island consumption should be symmetric");
    branch_consumed_reverse.validate().unwrap();
    branch_consumed_reverse
        .validate_intersection_against_sources(&branch_island_killer, &branch_source)
        .unwrap();
    let branch_consumed_preflight = preflight_boolean_exact(
        &branch_source,
        &branch_island_killer,
        ExactBooleanOperation::Intersection,
    )
    .expect("source-owned island consumption preflight should classify shortcut");
    branch_consumed_preflight.validate().unwrap();
    branch_consumed_preflight
        .validate_against_sources(&branch_source, &branch_island_killer)
        .unwrap();

    let scaled_branch_outer = rect_surface_i64(&[(0, 0, 10, 10)]);
    let scaled_branch_holes = rect_surface_i64(&[
        (2, 4, 4, 6),
        (2, 6, 4, 8),
        (4, 2, 6, 4),
        (4, 6, 6, 8),
        (6, 2, 8, 4),
        (6, 4, 8, 6),
        (6, 6, 8, 8),
    ]);
    let scaled_branch_source =
        arrange_coplanar_orthogonal_surface_difference(&scaled_branch_outer, &scaled_branch_holes)
            .expect("scaled point-branched retained-hole source should materialize")
            .mesh;
    let branch_partial_killer_hole = rect_surface_i64(&[(5, 4, 7, 6)]);
    let branch_partial_killer = arrange_coplanar_convex_surface_holed_difference(
        &scaled_branch_outer,
        &branch_partial_killer_hole,
    )
    .expect("partial point-branched island cutter should materialize")
    .mesh;
    let branch_clipped = arrange_coplanar_surface_component_holed_intersection(
        &scaled_branch_source,
        &branch_partial_killer,
    )
    .expect("opposite retained hole should clip the source-owned island");
    branch_clipped.validate().unwrap();
    branch_clipped
        .validate_intersection_against_sources(&scaled_branch_source, &branch_partial_killer)
        .unwrap();
    assert_eq!(branch_clipped.components.len(), 2);
    assert!(
        branch_clipped
            .components
            .iter()
            .any(|component| component.holes.is_empty())
    );
    let branch_clipped_reverse = arrange_coplanar_surface_component_holed_intersection(
        &branch_partial_killer,
        &scaled_branch_source,
    )
    .expect("source-owned island clipping should be symmetric");
    branch_clipped_reverse.validate().unwrap();
    branch_clipped_reverse
        .validate_intersection_against_sources(&branch_partial_killer, &scaled_branch_source)
        .unwrap();
    let branch_clipped_preflight = preflight_boolean_exact(
        &scaled_branch_source,
        &branch_partial_killer,
        ExactBooleanOperation::Intersection,
    )
    .expect("source-owned island clipping preflight should classify shortcut");
    branch_clipped_preflight.validate().unwrap();
    branch_clipped_preflight
        .validate_against_sources(&scaled_branch_source, &branch_partial_killer)
        .unwrap();

    let multi_clip_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
    let multi_clip_owner_hole = rect_surface_i64(&[(4, 4, 16, 16)]);
    let multi_clip_shell =
        arrange_coplanar_convex_surface_holed_difference(&multi_clip_outer, &multi_clip_owner_hole)
            .expect("multi-hole source-island owner shell should materialize")
            .mesh;
    let multi_clip_source_island = rect_surface_i64(&[(8, 8, 12, 12)]);
    let multi_clip_source = combine_open_exact_meshes(
        &[multi_clip_shell, multi_clip_source_island],
        "fuzz same-outer multi-hole clipped source island",
    );
    let multi_clip_right_holes = rect_surface_i64(&[(6, 6, 10, 10), (11, 11, 14, 14)]);
    let multi_clip_right =
        arrange_coplanar_convex_surface_multi_holed_difference(
            &multi_clip_outer,
            &multi_clip_right_holes,
        )
        .expect("multi-hole source-island cutter should materialize")
        .mesh;
    let multi_clip_intersection = arrange_coplanar_surface_component_holed_intersection(
        &multi_clip_source,
        &multi_clip_right,
    )
    .expect("opposite retained holes should clip the source-owned island");
    multi_clip_intersection.validate().unwrap();
    multi_clip_intersection
        .validate_intersection_against_sources(&multi_clip_source, &multi_clip_right)
        .unwrap();
    assert_eq!(multi_clip_intersection.components.len(), 2);
    assert!(
        multi_clip_intersection
            .components
            .iter()
            .any(|component| component.holes.is_empty())
    );
    let multi_clip_reverse = arrange_coplanar_surface_component_holed_intersection(
        &multi_clip_right,
        &multi_clip_source,
    )
    .expect("multi-hole source-owned island clipping should be symmetric");
    multi_clip_reverse.validate().unwrap();
    multi_clip_reverse
        .validate_intersection_against_sources(&multi_clip_right, &multi_clip_source)
        .unwrap();
    let multi_clip_preflight = preflight_boolean_exact(
        &multi_clip_source,
        &multi_clip_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("multi-hole source-owned island clipping preflight should classify shortcut");
    multi_clip_preflight.validate().unwrap();
    multi_clip_preflight
        .validate_against_sources(&multi_clip_source, &multi_clip_right)
        .unwrap();

    let split_source_island_hole = rect_surface_i64(&[(9, 8, 11, 12)]);
    let split_source_island_right = arrange_coplanar_convex_surface_holed_difference(
        &multi_clip_outer,
        &split_source_island_hole,
    )
    .expect("split source-island cutter should materialize")
    .mesh;
    let split_source_island_intersection = arrange_coplanar_surface_component_holed_intersection(
        &multi_clip_source,
        &split_source_island_right,
    )
    .expect("opposite retained hole should split the source-owned filled island");
    split_source_island_intersection.validate().unwrap();
    split_source_island_intersection
        .validate_intersection_against_sources(&multi_clip_source, &split_source_island_right)
        .unwrap();
    assert_eq!(split_source_island_intersection.components.len(), 3);
    assert_eq!(
        split_source_island_intersection
            .components
            .iter()
            .filter(|component| component.holes.is_empty())
            .count(),
        2
    );
    let split_source_island_reverse = arrange_coplanar_surface_component_holed_intersection(
        &split_source_island_right,
        &multi_clip_source,
    )
    .expect("split source-owned filled island clipping should be symmetric");
    split_source_island_reverse.validate().unwrap();
    split_source_island_reverse
        .validate_intersection_against_sources(&split_source_island_right, &multi_clip_source)
        .unwrap();
    let split_source_island_preflight = preflight_boolean_exact(
        &multi_clip_source,
        &split_source_island_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("split source-owned filled island preflight should classify shortcut");
    split_source_island_preflight.validate().unwrap();
    split_source_island_preflight
        .validate_against_sources(&multi_clip_source, &split_source_island_right)
        .unwrap();

    let holed_island_outer = rect_surface_i64(&[(0, 0, 24, 24)]);
    let holed_island_owner_hole = rect_surface_i64(&[(4, 4, 20, 20)]);
    let holed_island_shell = arrange_coplanar_convex_surface_holed_difference(
        &holed_island_outer,
        &holed_island_owner_hole,
    )
    .expect("holed source-island owner shell should materialize")
    .mesh;
    let holed_island_component_outer = rect_surface_i64(&[(8, 8, 18, 18)]);
    let holed_island_component_hole = rect_surface_i64(&[(10, 10, 14, 14)]);
    let holed_island_component = arrange_coplanar_convex_surface_holed_difference(
        &holed_island_component_outer,
        &holed_island_component_hole,
    )
    .expect("holed source island should materialize")
    .mesh;
    let holed_island_source = combine_open_exact_meshes(
        &[holed_island_shell, holed_island_component],
        "fuzz same-outer holed source island",
    );
    let holed_island_right_holes = rect_surface_i64(&[(11, 11, 12, 12), (15, 15, 17, 17)]);
    let holed_island_right = arrange_coplanar_convex_surface_multi_holed_difference(
        &holed_island_outer,
        &holed_island_right_holes,
    )
    .expect("holed source-island cutter should materialize")
    .mesh;
    let holed_island_intersection = arrange_coplanar_surface_component_holed_intersection(
        &holed_island_source,
        &holed_island_right,
    )
    .expect("same-outer holed source island should be retained");
    holed_island_intersection.validate().unwrap();
    holed_island_intersection
        .validate_intersection_against_sources(&holed_island_source, &holed_island_right)
        .unwrap();
    assert_eq!(holed_island_intersection.components.len(), 2);
    assert!(
        holed_island_intersection
            .components
            .iter()
            .any(|component| component.holes.len() == 2)
    );
    let holed_island_reverse = arrange_coplanar_surface_component_holed_intersection(
        &holed_island_right,
        &holed_island_source,
    )
    .expect("same-outer holed source island retention should be symmetric");
    holed_island_reverse.validate().unwrap();
    holed_island_reverse
        .validate_intersection_against_sources(&holed_island_right, &holed_island_source)
        .unwrap();
    let holed_island_preflight = preflight_boolean_exact(
        &holed_island_source,
        &holed_island_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("same-outer holed source island preflight should classify shortcut");
    holed_island_preflight.validate().unwrap();
    holed_island_preflight
        .validate_against_sources(&holed_island_source, &holed_island_right)
        .unwrap();

    let clipped_holed_island_outer = rect_surface_i64(&[(0, 0, 24, 24)]);
    let clipped_holed_island_owner_hole = rect_surface_i64(&[(4, 4, 20, 20)]);
    let clipped_holed_island_shell = arrange_coplanar_convex_surface_holed_difference(
        &clipped_holed_island_outer,
        &clipped_holed_island_owner_hole,
    )
    .expect("clipped holed source-island owner shell should materialize")
    .mesh;
    let clipped_holed_island_component_outer = rect_surface_i64(&[(8, 8, 18, 18)]);
    let clipped_holed_island_component_hole = rect_surface_i64(&[(10, 10, 12, 12)]);
    let clipped_holed_island_component = arrange_coplanar_convex_surface_holed_difference(
        &clipped_holed_island_component_outer,
        &clipped_holed_island_component_hole,
    )
    .expect("clipped holed source island should materialize")
    .mesh;
    let clipped_holed_island_source = combine_open_exact_meshes(
        &[
            clipped_holed_island_shell,
            clipped_holed_island_component,
        ],
        "fuzz same-outer clipped holed source island",
    );
    let clipped_holed_island_right_holes = rect_surface_i64(&[(16, 8, 22, 18), (13, 13, 15, 15)]);
    let clipped_holed_island_right = arrange_coplanar_convex_surface_multi_holed_difference(
        &clipped_holed_island_outer,
        &clipped_holed_island_right_holes,
    )
    .expect("clipped holed source-island cutter should materialize")
    .mesh;
    let clipped_holed_island_intersection = arrange_coplanar_surface_component_holed_intersection(
        &clipped_holed_island_source,
        &clipped_holed_island_right,
    )
    .expect("same-outer holed source island should clip to one holed remnant");
    clipped_holed_island_intersection.validate().unwrap();
    clipped_holed_island_intersection
        .validate_intersection_against_sources(
            &clipped_holed_island_source,
            &clipped_holed_island_right,
        )
        .unwrap();
    assert_eq!(clipped_holed_island_intersection.components.len(), 2);
    assert!(
        clipped_holed_island_intersection
            .components
            .iter()
            .any(|component| component.holes.len() == 2)
    );
    let clipped_holed_island_reverse = arrange_coplanar_surface_component_holed_intersection(
        &clipped_holed_island_right,
        &clipped_holed_island_source,
    )
    .expect("same-outer clipped holed source island should be symmetric");
    clipped_holed_island_reverse.validate().unwrap();
    clipped_holed_island_reverse
        .validate_intersection_against_sources(
            &clipped_holed_island_right,
            &clipped_holed_island_source,
        )
        .unwrap();
    let clipped_holed_island_preflight = preflight_boolean_exact(
        &clipped_holed_island_source,
        &clipped_holed_island_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("same-outer clipped holed source island preflight should classify shortcut");
    clipped_holed_island_preflight.validate().unwrap();
    clipped_holed_island_preflight
        .validate_against_sources(&clipped_holed_island_source, &clipped_holed_island_right)
        .unwrap();

    let split_holed_island_outer = rect_surface_i64(&[(0, 0, 24, 24)]);
    let split_holed_island_owner_hole = rect_surface_i64(&[(4, 4, 20, 20)]);
    let split_holed_island_shell = arrange_coplanar_convex_surface_holed_difference(
        &split_holed_island_outer,
        &split_holed_island_owner_hole,
    )
    .expect("split holed source-island owner shell should materialize")
    .mesh;
    let split_holed_island_component_outer = rect_surface_i64(&[(8, 8, 18, 18)]);
    let split_holed_island_component_hole = rect_surface_i64(&[(9, 10, 11, 12)]);
    let split_holed_island_component = arrange_coplanar_convex_surface_holed_difference(
        &split_holed_island_component_outer,
        &split_holed_island_component_hole,
    )
    .expect("split holed source island should materialize")
    .mesh;
    let split_holed_island_source = combine_open_exact_meshes(
        &[
            split_holed_island_shell,
            split_holed_island_component,
        ],
        "fuzz same-outer split holed source island",
    );
    let split_holed_island_right_holes = rect_surface_i64(&[(12, 8, 14, 18), (15, 13, 17, 15)]);
    let split_holed_island_right = arrange_coplanar_convex_surface_multi_holed_difference(
        &split_holed_island_outer,
        &split_holed_island_right_holes,
    )
    .expect("split holed source-island cutter should materialize")
    .mesh;
    let split_holed_island_intersection = arrange_coplanar_surface_component_holed_intersection(
        &split_holed_island_source,
        &split_holed_island_right,
    )
    .expect("same-outer holed source island should split into two remnants");
    split_holed_island_intersection.validate().unwrap();
    split_holed_island_intersection
        .validate_intersection_against_sources(
            &split_holed_island_source,
            &split_holed_island_right,
        )
        .unwrap();
    assert_eq!(split_holed_island_intersection.components.len(), 3);
    assert!(
        split_holed_island_intersection
            .components
            .iter()
            .filter(|component| component.holes.len() == 1)
            .count()
            >= 2
    );
    let split_holed_island_reverse = arrange_coplanar_surface_component_holed_intersection(
        &split_holed_island_right,
        &split_holed_island_source,
    )
    .expect("same-outer split holed source island should be symmetric");
    split_holed_island_reverse.validate().unwrap();
    split_holed_island_reverse
        .validate_intersection_against_sources(
            &split_holed_island_right,
            &split_holed_island_source,
        )
        .unwrap();
    let split_holed_island_preflight = preflight_boolean_exact(
        &split_holed_island_source,
        &split_holed_island_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("same-outer split holed source island preflight should classify shortcut");
    split_holed_island_preflight.validate().unwrap();
    split_holed_island_preflight
        .validate_against_sources(&split_holed_island_source, &split_holed_island_right)
        .unwrap();

    let affine_origin = (0, 0, 0);
    let affine_basis_u = (2, 1, 0);
    let affine_basis_v = (-1, 2, 0);
    let affine_outer = affine_rect_surface_i64(
        &[(0, 0, 20, 20)],
        affine_origin,
        affine_basis_u,
        affine_basis_v,
    );
    let affine_left_holes = affine_rect_surface_i64(
        &[(4, 4, 13, 8), (4, 8, 8, 17)],
        affine_origin,
        affine_basis_u,
        affine_basis_v,
    );
    let affine_right_holes = affine_rect_surface_i64(
        &[(12, 4, 16, 13), (7, 12, 16, 16)],
        affine_origin,
        affine_basis_u,
        affine_basis_v,
    );
    let affine_left =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_left_holes)
            .expect("same-outer affine island left fixture should materialize")
            .mesh;
    let affine_right =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_right_holes)
            .expect("same-outer affine island right fixture should materialize")
            .mesh;
    assert!(
        arrange_coplanar_orthogonal_surface_intersection(&affine_left, &affine_right).is_none()
    );
    let affine_intersection =
        arrange_coplanar_surface_component_holed_intersection(&affine_left, &affine_right)
            .expect("same-outer affine retained-hole frame should preserve its central island");
    affine_intersection.validate().unwrap();
    affine_intersection
        .validate_intersection_against_sources(&affine_left, &affine_right)
        .unwrap();
    assert_eq!(affine_intersection.components.len(), 2);
    assert_eq!(
        affine_intersection
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    assert!(
        affine_intersection
            .components
            .iter()
            .any(|component| component.holes.is_empty()),
        "the complement of the nonrectilinear frame must be retained as an island"
    );
    let affine_reverse =
        arrange_coplanar_surface_component_holed_intersection(&affine_right, &affine_left)
            .expect("same-outer affine retained-hole island intersection should be symmetric");
    affine_reverse.validate().unwrap();
    affine_reverse
        .validate_intersection_against_sources(&affine_right, &affine_left)
        .unwrap();

    let mut stale_affine = affine_intersection.clone();
    let affine_island = stale_affine
        .components
        .iter()
        .position(|component| component.holes.is_empty())
        .expect("affine fixture should expose an island component");
    stale_affine.components.remove(affine_island);
    assert!(
        stale_affine
            .validate_intersection_against_sources(&affine_left, &affine_right)
            .is_err()
    );

    let affine_preflight =
        preflight_boolean_exact(&affine_left, &affine_right, ExactBooleanOperation::Intersection)
            .expect("same-outer affine retained-hole island preflight should classify shortcut");
    affine_preflight.validate().unwrap();
    affine_preflight
        .validate_against_sources(&affine_left, &affine_right)
        .unwrap();
    assert_eq!(
        affine_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    hypermesh::exact::boolean_exact(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer affine retained-hole island boolean should materialize")
    .validate_operation_against_sources(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_source_island_point_touch_replay() {
    let outer = rect_surface_i64(&[(0, 0, 20, 20)]);
    let owner_hole = rect_surface_i64(&[(4, 4, 16, 16)]);
    let source_shell = arrange_coplanar_convex_surface_holed_difference(&outer, &owner_hole)
        .expect("source shell with retained owner hole should materialize")
        .mesh;
    let source_island = rect_surface_i64(&[(8, 8, 12, 12)]);
    let source = combine_open_exact_meshes(
        &[source_shell, source_island],
        "fuzz same-outer point-touch source island",
    );

    let point_touch_hole = rect_surface_i64(&[(12, 12, 14, 14)]);
    let point_touch_opposing =
        arrange_coplanar_convex_surface_holed_difference(&outer, &point_touch_hole)
            .expect("point-touch source-island cutter should materialize")
            .mesh;
    let point_touch_intersection = arrange_coplanar_surface_component_holed_intersection(
        &source,
        &point_touch_opposing,
    )
    .expect("point-only contact with a source-owned island should retain the island");
    point_touch_intersection.validate().unwrap();
    point_touch_intersection
        .validate_intersection_against_sources(&source, &point_touch_opposing)
        .unwrap();
    assert_eq!(point_touch_intersection.components.len(), 2);
    assert!(
        point_touch_intersection.components.iter().any(|component| {
            component.holes.is_empty()
                && component
                    .outer
                    .iter()
                    .any(|point| point.x == ExactReal::from(8))
                && component
                    .outer
                    .iter()
                    .any(|point| point.x == ExactReal::from(12))
        }),
        "the source-owned island should survive unchanged across point-only cutter contact"
    );

    let reverse = arrange_coplanar_surface_component_holed_intersection(
        &point_touch_opposing,
        &source,
    )
    .expect("point-only source-island contact should be symmetric");
    reverse.validate().unwrap();
    reverse
        .validate_intersection_against_sources(&point_touch_opposing, &source)
        .unwrap();

    let preflight =
        preflight_boolean_exact(&source, &point_touch_opposing, ExactBooleanOperation::Intersection)
            .expect("point-only source-island contact preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&source, &point_touch_opposing)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );

    hypermesh::exact::boolean_exact(
        &source,
        &point_touch_opposing,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only source-island contact boolean should materialize")
    .validate_operation_against_sources(
        &source,
        &point_touch_opposing,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_component_holed_coplanar_difference() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed difference outer fixture must import");
    let small_hole = ExactMesh::from_i64_triangles_with_policy(
        &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed difference small hole fixture must import");
    let large_hole = ExactMesh::from_i64_triangles_with_policy(
        &[3, 3, 0, 7, 3, 0, 7, 7, 0, 3, 7, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed difference large hole fixture must import");
    let left = arrange_coplanar_convex_surface_holed_difference(&outer, &small_hole)
        .expect("same-outer small-hole left should materialize")
        .mesh;
    let right = arrange_coplanar_convex_surface_holed_difference(&outer, &large_hole)
        .expect("same-outer large-hole right should materialize")
        .mesh;

    let difference = arrange_coplanar_surface_component_holed_difference(&left, &right)
        .expect("nested same-outer holes should materialize the difference annulus");
    difference.validate().unwrap();
    difference
        .validate_surface_difference_against_sources(&left, &right)
        .unwrap();
    assert_eq!(difference.components.len(), 1);
    assert_eq!(difference.components[0].holes.len(), 1);
    assert!(arrange_coplanar_surface_component_holed_difference(&right, &left).is_none());

    let crossing_hole = ExactMesh::from_i64_triangles_with_policy(
        &[5, 3, 0, 8, 3, 0, 8, 6, 0, 5, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer crossing hole fixture must import");
    let crossing = arrange_coplanar_convex_surface_holed_difference(&outer, &crossing_hole)
        .expect("same-outer crossing annulus should materialize")
        .mesh;
    assert!(arrange_coplanar_surface_component_holed_difference(&left, &crossing).is_none());

    let partial_outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 12, 0, 0, 12, 12, 0, 0, 12, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial same-outer outer fixture must import");
    let retained_and_cutting_left_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
            8, 1, 0, 11, 1, 0, 11, 5, 0, 8, 5, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial same-outer left holes fixture must import");
    let partial_right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 10, 2, 0, 10, 10, 0, 2, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial same-outer right hole fixture must import");
    let partial_left = arrange_coplanar_convex_surface_multi_holed_difference(
        &partial_outer,
        &retained_and_cutting_left_holes,
    )
    .expect("partial same-outer multi-holed left should materialize")
    .mesh;
    let partial_right =
        arrange_coplanar_convex_surface_holed_difference(&partial_outer, &partial_right_hole)
            .expect("partial same-outer right should materialize")
            .mesh;
    let partial_difference =
        arrange_coplanar_surface_component_holed_difference(&partial_left, &partial_right)
            .expect("partial rectangular overlap should retain a holed orthogonal remnant");
    partial_difference.validate().unwrap();
    partial_difference
        .validate_surface_difference_against_sources(&partial_left, &partial_right)
        .unwrap();
    assert_eq!(partial_difference.components.len(), 1);
    assert_eq!(partial_difference.components[0].holes.len(), 1);
    assert!(partial_difference.components[0].outer.len() > 4);

    let retained_and_nonrect_cutting_left_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
            8, 1, 0, 11, 1, 0, 11, 5, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial same-outer nonrect left holes fixture must import");
    let nonrect_partial_left = arrange_coplanar_convex_surface_multi_holed_difference(
        &partial_outer,
        &retained_and_nonrect_cutting_left_holes,
    )
    .expect("partial same-outer nonrect left should materialize")
    .mesh;
    let nonrect_partial_difference =
        arrange_coplanar_surface_component_holed_difference(&nonrect_partial_left, &partial_right)
            .expect("nonrectangular same-outer overlap should retain a holed remnant");
    nonrect_partial_difference.validate().unwrap();
    nonrect_partial_difference
        .validate_surface_difference_against_sources(&nonrect_partial_left, &partial_right)
        .unwrap();
    assert_eq!(nonrect_partial_difference.components.len(), 1);
    assert_eq!(nonrect_partial_difference.components[0].holes.len(), 1);

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .expect("same-outer holed difference preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed difference should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let partial_preflight =
        preflight_boolean_exact(&partial_left, &partial_right, ExactBooleanOperation::Difference)
            .expect("partial same-outer holed difference preflight should classify shortcut");
    partial_preflight.validate().unwrap();
    partial_preflight
        .validate_against_sources(&partial_left, &partial_right)
        .unwrap();
    assert_eq!(
        partial_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &partial_left,
        &partial_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial same-outer holed difference should materialize")
    .validate_operation_against_sources(
        &partial_left,
        &partial_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let nonrect_partial_preflight = preflight_boolean_exact(
        &nonrect_partial_left,
        &partial_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectangular same-outer holed difference preflight should classify shortcut");
    nonrect_partial_preflight.validate().unwrap();
    nonrect_partial_preflight
        .validate_against_sources(&nonrect_partial_left, &partial_right)
        .unwrap();
    assert_eq!(
        nonrect_partial_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &nonrect_partial_left,
        &partial_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectangular same-outer holed difference should materialize")
    .validate_operation_against_sources(
        &nonrect_partial_left,
        &partial_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let mixed_outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 14, 0, 0, 14, 14, 0, 0, 14, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed same-outer component-holed outer fixture must import");
    let mixed_left_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            6, 6, 0, 7, 6, 0, 7, 7, 0, 6, 7, 0, //
            1, 3, 0, 5, 3, 0, 5, 5, 0, 1, 5, 0, //
            10, 8, 0, 13, 8, 0, 10, 11, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7, 8, 9, 10],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed same-outer component-holed left holes fixture must import");
    let mixed_right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 12, 2, 0, 12, 12, 0, 2, 12, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed same-outer component-holed right hole fixture must import");
    let mixed_left =
        arrange_coplanar_convex_surface_multi_holed_difference(&mixed_outer, &mixed_left_holes)
            .expect("mixed same-outer component-holed left should materialize")
            .mesh;
    let mixed_right =
        arrange_coplanar_convex_surface_holed_difference(&mixed_outer, &mixed_right_hole)
            .expect("mixed same-outer component-holed right should materialize")
            .mesh;
    let mixed_difference =
        arrange_coplanar_surface_component_holed_difference(&mixed_left, &mixed_right)
            .expect("mixed same-outer component-holed difference should materialize");
    mixed_difference.validate().unwrap();
    mixed_difference
        .validate_surface_difference_against_sources(&mixed_left, &mixed_right)
        .unwrap();
    assert_eq!(mixed_difference.components.len(), 1);
    assert_eq!(mixed_difference.components[0].holes.len(), 1);
    let mixed_preflight =
        preflight_boolean_exact(&mixed_left, &mixed_right, ExactBooleanOperation::Difference)
            .expect("mixed component-holed preflight should classify shortcut");
    mixed_preflight.validate().unwrap();
    mixed_preflight
        .validate_against_sources(&mixed_left, &mixed_right)
        .unwrap();
    assert_eq!(
        mixed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &mixed_left,
        &mixed_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed component-holed difference should materialize")
    .validate_operation_against_sources(
        &mixed_left,
        &mixed_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let orthogonal_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
    let orthogonal_right_hole = rect_surface_i64(&[(4, 4, 16, 8), (4, 8, 8, 16)]);
    let orthogonal_left_holes = rect_surface_i64(&[(12, 5, 14, 7), (6, 10, 10, 14)]);
    let orthogonal_left = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_left_holes,
    )
    .expect("orthogonal same-outer left should materialize")
    .mesh;
    let orthogonal_right = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_right_hole,
    )
    .expect("orthogonal same-outer right should materialize")
    .mesh;
    let orthogonal_difference =
        arrange_coplanar_surface_component_holed_difference(&orthogonal_left, &orthogonal_right)
            .expect("orthogonal same-outer component-holed difference should materialize");
    orthogonal_difference.validate().unwrap();
    orthogonal_difference
        .validate_surface_difference_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(orthogonal_difference.components.len(), 1);
    assert_eq!(orthogonal_difference.components[0].holes.len(), 1);
    assert!(orthogonal_difference.components[0].outer.len() > 6);
    let orthogonal_preflight = preflight_boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Difference,
    )
    .expect("orthogonal component-holed preflight should classify shortcut");
    orthogonal_preflight.validate().unwrap();
    orthogonal_preflight
        .validate_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(
        orthogonal_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("orthogonal component-holed difference should materialize")
    .validate_operation_against_sources(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_holed_coplanar_multi_difference() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed multi-difference outer fixture must import");
    let left_hole = ExactMesh::from_i64_triangles_with_policy(
        &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed multi-difference left hole fixture must import");
    let right_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0, //
            7, 7, 0, 9, 7, 0, 9, 9, 0, 7, 9, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed multi-difference right holes fixture must import");
    let left = arrange_coplanar_convex_surface_holed_difference(&outer, &left_hole)
        .expect("same-outer one-hole left should materialize")
        .mesh;
    let right = arrange_coplanar_convex_surface_multi_holed_difference(&outer, &right_holes)
        .expect("same-outer multi-hole right should materialize")
        .mesh;

    assert!(arrange_coplanar_surface_component_holed_difference(&left, &right).is_none());
    let difference = arrange_coplanar_surface_multi_difference(&left, &right)
        .expect("same-outer disjoint right holes should materialize as filled components");
    difference.validate().unwrap();
    difference
        .validate_difference_against_sources(&left, &right)
        .unwrap();
    assert_eq!(difference.polygons.len(), 2);

    let crossing_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0, //
            5, 3, 0, 8, 3, 0, 8, 6, 0, 5, 6, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer crossing holes fixture must import");
    let crossing = arrange_coplanar_convex_surface_multi_holed_difference(&outer, &crossing_holes)
        .expect("same-outer crossing right should materialize")
        .mesh;
    let crossing_difference = arrange_coplanar_surface_multi_difference(&left, &crossing)
        .expect("same-outer rectangular hole overlap should replay as multi no-hole cells");
    crossing_difference.validate().unwrap();
    crossing_difference
        .validate_difference_against_sources(&left, &crossing)
        .unwrap();

    let orthogonal_outer = rect_surface_i64(&[(0, 0, 20, 20)]);
    let orthogonal_right_hole = rect_surface_i64(&[(4, 4, 16, 8), (4, 8, 8, 16)]);
    let orthogonal_left_hole = rect_surface_i64(&[(6, 4, 8, 16)]);
    let orthogonal_left = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_left_hole,
    )
    .expect("same-outer orthogonal no-hole left should materialize")
    .mesh;
    let orthogonal_right = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_right_hole,
    )
    .expect("same-outer orthogonal no-hole right should materialize")
    .mesh;
    assert!(
        arrange_coplanar_surface_component_difference(&orthogonal_left, &orthogonal_right).is_none()
    );
    let orthogonal_difference =
        arrange_coplanar_surface_multi_difference(&orthogonal_left, &orthogonal_right)
            .expect("same-outer orthogonal no-hole difference should split into components");
    orthogonal_difference.validate().unwrap();
    orthogonal_difference
        .validate_difference_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(orthogonal_difference.polygons.len(), 2);
    let orthogonal_preflight = preflight_boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Difference,
    )
    .expect("same-outer orthogonal no-hole multi-difference preflight should classify shortcut");
    orthogonal_preflight.validate().unwrap();
    orthogonal_preflight
        .validate_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(
        orthogonal_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    hypermesh::exact::boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer orthogonal no-hole multi-difference should materialize")
    .validate_operation_against_sources(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let touching_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 2, 1, 0, 2, 2, 0, 1, 2, 0, //
            6, 4, 0, 8, 4, 0, 8, 6, 0, 6, 6, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer touching multi-difference fixture must import");
    let touching = arrange_coplanar_convex_surface_multi_holed_difference(&outer, &touching_holes)
        .expect("same-outer touching right holes should materialize")
        .mesh;
    let touching_difference = arrange_coplanar_surface_multi_difference(&left, &touching)
        .expect("same-outer touching right holes should replay as filled components");
    touching_difference.validate().unwrap();
    touching_difference
        .validate_difference_against_sources(&left, &touching)
        .unwrap();
    assert_eq!(touching_difference.polygons.len(), 2);
    let touching_preflight =
        preflight_boolean_exact(&left, &touching, ExactBooleanOperation::Difference)
            .expect("same-outer touching multi-difference preflight should classify shortcut");
    touching_preflight.validate().unwrap();
    touching_preflight
        .validate_against_sources(&left, &touching)
        .unwrap();
    assert_eq!(
        touching_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    hypermesh::exact::boolean_exact(
        &left,
        &touching,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer touching multi-difference should materialize")
    .validate_operation_against_sources(
        &left,
        &touching,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .expect("same-outer holed multi-difference preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed multi-difference should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_holed_coplanar_component_difference() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed component-difference outer fixture must import");
    let left_hole = ExactMesh::from_i64_triangles_with_policy(
        &[4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed component-difference left hole fixture must import");
    let right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed component-difference right hole fixture must import");
    let left = arrange_coplanar_convex_surface_holed_difference(&outer, &left_hole)
        .expect("same-outer one-hole left should materialize")
        .mesh;
    let right = arrange_coplanar_convex_surface_holed_difference(&outer, &right_hole)
        .expect("same-outer one-hole right should materialize")
        .mesh;

    assert!(arrange_coplanar_surface_component_holed_difference(&left, &right).is_none());
    assert!(arrange_coplanar_surface_multi_difference(&left, &right).is_none());
    let difference = arrange_coplanar_surface_component_difference(&left, &right)
        .expect("same-outer single right hole should materialize as one filled component");
    difference.validate().unwrap();
    difference
        .validate_component_difference_against_sources(&left, &right)
        .unwrap();

    let crossing_hole = ExactMesh::from_i64_triangles_with_policy(
        &[5, 3, 0, 8, 3, 0, 8, 6, 0, 5, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer crossing component hole fixture must import");
    let crossing = arrange_coplanar_convex_surface_holed_difference(&outer, &crossing_hole)
        .expect("same-outer crossing component annulus should materialize")
        .mesh;
    let crossing_difference = arrange_coplanar_surface_component_difference(&left, &crossing)
        .expect("same-outer rectangular hole overlap should replay as one no-hole cell loop");
    crossing_difference.validate().unwrap();
    crossing_difference
        .validate_component_difference_against_sources(&left, &crossing)
        .unwrap();

    let nonrect_left_hole = ExactMesh::from_i64_triangles_with_policy(
        &[6, 1, 0, 9, 1, 0, 9, 5, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nonrect component left hole fixture must import");
    let nonrect_right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 8, 2, 0, 8, 8, 0, 2, 8, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nonrect component right hole fixture must import");
    let nonrect_left = arrange_coplanar_convex_surface_holed_difference(&outer, &nonrect_left_hole)
        .expect("same-outer nonrect left should materialize")
        .mesh;
    let nonrect_right =
        arrange_coplanar_convex_surface_holed_difference(&outer, &nonrect_right_hole)
            .expect("same-outer nonrect right should materialize")
            .mesh;
    let nonrect_difference =
        arrange_coplanar_surface_component_difference(&nonrect_left, &nonrect_right)
            .expect("same-outer nonrectangular overlap should replay as one no-hole loop");
    nonrect_difference.validate().unwrap();
    nonrect_difference
        .validate_component_difference_against_sources(&nonrect_left, &nonrect_right)
        .unwrap();
    let nonrect_preflight =
        preflight_boolean_exact(&nonrect_left, &nonrect_right, ExactBooleanOperation::Difference)
            .expect("same-outer nonrect component-difference preflight should classify shortcut");
    nonrect_preflight.validate().unwrap();
    nonrect_preflight
        .validate_against_sources(&nonrect_left, &nonrect_right)
        .unwrap();
    assert_eq!(
        nonrect_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    hypermesh::exact::boolean_exact(
        &nonrect_left,
        &nonrect_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nonrect component-difference should materialize")
    .validate_operation_against_sources(
        &nonrect_left,
        &nonrect_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let origin = (0, 0, 0);
    let basis_u = (2, 1, 0);
    let basis_v = (-1, 2, 0);
    let affine_outer = affine_rect_surface_i64(&[(0, 0, 14, 14)], origin, basis_u, basis_v);
    let affine_crossing_left_hole =
        affine_rect_surface_i64(&[(7, 4, 13, 12)], origin, basis_u, basis_v);
    let affine_nonconvex_right_hole =
        affine_rect_surface_i64(&[(3, 2, 12, 5), (8, 5, 12, 10)], origin, basis_u, basis_v);
    let affine_left =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_crossing_left_hole)
            .expect("affine same-outer left source should materialize")
            .mesh;
    let affine_right =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_nonconvex_right_hole)
            .expect("affine same-outer nonconvex right source should materialize")
            .mesh;
    assert!(arrange_coplanar_orthogonal_surface_difference(&affine_left, &affine_right).is_none());
    let affine_difference =
        arrange_coplanar_surface_component_difference(&affine_left, &affine_right)
            .expect("nonrectilinear nonconvex same-outer overlap should replay as one component");
    affine_difference.validate().unwrap();
    affine_difference
        .validate_component_difference_against_sources(&affine_left, &affine_right)
        .unwrap();
    let affine_preflight =
        preflight_boolean_exact(&affine_left, &affine_right, ExactBooleanOperation::Difference)
            .expect("nonrectilinear nonconvex same-outer preflight should classify shortcut");
    affine_preflight.validate().unwrap();
    affine_preflight
        .validate_against_sources(&affine_left, &affine_right)
        .unwrap();
    assert_eq!(
        affine_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    hypermesh::exact::boolean_exact(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectilinear nonconvex same-outer component difference should materialize")
    .validate_operation_against_sources(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let mixed_outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 14, 0, 0, 14, 14, 0, 0, 14, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed same-outer component-difference outer fixture must import");
    let mixed_left_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 3, 0, 5, 3, 0, 5, 5, 0, 1, 5, 0, //
            10, 8, 0, 13, 8, 0, 10, 11, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed same-outer component-difference left holes fixture must import");
    let mixed_right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 12, 2, 0, 12, 12, 0, 2, 12, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed same-outer component-difference right hole fixture must import");
    let mixed_left =
        arrange_coplanar_convex_surface_multi_holed_difference(&mixed_outer, &mixed_left_holes)
            .expect("mixed same-outer component-difference left should materialize")
            .mesh;
    let mixed_right =
        arrange_coplanar_convex_surface_holed_difference(&mixed_outer, &mixed_right_hole)
            .expect("mixed same-outer component-difference right should materialize")
            .mesh;
    let mixed_difference =
        arrange_coplanar_surface_component_difference(&mixed_left, &mixed_right)
            .expect("mixed same-outer component-difference should materialize");
    mixed_difference.validate().unwrap();
    mixed_difference
        .validate_component_difference_against_sources(&mixed_left, &mixed_right)
        .unwrap();
    let mixed_preflight =
        preflight_boolean_exact(&mixed_left, &mixed_right, ExactBooleanOperation::Difference)
            .expect("mixed same-outer component-difference preflight should classify shortcut");
    mixed_preflight.validate().unwrap();
    mixed_preflight
        .validate_against_sources(&mixed_left, &mixed_right)
        .unwrap();
    assert_eq!(
        mixed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    hypermesh::exact::boolean_exact(
        &mixed_left,
        &mixed_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed same-outer component-difference should materialize")
    .validate_operation_against_sources(
        &mixed_left,
        &mixed_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let touching_hole = ExactMesh::from_i64_triangles_with_policy(
        &[6, 4, 0, 8, 4, 0, 8, 6, 0, 6, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer touching hole fixture must import");
    let touching = arrange_coplanar_convex_surface_holed_difference(&outer, &touching_hole)
        .expect("same-outer touching annulus should materialize")
        .mesh;
    let touching_difference = arrange_coplanar_surface_component_difference(&left, &touching)
        .expect("same-outer touching right hole should replay as one filled loop");
    touching_difference.validate().unwrap();
    touching_difference
        .validate_component_difference_against_sources(&left, &touching)
        .unwrap();
    assert_eq!(touching_difference.polygon.len(), 4);
    let touching_preflight =
        preflight_boolean_exact(&left, &touching, ExactBooleanOperation::Difference)
            .expect("same-outer touching component-difference preflight should classify shortcut");
    touching_preflight.validate().unwrap();
    touching_preflight
        .validate_against_sources(&left, &touching)
        .unwrap();
    assert_eq!(
        touching_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    hypermesh::exact::boolean_exact(
        &left,
        &touching,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer touching component-difference should materialize")
    .validate_operation_against_sources(
        &left,
        &touching,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .expect("same-outer holed component-difference preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed component-difference should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_holed_coplanar_filled_union() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed filled-union outer fixture must import");
    let left_hole = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed filled-union left hole fixture must import");
    let right_hole = ExactMesh::from_i64_triangles_with_policy(
        &[6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer holed filled-union right hole fixture must import");
    let left = arrange_coplanar_convex_surface_holed_difference(&outer, &left_hole)
        .expect("same-outer filled-union left should materialize")
        .mesh;
    let right = arrange_coplanar_convex_surface_holed_difference(&outer, &right_hole)
        .expect("same-outer filled-union right should materialize")
        .mesh;

    assert!(arrange_coplanar_surface_component_holed_union(&left, &right).is_none());
    let union = arrange_coplanar_surface_component_union(&left, &right)
        .expect("same-outer disjoint retained holes should fill the outer sheet");
    union.validate().unwrap();
    union
        .validate_component_union_against_sources(&left, &right)
        .unwrap();

    let touching_hole = ExactMesh::from_i64_triangles_with_policy(
        &[4, 2, 0, 6, 2, 0, 6, 4, 0, 4, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer touching hole fixture must import");
    let touching = arrange_coplanar_convex_surface_holed_difference(&outer, &touching_hole)
        .expect("same-outer touching annulus should materialize")
        .mesh;
    let touching_union = arrange_coplanar_surface_component_union(&left, &touching)
        .expect("edge-touching same-outer holes should fill the outer sheet");
    touching_union.validate().unwrap();
    touching_union
        .validate_component_union_against_sources(&left, &touching)
        .unwrap();
    let touching_preflight = preflight_boolean_exact(&left, &touching, ExactBooleanOperation::Union)
        .expect("edge-touching same-outer union preflight should classify shortcut");
    touching_preflight.validate().unwrap();
    touching_preflight
        .validate_against_sources(&left, &touching)
        .unwrap();
    assert_eq!(
        touching_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &left,
        &touching,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("edge-touching same-outer union should materialize")
    .validate_operation_against_sources(
        &left,
        &touching,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("same-outer filled union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer filled union should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_same_outer_holed_coplanar_retained_union() {
    let outer = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer retained-union outer fixture must import");
    let left_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
            6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer retained-union left holes fixture must import");
    let right_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
            6, 1, 0, 8, 1, 0, 8, 3, 0, 6, 3, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer retained-union right holes fixture must import");
    let left = arrange_coplanar_convex_surface_multi_holed_difference(&outer, &left_holes)
        .expect("same-outer retained-union left should materialize")
        .mesh;
    let right = arrange_coplanar_convex_surface_multi_holed_difference(&outer, &right_holes)
        .expect("same-outer retained-union right should materialize")
        .mesh;

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    let union = arrange_coplanar_surface_component_holed_union(&left, &right)
        .expect("same-outer retained-hole union should materialize");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    let reverse = arrange_coplanar_surface_component_holed_union(&right, &left)
        .expect("same-outer retained-hole union should be symmetric");
    reverse.validate().unwrap();
    reverse.validate_union_against_sources(&right, &left).unwrap();
    assert_eq!(reverse.components.len(), union.components.len());

    let mut stale = union.clone();
    if let Some(hole) = stale.components.first_mut().and_then(|component| component.holes.first_mut())
    {
        hole.reverse();
        assert!(stale.validate().is_err());
    }

    let overlapping_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 2, 0, 5, 2, 0, 5, 4, 0, 3, 4, 0, //
            6, 1, 0, 8, 1, 0, 8, 3, 0, 6, 3, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer overlapping retained-union fixture must import");
    let overlapping =
        arrange_coplanar_convex_surface_multi_holed_difference(&outer, &overlapping_holes)
            .expect("same-outer overlapping source should materialize")
            .mesh;
    let overlapping_union = arrange_coplanar_surface_component_holed_union(&left, &overlapping)
        .expect("same-outer rectangular retained-hole overlap should materialize");
    overlapping_union.validate().unwrap();
    overlapping_union
        .validate_union_against_sources(&left, &overlapping)
        .unwrap();
    assert_eq!(overlapping_union.components.len(), 1);
    assert_eq!(overlapping_union.components[0].holes.len(), 1);
    let overlapping_reverse = arrange_coplanar_surface_component_holed_union(&overlapping, &left)
        .expect("same-outer rectangular retained-hole overlap should be symmetric");
    overlapping_reverse.validate().unwrap();
    overlapping_reverse
        .validate_union_against_sources(&overlapping, &left)
        .unwrap();

    let nonrectangular_overlap_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 2, 0, 5, 2, 0, 5, 4, 0, //
            6, 1, 0, 8, 1, 0, 8, 3, 0, 6, 3, 0,
        ],
        &[0, 1, 2, 3, 4, 5, 3, 5, 6],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nonrectangular retained-union fixture must import");
    let nonrectangular_overlap = arrange_coplanar_convex_surface_multi_holed_difference(
        &outer,
        &nonrectangular_overlap_holes,
    )
    .expect("same-outer nonrectangular source should materialize")
    .mesh;
    let nonrectangular_union =
        arrange_coplanar_surface_component_holed_union(&left, &nonrectangular_overlap)
            .expect("same-outer nonrectangular retained-hole overlap should materialize");
    nonrectangular_union.validate().unwrap();
    nonrectangular_union
        .validate_union_against_sources(&left, &nonrectangular_overlap)
        .unwrap();
    assert_eq!(nonrectangular_union.components.len(), 1);
    assert_eq!(nonrectangular_union.components[0].holes.len(), 1);
    let nonrectangular_reverse =
        arrange_coplanar_surface_component_holed_union(&nonrectangular_overlap, &left)
            .expect("same-outer nonrectangular retained-hole overlap should be symmetric");
    nonrectangular_reverse.validate().unwrap();
    nonrectangular_reverse
        .validate_union_against_sources(&nonrectangular_overlap, &left)
        .unwrap();

    let orthogonal_outer = rect_surface_i64(&[(0, 0, 10, 10)]);
    let orthogonal_left_hole = rect_surface_i64(&[(2, 2, 6, 6), (6, 2, 8, 4)]);
    let orthogonal_right_hole = rect_surface_i64(&[(4, 3, 9, 7)]);
    let orthogonal_left = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_left_hole,
    )
    .expect("same-outer orthogonal retained-union left should materialize")
    .mesh;
    let orthogonal_right = arrange_coplanar_orthogonal_surface_difference(
        &orthogonal_outer,
        &orthogonal_right_hole,
    )
    .expect("same-outer orthogonal retained-union right should materialize")
    .mesh;
    let orthogonal_union =
        arrange_coplanar_surface_component_holed_union(&orthogonal_left, &orthogonal_right)
            .expect("same-outer orthogonal retained-hole overlap should materialize");
    orthogonal_union.validate().unwrap();
    orthogonal_union
        .validate_union_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(orthogonal_union.components.len(), 1);
    assert_eq!(orthogonal_union.components[0].holes.len(), 1);
    assert_eq!(orthogonal_union.components[0].holes[0].len(), 6);
    let orthogonal_reverse =
        arrange_coplanar_surface_component_holed_union(&orthogonal_right, &orthogonal_left)
            .expect("same-outer orthogonal retained-hole overlap should be symmetric");
    orthogonal_reverse.validate().unwrap();
    orthogonal_reverse
        .validate_union_against_sources(&orthogonal_right, &orthogonal_left)
        .unwrap();

    let touching_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 2, 0, 6, 2, 0, 6, 4, 0, 4, 4, 0, //
            7, 1, 0, 9, 1, 0, 9, 3, 0, 7, 3, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer touching retained-union fixture must import");
    let touching = arrange_coplanar_convex_surface_multi_holed_difference(&outer, &touching_holes)
        .expect("same-outer touching source should materialize")
        .mesh;
    assert!(arrange_coplanar_surface_component_holed_union(&left, &touching).is_none());
    let touching_union = arrange_coplanar_surface_component_union(&left, &touching)
        .expect("same-outer touching retained holes should fill the outer sheet");
    touching_union.validate().unwrap();
    touching_union
        .validate_component_union_against_sources(&left, &touching)
        .unwrap();
    let touching_preflight =
        preflight_boolean_exact(&left, &touching, ExactBooleanOperation::Union)
            .expect("same-outer touching retained-union preflight should classify shortcut");
    touching_preflight.validate().unwrap();
    touching_preflight
        .validate_against_sources(&left, &touching)
        .unwrap();
    assert_eq!(
        touching_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &left,
        &touching,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer touching retained union should materialize")
    .validate_operation_against_sources(
        &left,
        &touching,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("same-outer retained union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer retained union should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let overlapping_preflight =
        preflight_boolean_exact(&left, &overlapping, ExactBooleanOperation::Union)
            .expect("same-outer rectangular retained-union preflight should classify shortcut");
    overlapping_preflight.validate().unwrap();
    overlapping_preflight
        .validate_against_sources(&left, &overlapping)
        .unwrap();
    assert_eq!(
        overlapping_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &left,
        &overlapping,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer rectangular retained union should materialize")
    .validate_operation_against_sources(
        &left,
        &overlapping,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let nonrectangular_preflight =
        preflight_boolean_exact(&left, &nonrectangular_overlap, ExactBooleanOperation::Union)
            .expect("same-outer nonrectangular retained-union preflight should classify shortcut");
    nonrectangular_preflight.validate().unwrap();
    nonrectangular_preflight
        .validate_against_sources(&left, &nonrectangular_overlap)
        .unwrap();
    assert_eq!(
        nonrectangular_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &left,
        &nonrectangular_overlap,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer nonrectangular retained union should materialize")
    .validate_operation_against_sources(
        &left,
        &nonrectangular_overlap,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let origin = (0, 0, 0);
    let basis_u = (2, 1, 0);
    let basis_v = (-1, 2, 0);
    let affine_outer = affine_rect_surface_i64(&[(0, 0, 14, 14)], origin, basis_u, basis_v);
    let affine_left_hole =
        affine_rect_surface_i64(&[(3, 2, 12, 5), (8, 5, 12, 10)], origin, basis_u, basis_v);
    let affine_right_hole = affine_rect_surface_i64(&[(7, 4, 13, 12)], origin, basis_u, basis_v);
    let affine_left =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_left_hole)
            .expect("same-outer affine nonconvex retained-union left should materialize")
            .mesh;
    let affine_right =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_right_hole)
            .expect("same-outer affine retained-union right should materialize")
            .mesh;
    assert!(arrange_coplanar_orthogonal_surface_union(&affine_left, &affine_right).is_none());
    let affine_union = arrange_coplanar_surface_component_holed_union(&affine_left, &affine_right)
        .expect("same-outer affine nonconvex retained-hole overlap should materialize");
    affine_union.validate().unwrap();
    affine_union
        .validate_union_against_sources(&affine_left, &affine_right)
        .unwrap();
    assert_eq!(affine_union.components.len(), 1);
    assert_eq!(affine_union.components[0].holes.len(), 1);
    let affine_reverse =
        arrange_coplanar_surface_component_holed_union(&affine_right, &affine_left)
            .expect("same-outer affine retained-hole overlap should be symmetric");
    affine_reverse.validate().unwrap();
    affine_reverse
        .validate_union_against_sources(&affine_right, &affine_left)
        .unwrap();
    let affine_preflight =
        preflight_boolean_exact(&affine_left, &affine_right, ExactBooleanOperation::Union)
            .expect("same-outer affine retained-union preflight should classify shortcut");
    affine_preflight.validate().unwrap();
    affine_preflight
        .validate_against_sources(&affine_left, &affine_right)
        .unwrap();
    assert_eq!(
        affine_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer affine retained union should materialize")
    .validate_operation_against_sources(
        &affine_left,
        &affine_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    let affine_touching_hole =
        affine_rect_surface_i64(&[(12, 3, 13, 7)], origin, basis_u, basis_v);
    let affine_touching =
        arrange_coplanar_affine_surface_difference(&affine_outer, &affine_touching_hole)
            .expect("same-outer affine touching retained-union source should materialize")
            .mesh;
    assert!(
        arrange_coplanar_surface_component_holed_union(&affine_left, &affine_touching).is_none()
    );

    let orthogonal_preflight = preflight_boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Union,
    )
    .expect("same-outer orthogonal retained-union preflight should classify shortcut");
    orthogonal_preflight.validate().unwrap();
    orthogonal_preflight
        .validate_against_sources(&orthogonal_left, &orthogonal_right)
        .unwrap();
    assert_eq!(
        orthogonal_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );
    hypermesh::exact::boolean_exact(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-outer orthogonal retained union should materialize")
    .validate_operation_against_sources(
        &orthogonal_left,
        &orthogonal_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_nonconvex_component_union_loop() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 2, 0, 4, -2, 0, 5, 2, 0, //
            2, 5, 0, -2, 4, 0, 2, 3, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex component union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex component union right fixture must import");

    assert!(arrange_coplanar_convex_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_convex_surface_multi_union(&left, &right).is_none());
    assert!(arrange_coplanar_orthogonal_surface_union(&left, &right).is_none());

    let union = arrange_coplanar_surface_component_union(&left, &right)
        .expect("nonconvex component contact graph should materialize one exact loop");
    union.validate().unwrap();
    union
        .validate_component_union_against_sources(&left, &right)
        .unwrap();
    let mut stale = union.clone();
    stale.polygon.reverse();
    assert!(stale.validate().is_err());

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("nonconvex component union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex component union boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_only_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 4, -2, 0, 6, -2, 0, //
            2, 6, 0, -2, 6, 0, -2, 4, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only component union fixture must import");
    assert!(arrange_coplanar_surface_component_union(&point_only_left, &right).is_none());
    let point_only_union = arrange_coplanar_surface_point_touch_union(&point_only_left, &right)
        .expect("exact vertex-vertex point-only component union should materialize separately");
    point_only_union.validate().unwrap();
    point_only_union
        .validate_union_against_sources(&point_only_left, &right)
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_nonconvex_multi_component_union_loop() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 2, 0, 4, -2, 0, 5, 2, 0, //
            2, 5, 0, -2, 4, 0, 2, 3, 0, //
            -7, -5, 0, -5, -5, 0, -6, -3, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, //
            6, 7, 8,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex multi-component union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex multi-component union right fixture must import");

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_convex_surface_multi_union(&left, &right).is_none());
    assert!(arrange_coplanar_orthogonal_surface_union(&left, &right).is_none());

    let union = arrange_coplanar_surface_multi_component_union(&left, &right)
        .expect("disconnected nonconvex component union should materialize retained loops");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    assert_eq!(union.polygons.len(), 2);
    assert!(union.polygons.iter().any(|polygon| polygon.len() > 6));
    let mut stale = union.clone();
    stale.polygons.swap(0, 1);
    assert!(stale.validate_union_against_sources(&left, &right).is_err());

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("nonconvex multi-component union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex multi-component union boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_only_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 4, -2, 0, 6, -2, 0, //
            2, 6, 0, -2, 6, 0, -2, 4, 0, //
            -7, -5, 0, -5, -5, 0, -6, -3, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, //
            6, 7, 8,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only disconnected component union fixture must import");
    assert!(arrange_coplanar_surface_multi_component_union(&point_only_left, &right).is_none());
    let point_only_union = arrange_coplanar_surface_point_touch_union(&point_only_left, &right)
        .expect("disconnected exact vertex-vertex point-touch union should materialize");
    point_only_union.validate().unwrap();
    point_only_union
        .validate_union_against_sources(&point_only_left, &right)
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_component_holed_coplanar_union() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 4, 0, 0, 2, 0, 2, 0, 0, 4, 0, 0, //
            0, -4, 0, 0, -2, 0, -2, 0, 0, -4, 0, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 0, 0, 2, 0, 0, 0, -2, 0, 0, -4, 0, //
            -4, 0, 0, -2, 0, 0, 0, 2, 0, 0, 4, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed union right fixture must import");

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_multi_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_point_touch_union(&left, &right).is_none());
    assert!(arrange_coplanar_orthogonal_surface_union(&left, &right).is_none());
    assert!(arrange_coplanar_affine_surface_union(&left, &right).is_none());

    let union = arrange_coplanar_surface_component_holed_union(&left, &right)
        .expect("annular positive-length component graph should retain a strict hole");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    assert_eq!(union.components.len(), 1);
    assert_eq!(union.components[0].holes.len(), 1);
    let mut stale = union.clone();
    stale.components[0].holes[0].reverse();
    assert!(stale.validate().is_err());

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("component-holed union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed union boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let incomplete_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 2, 0, 0, 0, -2, 0, 0, -4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("incomplete annular union fixture must import");
    assert!(arrange_coplanar_surface_component_holed_union(&left, &incomplete_right).is_none());

    let point_only_disconnected_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 0, 0, 2, 0, 0, 0, -2, 0, 1, -3, 0, //
            -5, 0, 0, -3, 0, 0, -1, 2, 0, -1, 4, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only disconnected annular union fixture must import");
    assert!(
        arrange_coplanar_surface_component_holed_union(&left, &point_only_disconnected_right)
            .is_none()
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_disconnected_component_holed_coplanar_union() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 4, 0, 0, 2, 0, 2, 0, 0, 4, 0, 0, //
            0, -4, 0, 0, -2, 0, -2, 0, 0, -4, 0, 0, //
            12, 4, 0, 12, 2, 0, 14, 0, 0, 16, 0, 0, //
            12, -4, 0, 12, -2, 0, 10, 0, 0, 8, 0, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("disconnected component-holed union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 0, 0, 2, 0, 0, 0, -2, 0, 0, -4, 0, //
            -4, 0, 0, -2, 0, 0, 0, 2, 0, 0, 4, 0, //
            16, 0, 0, 14, 0, 0, 12, -2, 0, 12, -4, 0, //
            8, 0, 0, 10, 0, 0, 12, 2, 0, 12, 4, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("disconnected component-holed union right fixture must import");

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_multi_component_union(&left, &right).is_none());
    let union = arrange_coplanar_surface_component_holed_union(&left, &right)
        .expect("disconnected annular component groups should materialize");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    assert_eq!(union.components.len(), 2);
    assert!(union.components.iter().all(|component| component.holes.len() == 1));

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("disconnected component-holed union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("disconnected component-holed union boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_two_disk_component_holed_coplanar_union() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 4, 0, -4, 0, 0, 0, -4, 0, //
            0, -2, 0, -2, 0, 0, 0, 2, 0,
        ],
        &[
            0, 1, 4, 0, 4, 5, //
            1, 2, 3, 1, 3, 4,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("two-disk component-holed union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, -4, 0, 4, 0, 0, 0, 4, 0, //
            0, 2, 0, 2, 0, 0, 0, -2, 0,
        ],
        &[
            1, 2, 3, 1, 3, 4, //
            0, 1, 4, 0, 4, 5,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("two-disk component-holed union right fixture must import");

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_multi_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_point_touch_union(&left, &right).is_none());
    let union = arrange_coplanar_surface_component_holed_union(&left, &right)
        .expect("two nonconvex source disks should replay one annular union");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    assert_eq!(union.components.len(), 1);
    assert_eq!(union.components[0].holes.len(), 1);

    let mut filled_hole = union.clone();
    filled_hole.components[0].holes.clear();
    assert!(
        filled_hole
            .validate_union_against_sources(&left, &right)
            .is_err()
    );

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("two-disk component-holed union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );

    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("two-disk component-holed union boolean should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_overlapping_component_holed_coplanar_union() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            -1, 5, 0, 0, 2, 0, 2, 0, 0, 5, -1, 0, //
            1, -5, 0, 0, -2, 0, -2, 0, 0, -5, 1, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("overlapping component-holed union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            5, 1, 0, 2, 0, 0, 0, -2, 0, 1, -5, 0, //
            -5, -1, 0, -2, 0, 0, 0, 2, 0, -1, 5, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("overlapping component-holed union right fixture must import");

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_multi_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_point_touch_union(&left, &right).is_none());

    let union = arrange_coplanar_surface_component_holed_union(&left, &right)
        .expect("convex overlaps should replay one retained component-holed annulus");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    assert_eq!(union.components.len(), 1);
    assert_eq!(union.components[0].holes.len(), 1);

    let mut filled_hole = union.clone();
    filled_hole.components[0].holes.clear();
    assert!(
        filled_hole
            .validate_union_against_sources(&left, &right)
            .is_err()
    );

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("overlapping component-holed union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );

    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("overlapping component-holed union boolean should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_nonconvex_overlap_component_holed_coplanar_union() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            -1, 5, 0, 0, 2, 0, 2, 0, 0, 5, -1, 0, 2, 1, 0, //
            1, -5, 0, 0, -2, 0, -2, 0, 0, -5, 1, 0,
        ],
        &[
            4, 0, 1, 4, 1, 2, 4, 2, 3, //
            5, 6, 7, 5, 7, 8,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex-overlap component-holed union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            5, 1, 0, 2, 0, 0, 0, -2, 0, 1, -5, 0, //
            -5, -1, 0, -2, 0, 0, 0, 2, 0, -1, 5, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex-overlap component-holed union right fixture must import");

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_multi_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_point_touch_union(&left, &right).is_none());

    let union = arrange_coplanar_surface_component_holed_union(&left, &right)
        .expect("nonconvex positive-area overlap should retain a strict hole");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    assert_eq!(union.components.len(), 1);
    assert_eq!(union.components[0].holes.len(), 1);

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("nonconvex-overlap component-holed union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );

    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex-overlap component-holed union boolean should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_point_branch_component_holed_coplanar_union() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            -1, 5, 0, 0, 2, 0, 2, 0, 0, 5, -1, 0, //
            1, -5, 0, 0, -2, 0, -2, 0, 0, -5, 1, 0, //
            5, 1, 0, 7, 1, 0, 6, 3, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-branch component-holed union left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            5, 1, 0, 2, 0, 0, 0, -2, 0, 1, -5, 0, //
            -5, -1, 0, -2, 0, 0, 0, 2, 0, -1, 5, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-branch component-holed union right fixture must import");

    assert!(arrange_coplanar_surface_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_multi_component_union(&left, &right).is_none());
    assert!(arrange_coplanar_surface_point_touch_union(&left, &right).is_none());

    let union = arrange_coplanar_surface_component_holed_union(&left, &right)
        .expect("point-branched annular union should retain two output components");
    union.validate().unwrap();
    union.validate_union_against_sources(&left, &right).unwrap();
    assert_eq!(union.components.len(), 2);
    assert_eq!(
        union
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("point-branch component-holed union preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementUnion
    );

    hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-branch component-holed union boolean should materialize")
    .validate_operation_against_sources(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_contact_opening_with_retained_hole() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("contact opening retained-hole left fixture must import");
    let opening_plus_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
            15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, 7, 9, 10,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("contact opening retained-hole right fixture must import");

    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &opening_plus_hole)
            .is_none()
    );
    let holed =
        arrange_coplanar_convex_surface_component_holed_difference(&left, &opening_plus_hole)
            .expect("contact opening should retain unrelated strict holes");
    holed.validate().unwrap();
    holed
        .validate_against_sources(&left, &opening_plus_hole)
        .unwrap();
    assert_eq!(holed.components.len(), 1);
    assert_eq!(holed.components[0].holes.len(), 1);

    let mut stale = holed.clone();
    stale.components[0].holes.clear();
    assert!(stale.validate().is_err());

    let preflight =
        preflight_boolean_exact(&left, &opening_plus_hole, ExactBooleanOperation::Difference)
            .expect("contact opening retained-hole preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&left, &opening_plus_hole)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &opening_plus_hole,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("contact opening retained-hole boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &opening_plus_hole,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_only_opening_plus_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 10, 0, 10, 8, 0, 10, 12, 0, //
            0, 8, 0, 8, 10, 0, 0, 12, 0, //
            15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, //
            6, 7, 8, 6, 8, 9,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only retained-hole fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &left,
            &point_only_opening_plus_hole,
        )
        .is_none()
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_independent_contact_openings() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("independent opening left fixture must import");
    let openings = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
            12, 4, 0, 16, 6, 0, 12, 8, 0, //
            20, 5, 0, 14, 4, 0, 14, 8, 0, 20, 7, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, //
            10, 11, 12, 10, 12, 13,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("independent opening right fixture must import");

    let opened = arrange_coplanar_surface_cutter_hole_contact_difference(&left, &openings)
        .expect("independent openings should materialize");
    opened.validate().unwrap();
    opened
        .validate_cutter_hole_contact_difference_against_sources(&left, &openings)
        .unwrap();

    let preflight = preflight_boolean_exact(&left, &openings, ExactBooleanOperation::Difference)
        .expect("independent opening preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &openings).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );

    let with_retained_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
            12, 4, 0, 16, 6, 0, 12, 8, 0, //
            20, 5, 0, 14, 4, 0, 14, 8, 0, 20, 7, 0, //
            3, 15, 0, 5, 15, 0, 5, 17, 0, 3, 17, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, //
            10, 11, 12, 10, 12, 13, //
            14, 15, 16, 14, 16, 17,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("independent opening retained-hole right fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &with_retained_hole)
            .is_none()
    );
    let holed =
        arrange_coplanar_convex_surface_component_holed_difference(&left, &with_retained_hole)
            .expect("independent openings should retain unrelated strict holes");
    holed.validate().unwrap();
    holed
        .validate_against_sources(&left, &with_retained_hole)
        .unwrap();
    assert_eq!(holed.components.len(), 1);
    assert_eq!(holed.components[0].holes.len(), 1);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_connected_multi_cutter_opening_with_retained_hole() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("connected multi-cutter opening left fixture must import");
    let connected_opening = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
            -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("connected multi-cutter opening right fixture must import");

    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &connected_opening)
            .is_none()
    );
    let holed =
        arrange_coplanar_convex_surface_component_holed_difference(&left, &connected_opening)
            .expect("connected multi-cutter opening should retain the unrelated strict hole");
    holed.validate().unwrap();
    holed
        .validate_against_sources(&left, &connected_opening)
        .unwrap();
    assert_eq!(holed.components.len(), 1);
    assert_eq!(holed.components[0].holes.len(), 1);
    assert!(holed.components[0].outer.len() > 8);

    let mut stale = holed.clone();
    stale.components[0].outer.reverse();
    assert!(stale.validate().is_err());
    stale.components[0].outer.reverse();
    stale.components[0].holes.clear();
    assert!(stale.validate().is_err());

    let preflight =
        preflight_boolean_exact(&left, &connected_opening, ExactBooleanOperation::Difference)
            .expect("connected multi-cutter opening preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&left, &connected_opening)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &connected_opening,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("connected multi-cutter opening boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &connected_opening,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_only_graph = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 15, 0, 17, 15, 0, 17, 17, 0, 15, 17, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
            -2, 13, 0, 7, 10, 0, 10, 14, 0, -2, 18, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only multi-cutter graph fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(&left, &point_only_graph)
            .is_none()
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_multiple_side_cutter_openings_with_retained_hole() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multiple side-cutter opening left fixture must import");
    let openings = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
            -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
            11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multiple side-cutter opening right fixture must import");

    assert!(arrange_coplanar_surface_cutter_hole_contact_difference(&left, &openings).is_none());
    let holed = arrange_coplanar_convex_surface_component_holed_difference(&left, &openings)
        .expect("multiple side-cutter openings should retain the unrelated strict hole");
    holed.validate().unwrap();
    holed.validate_against_sources(&left, &openings).unwrap();
    assert_eq!(holed.components.len(), 1);
    assert_eq!(holed.components[0].holes.len(), 1);
    assert!(holed.components[0].outer.len() > 10);

    let mut stale = holed.clone();
    stale.components[0].holes[0].reverse();
    assert!(stale.validate().is_err());

    let preflight = preflight_boolean_exact(&left, &openings, ExactBooleanOperation::Difference)
        .expect("multiple side-cutter opening preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &openings).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &openings,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multiple side-cutter opening boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &openings,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let consumed_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 8, 0, 5, 8, 0, 5, 9, 0, 4, 9, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
            -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
            11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("consumed-hole multiple side-cutter fixture must import");
    assert!(arrange_coplanar_convex_surface_component_holed_difference(
        &left,
        &consumed_hole
    )
    .is_none());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_consumed_hole_side_cutter_openings() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("consumed-hole side-cutter opening left fixture must import");
    let single_retained_and_consumed = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
            4, 8, 0, 5, 8, 0, 5, 9, 0, 4, 9, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single retained-and-consumed side-cutter fixture must import");
    let single_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &left,
        &single_retained_and_consumed,
    )
    .expect("single side opening should retain and consume exact strict holes");
    single_holed.validate().unwrap();
    single_holed
        .validate_against_sources(&left, &single_retained_and_consumed)
        .unwrap();
    assert_eq!(single_holed.components[0].holes.len(), 1);
    let single_holed_preflight = preflight_boolean_exact(
        &left,
        &single_retained_and_consumed,
        ExactBooleanOperation::Difference,
    )
    .expect("single retained-and-consumed side opening preflight should classify shortcut");
    single_holed_preflight.validate().unwrap();
    single_holed_preflight
        .validate_against_sources(&left, &single_retained_and_consumed)
        .unwrap();
    assert_eq!(
        single_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let single_consumed_only = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 8, 0, 5, 8, 0, 5, 9, 0, 4, 9, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single consumed-only side-cutter fixture must import");
    assert!(arrange_coplanar_surface_component_difference(&left, &single_consumed_only).is_none());
    let single_consumed =
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &single_consumed_only)
            .expect("single-source consumed holes stay in cutter/hole-contact replay");
    single_consumed.validate().unwrap();
    single_consumed
        .validate_cutter_hole_contact_difference_against_sources(&left, &single_consumed_only)
        .unwrap();
    let single_consumed_preflight =
        preflight_boolean_exact(&left, &single_consumed_only, ExactBooleanOperation::Difference)
            .expect("single consumed-only side opening preflight should classify shortcut");
    single_consumed_preflight.validate().unwrap();
    single_consumed_preflight
        .validate_against_sources(&left, &single_consumed_only)
        .unwrap();
    assert_eq!(
        single_consumed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );

    let retained_and_consumed = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
            4, 8, 0, 5, 8, 0, 5, 9, 0, 4, 9, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
            -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
            11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("retained-and-consumed side-cutter fixture must import");

    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &retained_and_consumed)
            .is_none()
    );
    let holed =
        arrange_coplanar_convex_surface_component_holed_difference(&left, &retained_and_consumed)
            .expect("fully consumed strict hole should be omitted while retained hole remains");
    holed.validate().unwrap();
    holed
        .validate_against_sources(&left, &retained_and_consumed)
        .unwrap();
    assert_eq!(holed.components.len(), 1);
    assert_eq!(holed.components[0].holes.len(), 1);

    let mut stale = holed.clone();
    stale.components[0].holes.push(vec![
        point3(4, 8, 0),
        point3(5, 8, 0),
        point3(5, 9, 0),
        point3(4, 9, 0),
    ]);
    assert!(
        stale
            .validate_against_sources(&left, &retained_and_consumed)
            .is_err()
    );

    let preflight = preflight_boolean_exact(
        &left,
        &retained_and_consumed,
        ExactBooleanOperation::Difference,
    )
    .expect("consumed-hole side-cutter opening preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&left, &retained_and_consumed)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let straddling_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
            8, 12, 0, 10, 12, 0, 10, 14, 0, 8, 14, 0, //
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
            -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
            11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("straddling consumed-hole fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &straddling_hole)
            .is_none()
    );
    let straddling =
        arrange_coplanar_convex_surface_component_holed_difference(&left, &straddling_hole)
            .expect("straddling strict hole should be consumed by its side-opening group");
    straddling.validate().unwrap();
    straddling
        .validate_against_sources(&left, &straddling_hole)
        .unwrap();
    assert_eq!(straddling.components.len(), 1);
    assert_eq!(straddling.components[0].holes.len(), 1);
    assert!(
        straddling.components[0]
            .outer
            .iter()
            .any(|point| point.x == hypermesh::exact::ExactReal::from(8)
                && point.y == hypermesh::exact::ExactReal::from(14))
    );
    let mut stale_straddling = straddling.clone();
    stale_straddling.components[0].holes.push(vec![
        point3(8, 12, 0),
        point3(10, 12, 0),
        point3(10, 14, 0),
        point3(8, 14, 0),
    ]);
    assert!(
        stale_straddling
            .validate_against_sources(&left, &straddling_hole)
            .is_err()
    );
    let straddling_preflight =
        preflight_boolean_exact(&left, &straddling_hole, ExactBooleanOperation::Difference)
            .expect("straddling consumed-hole preflight should classify component-holed shortcut");
    straddling_preflight.validate().unwrap();
    straddling_preflight
        .validate_against_sources(&left, &straddling_hole)
        .unwrap();
    assert_eq!(
        straddling_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let split_straddling_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
            9, 10, 0, 11, 10, 0, 11, 14, 0, 9, 14, 0, //
            -2, 8, 0, 10, 8, 0, 10, 12, 0, -2, 12, 0, //
            10, 8, 0, 22, 8, 0, 22, 12, 0, 10, 13, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("split straddling-hole fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &split_straddling_hole)
            .is_none()
    );
    let split =
        arrange_coplanar_convex_surface_component_holed_difference(&left, &split_straddling_hole)
            .expect("side-to-side straddling-hole group should split the source");
    split.validate().unwrap();
    split
        .validate_against_sources(&left, &split_straddling_hole)
        .unwrap();
    assert_eq!(split.components.len(), 2);
    assert_eq!(
        split
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let mut stale_split = split.clone();
    stale_split.components[0].holes.push(vec![
        point3(9, 10, 0),
        point3(11, 10, 0),
        point3(11, 14, 0),
        point3(9, 14, 0),
    ]);
    assert!(
        stale_split
            .validate_against_sources(&left, &split_straddling_hole)
            .is_err()
    );
    let split_preflight =
        preflight_boolean_exact(&left, &split_straddling_hole, ExactBooleanOperation::Difference)
            .expect("split straddling-hole preflight should classify component-holed shortcut");
    split_preflight.validate().unwrap();
    assert_eq!(
        split_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let crossing_side_cutter_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("crossing side-cutter source fixture must import");
    let crossing_side_cutter_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            -4, 8, 0, 34, 18, 0, 34, 22, 0, -4, 12, 0, //
            8, -4, 0, 12, -4, 0, 22, 34, 0, 18, 34, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("crossing side-cutter right fixture must import");
    let crossing_split = arrange_coplanar_surface_multi_difference(
        &crossing_side_cutter_left,
        &crossing_side_cutter_right,
    )
    .expect("crossing side-cutter union should split into retained loops");
    crossing_split.validate().unwrap();
    crossing_split
        .validate_difference_against_sources(
            &crossing_side_cutter_left,
            &crossing_side_cutter_right,
        )
        .unwrap();
    assert_eq!(crossing_split.polygons.len(), 4);
    let crossing_preflight = preflight_boolean_exact(
        &crossing_side_cutter_left,
        &crossing_side_cutter_right,
        ExactBooleanOperation::Difference,
    )
    .expect("crossing side-cutter preflight should classify multi-difference shortcut");
    crossing_preflight.validate().unwrap();
    crossing_preflight
        .validate_against_sources(&crossing_side_cutter_left, &crossing_side_cutter_right)
        .unwrap();
    assert_eq!(
        crossing_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let crossing_side_cutter_holed_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 4, 2, 0, 4, 4, 0, 2, 4, 0, //
            26, 2, 0, 28, 2, 0, 28, 4, 0, 26, 4, 0, //
            26, 26, 0, 28, 26, 0, 28, 28, 0, 26, 28, 0, //
            2, 26, 0, 4, 26, 0, 4, 28, 0, 2, 28, 0, //
            -4, 8, 0, 34, 18, 0, 34, 22, 0, -4, 12, 0, //
            8, -4, 0, 12, -4, 0, 22, 34, 0, 18, 34, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19, //
            20, 21, 22, 20, 22, 23,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("crossing side-cutter holed right fixture must import");
    assert!(arrange_coplanar_surface_multi_difference(
        &crossing_side_cutter_left,
        &crossing_side_cutter_holed_right,
    )
    .is_none());
    let crossing_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &crossing_side_cutter_left,
        &crossing_side_cutter_holed_right,
    )
    .expect("crossing side-cutter split should retain strict corner holes");
    crossing_holed.validate().unwrap();
    crossing_holed
        .validate_against_sources(&crossing_side_cutter_left, &crossing_side_cutter_holed_right)
        .unwrap();
    assert_eq!(crossing_holed.components.len(), 4);
    assert_eq!(
        crossing_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        4
    );
    let crossing_holed_preflight = preflight_boolean_exact(
        &crossing_side_cutter_left,
        &crossing_side_cutter_holed_right,
        ExactBooleanOperation::Difference,
    )
    .expect("crossing side-cutter holed preflight should classify component-holed shortcut");
    crossing_holed_preflight.validate().unwrap();
    crossing_holed_preflight
        .validate_against_sources(&crossing_side_cutter_left, &crossing_side_cutter_holed_right)
        .unwrap();
    assert_eq!(
        crossing_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let crossing_straddling_hole_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            12, 12, 0, 18, 12, 0, 18, 18, 0, 12, 18, 0, //
            -4, 8, 0, 34, 18, 0, 34, 22, 0, -4, 12, 0, //
            8, -4, 0, 12, -4, 0, 22, 34, 0, 18, 34, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("crossing straddling-hole right fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &crossing_side_cutter_left,
            &crossing_straddling_hole_right,
        )
        .is_none()
    );
    let crossing_straddling_difference = arrange_coplanar_surface_multi_difference(
        &crossing_side_cutter_left,
        &crossing_straddling_hole_right,
    )
    .expect("crossing side cutters should consume a straddling strict hole");
    crossing_straddling_difference.validate().unwrap();
    crossing_straddling_difference
        .validate_difference_against_sources(
            &crossing_side_cutter_left,
            &crossing_straddling_hole_right,
        )
        .unwrap();
    assert_eq!(crossing_straddling_difference.polygons.len(), 4);
    let crossing_straddling_preflight = preflight_boolean_exact(
        &crossing_side_cutter_left,
        &crossing_straddling_hole_right,
        ExactBooleanOperation::Difference,
    )
    .expect("crossing straddling-hole preflight should classify multi-difference shortcut");
    crossing_straddling_preflight.validate().unwrap();
    assert_eq!(
        crossing_straddling_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let multi_branch_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-branch consumed-hole left fixture must import");
    let multi_branch_with_retained_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 2, 0, 6, 2, 0, 6, 4, 0, 4, 4, 0, //
            4, 14, 0, 6, 14, 0, 6, 16, 0, 4, 16, 0, //
            4, 26, 0, 6, 26, 0, 6, 28, 0, 4, 28, 0, //
            13, 9, 0, 17, 9, 0, 17, 13, 0, 13, 13, 0, //
            -2, 7, 0, 15, 7, 0, 15, 11, 0, -2, 11, 0, //
            15, 7, 0, 32, 7, 0, 32, 11, 0, 15, 11, 0, //
            13, 21, 0, 17, 21, 0, 17, 25, 0, 13, 25, 0, //
            -2, 19, 0, 15, 19, 0, 15, 23, 0, -2, 23, 0, //
            15, 19, 0, 32, 19, 0, 32, 23, 0, 15, 23, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19, //
            20, 21, 22, 20, 22, 23, //
            24, 25, 26, 24, 26, 27, //
            28, 29, 30, 28, 30, 31, //
            32, 33, 34, 32, 34, 35,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-branch consumed-hole retained-hole fixture must import");
    let multi_branch = arrange_coplanar_convex_surface_component_holed_difference(
        &multi_branch_left,
        &multi_branch_with_retained_holes,
    )
    .expect("multi-branch consumed groups should split and retain local holes");
    multi_branch.validate().unwrap();
    multi_branch
        .validate_against_sources(&multi_branch_left, &multi_branch_with_retained_holes)
        .unwrap();
    assert_eq!(multi_branch.components.len(), 3);
    assert_eq!(
        multi_branch
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        3
    );
    let mut stale_multi_branch = multi_branch.clone();
    stale_multi_branch.components[0].holes.push(vec![
        point3(13, 9, 0),
        point3(17, 9, 0),
        point3(17, 13, 0),
        point3(13, 13, 0),
    ]);
    assert!(
        stale_multi_branch
            .validate_against_sources(&multi_branch_left, &multi_branch_with_retained_holes)
            .is_err()
    );
    let multi_branch_preflight = preflight_boolean_exact(
        &multi_branch_left,
        &multi_branch_with_retained_holes,
        ExactBooleanOperation::Difference,
    )
    .expect("multi-branch consumed-hole preflight should classify component-holed shortcut");
    multi_branch_preflight.validate().unwrap();
    assert_eq!(
        multi_branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let branch_group_with_retained_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
            25, 3, 0, 27, 3, 0, 27, 5, 0, 25, 5, 0, //
            3, 25, 0, 5, 25, 0, 5, 27, 0, 3, 27, 0, //
            25, 25, 0, 27, 25, 0, 27, 27, 0, 25, 27, 0, //
            12, 12, 0, 18, 12, 0, 18, 18, 0, 12, 18, 0, //
            -2, 14, 0, 14, 14, 0, 14, 16, 0, -2, 16, 0, //
            16, 14, 0, 32, 14, 0, 32, 16, 0, 16, 16, 0, //
            14, -2, 0, 16, -2, 0, 16, 14, 0, 14, 14, 0, //
            14, 16, 0, 16, 16, 0, 16, 32, 0, 14, 32, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19, //
            20, 21, 22, 20, 22, 23, //
            24, 25, 26, 24, 26, 27, //
            28, 29, 30, 28, 30, 31, //
            32, 33, 34, 32, 34, 35,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("branch consumed-hole retained-hole fixture must import");
    let branch_group = arrange_coplanar_convex_surface_component_holed_difference(
        &multi_branch_left,
        &branch_group_with_retained_holes,
    )
    .expect("one four-sided consumed branch should split and retain corner holes");
    branch_group.validate().unwrap();
    branch_group
        .validate_against_sources(&multi_branch_left, &branch_group_with_retained_holes)
        .unwrap();
    assert_eq!(branch_group.components.len(), 4);
    assert_eq!(
        branch_group
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        4
    );
    let mut stale_branch_group = branch_group.clone();
    stale_branch_group.components[0].holes.push(vec![
        point3(12, 12, 0),
        point3(18, 12, 0),
        point3(18, 18, 0),
        point3(12, 18, 0),
    ]);
    assert!(
        stale_branch_group
            .validate_against_sources(&multi_branch_left, &branch_group_with_retained_holes)
            .is_err()
    );
    let branch_group_preflight = preflight_boolean_exact(
        &multi_branch_left,
        &branch_group_with_retained_holes,
        ExactBooleanOperation::Difference,
    )
    .expect("four-sided consumed branch preflight should classify component-holed shortcut");
    branch_group_preflight.validate().unwrap();
    assert_eq!(
        branch_group_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let multi_component_consumed = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0, //
            30, 0, 0, 40, 0, 0, 40, 10, 0, 30, 10, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component consumed-hole left fixture must import");
    let multi_component_consumed_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
            33, 3, 0, 35, 3, 0, 35, 5, 0, 33, 5, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, 7, 9, 10,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component consumed-hole right fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(
            &multi_component_consumed,
            &multi_component_consumed_right,
        )
        .is_none()
    );
    let multi_component_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &multi_component_consumed,
        &multi_component_consumed_right,
    )
    .expect("multi-component difference should retain a no-hole opening and a strict hole");
    multi_component_holed.validate().unwrap();
    multi_component_holed
        .validate_against_sources(&multi_component_consumed, &multi_component_consumed_right)
        .unwrap();
    assert_eq!(multi_component_holed.components.len(), 2);
    assert!(
        multi_component_holed
            .components
            .iter()
            .any(|component| component.holes.is_empty())
    );
    assert!(
        multi_component_holed
            .components
            .iter()
            .any(|component| component.holes.len() == 1)
    );
    let multi_component_preflight = preflight_boolean_exact(
        &multi_component_consumed,
        &multi_component_consumed_right,
        ExactBooleanOperation::Difference,
    )
    .expect("multi-component consumed-hole preflight should classify component-holed shortcut");
    multi_component_preflight.validate().unwrap();
    assert_eq!(
        multi_component_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    let multi_component_result = hypermesh::exact::boolean_exact(
        &multi_component_consumed,
        &multi_component_consumed_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component consumed-hole boolean should materialize");
    multi_component_result.validate().unwrap();

    let multi_no_hole_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0, //
            30, 0, 0, 50, 0, 0, 50, 20, 0, 30, 20, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component no-hole consumed-opening left fixture must import");
    let multi_no_hole_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0, //
            38, 8, 0, 42, 10, 0, 38, 12, 0, //
            30, 9, 0, 40, 8, 0, 40, 12, 0, 30, 11, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, //
            10, 11, 12, 10, 12, 13,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component no-hole consumed-opening right fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(
            &multi_no_hole_left,
            &multi_no_hole_right,
        )
        .is_none()
    );
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &multi_no_hole_left,
            &multi_no_hole_right,
        )
        .is_none()
    );
    let multi_no_hole = arrange_coplanar_surface_multi_difference(
        &multi_no_hole_left,
        &multi_no_hole_right,
    )
    .expect("independent consumed-hole openings should emit a no-hole multi-difference");
    multi_no_hole.validate().unwrap();
    multi_no_hole
        .validate_difference_against_sources(&multi_no_hole_left, &multi_no_hole_right)
        .unwrap();
    assert_eq!(multi_no_hole.polygons.len(), 2);
    let multi_no_hole_preflight = preflight_boolean_exact(
        &multi_no_hole_left,
        &multi_no_hole_right,
        ExactBooleanOperation::Difference,
    )
    .expect("no-hole consumed-opening preflight should classify multi-difference shortcut");
    multi_no_hole_preflight.validate().unwrap();
    assert_eq!(
        multi_no_hole_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let split_all_consumed = ExactMesh::from_i64_triangles_with_policy(
        &[
            9, 10, 0, 11, 10, 0, 11, 14, 0, 9, 14, 0, //
            -2, 8, 0, 10, 8, 0, 10, 12, 0, -2, 12, 0, //
            10, 8, 0, 22, 8, 0, 22, 12, 0, 10, 13, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("split all-consumed no-hole fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(&left, &split_all_consumed)
            .is_none()
    );
    let split_no_hole = arrange_coplanar_surface_multi_difference(&left, &split_all_consumed)
        .expect("fully consumed side-to-side cutter/hole group should emit no-hole split loops");
    split_no_hole.validate().unwrap();
    split_no_hole
        .validate_difference_against_sources(&left, &split_all_consumed)
        .unwrap();
    assert_eq!(split_no_hole.polygons.len(), 2);
    let split_no_hole_preflight =
        preflight_boolean_exact(&left, &split_all_consumed, ExactBooleanOperation::Difference)
            .expect("all-consumed split preflight should classify multi-difference shortcut");
    split_no_hole_preflight.validate().unwrap();
    assert_eq!(
        split_no_hole_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let single_split_consumed = ExactMesh::from_i64_triangles_with_policy(
        &[
            9, 10, 0, 11, 10, 0, 11, 11, 0, 9, 11, 0, //
            -2, 8, 0, 22, 8, 0, 22, 15, 0, -2, 12, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single side-to-side consumed-hole fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&left, &single_split_consumed)
            .is_none()
    );
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &left,
            &single_split_consumed,
        )
        .is_none()
    );
    let single_split_no_hole =
        arrange_coplanar_surface_multi_difference(&left, &single_split_consumed)
            .expect("one consumed side-to-side cutter should emit split no-hole loops");
    single_split_no_hole.validate().unwrap();
    single_split_no_hole
        .validate_difference_against_sources(&left, &single_split_consumed)
        .unwrap();
    assert_eq!(single_split_no_hole.polygons.len(), 2);
    let single_split_preflight =
        preflight_boolean_exact(&left, &single_split_consumed, ExactBooleanOperation::Difference)
            .expect("single side-to-side consumed-hole split should classify multi-difference");
    single_split_preflight.validate().unwrap();
    assert_eq!(
        single_split_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let single_split_retained = ExactMesh::from_i64_triangles_with_policy(
        &[
            15, 16, 0, 17, 16, 0, 17, 18, 0, 15, 18, 0, //
            9, 10, 0, 11, 10, 0, 11, 11, 0, 9, 11, 0, //
            -2, 8, 0, 22, 8, 0, 22, 15, 0, -2, 12, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single side-to-side retained-and-consumed-hole fixture must import");
    let single_split_holed =
        arrange_coplanar_convex_surface_component_holed_difference(&left, &single_split_retained)
            .expect("one consumed side-to-side cutter should split while retaining holes");
    single_split_holed.validate().unwrap();
    single_split_holed
        .validate_against_sources(&left, &single_split_retained)
        .unwrap();
    assert_eq!(single_split_holed.components.len(), 2);
    assert_eq!(
        single_split_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let single_split_holed_preflight =
        preflight_boolean_exact(&left, &single_split_retained, ExactBooleanOperation::Difference)
            .expect("single side-to-side retained-hole split should classify component-holed");
    single_split_holed_preflight.validate().unwrap();
    assert_eq!(
        single_split_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let multi_branch_all_consumed = ExactMesh::from_i64_triangles_with_policy(
        &[
            13, 9, 0, 17, 9, 0, 17, 13, 0, 13, 13, 0, //
            -2, 7, 0, 15, 7, 0, 15, 11, 0, -2, 11, 0, //
            15, 7, 0, 32, 7, 0, 32, 11, 0, 15, 11, 0, //
            13, 21, 0, 17, 21, 0, 17, 25, 0, 13, 25, 0, //
            -2, 19, 0, 15, 19, 0, 15, 23, 0, -2, 23, 0, //
            15, 19, 0, 32, 19, 0, 32, 23, 0, 15, 23, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19, //
            20, 21, 22, 20, 22, 23,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-branch all-consumed no-hole fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &multi_branch_left,
            &multi_branch_all_consumed,
        )
        .is_none()
    );
    let multi_branch_no_hole =
        arrange_coplanar_surface_multi_difference(&multi_branch_left, &multi_branch_all_consumed)
            .expect("two consumed side-to-side groups should emit three no-hole split loops");
    multi_branch_no_hole.validate().unwrap();
    multi_branch_no_hole
        .validate_difference_against_sources(&multi_branch_left, &multi_branch_all_consumed)
        .unwrap();
    assert_eq!(multi_branch_no_hole.polygons.len(), 3);
    let multi_branch_no_hole_preflight = preflight_boolean_exact(
        &multi_branch_left,
        &multi_branch_all_consumed,
        ExactBooleanOperation::Difference,
    )
    .expect("multi-branch no-hole consumed split should classify multi-difference shortcut");
    multi_branch_no_hole_preflight.validate().unwrap();
    assert_eq!(
        multi_branch_no_hole_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let branch_group_all_consumed = ExactMesh::from_i64_triangles_with_policy(
        &[
            12, 12, 0, 18, 12, 0, 18, 18, 0, 12, 18, 0, //
            -2, 14, 0, 14, 14, 0, 14, 16, 0, -2, 16, 0, //
            16, 14, 0, 32, 14, 0, 32, 16, 0, 16, 16, 0, //
            14, -2, 0, 16, -2, 0, 16, 14, 0, 14, 14, 0, //
            14, 16, 0, 16, 16, 0, 16, 32, 0, 14, 32, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("branch all-consumed no-hole fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &multi_branch_left,
            &branch_group_all_consumed,
        )
        .is_none()
    );
    let branch_group_no_hole =
        arrange_coplanar_surface_multi_difference(&multi_branch_left, &branch_group_all_consumed)
            .expect("one four-sided consumed branch should emit four retained loops");
    branch_group_no_hole.validate().unwrap();
    branch_group_no_hole
        .validate_difference_against_sources(&multi_branch_left, &branch_group_all_consumed)
        .unwrap();
    assert_eq!(branch_group_no_hole.polygons.len(), 4);
    let branch_group_no_hole_preflight = preflight_boolean_exact(
        &multi_branch_left,
        &branch_group_all_consumed,
        ExactBooleanOperation::Difference,
    )
    .expect("four-sided consumed branch no-hole split should classify multi-difference shortcut");
    branch_group_no_hole_preflight.validate().unwrap();
    assert_eq!(
        branch_group_no_hole_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_mixed_consumed_hole_and_side_openings_without_retained_holes() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed consumed-hole left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            6, 8, 0, 8, 9, 0, 8, 11, 0, 6, 12, 0, //
            0, 9, 0, 6, 8, 0, 6, 12, 0, 0, 11, 0, //
            11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, 7, 9, 10, //
            11, 12, 13, 11, 13, 14,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("mixed consumed-hole right fixture must import");

    assert!(arrange_coplanar_surface_side_cutter_difference(&left, &right).is_none());
    assert!(arrange_coplanar_convex_surface_component_holed_difference(&left, &right).is_none());
    let opened = arrange_coplanar_surface_cutter_hole_contact_difference(&left, &right)
        .expect("mixed consumed-hole and side openings should replay as one loop");
    opened.validate().unwrap();
    opened
        .validate_cutter_hole_contact_difference_against_sources(&left, &right)
        .unwrap();
    assert!(opened.polygon.iter().any(|point| {
        point.x == hypermesh::exact::ExactReal::from(6)
            && point.y == hypermesh::exact::ExactReal::from(8)
    }));
    let mut stale = opened.clone();
    stale.polygon.reverse();
    assert!(stale.validate().is_err());
    assert!(
        stale
            .validate_cutter_hole_contact_difference_against_sources(&left, &right)
            .is_err()
    );

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .expect("mixed consumed-hole preflight should classify cutter-hole shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_side_cutter_opening_without_holes() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("side-cutter opening left fixture must import");
    let cutters = ExactMesh::from_i64_triangles_with_policy(
        &[
            -2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0, //
            -2, 8, 0, 8, 7, 0, 10, 13, 0, -2, 13, 0, //
            11, 3, 0, 22, 3, 0, 22, 11, 0, 13, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("side-cutter opening right fixture must import");
    let single_cutter = ExactMesh::from_i64_triangles_with_policy(
        &[-2, 4, 0, 9, 4, 0, 7, 10, 0, -2, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single side-cutter opening right fixture must import");

    assert!(arrange_coplanar_surface_multi_difference(&left, &cutters).is_none());
    assert!(arrange_coplanar_surface_cutter_hole_contact_difference(&left, &cutters).is_none());
    let single_opening = arrange_coplanar_surface_side_cutter_difference(&left, &single_cutter)
        .expect("one side cutter should materialize one nonconvex no-hole loop");
    single_opening.validate().unwrap();
    single_opening
        .validate_side_cutter_difference_against_sources(&left, &single_cutter)
        .unwrap();
    let single_preflight = preflight_boolean_exact(
        &left,
        &single_cutter,
        ExactBooleanOperation::Difference,
    )
    .expect("single side-cutter opening preflight should classify shortcut");
    single_preflight.validate().unwrap();
    single_preflight
        .validate_against_sources(&left, &single_cutter)
        .unwrap();
    assert_eq!(
        single_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceArrangementDifference
    );
    let single_result = hypermesh::exact::boolean_exact(
        &left,
        &single_cutter,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single side-cutter opening boolean should materialize");
    single_result
        .validate_operation_against_sources(
            &left,
            &single_cutter,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let opening = arrange_coplanar_surface_side_cutter_difference(&left, &cutters)
        .expect("side-cutter opening should materialize one nonconvex no-hole loop");
    opening.validate().unwrap();
    opening
        .validate_side_cutter_difference_against_sources(&left, &cutters)
        .unwrap();
    assert!(opening.polygon.len() > 10);

    let mut stale = opening.clone();
    stale.polygon.reverse();
    assert!(stale.validate().is_err());

    let preflight = preflight_boolean_exact(&left, &cutters, ExactBooleanOperation::Difference)
        .expect("side-cutter opening preflight should classify shortcut");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &cutters).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceSideCutterDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &cutters,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("side-cutter opening boolean should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &cutters,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let multi_component_side_opening_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0, //
            30, 0, 0, 40, 0, 0, 40, 10, 0, 30, 10, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component side-cutter opening source fixture must import");
    assert!(
        arrange_coplanar_surface_side_cutter_difference(
            &multi_component_side_opening_left,
            &cutters,
        )
        .is_none()
    );
    let multi_component_side_opening = arrange_coplanar_surface_multi_difference(
        &multi_component_side_opening_left,
        &cutters,
    )
    .expect("source-local side-cutter opening should emit multi-difference");
    multi_component_side_opening.validate().unwrap();
    multi_component_side_opening
        .validate_difference_against_sources(&multi_component_side_opening_left, &cutters)
        .unwrap();
    assert_eq!(multi_component_side_opening.polygons.len(), 2);
    let multi_component_side_opening_preflight = preflight_boolean_exact(
        &multi_component_side_opening_left,
        &cutters,
        ExactBooleanOperation::Difference,
    )
    .expect("source-local side-cutter opening preflight should classify shortcut");
    multi_component_side_opening_preflight.validate().unwrap();
    multi_component_side_opening_preflight
        .validate_against_sources(&multi_component_side_opening_left, &cutters)
        .unwrap();
    assert_eq!(
        multi_component_side_opening_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    let multi_component_side_opening_result = hypermesh::exact::boolean_exact(
        &multi_component_side_opening_left,
        &cutters,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("source-local side-cutter opening boolean should materialize");
    multi_component_side_opening_result
        .validate_operation_against_sources(
            &multi_component_side_opening_left,
            &cutters,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    assert!(
        arrange_coplanar_surface_side_cutter_difference(
            &multi_component_side_opening_left,
            &single_cutter,
        )
        .is_none()
    );
    let multi_component_single_opening = arrange_coplanar_surface_multi_difference(
        &multi_component_side_opening_left,
        &single_cutter,
    )
    .expect("source-local single side-cutter opening should emit multi-difference");
    multi_component_single_opening.validate().unwrap();
    multi_component_single_opening
        .validate_difference_against_sources(&multi_component_side_opening_left, &single_cutter)
        .unwrap();
    assert_eq!(multi_component_single_opening.polygons.len(), 2);
    let multi_component_single_preflight = preflight_boolean_exact(
        &multi_component_side_opening_left,
        &single_cutter,
        ExactBooleanOperation::Difference,
    )
    .expect("source-local single side-cutter preflight should classify shortcut");
    multi_component_single_preflight.validate().unwrap();
    multi_component_single_preflight
        .validate_against_sources(&multi_component_side_opening_left, &single_cutter)
        .unwrap();
    assert_eq!(
        multi_component_single_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    let multi_component_single_result = hypermesh::exact::boolean_exact(
        &multi_component_side_opening_left,
        &single_cutter,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("source-local single side-cutter boolean should materialize");
    multi_component_single_result
        .validate_operation_against_sources(
            &multi_component_side_opening_left,
            &single_cutter,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_only = ExactMesh::from_i64_triangles_with_policy(
        &[
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only side-cutter fixture must import");
    assert!(arrange_coplanar_surface_side_cutter_difference(&left, &point_only).is_none());
    let point_branch = arrange_coplanar_surface_point_touch_difference(&left, &point_only)
        .expect("point-touch side-cutter difference should materialize branched loops");
    point_branch.validate().unwrap();
    point_branch
        .validate_difference_against_sources(&left, &point_only)
        .unwrap();
    assert!(point_branch.polygons.len() >= 2);
    let point_branch_preflight =
        preflight_boolean_exact(&left, &point_only, ExactBooleanOperation::Difference)
            .expect("point-touch side-cutter preflight should classify shortcut");
    point_branch_preflight.validate().unwrap();
    point_branch_preflight
        .validate_against_sources(&left, &point_only)
        .unwrap();
    assert_eq!(
        point_branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    let point_branch_result = hypermesh::exact::boolean_exact(
        &left,
        &point_only,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-touch side-cutter boolean should materialize");
    point_branch_result
        .validate_operation_against_sources(
            &left,
            &point_only,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(point_branch_result.mesh, point_branch.mesh);

    let vertex_edge_branch_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("vertex-edge point-branch source fixture must import");
    let vertex_edge_branch_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
            10, -2, 0, 14, -2, 0, 12, 8, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("vertex-edge point-branch cutter fixture must import");
    assert!(arrange_coplanar_surface_multi_difference(
        &vertex_edge_branch_left,
        &vertex_edge_branch_right,
    )
    .is_none());
    let vertex_edge_branch = arrange_coplanar_surface_point_touch_difference(
        &vertex_edge_branch_left,
        &vertex_edge_branch_right,
    )
    .expect("vertex-edge point-branch side-cutter difference should materialize");
    vertex_edge_branch.validate().unwrap();
    vertex_edge_branch
        .validate_difference_against_sources(&vertex_edge_branch_left, &vertex_edge_branch_right)
        .unwrap();
    let vertex_edge_preflight = preflight_boolean_exact(
        &vertex_edge_branch_left,
        &vertex_edge_branch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("vertex-edge point-branch preflight should classify shortcut");
    vertex_edge_preflight.validate().unwrap();
    vertex_edge_preflight
        .validate_against_sources(&vertex_edge_branch_left, &vertex_edge_branch_right)
        .unwrap();
    assert_eq!(
        vertex_edge_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );

    let nonconvex_vertex_edge_branch_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 12, 20, 0, 12, 12, 0, 8, 12, 0, 8, 20,
            0, 0, 20, 0, 20, 12, 0, 0, 12, 0,
        ],
        &[
            0, 1, 8, 0, 8, 4, 0, 4, 5, 0, 5, 9, //
            9, 5, 6, 9, 6, 7, //
            4, 8, 2, 4, 2, 3,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex vertex-edge source fixture must import");
    let nonconvex_vertex_edge_branch_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            -2, 8, 0, 12, 8, 0, 12, 10, 0, -2, 10, 0, //
            10, -2, 0, 14, -2, 0, 12, 8, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex vertex-edge cutter fixture must import");
    assert!(arrange_coplanar_surface_multi_difference(
        &nonconvex_vertex_edge_branch_left,
        &nonconvex_vertex_edge_branch_right,
    )
    .is_none());
    let nonconvex_vertex_edge_branch = arrange_coplanar_surface_point_touch_difference(
        &nonconvex_vertex_edge_branch_left,
        &nonconvex_vertex_edge_branch_right,
    )
    .expect("nonconvex source vertex-edge branch should replay retained point-touch loops");
    nonconvex_vertex_edge_branch.validate().unwrap();
    nonconvex_vertex_edge_branch
        .validate_difference_against_sources(
            &nonconvex_vertex_edge_branch_left,
            &nonconvex_vertex_edge_branch_right,
        )
        .unwrap();
    let nonconvex_vertex_edge_preflight = preflight_boolean_exact(
        &nonconvex_vertex_edge_branch_left,
        &nonconvex_vertex_edge_branch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex vertex-edge point-branch preflight should classify shortcut");
    nonconvex_vertex_edge_preflight.validate().unwrap();
    nonconvex_vertex_edge_preflight
        .validate_against_sources(
            &nonconvex_vertex_edge_branch_left,
            &nonconvex_vertex_edge_branch_right,
        )
        .unwrap();
    assert_eq!(
        nonconvex_vertex_edge_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );

    let point_branch_consumed_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 5, 0, 3, 5, 0, 3, 6, 0, 2, 6, 0, //
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-branch consumed-hole side-cutter fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(&left, &point_branch_consumed_hole).is_none()
    );
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &left,
            &point_branch_consumed_hole,
        )
        .is_none()
    );
    let point_branch_consumed =
        arrange_coplanar_surface_point_touch_difference(&left, &point_branch_consumed_hole)
            .expect("point-touch side cutters should consume owned strict holes");
    point_branch_consumed.validate().unwrap();
    point_branch_consumed
        .validate_difference_against_sources(&left, &point_branch_consumed_hole)
        .unwrap();
    let point_branch_consumed_preflight = preflight_boolean_exact(
        &left,
        &point_branch_consumed_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("point-touch consumed-hole preflight should classify shortcut");
    point_branch_consumed_preflight.validate().unwrap();
    point_branch_consumed_preflight
        .validate_against_sources(&left, &point_branch_consumed_hole)
        .unwrap();
    assert_eq!(
        point_branch_consumed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );

    let point_branch_straddling_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-branch straddling-hole side-cutter fixture must import");
    assert!(arrange_coplanar_surface_multi_difference(&left, &point_branch_straddling_hole).is_none());
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &left,
            &point_branch_straddling_hole,
        )
        .is_none()
    );
    let point_branch_straddling =
        arrange_coplanar_surface_point_touch_difference(&left, &point_branch_straddling_hole)
            .expect("point-touch side cutters should consume an owned straddling hole");
    point_branch_straddling.validate().unwrap();
    point_branch_straddling
        .validate_difference_against_sources(&left, &point_branch_straddling_hole)
        .unwrap();
    let point_branch_straddling_preflight = preflight_boolean_exact(
        &left,
        &point_branch_straddling_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("point-touch straddling-hole preflight should classify shortcut");
    point_branch_straddling_preflight.validate().unwrap();
    point_branch_straddling_preflight
        .validate_against_sources(&left, &point_branch_straddling_hole)
        .unwrap();
    assert_eq!(
        point_branch_straddling_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );

    let grouped_straddling_branch_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("grouped point-branch straddling-hole source fixture must import");
    let grouped_straddling_branch_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
            -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
            12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
            12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("grouped point-branch straddling-hole cutter fixture must import");
    assert!(arrange_coplanar_surface_multi_difference(
        &grouped_straddling_branch_left,
        &grouped_straddling_branch_right,
    )
    .is_none());
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &grouped_straddling_branch_left,
            &grouped_straddling_branch_right,
        )
        .is_none()
    );
    let grouped_straddling_branch = arrange_coplanar_surface_point_touch_difference(
        &grouped_straddling_branch_left,
        &grouped_straddling_branch_right,
    )
    .expect("point-touch replay should consume a grouped straddling hole");
    grouped_straddling_branch.validate().unwrap();
    grouped_straddling_branch
        .validate_difference_against_sources(
            &grouped_straddling_branch_left,
            &grouped_straddling_branch_right,
        )
        .unwrap();
    let grouped_straddling_preflight = preflight_boolean_exact(
        &grouped_straddling_branch_left,
        &grouped_straddling_branch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("grouped straddling-hole preflight should classify point-touch shortcut");
    grouped_straddling_preflight.validate().unwrap();
    grouped_straddling_preflight
        .validate_against_sources(
            &grouped_straddling_branch_left,
            &grouped_straddling_branch_right,
        )
        .unwrap();
    assert_eq!(
        grouped_straddling_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    let grouped_straddling_result = hypermesh::exact::boolean_exact(
        &grouped_straddling_branch_left,
        &grouped_straddling_branch_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("grouped straddling-hole boolean should materialize");
    grouped_straddling_result
        .validate_operation_against_sources(
            &grouped_straddling_branch_left,
            &grouped_straddling_branch_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let orthogonal_grouped_straddling_branch_right =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                12, 12, 0, 16, 12, 0, 16, 32, 0, 12, 32, 0, //
                12, -2, 0, 16, -2, 0, 16, 8, 0, 12, 8, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("orthogonal grouped straddling-hole cutter fixture must import");
    assert!(arrange_coplanar_surface_multi_difference(
        &grouped_straddling_branch_left,
        &orthogonal_grouped_straddling_branch_right,
    )
    .is_none());
    let orthogonal_grouped_straddling = arrange_coplanar_surface_point_touch_difference(
        &grouped_straddling_branch_left,
        &orthogonal_grouped_straddling_branch_right,
    )
    .expect("orthogonal point-touch replay should consume a grouped straddling hole");
    orthogonal_grouped_straddling.validate().unwrap();
    orthogonal_grouped_straddling
        .validate_difference_against_sources(
            &grouped_straddling_branch_left,
            &orthogonal_grouped_straddling_branch_right,
        )
        .unwrap();
    let orthogonal_grouped_preflight = preflight_boolean_exact(
        &grouped_straddling_branch_left,
        &orthogonal_grouped_straddling_branch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("orthogonal grouped preflight should keep the point-touch shortcut");
    orthogonal_grouped_preflight.validate().unwrap();
    orthogonal_grouped_preflight
        .validate_against_sources(
            &grouped_straddling_branch_left,
            &orthogonal_grouped_straddling_branch_right,
        )
        .unwrap();
    assert_eq!(
        orthogonal_grouped_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    let orthogonal_grouped_result = hypermesh::exact::boolean_exact(
        &grouped_straddling_branch_left,
        &orthogonal_grouped_straddling_branch_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("orthogonal grouped straddling-hole boolean should materialize");
    orthogonal_grouped_result
        .validate_operation_against_sources(
            &grouped_straddling_branch_left,
            &orthogonal_grouped_straddling_branch_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let grouped_straddling_retained_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
            11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
            -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
            12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
            12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("grouped point-branch retained-hole cutter fixture must import");
    assert!(
        arrange_coplanar_surface_point_touch_difference(
            &grouped_straddling_branch_left,
            &grouped_straddling_retained_right,
        )
        .is_none()
    );
    let grouped_straddling_retained =
        arrange_coplanar_convex_surface_component_holed_difference(
            &grouped_straddling_branch_left,
            &grouped_straddling_retained_right,
        )
        .expect("component-holed replay should retain unrelated holes in grouped branch case");
    grouped_straddling_retained.validate().unwrap();
    grouped_straddling_retained
        .validate_against_sources(
            &grouped_straddling_branch_left,
            &grouped_straddling_retained_right,
        )
        .unwrap();
    assert_eq!(
        grouped_straddling_retained
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let grouped_retained_preflight = preflight_boolean_exact(
        &grouped_straddling_branch_left,
        &grouped_straddling_retained_right,
        ExactBooleanOperation::Difference,
    )
    .expect("grouped retained-hole preflight should classify component-holed shortcut");
    grouped_retained_preflight.validate().unwrap();
    grouped_retained_preflight
        .validate_against_sources(
            &grouped_straddling_branch_left,
            &grouped_straddling_retained_right,
        )
        .unwrap();
    assert_eq!(
        grouped_retained_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    let grouped_retained_result = hypermesh::exact::boolean_exact(
        &grouped_straddling_branch_left,
        &grouped_straddling_retained_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("grouped retained-hole boolean should materialize");
    grouped_retained_result
        .validate_operation_against_sources(
            &grouped_straddling_branch_left,
            &grouped_straddling_retained_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let orthogonal_grouped_straddling_retained_right =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
                11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
                -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
                12, 12, 0, 16, 12, 0, 16, 32, 0, 12, 32, 0, //
                12, -2, 0, 16, -2, 0, 16, 8, 0, 12, 8, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15, //
                16, 17, 18, 16, 18, 19,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("orthogonal grouped retained-hole cutter fixture must import");
    assert!(
        arrange_coplanar_surface_point_touch_difference(
            &grouped_straddling_branch_left,
            &orthogonal_grouped_straddling_retained_right,
        )
        .is_none()
    );
    let orthogonal_grouped_retained =
        arrange_coplanar_convex_surface_component_holed_difference(
            &grouped_straddling_branch_left,
            &orthogonal_grouped_straddling_retained_right,
        )
        .expect("orthogonal component-holed replay should retain unrelated holes");
    orthogonal_grouped_retained.validate().unwrap();
    orthogonal_grouped_retained
        .validate_against_sources(
            &grouped_straddling_branch_left,
            &orthogonal_grouped_straddling_retained_right,
        )
        .unwrap();
    assert_eq!(
        orthogonal_grouped_retained
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let orthogonal_grouped_retained_preflight = preflight_boolean_exact(
        &grouped_straddling_branch_left,
        &orthogonal_grouped_straddling_retained_right,
        ExactBooleanOperation::Difference,
    )
    .expect("orthogonal retained-hole preflight should classify component-holed shortcut");
    orthogonal_grouped_retained_preflight.validate().unwrap();
    orthogonal_grouped_retained_preflight
        .validate_against_sources(
            &grouped_straddling_branch_left,
            &orthogonal_grouped_straddling_retained_right,
        )
        .unwrap();
    assert_eq!(
        orthogonal_grouped_retained_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let multi_component_grouped_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 30, 0, 0, 30, 30, 0, 0, 30, 0, //
            40, 0, 0, 50, 0, 0, 50, 10, 0, 40, 10, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component grouped point-branch source fixture must import");
    let multi_component_grouped_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
            -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
            12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
            12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0, //
            43, 3, 0, 45, 3, 0, 45, 5, 0, 43, 5, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component grouped point-branch right fixture must import");
    assert!(
        arrange_coplanar_surface_point_touch_difference(
            &multi_component_grouped_left,
            &multi_component_grouped_right,
        )
        .is_none()
    );
    let multi_component_grouped =
        arrange_coplanar_convex_surface_component_holed_difference(
            &multi_component_grouped_left,
            &multi_component_grouped_right,
        )
        .expect("component-holed replay should carry source-local grouped branch output");
    multi_component_grouped.validate().unwrap();
    multi_component_grouped
        .validate_against_sources(&multi_component_grouped_left, &multi_component_grouped_right)
        .unwrap();
    assert_eq!(
        multi_component_grouped
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let multi_component_grouped_preflight = preflight_boolean_exact(
        &multi_component_grouped_left,
        &multi_component_grouped_right,
        ExactBooleanOperation::Difference,
    )
    .expect("multi-component grouped preflight should classify component-holed shortcut");
    multi_component_grouped_preflight.validate().unwrap();
    assert_eq!(
        multi_component_grouped_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &multi_component_grouped_left,
        &multi_component_grouped_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component grouped boolean should materialize")
    .validate_operation_against_sources(
        &multi_component_grouped_left,
        &multi_component_grouped_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let point_branch_straddling_retained = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 1, 0, 4, 1, 0, 4, 3, 0, 2, 3, 0, //
            7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-branch retained/straddling-hole fixture must import");
    assert!(
        arrange_coplanar_surface_point_touch_difference(&left, &point_branch_straddling_retained)
            .is_none()
    );
    let point_branch_straddling_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &left,
        &point_branch_straddling_retained,
    )
    .expect("point-branch component-holed replay should consume straddling holes");
    point_branch_straddling_holed.validate().unwrap();
    point_branch_straddling_holed
        .validate_against_sources(&left, &point_branch_straddling_retained)
        .unwrap();
    assert_eq!(
        point_branch_straddling_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let point_branch_straddling_holed_preflight = preflight_boolean_exact(
        &left,
        &point_branch_straddling_retained,
        ExactBooleanOperation::Difference,
    )
    .expect("point-branch component-holed preflight should classify shortcut");
    point_branch_straddling_holed_preflight
        .validate()
        .unwrap();
    point_branch_straddling_holed_preflight
        .validate_against_sources(&left, &point_branch_straddling_retained)
        .unwrap();
    assert_eq!(
        point_branch_straddling_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let multi_component_point_branch_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0, //
            30, 0, 0, 40, 0, 0, 40, 10, 0, 30, 10, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component point-branch source fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(&multi_component_point_branch_left, &point_only)
            .is_none()
    );
    let multi_component_point_branch = arrange_coplanar_surface_point_touch_difference(
        &multi_component_point_branch_left,
        &point_only,
    )
    .expect("source-local point-touch side-cutter difference should materialize");
    multi_component_point_branch.validate().unwrap();
    multi_component_point_branch
        .validate_difference_against_sources(&multi_component_point_branch_left, &point_only)
        .unwrap();
    assert!(multi_component_point_branch.polygons.len() >= 3);
    let multi_component_point_branch_preflight = preflight_boolean_exact(
        &multi_component_point_branch_left,
        &point_only,
        ExactBooleanOperation::Difference,
    )
    .expect("source-local point-touch side-cutter preflight should classify shortcut");
    multi_component_point_branch_preflight.validate().unwrap();
    multi_component_point_branch_preflight
        .validate_against_sources(&multi_component_point_branch_left, &point_only)
        .unwrap();
    assert_eq!(
        multi_component_point_branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    let multi_component_point_branch_result = hypermesh::exact::boolean_exact(
        &multi_component_point_branch_left,
        &point_only,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("source-local point-touch side-cutter boolean should materialize");
    multi_component_point_branch_result
        .validate_operation_against_sources(
            &multi_component_point_branch_left,
            &point_only,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        multi_component_point_branch_result.mesh,
        multi_component_point_branch.mesh
    );

    let multi_component_point_branch_straddling_right =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
                -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0, //
                33, 3, 0, 35, 3, 0, 35, 5, 0, 33, 5, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("multi-component point-branch straddling-hole fixture must import");
    let multi_component_point_branch_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &multi_component_point_branch_left,
            &multi_component_point_branch_straddling_right,
        )
        .expect("component-holed wrapper should carry a no-hole branch beside a retained hole");
    multi_component_point_branch_holed.validate().unwrap();
    multi_component_point_branch_holed
        .validate_against_sources(
            &multi_component_point_branch_left,
            &multi_component_point_branch_straddling_right,
        )
        .unwrap();
    assert_eq!(
        multi_component_point_branch_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let multi_component_point_branch_holed_preflight = preflight_boolean_exact(
        &multi_component_point_branch_left,
        &multi_component_point_branch_straddling_right,
        ExactBooleanOperation::Difference,
    )
    .expect("source-local branch/retained-hole preflight should classify component-holed shortcut");
    multi_component_point_branch_holed_preflight
        .validate()
        .unwrap();
    assert_eq!(
        multi_component_point_branch_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    let multi_component_point_branch_holed_result = hypermesh::exact::boolean_exact(
        &multi_component_point_branch_left,
        &multi_component_point_branch_straddling_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("source-local branch/retained-hole boolean should materialize");
    multi_component_point_branch_holed_result
        .validate_operation_against_sources(
            &multi_component_point_branch_left,
            &multi_component_point_branch_straddling_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let nonconvex_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 12, 20, 0, 12, 12, 0, 8, 12, 0, 8, 20, 0, 0, 20, 0,
            20, 12, 0, 0, 12, 0,
        ],
        &[
            0, 1, 8, 0, 8, 4, 0, 4, 5, 0, 5, 9, //
            9, 5, 6, 9, 6, 7, //
            4, 8, 2, 4, 2, 3,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex point-branch source fixture must import");
    let nonconvex_point_branch = arrange_coplanar_surface_point_touch_difference(
        &nonconvex_left,
        &point_only,
    )
    .expect("nonconvex source point-touch side cutters should materialize");
    nonconvex_point_branch.validate().unwrap();
    nonconvex_point_branch
        .validate_difference_against_sources(&nonconvex_left, &point_only)
        .unwrap();
    let nonconvex_point_branch_preflight =
        preflight_boolean_exact(&nonconvex_left, &point_only, ExactBooleanOperation::Difference)
            .expect("nonconvex point-touch side-cutter preflight should classify shortcut");
    nonconvex_point_branch_preflight.validate().unwrap();
    nonconvex_point_branch_preflight
        .validate_against_sources(&nonconvex_left, &point_only)
        .unwrap();
    assert_eq!(
        nonconvex_point_branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    let nonconvex_point_branch_result = hypermesh::exact::boolean_exact(
        &nonconvex_left,
        &point_only,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex point-touch side-cutter boolean should materialize");
    nonconvex_point_branch_result
        .validate_operation_against_sources(
            &nonconvex_left,
            &point_only,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(nonconvex_point_branch_result.mesh, nonconvex_point_branch.mesh);

    let nonconvex_point_branch_consumed =
        arrange_coplanar_surface_point_touch_difference(&nonconvex_left, &point_branch_consumed_hole)
            .expect("nonconvex point-touch side cutters should consume owned strict holes");
    nonconvex_point_branch_consumed.validate().unwrap();
    nonconvex_point_branch_consumed
        .validate_difference_against_sources(&nonconvex_left, &point_branch_consumed_hole)
        .unwrap();
    let nonconvex_consumed_preflight = preflight_boolean_exact(
        &nonconvex_left,
        &point_branch_consumed_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex point-touch consumed-hole preflight should classify shortcut");
    nonconvex_consumed_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_consumed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );

    let nonconvex_point_branch_straddling_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex point-branch straddling-hole fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(
            &nonconvex_left,
            &nonconvex_point_branch_straddling_hole,
        )
        .is_none()
    );
    let nonconvex_straddling_branch = arrange_coplanar_surface_point_touch_difference(
        &nonconvex_left,
        &nonconvex_point_branch_straddling_hole,
    )
    .expect("nonconvex point-touch side cutters should consume a straddling hole");
    nonconvex_straddling_branch.validate().unwrap();
    nonconvex_straddling_branch
        .validate_difference_against_sources(
            &nonconvex_left,
            &nonconvex_point_branch_straddling_hole,
        )
        .unwrap();
    let nonconvex_straddling_preflight = preflight_boolean_exact(
        &nonconvex_left,
        &nonconvex_point_branch_straddling_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex point-touch straddling-hole preflight should classify shortcut");
    nonconvex_straddling_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_straddling_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );

    let nonconvex_grouped_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 30, 0, 0, 30, 26, 0, 30, 30, 0, 22, 30, 0, 22, 26, 0, 20, 26, 0, 20, 30, 0,
            0, 30, 0, 0, 26, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 5, //
            0, 5, 6, //
            0, 6, 9, //
            9, 6, 7, //
            9, 7, 8, //
            5, 2, 3, //
            5, 3, 4,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex grouped source fixture must import");
    let nonconvex_grouped_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
            -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
            12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
            12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex grouped right fixture must import");
    let nonconvex_grouped = arrange_coplanar_surface_point_touch_difference(
        &nonconvex_grouped_left,
        &nonconvex_grouped_right,
    )
    .expect("nonconvex grouped branch should consume a straddling hole");
    nonconvex_grouped.validate().unwrap();
    nonconvex_grouped
        .validate_difference_against_sources(&nonconvex_grouped_left, &nonconvex_grouped_right)
        .unwrap();
    let nonconvex_grouped_preflight = preflight_boolean_exact(
        &nonconvex_grouped_left,
        &nonconvex_grouped_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex grouped branch preflight should classify shortcut");
    nonconvex_grouped_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_grouped_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    hypermesh::exact::boolean_exact(
        &nonconvex_grouped_left,
        &nonconvex_grouped_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex grouped branch boolean should materialize")
    .validate_operation_against_sources(
        &nonconvex_grouped_left,
        &nonconvex_grouped_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let nonconvex_grouped_retained_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 3, 0, 5, 3, 0, 5, 5, 0, 3, 5, 0, //
            11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
            -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
            12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
            12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex grouped retained right fixture must import");
    assert!(
        arrange_coplanar_surface_point_touch_difference(
            &nonconvex_grouped_left,
            &nonconvex_grouped_retained_right,
        )
        .is_none()
    );
    let nonconvex_grouped_retained =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_grouped_left,
            &nonconvex_grouped_retained_right,
        )
        .expect("nonconvex grouped component-holed branch should retain unrelated holes");
    nonconvex_grouped_retained.validate().unwrap();
    nonconvex_grouped_retained
        .validate_against_sources(&nonconvex_grouped_left, &nonconvex_grouped_retained_right)
        .unwrap();
    let nonconvex_grouped_retained_preflight = preflight_boolean_exact(
        &nonconvex_grouped_left,
        &nonconvex_grouped_retained_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex grouped retained preflight should classify shortcut");
    nonconvex_grouped_retained_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_grouped_retained_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &nonconvex_grouped_left,
        &nonconvex_grouped_retained_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex grouped retained boolean should materialize")
    .validate_operation_against_sources(
        &nonconvex_grouped_left,
        &nonconvex_grouped_retained_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let multi_component_nonconvex_grouped_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 30, 0, 0, 30, 26, 0, 30, 30, 0, 22, 30, 0, 22, 26, 0, 20, 26, 0, 20, 30, 0,
            0, 30, 0, 0, 26, 0, //
            40, 0, 0, 50, 0, 0, 50, 10, 0, 40, 10, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 5, //
            0, 5, 6, //
            0, 6, 9, //
            9, 6, 7, //
            9, 7, 8, //
            5, 2, 3, //
            5, 3, 4, //
            10, 11, 12, 10, 12, 13,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component nonconvex grouped source fixture must import");
    let multi_component_nonconvex_grouped_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            11, 11, 0, 13, 11, 0, 13, 13, 0, 11, 13, 0, //
            -2, 8, 0, 12, 8, 0, 12, 12, 0, -2, 12, 0, //
            12, 12, 0, 32, 12, 0, 32, 16, 0, 14, 16, 0, //
            12, 16, 0, 14, 16, 0, 14, 32, 0, 12, 32, 0, //
            43, 3, 0, 45, 3, 0, 45, 5, 0, 43, 5, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component nonconvex grouped right fixture must import");
    assert!(
        arrange_coplanar_surface_point_touch_difference(
            &multi_component_nonconvex_grouped_left,
            &multi_component_nonconvex_grouped_right,
        )
        .is_none()
    );
    let multi_component_nonconvex_grouped =
        arrange_coplanar_convex_surface_component_holed_difference(
            &multi_component_nonconvex_grouped_left,
            &multi_component_nonconvex_grouped_right,
        )
        .expect("component-holed replay should carry source-local nonconvex grouped branch output");
    multi_component_nonconvex_grouped.validate().unwrap();
    multi_component_nonconvex_grouped
        .validate_against_sources(
            &multi_component_nonconvex_grouped_left,
            &multi_component_nonconvex_grouped_right,
        )
        .unwrap();
    assert_eq!(
        multi_component_nonconvex_grouped
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let multi_component_nonconvex_grouped_preflight = preflight_boolean_exact(
        &multi_component_nonconvex_grouped_left,
        &multi_component_nonconvex_grouped_right,
        ExactBooleanOperation::Difference,
    )
    .expect("multi-component nonconvex grouped preflight should classify component-holed shortcut");
    multi_component_nonconvex_grouped_preflight
        .validate()
        .unwrap();
    assert_eq!(
        multi_component_nonconvex_grouped_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    hypermesh::exact::boolean_exact(
        &multi_component_nonconvex_grouped_left,
        &multi_component_nonconvex_grouped_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component nonconvex grouped boolean should materialize")
    .validate_operation_against_sources(
        &multi_component_nonconvex_grouped_left,
        &multi_component_nonconvex_grouped_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let nonconvex_point_branch_straddling_retained = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 1, 0, 5, 1, 0, 5, 3, 0, 3, 3, 0, //
            7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex point-branch mixed-hole fixture must import");
    assert!(
        arrange_coplanar_surface_point_touch_difference(
            &nonconvex_left,
            &nonconvex_point_branch_straddling_retained,
        )
        .is_none()
    );
    let nonconvex_straddling_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_left,
            &nonconvex_point_branch_straddling_retained,
        )
        .expect("nonconvex point-touch branch should retain unrelated holes");
    nonconvex_straddling_holed.validate().unwrap();
    nonconvex_straddling_holed
        .validate_against_sources(
            &nonconvex_left,
            &nonconvex_point_branch_straddling_retained,
        )
        .unwrap();
    assert_eq!(
        nonconvex_straddling_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let nonconvex_straddling_holed_preflight = preflight_boolean_exact(
        &nonconvex_left,
        &nonconvex_point_branch_straddling_retained,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex mixed-hole branch preflight should classify component-holed shortcut");
    nonconvex_straddling_holed_preflight.validate().unwrap();
    nonconvex_straddling_holed_preflight
        .validate_against_sources(
            &nonconvex_left,
            &nonconvex_point_branch_straddling_retained,
        )
        .unwrap();
    assert_eq!(
        nonconvex_straddling_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let multi_component_nonconvex_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 12, 20, 0, 12, 12, 0, 8, 12, 0, 8, 20, 0, 0, 20, 0,
            20, 12, 0, 0, 12, 0, //
            30, 0, 0, 40, 0, 0, 40, 10, 0, 30, 10, 0,
        ],
        &[
            0, 1, 8, 0, 8, 4, 0, 4, 5, 0, 5, 9, //
            9, 5, 6, 9, 6, 7, //
            4, 8, 2, 4, 2, 3, //
            10, 11, 12, 10, 12, 13,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-component nonconvex point-branch source fixture must import");
    let multi_component_nonconvex_point_branch = arrange_coplanar_surface_point_touch_difference(
        &multi_component_nonconvex_left,
        &point_only,
    )
    .expect("source-local nonconvex point-touch side cutters should materialize");
    multi_component_nonconvex_point_branch.validate().unwrap();
    multi_component_nonconvex_point_branch
        .validate_difference_against_sources(&multi_component_nonconvex_left, &point_only)
        .unwrap();
    assert!(multi_component_nonconvex_point_branch.polygons.len() >= 3);
    let multi_component_nonconvex_preflight = preflight_boolean_exact(
        &multi_component_nonconvex_left,
        &point_only,
        ExactBooleanOperation::Difference,
    )
    .expect("source-local nonconvex point-touch preflight should classify shortcut");
    multi_component_nonconvex_preflight.validate().unwrap();
    multi_component_nonconvex_preflight
        .validate_against_sources(&multi_component_nonconvex_left, &point_only)
        .unwrap();
    assert_eq!(
        multi_component_nonconvex_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfacePointTouchDifference
    );
    let multi_component_nonconvex_result = hypermesh::exact::boolean_exact(
        &multi_component_nonconvex_left,
        &point_only,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("source-local nonconvex point-touch boolean should materialize");
    multi_component_nonconvex_result
        .validate_operation_against_sources(
            &multi_component_nonconvex_left,
            &point_only,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        multi_component_nonconvex_result.mesh,
        multi_component_nonconvex_point_branch.mesh
    );

    let multi_component_nonconvex_point_branch_straddling_right =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                7, 9, 0, 9, 9, 0, 9, 11, 0, 7, 11, 0, //
                -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
                10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0, //
                33, 3, 0, 35, 3, 0, 35, 5, 0, 33, 5, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11, //
                12, 13, 14, 12, 14, 15,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("multi-component nonconvex branch/retained-hole fixture must import");
    let multi_component_nonconvex_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &multi_component_nonconvex_left,
            &multi_component_nonconvex_point_branch_straddling_right,
        )
        .expect("simple-source component-holed wrapper should carry a no-hole branch");
    multi_component_nonconvex_holed.validate().unwrap();
    multi_component_nonconvex_holed
        .validate_against_sources(
            &multi_component_nonconvex_left,
            &multi_component_nonconvex_point_branch_straddling_right,
        )
        .unwrap();
    assert_eq!(
        multi_component_nonconvex_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let multi_component_nonconvex_holed_preflight = preflight_boolean_exact(
        &multi_component_nonconvex_left,
        &multi_component_nonconvex_point_branch_straddling_right,
        ExactBooleanOperation::Difference,
    )
    .expect("source-local nonconvex branch/retained-hole preflight should classify shortcut");
    multi_component_nonconvex_holed_preflight
        .validate()
        .unwrap();
    assert_eq!(
        multi_component_nonconvex_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    let multi_component_nonconvex_holed_result = hypermesh::exact::boolean_exact(
        &multi_component_nonconvex_left,
        &multi_component_nonconvex_point_branch_straddling_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("source-local nonconvex branch/retained-hole boolean should materialize");
    multi_component_nonconvex_holed_result
        .validate_operation_against_sources(
            &multi_component_nonconvex_left,
            &multi_component_nonconvex_point_branch_straddling_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let incidental_point_cutters = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 8, 0, 8, 8, 0, 8, 12, 0, 0, 12, 0, //
            0, 11, 0, 10, 12, 0, 0, 15, 0, //
            8, 12, 0, 0, 14, 0, 0, 18, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, //
            7, 8, 9,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("incidental point side-cutter fixture must import");
    let incidental_opening =
        arrange_coplanar_surface_side_cutter_difference(&left, &incidental_point_cutters)
            .expect("positive side-cutter group should ignore non-connective point contact");
    incidental_opening.validate().unwrap();
    incidental_opening
        .validate_side_cutter_difference_against_sources(&left, &incidental_point_cutters)
        .unwrap();
    let incidental_preflight = preflight_boolean_exact(
        &left,
        &incidental_point_cutters,
        ExactBooleanOperation::Difference,
    )
    .expect("incidental side-cutter preflight should classify shortcut");
    incidental_preflight.validate().unwrap();
    assert_eq!(
        incidental_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceSideCutterDifference
    );
    hypermesh::exact::boolean_exact(
        &left,
        &incidental_point_cutters,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("incidental side-cutter boolean should materialize")
    .validate_operation_against_sources(
        &left,
        &incidental_point_cutters,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_component_coplanar_intersection() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
            8, 0, 0, 12, 0, 0, 12, 4, 0, 8, 4, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component intersection left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[2, 1, 0, 10, 1, 0, 10, 3, 0, 2, 3, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component intersection right fixture must import");

    let intersection = arrange_coplanar_convex_surface_multi_intersection(&left, &right)
        .expect("component hull intersection should retain two exact loops");
    intersection.validate().unwrap();
    intersection
        .validate_intersection_against_sources(&left, &right)
        .unwrap();
    assert_eq!(intersection.polygons.len(), 2);
    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection)
        .expect("component intersection preflight should classify shortcut");
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceIntersection
    );

    let touching_right = ExactMesh::from_i64_triangles_with_policy(
        &[4, 0, 0, 8, 0, 0, 8, 4, 0, 4, 4, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component intersection touching fixture must import");
    assert!(arrange_coplanar_convex_surface_multi_intersection(&left, &touching_right).is_none());

    let nonconvex_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, 1, 2, 0, 0, 2, 0, 0, 1, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 0, 3, 6, 6, 3, 4, 6, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex intersection left fixture must import");
    let covering = ExactMesh::from_i64_triangles_with_policy(
        &[-1, -1, 0, 6, -1, 0, -1, 6, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex intersection covering fixture must import");
    let nonconvex = arrange_coplanar_surface_component_intersection(&nonconvex_left, &covering)
        .expect("adjacent face-cell clips should materialize as one nonconvex loop");
    nonconvex.validate().unwrap();
    nonconvex
        .validate_intersection_against_sources(&nonconvex_left, &covering)
        .unwrap();
    assert!(arrange_coplanar_surface_multi_component_intersection(&nonconvex_left, &covering)
        .is_none());
    let nonconvex_preflight =
        preflight_boolean_exact(&nonconvex_left, &covering, ExactBooleanOperation::Intersection)
            .expect("nonconvex face-cell intersection preflight should classify shortcut");
    nonconvex_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    let nonconvex_result = hypermesh::exact::boolean_exact(
        &nonconvex_left,
        &covering,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex face-cell intersection boolean should materialize");
    nonconvex_result
        .validate_operation_against_sources(
            &nonconvex_left,
            &covering,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let disconnected_nonconvex_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, 1, 2, 0, 0, 2, 0, 0, 1, 0, //
            5, 0, 0, 7, 0, 0, 7, 1, 0, 6, 1, 0, 6, 2, 0, 5, 2, 0, 5, 1, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, 0, 3, 6, 6, 3, 4, 6, 4, 5, //
            7, 8, 9, 7, 9, 10, 7, 10, 13, 13, 10, 11, 13, 11, 12,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("disconnected nonconvex intersection left fixture must import");
    let disconnected_covering = ExactMesh::from_i64_triangles_with_policy(
        &[-1, -1, 0, 12, -1, 0, -1, 6, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("disconnected nonconvex intersection covering fixture must import");
    assert!(
        arrange_coplanar_surface_component_intersection(
            &disconnected_nonconvex_left,
            &disconnected_covering
        )
        .is_none()
    );
    let disconnected_nonconvex = arrange_coplanar_surface_multi_component_intersection(
        &disconnected_nonconvex_left,
        &disconnected_covering,
    )
    .expect("disconnected adjacent face-cell clips should retain two nonconvex loops");
    disconnected_nonconvex.validate().unwrap();
    disconnected_nonconvex
        .validate_intersection_against_sources(&disconnected_nonconvex_left, &disconnected_covering)
        .unwrap();
    assert_eq!(disconnected_nonconvex.polygons.len(), 2);
    assert!(
        disconnected_nonconvex
            .polygons
            .iter()
            .all(|polygon| polygon.len() == 6)
    );
    let disconnected_preflight = preflight_boolean_exact(
        &disconnected_nonconvex_left,
        &disconnected_covering,
        ExactBooleanOperation::Intersection,
    )
    .expect("disconnected nonconvex intersection preflight should classify shortcut");
    disconnected_preflight.validate().unwrap();
    assert_eq!(
        disconnected_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceIntersection
    );
    let disconnected_result = hypermesh::exact::boolean_exact(
        &disconnected_nonconvex_left,
        &disconnected_covering,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("disconnected nonconvex intersection boolean should materialize");
    disconnected_result
        .validate_operation_against_sources(
            &disconnected_nonconvex_left,
            &disconnected_covering,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_component_coplanar_difference() {
    let left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
            4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component difference left fixture must import");
    let right = ExactMesh::from_i64_triangles_with_policy(
        &[1, -1, 0, 3, -1, 0, 3, 3, 0, 1, 3, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component difference right fixture must import");

    let difference = arrange_coplanar_convex_surface_multi_difference(&left, &right)
        .expect("component-wise difference should retain cut and untouched loops");
    difference.validate().unwrap();
    difference.validate_against_sources(&left, &right).unwrap();
    assert_eq!(difference.polygons.len(), 2);

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .expect("component-wise difference preflight should classify shortcut");
    preflight.validate().unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
    );

    let result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-wise difference should materialize");
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let boundary_bridge = ExactMesh::from_i64_triangles_with_policy(
        &[2, 0, 0, 4, 0, 0, 4, 2, 0, 2, 2, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component difference boundary-only fixture must import");
    assert!(arrange_coplanar_convex_surface_multi_difference(&left, &boundary_bridge).is_none());

    let multi_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, //
            4, 0, 0, 6, 0, 0, 6, 2, 0, 4, 2, 0, //
            8, 0, 0, 10, 0, 0, 10, 2, 0, 8, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-cutter component difference left fixture must import");
    let multi_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, -1, 0, 3, -1, 0, 3, 3, 0, 1, 3, 0, //
            5, -1, 0, 7, -1, 0, 7, 3, 0, 5, 3, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("multi-cutter component difference right fixture must import");
    let multi_difference =
        arrange_coplanar_convex_surface_multi_difference(&multi_left, &multi_right)
            .expect("independent right cutters should retain three exact output loops");
    multi_difference.validate().unwrap();
    multi_difference
        .validate_against_sources(&multi_left, &multi_right)
        .unwrap();
    assert_eq!(multi_difference.polygons.len(), 3);
    let multi_preflight =
        preflight_boolean_exact(&multi_left, &multi_right, ExactBooleanOperation::Difference)
            .expect("multi-cutter difference preflight should classify shortcut");
    multi_preflight.validate().unwrap();
    assert_eq!(
        multi_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
    );

    let wide_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 6, 0, 0, 6, 2, 0, 0, 2, 0, //
            10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-left double-cutter fixture must import");
    let two_cutters_one_component = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, -1, 0, 2, -1, 0, 2, 3, 0, 1, 3, 0, //
            4, -1, 0, 5, -1, 0, 5, 3, 0, 4, 3, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-left double-cutter right fixture must import");
    let same_component_multi_cutter =
        arrange_coplanar_convex_surface_multi_difference(&wide_left, &two_cutters_one_component)
            .expect("full-span double cutter should split one retained component exactly");
    same_component_multi_cutter.validate().unwrap();
    same_component_multi_cutter
        .validate_against_sources(&wide_left, &two_cutters_one_component)
        .unwrap();
    assert_eq!(same_component_multi_cutter.polygons.len(), 4);

    let corner_cutter_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
            20, 0, 0, 22, 0, 0, 22, 2, 0, 20, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectangular multi-cutter left fixture must import");
    let nonrectangular_corner_cutters = ExactMesh::from_i64_triangles_with_policy(
        &[-1, -1, 0, 3, -1, 0, -1, 3, 0, 7, 11, 0, 11, 7, 0, 11, 11, 0],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectangular multi-cutter right fixture must import");
    let nonrectangular_multi_cutter = arrange_coplanar_convex_surface_multi_difference(
        &corner_cutter_left,
        &nonrectangular_corner_cutters,
    )
    .expect("sequential exact corner cutters should retain convex remnants");
    nonrectangular_multi_cutter.validate().unwrap();
    nonrectangular_multi_cutter
        .validate_against_sources(&corner_cutter_left, &nonrectangular_corner_cutters)
        .unwrap();
    assert_eq!(nonrectangular_multi_cutter.polygons.len(), 2);
    assert!(
        nonrectangular_multi_cutter
            .polygons
            .iter()
            .any(|polygon| polygon.len() == 6)
    );
    let nonrectangular_preflight = preflight_boolean_exact(
        &corner_cutter_left,
        &nonrectangular_corner_cutters,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectangular multi-cutter preflight should classify shortcut");
    nonrectangular_preflight.validate().unwrap();
    assert_eq!(
        nonrectangular_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceMultiDifference
    );

    let nonconvex_multi_cutter_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            -1, -1, 0, 3, -1, 0, -1, 3, 0, //
            8, 4, 0, 11, 4, 0, 11, 6, 0, 8, 6, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex multi-cutter right fixture must import");
    assert!(
        arrange_coplanar_convex_surface_multi_difference(
            &corner_cutter_left,
            &nonconvex_multi_cutter_right,
        )
        .is_none()
    );
    let nonconvex_multi_cutter = arrange_coplanar_surface_multi_difference(
        &corner_cutter_left,
        &nonconvex_multi_cutter_right,
    )
    .expect("nonconvex simple loop plus a far component should materialize");
    nonconvex_multi_cutter.validate().unwrap();
    nonconvex_multi_cutter
        .validate_difference_against_sources(&corner_cutter_left, &nonconvex_multi_cutter_right)
        .unwrap();
    assert_eq!(nonconvex_multi_cutter.polygons.len(), 2);
    assert!(
        nonconvex_multi_cutter
            .polygons
            .iter()
            .any(|polygon| polygon.len() == 9)
    );
    let nonconvex_preflight = preflight_boolean_exact(
        &corner_cutter_left,
        &nonconvex_multi_cutter_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex multi-cutter preflight should classify shortcut");
    nonconvex_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let component_opening_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0, //
            10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-opening left fixture must import");
    let component_opening_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 6, 2, 0, 6, 6, 0, 2, 6, 0, //
            10, 0, 0, 12, 0, 0, 12, 2, 0, 10, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-opening right fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(&component_opening_left, &component_opening_right)
            .is_none()
    );
    let component_opening =
        arrange_coplanar_surface_component_difference(&component_opening_left, &component_opening_right)
            .expect("single retained nonconvex remnant should materialize");
    component_opening.validate().unwrap();
    component_opening
        .validate_component_difference_against_sources(
            &component_opening_left,
            &component_opening_right,
        )
        .unwrap();
    let component_opening_preflight = preflight_boolean_exact(
        &component_opening_left,
        &component_opening_right,
        ExactBooleanOperation::Difference,
    )
    .expect("component-opening preflight should classify single-loop shortcut");
    component_opening_preflight.validate().unwrap();
    assert_eq!(
        component_opening_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );
    let component_opening_result = hypermesh::exact::boolean_exact(
        &component_opening_left,
        &component_opening_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-opening difference should materialize");
    component_opening_result
        .validate_operation_against_sources(
            &component_opening_left,
            &component_opening_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let nonconvex_source_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 10, 0, 0, 10, 4, 0, 7, 4, 0, 6, 6, 0, 10, 8, 0, 10, 12, 0, 0, 12, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 3, 4, //
            0, 4, 7, //
            7, 4, 5, //
            7, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source side-opening left fixture must import");
    let nonconvex_source_opening = ExactMesh::from_i64_triangles_with_policy(
        &[2, 12, 0, 5, 9, 0, 7, 10, 0, 4, 12, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source side-opening right fixture must import");
    assert!(
        arrange_coplanar_convex_surface_multi_difference(
            &nonconvex_source_left,
            &nonconvex_source_opening,
        )
        .is_none()
    );
    let nonconvex_source_difference = arrange_coplanar_surface_component_difference(
        &nonconvex_source_left,
        &nonconvex_source_opening,
    )
    .expect("side-attached cutter on a nonconvex source disk should replay exactly");
    nonconvex_source_difference.validate().unwrap();
    nonconvex_source_difference
        .validate_component_difference_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_opening,
        )
        .unwrap();
    let nonconvex_source_preflight = preflight_boolean_exact(
        &nonconvex_source_left,
        &nonconvex_source_opening,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex source side-opening preflight should classify shortcut");
    nonconvex_source_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_source_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );

    let nonconvex_source_hole = ExactMesh::from_i64_triangles_with_policy(
        &[2, 2, 0, 3, 2, 0, 2, 3, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source strict-hole fixture must import");
    assert!(
        arrange_coplanar_surface_component_difference(
            &nonconvex_source_left,
            &nonconvex_source_hole,
        )
        .is_none()
    );

    let nonconvex_source_multi_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 10, 0, 0, 10, 4, 0, 7, 4, 0, 6, 6, 0, 10, 8, 0, 10, 12, 0, 0, 12, 0, //
            20, 0, 0, 24, 0, 0, 24, 4, 0, 20, 4, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 3, 4, //
            0, 4, 7, //
            7, 4, 5, //
            7, 5, 6, //
            8, 9, 10, //
            8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source side-opening multi-left fixture must import");
    let nonconvex_source_multi = arrange_coplanar_surface_multi_difference(
        &nonconvex_source_multi_left,
        &nonconvex_source_opening,
    )
    .expect("nonconvex source side opening plus untouched component should emit two loops");
    nonconvex_source_multi.validate().unwrap();
    nonconvex_source_multi
        .validate_difference_against_sources(
            &nonconvex_source_multi_left,
            &nonconvex_source_opening,
        )
        .unwrap();

    let nonconvex_source_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_source_left,
            &nonconvex_source_hole,
        )
        .expect("strict hole in a retained nonconvex source disk should materialize");
    nonconvex_source_holed.validate().unwrap();
    nonconvex_source_holed
        .validate_against_sources(&nonconvex_source_left, &nonconvex_source_hole)
        .unwrap();
    let nonconvex_source_holed_preflight = preflight_boolean_exact(
        &nonconvex_source_left,
        &nonconvex_source_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex source retained-hole preflight should classify shortcut");
    nonconvex_source_holed_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_source_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let nonconvex_source_opening_and_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 3, 2, 0, 2, 3, 0, //
            2, 12, 0, 5, 9, 0, 7, 10, 0, 4, 12, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source opening-and-hole fixture must import");
    let nonconvex_source_opening_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_source_left,
            &nonconvex_source_opening_and_hole,
        )
        .expect("side opening and unrelated hole should replay on a nonconvex source disk");
    nonconvex_source_opening_holed.validate().unwrap();
    nonconvex_source_opening_holed
        .validate_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_opening_and_hole,
        )
        .unwrap();

    let nonconvex_point_branch_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 20, 0, 12, 20, 0, 12, 12, 0, 8, 12, 0, 8, 20, 0, 0, 20, 0,
            20, 12, 0, 0, 12, 0,
        ],
        &[
            0, 1, 8, 0, 8, 4, 0, 4, 5, 0, 5, 9, //
            9, 5, 6, 9, 6, 7, //
            4, 8, 2, 4, 2, 3,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex point-branch component-holed source fixture must import");
    let nonconvex_point_branch_hole_and_cutters = ExactMesh::from_i64_triangles_with_policy(
        &[
            3, 1, 0, 5, 1, 0, 5, 3, 0, 3, 3, 0, //
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex point-branch component-holed right fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(
            &nonconvex_point_branch_left,
            &nonconvex_point_branch_hole_and_cutters,
        )
        .is_none()
    );
    let nonconvex_point_branch_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_point_branch_left,
            &nonconvex_point_branch_hole_and_cutters,
        )
        .expect("nonconvex point-branch side cutters should retain unrelated holes");
    nonconvex_point_branch_holed.validate().unwrap();
    nonconvex_point_branch_holed
        .validate_against_sources(
            &nonconvex_point_branch_left,
            &nonconvex_point_branch_hole_and_cutters,
        )
        .unwrap();
    assert!(
        nonconvex_point_branch_holed.components.len() >= 2
            && nonconvex_point_branch_holed
                .components
                .iter()
                .map(|component| component.holes.len())
                .sum::<usize>()
                == 1
    );
    let nonconvex_point_branch_holed_preflight = preflight_boolean_exact(
        &nonconvex_point_branch_left,
        &nonconvex_point_branch_hole_and_cutters,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex point-branch component-holed preflight should classify shortcut");
    nonconvex_point_branch_holed_preflight.validate().unwrap();
    nonconvex_point_branch_holed_preflight
        .validate_against_sources(
            &nonconvex_point_branch_left,
            &nonconvex_point_branch_hole_and_cutters,
        )
        .unwrap();
    assert_eq!(
        nonconvex_point_branch_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let nonconvex_source_crossing_opening = ExactMesh::from_i64_triangles_with_policy(
        &[4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source crossing-opening fixture must import");
    let nonconvex_source_clipped_difference = arrange_coplanar_surface_component_difference(
        &nonconvex_source_left,
        &nonconvex_source_crossing_opening,
    )
    .expect("crossing cutter should clip into a nonconvex source opening");
    nonconvex_source_clipped_difference.validate().unwrap();
    nonconvex_source_clipped_difference
        .validate_component_difference_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_crossing_opening,
        )
        .unwrap();

    let nonconvex_source_crossing_opening_consumed_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            5, 9, 0, 7, 9, 0, 7, 11, 0, 5, 11, 0, //
            4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source crossing consumed-hole fixture must import");
    assert!(
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_source_left,
            &nonconvex_source_crossing_opening_consumed_hole,
        )
        .is_none()
    );
    let nonconvex_source_consumed_clipped = arrange_coplanar_surface_component_difference(
        &nonconvex_source_left,
        &nonconvex_source_crossing_opening_consumed_hole,
    )
    .expect("clipped crossing opening should consume a partially overlapping strict hole");
    nonconvex_source_consumed_clipped.validate().unwrap();
    nonconvex_source_consumed_clipped
        .validate_component_difference_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_crossing_opening_consumed_hole,
        )
        .unwrap();
    let nonconvex_source_consumed_clipped_preflight = preflight_boolean_exact(
        &nonconvex_source_left,
        &nonconvex_source_crossing_opening_consumed_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("clipped consumed-hole preflight should classify component difference");
    nonconvex_source_consumed_clipped_preflight
        .validate()
        .unwrap();
    assert_eq!(
        nonconvex_source_consumed_clipped_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );

    let nonconvex_split_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 30, 0, 0, 30, 10, 0, 10, 10, 0, 10, 30, 0, 0, 30, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 3, 5, //
            3, 4, 5,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex split source fixture must import");
    let nonconvex_split_crossing_consumed_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            7, 12, 0, 9, 12, 0, 9, 14, 0, 7, 14, 0, //
            8, -2, 0, 12, -2, 0, 12, 32, 0, 8, 32, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex split consumed-hole fixture must import");
    assert!(
        arrange_coplanar_surface_component_difference(
            &nonconvex_split_left,
            &nonconvex_split_crossing_consumed_hole,
        )
        .is_none()
    );
    let nonconvex_split_consumed = arrange_coplanar_surface_multi_difference(
        &nonconvex_split_left,
        &nonconvex_split_crossing_consumed_hole,
    )
    .expect("clipped side-to-side opening should split after consuming its hole");
    nonconvex_split_consumed.validate().unwrap();
    nonconvex_split_consumed
        .validate_difference_against_sources(
            &nonconvex_split_left,
            &nonconvex_split_crossing_consumed_hole,
        )
        .unwrap();
    assert_eq!(nonconvex_split_consumed.polygons.len(), 2);
    let nonconvex_split_preflight = preflight_boolean_exact(
        &nonconvex_split_left,
        &nonconvex_split_crossing_consumed_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("split consumed-hole preflight should classify multi-difference");
    nonconvex_split_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_split_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let nonconvex_split_crossing_consumed_and_retained_holes =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                20, 2, 0, 22, 2, 0, 22, 4, 0, 20, 4, 0, //
                7, 12, 0, 9, 12, 0, 9, 14, 0, 7, 14, 0, //
                8, -2, 0, 12, -2, 0, 12, 32, 0, 8, 32, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("nonconvex split consumed-and-retained-hole fixture must import");
    let nonconvex_split_consumed_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_split_left,
            &nonconvex_split_crossing_consumed_and_retained_holes,
        )
        .expect("clipped side-to-side opening should split while retaining unrelated holes");
    nonconvex_split_consumed_holed.validate().unwrap();
    nonconvex_split_consumed_holed
        .validate_against_sources(
            &nonconvex_split_left,
            &nonconvex_split_crossing_consumed_and_retained_holes,
        )
        .unwrap();
    assert_eq!(nonconvex_split_consumed_holed.components.len(), 2);
    assert_eq!(
        nonconvex_split_consumed_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        1
    );
    let mut stale_nonconvex_split = nonconvex_split_consumed_holed.clone();
    stale_nonconvex_split.components[0].holes.push(vec![
        point3(7, 12, 0),
        point3(9, 12, 0),
        point3(9, 14, 0),
        point3(7, 14, 0),
    ]);
    assert!(
        stale_nonconvex_split
            .validate_against_sources(
                &nonconvex_split_left,
                &nonconvex_split_crossing_consumed_and_retained_holes,
            )
            .is_err()
    );

    let nonconvex_source_overlapping_crossing_openings =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0, //
                2, 8, 0, 8, 8, 0, 8, 14, 0, 2, 14, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("nonconvex source overlapping-crossing fixture must import");
    let nonconvex_source_merged_crossing = arrange_coplanar_surface_component_difference(
        &nonconvex_source_left,
        &nonconvex_source_overlapping_crossing_openings,
    )
    .expect("overlapping crossing cutters should merge before source subtraction");
    nonconvex_source_merged_crossing.validate().unwrap();
    nonconvex_source_merged_crossing
        .validate_component_difference_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_overlapping_crossing_openings,
        )
        .unwrap();

    let nonconvex_source_incidental_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 20, 0, 0, 20, 8, 0, 16, 8, 0, 16, 12, 0, 20, 12, 0, 20, 20, 0, 0, 20, 0,
        ],
        &[
            0, 1, 2, //
            0, 2, 3, //
            0, 3, 4, //
            0, 4, 7, //
            7, 4, 5, //
            7, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex incidental-point source fixture must import");
    let nonconvex_source_incidental_openings = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 8, 0, 8, 8, 0, 8, 12, 0, 0, 12, 0, //
            0, 11, 0, 10, 12, 0, 0, 15, 0, //
            8, 12, 0, 0, 14, 0, 0, 18, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, //
            7, 8, 9,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex incidental-point opening fixture must import");
    assert!(
        arrange_coplanar_convex_surface_multi_difference(
            &nonconvex_source_incidental_left,
            &nonconvex_source_incidental_openings,
        )
        .is_none()
    );
    let nonconvex_source_incidental = arrange_coplanar_surface_component_difference(
        &nonconvex_source_incidental_left,
        &nonconvex_source_incidental_openings,
    )
    .expect("incidental point-only openings should replay through positive contact");
    nonconvex_source_incidental.validate().unwrap();
    nonconvex_source_incidental
        .validate_component_difference_against_sources(
            &nonconvex_source_incidental_left,
            &nonconvex_source_incidental_openings,
        )
        .unwrap();
    let nonconvex_source_incidental_preflight = preflight_boolean_exact(
        &nonconvex_source_incidental_left,
        &nonconvex_source_incidental_openings,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex incidental-point preflight should classify shortcut");
    nonconvex_source_incidental_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_source_incidental_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceArrangementDifference
    );

    let nonconvex_source_crossing_opening_and_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 3, 2, 0, 2, 3, 0, //
            4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source crossing-opening-and-hole fixture must import");
    let nonconvex_source_clipped_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_source_left,
            &nonconvex_source_crossing_opening_and_hole,
        )
        .expect("crossing cutter should clip while retaining unrelated holes");
    nonconvex_source_clipped_holed.validate().unwrap();
    nonconvex_source_clipped_holed
        .validate_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_crossing_opening_and_hole,
        )
        .unwrap();
    assert_eq!(nonconvex_source_clipped_holed.components[0].holes.len(), 1);

    let nonconvex_source_overlapping_crossing_openings_and_hole =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 3, 2, 0, 2, 3, 0, //
                4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0, //
                2, 8, 0, 8, 8, 0, 8, 14, 0, 2, 14, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("nonconvex source overlapping-crossing-and-hole fixture must import");
    let nonconvex_source_merged_crossing_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_source_left,
            &nonconvex_source_overlapping_crossing_openings_and_hole,
        )
        .expect("overlapping crossing cutters should merge while retaining unrelated holes");
    nonconvex_source_merged_crossing_holed.validate().unwrap();
    nonconvex_source_merged_crossing_holed
        .validate_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_overlapping_crossing_openings_and_hole,
        )
        .unwrap();
    assert_eq!(
        nonconvex_source_merged_crossing_holed.components[0]
            .holes
            .len(),
        1
    );

    let nonconvex_source_clipped_straddling_hole_and_retained_hole =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                2, 2, 0, 3, 2, 0, 2, 3, 0, //
                5, 9, 0, 7, 9, 0, 7, 11, 0, 5, 11, 0, //
                4, 10, 0, 12, 10, 0, 12, 14, 0, 4, 14, 0,
            ],
            &[
                0, 1, 2, //
                3, 4, 5, 3, 5, 6, //
                7, 8, 9, 7, 9, 10,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("nonconvex source clipped straddling-hole fixture must import");
    let nonconvex_source_clipped_straddling_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_source_left,
            &nonconvex_source_clipped_straddling_hole_and_retained_hole,
        )
        .expect("clipped crossing opening should consume one hole and retain another");
    nonconvex_source_clipped_straddling_holed
        .validate()
        .unwrap();
    nonconvex_source_clipped_straddling_holed
        .validate_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_clipped_straddling_hole_and_retained_hole,
        )
        .unwrap();
    assert_eq!(
        nonconvex_source_clipped_straddling_holed.components[0]
            .holes
            .len(),
        1
    );
    let mut stale_clipped_straddling = nonconvex_source_clipped_straddling_holed.clone();
    stale_clipped_straddling.components[0].holes.push(vec![
        point3(5, 9, 0),
        point3(7, 9, 0),
        point3(7, 11, 0),
        point3(5, 11, 0),
    ]);
    assert!(
        stale_clipped_straddling
            .validate_against_sources(
                &nonconvex_source_left,
                &nonconvex_source_clipped_straddling_hole_and_retained_hole,
            )
            .is_err()
    );

    let nonconvex_source_straddling_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 3, 2, 0, 2, 3, 0, //
            5, 10, 0, 7, 10, 0, 7, 11, 0, 5, 11, 0, //
            2, 8, 0, 6, 8, 0, 6, 12, 0, 2, 12, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, 7, 9, 10,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source straddling-hole fixture must import");
    let nonconvex_source_straddling =
        arrange_coplanar_convex_surface_component_holed_difference(
            &nonconvex_source_left,
            &nonconvex_source_straddling_hole,
        )
        .expect("nonconvex source side opening should consume overlapping strict holes");
    nonconvex_source_straddling.validate().unwrap();
    nonconvex_source_straddling
        .validate_against_sources(
            &nonconvex_source_left,
            &nonconvex_source_straddling_hole,
        )
        .unwrap();
    assert_eq!(nonconvex_source_straddling.components[0].holes.len(), 1);
    let mut stale_nonconvex_source_straddling = nonconvex_source_straddling.clone();
    stale_nonconvex_source_straddling.components[0]
        .holes
        .push(vec![
            point3(5, 10, 0),
            point3(7, 10, 0),
            point3(7, 11, 0),
            point3(5, 11, 0),
        ]);
    assert!(
        stale_nonconvex_source_straddling
            .validate_against_sources(
                &nonconvex_source_left,
                &nonconvex_source_straddling_hole,
            )
            .is_err()
    );

    let nonconvex_source_boundary_touching_hole = ExactMesh::from_i64_triangles_with_policy(
        &[0, 4, 0, 1, 4, 0, 1, 5, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex source boundary-touching hole fixture must import");
    assert!(arrange_coplanar_convex_surface_component_holed_difference(
        &nonconvex_source_left,
        &nonconvex_source_boundary_touching_hole,
    )
    .is_none());

    let partial_height_cutters = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, //
            4, -1, 0, 5, -1, 0, 5, 3, 0, 4, 3, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial-height double-cutter right fixture must import");
    assert!(
        arrange_coplanar_convex_surface_multi_difference(&wide_left, &partial_height_cutters)
            .is_none()
    );
    let partial_height_nonconvex =
        arrange_coplanar_surface_multi_difference(&wide_left, &partial_height_cutters)
            .expect("partial-height multi-cutter should retain no-hole nonconvex loops");
    partial_height_nonconvex.validate().unwrap();
    partial_height_nonconvex
        .validate_difference_against_sources(&wide_left, &partial_height_cutters)
        .unwrap();
    assert_eq!(partial_height_nonconvex.polygons.len(), 3);
    let partial_height_preflight = preflight_boolean_exact(
        &wide_left,
        &partial_height_cutters,
        ExactBooleanOperation::Difference,
    )
    .expect("partial-height multi-cutter preflight should classify shortcut");
    partial_height_preflight.validate().unwrap();
    assert_eq!(
        partial_height_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    let partial_height_result = hypermesh::exact::boolean_exact(
        &wide_left,
        &partial_height_cutters,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial-height multi-cutter boolean should materialize");
    partial_height_result.validate().unwrap();
    assert_eq!(
        partial_height_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarSurfaceMultiDifference
        }
    );
    let contained_hole_cutters = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 0, 0, 2, 0, 0, 2, 1, 0, 1, 1, 0, //
            4, -1, 0, 5, -1, 0, 5, 3, 0, 4, 3, 0, //
            11, 1, 0, 12, 1, 0, 12, 2, 0, 11, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("hole-producing partial-height cutter fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(&wide_left, &contained_hole_cutters).is_none()
    );

    let channel_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectilinear channel left fixture must import");
    let nonrectilinear_channel_cutters = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
            -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectilinear channel cutter fixture must import");
    assert!(
        arrange_coplanar_convex_surface_multi_difference(
            &channel_left,
            &nonrectilinear_channel_cutters,
        )
        .is_none()
    );
    let channel_difference = arrange_coplanar_surface_multi_difference(
        &channel_left,
        &nonrectilinear_channel_cutters,
    )
    .expect("nonrectilinear side-cutter channel should retain split components");
    channel_difference.validate().unwrap();
    channel_difference
        .validate_difference_against_sources(&channel_left, &nonrectilinear_channel_cutters)
        .unwrap();
    assert_eq!(channel_difference.polygons.len(), 2);
    let channel_preflight = preflight_boolean_exact(
        &channel_left,
        &nonrectilinear_channel_cutters,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectilinear channel preflight should classify shortcut");
    channel_preflight.validate().unwrap();
    assert_eq!(
        channel_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );
    let nonrectilinear_channel_retained_hole_cutters =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                15, 4, 0, 17, 4, 0, 17, 6, 0, 15, 6, 0, //
                8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
                -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("nonrectilinear channel retained-hole cutter fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(
            &channel_left,
            &nonrectilinear_channel_retained_hole_cutters,
        )
        .is_none()
    );

    let nonrectilinear_channel_consumed_hole_cutters =
        ExactMesh::from_i64_triangles_with_policy(
            &[
                1, 5, 0, 2, 5, 0, 2, 6, 0, 1, 6, 0, //
                8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
                -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0,
            ],
            &[
                0, 1, 2, 0, 2, 3, //
                4, 5, 6, 4, 6, 7, //
                8, 9, 10, 8, 10, 11,
            ],
            ValidationPolicy::ALLOW_BOUNDARY,
        )
        .expect("nonrectilinear channel consumed-hole cutter fixture must import");
    let consumed_channel_difference = arrange_coplanar_surface_multi_difference(
        &channel_left,
        &nonrectilinear_channel_consumed_hole_cutters,
    )
    .expect("nonrectilinear side-cutter split should consume strict interior holes");
    consumed_channel_difference.validate().unwrap();
    consumed_channel_difference
        .validate_difference_against_sources(
            &channel_left,
            &nonrectilinear_channel_consumed_hole_cutters,
        )
        .unwrap();
    assert_eq!(consumed_channel_difference.polygons.len(), 2);
    let consumed_channel_preflight = preflight_boolean_exact(
        &channel_left,
        &nonrectilinear_channel_consumed_hole_cutters,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectilinear consumed-hole channel preflight should classify shortcut");
    consumed_channel_preflight.validate().unwrap();
    consumed_channel_preflight
        .validate_against_sources(&channel_left, &nonrectilinear_channel_consumed_hole_cutters)
        .unwrap();
    assert_eq!(
        consumed_channel_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceMultiDifference
    );

    let nonrectilinear_channel_with_holes = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 17, 0, 4, 17, 0, 4, 19, 0, 2, 19, 0, //
            15, 4, 0, 17, 4, 0, 17, 6, 0, 15, 6, 0, //
            8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
            -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0,
            -2, 12, 0, 4, 11, 0, 5, 15, 0, -2, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15,
            16, 17, 18, 16, 18, 19,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectilinear channel with retained holes fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(
            &channel_left,
            &nonrectilinear_channel_with_holes,
        )
        .is_none()
    );
    let channel_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &channel_left,
        &nonrectilinear_channel_with_holes,
    )
    .expect("nonrectilinear side-cutter split should retain holes per emitted loop");
    channel_holed.validate().unwrap();
    channel_holed
        .validate_against_sources(&channel_left, &nonrectilinear_channel_with_holes)
        .unwrap();
    assert_eq!(channel_holed.components.len(), 2);
    assert_eq!(
        channel_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        2
    );
    let mut stale_channel_holed = channel_holed.clone();
    stale_channel_holed.components[0].holes.clear();
    assert!(
        stale_channel_holed
            .validate_against_sources(&channel_left, &nonrectilinear_channel_with_holes)
            .is_err()
    );
    let channel_holed_preflight = preflight_boolean_exact(
        &channel_left,
        &nonrectilinear_channel_with_holes,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectilinear channel with holes preflight should classify shortcut");
    channel_holed_preflight.validate().unwrap();
    assert_eq!(
        channel_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let point_branch_side_cutters_with_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 1, 0, 4, 1, 0, 4, 3, 0, 2, 3, 0, //
            -2, 4, 0, 8, 4, 0, 10, 10, 0, -2, 10, 0, //
            10, 10, 0, 22, 10, 0, 22, 16, 0, 14, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-branch component-holed side-cutter fixture must import");
    assert!(
        arrange_coplanar_surface_multi_difference(&channel_left, &point_branch_side_cutters_with_hole)
            .is_none()
    );
    let point_branch_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &channel_left,
        &point_branch_side_cutters_with_hole,
    )
    .expect("point-branch side-cutter split should retain unrelated strict holes");
    point_branch_holed.validate().unwrap();
    point_branch_holed
        .validate_against_sources(&channel_left, &point_branch_side_cutters_with_hole)
        .unwrap();
    assert!(
        point_branch_holed.components.len() >= 2
            && point_branch_holed
                .components
                .iter()
                .map(|component| component.holes.len())
                .sum::<usize>()
                == 1
    );
    let point_branch_holed_preflight = preflight_boolean_exact(
        &channel_left,
        &point_branch_side_cutters_with_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("point-branch component-holed preflight should classify shortcut");
    point_branch_holed_preflight.validate().unwrap();
    point_branch_holed_preflight
        .validate_against_sources(&channel_left, &point_branch_side_cutters_with_hole)
        .unwrap();
    assert_eq!(
        point_branch_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let nonrectilinear_channel_with_consumed_hole = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 17, 0, 4, 17, 0, 4, 19, 0, 2, 19, 0, //
            15, 4, 0, 17, 4, 0, 17, 6, 0, 15, 6, 0, //
            1, 5, 0, 2, 5, 0, 2, 6, 0, 1, 6, 0, //
            8, -2, 0, 12, -2, 0, 12, 22, 0, 8, 22, 0, //
            -2, 4, 0, 5, 4, 0, 3, 8, 0, -2, 8, 0,
            -2, 12, 0, 4, 11, 0, 5, 15, 0, -2, 16, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11, //
            12, 13, 14, 12, 14, 15, //
            16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectilinear channel with consumed hole fixture must import");
    let consumed_channel_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &channel_left,
        &nonrectilinear_channel_with_consumed_hole,
    )
    .expect("nonrectilinear split should consume holes inside removed openings");
    consumed_channel_holed.validate().unwrap();
    consumed_channel_holed
        .validate_against_sources(&channel_left, &nonrectilinear_channel_with_consumed_hole)
        .unwrap();
    assert_eq!(consumed_channel_holed.components.len(), 2);
    assert_eq!(
        consumed_channel_holed
            .components
            .iter()
            .map(|component| component.holes.len())
            .sum::<usize>(),
        2
    );
    let consumed_channel_preflight = preflight_boolean_exact(
        &channel_left,
        &nonrectilinear_channel_with_consumed_hole,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectilinear channel with consumed hole preflight should classify shortcut");
    consumed_channel_preflight.validate().unwrap();
    consumed_channel_preflight
        .validate_against_sources(&channel_left, &nonrectilinear_channel_with_consumed_hole)
        .unwrap();
    assert_eq!(
        consumed_channel_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let holed_left = ExactMesh::from_i64_triangles_with_policy(
        &[
            0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0, //
            20, 0, 0, 22, 0, 0, 22, 2, 0, 20, 2, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed difference left fixture must import");
    let holed_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            6, 6, 0, 8, 6, 0, 8, 8, 0, 6, 8, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed difference right fixture must import");
    let component_holed =
        arrange_coplanar_convex_surface_component_holed_difference(&holed_left, &holed_right)
            .expect("component-holed difference should materialize retained holes and components");
    component_holed.validate().unwrap();
    component_holed
        .validate_against_sources(&holed_left, &holed_right)
        .unwrap();
    assert_eq!(component_holed.components.len(), 2);
    assert!(
        component_holed
            .components
            .iter()
            .any(|component| !component.holes.is_empty())
    );
    let holed_preflight =
        preflight_boolean_exact(&holed_left, &holed_right, ExactBooleanOperation::Difference)
            .expect("component-holed difference preflight should classify shortcut");
    holed_preflight.validate().unwrap();
    assert_eq!(
        holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let holed_and_cut_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            8, -1, 0, 11, -1, 0, 11, 11, 0, 8, 11, 0,
        ],
        &[0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed cut right fixture must import");
    let component_holed_cut = arrange_coplanar_convex_surface_component_holed_difference(
        &holed_left,
        &holed_and_cut_right,
    )
    .expect("component-holed difference should assign strict holes to cut remnants");
    component_holed_cut.validate().unwrap();
    component_holed_cut
        .validate_against_sources(&holed_left, &holed_and_cut_right)
        .unwrap();
    assert_eq!(component_holed_cut.components.len(), 2);
    assert!(
        component_holed_cut
            .components
            .iter()
            .any(|component| component.holes.len() == 1)
    );
    let single_holed_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("single component-holed left fixture must import");
    let single_component_holed_cut = arrange_coplanar_convex_surface_component_holed_difference(
        &single_holed_left,
        &holed_and_cut_right,
    )
    .expect("single component-holed cut should materialize a retained holed remnant");
    single_component_holed_cut.validate().unwrap();
    single_component_holed_cut
        .validate_against_sources(&single_holed_left, &holed_and_cut_right)
        .unwrap();
    assert_eq!(single_component_holed_cut.components.len(), 1);
    assert_eq!(single_component_holed_cut.components[0].holes.len(), 1);

    let holed_two_cutters_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            4, -1, 0, 5, -1, 0, 5, 11, 0, 4, 11, 0, //
            8, -1, 0, 11, -1, 0, 11, 11, 0, 8, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed double-cutter fixture must import");
    let component_holed_multi_cut = arrange_coplanar_convex_surface_component_holed_difference(
        &holed_left,
        &holed_two_cutters_right,
    )
    .expect("component-holed full-span double-cutter should materialize");
    component_holed_multi_cut.validate().unwrap();
    component_holed_multi_cut
        .validate_against_sources(&holed_left, &holed_two_cutters_right)
        .unwrap();
    assert_eq!(component_holed_multi_cut.components.len(), 3);
    assert!(
        component_holed_multi_cut
            .components
            .iter()
            .any(|component| component.holes.len() == 1)
    );
    let holed_multi_cut_preflight = preflight_boolean_exact(
        &holed_left,
        &holed_two_cutters_right,
        ExactBooleanOperation::Difference,
    )
    .expect("component-holed double-cutter preflight should classify shortcut");
    holed_multi_cut_preflight.validate().unwrap();
    assert_eq!(
        holed_multi_cut_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let holed_corner_cutters_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
            -1, -1, 0, 3, -1, 0, -1, 3, 0, //
            7, 11, 0, 11, 7, 0, 11, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, //
            7, 8, 9,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed corner-cutter fixture must import");
    let component_holed_corner_cut = arrange_coplanar_convex_surface_component_holed_difference(
        &holed_left,
        &holed_corner_cutters_right,
    )
    .expect("component-holed corner cutters should retain a holed convex remnant");
    component_holed_corner_cut.validate().unwrap();
    component_holed_corner_cut
        .validate_against_sources(&holed_left, &holed_corner_cutters_right)
        .unwrap();
    assert_eq!(component_holed_corner_cut.components.len(), 2);
    assert!(
        component_holed_corner_cut
            .components
            .iter()
            .any(|component| component.outer.len() == 6 && component.holes.len() == 1)
    );

    let nonconvex_holed_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex component-holed left fixture must import");
    let nonconvex_holed_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            2, 2, 0, 4, 2, 0, 3, 4, 0, //
            8, 8, 0, 24, 4, 0, 24, 12, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonconvex component-holed right fixture must import");
    let nonconvex_component_holed = arrange_coplanar_convex_surface_component_holed_difference(
        &nonconvex_holed_left,
        &nonconvex_holed_right,
    )
    .expect("component-holed nonconvex outer should retain a strict hole");
    nonconvex_component_holed.validate().unwrap();
    nonconvex_component_holed
        .validate_against_sources(&nonconvex_holed_left, &nonconvex_holed_right)
        .unwrap();
    assert_eq!(nonconvex_component_holed.components.len(), 1);
    assert_eq!(nonconvex_component_holed.components[0].holes.len(), 1);
    assert!(nonconvex_component_holed.components[0].outer.len() > 4);
    let nonconvex_holed_preflight = preflight_boolean_exact(
        &nonconvex_holed_left,
        &nonconvex_holed_right,
        ExactBooleanOperation::Difference,
    )
    .expect("component-holed nonconvex outer preflight should classify shortcut");
    nonconvex_holed_preflight.validate().unwrap();
    assert_eq!(
        nonconvex_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );

    let holed_partial_height_cutters_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            1, 1, 0, 3, 1, 0, 3, 3, 0, 1, 3, 0, //
            4, 0, 0, 5, 0, 0, 5, 5, 0, 4, 5, 0, //
            8, -1, 0, 11, -1, 0, 11, 11, 0, 8, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7, //
            8, 9, 10, 8, 10, 11,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed partial-height double-cutter fixture must import");
    let partial_height_component_holed =
        arrange_coplanar_convex_surface_component_holed_difference(
            &holed_left,
            &holed_partial_height_cutters_right,
        )
        .expect("component-holed partial-height double-cutter should materialize");
    partial_height_component_holed.validate().unwrap();
    partial_height_component_holed
        .validate_against_sources(&holed_left, &holed_partial_height_cutters_right)
        .unwrap();
    assert_eq!(partial_height_component_holed.components.len(), 2);
    assert!(
        partial_height_component_holed
            .components
            .iter()
            .any(|component| component.outer.len() > 4 && component.holes.len() == 1)
    );
    let partial_height_component_holed_preflight = preflight_boolean_exact(
        &holed_left,
        &holed_partial_height_cutters_right,
        ExactBooleanOperation::Difference,
    )
    .expect("component-holed partial-height double-cutter preflight should classify shortcut");
    partial_height_component_holed_preflight
        .validate()
        .unwrap();
    assert_eq!(
        partial_height_component_holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarConvexSurfaceComponentHoledDifference
    );
    let partial_height_component_holed_result = hypermesh::exact::boolean_exact(
        &holed_left,
        &holed_partial_height_cutters_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("component-holed partial-height double-cutter boolean should materialize");
    partial_height_component_holed_result.validate().unwrap();
    assert_eq!(
        partial_height_component_holed_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::
                CoplanarConvexSurfaceComponentHoledDifference
        }
    );

    let cutter_hole_contact_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 4, 0, 6, 4, 0, 6, 6, 0, 4, 6, 0, //
            -1, 5, 0, 4, 5, 0, 4, 6, 0, -1, 6, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("cutter-hole contact fixture must import");
    let cutter_hole_contact = arrange_coplanar_surface_cutter_hole_contact_difference(
        &single_holed_left,
        &cutter_hole_contact_right,
    )
    .expect("cutter-hole contact should materialize one nonconvex loop");
    cutter_hole_contact.validate().unwrap();
    cutter_hole_contact
        .validate_cutter_hole_contact_difference_against_sources(
            &single_holed_left,
            &cutter_hole_contact_right,
        )
        .unwrap();
    assert_eq!(cutter_hole_contact.polygon.len(), 10);
    let contact_preflight = preflight_boolean_exact(
        &single_holed_left,
        &cutter_hole_contact_right,
        ExactBooleanOperation::Difference,
    )
    .expect("cutter-hole contact preflight should classify shortcut");
    contact_preflight.validate().unwrap();
    assert_eq!(
        contact_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );

    let nonrect_contact_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 0, 0, 20, 20, 0, 0, 20, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectangular cutter-hole left fixture must import");
    let nonrect_contact_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            4, 9, 0, 8, 10, 0, 6, 8, 0, //
            0, 8, 0, 8, 10, 0, 0, 12, 0,
        ],
        &[0, 2, 1, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectangular cutter-hole right fixture must import");
    let nonrect_contact = arrange_coplanar_surface_cutter_hole_contact_difference(
        &nonrect_contact_left,
        &nonrect_contact_right,
    )
    .expect("nonrectangular cutter-hole contact should materialize one nonconvex loop");
    nonrect_contact.validate().unwrap();
    nonrect_contact
        .validate_cutter_hole_contact_difference_against_sources(
            &nonrect_contact_left,
            &nonrect_contact_right,
        )
        .unwrap();
    assert_eq!(nonrect_contact.polygon.len(), 9);
    let nonrect_contact_preflight = preflight_boolean_exact(
        &nonrect_contact_left,
        &nonrect_contact_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectangular cutter-hole contact preflight should classify shortcut");
    nonrect_contact_preflight.validate().unwrap();
    assert_eq!(
        nonrect_contact_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    let straddling_contact_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            0, 9, 0, 10, 8, 0, 10, 12, 0, 0, 11, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("straddling cutter-hole fixture must import");
    let straddling_contact = arrange_coplanar_surface_cutter_hole_contact_difference(
        &nonrect_contact_left,
        &straddling_contact_right,
    )
    .expect("overlapping cutter-hole pair should materialize one nonconvex loop");
    straddling_contact.validate().unwrap();
    straddling_contact
        .validate_cutter_hole_contact_difference_against_sources(
            &nonrect_contact_left,
            &straddling_contact_right,
        )
        .unwrap();
    let straddling_contact_preflight = preflight_boolean_exact(
        &nonrect_contact_left,
        &straddling_contact_right,
        ExactBooleanOperation::Difference,
    )
    .expect("straddling cutter-hole preflight should classify shortcut");
    straddling_contact_preflight.validate().unwrap();
    assert_eq!(
        straddling_contact_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    let affine_contact_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 20, 4, 0, 18, 18, 0, -2, 14, 0],
        &[0, 1, 2, 0, 2, 3],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("affine cutter-hole left fixture must import");
    let affine_contact_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            7, 5, 0, 12, 6, 0, 8, 9, 0, //
            5, 1, 0, 10, 2, 0, 8, 7, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("affine cutter-hole right fixture must import");
    let affine_contact = arrange_coplanar_surface_cutter_hole_contact_difference(
        &affine_contact_left,
        &affine_contact_right,
    )
    .expect("affine cutter-hole side opening should materialize one nonconvex loop");
    affine_contact.validate().unwrap();
    affine_contact
        .validate_cutter_hole_contact_difference_against_sources(
            &affine_contact_left,
            &affine_contact_right,
        )
        .unwrap();
    let affine_contact_preflight = preflight_boolean_exact(
        &affine_contact_left,
        &affine_contact_right,
        ExactBooleanOperation::Difference,
    )
    .expect("affine cutter-hole preflight should classify shortcut");
    affine_contact_preflight.validate().unwrap();
    assert_eq!(
        affine_contact_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    let affine_contact_boolean = hypermesh::exact::boolean_exact(
        &affine_contact_left,
        &affine_contact_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("affine cutter-hole boolean shortcut should materialize");
    affine_contact_boolean.validate().unwrap();
    assert_eq!(
        affine_contact_boolean.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::
                CoplanarSurfaceCutterHoleContactDifference
        }
    );
    let affine_corner_contact = ExactMesh::from_i64_triangles_with_policy(
        &[
            7, 5, 0, 12, 6, 0, 8, 9, 0, //
            0, 0, 0, 10, 2, 0, 8, 7, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("affine corner-contact fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(
            &affine_contact_left,
            &affine_corner_contact,
        )
        .is_none()
    );
    let rectangular_overlap_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 8, 0, 12, 12, 0, 8, 12, 0, //
            0, 9, 0, 10, 9, 0, 10, 11, 0, 0, 11, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, 4, 6, 7,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("rectangular cutter-hole overlap fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(
            &nonrect_contact_left,
            &rectangular_overlap_right,
        )
        .is_none()
    );
    let rectangular_overlap_cells = arrange_coplanar_orthogonal_surface_difference(
        &nonrect_contact_left,
        &rectangular_overlap_right,
    )
    .expect("overlapping rectangular cutter-hole pair should replay through orthogonal cells");
    rectangular_overlap_cells.validate().unwrap();
    rectangular_overlap_cells
        .validate_against_sources(&nonrect_contact_left, &rectangular_overlap_right)
        .unwrap();
    assert_eq!(rectangular_overlap_cells.components.len(), 1);
    assert!(rectangular_overlap_cells.components[0].holes.is_empty());
    let rectangular_overlap_preflight = preflight_boolean_exact(
        &nonrect_contact_left,
        &rectangular_overlap_right,
        ExactBooleanOperation::Difference,
    )
    .expect("rectangular overlap preflight should classify orthogonal shortcut");
    rectangular_overlap_preflight.validate().unwrap();
    assert_eq!(
        rectangular_overlap_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
    );
    let rectangular_overlap_result = hypermesh::exact::boolean_exact(
        &nonrect_contact_left,
        &rectangular_overlap_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("rectangular overlap boolean should materialize through orthogonal cells");
    rectangular_overlap_result.validate().unwrap();
    assert_eq!(
        rectangular_overlap_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::
                CoplanarOrthogonalSurfaceDifference
        }
    );
    let pairwise_overlap_graph_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 4, 0, 11, 7, 0, 8, 8, 0, //
            12, 10, 0, 11, 12, 0, 14, 12, 0, //
            0, 6, 0, 10, 6, 0, 12, 10, 0, 10, 14, 0, 0, 14, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, //
            6, 7, 8, 6, 8, 9, 6, 9, 10,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("pairwise overlapping cutter-hole graph fixture must import");
    let pairwise_overlap_graph = arrange_coplanar_surface_cutter_hole_contact_difference(
        &nonrect_contact_left,
        &pairwise_overlap_graph_right,
    )
    .expect("pairwise overlapping cutter-hole graph should materialize one nonconvex loop");
    pairwise_overlap_graph.validate().unwrap();
    pairwise_overlap_graph
        .validate_cutter_hole_contact_difference_against_sources(
            &nonrect_contact_left,
            &pairwise_overlap_graph_right,
        )
        .unwrap();
    let pairwise_overlap_graph_preflight = preflight_boolean_exact(
        &nonrect_contact_left,
        &pairwise_overlap_graph_right,
        ExactBooleanOperation::Difference,
    )
    .expect("pairwise overlapping cutter-hole graph preflight should classify shortcut");
    pairwise_overlap_graph_preflight.validate().unwrap();
    assert_eq!(
        pairwise_overlap_graph_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    let triple_overlap_graph_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            8, 9, 0, 12, 9, 0, 8, 13, 0, //
            0, 8, 0, 10, 8, 0, 12, 10, 0, 10, 12, 0, 0, 12, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, //
            6, 7, 8, 6, 8, 9, 6, 9, 10,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("triple-overlap cutter-hole graph fixture must import");
    let triple_overlap_graph = arrange_coplanar_surface_cutter_hole_contact_difference(
        &nonrect_contact_left,
        &triple_overlap_graph_right,
    )
    .expect("triple-overlap cutter-hole graph should materialize one nonconvex loop");
    triple_overlap_graph.validate().unwrap();
    triple_overlap_graph
        .validate_cutter_hole_contact_difference_against_sources(
            &nonrect_contact_left,
            &triple_overlap_graph_right,
        )
        .unwrap();
    let triple_overlap_graph_preflight = preflight_boolean_exact(
        &nonrect_contact_left,
        &triple_overlap_graph_right,
        ExactBooleanOperation::Difference,
    )
    .expect("triple-overlap cutter-hole graph preflight should classify shortcut");
    triple_overlap_graph_preflight.validate().unwrap();
    assert_eq!(
        triple_overlap_graph_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    let nonrect_contact_chain_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            6, 8, 0, 8, 9, 0, 8, 11, 0, 6, 12, 0, //
            0, 9, 0, 6, 8, 0, 6, 12, 0, 0, 11, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, 3, 5, 6, //
            7, 8, 9, 7, 9, 10,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("nonrectangular cutter-hole contact chain fixture must import");
    let nonrect_contact_chain = arrange_coplanar_surface_cutter_hole_contact_difference(
        &nonrect_contact_left,
        &nonrect_contact_chain_right,
    )
    .expect("nonrectangular cutter-hole contact chain should materialize one nonconvex loop");
    nonrect_contact_chain.validate().unwrap();
    nonrect_contact_chain
        .validate_cutter_hole_contact_difference_against_sources(
            &nonrect_contact_left,
            &nonrect_contact_chain_right,
        )
        .unwrap();
    let nonrect_contact_chain_preflight = preflight_boolean_exact(
        &nonrect_contact_left,
        &nonrect_contact_chain_right,
        ExactBooleanOperation::Difference,
    )
    .expect("nonrectangular cutter-hole contact chain preflight should classify shortcut");
    nonrect_contact_chain_preflight.validate().unwrap();
    assert_eq!(
        nonrect_contact_chain_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    let point_only_chain_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 8, 0, 12, 10, 0, 8, 12, 0, //
            6, 8, 0, 8, 10, 0, 6, 12, 0, //
            0, 9, 0, 6, 8, 0, 6, 12, 0, 0, 11, 0,
        ],
        &[
            0, 1, 2, //
            3, 4, 5, //
            6, 7, 8, 6, 8, 9,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only cutter-hole contact chain fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(
            &nonrect_contact_left,
            &point_only_chain_right,
        )
        .is_none()
    );
    let incidental_point_group_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            -1, 8, 0, 8, 8, 0, 8, 12, 0, -1, 12, 0, //
            6, 9, 0, 10, 10, 0, 6, 11, 0, //
            8, 10, 0, 12, 8, 0, 12, 12, 0,
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            4, 5, 6, //
            7, 8, 9,
        ],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("incidental point cutter-hole contact fixture must import");
    let incidental_point_group = arrange_coplanar_surface_cutter_hole_contact_difference(
        &nonrect_contact_left,
        &incidental_point_group_right,
    )
    .expect("incidental point contact inside positive removed group should materialize");
    incidental_point_group.validate().unwrap();
    incidental_point_group
        .validate_cutter_hole_contact_difference_against_sources(
            &nonrect_contact_left,
            &incidental_point_group_right,
        )
        .unwrap();
    let incidental_point_preflight = preflight_boolean_exact(
        &nonrect_contact_left,
        &incidental_point_group_right,
        ExactBooleanOperation::Difference,
    )
    .expect("incidental point removed group preflight should classify shortcut");
    incidental_point_preflight.validate().unwrap();
    assert_eq!(
        incidental_point_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarSurfaceCutterHoleContactDifference
    );
    hypermesh::exact::boolean_exact(
        &nonrect_contact_left,
        &incidental_point_group_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("incidental point removed group difference should materialize")
    .validate_operation_against_sources(
        &nonrect_contact_left,
        &incidental_point_group_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    let point_only_contact_right = ExactMesh::from_i64_triangles_with_policy(
        &[
            8, 10, 0, 10, 8, 0, 10, 12, 0, //
            0, 8, 0, 8, 10, 0, 0, 12, 0,
        ],
        &[0, 1, 2, 3, 4, 5],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("point-only cutter-hole fixture must import");
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(
            &nonrect_contact_left,
            &point_only_contact_right,
        )
        .is_none()
    );

    let l_left = rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)]);
    let l_right = rect_surface_i64(&[(2, 2, 4, 4)]);
    assert!(arrange_coplanar_convex_surface_union(&l_left, &l_right).is_none());
    let l_union = arrange_coplanar_orthogonal_surface_union(&l_left, &l_right)
        .expect("orthogonal L-shaped union fixture should materialize");
    l_union.validate().unwrap();
    l_union.validate_against_sources(&l_left, &l_right).unwrap();
    assert_eq!(l_union.components.len(), 1);
    assert_eq!(l_union.components[0].holes.len(), 0);
    let union_preflight = preflight_boolean_exact(&l_left, &l_right, ExactBooleanOperation::Union)
        .expect("orthogonal union preflight should classify shortcut");
    union_preflight.validate().unwrap();
    union_preflight
        .validate_against_sources(&l_left, &l_right)
        .unwrap();
    assert_eq!(
        union_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion
    );
    hypermesh::exact::boolean_exact(
        &l_left,
        &l_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap()
    .validate_operation_against_sources(
        &l_left,
        &l_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let fan_l_left = fan_rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)]);
    assert!(arrange_coplanar_convex_surface_union(&fan_l_left, &l_right).is_none());
    let fan_l_union = arrange_coplanar_orthogonal_surface_union(&fan_l_left, &l_right)
        .expect("fan-split orthogonal cell union fixture should materialize");
    fan_l_union.validate().unwrap();
    fan_l_union
        .validate_against_sources(&fan_l_left, &l_right)
        .unwrap();
    let fan_l_preflight =
        preflight_boolean_exact(&fan_l_left, &l_right, ExactBooleanOperation::Union)
            .expect("fan-split orthogonal union preflight should classify shortcut");
    fan_l_preflight.validate().unwrap();
    fan_l_preflight
        .validate_against_sources(&fan_l_left, &l_right)
        .unwrap();
    assert_eq!(
        fan_l_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion
    );
    let partial_cell = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial orthogonal cell fixture must import");
    assert!(arrange_coplanar_orthogonal_surface_union(&partial_cell, &l_right).is_none());

    let intersection_left = rect_surface_i64(&[(0, 0, 6, 2), (0, 2, 2, 6)]);
    let intersection_right = rect_surface_i64(&[(0, 0, 6, 6)]);
    let intersection =
        arrange_coplanar_orthogonal_surface_intersection(&intersection_left, &intersection_right)
            .expect("orthogonal nonconvex intersection fixture should materialize");
    intersection.validate().unwrap();
    intersection
        .validate_against_sources(&intersection_left, &intersection_right)
        .unwrap();
    assert_eq!(intersection.components.len(), 1);
    let intersection_preflight = preflight_boolean_exact(
        &intersection_left,
        &intersection_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("orthogonal intersection preflight should classify shortcut");
    intersection_preflight.validate().unwrap();
    assert_eq!(
        intersection_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceIntersection
    );

    let holed_left = rect_surface_i64(&[(0, 0, 10, 10), (10, 0, 12, 2)]);
    let holed_right = rect_surface_i64(&[(2, 2, 4, 4)]);
    let holed_difference =
        arrange_coplanar_orthogonal_surface_difference(&holed_left, &holed_right)
            .expect("orthogonal holed difference fixture should materialize");
    holed_difference.validate().unwrap();
    holed_difference
        .validate_against_sources(&holed_left, &holed_right)
        .unwrap();
    assert_eq!(holed_difference.components.len(), 1);
    assert_eq!(holed_difference.components[0].holes.len(), 1);
    let holed_preflight =
        preflight_boolean_exact(&holed_left, &holed_right, ExactBooleanOperation::Difference)
            .expect("orthogonal holed difference preflight should classify shortcut");
    holed_preflight.validate().unwrap();
    assert_eq!(
        holed_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
    );

    let nested_left = rect_surface_i64(&[(0, 0, 10, 10)]);
    let nested_right = rect_surface_i64(&[
        (2, 2, 8, 4),
        (2, 6, 8, 8),
        (2, 4, 4, 6),
        (6, 4, 8, 6),
    ]);
    let nested_difference =
        arrange_coplanar_orthogonal_surface_difference(&nested_left, &nested_right)
            .expect("orthogonal nested island difference fixture should materialize");
    nested_difference.validate().unwrap();
    nested_difference
        .validate_against_sources(&nested_left, &nested_right)
        .unwrap();
    assert_eq!(nested_difference.components.len(), 2);
    assert!(
        nested_difference
            .components
            .iter()
            .any(|component| component.holes.len() == 1)
    );
    let nested_preflight =
        preflight_boolean_exact(&nested_left, &nested_right, ExactBooleanOperation::Difference)
            .expect("orthogonal nested island preflight should classify shortcut");
    nested_preflight.validate().unwrap();
    assert_eq!(
        nested_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
    );

    let hole_branch_left = rect_surface_i64(&[(0, 0, 5, 5)]);
    let hole_branch_right = rect_surface_i64(&[
        (1, 2, 2, 3),
        (1, 3, 2, 4),
        (2, 1, 3, 2),
        (2, 3, 3, 4),
        (3, 1, 4, 2),
        (3, 2, 4, 3),
        (3, 3, 4, 4),
    ]);
    let hole_branch_difference =
        arrange_coplanar_orthogonal_surface_difference(&hole_branch_left, &hole_branch_right)
            .expect("orthogonal hole-boundary point branch should materialize");
    hole_branch_difference.validate().unwrap();
    hole_branch_difference
        .validate_against_sources(&hole_branch_left, &hole_branch_right)
        .unwrap();
    assert_eq!(hole_branch_difference.components.len(), 2);
    assert!(
        hole_branch_difference
            .components
            .iter()
            .any(|component| component.holes.len() == 1)
    );
    assert_eq!(
        hole_branch_difference
            .components
            .iter()
            .filter(|component| {
                component
                    .outer
                    .iter()
                    .chain(component.holes.iter().flat_map(|hole| hole.iter()))
                    .any(|point| point == &point3(2, 2, 0))
            })
            .count(),
        2
    );
    let hole_branch_preflight = preflight_boolean_exact(
        &hole_branch_left,
        &hole_branch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("orthogonal hole-boundary point branch preflight should classify shortcut");
    hole_branch_preflight.validate().unwrap();
    assert_eq!(
        hole_branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
    );
    let invalid_positive_hole_contact = CoplanarOrthogonalSurfaceArrangement {
        projection: CoplanarProjection::Xy,
        operation: CoplanarOrthogonalSurfaceOperation::Difference,
        components: vec![
            CoplanarOrthogonalSurfaceComponent {
                outer: vec![
                    point3(0, 0, 0),
                    point3(6, 0, 0),
                    point3(6, 6, 0),
                    point3(0, 6, 0),
                ],
                holes: vec![vec![
                    point3(4, 4, 0),
                    point3(4, 2, 0),
                    point3(2, 2, 0),
                    point3(2, 4, 0),
                ]],
            },
            CoplanarOrthogonalSurfaceComponent {
                outer: vec![
                    point3(4, 3, 0),
                    point3(5, 3, 0),
                    point3(5, 4, 0),
                    point3(4, 4, 0),
                ],
                holes: Vec::new(),
            },
        ],
        mesh: rect_surface_i64(&[(0, 0, 6, 6), (4, 3, 5, 4)]),
    };
    assert!(invalid_positive_hole_contact.validate().is_err());

    let graph_left = rect_surface_i64(&[(0, 0, 12, 10)]);
    let graph_right = rect_surface_i64(&[(3, 3, 5, 5), (7, 3, 9, 5), (5, 4, 7, 5), (-1, 4, 3, 5)]);
    assert!(
        arrange_coplanar_surface_cutter_hole_contact_difference(&graph_left, &graph_right)
            .is_none()
    );
    let graph_difference =
        arrange_coplanar_orthogonal_surface_difference(&graph_left, &graph_right)
            .expect("orthogonal cutter/hole contact graph fixture should materialize");
    graph_difference.validate().unwrap();
    graph_difference
        .validate_against_sources(&graph_left, &graph_right)
        .unwrap();
    if let Some(mesh) = fan_surface_mesh_from_points(&graph_difference.components[0].outer) {
        let mut crossing_fan = graph_difference.clone();
        crossing_fan.mesh = mesh;
        assert!(crossing_fan.validate().is_err());
    }

    let branch_left = rect_surface_i64(&[(0, 0, 4, 4)]);
    let branch_right = rect_surface_i64(&[(0, 2, 2, 4), (2, 0, 4, 2)]);
    let branch_difference =
        arrange_coplanar_orthogonal_surface_difference(&branch_left, &branch_right)
            .expect("orthogonal point-branch cell difference should materialize");
    branch_difference.validate().unwrap();
    branch_difference
        .validate_against_sources(&branch_left, &branch_right)
        .unwrap();
    assert_eq!(branch_difference.components.len(), 2);
    assert!(branch_difference.components.iter().all(|component| {
        component.holes.is_empty()
            && component
                .outer
                .iter()
                .any(|point| point == &point3(2, 2, 0))
    }));
    let branch_preflight =
        preflight_boolean_exact(&branch_left, &branch_right, ExactBooleanOperation::Difference)
            .expect("orthogonal point-branch preflight should classify shortcut");
    branch_preflight.validate().unwrap();
    assert_eq!(
        branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceDifference
    );
    let mut stale_branch = branch_difference.clone();
    stale_branch.components[0].outer[0] = point3(99, 99, 0);
    assert!(
        stale_branch
            .validate_against_sources(&branch_left, &branch_right)
            .is_err()
    );
    let invalid_edge_touch = CoplanarOrthogonalSurfaceArrangement {
        projection: CoplanarProjection::Xy,
        operation: CoplanarOrthogonalSurfaceOperation::Difference,
        components: vec![
            CoplanarOrthogonalSurfaceComponent {
                outer: vec![
                    point3(0, 0, 0),
                    point3(2, 0, 0),
                    point3(2, 2, 0),
                    point3(0, 2, 0),
                ],
                holes: Vec::new(),
            },
            CoplanarOrthogonalSurfaceComponent {
                outer: vec![
                    point3(2, 0, 0),
                    point3(4, 0, 0),
                    point3(4, 2, 0),
                    point3(2, 2, 0),
                ],
                holes: Vec::new(),
            },
        ],
        mesh: rect_surface_i64(&[(0, 0, 2, 2), (2, 0, 4, 2)]),
    };
    assert!(invalid_edge_touch.validate().is_err());

    let overlap_source_left = rect_surface_i64(&[(0, 0, 4, 6), (2, 2, 8, 4)]);
    let overlap_source_right = rect_surface_i64(&[(8, 2, 10, 4)]);
    let overlap_union =
        arrange_coplanar_orthogonal_surface_union(&overlap_source_left, &overlap_source_right)
            .expect("same-side overlapping rectangles should replay as set occupancy");
    overlap_union.validate().unwrap();
    overlap_union
        .validate_against_sources(&overlap_source_left, &overlap_source_right)
        .unwrap();
    assert_eq!(overlap_union.components.len(), 1);
    assert!(overlap_union.components[0].holes.is_empty());
    let overlap_union_preflight = preflight_boolean_exact(
        &overlap_source_left,
        &overlap_source_right,
        ExactBooleanOperation::Union,
    )
    .expect("same-side overlap union preflight should classify shortcut");
    overlap_union_preflight.validate().unwrap();
    assert_eq!(
        overlap_union_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarOrthogonalSurfaceUnion
    );
    let overlap_union_result = hypermesh::exact::boolean_exact(
        &overlap_source_left,
        &overlap_source_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("same-side overlap union boolean should materialize");
    overlap_union_result.validate().unwrap();
    assert_eq!(
        overlap_union_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::CoplanarOrthogonalSurfaceUnion
        }
    );

    let origin = (0, 0, 0);
    let basis_u = (2, 1, 0);
    let basis_v = (-1, 2, 0);
    let affine_l_left =
        affine_rect_surface_i64(&[(0, 0, 2, 6), (2, 0, 6, 2)], origin, basis_u, basis_v);
    let affine_l_right = affine_rect_surface_i64(&[(2, 2, 4, 4)], origin, basis_u, basis_v);
    assert!(arrange_coplanar_orthogonal_surface_union(&affine_l_left, &affine_l_right).is_none());
    let affine_union = arrange_coplanar_affine_surface_union(&affine_l_left, &affine_l_right)
        .expect("affine L-shaped union fixture should materialize");
    affine_union.validate().unwrap();
    affine_union
        .validate_against_sources(&affine_l_left, &affine_l_right)
        .unwrap();
    let affine_union_preflight = preflight_boolean_exact(
        &affine_l_left,
        &affine_l_right,
        ExactBooleanOperation::Union,
    )
    .expect("affine union preflight should classify shortcut");
    affine_union_preflight.validate().unwrap();
    assert_eq!(
        affine_union_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarAffineSurfaceUnion
    );

    let affine_fan_l_left = affine_fan_rect_surface_i64(
        &[(0, 0, 2, 6), (2, 0, 6, 2)],
        origin,
        basis_u,
        basis_v,
    );
    assert!(arrange_coplanar_orthogonal_surface_union(&affine_fan_l_left, &affine_l_right).is_none());
    let affine_fan_union =
        arrange_coplanar_affine_surface_union(&affine_fan_l_left, &affine_l_right)
            .expect("affine fan-split cell union fixture should materialize");
    affine_fan_union.validate().unwrap();
    affine_fan_union
        .validate_against_sources(&affine_fan_l_left, &affine_l_right)
        .unwrap();
    let affine_fan_union_preflight = preflight_boolean_exact(
        &affine_fan_l_left,
        &affine_l_right,
        ExactBooleanOperation::Union,
    )
    .expect("affine fan-split union preflight should classify shortcut");
    affine_fan_union_preflight.validate().unwrap();
    affine_fan_union_preflight
        .validate_against_sources(&affine_fan_l_left, &affine_l_right)
        .unwrap();
    assert_eq!(
        affine_fan_union_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarAffineSurfaceUnion
    );
    let partial_affine_cell = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 2, 0, -2, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("partial affine cell fixture must import");
    assert!(
        arrange_coplanar_affine_surface_union(&partial_affine_cell, &affine_l_right).is_none()
    );

    let affine_intersection_left =
        affine_rect_surface_i64(&[(0, 0, 6, 2), (0, 2, 2, 6)], origin, basis_u, basis_v);
    let affine_intersection_right =
        affine_rect_surface_i64(&[(0, 0, 6, 6)], origin, basis_u, basis_v);
    let affine_intersection = arrange_coplanar_affine_surface_intersection(
        &affine_intersection_left,
        &affine_intersection_right,
    )
    .expect("affine nonconvex intersection fixture should materialize");
    affine_intersection.validate().unwrap();
    affine_intersection
        .validate_against_sources(&affine_intersection_left, &affine_intersection_right)
        .unwrap();

    let affine_holed_left =
        affine_rect_surface_i64(&[(0, 0, 10, 10), (10, 0, 12, 2)], origin, basis_u, basis_v);
    let affine_holed_right = affine_rect_surface_i64(&[(2, 2, 4, 4)], origin, basis_u, basis_v);
    let affine_difference =
        arrange_coplanar_affine_surface_difference(&affine_holed_left, &affine_holed_right)
            .expect("affine holed difference fixture should materialize");
    affine_difference.validate().unwrap();
    affine_difference
        .validate_against_sources(&affine_holed_left, &affine_holed_right)
        .unwrap();
    let affine_nested_left = affine_rect_surface_i64(&[(0, 0, 10, 10)], origin, basis_u, basis_v);
    let affine_nested_right = affine_rect_surface_i64(
        &[
            (2, 2, 8, 4),
            (2, 6, 8, 8),
            (2, 4, 4, 6),
            (6, 4, 8, 6),
        ],
        origin,
        basis_u,
        basis_v,
    );
    let affine_nested_difference =
        arrange_coplanar_affine_surface_difference(&affine_nested_left, &affine_nested_right)
            .expect("affine nested island difference fixture should materialize");
    affine_nested_difference.validate().unwrap();
    affine_nested_difference
        .validate_against_sources(&affine_nested_left, &affine_nested_right)
        .unwrap();
    assert_eq!(affine_nested_difference.components.len(), 2);
    let affine_nested_preflight = preflight_boolean_exact(
        &affine_nested_left,
        &affine_nested_right,
        ExactBooleanOperation::Difference,
    )
    .expect("affine nested island preflight should classify shortcut");
    affine_nested_preflight.validate().unwrap();
    assert_eq!(
        affine_nested_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarAffineSurfaceDifference
    );

    let affine_hole_branch_left =
        affine_rect_surface_i64(&[(0, 0, 5, 5)], origin, basis_u, basis_v);
    let affine_hole_branch_right = affine_rect_surface_i64(
        &[
            (1, 2, 2, 3),
            (1, 3, 2, 4),
            (2, 1, 3, 2),
            (2, 3, 3, 4),
            (3, 1, 4, 2),
            (3, 2, 4, 3),
            (3, 3, 4, 4),
        ],
        origin,
        basis_u,
        basis_v,
    );
    let affine_hole_branch_difference = arrange_coplanar_affine_surface_difference(
        &affine_hole_branch_left,
        &affine_hole_branch_right,
    )
    .expect("affine hole-boundary point branch should materialize");
    affine_hole_branch_difference.validate().unwrap();
    affine_hole_branch_difference
        .validate_against_sources(&affine_hole_branch_left, &affine_hole_branch_right)
        .unwrap();
    assert_eq!(affine_hole_branch_difference.components.len(), 2);
    assert_eq!(
        affine_hole_branch_difference
            .components
            .iter()
            .filter(|component| {
                component
                    .outer
                    .iter()
                    .chain(component.holes.iter().flat_map(|hole| hole.iter()))
                    .any(|point| point == &point3(2, 6, 0))
            })
            .count(),
        2
    );
    let affine_hole_branch_preflight = preflight_boolean_exact(
        &affine_hole_branch_left,
        &affine_hole_branch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("affine hole-boundary point branch preflight should classify shortcut");
    affine_hole_branch_preflight.validate().unwrap();
    assert_eq!(
        affine_hole_branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarAffineSurfaceDifference
    );

    let affine_graph_left = affine_rect_surface_i64(&[(0, 0, 12, 10)], origin, basis_u, basis_v);
    let affine_graph_right = affine_rect_surface_i64(
        &[(3, 3, 5, 5), (7, 3, 9, 5), (5, 4, 7, 5), (-1, 4, 3, 5)],
        origin,
        basis_u,
        basis_v,
    );
    let affine_graph_difference =
        arrange_coplanar_affine_surface_difference(&affine_graph_left, &affine_graph_right)
            .expect("affine graph fixture should materialize");
    affine_graph_difference.validate().unwrap();
    affine_graph_difference
        .validate_against_sources(&affine_graph_left, &affine_graph_right)
        .unwrap();
    if let Some(mesh) = fan_surface_mesh_from_points(&affine_graph_difference.components[0].outer)
    {
        let mut crossing_fan = affine_graph_difference.clone();
        crossing_fan.mesh = mesh;
        assert!(crossing_fan.validate().is_err());
    }

    let affine_branch_left = affine_rect_surface_i64(&[(0, 0, 4, 4)], origin, basis_u, basis_v);
    let affine_branch_right =
        affine_rect_surface_i64(&[(0, 2, 2, 4), (2, 0, 4, 2)], origin, basis_u, basis_v);
    let affine_branch_difference = arrange_coplanar_affine_surface_difference(
        &affine_branch_left,
        &affine_branch_right,
    )
    .expect("affine point-branch cell difference should materialize");
    affine_branch_difference.validate().unwrap();
    affine_branch_difference
        .validate_against_sources(&affine_branch_left, &affine_branch_right)
        .unwrap();
    assert_eq!(affine_branch_difference.components.len(), 2);
    assert!(affine_branch_difference.components.iter().all(|component| {
        component.holes.is_empty()
            && component
                .outer
                .iter()
                .any(|point| point == &point3(2, 6, 0))
    }));
    let affine_branch_preflight = preflight_boolean_exact(
        &affine_branch_left,
        &affine_branch_right,
        ExactBooleanOperation::Difference,
    )
    .expect("affine point-branch preflight should classify shortcut");
    affine_branch_preflight.validate().unwrap();
    assert_eq!(
        affine_branch_preflight.support,
        ExactBooleanSupport::CertifiedCoplanarAffineSurfaceDifference
    );

    let retained_outer = vec![
        point3(0, 0, 0),
        point3(6, 0, 0),
        point3(6, 1, 0),
        point3(1, 1, 0),
        point3(1, 5, 0),
        point3(6, 5, 0),
        point3(6, 6, 0),
        point3(0, 6, 0),
    ];
    let orthogonal_fan = CoplanarOrthogonalSurfaceArrangement {
        projection: CoplanarProjection::Xy,
        operation: CoplanarOrthogonalSurfaceOperation::Union,
        components: vec![CoplanarOrthogonalSurfaceComponent {
            outer: retained_outer.clone(),
            holes: Vec::new(),
        }],
        mesh: fan_surface_mesh_from_points(&retained_outer)
            .expect("reflex fan fixture should import"),
    };
    assert!(orthogonal_fan.validate().is_err());

    let lift = |u: i64, v: i64| point3(2 * u - v, u + 2 * v, 0);
    let affine_outer = vec![
        lift(0, 0),
        lift(6, 0),
        lift(6, 1),
        lift(1, 1),
        lift(1, 5),
        lift(6, 5),
        lift(6, 6),
        lift(0, 6),
    ];
    let affine_fan = CoplanarAffineSurfaceArrangement {
        basis: CoplanarAffineSurfaceBasis {
            projection: CoplanarProjection::Xy,
            origin: point3(0, 0, 0),
            basis_u: point3(2, 1, 0),
            basis_v: point3(-1, 2, 0),
        },
        operation: CoplanarOrthogonalSurfaceOperation::Union,
        components: vec![CoplanarOrthogonalSurfaceComponent {
            outer: affine_outer.clone(),
            holes: Vec::new(),
        }],
        mesh: fan_surface_mesh_from_points(&affine_outer)
            .expect("affine reflex fan fixture should import"),
    };
    assert!(affine_fan.validate().is_err());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_face_interior_steiner_boundary() {
    let mesh = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 4, 0, 0, 0, 4, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let triangulation = FaceRegionTriangulation {
        side: hypermesh::exact::MeshSide::Left,
        face: 0,
        projection: CoplanarProjection::Xy,
        boundary: vec![
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: point3(0, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 1,
                point: point3(4, 0, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 2,
                point: point3(0, 4, 0),
            },
            FaceSplitBoundaryNode::FaceInterior {
                point: point3(1, 1, 0),
            },
        ],
        vertices: vec![point2(0, 0), point2(4, 0), point2(0, 4), point2(1, 1)],
        triangles: vec![0, 1, 3, 0, 3, 2],
    };

    triangulation.validate().unwrap();
    let assembly = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        std::slice::from_ref(&triangulation),
        ExactRegionSelection::KeepAll,
        &mesh,
        &mesh,
    )
    .unwrap();
    assembly
        .validate_source_face_incidence(&mesh, &mesh)
        .unwrap();
    assembly
        .checked_to_exact_mesh_with_sources(&mesh, &mesh, ValidationPolicy::ALLOW_BOUNDARY)
        .unwrap();

    let mut off_plane = triangulation;
    off_plane.boundary[3] = FaceSplitBoundaryNode::FaceInterior {
        point: point3(1, 1, 1),
    };
    off_plane.validate().unwrap();
    let bad = ExactBooleanAssemblyPlan::from_region_triangulations_with_sources(
        std::slice::from_ref(&off_plane),
        ExactRegionSelection::KeepAll,
        &mesh,
        &mesh,
    )
    .unwrap();
    assert!(bad.validate_source_face_incidence(&mesh, &mesh).is_err());

    let crossing_left = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let crossing_right = ExactMesh::from_i64_triangles_with_policy(
        &[0, 0, -1, 2, 0, 1, 0, 2, 1],
        &[0, 1, 2],
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .unwrap();
    let preflight =
        preflight_boolean_exact(&crossing_left, &crossing_right, ExactBooleanOperation::Union)
            .expect("open-surface crossing union preflight should classify arrangement union");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&crossing_left, &crossing_right)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedOpenSurfaceArrangementUnion
    );
    assert!(preflight.blocker.is_none());
    assert!(preflight.region_count > 0);

    let union = hypermesh::exact::boolean_exact(
        &crossing_left,
        &crossing_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("open-surface crossing union should materialize from split regions");
    union
        .validate_operation_against_sources(
            &crossing_left,
            &crossing_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::OpenSurfaceArrangement {
            operation: ExactBooleanOperation::Union,
        }
    );

    let difference = preflight_boolean_exact(
        &crossing_left,
        &crossing_right,
        ExactBooleanOperation::Difference,
    )
    .expect("open-surface crossing difference preflight should classify arrangement difference");
    difference.validate().unwrap();
    difference
        .validate_against_sources(&crossing_left, &crossing_right)
        .unwrap();
    assert_eq!(
        difference.support,
        ExactBooleanSupport::CertifiedOpenSurfaceArrangementDifference
    );
    let difference_result = hypermesh::exact::boolean_exact(
        &crossing_left,
        &crossing_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("open-surface crossing difference should retain left split regions");
    difference_result
        .validate_operation_against_sources(
            &crossing_left,
            &crossing_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::ALLOW_BOUNDARY,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference_result.kind,
        hypermesh::exact::ExactBooleanResultKind::OpenSurfaceArrangement {
            operation: ExactBooleanOperation::Difference,
        }
    );

    let intersection = preflight_boolean_exact(
        &crossing_left,
        &crossing_right,
        ExactBooleanOperation::Intersection,
    )
    .expect("open-surface crossing intersection preflight should stay outside union shortcut");
    intersection.validate().unwrap();
    assert_eq!(
        intersection.support,
        ExactBooleanSupport::RequiresCertifiedWinding
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_boundary_centroid_volumetric_representative() {
    let target = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 12],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let triangulation = FaceRegionTriangulation {
        side: hypermesh::exact::MeshSide::Left,
        face: 0,
        projection: CoplanarProjection::Xy,
        boundary: vec![
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: point3(2, 1, 1),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 1,
                point: point3(14, 1, 1),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 2,
                point: point3(1, 14, 1),
            },
        ],
        vertices: vec![point2(2, 1), point2(14, 1), point2(1, 14)],
        triangles: vec![0, 1, 2],
    };
    let centroid = hyperlimit::Point3::new(
        rational(17, 3),
        rational(16, 3),
        hypermesh::exact::ExactReal::from(1),
    );
    let centroid_report =
        hypermesh::exact::classify_point_against_closed_mesh_winding_report(&centroid, &target);
    assert_eq!(
        centroid_report.relation,
        hypermesh::exact::ClosedMeshWindingRelation::Boundary
    );
    centroid_report
        .validate_against_sources(&centroid, &target)
        .unwrap();

    let classification =
        hypermesh::exact::classify_triangulated_region_triangle_against_closed_mesh(
            &triangulation,
            [0, 1, 2],
            &target,
        )
        .unwrap();
    assert_eq!(
        classification.relation,
        hypermesh::exact::ExactVolumetricRegionRelation::Inside
    );
    assert_eq!(
        classification.representative_witness,
        hypermesh::exact::ExactTriangleInteriorWitness::new([2, 1, 1])
    );
    assert_eq!(classification.witness_attempts.len(), 2);
    classification.representative_witness.validate().unwrap();
    classification
        .validate_against_sources(&triangulation, &target)
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_exhausted_boundary_volumetric_representatives() {
    let target = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 12, 0, 0, 0, 12, 0, 0, 0, 12],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .unwrap();
    let triangulation = FaceRegionTriangulation {
        side: hypermesh::exact::MeshSide::Left,
        face: 0,
        projection: CoplanarProjection::Xy,
        boundary: vec![
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 0,
                point: point3(1, 1, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 1,
                point: point3(5, 1, 0),
            },
            FaceSplitBoundaryNode::OriginalVertex {
                vertex: 2,
                point: point3(1, 5, 0),
            },
        ],
        vertices: vec![point2(1, 1), point2(5, 1), point2(1, 5)],
        triangles: vec![0, 1, 2],
    };

    let classification =
        hypermesh::exact::classify_triangulated_region_triangle_against_closed_mesh(
            &triangulation,
            [0, 1, 2],
            &target,
        )
        .unwrap();
    assert_eq!(
        classification.relation,
        hypermesh::exact::ExactVolumetricRegionRelation::Boundary
    );
    assert_eq!(
        classification.witness_attempts.len(),
        hypermesh::exact::EXACT_TRIANGLE_INTERIOR_WITNESSES.len()
    );
    classification
        .validate_against_sources(&triangulation, &target)
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_closed_coplanar_overlap_boundary_policy() {
    let left = axis_aligned_box_i64([0, 0, -2], [2, 2, 0]);
    let right = top_subdivided_axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);

    let graph = build_intersection_graph(&left, &right)
        .expect("closed coplanar-contact graph should build");
    graph.validate().expect("graph should validate locally");
    graph
        .validate_against_meshes(&left, &right)
        .expect("graph should replay against sources");
    assert!(graph.face_pairs.iter().any(|pair| {
        pair.relation == hypermesh::exact::MeshFacePairRelation::CoplanarOverlapping
    }));

    let boundary_report = certify_boundary_touching_report(&left, &right)
        .expect("closed coplanar contact should certify boundary policy");
    boundary_report.validate().unwrap();
    boundary_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::Certified
    );
    assert!(boundary_report.blocker.coplanar_overlapping_pairs > 0);

    let planar_report =
        certify_planar_arrangement_report(&left, &right, ExactBooleanOperation::Union).unwrap();
    planar_report.validate().unwrap();
    planar_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        planar_report.status,
        hypermesh::exact::ExactPlanarArrangementStatus::BoundaryPolicyRequired
    );

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("closed coplanar contact preflight should classify boundary policy");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::RequiresBoundaryPolicy
    );
    assert!(
        hypermesh::exact::boolean_exact(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
        )
        .is_err()
    );
    let shortcut = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    shortcut.validate().unwrap();
    shortcut.validate_against_sources(&left, &right).unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_closed_vertex_touch_boundary_policy() {
    let left = ExactMesh::from_i64_triangles(
        &[0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("closed vertex-touch left fixture must import");
    let right = ExactMesh::from_i64_triangles(
        &[0, 0, 0, -2, 0, 0, 0, -2, 0, 0, 0, -2],
        &[0, 1, 2, 0, 3, 1, 1, 3, 2, 2, 3, 0],
    )
    .expect("closed vertex-touch right fixture must import");

    let graph =
        build_intersection_graph(&left, &right).expect("closed vertex-touch graph should build");
    graph.validate().expect("graph should validate locally");
    graph
        .validate_against_meshes(&left, &right)
        .expect("graph should replay against sources");
    assert!(
        graph
            .face_pairs
            .iter()
            .any(|pair| { pair.relation == hypermesh::exact::MeshFacePairRelation::Candidate })
    );

    let boundary_report = certify_boundary_touching_report(&left, &right)
        .expect("closed vertex touch should certify boundary policy");
    boundary_report.validate().unwrap();
    boundary_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::Certified
    );
    assert!(boundary_report.blocker.candidate_pairs > 0);

    let preflight = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("closed vertex-touch preflight should classify boundary policy");
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
    );
    let union = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("closed vertex-touch union should preserve separate closed shells");
    union
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        union.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
        }
    );

    let intersection_preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection)
            .expect("closed vertex-touch intersection should classify regularized shortcut");
    intersection_preflight.validate().unwrap();
    intersection_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        intersection_preflight.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
    );
    let intersection = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("closed vertex-touch intersection should regularize to empty");
    intersection
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(intersection.mesh.triangles().is_empty());

    let difference_preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
            .expect("closed vertex-touch difference should classify regularized shortcut");
    difference_preflight.validate().unwrap();
    difference_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        difference_preflight.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
    );
    let difference = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("closed vertex-touch difference should preserve left");
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(difference.mesh.vertices(), left.vertices());
    assert_eq!(difference.mesh.triangles(), left.triangles());

    let shortcut = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::PreserveSeparateShells,
    )
    .unwrap();
    shortcut.validate().unwrap();
    shortcut.validate_against_sources(&left, &right).unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .expect("axis-aligned box fixture must import")
}

#[cfg(feature = "exact-triangulation")]
fn tetrahedron_i64(a: [i64; 3], b: [i64; 3], c: [i64; 3], d: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2],
        ],
        &[0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3],
    )
    .expect("tetrahedron fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], apex[0],
            apex[1], apex[2],
        ],
        &[
            0, 1, 2, 0, 2, 3, //
            0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0,
        ],
    )
    .expect("square pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_opposite_diagonal_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], apex[0],
            apex[1], apex[2],
        ],
        &[
            0, 1, 3, 1, 2, 3, //
            0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0,
        ],
    )
    .expect("opposite-diagonal square pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_quad_fan_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], center[0],
            center[1], center[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4, //
            0, 5, 1, 1, 5, 2, 2, 5, 3, 3, 5, 0,
        ],
    )
    .expect("quad-fan square pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_square_pyramid_quad_fan_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], center[0],
            center[1], center[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 4, 1, 1, 4, 2, 2, 4, 3, 3, 4, 0, //
            0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 0, 5,
        ],
    )
    .expect("upward quad-fan square pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_square_pyramid_two_branch_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    p: [i64; 3],
    q: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], p[0], p[1],
            p[2], q[0], q[1], q[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 1, 4, 1, 5, 4, 1, 2, 5, 2, 3, 5, 3, 4, 5, 3, 0, 4, //
            0, 6, 1, 1, 6, 2, 2, 6, 3, 3, 6, 0,
        ],
    )
    .expect("downward two-branch square pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_square_pyramid_two_branch_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    p: [i64; 3],
    q: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], p[0], p[1],
            p[2], q[0], q[1], q[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 4, 1, 1, 4, 5, 1, 5, 2, 2, 5, 3, 3, 5, 4, 3, 4, 0, //
            0, 1, 6, 1, 2, 6, 2, 3, 6, 3, 0, 6,
        ],
    )
    .expect("upward two-branch square pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_l_prism_i64(points: [[i64; 2]; 6], top_z: i64) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            0,
            points[1][0],
            points[1][1],
            0,
            points[2][0],
            points[2][1],
            0,
            points[3][0],
            points[3][1],
            0,
            points[4][0],
            points[4][1],
            0,
            points[5][0],
            points[5][1],
            0,
            points[0][0],
            points[0][1],
            top_z,
            points[1][0],
            points[1][1],
            top_z,
            points[2][0],
            points[2][1],
            top_z,
            points[3][0],
            points[3][1],
            top_z,
            points[4][0],
            points[4][1],
            top_z,
            points[5][0],
            points[5][1],
            top_z,
        ],
        &[
            0, 3, 1, 1, 3, 2, 0, 5, 3, 3, 5, 4, //
            6, 7, 8, 6, 8, 9, 6, 9, 11, 9, 10, 11, //
            0, 1, 7, 0, 7, 6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, //
            3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5, 0, 6, 5, 6, 11,
        ],
    )
    .expect("upward L-prism fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_l_prism_i64(points: [[i64; 2]; 6], bottom_z: i64) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            points[0][0],
            points[0][1],
            0,
            points[1][0],
            points[1][1],
            0,
            points[2][0],
            points[2][1],
            0,
            points[3][0],
            points[3][1],
            0,
            points[4][0],
            points[4][1],
            0,
            points[5][0],
            points[5][1],
            0,
            points[0][0],
            points[0][1],
            bottom_z,
            points[1][0],
            points[1][1],
            bottom_z,
            points[2][0],
            points[2][1],
            bottom_z,
            points[3][0],
            points[3][1],
            bottom_z,
            points[4][0],
            points[4][1],
            bottom_z,
            points[5][0],
            points[5][1],
            bottom_z,
        ],
        &[
            0, 1, 2, 0, 2, 3, 0, 3, 5, 3, 4, 5, //
            6, 9, 7, 7, 9, 8, 6, 11, 9, 9, 11, 10, //
            0, 7, 1, 0, 6, 7, 1, 8, 2, 1, 7, 8, 2, 9, 3, 2, 8, 9, //
            3, 10, 4, 3, 9, 10, 4, 11, 5, 4, 10, 11, 5, 6, 0, 5, 11, 6,
        ],
    )
    .expect("downward L-prism fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_pentagonal_pyramid_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    e: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], e[0], e[1],
            e[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, //
            0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 0, 5,
        ],
    )
    .expect("upward pentagonal pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_pentagonal_pyramid_fan_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    e: [i64; 3],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], e[0], e[1],
            e[2], center[0], center[1], center[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 5, 1, 1, 5, 2, 2, 5, 3, 3, 5, 4, 4, 5, 0, //
            0, 1, 6, 1, 2, 6, 2, 3, 6, 3, 4, 6, 4, 0, 6,
        ],
    )
    .expect("upward pentagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_pentagonal_pyramid_fan_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    e: [i64; 3],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], e[0], e[1],
            e[2], center[0], center[1], center[2], apex[0], apex[1], apex[2],
        ],
        &[
            0, 1, 5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 0, 5, //
            0, 6, 1, 1, 6, 2, 2, 6, 3, 3, 6, 4, 4, 6, 0,
        ],
    )
    .expect("downward pentagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_hexagonal_pyramid_i64(points: [[i64; 3]; 6], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, //
            0, 1, 6, 1, 2, 6, 2, 3, 6, 3, 4, 6, 4, 5, 6, 5, 0, 6,
        ],
    )
    .expect("upward hexagonal pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_hexagonal_pyramid_fan_i64(
    points: [[i64; 3]; 6],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 6, 1, 1, 6, 2, 2, 6, 3, 3, 6, 4, 4, 6, 5, 5, 6, 0, //
            0, 1, 7, 1, 2, 7, 2, 3, 7, 3, 4, 7, 4, 5, 7, 5, 0, 7,
        ],
    )
    .expect("upward hexagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_hexagonal_pyramid_fan_i64(
    points: [[i64; 3]; 6],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 6, 1, 2, 6, 2, 3, 6, 3, 4, 6, 4, 5, 6, 5, 0, 6, //
            0, 7, 1, 1, 7, 2, 2, 7, 3, 3, 7, 4, 4, 7, 5, 5, 7, 0,
        ],
    )
    .expect("downward hexagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_heptagonal_pyramid_i64(points: [[i64; 3]; 7], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, 0, 6, 5, //
            0, 1, 7, 1, 2, 7, 2, 3, 7, 3, 4, 7, 4, 5, 7, 5, 6, 7, 6, 0, 7,
        ],
    )
    .expect("upward heptagonal pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_heptagonal_pyramid_fan_i64(
    points: [[i64; 3]; 7],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 7, 1, 1, 7, 2, 2, 7, 3, 3, 7, 4, 4, 7, 5, 5, 7, 6, 6, 7, 0, //
            0, 1, 8, 1, 2, 8, 2, 3, 8, 3, 4, 8, 4, 5, 8, 5, 6, 8, 6, 0, 8,
        ],
    )
    .expect("upward heptagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_heptagonal_pyramid_fan_i64(
    points: [[i64; 3]; 7],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 7, 1, 2, 7, 2, 3, 7, 3, 4, 7, 4, 5, 7, 5, 6, 7, 6, 0, 7, //
            0, 8, 1, 1, 8, 2, 2, 8, 3, 3, 8, 4, 4, 8, 5, 5, 8, 6, 6, 8, 0,
        ],
    )
    .expect("downward heptagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_octagonal_pyramid_i64(points: [[i64; 3]; 8], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, 0, 6, 5, 0, 7, 6, //
            0, 1, 8, 1, 2, 8, 2, 3, 8, 3, 4, 8, 4, 5, 8, 5, 6, 8, 6, 7, 8,
            7, 0, 8,
        ],
    )
    .expect("upward octagonal pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_octagonal_pyramid_fan_i64(
    points: [[i64; 3]; 8],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 8, 1, 1, 8, 2, 2, 8, 3, 3, 8, 4, 4, 8, 5, 5, 8, 6, 6, 8, 7,
            7, 8, 0, //
            0, 1, 9, 1, 2, 9, 2, 3, 9, 3, 4, 9, 4, 5, 9, 5, 6, 9, 6, 7, 9,
            7, 0, 9,
        ],
    )
    .expect("upward octagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_octagonal_pyramid_fan_i64(
    points: [[i64; 3]; 8],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 8, 1, 2, 8, 2, 3, 8, 3, 4, 8, 4, 5, 8, 5, 6, 8, 6, 7, 8,
            7, 0, 8, //
            0, 9, 1, 1, 9, 2, 2, 9, 3, 3, 9, 4, 4, 9, 5, 5, 9, 6, 6, 9, 7,
            7, 9, 0,
        ],
    )
    .expect("downward octagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_nonagonal_pyramid_i64(points: [[i64; 3]; 9], apex: [i64; 3]) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            points[8][0],
            points[8][1],
            points[8][2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4, 0, 6, 5, 0, 7, 6, 0, 8, 7, //
            0, 1, 9, 1, 2, 9, 2, 3, 9, 3, 4, 9, 4, 5, 9, 5, 6, 9, 6, 7, 9, 7, 8, 9, 8, 0,
            9,
        ],
    )
    .expect("upward nonagonal pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_nonagonal_pyramid_fan_i64(
    points: [[i64; 3]; 9],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            points[8][0],
            points[8][1],
            points[8][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 9, 1, 1, 9, 2, 2, 9, 3, 3, 9, 4, 4, 9, 5, 5, 9, 6, 6, 9, 7, 7, 9, 8, 8, 9,
            0, //
            0, 1, 10, 1, 2, 10, 2, 3, 10, 3, 4, 10, 4, 5, 10, 5, 6, 10, 6, 7, 10, 7, 8, 10,
            8, 0, 10,
        ],
    )
    .expect("upward nonagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_nonagonal_pyramid_fan_i64(
    points: [[i64; 3]; 9],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
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
            points[4][0],
            points[4][1],
            points[4][2],
            points[5][0],
            points[5][1],
            points[5][2],
            points[6][0],
            points[6][1],
            points[6][2],
            points[7][0],
            points[7][1],
            points[7][2],
            points[8][0],
            points[8][1],
            points[8][2],
            center[0],
            center[1],
            center[2],
            apex[0],
            apex[1],
            apex[2],
        ],
        &[
            0, 1, 9, 1, 2, 9, 2, 3, 9, 3, 4, 9, 4, 5, 9, 5, 6, 9, 6, 7, 9, 7, 8, 9, 8, 0,
            9, //
            0, 10, 1, 1, 10, 2, 2, 10, 3, 3, 10, 4, 4, 10, 5, 5, 10, 6, 6, 10, 7, 7, 10, 8,
            8, 10, 0,
        ],
    )
    .expect("downward nonagonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_polygonal_pyramid_i64(points: &[[i64; 3]], apex: [i64; 3]) -> ExactMesh {
    assert!(points.len() >= 3);
    let apex_index = points.len();
    let mut coordinates = Vec::with_capacity((points.len() + 1) * 3);
    for point in points {
        coordinates.extend_from_slice(point);
    }
    coordinates.extend_from_slice(&apex);
    let mut indices = Vec::with_capacity((points.len() - 2 + points.len()) * 3);
    for index in 1..points.len() - 1 {
        indices.extend([0, index + 1, index]);
    }
    for index in 0..points.len() {
        indices.extend([index, (index + 1) % points.len(), apex_index]);
    }
    ExactMesh::from_i64_triangles(&coordinates, &indices)
        .expect("upward polygonal pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn downward_polygonal_pyramid_fan_i64(
    points: &[[i64; 3]],
    center: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    assert!(points.len() >= 3);
    let center_index = points.len();
    let apex_index = points.len() + 1;
    let mut coordinates = Vec::with_capacity((points.len() + 2) * 3);
    for point in points {
        coordinates.extend_from_slice(point);
    }
    coordinates.extend_from_slice(&center);
    coordinates.extend_from_slice(&apex);
    let mut indices = Vec::with_capacity(points.len() * 6);
    for index in 0..points.len() {
        indices.extend([index, (index + 1) % points.len(), center_index]);
    }
    for index in 0..points.len() {
        indices.extend([index, apex_index, (index + 1) % points.len()]);
    }
    ExactMesh::from_i64_triangles(&coordinates, &indices)
        .expect("downward polygonal fan pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upward_square_pyramid_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    d: [i64; 3],
    apex: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2], apex[0],
            apex[1], apex[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, //
            0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4,
        ],
    )
    .expect("upward square pyramid fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn combine_exact_meshes(meshes: &[ExactMesh], label: &'static str) -> ExactMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for mesh in meshes {
        let offset = vertices.len();
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(mesh.triangles().iter().map(|triangle| {
            Triangle([
                triangle.0[0] + offset,
                triangle.0[1] + offset,
                triangle.0[2] + offset,
            ])
        }));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::CLOSED,
    )
    .expect("combined fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn combine_open_exact_meshes(meshes: &[ExactMesh], label: &'static str) -> ExactMesh {
    let mut vertices = Vec::new();
    let mut triangles = Vec::new();
    for mesh in meshes {
        let offset = vertices.len();
        vertices.extend(mesh.vertices().iter().cloned());
        triangles.extend(mesh.triangles().iter().map(|triangle| {
            Triangle([
                triangle.0[0] + offset,
                triangle.0[1] + offset,
                triangle.0[2] + offset,
            ])
        }));
    }
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact(label),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("combined open fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn base_fan_tetrahedron_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    center: [i64; 3],
    d: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], center[0], center[1],
            center[2], d[0], d[1], d[2],
        ],
        &[
            0, 1, 3, 1, 2, 3, 2, 0, 3, 0, 2, 4, 2, 1, 4, 1, 0, 4,
        ],
    )
    .expect("base fan tetrahedron fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn upper_base_fan_tetrahedron_i64(
    a: [i64; 3],
    b: [i64; 3],
    c: [i64; 3],
    center: [i64; 3],
    d: [i64; 3],
) -> ExactMesh {
    ExactMesh::from_i64_triangles(
        &[
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], center[0], center[1],
            center[2], d[0], d[1], d[2],
        ],
        &[
            0, 3, 1, 1, 3, 2, 2, 3, 0, 0, 1, 4, 1, 2, 4, 2, 0, 4,
        ],
    )
    .expect("upper base fan tetrahedron fixture should import")
}

#[cfg(feature = "exact-triangulation")]
fn affine_box_i64(
    min: [i64; 3],
    max: [i64; 3],
    origin: [i64; 3],
    basis_u: [i64; 3],
    basis_v: [i64; 3],
    basis_w: [i64; 3],
) -> ExactMesh {
    let corners = [
        [min[0], min[1], min[2]],
        [max[0], min[1], min[2]],
        [max[0], max[1], min[2]],
        [min[0], max[1], min[2]],
        [min[0], min[1], max[2]],
        [max[0], min[1], max[2]],
        [max[0], max[1], max[2]],
        [min[0], max[1], max[2]],
    ];
    let mut coordinates = Vec::with_capacity(24);
    for [u, v, w] in corners {
        coordinates.extend_from_slice(&[
            origin[0] + u * basis_u[0] + v * basis_v[0] + w * basis_w[0],
            origin[1] + u * basis_u[1] + v * basis_v[1] + w * basis_w[1],
            origin[2] + u * basis_u[2] + v * basis_v[2] + w * basis_w[2],
        ]);
    }
    let mut indices = vec![
        0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6,
        3, 0, 4, 3, 4, 7,
    ];
    if determinant_i128(basis_u, basis_v, basis_w) < 0 {
        for triangle in indices.chunks_exact_mut(3) {
            triangle.swap(1, 2);
        }
    }
    ExactMesh::from_i64_triangles(&coordinates, &indices).expect("affine box fixture must import")
}

#[cfg(feature = "exact-triangulation")]
fn top_subdivided_axis_aligned_box_i64(min: [i64; 3], max: [i64; 3]) -> ExactMesh {
    let mid_x = (min[0] + max[0]) / 2;
    let mid_y = (min[1] + max[1]) / 2;
    ExactMesh::from_i64_triangles(
        &[
            min[0], min[1], min[2], max[0], min[1], min[2], max[0], max[1], min[2], min[0], max[1],
            min[2], min[0], min[1], max[2], max[0], min[1], max[2], max[0], max[1], max[2], min[0],
            max[1], max[2], mid_x, mid_y, max[2],
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 8, 5, 6, 8, 6, 7, 8, 7, 4, 8, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6,
            5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .expect("top-subdivided axis-aligned box fixture must import")
}

#[cfg(feature = "exact-triangulation")]
fn determinant_i128(a: [i64; 3], b: [i64; 3], c: [i64; 3]) -> i128 {
    let a = a.map(i128::from);
    let b = b.map(i128::from);
    let c = c.map(i128::from);
    a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
        + a[2] * (b[0] * c[1] - b[1] * c[0])
}

#[cfg(feature = "exact-triangulation")]
fn exercise_full_face_adjacent_union() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = tetrahedron_i64([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, -4]);

    let union =
        hypermesh::exact::materialize_full_face_adjacent_union(&left, &right, ValidationPolicy::CLOSED)
            .expect("full-face adjacent fuzz fixture should materialize");
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(union.mesh.vertices().len(), 5);
    assert_eq!(union.mesh.triangles().len(), 6);

    let mut stale_faces = union.clone();
    stale_faces.shared_faces[0].right_face = 1;
    assert!(stale_faces.validate_against_sources(&left, &right).is_err());

    let preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedFullFaceAdjacentUnion
    );

    let result = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::FullFaceAdjacentUnion
        }
    );

    let intersection_preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection).unwrap();
    intersection_preflight.validate().unwrap();
    intersection_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        intersection_preflight.support,
        ExactBooleanSupport::CertifiedFullFaceAdjacentIntersection
    );
    let intersection = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    intersection
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::FullFaceAdjacentIntersection
        }
    );
    assert!(intersection.mesh.triangles().is_empty());

    let difference_preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference).unwrap();
    difference_preflight.validate().unwrap();
    difference_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        difference_preflight.support,
        ExactBooleanSupport::CertifiedFullFaceAdjacentDifference
    );
    let difference = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::FullFaceAdjacentDifference
        }
    );
    assert_eq!(difference.mesh.vertices(), left.vertices());
    assert_eq!(difference.mesh.triangles(), left.triangles());

    let same_orientation = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, -4]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &left,
            &same_orientation,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let same_side_overlap = tetrahedron_i64([0, 0, 0], [0, 4, 0], [4, 0, 0], [0, 0, 2]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &left,
            &same_side_overlap,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let fan_right =
        base_fan_tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [1, 1, 0], [0, 0, -4]);
    let fan_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &left,
        &fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("fan-patch adjacent fuzz fixture should materialize");
    fan_union.validate().unwrap();
    fan_union
        .validate_against_sources(&left, &fan_right)
        .unwrap();
    assert!(fan_union.shared_faces.is_empty());
    assert_eq!(fan_union.shared_patches.len(), 1);
    assert_eq!(fan_union.mesh.vertices().len(), 5);
    assert_eq!(fan_union.mesh.triangles().len(), 6);

    let fan_preflight =
        preflight_boolean_exact(&left, &fan_right, ExactBooleanOperation::Union).unwrap();
    fan_preflight.validate().unwrap();
    fan_preflight
        .validate_against_sources(&left, &fan_right)
        .unwrap();
    assert_eq!(
        fan_preflight.support,
        ExactBooleanSupport::CertifiedFullFaceAdjacentUnion
    );

    let fan_intersection_preflight =
        preflight_boolean_exact(&left, &fan_right, ExactBooleanOperation::Intersection).unwrap();
    fan_intersection_preflight.validate().unwrap();
    fan_intersection_preflight
        .validate_against_sources(&left, &fan_right)
        .unwrap();
    assert_eq!(
        fan_intersection_preflight.support,
        ExactBooleanSupport::CertifiedFullFaceAdjacentIntersection
    );
    let fan_intersection = boolean_exact_with_boundary_policy(
        &left,
        &fan_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    fan_intersection
        .validate_operation_against_sources(
            &left,
            &fan_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(fan_intersection.mesh.triangles().is_empty());

    let fan_difference_preflight =
        preflight_boolean_exact(&left, &fan_right, ExactBooleanOperation::Difference).unwrap();
    fan_difference_preflight.validate().unwrap();
    fan_difference_preflight
        .validate_against_sources(&left, &fan_right)
        .unwrap();
    assert_eq!(
        fan_difference_preflight.support,
        ExactBooleanSupport::CertifiedFullFaceAdjacentDifference
    );
    let fan_difference = boolean_exact_with_boundary_policy(
        &left,
        &fan_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    fan_difference
        .validate_operation_against_sources(
            &left,
            &fan_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(fan_difference.mesh.vertices(), left.vertices());
    assert_eq!(fan_difference.mesh.triangles(), left.triangles());

    let same_side_fan =
        base_fan_tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [1, 1, 0], [0, 0, 2]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &left,
            &same_side_fan,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let dual_fan_left =
        upper_base_fan_tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [1, 1, 0], [0, 0, 4]);
    let dual_fan_right =
        base_fan_tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [2, 1, 0], [0, 0, -4]);
    let dual_fan_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &dual_fan_left,
        &dual_fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("dual fan-patch adjacent fuzz fixture should materialize");
    dual_fan_union.validate().unwrap();
    dual_fan_union
        .validate_against_sources(&dual_fan_left, &dual_fan_right)
        .unwrap();
    assert!(dual_fan_union.shared_faces.is_empty());
    assert_eq!(dual_fan_union.shared_patches.len(), 1);
    assert_eq!(dual_fan_union.mesh.vertices().len(), 5);
    assert_eq!(dual_fan_union.mesh.triangles().len(), 6);

    let dual_fan_result = boolean_exact_with_boundary_policy(
        &dual_fan_left,
        &dual_fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    dual_fan_result
        .validate_operation_against_sources(
            &dual_fan_left,
            &dual_fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(dual_fan_result.mesh, dual_fan_union.mesh);

    let quad_left = upward_square_pyramid_i64(
        [0, 0, 0],
        [4, 0, 0],
        [4, 4, 0],
        [0, 4, 0],
        [2, 2, 4],
    );
    let quad_right = downward_square_pyramid_opposite_diagonal_i64(
        [0, 0, 0],
        [4, 0, 0],
        [4, 4, 0],
        [0, 4, 0],
        [2, 2, -4],
    );
    let quad_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &quad_left,
        &quad_right,
        ValidationPolicy::CLOSED,
    )
    .expect("opposite-diagonal square patch should materialize");
    quad_union.validate().unwrap();
    quad_union
        .validate_against_sources(&quad_left, &quad_right)
        .unwrap();
    assert!(quad_union.shared_faces.is_empty());
    assert_eq!(quad_union.shared_patches.len(), 1);
    assert_eq!(quad_union.shared_patches[0].left_faces, vec![0, 1]);
    assert_eq!(quad_union.shared_patches[0].right_faces, vec![0, 1]);

    let quad_result = boolean_exact_with_boundary_policy(
        &quad_left,
        &quad_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    quad_result
        .validate_operation_against_sources(
            &quad_left,
            &quad_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(quad_result.mesh, quad_union.mesh);

    let quad_fan_right = downward_square_pyramid_quad_fan_i64(
        [0, 0, 0],
        [4, 0, 0],
        [4, 4, 0],
        [0, 4, 0],
        [2, 2, 0],
        [2, 2, -4],
    );
    let quad_fan_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &quad_left,
        &quad_fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("quadrilateral fan patch should materialize");
    quad_fan_union.validate().unwrap();
    quad_fan_union
        .validate_against_sources(&quad_left, &quad_fan_right)
        .unwrap();
    assert!(quad_fan_union.shared_faces.is_empty());
    assert_eq!(quad_fan_union.shared_patches[0].left_faces, vec![0, 1]);
    assert_eq!(
        quad_fan_union.shared_patches[0].right_faces,
        vec![0, 1, 2, 3]
    );

    let same_side_quad_fan = upward_square_pyramid_quad_fan_i64(
        [0, 0, 0],
        [4, 0, 0],
        [4, 4, 0],
        [0, 4, 0],
        [2, 2, 0],
        [2, 2, 4],
    );
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &quad_left,
            &same_side_quad_fan,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let quad_fan_result = boolean_exact_with_boundary_policy(
        &quad_left,
        &quad_fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    quad_fan_result
        .validate_operation_against_sources(
            &quad_left,
            &quad_fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(quad_fan_result.mesh, quad_fan_union.mesh);

    let two_branch_right = downward_square_pyramid_two_branch_i64(
        [0, 0, 0],
        [6, 0, 0],
        [6, 6, 0],
        [0, 6, 0],
        [2, 3, 0],
        [4, 3, 0],
        [3, 3, -5],
    );
    let two_branch_left =
        upward_square_pyramid_i64([0, 0, 0], [6, 0, 0], [6, 6, 0], [0, 6, 0], [3, 3, 5]);
    let two_branch_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &two_branch_left,
        &two_branch_right,
        ValidationPolicy::CLOSED,
    )
    .expect("two-branch square patch should materialize");
    two_branch_union.validate().unwrap();
    two_branch_union
        .validate_against_sources(&two_branch_left, &two_branch_right)
        .unwrap();
    assert_eq!(two_branch_union.shared_patches[0].left_faces, vec![0, 1]);
    assert_eq!(
        two_branch_union.shared_patches[0].right_faces,
        vec![0, 1, 2, 3, 4, 5]
    );

    let same_side_two_branch = upward_square_pyramid_two_branch_i64(
        [0, 0, 0],
        [6, 0, 0],
        [6, 6, 0],
        [0, 6, 0],
        [2, 3, 0],
        [4, 3, 0],
        [3, 3, 5],
    );
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &two_branch_left,
            &same_side_two_branch,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let two_branch_result = boolean_exact_with_boundary_policy(
        &two_branch_left,
        &two_branch_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    two_branch_result
        .validate_operation_against_sources(
            &two_branch_left,
            &two_branch_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(two_branch_result.mesh, two_branch_union.mesh);

    let l_boundary = [[0, 0], [4, 0], [4, 2], [2, 2], [2, 4], [0, 4]];
    let l_left = upward_l_prism_i64(l_boundary, 5);
    let l_right = downward_l_prism_i64(l_boundary, -5);
    let l_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &l_left,
        &l_right,
        ValidationPolicy::CLOSED,
    )
    .expect("nonconvex L-prism patch should materialize");
    l_union.validate().unwrap();
    l_union
        .validate_against_sources(&l_left, &l_right)
        .unwrap();
    assert_eq!(l_union.shared_faces.len(), 2);
    assert_eq!(l_union.shared_patches[0].left_faces, vec![0, 1]);
    assert_eq!(l_union.shared_patches[0].right_faces, vec![0, 1]);

    let same_side_l = upward_l_prism_i64(l_boundary, 5);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &l_left,
            &same_side_l,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let l_result = boolean_exact_with_boundary_policy(
        &l_left,
        &l_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    l_result
        .validate_operation_against_sources(
            &l_left,
            &l_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(l_result.mesh, l_union.mesh);

    let pentagon_left = upward_pentagonal_pyramid_i64(
        [0, 0, 0],
        [4, 0, 0],
        [5, 3, 0],
        [2, 5, 0],
        [-1, 3, 0],
        [2, 2, 4],
    );
    let pentagon_fan_right = downward_pentagonal_pyramid_fan_i64(
        [0, 0, 0],
        [4, 0, 0],
        [5, 3, 0],
        [2, 5, 0],
        [-1, 3, 0],
        [2, 2, 0],
        [2, 2, -4],
    );
    let pentagon_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &pentagon_left,
        &pentagon_fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("pentagonal fan patch should materialize");
    pentagon_union.validate().unwrap();
    pentagon_union
        .validate_against_sources(&pentagon_left, &pentagon_fan_right)
        .unwrap();
    assert_eq!(pentagon_union.shared_patches[0].left_faces, vec![0, 1, 2]);
    assert_eq!(
        pentagon_union.shared_patches[0].right_faces,
        vec![0, 1, 2, 3, 4]
    );

    let same_side_pentagon_fan = upward_pentagonal_pyramid_fan_i64(
        [0, 0, 0],
        [4, 0, 0],
        [5, 3, 0],
        [2, 5, 0],
        [-1, 3, 0],
        [2, 2, 0],
        [2, 2, 4],
    );
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &pentagon_left,
            &same_side_pentagon_fan,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let pentagon_result = boolean_exact_with_boundary_policy(
        &pentagon_left,
        &pentagon_fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    pentagon_result
        .validate_operation_against_sources(
            &pentagon_left,
            &pentagon_fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(pentagon_result.mesh, pentagon_union.mesh);

    let hexagon_boundary = [
        [0, 0, 0],
        [4, 0, 0],
        [6, 3, 0],
        [4, 6, 0],
        [0, 6, 0],
        [-2, 3, 0],
    ];
    let hexagon_left = upward_hexagonal_pyramid_i64(hexagon_boundary, [2, 3, 5]);
    let hexagon_fan_right =
        downward_hexagonal_pyramid_fan_i64(hexagon_boundary, [2, 3, 0], [2, 3, -5]);
    let hexagon_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &hexagon_left,
        &hexagon_fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("hexagonal fan patch should materialize");
    hexagon_union.validate().unwrap();
    hexagon_union
        .validate_against_sources(&hexagon_left, &hexagon_fan_right)
        .unwrap();
    assert_eq!(
        hexagon_union.shared_patches[0].left_faces,
        vec![0, 1, 2, 3]
    );
    assert_eq!(
        hexagon_union.shared_patches[0].right_faces,
        vec![0, 1, 2, 3, 4, 5]
    );

    let same_side_hexagon_fan =
        upward_hexagonal_pyramid_fan_i64(hexagon_boundary, [2, 3, 0], [2, 3, 5]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &hexagon_left,
            &same_side_hexagon_fan,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let hexagon_result = boolean_exact_with_boundary_policy(
        &hexagon_left,
        &hexagon_fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    hexagon_result
        .validate_operation_against_sources(
            &hexagon_left,
            &hexagon_fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(hexagon_result.mesh, hexagon_union.mesh);

    let heptagon_boundary = [
        [0, 0, 0],
        [4, 0, 0],
        [7, 3, 0],
        [5, 6, 0],
        [2, 8, 0],
        [-1, 6, 0],
        [-3, 3, 0],
    ];
    let heptagon_left = upward_heptagonal_pyramid_i64(heptagon_boundary, [2, 4, 6]);
    let heptagon_fan_right =
        downward_heptagonal_pyramid_fan_i64(heptagon_boundary, [2, 4, 0], [2, 4, -6]);
    let heptagon_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &heptagon_left,
        &heptagon_fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("heptagonal fan patch should materialize");
    heptagon_union.validate().unwrap();
    heptagon_union
        .validate_against_sources(&heptagon_left, &heptagon_fan_right)
        .unwrap();
    assert_eq!(
        heptagon_union.shared_patches[0].left_faces,
        vec![0, 1, 2, 3, 4]
    );
    assert_eq!(
        heptagon_union.shared_patches[0].right_faces,
        vec![0, 1, 2, 3, 4, 5, 6]
    );

    let same_side_heptagon_fan =
        upward_heptagonal_pyramid_fan_i64(heptagon_boundary, [2, 4, 0], [2, 4, 6]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &heptagon_left,
            &same_side_heptagon_fan,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let heptagon_result = boolean_exact_with_boundary_policy(
        &heptagon_left,
        &heptagon_fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    heptagon_result
        .validate_operation_against_sources(
            &heptagon_left,
            &heptagon_fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(heptagon_result.mesh, heptagon_union.mesh);

    let octagon_boundary = [
        [0, 0, 0],
        [4, 0, 0],
        [7, 2, 0],
        [8, 5, 0],
        [5, 8, 0],
        [1, 9, 0],
        [-2, 6, 0],
        [-3, 3, 0],
    ];
    let octagon_left = upward_octagonal_pyramid_i64(octagon_boundary, [2, 4, 7]);
    let octagon_fan_right =
        downward_octagonal_pyramid_fan_i64(octagon_boundary, [2, 4, 0], [2, 4, -7]);
    let octagon_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &octagon_left,
        &octagon_fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("octagonal fan patch should materialize");
    octagon_union.validate().unwrap();
    octagon_union
        .validate_against_sources(&octagon_left, &octagon_fan_right)
        .unwrap();
    assert_eq!(
        octagon_union.shared_patches[0].left_faces,
        vec![0, 1, 2, 3, 4, 5]
    );
    assert_eq!(
        octagon_union.shared_patches[0].right_faces,
        vec![0, 1, 2, 3, 4, 5, 6, 7]
    );

    let same_side_octagon_fan =
        upward_octagonal_pyramid_fan_i64(octagon_boundary, [2, 4, 0], [2, 4, 7]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &octagon_left,
            &same_side_octagon_fan,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let octagon_result = boolean_exact_with_boundary_policy(
        &octagon_left,
        &octagon_fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    octagon_result
        .validate_operation_against_sources(
            &octagon_left,
            &octagon_fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(octagon_result.mesh, octagon_union.mesh);

    exercise_nonagon_full_face_adjacent_union();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_nonagon_full_face_adjacent_union() {
    let nonagon_boundary = [
        [0, 0, 0],
        [4, 0, 0],
        [7, 2, 0],
        [9, 4, 0],
        [8, 7, 0],
        [5, 9, 0],
        [2, 10, 0],
        [-1, 8, 0],
        [-3, 4, 0],
    ];
    let nonagon_left = upward_nonagonal_pyramid_i64(nonagon_boundary, [2, 4, 8]);
    let nonagon_fan_right =
        downward_nonagonal_pyramid_fan_i64(nonagon_boundary, [2, 4, 0], [2, 4, -8]);
    let nonagon_union = hypermesh::exact::materialize_full_face_adjacent_union(
        &nonagon_left,
        &nonagon_fan_right,
        ValidationPolicy::CLOSED,
    )
    .expect("nonagonal fan patch should materialize");
    nonagon_union.validate().unwrap();
    nonagon_union
        .validate_against_sources(&nonagon_left, &nonagon_fan_right)
        .unwrap();
    assert_eq!(
        nonagon_union.shared_patches[0].left_faces,
        vec![0, 1, 2, 3, 4, 5, 6]
    );
    assert_eq!(
        nonagon_union.shared_patches[0].right_faces,
        vec![0, 1, 2, 3, 4, 5, 6, 7, 8]
    );

    let same_side_nonagon_fan =
        upward_nonagonal_pyramid_fan_i64(nonagon_boundary, [2, 4, 0], [2, 4, 8]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &nonagon_left,
            &same_side_nonagon_fan,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let nonagon_result = boolean_exact_with_boundary_policy(
        &nonagon_left,
        &nonagon_fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    nonagon_result
        .validate_operation_against_sources(
            &nonagon_left,
            &nonagon_fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(nonagon_result.mesh, nonagon_union.mesh);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_decagon_full_face_adjacent_union() {
    let boundary = [
        [0, 0, 0],
        [4, 0, 0],
        [8, 2, 0],
        [10, 5, 0],
        [9, 8, 0],
        [6, 10, 0],
        [2, 11, 0],
        [-1, 9, 0],
        [-3, 6, 0],
        [-2, 2, 0],
    ];
    let left = upward_polygonal_pyramid_i64(&boundary, [3, 5, 9]);
    let right = downward_polygonal_pyramid_fan_i64(&boundary, [3, 5, 0], [3, 5, -9]);
    let union = hypermesh::exact::materialize_full_face_adjacent_union(
        &left,
        &right,
        ValidationPolicy::CLOSED,
    )
    .expect("decagon full connected source disk should materialize");
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(union.shared_patches[0].left_faces, (0..8).collect::<Vec<_>>());
    assert_eq!(
        union.shared_patches[0].right_faces,
        (0..10).collect::<Vec<_>>()
    );

    let same_side = upward_polygonal_pyramid_i64(&boundary, [3, 5, 9]);
    assert!(
        hypermesh::exact::materialize_full_face_adjacent_union(
            &left,
            &same_side,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let result = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(result.mesh, union.mesh);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_contained_face_adjacent_union() {
    let left = tetrahedron_i64([0, 0, 0], [6, 0, 0], [0, 6, 0], [0, 0, 6]);
    let right = tetrahedron_i64([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -3]);

    let boundary_report = certify_boundary_touching_report(&left, &right)
        .expect("contained-face contact should build a boundary report");
    boundary_report.validate().unwrap();
    boundary_report
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        boundary_report.status,
        hypermesh::exact::ExactBoundaryTouchingStatus::Certified
    );

    let union = hypermesh::exact::materialize_contained_face_adjacent_union(
        &left,
        &right,
        ValidationPolicy::CLOSED,
    )
    .expect("strictly contained face should materialize as a holed adjacent union");
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert!(union.mesh.facts().mesh.closed_manifold);
    assert!(union.mesh.triangles().len() > left.triangles().len() + right.triangles().len());

    let mut stale_face = union.clone();
    stale_face.contained_face = 1;
    assert!(stale_face.validate_against_sources(&left, &right).is_err());

    let preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union).unwrap();
    preflight.validate().unwrap();
    preflight.validate_against_sources(&left, &right).unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedContainedFaceAdjacentUnion
    );

    let result = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ContainedFaceAdjacentUnion
        }
    );
    assert_eq!(result.mesh, union.mesh);

    let intersection_preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection).unwrap();
    intersection_preflight.validate().unwrap();
    intersection_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        intersection_preflight.support,
        ExactBooleanSupport::CertifiedContainedFaceAdjacentIntersection
    );
    let intersection = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    intersection
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ContainedFaceAdjacentIntersection
        }
    );
    assert!(intersection.mesh.vertices().is_empty());
    assert!(intersection.mesh.triangles().is_empty());

    let difference_preflight =
        preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference).unwrap();
    difference_preflight.validate().unwrap();
    difference_preflight
        .validate_against_sources(&left, &right)
        .unwrap();
    assert_eq!(
        difference_preflight.support,
        ExactBooleanSupport::CertifiedContainedFaceAdjacentDifference
    );

    let difference = boolean_exact_with_boundary_policy(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    difference
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ContainedFaceAdjacentDifference
        }
    );
    assert_eq!(difference.mesh.vertices(), left.vertices());
    assert_eq!(difference.mesh.triangles(), left.triangles());

    let reverse_difference = boolean_exact_with_boundary_policy(
        &right,
        &left,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    reverse_difference
        .validate_operation_against_sources(
            &right,
            &left,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        reverse_difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ContainedFaceAdjacentDifference
        }
    );
    assert_eq!(reverse_difference.mesh.vertices(), right.vertices());
    assert_eq!(reverse_difference.mesh.triangles(), right.triangles());

    let same_side_inner = tetrahedron_i64([1, 1, 0], [2, 1, 0], [1, 2, 0], [1, 1, 3]);
    assert!(
        hypermesh::exact::materialize_contained_face_adjacent_union(
            &left,
            &same_side_inner,
            ValidationPolicy::CLOSED,
        )
        .is_none()
    );

    let left_a = tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
    let left_b = tetrahedron_i64([20, 0, 0], [28, 0, 0], [20, 8, 0], [20, 0, 8]);
    let multi_left = combine_exact_meshes(
        &[left_a, left_b],
        "contained-face adjacent fuzz two-container fixture",
    );
    let right_a = tetrahedron_i64([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -3]);
    let right_b = tetrahedron_i64([21, 1, 0], [21, 2, 0], [22, 1, 0], [21, 1, -3]);
    let multi_right = combine_exact_meshes(
        &[right_a, right_b],
        "contained-face adjacent fuzz two-cap fixture",
    );
    let multi_union = hypermesh::exact::materialize_contained_face_adjacent_union(
        &multi_left,
        &multi_right,
        ValidationPolicy::CLOSED,
    )
    .expect("independent contained-face patches should materialize");
    multi_union.validate().unwrap();
    multi_union
        .validate_against_sources(&multi_left, &multi_right)
        .unwrap();
    assert_eq!(multi_union.contained_faces, vec![0, 4]);
    assert_eq!(multi_union.containing_faces, vec![0, 4]);

    let multi_result = boolean_exact_with_boundary_policy(
        &multi_left,
        &multi_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    multi_result
        .validate_operation_against_sources(
            &multi_left,
            &multi_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(multi_result.mesh, multi_union.mesh);

    let same_face_left = tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
    let same_face_right = combine_exact_meshes(
        &[
            tetrahedron_i64([1, 1, 0], [1, 2, 0], [2, 1, 0], [1, 1, -3]),
            tetrahedron_i64([2, 4, 0], [2, 5, 0], [3, 4, 0], [2, 4, -3]),
        ],
        "contained-face adjacent fuzz same-face two-hole fixture",
    );
    let same_face_union = hypermesh::exact::materialize_contained_face_adjacent_union(
        &same_face_left,
        &same_face_right,
        ValidationPolicy::CLOSED,
    )
    .expect("same containing face with two caps should materialize");
    same_face_union.validate().unwrap();
    same_face_union
        .validate_against_sources(&same_face_left, &same_face_right)
        .unwrap();
    assert_eq!(same_face_union.contained_faces, vec![0, 4]);
    assert_eq!(same_face_union.containing_faces, vec![0]);

    let same_face_result = boolean_exact_with_boundary_policy(
        &same_face_left,
        &same_face_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    same_face_result
        .validate_operation_against_sources(
            &same_face_left,
            &same_face_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(same_face_result.mesh, same_face_union.mesh);

    let component_hole_left = tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
    let component_hole_right = downward_square_pyramid_i64(
        [1, 1, 0],
        [3, 1, 0],
        [3, 3, 0],
        [1, 3, 0],
        [2, 2, -3],
    );
    let component_hole_union = hypermesh::exact::materialize_contained_face_adjacent_union(
        &component_hole_left,
        &component_hole_right,
        ValidationPolicy::CLOSED,
    )
    .expect("connected square cap should materialize as one component hole");
    component_hole_union.validate().unwrap();
    component_hole_union
        .validate_against_sources(&component_hole_left, &component_hole_right)
        .unwrap();
    assert_eq!(component_hole_union.contained_faces, vec![0, 1]);
    assert_eq!(component_hole_union.containing_faces, vec![0]);

    let component_hole_result = boolean_exact_with_boundary_policy(
        &component_hole_left,
        &component_hole_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    component_hole_result
        .validate_operation_against_sources(
            &component_hole_left,
            &component_hole_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(component_hole_result.mesh, component_hole_union.mesh);

    let multi_face_left = upward_square_pyramid_i64(
        [0, 0, 0],
        [8, 0, 0],
        [8, 8, 0],
        [0, 8, 0],
        [4, 4, 5],
    );
    let multi_face_right = downward_square_pyramid_i64(
        [3, 2, 0],
        [6, 2, 0],
        [6, 5, 0],
        [3, 5, 0],
        [4, 3, -3],
    );
    let multi_face_union = hypermesh::exact::materialize_contained_face_adjacent_union(
        &multi_face_left,
        &multi_face_right,
        ValidationPolicy::CLOSED,
    )
    .expect("multi-face containing component should materialize");
    multi_face_union.validate().unwrap();
    multi_face_union
        .validate_against_sources(&multi_face_left, &multi_face_right)
        .unwrap();
    assert_eq!(multi_face_union.contained_faces, vec![0, 1]);
    assert_eq!(multi_face_union.containing_faces, vec![0, 1]);

    let multi_face_result = boolean_exact_with_boundary_policy(
        &multi_face_left,
        &multi_face_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    multi_face_result
        .validate_operation_against_sources(
            &multi_face_left,
            &multi_face_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(multi_face_result.mesh, multi_face_union.mesh);

    let independent_multi_face_left = combine_exact_meshes(
        &[
            upward_square_pyramid_i64(
                [0, 0, 0],
                [8, 0, 0],
                [8, 8, 0],
                [0, 8, 0],
                [4, 4, 5],
            ),
            upward_square_pyramid_i64(
                [20, 0, 0],
                [28, 0, 0],
                [28, 8, 0],
                [20, 8, 0],
                [24, 4, 5],
            ),
        ],
        "contained-face adjacent fuzz independent multi-face containers",
    );
    let independent_multi_face_right = combine_exact_meshes(
        &[
            downward_square_pyramid_i64(
                [3, 2, 0],
                [6, 2, 0],
                [6, 5, 0],
                [3, 5, 0],
                [4, 3, -3],
            ),
            downward_square_pyramid_i64(
                [23, 2, 0],
                [26, 2, 0],
                [26, 5, 0],
                [23, 5, 0],
                [24, 3, -3],
            ),
        ],
        "contained-face adjacent fuzz independent multi-face caps",
    );
    let independent_multi_face_union =
        hypermesh::exact::materialize_contained_face_adjacent_union(
            &independent_multi_face_left,
            &independent_multi_face_right,
            ValidationPolicy::CLOSED,
        )
        .expect("independent multi-face containing components should materialize");
    independent_multi_face_union.validate().unwrap();
    independent_multi_face_union
        .validate_against_sources(&independent_multi_face_left, &independent_multi_face_right)
        .unwrap();
    assert_eq!(
        independent_multi_face_union.contained_faces,
        vec![0, 1, 6, 7]
    );
    assert_eq!(
        independent_multi_face_union.containing_faces,
        vec![0, 1, 6, 7]
    );

    let independent_multi_face_result = boolean_exact_with_boundary_policy(
        &independent_multi_face_left,
        &independent_multi_face_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
    independent_multi_face_result
        .validate_operation_against_sources(
            &independent_multi_face_left,
            &independent_multi_face_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        independent_multi_face_result.mesh,
        independent_multi_face_union.mesh
    );
}

#[cfg(feature = "exact-triangulation")]
fn exercise_axis_aligned_coplanar_volumetric_boxes() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([1, 0, 0], [3, 2, 2]);

    let union = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("axis-aligned box union preflight should classify shortcut");
    union.validate().unwrap();
    assert_eq!(
        union.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
    );
    let union_result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box union should materialize");
    union_result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let face_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let face_right = axis_aligned_box_i64([2, 0, 0], [4, 2, 2]);
    let face_union = preflight_boolean_exact(&face_left, &face_right, ExactBooleanOperation::Union)
        .expect("face-adjacent axis-aligned box union preflight should classify shortcut");
    face_union.validate().unwrap();
    assert_eq!(
        face_union.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
    );
    let face_union_result = hypermesh::exact::boolean_exact(
        &face_left,
        &face_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("face-adjacent axis-aligned box union should materialize");
    face_union_result
        .validate_operation_against_sources(
            &face_left,
            &face_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    let face_difference =
        preflight_boolean_exact(&face_left, &face_right, ExactBooleanOperation::Difference)
            .expect("face-adjacent axis-aligned box difference should classify shortcut");
    face_difference.validate().unwrap();
    assert_eq!(
        face_difference.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
    );
    assert!(intersect_closed_convex_solids(&face_left, &face_right).is_none());
    let face_difference_result = hypermesh::exact::boolean_exact(
        &face_left,
        &face_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("face-adjacent axis-aligned box difference should regularize to left box");
    face_difference_result
        .validate_operation_against_sources(
            &face_left,
            &face_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(face_difference_result.mesh.vertices(), face_left.vertices());
    assert_eq!(
        face_difference_result.mesh.triangles(),
        face_left.triangles()
    );
    let face_intersection =
        preflight_boolean_exact(&face_left, &face_right, ExactBooleanOperation::Intersection)
            .expect("face-adjacent box intersection should classify regularized shortcut");
    face_intersection.validate().unwrap();
    assert_eq!(
        face_intersection.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
    );
    let face_intersection_result = hypermesh::exact::boolean_exact(
        &face_left,
        &face_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("face-adjacent box intersection should regularize to empty");
    face_intersection_result
        .validate_operation_against_sources(
            &face_left,
            &face_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(face_intersection_result.mesh.triangles().is_empty());

    let edge_right = axis_aligned_box_i64([2, 2, 0], [4, 4, 2]);
    assert!(intersect_closed_convex_solids(&face_left, &edge_right).is_none());
    let edge_union = preflight_boolean_exact(&face_left, &edge_right, ExactBooleanOperation::Union)
        .expect("edge-adjacent axis-aligned box union should preserve separate shells");
    edge_union.validate().unwrap();
    assert_eq!(
        edge_union.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingUnion
    );
    let edge_union_result = hypermesh::exact::boolean_exact(
        &face_left,
        &edge_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("edge-adjacent axis-aligned box union should materialize separate shells");
    edge_union_result
        .validate_operation_against_sources(
            &face_left,
            &edge_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        edge_union_result.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ClosedBoundaryTouchingUnion
        }
    );
    let edge_intersection =
        preflight_boolean_exact(&face_left, &edge_right, ExactBooleanOperation::Intersection)
            .expect("edge-adjacent box intersection should classify regularized shortcut");
    edge_intersection.validate().unwrap();
    assert_eq!(
        edge_intersection.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingIntersection
    );
    let edge_intersection_result = hypermesh::exact::boolean_exact(
        &face_left,
        &edge_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("edge-adjacent box intersection should regularize to empty");
    edge_intersection_result
        .validate_operation_against_sources(
            &face_left,
            &edge_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(edge_intersection_result.mesh.triangles().is_empty());
    let edge_difference =
        preflight_boolean_exact(&face_left, &edge_right, ExactBooleanOperation::Difference)
            .expect("edge-adjacent box difference should classify regularized shortcut");
    edge_difference.validate().unwrap();
    assert_eq!(
        edge_difference.support,
        ExactBooleanSupport::CertifiedClosedBoundaryTouchingDifference
    );
    let edge_difference_result = hypermesh::exact::boolean_exact(
        &face_left,
        &edge_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("edge-adjacent box difference should preserve left");
    edge_difference_result
        .validate_operation_against_sources(
            &face_left,
            &edge_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(edge_difference_result.mesh.vertices(), face_left.vertices());
    assert_eq!(edge_difference_result.mesh.triangles(), face_left.triangles());

    let difference = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .expect("axis-aligned box difference preflight should classify shortcut");
    difference.validate().unwrap();
    assert_eq!(
        difference.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxDifference
    );
    let difference_result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box difference should materialize");
    difference_result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let split_left = axis_aligned_box_i64([0, 0, 0], [4, 2, 2]);
    let split_right = axis_aligned_box_i64([1, 0, 0], [3, 2, 2]);
    let split_difference =
        preflight_boolean_exact(&split_left, &split_right, ExactBooleanOperation::Difference)
            .expect("axis-aligned box split difference preflight should classify shortcut");
    split_difference.validate().unwrap();
    assert_eq!(
        split_difference.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxMultiDifference
    );
    let split_result = hypermesh::exact::boolean_exact(
        &split_left,
        &split_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box split difference should materialize");
    split_result
        .validate_operation_against_sources(
            &split_left,
            &split_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let nested_left = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
    let nested_right = axis_aligned_box_i64([1, 1, 1], [3, 3, 3]);
    let nested_difference = preflight_boolean_exact(
        &nested_left,
        &nested_right,
        ExactBooleanOperation::Difference,
    )
    .expect("axis-aligned box nested difference preflight should classify shortcut");
    nested_difference.validate().unwrap();
    assert_eq!(
        nested_difference.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxNestedDifference
    );
    let nested_result = hypermesh::exact::boolean_exact(
        &nested_left,
        &nested_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box nested difference should materialize");
    nested_result
        .validate_operation_against_sources(
            &nested_left,
            &nested_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let contained_outer = axis_aligned_box_i64([0, 0, 0], [4, 4, 4]);
    let contained_inner = axis_aligned_box_i64([1, 1, 1], [3, 3, 3]);
    let contained_union = preflight_boolean_exact(
        &contained_outer,
        &contained_inner,
        ExactBooleanOperation::Union,
    )
    .expect("axis-aligned contained box union preflight should classify shortcut");
    contained_union.validate().unwrap();
    assert_eq!(
        contained_union.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxUnion
    );
    let contained_union_result = hypermesh::exact::boolean_exact(
        &contained_outer,
        &contained_inner,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned contained box union should materialize");
    contained_union_result
        .validate_operation_against_sources(
            &contained_outer,
            &contained_inner,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let empty_difference = preflight_boolean_exact(
        &contained_inner,
        &contained_outer,
        ExactBooleanOperation::Difference,
    )
    .expect("axis-aligned contained-left difference preflight should classify shortcut");
    empty_difference.validate().unwrap();
    assert_eq!(
        empty_difference.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
    );
    let empty_difference_result = hypermesh::exact::boolean_exact(
        &contained_inner,
        &contained_outer,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned contained-left difference should materialize");
    empty_difference_result
        .validate_operation_against_sources(
            &contained_inner,
            &contained_outer,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let boundary_touching_inner = axis_aligned_box_i64([0, 1, 1], [2, 3, 3]);
    let boundary_touching_difference = preflight_boolean_exact(
        &boundary_touching_inner,
        &contained_outer,
        ExactBooleanOperation::Difference,
    )
    .expect("axis-aligned boundary-touching containment preflight should classify exactly");
    boundary_touching_difference.validate().unwrap();
    assert_eq!(
        boundary_touching_difference.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
    );
    let boundary_touching_result = hypermesh::exact::boolean_exact(
        &boundary_touching_inner,
        &contained_outer,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned boundary-touching contained-left difference should materialize");
    boundary_touching_result
        .validate_operation_against_sources(
            &boundary_touching_inner,
            &contained_outer,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let protruding_overlap = axis_aligned_box_i64([-1, 1, 1], [2, 3, 3]);
    let protruding_difference = preflight_boolean_exact(
        &protruding_overlap,
        &contained_outer,
        ExactBooleanOperation::Difference,
    )
    .expect("axis-aligned protruding overlap preflight should classify exactly");
    protruding_difference.validate().unwrap();
    assert_ne!(
        protruding_difference.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxEmptyDifference
    );

    let cell_left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let cell_right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
    let cell_union = preflight_boolean_exact(&cell_left, &cell_right, ExactBooleanOperation::Union)
        .expect("axis-aligned box cell union preflight should classify shortcut");
    cell_union.validate().unwrap();
    assert_eq!(
        cell_union.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxCellUnion
    );
    let cell_union_result = hypermesh::exact::boolean_exact(
        &cell_left,
        &cell_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box cell union should materialize");
    cell_union_result
        .validate_operation_against_sources(
            &cell_left,
            &cell_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let cell_intersection =
        preflight_boolean_exact(&cell_left, &cell_right, ExactBooleanOperation::Intersection)
            .expect("axis-aligned box intersection preflight should classify shortcut");
    cell_intersection.validate().unwrap();
    assert_eq!(
        cell_intersection.support,
        ExactBooleanSupport::CertifiedAxisAlignedBoxIntersection
    );
    let cell_intersection_result = hypermesh::exact::boolean_exact(
        &cell_left,
        &cell_right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box intersection should materialize");
    cell_intersection_result
        .validate_operation_against_sources(
            &cell_left,
            &cell_right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let cell_difference =
        preflight_boolean_exact(&cell_left, &cell_right, ExactBooleanOperation::Difference)
            .expect("axis-aligned box cell difference preflight should classify shortcut");
    cell_difference.validate().unwrap();
    assert_eq!(
        cell_difference.support,
        hypermesh::exact::ExactBooleanSupport::CertifiedAxisAlignedBoxCellDifference
    );
    let cell_result = hypermesh::exact::boolean_exact(
        &cell_left,
        &cell_right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box cell difference should materialize");
    cell_result
        .validate_operation_against_sources(
            &cell_left,
            &cell_right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_axis_aligned_orthogonal_solid_cell_complexes() {
    let left = axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
    let fan_left = top_subdivided_axis_aligned_box_i64([0, 0, 0], [2, 2, 2]);
    let fan_right = axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);
    let complex = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned box cell union should materialize")
    .mesh;
    let cutter = axis_aligned_box_i64([2, 0, 0], [3, 2, 2]);

    let union = preflight_boolean_exact(&complex, &cutter, ExactBooleanOperation::Union)
        .expect("orthogonal solid cell union preflight should classify shortcut");
    union.validate().unwrap();
    assert_eq!(
        union.support,
        ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellUnion
    );
    let union_result = hypermesh::exact::boolean_exact(
        &complex,
        &cutter,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("orthogonal solid cell union should materialize");
    union_result
        .validate_operation_against_sources(
            &complex,
            &cutter,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let intersection =
        preflight_boolean_exact(&complex, &cutter, ExactBooleanOperation::Intersection)
            .expect("orthogonal solid cell intersection preflight should classify shortcut");
    intersection.validate().unwrap();
    assert_eq!(
        intersection.support,
        ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection
    );
    let intersection_result = hypermesh::exact::boolean_exact(
        &complex,
        &cutter,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("orthogonal solid cell intersection should materialize");
    intersection_result
        .validate_operation_against_sources(
            &complex,
            &cutter,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let difference = preflight_boolean_exact(&complex, &cutter, ExactBooleanOperation::Difference)
        .expect("orthogonal solid cell difference preflight should classify shortcut");
    difference.validate().unwrap();
    assert_eq!(
        difference.support,
        ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellDifference
    );
    let difference_result = hypermesh::exact::boolean_exact(
        &complex,
        &cutter,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("orthogonal solid cell difference should materialize");
    difference_result
        .validate_operation_against_sources(
            &complex,
            &cutter,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let fan_union = preflight_boolean_exact(&fan_left, &fan_right, ExactBooleanOperation::Union)
        .expect("face-cell triangulated orthogonal solid union should classify shortcut");
    fan_union.validate().unwrap();
    assert_eq!(
        fan_union.support,
        ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellUnion
    );
    let fan_union_result = hypermesh::exact::boolean_exact(
        &fan_left,
        &fan_right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("face-cell triangulated orthogonal solid union should materialize");
    fan_union_result
        .validate_operation_against_sources(
            &fan_left,
            &fan_right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let fan_intersection =
        preflight_boolean_exact(&fan_left, &fan_right, ExactBooleanOperation::Intersection)
            .expect("face-cell triangulated orthogonal solid intersection should classify shortcut");
    fan_intersection.validate().unwrap();
    assert_eq!(
        fan_intersection.support,
        ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection
    );

    let fan_difference =
        preflight_boolean_exact(&fan_left, &fan_right, ExactBooleanOperation::Difference)
            .expect("face-cell triangulated orthogonal solid difference should classify shortcut");
    fan_difference.validate().unwrap();
    assert_eq!(
        fan_difference.support,
        ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellDifference
    );

    let outer = axis_aligned_box_i64([0, 0, 0], [8, 8, 8]);
    let cavity = axis_aligned_box_i64([2, 2, 2], [6, 6, 6]);
    let hollow = hypermesh::exact::boolean_exact(
        &outer,
        &cavity,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("axis-aligned hollow shell should materialize")
    .mesh;
    let floating = axis_aligned_box_i64([3, 3, 3], [5, 5, 5]);
    let empty_intersection =
        preflight_boolean_exact(&hollow, &floating, ExactBooleanOperation::Intersection)
            .expect("empty cavity cell intersection should classify shortcut");
    empty_intersection.validate().unwrap();
    assert_eq!(
        empty_intersection.support,
        ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection
    );
    let empty_intersection_result = hypermesh::exact::boolean_exact(
        &hollow,
        &floating,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("empty cavity cell intersection should materialize");
    empty_intersection_result
        .validate_operation_against_sources(
            &hollow,
            &floating,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(empty_intersection_result.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_affine_coplanar_volumetric_boxes() {
    let origin = [0, 0, 0];
    let basis_u = [2, 1, 0];
    let basis_v = [-1, 2, 0];
    let basis_w = [0, 1, 2];
    let left = affine_box_i64([0, 0, 0], [2, 2, 2], origin, basis_u, basis_v, basis_w);
    let right = affine_box_i64([1, 1, 0], [3, 3, 2], origin, basis_u, basis_v, basis_w);

    let arrangement =
        hypermesh::exact::materialize_affine_box_union(&left, &right, ValidationPolicy::CLOSED)
            .expect("affine box union fixture should not error")
            .expect("affine box union should materialize");
    arrangement.validate().unwrap();
    arrangement.validate_against_sources(&left, &right).unwrap();

    let union = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Union)
        .expect("affine box union preflight should classify shortcut");
    union.validate().unwrap();
    union.validate_against_sources(&left, &right).unwrap();
    assert_eq!(union.support, ExactBooleanSupport::CertifiedAffineBoxUnion);
    let union_result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("affine box union should materialize");
    union_result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Union,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let intersection = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Intersection)
        .expect("affine box intersection preflight should classify shortcut");
    intersection.validate().unwrap();
    assert_eq!(
        intersection.support,
        ExactBooleanSupport::CertifiedAffineBoxIntersection
    );
    let intersection_result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("affine box intersection should materialize");
    intersection_result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let difference = preflight_boolean_exact(&left, &right, ExactBooleanOperation::Difference)
        .expect("affine box difference preflight should classify shortcut");
    difference.validate().unwrap();
    assert_eq!(
        difference.support,
        ExactBooleanSupport::CertifiedAffineBoxDifference
    );
    let difference_result = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("affine box difference should materialize");
    difference_result
        .validate_operation_against_sources(
            &left,
            &right,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();

    let point_touch = affine_box_i64([2, 2, 2], [3, 3, 3], origin, basis_u, basis_v, basis_w);
    assert!(
        hypermesh::exact::materialize_affine_box_union(
            &left,
            &point_touch,
            ValidationPolicy::CLOSED
        )
        .expect("affine point contact should not error")
        .is_none()
    );

    let basis_u = [-1, 2, 0];
    let basis_v = [2, 1, 0];
    let basis_w = [0, 1, 2];
    assert!(determinant_i128(basis_u, basis_v, basis_w) < 0);
    let left = affine_box_i64([0, 0, 0], [2, 2, 2], origin, basis_u, basis_v, basis_w);
    let right = affine_box_i64([1, 1, 0], [3, 3, 2], origin, basis_u, basis_v, basis_w);
    let arrangement =
        hypermesh::exact::materialize_affine_box_union(&left, &right, ValidationPolicy::CLOSED)
            .expect("left-handed affine box union fixture should not error")
            .expect("left-handed affine box union should materialize");
    arrangement.validate().unwrap();
    arrangement.validate_against_sources(&left, &right).unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_affine_orthogonal_solid_cell_complexes() {
    let origin = [0, 0, 0];
    let basis_u = [2, 1, 0];
    let basis_v = [-1, 2, 0];
    let basis_w = [0, 1, 2];
    let left = affine_box_i64([0, 0, 0], [2, 2, 2], origin, basis_u, basis_v, basis_w);
    let right = affine_box_i64([1, 1, 0], [3, 3, 2], origin, basis_u, basis_v, basis_w);
    let complex = hypermesh::exact::boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("affine box cell union should materialize")
    .mesh;
    let cutter = affine_box_i64([2, 0, 0], [3, 2, 2], origin, basis_u, basis_v, basis_w);

    let arrangement = hypermesh::exact::materialize_affine_orthogonal_solid_union(
        &complex,
        &cutter,
        ValidationPolicy::CLOSED,
    )
    .expect("affine orthogonal solid union fixture should not error")
    .expect("affine orthogonal solid union should materialize");
    arrangement.validate().unwrap();
    arrangement
        .validate_against_sources(&complex, &cutter)
        .unwrap();

    let union = preflight_boolean_exact(&complex, &cutter, ExactBooleanOperation::Union)
        .expect("affine orthogonal solid union preflight should classify shortcut");
    union.validate().unwrap();
    assert_eq!(
        union.support,
        ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellUnion
    );
    hypermesh::exact::boolean_exact(
        &complex,
        &cutter,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("affine orthogonal solid union should materialize")
    .validate_operation_against_sources(
        &complex,
        &cutter,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let intersection =
        preflight_boolean_exact(&complex, &cutter, ExactBooleanOperation::Intersection)
            .expect("affine orthogonal solid intersection preflight should classify shortcut");
    intersection.validate().unwrap();
    assert_eq!(
        intersection.support,
        ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellIntersection
    );
    hypermesh::exact::boolean_exact(
        &complex,
        &cutter,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("affine orthogonal solid intersection should materialize")
    .validate_operation_against_sources(
        &complex,
        &cutter,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let difference = preflight_boolean_exact(&complex, &cutter, ExactBooleanOperation::Difference)
        .expect("affine orthogonal solid difference preflight should classify shortcut");
    difference.validate().unwrap();
    assert_eq!(
        difference.support,
        ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellDifference
    );
    hypermesh::exact::boolean_exact(
        &complex,
        &cutter,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("affine orthogonal solid difference should materialize")
    .validate_operation_against_sources(
        &complex,
        &cutter,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let outer = affine_box_i64([0, 0, 0], [8, 8, 8], origin, basis_u, basis_v, basis_w);
    let cavity = affine_box_i64([2, 2, 2], [6, 6, 6], origin, basis_u, basis_v, basis_w);
    let hollow = hypermesh::exact::boolean_exact(
        &outer,
        &cavity,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("affine hollow shell should materialize")
    .mesh;
    let floating = affine_box_i64([3, 3, 3], [5, 5, 5], origin, basis_u, basis_v, basis_w);
    let empty_intersection =
        preflight_boolean_exact(&hollow, &floating, ExactBooleanOperation::Intersection)
            .expect("affine empty cavity cell intersection should classify shortcut");
    empty_intersection.validate().unwrap();
    assert_eq!(
        empty_intersection.support,
        ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellIntersection
    );
    let empty_intersection_result = hypermesh::exact::boolean_exact(
        &hollow,
        &floating,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("affine empty cavity cell intersection should materialize");
    empty_intersection_result
        .validate_operation_against_sources(
            &hollow,
            &floating,
            ExactBooleanOperation::Intersection,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert!(empty_intersection_result.mesh.triangles().is_empty());
}

#[cfg(feature = "exact-triangulation")]
fn exercise_affine_orthogonal_solid_cell_complex_frame_discovery() {
    let origin = [0, 0, 0];
    let basis_u = [2, 1, 0];
    let basis_v = [-1, 2, 0];
    let basis_w = [0, 1, 2];
    let left_a = affine_box_i64([0, 0, 0], [2, 2, 2], origin, basis_u, basis_v, basis_w);
    let left_b = affine_box_i64([1, 1, 0], [3, 3, 2], origin, basis_u, basis_v, basis_w);
    let right_a = affine_box_i64([2, 0, 0], [4, 2, 2], origin, basis_u, basis_v, basis_w);
    let right_b = affine_box_i64([3, 1, 0], [5, 3, 2], origin, basis_u, basis_v, basis_w);
    let left_complex = hypermesh::exact::boolean_exact(
        &left_a,
        &left_b,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("left affine complex should materialize")
    .mesh;
    let right_complex = hypermesh::exact::boolean_exact(
        &right_a,
        &right_b,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("right affine complex should materialize")
    .mesh;

    let union =
        preflight_boolean_exact(&left_complex, &right_complex, ExactBooleanOperation::Union)
            .expect("affine complex frame-discovery union preflight should classify shortcut");
    union.validate().unwrap();
    union
        .validate_against_sources(&left_complex, &right_complex)
        .unwrap();
    assert_eq!(
        union.support,
        ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellUnion
    );
    hypermesh::exact::boolean_exact(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
    )
    .expect("affine complex frame-discovery union should materialize")
    .validate_operation_against_sources(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Union,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let intersection = preflight_boolean_exact(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Intersection,
    )
    .expect("affine complex frame-discovery intersection preflight should classify shortcut");
    intersection.validate().unwrap();
    intersection
        .validate_against_sources(&left_complex, &right_complex)
        .unwrap();
    assert_eq!(
        intersection.support,
        ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellIntersection
    );
    hypermesh::exact::boolean_exact(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
    )
    .expect("affine complex frame-discovery intersection should materialize")
    .validate_operation_against_sources(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Intersection,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();

    let difference = preflight_boolean_exact(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Difference,
    )
    .expect("affine complex frame-discovery difference preflight should classify shortcut");
    difference.validate().unwrap();
    difference
        .validate_against_sources(&left_complex, &right_complex)
        .unwrap();
    assert_eq!(
        difference.support,
        ExactBooleanSupport::CertifiedAffineOrthogonalSolidCellDifference
    );
    hypermesh::exact::boolean_exact(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("affine complex frame-discovery difference should materialize")
    .validate_operation_against_sources(
        &left_complex,
        &right_complex,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
        ExactBoundaryBooleanPolicy::Reject,
    )
    .unwrap();
}

#[cfg(feature = "exact-triangulation")]
fn exercise_mixed_coplanar_volumetric_materialization() {
    let left = ExactMesh::from_i64_triangles(
        &[
            0, 0, 0, 2, 0, 0, 2, 2, 0, 0, 2, 0, 0, 0, 2, 2, 0, 2, 2, 2, 2, 0, 2, 2,
        ],
        &[
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7,
            6, 3, 0, 4, 3, 4, 7,
        ],
    )
    .expect("mixed coplanar-volumetric left fixture must import");
    let right = top_subdivided_axis_aligned_box_i64([1, 1, 0], [3, 3, 2]);

    let graph = build_intersection_graph(&left, &right)
        .expect("mixed coplanar-volumetric graph should build");
    graph.validate().expect("graph should validate locally");
    graph
        .validate_against_meshes(&left, &right)
        .expect("graph should replay against sources");
    assert!(graph.face_pairs.iter().any(|pair| {
        pair.relation == hypermesh::exact::MeshFacePairRelation::CoplanarOverlapping
    }));

    for (operation, support) in [
        (
            ExactBooleanOperation::Union,
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellUnion,
        ),
        (
            ExactBooleanOperation::Intersection,
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellIntersection,
        ),
        (
            ExactBooleanOperation::Difference,
            ExactBooleanSupport::CertifiedAxisAlignedOrthogonalSolidCellDifference,
        ),
    ] {
        let preflight = preflight_boolean_exact(&left, &right, operation)
            .expect("mixed coplanar-volumetric preflight should classify materialization");
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();
        assert_eq!(preflight.support, support);

        let winding = certify_winding_readiness_report(&left, &right, operation)
            .expect("mixed coplanar-volumetric winding report should classify readiness");
        winding.validate().unwrap();
        assert_eq!(
            winding.status,
            hypermesh::exact::ExactWindingReadinessStatus::Ready
        );

        let result =
            hypermesh::exact::boolean_exact(&left, &right, operation, ValidationPolicy::CLOSED)
                .expect("mixed coplanar-volumetric boolean should materialize");
        result.validate().unwrap();
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
    }
}

#[cfg(feature = "exact-triangulation")]
fn exercise_non_rectilinear_coplanar_volumetric_materialization() {
    let left = tetrahedron_i64([0, 0, 0], [4, 0, 0], [0, 4, 0], [0, 0, 4]);
    let right = tetrahedron_i64([1, 1, 0], [5, 1, 0], [1, 5, 0], [1, 1, 4]);

    let graph = build_intersection_graph(&left, &right)
        .expect("non-rectilinear coplanar-volumetric graph should build");
    graph.validate().expect("graph should validate locally");
    graph
        .validate_against_meshes(&left, &right)
        .expect("graph should replay against sources");
    assert!(graph.face_pairs.iter().any(|pair| {
        pair.relation == hypermesh::exact::MeshFacePairRelation::CoplanarOverlapping
    }));

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = preflight_boolean_exact(&left, &right, operation).expect(
            "non-rectilinear coplanar-volumetric preflight should classify materialization",
        );
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedWindingMaterialized
        );
        let evidence = preflight
            .coplanar_volumetric_evidence
            .as_ref()
            .expect("coplanar-volumetric preflight should retain source evidence");
        evidence.validate().unwrap();
        assert!(evidence.obstacle.requires_coplanar_volumetric_cells());

        let result =
            hypermesh::exact::boolean_exact(&left, &right, operation, ValidationPolicy::CLOSED)
                .expect("non-rectilinear coplanar-volumetric boolean should materialize");
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
        if operation == ExactBooleanOperation::Union {
            let mut mislabeled = result.clone();
            mislabeled.kind = hypermesh::exact::ExactBooleanResultKind::WindingMaterialized {
                operation: ExactBooleanOperation::Difference,
            };
            assert!(matches!(
                mislabeled.validate(),
                Err(
                    hypermesh::exact::ExactReportValidationError::
                        WindingMaterializedAssemblyViolatesOperation
                )
            ));
            let mut wrong_orientation = result.clone();
            if let Some(triangle) = wrong_orientation.assembly.triangles.first_mut() {
                triangle.orientation =
                    hypermesh::exact::ExactOutputTriangleOrientation::ReverseSource;
                assert!(matches!(
                    wrong_orientation.validate(),
                    Err(
                        hypermesh::exact::ExactReportValidationError::
                            WindingMaterializedAssemblyViolatesOperation
                    )
                ));
            }
        }
    }

    let outer = tetrahedron_i64([0, 0, 0], [8, 0, 0], [0, 8, 0], [0, 0, 8]);
    let inner = tetrahedron_i64([1, 1, 0], [3, 1, 0], [1, 3, 0], [1, 1, 2]);
    let evidence = certify_coplanar_volumetric_cell_evidence(&outer, &inner)
        .expect("boundary-contained convex solids should expose coplanar-volumetric evidence");
    evidence.validate().unwrap();
    evidence.validate_against_sources(&outer, &inner).unwrap();
    assert!(evidence.obstacle.requires_coplanar_volumetric_cells());
    assert_eq!(
        classify_mesh_vertices_against_convex_solid(&inner, &outer),
        hypermesh::exact::ConvexSolidMeshRelation::BoundaryOrMixed
    );

    for (operation, support, shortcut) in [
        (
            ExactBooleanOperation::Union,
            ExactBooleanSupport::CertifiedConvexContainment,
            true,
        ),
        (
            ExactBooleanOperation::Intersection,
            ExactBooleanSupport::CertifiedConvexContainment,
            true,
        ),
        (
            ExactBooleanOperation::Difference,
            ExactBooleanSupport::CertifiedConvexContainment,
            true,
        ),
    ] {
        let preflight = preflight_boolean_exact(&outer, &inner, operation)
            .expect("boundary-contained convex preflight should classify exactly");
        preflight.validate().unwrap();
        preflight.validate_against_sources(&outer, &inner).unwrap();
        assert_eq!(preflight.support, support);
        assert_eq!(preflight.coplanar_volumetric_evidence.is_none(), shortcut);

        let result =
            hypermesh::exact::boolean_exact(&outer, &inner, operation, ValidationPolicy::CLOSED)
                .expect("boundary-contained convex boolean should materialize");
        result.validate().unwrap();
        result
            .validate_operation_against_sources(
                &outer,
                &inner,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert_eq!(
            result.kind,
            hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
                shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexContainment,
            }
        );
        if operation == ExactBooleanOperation::Difference {
            assert!(result.mesh.facts().mesh.closed_manifold);
            assert!(result.mesh.triangles().len() > outer.triangles().len());
        }
    }

    let reverse_difference = hypermesh::exact::boolean_exact(
        &inner,
        &outer,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("contained convex minus its container should materialize empty");
    reverse_difference
        .validate_operation_against_sources(
            &inner,
            &outer,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(
        reverse_difference.kind,
        hypermesh::exact::ExactBooleanResultKind::CertifiedShortcut {
            shortcut: hypermesh::exact::ExactBooleanShortcutKind::ConvexContainment,
        }
    );
    assert!(reverse_difference.mesh.triangles().is_empty());

    let nonconvex_container =
        upward_l_prism_i64([[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]], 8);
    let boundary_cutter = axis_aligned_box_i64([1, 3, 4], [2, 4, 8]);
    let boundary_difference = hypermesh::exact::materialize_contained_boundary_difference(
        &nonconvex_container,
        &boundary_cutter,
        ValidationPolicy::CLOSED,
    )
    .expect("nonconvex boundary-contained difference should replay");
    boundary_difference.validate().unwrap();
    boundary_difference
        .validate_against_sources(&nonconvex_container, &boundary_cutter)
        .unwrap();
    assert_eq!(boundary_difference.containing_faces.len(), 1);
    assert_eq!(boundary_difference.contained_faces.len(), 2);
    let preflight = preflight_boolean_exact(
        &nonconvex_container,
        &boundary_cutter,
        ExactBooleanOperation::Difference,
    )
    .expect("nonconvex boundary-contained preflight should classify");
    preflight.validate().unwrap();
    preflight
        .validate_against_sources(&nonconvex_container, &boundary_cutter)
        .unwrap();
    assert_eq!(
        preflight.support,
        ExactBooleanSupport::CertifiedContainedBoundaryDifference
    );
    let result = hypermesh::exact::boolean_exact(
        &nonconvex_container,
        &boundary_cutter,
        ExactBooleanOperation::Difference,
        ValidationPolicy::CLOSED,
    )
    .expect("nonconvex boundary-contained difference should materialize");
    result
        .validate_operation_against_sources(
            &nonconvex_container,
            &boundary_cutter,
            ExactBooleanOperation::Difference,
            ValidationPolicy::CLOSED,
            ExactBoundaryBooleanPolicy::Reject,
        )
        .unwrap();
    assert_eq!(result.mesh, boundary_difference.mesh);

    let fan_container = top_subdivided_axis_aligned_box_i64([0, 0, 0], [8, 8, 8]);
    let fan_removed = axis_aligned_box_i64([1, 1, 4], [7, 7, 8]);
    let fan_difference = hypermesh::exact::materialize_contained_boundary_difference(
        &fan_container,
        &fan_removed,
        ValidationPolicy::CLOSED,
    )
    .expect("component certificate should handle a cap spanning multiple source faces");
    fan_difference.validate().unwrap();
    fan_difference
        .validate_against_sources(&fan_container, &fan_removed)
        .unwrap();
    assert!(fan_difference.containing_faces.len() > 1);
    assert_eq!(fan_difference.contained_faces.len(), 2);
}

#[cfg(feature = "exact-triangulation")]
fn exercise_nonconvex_coplanar_volumetric_difference_fan_split() {
    let left = upward_l_prism_i64([[0, 0], [8, 0], [8, 3], [3, 3], [3, 8], [0, 8]], 5);
    let right = tetrahedron_i64([1, 1, 0], [7, 1, 0], [1, 7, 0], [1, 1, 5]);
    let graph = build_intersection_graph(&left, &right)
        .expect("nonconvex coplanar-volumetric graph should build");
    graph.validate().expect("graph should validate locally");
    graph
        .validate_against_meshes(&left, &right)
        .expect("graph should replay against sources");
    assert!(graph.face_pairs.iter().any(|pair| {
        pair.relation == hypermesh::exact::MeshFacePairRelation::CoplanarOverlapping
    }));

    for operation in [
        ExactBooleanOperation::Union,
        ExactBooleanOperation::Intersection,
        ExactBooleanOperation::Difference,
    ] {
        let preflight = preflight_boolean_exact(&left, &right, operation)
            .expect("nonconvex coplanar-volumetric preflight should build");
        preflight.validate().unwrap();
        preflight.validate_against_sources(&left, &right).unwrap();
        assert_eq!(
            preflight.support,
            ExactBooleanSupport::CertifiedWindingMaterialized
        );
        let result =
            hypermesh::exact::boolean_exact(&left, &right, operation, ValidationPolicy::CLOSED)
                .expect("nonconvex coplanar-volumetric boolean should materialize");
        result
            .validate_operation_against_sources(
                &left,
                &right,
                operation,
                ValidationPolicy::CLOSED,
                ExactBoundaryBooleanPolicy::Reject,
            )
            .unwrap();
        assert!(result.mesh.facts().mesh.closed_manifold);
        if operation == ExactBooleanOperation::Difference {
            assert!(assembly_has_duplicate_exact_point(&result.assembly));
        }
    }
}

#[cfg(feature = "exact-triangulation")]
fn assembly_has_duplicate_exact_point(assembly: &ExactBooleanAssemblyPlan) -> bool {
    assembly.vertices.iter().enumerate().any(|(left_index, left)| {
        assembly
            .vertices
            .iter()
            .skip(left_index + 1)
            .any(|right| exact_point3_eq(&left.point, &right.point))
    })
}

#[cfg(feature = "exact-triangulation")]
fn exact_point3_eq(left: &Point3, right: &Point3) -> bool {
    compare_reals(&left.x, &right.x).value() == Some(Ordering::Equal)
        && compare_reals(&left.y, &right.y).value() == Some(Ordering::Equal)
        && compare_reals(&left.z, &right.z).value() == Some(Ordering::Equal)
}

#[cfg(feature = "exact-triangulation")]
fn rect_surface_i64(rectangles: &[(i64, i64, i64, i64)]) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 12);
    let mut indices = Vec::with_capacity(rectangles.len() * 6);
    for (rectangle, &(x0, y0, x1, y1)) in rectangles.iter().enumerate() {
        let base = rectangle * 4;
        coordinates.extend_from_slice(&[
            x0, y0, 0, //
            x1, y0, 0, //
            x1, y1, 0, //
            x0, y1, 0,
        ]);
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("rectangular fuzz surface fixture must import")
}

#[cfg(feature = "exact-triangulation")]
fn fan_rect_surface_i64(rectangles: &[(i64, i64, i64, i64)]) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 15);
    let mut indices = Vec::with_capacity(rectangles.len() * 12);
    for (rectangle, &(x0, y0, x1, y1)) in rectangles.iter().enumerate() {
        let base = rectangle * 5;
        assert_eq!((x0 + x1) % 2, 0);
        assert_eq!((y0 + y1) % 2, 0);
        let cx = (x0 + x1) / 2;
        let cy = (y0 + y1) / 2;
        coordinates.extend_from_slice(&[
            x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y1, 0, cx, cy, 0,
        ]);
        indices.extend_from_slice(&[
            base,
            base + 1,
            base + 4,
            base + 1,
            base + 2,
            base + 4,
            base + 2,
            base + 3,
            base + 4,
            base + 3,
            base,
            base + 4,
        ]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("fan-split rectangular fuzz surface fixture must import")
}

#[cfg(feature = "exact-triangulation")]
fn affine_rect_surface_i64(
    rectangles: &[(i64, i64, i64, i64)],
    origin: (i64, i64, i64),
    basis_u: (i64, i64, i64),
    basis_v: (i64, i64, i64),
) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 12);
    let mut indices = Vec::with_capacity(rectangles.len() * 6);
    let lift = |u: i64, v: i64| -> [i64; 3] {
        [
            origin.0 + u * basis_u.0 + v * basis_v.0,
            origin.1 + u * basis_u.1 + v * basis_v.1,
            origin.2 + u * basis_u.2 + v * basis_v.2,
        ]
    };
    for (rectangle, &(u0, v0, u1, v1)) in rectangles.iter().enumerate() {
        let base = rectangle * 4;
        for point in [lift(u0, v0), lift(u1, v0), lift(u1, v1), lift(u0, v1)] {
            coordinates.extend_from_slice(&point);
        }
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("affine rectangular fuzz surface fixture must import")
}

#[cfg(feature = "exact-triangulation")]
fn affine_fan_rect_surface_i64(
    rectangles: &[(i64, i64, i64, i64)],
    origin: (i64, i64, i64),
    basis_u: (i64, i64, i64),
    basis_v: (i64, i64, i64),
) -> ExactMesh {
    let mut coordinates = Vec::with_capacity(rectangles.len() * 15);
    let mut indices = Vec::with_capacity(rectangles.len() * 12);
    let lift = |u: i64, v: i64| -> [i64; 3] {
        [
            origin.0 + u * basis_u.0 + v * basis_v.0,
            origin.1 + u * basis_u.1 + v * basis_v.1,
            origin.2 + u * basis_u.2 + v * basis_v.2,
        ]
    };
    for (rectangle, &(u0, v0, u1, v1)) in rectangles.iter().enumerate() {
        let base = rectangle * 5;
        assert_eq!((u0 + u1) % 2, 0);
        assert_eq!((v0 + v1) % 2, 0);
        for point in [
            lift(u0, v0),
            lift(u1, v0),
            lift(u1, v1),
            lift(u0, v1),
            lift((u0 + u1) / 2, (v0 + v1) / 2),
        ] {
            coordinates.extend_from_slice(&point);
        }
        indices.extend_from_slice(&[
            base,
            base + 1,
            base + 4,
            base + 1,
            base + 2,
            base + 4,
            base + 2,
            base + 3,
            base + 4,
            base + 3,
            base,
            base + 4,
        ]);
    }
    ExactMesh::from_i64_triangles_with_policy(
        &coordinates,
        &indices,
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .expect("affine fan-split rectangular fuzz surface fixture must import")
}

#[cfg(feature = "exact-triangulation")]
fn point2(x: i64, y: i64) -> hypertri::ExactPoint {
    hypertri::ExactPoint::new(
        hypermesh::exact::ExactReal::from(x),
        hypermesh::exact::ExactReal::from(y),
    )
}

#[cfg(feature = "exact-triangulation")]
fn point3(x: i64, y: i64, z: i64) -> hyperlimit::Point3 {
    hyperlimit::Point3::new(
        hypermesh::exact::ExactReal::from(x),
        hypermesh::exact::ExactReal::from(y),
        hypermesh::exact::ExactReal::from(z),
    )
}

#[cfg(feature = "exact-triangulation")]
fn rational(numerator: i64, denominator: i64) -> hypermesh::exact::ExactReal {
    (hypermesh::exact::ExactReal::from(numerator) / hypermesh::exact::ExactReal::from(denominator))
        .expect("nonzero denominator")
}

fn reversed_surface_mesh(mesh: &ExactMesh) -> Option<ExactMesh> {
    let triangles = mesh
        .triangles()
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.0;
            Triangle([a, c, b])
        })
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        triangles,
        SourceProvenance::exact("fuzz reversed surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn reversed_vertex_fan_surface_mesh(mesh: &ExactMesh) -> Option<ExactMesh> {
    if mesh.vertices().len() < 3 {
        return None;
    }
    let mut vertices = mesh.vertices().to_vec();
    vertices.reverse();
    let triangles = (1..vertices.len() - 1)
        .map(|index| Triangle([0, index, index + 1]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("fuzz polygon surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn cross_component_surface_mesh(
    mesh: &ExactMesh,
    second_component_start: Option<usize>,
) -> Option<ExactMesh> {
    let second_component_start = second_component_start?;
    if second_component_start < 3 || mesh.triangles().is_empty() {
        return None;
    }
    if second_component_start >= mesh.vertices().len() {
        return None;
    }
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([0, 2, second_component_start])],
        SourceProvenance::exact("fuzz cross-component surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn filled_hole_surface_mesh(mesh: &ExactMesh, hole_start: usize) -> Option<ExactMesh> {
    if hole_start + 2 >= mesh.vertices().len() {
        return None;
    }
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([hole_start, hole_start + 1, hole_start + 2])],
        SourceProvenance::exact("fuzz filled-hole surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn first_triangle_only_surface_mesh(mesh: &ExactMesh) -> Option<ExactMesh> {
    if mesh.vertices().len() <= 3 {
        return None;
    }
    let first = *mesh.triangles().first()?;
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![first],
        SourceProvenance::exact("fuzz isolated retained surface vertices"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn fan_surface_mesh_with_swapped_tail(mesh: &ExactMesh) -> Option<ExactMesh> {
    if mesh.vertices().len() < 4 {
        return None;
    }
    let mut vertices = mesh.vertices().to_vec();
    vertices.swap(2, 3);
    let triangles = (1..vertices.len() - 1)
        .map(|index| Triangle([0, index, index + 1]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("fuzz fan surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

#[cfg(feature = "exact-triangulation")]
fn fan_surface_mesh_from_points(points: &[hyperlimit::Point3]) -> Option<ExactMesh> {
    if points.len() < 3 {
        return None;
    }
    let vertices = points
        .iter()
        .map(|point| {
            hypermesh::exact::ExactPoint3::new(
                point.x.clone(),
                point.y.clone(),
                point.z.clone(),
            )
        })
        .collect::<Vec<_>>();
    let triangles = (1..points.len() - 1)
        .map(|index| Triangle([0, index, index + 1]))
        .collect::<Vec<_>>();
    ExactMesh::new_with_policy(
        vertices,
        triangles,
        SourceProvenance::exact("fuzz retained-ring fan surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn boundary_mismatched_surface_mesh(mesh: &ExactMesh) -> Option<ExactMesh> {
    if mesh.vertices().len() < 4 {
        return None;
    }
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([0, 1, 2]), Triangle([0, 3, 1])],
        SourceProvenance::exact("fuzz mismatched retained surface boundary"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}

fn retained_ring_crossing_surface_mesh(mesh: &ExactMesh) -> Option<ExactMesh> {
    if mesh.vertices().len() <= 6 {
        return None;
    }
    ExactMesh::new_with_policy(
        mesh.vertices().to_vec(),
        vec![Triangle([0, 1, 6]), Triangle([0, 6, 3])],
        SourceProvenance::exact("fuzz retained ring crossing surface mesh"),
        ValidationPolicy::ALLOW_BOUNDARY,
    )
    .ok()
}
