#![no_main]

use hypermesh::exact::{
    CoplanarSurfaceContainment, CoplanarSurfaceContainmentStatus, CoplanarTriangleRelation,
    ExactBooleanOperation, ExactBooleanPolicy, ExactBoundaryBooleanPolicy, ExactMesh,
    ExactRegionSelection, FaceRegionPlaneRelation, FaceSplitBoundaryNode, SourceProvenance,
    Triangle, ValidationPolicy,
    arrange_single_triangle_coplanar_difference, arrange_single_triangle_coplanar_holed_difference,
    arrange_single_triangle_coplanar_union, arrange_coplanar_convex_surface_difference,
    arrange_coplanar_convex_surface_holed_difference, arrange_coplanar_convex_surface_intersection,
    arrange_coplanar_convex_surface_multi_difference, arrange_coplanar_convex_surface_union,
    boolean_exact_with_boundary_policy, boolean_selected_regions, certify_boundary_touching_report,
    certify_convex_solid, certify_open_surface_disjoint_report, certify_planar_arrangement_report,
    certify_refinement_report, certify_coplanar_convex_surface_containment,
    certify_coplanar_convex_surface_equivalence, certify_coplanar_convex_surface_report,
    certify_same_surface_report,
    certify_single_triangle_coplanar_containment, certify_single_triangle_coplanar_containment_report,
    certify_winding_readiness_report, classify_mesh_vertices_against_convex_solid,
    classify_mesh_vertices_against_convex_solid_report, difference_single_triangle_coplanar_surfaces,
    build_intersection_graph, intersect_single_triangle_coplanar_surfaces, preflight_boolean_exact,
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
    let _ = preflight_boolean_exact(
        &left,
        &right,
        ExactBooleanOperation::SelectedRegions(ExactRegionSelection::KeepAll),
    )
    .map(|report| {
        let _ = report.validate();
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
        let _ = certificate.validate();
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
        let _ = certificate.validate();
        let mut reversed_hull = certificate.clone();
        reversed_hull.polygon.reverse();
        assert!(reversed_hull.validate().is_err());
        let mut repeated_hull_point = certificate;
        if repeated_hull_point.polygon.len() > 1 {
            repeated_hull_point.polygon[1] = repeated_hull_point.polygon[0].clone();
            assert!(repeated_hull_point.validate().is_err());
        }
    }
    let _ = certify_coplanar_convex_surface_report(&left, &right).validate();
    let _ = certify_coplanar_convex_surface_report(&right, &left).validate();
    if let Some(certificate) = certify_coplanar_convex_surface_containment(&left, &right) {
        let _ = certificate.validate();
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
        let _ = certificate.validate();
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
        if matches!(
            report.status,
            hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind = hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding;
            assert!(wrong_kind.validate().is_err());
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
        if matches!(
            report.status,
            hypermesh::exact::ExactOpenSurfaceDisjointStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind = hypermesh::exact::ExactBooleanBlockerKind::NeedsWinding;
            assert!(wrong_kind.validate().is_err());
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
        if matches!(
            report.status,
            hypermesh::exact::ExactBoundaryTouchingStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind =
                hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy;
            assert!(wrong_kind.validate().is_err());
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
        if matches!(
            report.status,
            hypermesh::exact::ExactBoundaryTouchingStatus::GraphUnknowns
        ) {
            let mut wrong_kind = report.clone();
            wrong_kind.blocker.kind =
                hypermesh::exact::ExactBooleanBlockerKind::NeedsBoundaryPolicy;
            assert!(wrong_kind.validate().is_err());
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
            if matches!(
                report.status,
                hypermesh::exact::ExactPlanarArrangementStatus::GraphUnknowns
            ) {
                let mut wrong_kind = report.clone();
                wrong_kind.blocker.kind =
                    hypermesh::exact::ExactBooleanBlockerKind::NeedsPlanarArrangement;
                assert!(wrong_kind.validate().is_err());
            }
            if report.arrangement_readiness.is_some() {
                let mut mismatched_readiness = report.clone();
                if let Some(readiness) = mismatched_readiness.arrangement_readiness.as_mut() {
                    readiness.graph_count += 1;
                    readiness.touching_graphs += 1;
                }
                assert!(mismatched_readiness.validate().is_err());
            }
        },
    );
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
    let _ = certify_winding_readiness_report(&left, &right, ExactBooleanOperation::Union).map(
        |report| {
            let _ = report.validate();
            if report.arrangement_readiness.is_some() {
                let mut mismatched_readiness = report.clone();
                if let Some(readiness) = mismatched_readiness.arrangement_readiness.as_mut() {
                    readiness.graph_count += 1;
                    readiness.touching_graphs += 1;
                }
                assert!(mismatched_readiness.validate().is_err());
            }
            let mut undecided_region = report;
            if let Some(classification) = undecided_region.region_classifications.first_mut() {
                classification.relation = FaceRegionPlaneRelation::Unknown;
                classification.node_sides.fill(None);
                assert!(undecided_region.validate().is_err());
            }
        },
    );
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
    if let Ok(graph) = build_intersection_graph(&left, &right) {
        let _ = graph.validate();
        let _ = graph.validate_against_meshes(&left, &right);
        let mut relabeled_graph = graph.clone();
        if let Some(pair) = relabeled_graph.face_pairs.first_mut() {
            pair.left_face = usize::MAX;
            assert!(relabeled_graph.validate_against_meshes(&left, &right).is_err());
        }
        let _ = graph
            .coplanar_overlap_split_plan(&left, &right)
            .map(|plan| plan.validate_against_meshes(&left, &right));
        let _ = graph
            .coplanar_arrangement_readiness_report(&left, &right)
            .map(|report| report.validate());
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
        if let Some(coplanar) = &report.coplanar {
            report.status = match coplanar.relation {
                CoplanarTriangleRelation::Disjoint | CoplanarTriangleRelation::Unknown => {
                    CoplanarSurfaceContainmentStatus::AmbiguousOrIdentical
                }
                CoplanarTriangleRelation::Touching => {
                    CoplanarSurfaceContainmentStatus::Certified(
                        CoplanarSurfaceContainment::LeftInsideRight,
                    )
                }
                CoplanarTriangleRelation::Overlapping => {
                    CoplanarSurfaceContainmentStatus::DisjointOrUnknown
                }
            };
            assert!(report.validate().is_err());
        }
    }
    if let Some(output) = intersect_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate();
    }
    if let Some(output) = intersect_single_triangle_coplanar_surfaces(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = union_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate();
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
        let _ = output.validate();
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
        let _ = output.validate();
        let mut reversed_loop = output.clone();
        reversed_loop.polygon.reverse();
        if let Some(mesh) = reversed_vertex_fan_surface_mesh(&output.mesh) {
            reversed_loop.mesh = mesh;
            assert!(reversed_loop.validate().is_err());
        }
    }
    if let Some(output) = arrange_single_triangle_coplanar_union(&right, &left) {
        let _ = output.validate();
        let mut reversed_loop = output.clone();
        reversed_loop.polygon.reverse();
        if let Some(mesh) = reversed_vertex_fan_surface_mesh(&output.mesh) {
            reversed_loop.mesh = mesh;
            assert!(reversed_loop.validate().is_err());
        }
    }
    if let Some(output) = difference_single_triangle_coplanar_surfaces(&left, &right) {
        let _ = output.validate();
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
        let _ = output.validate();
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
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_difference(&right, &left) {
        let _ = output.validate();
    }
    if let Some(output) = arrange_single_triangle_coplanar_holed_difference(&left, &right) {
        let _ = output.validate();
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
        let _ = output.validate();
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
        let _ = output.validate();
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
        let _ = output.validate();
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
    if let Some(output) = arrange_coplanar_convex_surface_multi_difference(&left, &right) {
        let _ = output.validate();
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
        let _ = output.validate();
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
    let _ = classify_mesh_vertices_against_convex_solid_report(&left, &right).validate();
    let _ = classify_mesh_vertices_against_convex_solid_report(&right, &left).validate();

    if left.triangles().len() <= 4 && right.triangles().len() <= 4 {
        let _ =
            boolean_selected_regions(&left, &right, ExactBooleanPolicy::KEEP_ALL_BOUNDARY).map(
                |result| {
                    let _ = result.validate();
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
                        let point = outside_triangulation.assembly.vertices[vertex].point.clone();
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
                        assert!(reversed_orientation
                            .assembly
                            .validate_source_face_incidence(&left, &right)
                            .is_err());
                        assert!(reversed_orientation
                            .assembly
                            .checked_to_exact_mesh_with_sources(
                                &left,
                                &right,
                                ValidationPolicy::ALLOW_BOUNDARY,
                            )
                            .is_err());
                    }
                    let mut unreferenced_vertex = unclassified_triangulation.clone();
                    if let Some(vertex) = unreferenced_vertex.assembly.vertices.first().cloned() {
                        unreferenced_vertex.assembly.vertices.push(vertex);
                        assert!(unreferenced_vertex.assembly.validate().is_err());
                        assert!(unreferenced_vertex
                            .assembly
                            .to_exact_mesh(ValidationPolicy::ALLOW_BOUNDARY)
                            .is_err());
                        assert!(unreferenced_vertex.validate().is_err());
                    }
                    if let Some(triangulation) = unclassified_triangulation.triangulations.first_mut()
                    {
                        triangulation.face = usize::MAX;
                        assert!(unclassified_triangulation.validate().is_err());
                    }
                },
            );
    }
});

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
