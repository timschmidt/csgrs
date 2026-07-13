use super::probe_cache::{
    cached_halfspace_cell_seed_families_from_optional_report_with,
    cached_optional_halfspace_feasibility_report_with,
};
use super::*;
use crate::error::HypermeshError;
use crate::geometry::{axis_ref, classify_point, classify_real, compare_real};
use crate::halfspace::{aabb_core_halfspaces, axis_halfspace, support_side_halfspace};
use crate::polygon::{ConvexPolygon, make_quad, make_triangle};
use hyperlimit::Plane3 as LimitPlane3;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn quadrilateral_halfspace_cell_fixture() -> (Aabb, Vec<LimitPlane3>, Point3) {
    let bounds = Aabb::new(p(0, 0, 0), p(5, 4, 0));
    let support = Plane::axis_aligned(2, r(0));
    let interior = Point3::new(q(9, 4), r(2), r(0));
    let vertices = [p(0, 0, 0), p(4, 0, 0), p(5, 4, 0), p(0, 4, 0)];
    let mut halfspaces = vec![
        limit_plane_from_plane(&support),
        limit_plane_from_plane(&support.inverted()),
    ];

    for index in 0..vertices.len() {
        let next = (index + 1) % vertices.len();
        let mut edge_plane = Plane::from_points(
            &vertices[index],
            &vertices[next],
            &Point3::new(
                axis_ref(&vertices[index], 0).clone(),
                axis_ref(&vertices[index], 1).clone(),
                r(1),
            ),
        );
        if classify_real(&edge_plane.expression_at_point(&interior)).unwrap()
            == Classification::Positive
        {
            edge_plane = edge_plane.inverted();
        }
        halfspaces.push(limit_plane_from_plane(&edge_plane));
    }

    (bounds, halfspaces, Point3::new(q(5, 2), r(2), r(0)))
}

fn px(x: Real, y: i32, z: i32) -> Point3 {
    Point3::new(x, r(y), r(z))
}

#[test]
fn trace_retry_only_suppresses_unknown_classification() {
    assert_eq!(
        retryable_trace::<Vec<i32>>(Err(HypermeshError::UnknownClassification)).unwrap(),
        None
    );
    assert_eq!(
        retryable_trace::<Vec<i32>>(Err(HypermeshError::PointAtInfinity)),
        Err(HypermeshError::PointAtInfinity)
    );
}

#[test]
fn trace_axis_segment_rejects_transition_dimension_mismatch() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];

    assert_eq!(
        trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0, 0], &[wall]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn trace_axis_segment_reports_unknown_for_unmatched_edge_crossing() {
    let wall = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);

    assert_eq!(
        trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0], &[wall]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn trace_axis_segment_preserves_duplicate_strict_crossing_multiplicity() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];

    let traced =
        trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0], &[wall.clone(), wall]).unwrap();

    assert_eq!(traced.winding, vec![-2]);
}

#[test]
fn trace_axis_segment_pairs_each_coplanar_shared_edge_incidence() {
    let mut lower = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 1, 1), 0, 0);
    let mut upper = make_triangle(&p(1, -1, -1), &p(1, 1, 1), &p(1, -1, 1), 0, 1);
    lower.delta_w = vec![1];
    upper.delta_w = vec![1];

    let traced = trace_axis_segment(
        &p(0, 0, 0),
        &p(2, 0, 0),
        0,
        &[0],
        &[lower.clone(), upper.clone(), lower, upper],
    )
    .unwrap();

    assert_eq!(traced.winding, vec![-2]);
}

#[test]
fn trace_axis_segment_combines_strict_and_paired_edge_layers() {
    let mut full = make_quad(&p(1, -1, -1), &p(1, 1, -1), &p(1, 1, 1), &p(1, -1, 1), 0, 0);
    let mut lower = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 1, 1), 0, 1);
    let mut upper = make_triangle(&p(1, -1, -1), &p(1, 1, 1), &p(1, -1, 1), 0, 2);
    full.delta_w = vec![1];
    lower.delta_w = vec![1];
    upper.delta_w = vec![1];

    let traced =
        trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0], &[full, lower, upper]).unwrap();

    assert_eq!(traced.winding, vec![-2]);
}

#[test]
fn trace_axis_segment_rejects_duplicated_vertex_crossing() {
    let mut wall = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];

    assert_eq!(
        trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0], &[wall.clone(), wall],),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn trace_axis_segment_reports_unknown_for_endpoint_surface_contact() {
    let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

    assert_eq!(
        trace_axis_segment(&p(1, 0, 0), &p(2, 0, 0), 0, &[0], &[wall]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn trace_axis_segment_reports_unknown_for_zero_length_surface_contact() {
    let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

    assert_eq!(
        trace_axis_segment(&p(1, 0, 0), &p(1, 0, 0), 0, &[0], &[wall]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn trace_direct_segment_reports_unknown_for_unmatched_edge_crossing() {
    let wall = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);

    assert_eq!(
        trace_direct_segment(&p(0, 0, 0), &p(2, 0, 0), &[0], &[wall]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn trace_direct_segment_preserves_duplicate_strict_crossing_multiplicity() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];

    let traced =
        trace_direct_segment(&p(0, 0, 0), &p(2, 0, 0), &[0], &[wall.clone(), wall]).unwrap();

    assert_eq!(traced.winding, vec![-2]);
}

#[test]
fn trace_direct_segment_reports_unknown_for_endpoint_surface_contact() {
    let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

    assert_eq!(
        trace_direct_segment(&p(1, 0, 0), &p(2, 0, 0), &[0], &[wall]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn trace_direct_segment_reports_unknown_for_zero_length_surface_contact() {
    let wall = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

    assert_eq!(
        trace_direct_segment(&p(1, 0, 0), &p(1, 0, 0), &[0], &[wall]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn centroid_is_fallible_and_reports_empty_input() {
    assert_eq!(centroid(&[]).unwrap(), None);
    assert_eq!(
        centroid(&[p(0, 0, 0), p(2, 2, 2)]).unwrap(),
        Some(p(1, 1, 1))
    );
}

#[test]
fn endpoint_box_detours_are_cut_by_surface_crossings() {
    let slanted = make_triangle(&p(0, 2, -2), &p(0, 2, 2), &p(4, -2, 0), 0, 0);

    let cursor =
        InteriorBoxDetourTargetCursor::new(&p(0, 0, 0), &p(4, 4, 4), &[slanted], &[], None)
            .unwrap();

    assert!(
        cursor
            .bounds
            .iter()
            .any(|bounds| bounds.min.x == r(0) && bounds.max.x == r(2))
    );
    assert!(
        cursor
            .bounds
            .iter()
            .any(|bounds| bounds.min.x == r(2) && bounds.max.x == r(4))
    );
}

#[test]
fn strict_aabb_arrangement_cell_uses_strict_side_after_boundary_touch() {
    let bounds = Aabb::new(p(1, 0, 0), p(2, 1, 1));
    let planes = [Plane::axis_aligned(0, r(1))];

    assert_eq!(
        strict_aabb_arrangement_cell(&bounds, &planes).unwrap(),
        Some(vec![Classification::Positive])
    );
}

#[test]
fn endpoint_box_cursor_skips_boxes_confined_to_endpoint_arrangement_cell() {
    let start = p(1, 0, 0);
    let end = p(2, 1, 1);
    let planes = [Plane::axis_aligned(0, r(1))];

    let mut cursor = InteriorBoxDetourTargetCursor::new(&start, &end, &[], &planes, None).unwrap();

    assert!(cursor.bounds.is_empty());
    assert!(cursor.next_batch().unwrap().is_none());
}

#[test]
fn endpoint_box_cursor_keeps_boxes_crossing_arrangement_plane() {
    let start = p(-1, 0, 0);
    let end = p(1, 1, 1);
    let planes = [Plane::axis_aligned(0, r(0))];

    let cursor = InteriorBoxDetourTargetCursor::new(&start, &end, &[], &planes, None).unwrap();

    assert_eq!(cursor.bounds.len(), 1);
}

#[test]
fn bounded_cursor_keeps_unsplit_domain_crossing_surface_plane() {
    let start = p(0, 0, 0);
    let end = p(2, 1, 0);
    let wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    let planes = detour_arrangement_planes(std::slice::from_ref(&wall));
    let trace_bounds = Aabb::new(p(0, -1, -1), p(3, 2, 1));

    let cursor =
        InteriorBoxDetourTargetCursor::new(&start, &end, &[wall], &planes, Some(&trace_bounds))
            .unwrap();

    assert!(cursor.bounds.contains(&trace_bounds));
}

#[test]
fn detour_arrangement_uses_unique_polygon_support_planes() {
    let first = make_triangle(&p(0, 0, 0), &p(0, 2, 0), &p(0, 0, 2), 0, 0);
    let second = make_triangle(&p(0, 1, 1), &p(0, 3, 1), &p(0, 1, 3), 0, 1);

    let planes = detour_arrangement_planes(&[first.clone(), second]);

    assert_eq!(planes, vec![first.support]);
}

#[test]
fn open_arrangement_cell_certifies_unchanged_winding_path() {
    let planes = [Plane::axis_aligned(0, r(0)), Plane::axis_aligned(1, r(0))];

    assert!(points_share_open_arrangement_cell(&p(1, 1, 0), &p(2, 3, 4), &planes).unwrap());
    assert!(!points_share_open_arrangement_cell(&p(1, 1, 0), &p(-1, 1, 0), &planes).unwrap());
}

#[test]
fn arrangement_cell_shortcut_rejects_support_plane_boundary() {
    let planes = [Plane::axis_aligned(0, r(0))];

    assert!(!points_share_open_arrangement_cell(&p(0, 1, 0), &p(0, 2, 0), &planes).unwrap());
}

#[test]
fn detour_arrangement_cell_state_prefers_later_certified_target() {
    let cell = vec![Classification::Negative, Classification::Positive];
    let mut seen = Vec::new();

    record_detour_arrangement_cell_state(&mut seen, cell.clone(), true);
    assert!(detour_arrangement_cell_state_is_dominated(
        &seen, &cell, true
    ));
    assert!(!detour_arrangement_cell_state_is_dominated(
        &seen, &cell, false
    ));

    record_detour_arrangement_cell_state(&mut seen, cell.clone(), false);
    assert!(detour_arrangement_cell_state_is_dominated(
        &seen, &cell, true
    ));
    assert!(detour_arrangement_cell_state_is_dominated(
        &seen, &cell, false
    ));
}

#[test]
fn strict_aabb_targets_handle_degenerate_axis_boxes() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 6, 0));
    let targets = strict_aabb_targets(&bounds).unwrap();

    assert!(!targets.is_empty());
    for target in targets {
        assert_eq!(target.point.z, r(0));
        assert!(compare_real(&target.point.x, &r(0)).unwrap().is_gt());
        assert!(compare_real(&target.point.x, &r(4)).unwrap().is_lt());
        assert!(compare_real(&target.point.y, &r(0)).unwrap().is_gt());
        assert!(compare_real(&target.point.y, &r(6)).unwrap().is_lt());
        assert!(!target.definitions.is_empty());
    }
}

#[test]
fn strict_aabb_target_cursor_exhausts_legacy_target_set() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 6, 0));
    let expected = strict_aabb_targets_with_seed_families(
        &bounds,
        |bounds, halfspaces, report, saw_unknown| {
            halfspace_cell_seed_families_from_optional_report(
                bounds,
                halfspaces,
                report,
                saw_unknown,
            )
        },
    )
    .unwrap();
    let mut normalized_expected = Vec::new();
    for target in expected {
        push_unique_detour_target(&mut normalized_expected, target);
    }
    let mut cursor = StrictAabbTargetCursor::new(&bounds).unwrap();
    let mut actual = Vec::new();
    let mut batches = 0;
    while let Some(batch) = cursor.next_batch().unwrap() {
        batches += 1;
        actual.extend(batch);
    }

    assert!(batches >= 2);
    assert!(cursor.saw_unknown);
    let mut normalized_actual = Vec::new();
    for target in actual {
        push_unique_detour_target(&mut normalized_actual, target);
    }
    assert_eq!(normalized_actual.len(), normalized_expected.len());
    assert!(
        normalized_expected
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
    assert!(
        normalized_expected
            .iter()
            .all(|target| normalized_actual.iter().any(|candidate| {
                candidate.point == target.point
                    && definition_families_match_as_sets(
                        &candidate.definitions,
                        &target.definitions,
                    )
            }))
    );
}

#[test]
fn strict_aabb_target_cursor_exhausts_direct_targets_before_shifted_families() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 6, 8));
    let mut cursor = StrictAabbTargetCursor::new(&bounds).unwrap();

    assert_eq!(cursor.stage, StrictAabbTargetCursorStage::FrontDirect);
    assert!(!cursor.next_batch().unwrap().unwrap().is_empty());
    assert_eq!(cursor.stage, StrictAabbTargetCursorStage::DeferredDirect);
    assert!(!cursor.next_batch().unwrap().unwrap().is_empty());
    assert_eq!(cursor.stage, StrictAabbTargetCursorStage::Shifted);
}

#[test]
fn interior_box_target_batches_preserve_legacy_target_set_and_cursor_unknown() {
    let start = p(0, 0, 0);
    let end = p(4, 4, 0);
    let slanted = make_triangle(&p(0, 2, -2), &p(0, 2, 2), &p(4, -2, 0), 0, 0);
    let polygons = [slanted];
    let expected = interior_box_detour_targets(&start, &end, &polygons).unwrap();
    let mut cache = InteriorBoxDetourTargetBatchCache::default();
    let mut actual = Vec::new();
    let mut batch_index = 0;
    loop {
        match cache.batch_for(&start, &end, batch_index, &polygons, &[], None) {
            Ok(Some(batch)) => {
                assert_eq!(
                    cache
                        .batch_for(&start, &end, batch_index, &polygons, &[], None)
                        .unwrap(),
                    Some(batch.clone())
                );
                actual.extend(batch);
                batch_index += 1;
            }
            Err(HypermeshError::UnknownClassification) => break,
            other => panic!("expected uncertainty at cursor exhaustion, got {other:?}"),
        }
    }

    assert!(batch_index >= 2);
    assert_eq!(cache.entries.len(), 1);
    let mut normalized_expected = Vec::new();
    for target in expected {
        push_unique_detour_target(&mut normalized_expected, target);
    }
    let mut normalized_actual = Vec::new();
    for target in actual {
        push_unique_detour_target(&mut normalized_actual, target);
    }
    assert_eq!(normalized_actual.len(), normalized_expected.len());
    assert!(normalized_expected.iter().all(|target| {
        normalized_actual.iter().any(|candidate| {
            candidate.point == target.point
                && definition_families_match_as_sets(&candidate.definitions, &target.definitions)
        })
    }));
}

#[test]
fn detour_target_family_marks_surviving_targets_uncertain_after_unknown() {
    let targets = detour_target_family_result_from_targets(
        vec![
            DetourTarget {
                point: p(1, 1, 1),
                definitions: vec![axis_plane_definition(&p(1, 1, 1))],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: p(2, 2, 2),
                definitions: vec![axis_plane_definition(&p(2, 2, 2))],
                uncertified_definition_fallback: false,
            },
        ],
        true,
    )
    .unwrap();

    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn interior_box_target_batches_preserve_unknown_after_emitting_targets() {
    let start = p(0, 0, 0);
    let end = p(4, 4, 0);
    let slanted = make_triangle(&p(0, 2, -2), &p(0, 2, 2), &p(4, -2, 0), 0, 0);
    let polygons = [slanted];
    let mut cache = InteriorBoxDetourTargetBatchCache::default();

    let first = cache
        .batch_for(&start, &end, 0, &polygons, &[], None)
        .unwrap()
        .expect("detour cursor should emit an initial target batch");
    assert!(!first.is_empty());
    cache.entries[0].cursor.saw_unknown = true;

    let mut batch_index = 1;
    loop {
        match cache.batch_for(&start, &end, batch_index, &polygons, &[], None) {
            Ok(Some(_)) => batch_index += 1,
            Err(HypermeshError::UnknownClassification) => break,
            other => panic!("expected uncertainty at cursor exhaustion, got {other:?}"),
        }
    }
}

#[test]
fn strict_aabb_targets_try_shifted_search_from_report_witness_seed() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let targets = strict_aabb_targets_with_seed_families(
        &bounds,
        |_bounds, _halfspaces, _report, _saw_unknown| Ok((Vec::new(), Vec::new(), Vec::new())),
    )
    .unwrap();

    assert!(!targets.is_empty());
    assert!(targets.iter().all(|target| !target.definitions.is_empty()));
}

#[test]
fn search_strict_aabb_targets_progressively_stops_after_first_certified_direct_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let mut evaluated = 0usize;

    let found = search_strict_aabb_targets_progressively_with_seed_families(
        &bounds,
        |_bounds, _halfspaces, _report, _saw_unknown| {
            Ok((vec![p(1, 1, 1)], vec![p(2, 2, 2)], vec![p(3, 3, 3)]))
        },
        &mut |_target| {
            evaluated += 1;
            Ok(true)
        },
    )
    .unwrap();

    assert!(found);
    assert_eq!(evaluated, 1);
}

#[test]
fn search_strict_aabb_targets_progressively_ranks_front_direct_slice_before_evaluation() {
    let mut evaluated = Vec::new();

    let found = evaluate_strict_aabb_target_families_with_direct_ranking(
        StrictAabbTargetFamilies {
            direct_targets: vec![
                DetourTarget {
                    point: p(1, 1, 1),
                    definitions: vec![axis_plane_definition(&p(1, 1, 1))],
                    uncertified_definition_fallback: false,
                },
                DetourTarget {
                    point: p(2, 2, 2),
                    definitions: vec![axis_plane_definition(&p(2, 2, 2))],
                    uncertified_definition_fallback: false,
                },
            ],
            shifted_targets: Vec::new(),
            saw_unknown: false,
        },
        &mut |target| {
            if target.point == p(2, 2, 2) {
                Ok([0u8, 0u8])
            } else {
                Ok([1u8, 1u8])
            }
        },
        &mut |target| {
            evaluated.push(target.point.clone());
            Ok(true)
        },
    )
    .unwrap();

    assert!(found);
    assert_eq!(evaluated, vec![p(2, 2, 2)]);
}

#[test]
fn search_strict_aabb_targets_progressively_tries_shifted_targets_before_deferred_direct_targets() {
    let mut evaluated = Vec::new();

    let found = evaluate_strict_aabb_target_families_with_direct_ranking(
        StrictAabbTargetFamilies {
            direct_targets: vec![
                DetourTarget {
                    point: p(1, 1, 1),
                    definitions: vec![axis_plane_definition(&p(1, 1, 1))],
                    uncertified_definition_fallback: false,
                },
                DetourTarget {
                    point: p(2, 2, 2),
                    definitions: vec![axis_plane_definition(&p(2, 2, 2))],
                    uncertified_definition_fallback: false,
                },
            ],
            shifted_targets: vec![DetourTarget {
                point: p(4, 1, 1),
                definitions: vec![axis_plane_definition(&p(4, 1, 1))],
                uncertified_definition_fallback: false,
            }],
            saw_unknown: false,
        },
        &mut |_target| Ok([0u8, 0u8]),
        &mut |target| {
            evaluated.push(target.point.clone());
            Ok(target.point == p(4, 1, 1))
        },
    )
    .unwrap();

    assert!(found);
    assert_eq!(evaluated, vec![p(1, 1, 1), p(2, 2, 2), p(4, 1, 1)]);
}

#[test]
fn strict_aabb_target_evaluation_accepts_proven_fallback_target() {
    let target = DetourTarget {
        point: p(1, 1, 1),
        definitions: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: true,
    };

    let found = evaluate_strict_aabb_target_families_with_direct_ranking(
        StrictAabbTargetFamilies {
            direct_targets: vec![target],
            shifted_targets: Vec::new(),
            saw_unknown: true,
        },
        &mut |_target| Ok(()),
        &mut |_target| Ok(true),
    )
    .unwrap();

    assert!(found);
}

#[test]
fn strict_aabb_target_evaluation_preserves_unproven_fallback_unknown() {
    let target = DetourTarget {
        point: p(1, 1, 1),
        definitions: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: true,
    };

    let err = evaluate_strict_aabb_target_families_with_direct_ranking(
        StrictAabbTargetFamilies {
            direct_targets: vec![target],
            shifted_targets: Vec::new(),
            saw_unknown: false,
        },
        &mut |_target| Ok(()),
        &mut |_target| Ok(false),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn search_strict_aabb_targets_progressively_preserves_unknown_in_exhausted_families() {
    let outcome =
        search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome(
            &Aabb::new(p(0, 0, 0), p(3, 3, 3)),
            |_bounds, _halfspaces, _report, _saw_unknown| {
                Ok((vec![p(1, 1, 1), p(2, 2, 2)], vec![p(4, 4, 4)], vec![]))
            },
            &mut |_target| Ok([0u8, 0u8]),
            &mut |_target| Ok(false),
        );

    assert_eq!(outcome.result, Err(HypermeshError::UnknownClassification));
    let families = outcome
        .exhausted_families
        .expect("progressive miss should retain the full family set");
    assert_eq!(families.direct_targets.len(), 2);
    assert!(!families.shifted_targets.is_empty());
    assert!(families.saw_unknown);
}

#[test]
fn search_strict_aabb_targets_progressively_dedupes_duplicate_direct_targets() {
    let mut evaluated = 0usize;

    let found =
        search_strict_aabb_targets_progressively_with_seed_families_and_direct_ranking_outcome(
            &Aabb::new(p(0, 0, 0), p(3, 3, 3)),
            |_bounds, _halfspaces, _report, _saw_unknown| {
                Ok((vec![p(1, 1, 1), p(1, 1, 1)], vec![], vec![]))
            },
            &mut |_target| Ok([0u8, 0u8, 0u8]),
            &mut |_target| {
                evaluated += 1;
                Ok(true)
            },
        )
        .result
        .unwrap();

    assert!(found);
    assert_eq!(evaluated, 1);
}

#[test]
fn no_plane_detour_target_evaluation_prefers_lower_ranked_leg_first() {
    let point = p(1, 1, 1);
    let start_definitions = axis_plane_definition(&p(1, 1, 1));
    let detour_definitions = axis_plane_definition(&p(2, 2, 2));
    let end_definitions = axis_plane_definition(&p(3, 3, 3));
    let detour = DetourTarget {
        point: point.clone(),
        definitions: vec![detour_definitions.clone()],
        uncertified_definition_fallback: false,
    };
    let mut trace_calls = Vec::new();
    let mut saw_unknown = false;

    let result = evaluate_probe_detour_target_without_plane_replacement_with_surface_query(
        &detour,
        &point,
        &point,
        &[],
        &[],
        std::slice::from_ref(&start_definitions),
        std::slice::from_ref(&end_definitions),
        true,
        &mut DefinitionNoPlaneReplacementCycleGuardCache::default(),
        &DefinitionNoPlaneReplacementReachabilityCache::default(),
        &mut Vec::new(),
        &mut Vec::new(),
        &mut StrictAabbTargetFamilyCache::default(),
        &mut InteriorBoxAxisIntervalsCache::default(),
        &mut Vec::new(),
        &mut |_point| Ok(false),
        &mut |_from, _to, from_definitions, to_definitions| {
            let call = if from_definitions == [start_definitions.clone()]
                && to_definitions == [detour_definitions.clone()]
            {
                "start_to_detour"
            } else if from_definitions == [detour_definitions.clone()]
                && to_definitions == [end_definitions.clone()]
            {
                "detour_to_end"
            } else {
                panic!("unexpected trace leg")
            };
            trace_calls.push(call);
            if call == "start_to_detour" {
                Ok(false)
            } else {
                Ok(true)
            }
        },
        &mut DetourTargetFamilyCache::default(),
        &mut |_from, _to| Ok(Vec::new()),
        &mut saw_unknown,
    )
    .unwrap();

    assert!(!result);
    assert!(!saw_unknown);
    assert_eq!(trace_calls, vec!["start_to_detour", "detour_to_end"]);
}

#[test]
fn detour_target_marking_marks_existing_targets_uncertain() {
    let mut targets = vec![DetourTarget {
        point: p(1, 2, 3),
        definitions: vec![axis_plane_definition(&p(1, 2, 3))],
        uncertified_definition_fallback: false,
    }];

    mark_all_detour_targets_uncertified(&mut targets);

    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn detour_target_build_collection_backtracks_after_uncertified_candidate() {
    let first = p(1, 2, 3);
    let second = p(1, 2, 4);
    let mut targets = Vec::new();

    extend_detour_target_builds_backtracking_unknown(&mut targets, [&first, &second], |point| {
        if *point == first {
            Err(HypermeshError::UnknownClassification)
        } else {
            Ok(DetourTarget {
                point: point.clone(),
                definitions: vec![axis_plane_definition(point)],
                uncertified_definition_fallback: false,
            })
        }
    })
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, second);
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn detour_target_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let first = p(1, 2, 3);
    let second = p(1, 2, 4);
    let mut targets = Vec::new();

    let err = extend_detour_target_builds_backtracking_unknown(
        &mut targets,
        [&first, &second],
        |_point| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert!(targets.is_empty());
}

#[test]
fn detour_target_build_collection_marks_later_targets_uncertain_after_uncertain_candidate_result() {
    let first = p(1, 2, 3);
    let second = p(1, 2, 4);
    let mut targets = Vec::new();

    extend_detour_target_builds_backtracking_unknown(&mut targets, [&first, &second], |point| {
        Ok(DetourTarget {
            point: point.clone(),
            definitions: vec![axis_plane_definition(point)],
            uncertified_definition_fallback: *point == first,
        })
    })
    .unwrap();

    assert_eq!(targets.len(), 2);
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn detour_target_build_collection_keeps_certified_duplicate_state_certified() {
    let point = p(1, 2, 3);
    let definition = axis_plane_definition(&point);
    let mut targets = Vec::new();

    extend_detour_target_builds_backtracking_unknown(&mut targets, [0, 1].iter(), |candidate| {
        Ok(DetourTarget {
            point: point.clone(),
            definitions: vec![definition.clone()],
            uncertified_definition_fallback: *candidate == 0,
        })
    })
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn detour_target_from_shifted_witness_stays_certified_when_one_family_is_singular() {
    let witness = ShiftedHalfspaceWitness {
        point: p(1, 1, 1),
        families: vec![
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(2, false, r(2))],
                active_planes: [Some(9), None, None],
            },
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(2))],
                active_planes: [Some(0), None, None],
            },
        ],
        uncertified_definition_fallback: false,
    };

    let target = build_detour_target_from_shifted_witness(&witness).unwrap();

    assert_eq!(target.point, witness.point);
    assert!(!target.uncertified_definition_fallback);
    assert!(!target.definitions.is_empty());
}

#[test]
fn detour_target_family_collection_marks_later_targets_uncertain_after_uncertified_family() {
    let mut targets = Vec::new();

    extend_detour_target_families_backtracking_unknown(
        &mut targets,
        [
            Err(HypermeshError::UnknownClassification),
            Ok(vec![DetourTarget {
                point: p(1, 2, 4),
                definitions: vec![axis_plane_definition(&p(1, 2, 4))],
                uncertified_definition_fallback: false,
            }]),
        ],
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(1, 2, 4));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn detour_target_family_collection_marks_later_targets_uncertain_after_uncertain_family_result() {
    let mut targets = Vec::new();

    extend_detour_target_families_backtracking_unknown(
        &mut targets,
        [
            Ok(vec![DetourTarget {
                point: p(1, 2, 3),
                definitions: vec![axis_plane_definition(&p(1, 2, 3))],
                uncertified_definition_fallback: true,
            }]),
            Ok(vec![DetourTarget {
                point: p(1, 2, 4),
                definitions: vec![axis_plane_definition(&p(1, 2, 4))],
                uncertified_definition_fallback: false,
            }]),
        ],
    )
    .unwrap();

    assert_eq!(targets.len(), 2);
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn detour_target_family_collection_reports_unknown_if_all_families_are_uncertified() {
    let mut targets = Vec::new();

    let err = extend_detour_target_families_backtracking_unknown(
        &mut targets,
        [
            Err(HypermeshError::UnknownClassification),
            Err(HypermeshError::UnknownClassification),
        ],
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert!(targets.is_empty());
}

#[test]
fn interior_box_detour_target_collection_backtracks_after_uncertified_box_family() {
    let intervals = vec![
        vec![(r(0), r(1)), (r(1), r(2))],
        vec![(r(0), r(1))],
        vec![(r(0), r(1))],
    ];

    let targets = collect_detour_targets_from_axis_intervals(&intervals, |bounds| {
        if bounds.min == p(0, 0, 0) && bounds.max == p(1, 1, 1) {
            Err(HypermeshError::UnknownClassification)
        } else {
            let point = Point3::new(r(1), q(1, 2), q(1, 2));
            Ok(vec![DetourTarget {
                point: point.clone(),
                definitions: vec![axis_plane_definition(&point)],
                uncertified_definition_fallback: false,
            }])
        }
    })
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, Point3::new(r(1), q(1, 2), q(1, 2)));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn axis_box_surface_cut_collection_backtracks_after_uncertified_crossing() {
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
        &start,
        &end,
        &[first, second],
        &mut |_edge_start, _edge_end, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(p(1, 0, 0)))
            }
        },
        &mut |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(intervals[0], vec![(r(0), r(1)), (r(1), r(2))]);
}

#[test]
fn interior_box_detour_target_collection_marks_surviving_targets_uncertain_after_uncertified_surface_cut()
 {
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let targets = interior_box_detour_targets_with_queries(
        &start,
        &end,
        &[first, second],
        |_edge_start, _edge_end, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(p(1, 0, 0)))
            }
        },
        |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
        |bounds| {
            if *bounds == Aabb::new(p(1, 0, 0), p(2, 0, 0)) {
                let point = p(1, 0, 0);
                Ok(vec![DetourTarget {
                    point: point.clone(),
                    definitions: vec![axis_plane_definition(&point)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(1, 0, 0));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn interior_box_detour_target_collection_reports_unknown_when_surface_cut_family_is_partially_uncertified_and_boxes_fail()
 {
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let err = interior_box_detour_targets_with_queries(
        &start,
        &end,
        &[first, second],
        |_edge_start, _edge_end, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(p(1, 0, 0)))
            }
        },
        |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
        |_bounds| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn axis_box_surface_cut_collection_treats_boundary_crossing_as_unknown_and_keeps_later_cut() {
    let start = p(0, 0, 0);
    let end = p(3, 0, 0);
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
        &start,
        &end,
        &[first, second],
        &mut |_edge_start, _edge_end, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            Ok(Some(Point3::new(x, r(0), r(0))))
        },
        &mut |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(intervals[0], vec![(r(0), r(1)), (r(1), r(2)), (r(2), r(3))]);
}

#[test]
fn axis_box_surface_cut_collection_treats_endpoint_boundary_contact_as_unknown_and_keeps_later_cut()
{
    let start = p(0, 0, 0);
    let end = p(3, 0, 0);
    let first = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
        &start,
        &end,
        &[first, second],
        &mut |_edge_start, edge_end, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            if x == r(3) {
                Ok(Some(edge_end.clone()))
            } else {
                Ok(Some(Point3::new(x, r(0), r(0))))
            }
        },
        &mut |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(3) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(intervals[0], vec![(r(0), r(2)), (r(2), r(3))]);
}

#[test]
fn axis_box_surface_cut_collection_treats_start_boundary_contact_as_unknown_and_keeps_later_cut() {
    let start = p(0, 0, 0);
    let end = p(3, 0, 0);
    let first = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (intervals, saw_unknown) = interior_box_axis_intervals_with_surface_queries(
        &start,
        &end,
        &[first, second],
        &mut |edge_start, _edge_end, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            if x == r(0) {
                Ok(Some(edge_start.clone()))
            } else {
                Ok(Some(Point3::new(x, r(0), r(0))))
            }
        },
        &mut |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(0) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(intervals[0], vec![(r(0), r(2)), (r(2), r(3))]);
}

#[test]
fn interior_box_detour_target_collection_marks_surviving_targets_uncertain_after_boundary_surface_cut()
 {
    let start = p(0, 0, 0);
    let end = p(3, 0, 0);
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let targets = interior_box_detour_targets_with_queries(
        &start,
        &end,
        &[first, second],
        |_edge_start, _edge_end, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            Ok(Some(Point3::new(x, r(0), r(0))))
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
        |bounds| {
            if *bounds == Aabb::new(p(2, 0, 0), p(3, 0, 0)) {
                let point = p(2, 0, 0);
                Ok(vec![DetourTarget {
                    point: point.clone(),
                    definitions: vec![axis_plane_definition(&point)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(2, 0, 0));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn interior_box_detour_target_collection_marks_surviving_targets_uncertain_after_endpoint_boundary_surface_cut()
 {
    let start = p(0, 0, 0);
    let end = p(3, 0, 0);
    let first = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let targets = interior_box_detour_targets_with_queries(
        &start,
        &end,
        &[first, second],
        |_edge_start, edge_end, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            if x == r(3) {
                Ok(Some(edge_end.clone()))
            } else {
                Ok(Some(Point3::new(x, r(0), r(0))))
            }
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(3) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
        |bounds| {
            if *bounds == Aabb::new(p(2, 0, 0), p(3, 0, 0)) {
                let point = p(2, 0, 0);
                Ok(vec![DetourTarget {
                    point: point.clone(),
                    definitions: vec![axis_plane_definition(&point)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(2, 0, 0));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn interior_box_detour_target_collection_marks_surviving_targets_uncertain_after_start_boundary_surface_cut()
 {
    let start = p(0, 0, 0);
    let end = p(3, 0, 0);
    let first = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let targets = interior_box_detour_targets_with_queries(
        &start,
        &end,
        &[first, second],
        |edge_start, _edge_end, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            if x == r(0) {
                Ok(Some(edge_start.clone()))
            } else {
                Ok(Some(Point3::new(x, r(0), r(0))))
            }
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(0) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
        |bounds| {
            if *bounds == Aabb::new(p(2, 0, 0), p(3, 0, 0)) {
                let point = p(2, 0, 0);
                Ok(vec![DetourTarget {
                    point: point.clone(),
                    definitions: vec![axis_plane_definition(&point)],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(2, 0, 0));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn detour_target_build_preserves_inherited_uncertified_definition_fallback() {
    let point = p(1, 1, 1);
    let halfspaces = vec![axis_halfspace(2, false, r(1))];

    let target = build_detour_target(&point, &halfspaces, [None, None, None], true).unwrap();

    assert!(target.uncertified_definition_fallback);
}

#[test]
fn duplicate_detour_targets_merge_permuted_plane_definitions() {
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let mut targets = vec![DetourTarget {
        point: point.clone(),
        definitions: vec![definition],
        uncertified_definition_fallback: false,
    }];

    push_unique_detour_target(
        &mut targets,
        DetourTarget {
            point,
            definitions: vec![permuted.clone()],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].definitions.len(), 1);
    assert!(definition_planes_match_as_sets(
        &targets[0].definitions[0],
        &permuted
    ));
}

#[test]
fn duplicate_detour_targets_prefer_certified_duplicate_definitions() {
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);
    let mut targets = vec![DetourTarget {
        point: point.clone(),
        definitions: vec![definition.clone()],
        uncertified_definition_fallback: true,
    }];

    push_unique_detour_target(
        &mut targets,
        DetourTarget {
            point,
            definitions: vec![definition],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(targets.len(), 1);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn shifted_halfspace_cell_vertex_witnesses_return_strict_points() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let witnesses = shifted_halfspace_cell_vertex_witnesses(&bounds, &halfspaces).unwrap();

    assert!(!witnesses.is_empty());
    for witness in &witnesses {
        assert!(
            point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
        );
    }
}

#[test]
fn shifted_halfspace_cell_geometry_witnesses_return_strict_points() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = vec![
        axis_halfspace(0, true, r(0)),
        axis_halfspace(1, true, r(0)),
        axis_halfspace(2, true, r(0)),
        LimitPlane3::new(p(1, 1, 1), r(-4)),
    ];

    let witnesses = shifted_halfspace_cell_geometry_witnesses(&bounds, &halfspaces).unwrap();

    assert!(!witnesses.is_empty());
    for witness in &witnesses {
        assert!(
            point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
        );
    }
}

#[test]
fn shifted_halfspace_cell_witnesses_from_seed_returns_only_strict_points() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let witnesses =
        shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

    assert!(!witnesses.is_empty());
    for witness in &witnesses {
        assert!(
            point_strictly_inside_halfspace_cell(&witness.point, &bounds, &halfspaces).unwrap()
        );
    }
}

#[test]
fn feasible_halfspace_cell_vertices_backtrack_after_uncertified_candidate() {
    let halfspaces = vec![
        axis_halfspace(0, true, r(0)),
        axis_halfspace(0, false, r(0)),
        axis_halfspace(1, true, r(0)),
        axis_halfspace(1, false, r(0)),
        axis_halfspace(2, true, r(0)),
        axis_halfspace(2, false, r(1)),
    ];
    let first = p(0, 0, 0);
    let second = p(0, 0, 1);

    let vertices = feasible_halfspace_cell_vertices_with_contains(&halfspaces, |point, _| {
        if point == &first {
            Err(HypermeshError::UnknownClassification)
        } else {
            Ok(point == &second)
        }
    })
    .unwrap();

    assert_eq!(vertices, vec![second]);
}

#[test]
fn feasible_halfspace_cell_vertex_family_tracks_unknown_after_later_vertex() {
    let halfspaces = vec![
        axis_halfspace(0, true, r(0)),
        axis_halfspace(0, false, r(0)),
        axis_halfspace(1, true, r(0)),
        axis_halfspace(1, false, r(0)),
        axis_halfspace(2, true, r(0)),
        axis_halfspace(2, false, r(1)),
    ];
    let first = p(0, 0, 0);
    let second = p(0, 0, 1);

    let family = feasible_halfspace_cell_vertex_family_with_contains(&halfspaces, |point, _| {
        if point == &first {
            Err(HypermeshError::UnknownClassification)
        } else {
            Ok(point == &second)
        }
    })
    .unwrap();

    assert_eq!(family.seeds, vec![second]);
    assert!(family.saw_unknown);
}

#[test]
fn feasible_halfspace_cell_vertices_report_unknown_if_all_candidates_are_uncertified() {
    let halfspaces = vec![
        axis_halfspace(0, true, r(0)),
        axis_halfspace(0, false, r(0)),
        axis_halfspace(1, true, r(0)),
        axis_halfspace(1, false, r(0)),
        axis_halfspace(2, true, r(0)),
        axis_halfspace(2, false, r(1)),
    ];

    let err = feasible_halfspace_cell_vertices_with_contains(&halfspaces, |_point, _| {
        Err(HypermeshError::UnknownClassification)
    })
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn halfspace_cell_geometry_seed_candidates_from_vertices_matches_direct_query() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let vertices = feasible_halfspace_cell_vertices(&halfspaces).unwrap();

    let from_vertices = halfspace_cell_geometry_seed_candidates_from_vertices(&vertices).unwrap();
    let from_query = halfspace_cell_geometry_seed_candidates(&halfspaces).unwrap();

    assert_eq!(from_vertices, from_query);
}

#[test]
fn halfspace_centroid_subset_seed_family_tracks_unknown_after_later_centroid() {
    let vertices = vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)];
    let blocked_subset = vec![vertices[0].clone(), vertices[1].clone()];

    let family = halfspace_centroid_subset_seed_family_from_vertices_with(&vertices, |subset| {
        if subset == blocked_subset.as_slice() {
            Err(HypermeshError::UnknownClassification)
        } else {
            centroid(subset)
        }
    })
    .unwrap();

    assert!(family.saw_unknown);
    assert!(!family.seeds.is_empty());
}

#[test]
fn shifted_halfspace_cell_witnesses_from_seed_include_shifted_vertex_targets() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let witnesses =
        shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

    assert!(
        witnesses
            .iter()
            .any(|witness| witness.point == Point3::new(r(3), q(5, 2), q(7, 2)))
    );
    assert!(
        witnesses
            .iter()
            .find(|witness| witness.point == Point3::new(r(3), q(5, 2), q(7, 2)))
            .is_some_and(|witness| witness
                .families
                .iter()
                .any(|family| family.active_planes == [None, None, None]))
    );
}

#[test]
fn shifted_halfspace_cell_witnesses_from_seed_include_shifted_geometry_targets() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let witnesses =
        shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

    assert!(
        witnesses
            .iter()
            .any(|witness| witness.point == Point3::new(r(1), q(7, 6), q(13, 6)))
    );
    assert!(
        witnesses
            .iter()
            .find(|witness| witness.point == Point3::new(r(1), q(7, 6), q(13, 6)))
            .is_some_and(|witness| witness
                .families
                .iter()
                .any(|family| family.active_planes == [None, None, None]))
    );
}

#[test]
fn shifted_halfspace_witness_collection_backtracks_after_uncertified_candidate() {
    let mut witnesses = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        [first.clone(), second.clone()],
        |candidate| {
            if *candidate == first {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    candidate.clone(),
                    Vec::new(),
                    [None, None, None],
                    false,
                )])
            }
        },
    )
    .unwrap();

    assert_eq!(witnesses.len(), 1);
    assert_eq!(witnesses[0].point, second);
    assert!(witnesses[0].uncertified_definition_fallback);
}

#[test]
fn shifted_halfspace_witness_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let mut witnesses = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    let err = extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        [first, second],
        |_candidate| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn shifted_halfspace_witness_collection_backtracks_after_uncertified_seed() {
    let first_seed = p(1, 1, 1);
    let second_seed = p(2, 2, 2);
    let kept =
        ShiftedHalfspaceWitness::with_family(p(3, 3, 3), Vec::new(), [None, None, None], false);
    let mut witnesses = Vec::new();

    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        vec![first_seed.clone(), second_seed.clone()],
        |seed| {
            if seed == &first_seed {
                Err(HypermeshError::UnknownClassification)
            } else if seed == &second_seed {
                Ok(vec![kept.clone()])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(witnesses.len(), 1);
    assert_eq!(witnesses[0].point, kept.point);
    assert!(witnesses[0].uncertified_definition_fallback);
}

#[test]
fn shifted_halfspace_witness_collection_marks_later_witnesses_uncertain_after_uncertain_candidate_result()
 {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut witnesses = Vec::new();

    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        [first.clone(), second.clone()],
        |seed| {
            if *seed == first {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    seed.clone(),
                    Vec::new(),
                    [None, None, None],
                    true,
                )])
            } else {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    seed.clone(),
                    Vec::new(),
                    [None, None, None],
                    false,
                )])
            }
        },
    )
    .unwrap();

    assert_eq!(witnesses.len(), 2);
    assert!(
        witnesses
            .iter()
            .all(|witness| witness.uncertified_definition_fallback)
    );
}

#[test]
fn shifted_halfspace_witness_collection_keeps_certified_duplicate_state_certified() {
    let first_seed = p(1, 1, 1);
    let second_seed = p(2, 2, 2);
    let witness_point = p(3, 3, 3);
    let mut witnesses = Vec::new();

    extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        [first_seed, second_seed],
        |seed| {
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness_point.clone(),
                Vec::new(),
                [None, None, None],
                *seed == p(1, 1, 1),
            )])
        },
    )
    .unwrap();

    assert_eq!(witnesses.len(), 1);
    assert_eq!(witnesses[0].point, witness_point);
    assert!(!witnesses[0].uncertified_definition_fallback);
}

#[test]
fn duplicate_shifted_halfspace_witnesses_merge_distinct_active_plane_families() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let point = p(1, 1, 1);
    let mut witnesses = vec![ShiftedHalfspaceWitness::with_family(
        point.clone(),
        vec![halfspaces[0].clone()],
        [None, None, None],
        true,
    )];

    push_unique_shifted_halfspace_witness(
        &mut witnesses,
        ShiftedHalfspaceWitness::with_family(
            point,
            halfspaces.clone(),
            [Some(0), Some(1), None],
            false,
        ),
    );

    assert_eq!(witnesses.len(), 1);
    assert_eq!(witnesses[0].families.len(), 2);
    assert!(
        witnesses[0]
            .families
            .iter()
            .any(|family| family.active_planes == [None, None, None])
    );
    assert!(witnesses[0].families.iter().any(|family| {
        family.active_planes == [Some(0), Some(1), None] && family.halfspaces == halfspaces
    }));
    assert!(witnesses[0].uncertified_definition_fallback);
}

#[test]
fn duplicate_shifted_halfspace_witnesses_merge_permuted_halfspace_families() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted_halfspaces = halfspaces.clone();
    permuted_halfspaces.swap(0, 1);
    let point = p(1, 1, 1);
    let mut witnesses = vec![ShiftedHalfspaceWitness::with_family(
        point.clone(),
        halfspaces,
        [Some(0), Some(1), None],
        false,
    )];

    push_unique_shifted_halfspace_witness(
        &mut witnesses,
        ShiftedHalfspaceWitness::with_family(
            point,
            permuted_halfspaces,
            [Some(0), Some(1), None],
            false,
        ),
    );

    assert_eq!(witnesses.len(), 1);
    assert_eq!(witnesses[0].families.len(), 1);
}

#[test]
fn duplicate_shifted_halfspace_witnesses_prefer_certified_duplicate_families() {
    let point = p(1, 1, 1);
    let mut witnesses = vec![ShiftedHalfspaceWitness::with_family(
        point.clone(),
        Vec::new(),
        [None, None, None],
        true,
    )];

    push_unique_shifted_halfspace_witness(
        &mut witnesses,
        ShiftedHalfspaceWitness::with_family(point, Vec::new(), [None, None, None], false),
    );

    assert_eq!(witnesses.len(), 1);
    assert!(!witnesses[0].uncertified_definition_fallback);
}

#[test]
fn shifted_halfspace_witness_collection_reports_unknown_if_all_seeds_are_uncertified() {
    let first_seed = p(1, 1, 1);
    let second_seed = p(2, 2, 2);
    let mut witnesses = Vec::new();

    let err = extend_shifted_halfspace_witnesses_backtracking_unknown(
        &mut witnesses,
        vec![first_seed, second_seed],
        |_seed| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn strict_halfspace_cell_seeds_include_direct_strict_feasibility_witness() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let direct = p(2, 1, 3);
    let report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(direct.clone(), [None, None, None]);

    let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    assert!(seeds.iter().any(|seed| seed == &direct));
}

#[test]
fn strict_halfspace_cell_seeds_include_strict_feasible_vertices() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(3)),
        axis_halfspace(2, false, r(3)),
    ];
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(
        Point3::new(r(1), r(2), r(0)),
        [None, None, None],
    );

    let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    assert_eq!(seeds, vec![Point3::new(r(1), r(2), r(3))]);
}

#[test]
fn strict_halfspace_cell_seeds_include_strict_geometry_seeds() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(
        Point3::new(r(0), r(0), r(0)),
        [None, None, None],
    );
    let tetra_center = p(1, 1, 1);

    let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    assert!(seeds.iter().any(|seed| seed == &p(2, 2, 2)));
    assert!(seeds.iter().any(|seed| seed == &tetra_center));
}

#[test]
fn strict_halfspace_cell_seed_collection_backtracks_after_uncertified_candidate() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut seeds = Vec::new();

    extend_strict_halfspace_seeds_backtracking_unknown(
        &mut seeds,
        vec![first.clone(), second.clone()],
        |candidate| {
            if candidate == &first {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(candidate == &second)
            }
        },
    )
    .unwrap();

    assert_eq!(seeds, vec![second]);
}

#[test]
fn strict_halfspace_cell_seed_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut seeds = Vec::new();

    let err = extend_strict_halfspace_seeds_backtracking_unknown(
        &mut seeds,
        vec![first, second],
        |_candidate| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn strict_halfspace_cell_seed_family_search_backtracks_after_uncertified_earlier_family() {
    let mut seeds = Vec::new();

    extend_strict_halfspace_seed_families_backtracking_unknown(
        &mut seeds,
        [
            Err(HypermeshError::UnknownClassification),
            Ok(HalfspaceSeedFamilyState {
                seeds: vec![p(2, 2, 2)],
                saw_unknown: false,
            }),
        ],
    )
    .unwrap();

    assert_eq!(seeds, vec![p(2, 2, 2)]);
}

#[test]
fn strict_halfspace_cell_seed_family_search_tracks_unknown_after_uncertain_family_result() {
    let mut seeds = Vec::new();

    let saw_unknown = extend_strict_halfspace_seed_families_collect_unknown(
        &mut seeds,
        [
            Ok(HalfspaceSeedFamilyState {
                seeds: vec![p(1, 1, 1)],
                saw_unknown: true,
            }),
            Ok(HalfspaceSeedFamilyState {
                seeds: vec![p(2, 2, 2)],
                saw_unknown: false,
            }),
        ],
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(seeds, vec![p(1, 1, 1), p(2, 2, 2)]);
}

#[test]
fn strict_halfspace_cell_seed_family_search_reports_unknown_if_all_families_are_uncertified() {
    let mut seeds = Vec::new();

    let err = extend_strict_halfspace_seed_families_backtracking_unknown(
        &mut seeds,
        [
            Err(HypermeshError::UnknownClassification),
            Err(HypermeshError::UnknownClassification),
        ],
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn collect_strict_halfspace_seed_family_tracks_unknown_after_later_strict_seed() {
    let family =
        collect_strict_halfspace_seed_family(Ok(vec![p(1, 1, 1), p(2, 2, 2)]), |candidate| {
            if *candidate == p(1, 1, 1) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(true)
            }
        })
        .unwrap();

    assert_eq!(family.seeds, vec![p(2, 2, 2)]);
    assert!(family.saw_unknown);
}

#[test]
fn collect_strict_halfspace_seed_family_tracks_unknown_after_halfspace_boundary_candidate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let family =
        collect_strict_halfspace_seed_family(Ok(vec![p(0, 2, 2), p(1, 1, 1)]), |candidate| {
            point_strictly_inside_halfspace_cell_or_unknown(candidate, &bounds, &halfspaces)
        })
        .unwrap();

    assert_eq!(family.seeds, vec![p(1, 1, 1)]);
    assert!(family.saw_unknown);
}

#[test]
fn collect_strict_halfspace_seed_family_tracks_unknown_after_leaf_boundary_candidate() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);

    let family =
        collect_strict_halfspace_seed_family(Ok(vec![p(3, 0, 0), p(1, 1, 1)]), |candidate| {
            point_strictly_inside_leaf_or_unknown(candidate, &leaf)
        })
        .unwrap();

    assert_eq!(family.seeds, vec![p(1, 1, 1)]);
    assert!(family.saw_unknown);
}

#[test]
fn seed_family_search_failure_allows_later_shifted_seeds_after_unknown_strict_family() {
    assert!(!seed_family_search_failed_without_any_seed(
        &[],
        &[p(1, 1, 1)],
        &[],
        true,
    ));
    assert!(!seed_family_search_failed_without_any_seed(
        &[],
        &[],
        &[p(2, 2, 2)],
        true,
    ));
}

#[test]
fn seed_family_search_failure_reports_unknown_only_when_every_seed_family_is_empty() {
    assert!(seed_family_search_failed_without_any_seed(
        &[],
        &[],
        &[],
        true,
    ));
    assert!(!seed_family_search_failed_without_any_seed(
        &[p(3, 3, 3)],
        &[],
        &[],
        true,
    ));
}

#[test]
fn take_new_halfspace_seed_family_preserves_first_occurrence_order() {
    let mut seen = vec![p(0, 0, 0)];
    let fresh = take_new_halfspace_seed_family(
        vec![p(1, 1, 1), p(0, 0, 0), p(2, 2, 2), p(1, 1, 1)],
        &mut seen,
    );

    assert_eq!(fresh, vec![p(1, 1, 1), p(2, 2, 2)]);
    assert_eq!(seen, vec![p(0, 0, 0), p(1, 1, 1), p(2, 2, 2)]);
}

#[test]
fn shifted_halfspace_seed_families_with_report_seed_promote_report_witness_to_shifted_root() {
    let witness = p(1, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            Some(&witness),
            vec![p(2, 1, 1)],
            vec![p(2, 1, 1), witness.clone(), p(3, 1, 1)],
            vec![p(3, 1, 1), witness.clone(), p(4, 1, 1)],
        );

    assert_eq!(strict_seeds, vec![p(2, 1, 1), witness]);
    assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
    assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
}

#[test]
fn shifted_halfspace_seed_families_with_report_seed_skip_later_duplicates() {
    let witness = p(1, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_halfspace_seed_families_with_report_seed(
            Some(&witness),
            vec![witness.clone(), p(2, 1, 1)],
            vec![p(2, 1, 1), witness.clone(), p(3, 1, 1)],
            vec![p(3, 1, 1), witness.clone(), p(4, 1, 1)],
        );

    assert_eq!(strict_seeds, vec![witness, p(2, 1, 1)]);
    assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
    assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
}

#[test]
fn detour_shifted_seed_families_keep_shifted_sources_with_direct_targets() {
    let witness = p(1, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) = detour_shifted_seed_families(
        Some(&witness),
        &[p(9, 9, 9)],
        vec![p(2, 1, 1)],
        vec![p(2, 1, 1), witness.clone(), p(3, 1, 1)],
        vec![p(3, 1, 1), witness.clone(), p(4, 1, 1)],
    );

    assert_eq!(strict_seeds, vec![p(2, 1, 1), witness]);
    assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
    assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
}

#[test]
fn shifted_halfspace_witness_seed_family_search_skips_duplicate_seed_sources() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut witnesses = Vec::new();
    let visited = std::cell::RefCell::new(Vec::new());

    extend_shifted_halfspace_seed_families_backtracking_unknown(
        &mut witnesses,
        [
            vec![first.clone(), second.clone()],
            vec![second.clone(), first.clone()],
        ],
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                seed.clone(),
                Vec::new(),
                [None, None, None],
                false,
            )])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![first.clone(), second.clone()]);
    assert_eq!(
        witnesses
            .into_iter()
            .map(|witness| witness.point)
            .collect::<Vec<_>>(),
        vec![first, second]
    );
}

#[test]
fn shifted_halfspace_witness_seed_family_search_backtracks_after_uncertified_earlier_family() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut witnesses = Vec::new();

    extend_shifted_halfspace_seed_families_backtracking_unknown(
        &mut witnesses,
        [vec![first.clone()], vec![first, second.clone()]],
        |seed| {
            if *seed == p(2, 2, 2) {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    seed.clone(),
                    Vec::new(),
                    [None, None, None],
                    false,
                )])
            } else {
                Err(HypermeshError::UnknownClassification)
            }
        },
    )
    .unwrap();

    assert_eq!(
        witnesses
            .into_iter()
            .map(|witness| witness.point)
            .collect::<Vec<_>>(),
        vec![second]
    );
}

#[test]
fn shifted_halfspace_witness_seed_family_search_marks_existing_witnesses_uncertain_after_later_unknown()
 {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut witnesses = Vec::new();

    extend_shifted_halfspace_seed_families_backtracking_unknown(
        &mut witnesses,
        [vec![first.clone()], vec![second.clone()]],
        |seed| {
            if *seed == first {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    seed.clone(),
                    Vec::new(),
                    [None, None, None],
                    false,
                )])
            } else {
                Err(HypermeshError::UnknownClassification)
            }
        },
    )
    .unwrap();

    assert_eq!(witnesses.len(), 1);
    assert_eq!(witnesses[0].point, first);
    assert!(witnesses[0].uncertified_definition_fallback);
}

#[test]
fn shifted_halfspace_witness_seed_family_search_marks_later_witnesses_uncertain_after_uncertain_family_result()
 {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut witnesses = Vec::new();

    extend_shifted_halfspace_seed_families_backtracking_unknown(
        &mut witnesses,
        [vec![first.clone()], vec![second.clone()]],
        |seed| {
            if *seed == first {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    seed.clone(),
                    Vec::new(),
                    [None, None, None],
                    true,
                )])
            } else {
                Ok(vec![ShiftedHalfspaceWitness::with_family(
                    seed.clone(),
                    Vec::new(),
                    [None, None, None],
                    false,
                )])
            }
        },
    )
    .unwrap();

    assert_eq!(witnesses.len(), 2);
    assert!(
        witnesses
            .iter()
            .all(|witness| witness.uncertified_definition_fallback)
    );
}

#[test]
fn shifted_halfspace_witness_seed_family_search_keeps_certified_duplicate_state_certified() {
    let witness_point = p(3, 3, 3);
    let mut witnesses = Vec::new();

    extend_shifted_halfspace_seed_families_backtracking_unknown(
        &mut witnesses,
        [vec![p(1, 1, 1)], vec![p(2, 2, 2)]],
        |seed| {
            Ok(vec![ShiftedHalfspaceWitness::with_family(
                witness_point.clone(),
                Vec::new(),
                [None, None, None],
                *seed == p(1, 1, 1),
            )])
        },
    )
    .unwrap();

    assert_eq!(witnesses.len(), 1);
    assert_eq!(witnesses[0].point, witness_point);
    assert!(!witnesses[0].uncertified_definition_fallback);
}

#[test]
fn strict_halfspace_cell_seeds_include_strict_geometry_seeds_without_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let triangle_center = centroid(&[p(0, 0, 0), p(4, 0, 0), p(4, 4, 4)])
        .unwrap()
        .unwrap();
    let tetra_center = p(1, 1, 1);

    let seeds =
        strict_halfspace_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(point_strictly_inside_halfspace_cell(&triangle_center, &bounds, &halfspaces).unwrap());
    assert!(point_strictly_inside_halfspace_cell(&tetra_center, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &triangle_center));
    assert!(seeds.iter().any(|seed| seed == &tetra_center));
}

#[test]
fn halfspace_cell_seed_families_track_unknown_after_boundary_vertex_candidate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut saw_unknown = false;

    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            &bounds,
            &halfspaces,
            None,
            &mut saw_unknown,
        )
        .unwrap();

    assert!(saw_unknown);
    assert!(!strict_seeds.is_empty());
    assert!(!shifted_vertices.is_empty());
    assert!(!shifted_geometry_seeds.is_empty());
}

#[test]
fn strict_halfspace_cell_seeds_include_strict_edge_midpoints() {
    let (bounds, halfspaces, midpoint) = quadrilateral_halfspace_cell_fixture();

    let seeds =
        strict_halfspace_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(point_strictly_inside_halfspace_cell(&midpoint, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &midpoint));
}

#[test]
fn strict_halfspace_cell_seeds_include_strict_five_vertex_centroids() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let five_vertex_center = Point3::new(q(8, 5), q(8, 5), q(8, 5));

    let seeds =
        strict_halfspace_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(
        point_strictly_inside_halfspace_cell(&five_vertex_center, &bounds, &halfspaces).unwrap()
    );
    assert!(seeds.iter().any(|seed| seed == &five_vertex_center));
}

#[test]
fn shifted_halfspace_witnesses_mark_survivors_uncertain_after_boundary_seed_candidate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let witnesses =
        shifted_halfspace_cell_witnesses_from_seed(&bounds, &halfspaces, &p(1, 1, 1)).unwrap();

    assert!(!witnesses.is_empty());
    assert!(
        witnesses
            .iter()
            .all(|witness| witness.uncertified_definition_fallback)
    );
}

#[test]
fn strict_leaf_witness_seeds_include_strict_halfspace_triangle_centroid() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let vertices = leaf.vertices().unwrap();
    let bounds = leaf_bounds(&vertices).unwrap();
    let halfspaces = leaf_halfspaces(&leaf);
    let report = halfspace_feasibility_report(&halfspaces).unwrap();
    let center = centroid(&feasible_halfspace_cell_vertices(&halfspaces).unwrap())
        .unwrap()
        .unwrap();

    let seeds =
        strict_leaf_witness_seeds(&leaf, &vertices, &bounds, &halfspaces, Some(&report)).unwrap();

    assert!(point_strictly_inside_leaf(&center, &leaf).unwrap());
    assert!(seeds.iter().any(|seed| seed == &center));
}

#[test]
fn strict_leaf_witness_seeds_include_strict_halfspace_geometry_family() {
    let leaf = make_quad(&p(0, 0, 0), &p(4, 0, 0), &p(4, 4, 0), &p(0, 4, 0), 0, 0);
    let vertices = leaf.vertices().unwrap();
    let bounds = leaf_bounds(&vertices).unwrap();
    let halfspaces = leaf_halfspaces(&leaf);
    let report = halfspace_feasibility_report(&halfspaces).unwrap();
    let triangle_center = centroid(&[p(0, 0, 0), p(4, 0, 0), p(4, 4, 0)])
        .unwrap()
        .unwrap();

    let seeds =
        strict_leaf_witness_seeds(&leaf, &vertices, &bounds, &halfspaces, Some(&report)).unwrap();

    assert!(point_strictly_inside_leaf(&triangle_center, &leaf).unwrap());
    assert!(seeds.iter().any(|seed| seed == &triangle_center));
}

#[test]
fn shifted_edge_interior_points_move_vertices_inside_by_certified_margins() {
    let leaf = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
    let vertices = leaf.vertices().unwrap();
    let center = centroid(&vertices).unwrap().unwrap();
    let points = shifted_edge_interior_points(&leaf, &center).unwrap();

    assert_eq!(points.len(), 3);
    for point in &points {
        assert!(point_strictly_inside_leaf(&point.point, &leaf).unwrap());
    }

    let first = &points[0].point;
    let expected_first_edge_margin =
        (leaf.edges[0].expression_at_point(&center) / Real::from(2)).unwrap();
    let expected_second_edge_margin =
        (leaf.edges[1].expression_at_point(&center) / Real::from(2)).unwrap();

    assert_eq!(
        leaf.edges[0].expression_at_point(first),
        expected_first_edge_margin
    );
    assert_eq!(
        leaf.edges[1].expression_at_point(first),
        expected_second_edge_margin
    );
}

#[test]
fn bounded_probes_include_certified_normal_direction_probe() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let vertices = leaf.vertices().unwrap();
    let center = centroid(&vertices).unwrap().unwrap();
    let interior = shifted_edge_interior_points(&leaf, &center)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("shifted edge construction should retain defining planes");

    let probes =
        bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[]).unwrap();

    let probe = probes
        .iter()
        .find(|probe| probe.side == Classification::Positive && !probe.planes.is_empty())
        .expect("normal probe should preserve a shifted plane definition");
    let planes = &probe.planes[0];
    assert_eq!(affine_from_planes(planes).unwrap(), probe.point);
}

#[test]
fn bounded_probes_find_positive_probe_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let interior_points = certified_leaf_interior_points(&wall.support, &wall.edges).unwrap();

    assert!(!interior_points.is_empty());
    assert!(interior_points.iter().any(|point| !point.planes.is_empty()));

    let probes = bounded_probes_from_interior(
        &interior_points[0],
        &wall.support,
        &bounds,
        true,
        &[wall.clone()],
    )
    .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
}

#[test]
fn bounded_probes_keep_positive_probe_before_intervening_surface() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let mut blocker = make_triangle(&p(2, -10, -10), &p(2, 10, -10), &p(2, 0, 10), 1, 0);
    blocker.delta_w = vec![1];
    let bounds = Aabb::new(p(1, -2, -2), p(5, 2, 2));
    let interior_points = certified_leaf_interior_points(&wall.support, &wall.edges).unwrap();

    let probes = bounded_probes_from_interior(
        &interior_points[0],
        &wall.support,
        &bounds,
        true,
        &[wall.clone(), blocker],
    )
    .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
}

#[test]
fn positive_probe_traces_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let ref_point = p(0, 0, 0);
    let ref_definitions = vec![axis_plane_definition(&ref_point)];
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    assert!(!interior.uncertified_definition_fallback);
    let probe =
        bounded_probes_from_interior(&interior, &wall.support, &bounds, true, &[wall.clone()])
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("leaf should have a positive-side probe");
    assert!(probe.uncertified_definition_fallback);

    assert!(!point_lies_on_traced_surface(&probe.point, &[wall.clone()]).unwrap());
    assert!(
        probe_reaches_adjacent_cell_from_interior(
            &interior,
            &probe,
            &wall.support,
            &[wall.clone()],
        )
        .unwrap()
    );

    let winding =
        trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall.clone()]).unwrap();

    assert_eq!(winding.len(), 1);
}

#[test]
fn trace_probe_winding_with_query_caches_reuses_lower_trace_state_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let ref_point = p(0, 0, 0);
    let ref_definitions = vec![axis_plane_definition(&ref_point)];
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let probe =
        bounded_probes_from_interior(&interior, &wall.support, &bounds, true, &[wall.clone()])
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("leaf should have a positive-side probe");
    let mut query_caches = LeafProbeQueryCaches::default();
    let LeafProbeQueryCaches {
        probe_surface,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        definition_no_detour_trace,
        detour_target_families,
        ..
    } = &mut query_caches;

    let first = trace_probe_winding_with_caches(
        &ref_point,
        &ref_definitions,
        &probe,
        &[0],
        &[wall.clone()],
        probe_surface,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        definition_no_detour_trace,
        detour_target_families,
        Some(&bounds),
    )
    .unwrap();
    let after_first = (
        probe_surface.len(),
        axis_ordered_segment_traces.len(),
        plane_replacement_affine.len(),
        plane_replacement_trace_steps.len(),
        definition_no_detour_trace.len(),
        detour_target_families.len(),
    );

    let second = trace_probe_winding_with_caches(
        &ref_point,
        &ref_definitions,
        &probe,
        &[0],
        &[wall],
        probe_surface,
        axis_ordered_segment_traces,
        plane_replacement_affine,
        plane_replacement_trace_steps,
        definition_no_detour_trace,
        detour_target_families,
        Some(&bounds),
    )
    .unwrap();
    let after_second = (
        probe_surface.len(),
        axis_ordered_segment_traces.len(),
        plane_replacement_affine.len(),
        plane_replacement_trace_steps.len(),
        definition_no_detour_trace.len(),
        detour_target_families.len(),
    );

    assert_eq!(first, second);
    assert_eq!(after_first, after_second);
}

#[test]
fn probe_reachability_with_query_caches_reuses_lower_trace_state_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let probe = bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
        .unwrap()
        .into_iter()
        .find(|probe| probe.side == Classification::Positive)
        .expect("leaf should have a positive-side probe");
    let mut query_caches = LeafProbeQueryCaches::default();
    let LeafProbeQueryCaches {
        probe_surface,
        plane_replacement_affine,
        plane_replacement_reachability_paths,
        plane_replacement_reachability_steps,
        plane_replacement_no_nested_ordering_warmups,
        interior_box_axis_intervals,
        definition_cycle_guard_reachability,
        definition_no_step_detour_reachability,
        definition_no_plane_replacement_cycle_guard,
        definition_no_plane_replacement_reachability,
        halfspace_reports,
        halfspace_seed_families,
        no_step_detour_target_families,
        definition_full_no_detour_reachability,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
        ..
    } = &mut query_caches;

    let first = probe_reaches_adjacent_cell_from_interior_with_caches(
        &interior,
        &probe,
        &support,
        &[wall.clone()],
        probe_surface,
        plane_replacement_affine,
        plane_replacement_reachability_paths,
        plane_replacement_reachability_steps,
        plane_replacement_no_nested_ordering_warmups,
        interior_box_axis_intervals,
        definition_cycle_guard_reachability,
        definition_no_step_detour_reachability,
        definition_no_plane_replacement_cycle_guard,
        definition_no_plane_replacement_reachability,
        halfspace_reports,
        halfspace_seed_families,
        no_step_detour_target_families,
        definition_full_no_detour_reachability,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
        Some(&bounds),
    )
    .unwrap();
    let after_first = [
        probe_surface.len(),
        plane_replacement_affine.len(),
        plane_replacement_reachability_paths.len(),
        plane_replacement_reachability_steps.len(),
        plane_replacement_no_nested_ordering_warmups.len(),
        interior_box_axis_intervals.len(),
        definition_no_step_detour_reachability.len(),
        definition_no_plane_replacement_reachability.len(),
        no_step_detour_target_families.len(),
        definition_full_no_detour_reachability.len(),
        definition_no_detour_reachability.len(),
        direct_probe_reachability.len(),
        detour_target_families.len(),
    ];

    let second = probe_reaches_adjacent_cell_from_interior_with_caches(
        &interior,
        &probe,
        &support,
        &[wall],
        probe_surface,
        plane_replacement_affine,
        plane_replacement_reachability_paths,
        plane_replacement_reachability_steps,
        plane_replacement_no_nested_ordering_warmups,
        interior_box_axis_intervals,
        definition_cycle_guard_reachability,
        definition_no_step_detour_reachability,
        definition_no_plane_replacement_cycle_guard,
        definition_no_plane_replacement_reachability,
        halfspace_reports,
        halfspace_seed_families,
        no_step_detour_target_families,
        definition_full_no_detour_reachability,
        definition_no_detour_reachability,
        direct_probe_reachability,
        detour_target_families,
        Some(&bounds),
    )
    .unwrap();
    let after_second = [
        probe_surface.len(),
        plane_replacement_affine.len(),
        plane_replacement_reachability_paths.len(),
        plane_replacement_reachability_steps.len(),
        plane_replacement_no_nested_ordering_warmups.len(),
        interior_box_axis_intervals.len(),
        definition_no_step_detour_reachability.len(),
        definition_no_plane_replacement_reachability.len(),
        no_step_detour_target_families.len(),
        definition_full_no_detour_reachability.len(),
        definition_no_detour_reachability.len(),
        direct_probe_reachability.len(),
        detour_target_families.len(),
    ];

    assert_eq!(first, second);
    assert_eq!(after_first, after_second);
}

#[test]
fn no_step_definition_search_caches_whole_query_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let probe = bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
        .unwrap()
        .into_iter()
        .find(|probe| probe.side == Classification::Positive)
        .expect("leaf should have a positive-side probe");
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut path_cache = PlaneReplacementReachabilityPathCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut no_step_cache = DefinitionNoDetourReachabilityCache::default();
    let mut direct_probe_reachability_cache = Vec::new();

    let first = probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
        &interior.point,
        &probe.point,
        &support,
        &[wall.clone()],
        &interior.planes,
        &probe.planes,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_step_cache,
        &mut direct_probe_reachability_cache,
        None,
    )
    .unwrap();
    let after_first = (
        affine_cache.len(),
        path_cache.len(),
        step_cache.len(),
        no_step_cache.len(),
        direct_probe_reachability_cache.len(),
    );

    let second = probe_reaches_adjacent_cell_with_definitions_no_step_detours_with_caches(
        &interior.point,
        &probe.point,
        &support,
        &[wall],
        &interior.planes,
        &probe.planes,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_step_cache,
        &mut direct_probe_reachability_cache,
        None,
    )
    .unwrap();
    let after_second = (
        affine_cache.len(),
        path_cache.len(),
        step_cache.len(),
        no_step_cache.len(),
        direct_probe_reachability_cache.len(),
    );

    assert_eq!(first, second);
    assert_eq!(no_step_cache.len(), 1);
    assert_eq!(after_first, after_second);
}

#[test]
fn full_no_detour_definition_search_caches_whole_query_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let probe = bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
        .unwrap()
        .into_iter()
        .find(|probe| probe.side == Classification::Positive)
        .expect("leaf should have a positive-side probe");
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut path_cache = PlaneReplacementReachabilityPathCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut no_nested_ordering_warmup_cache =
        PlaneReplacementNoNestedOrderingWarmupCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut no_step_cache = DefinitionNoDetourReachabilityCache::default();
    let mut halfspace_reports = Vec::new();
    let mut halfspace_seed_families = Vec::new();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut no_step_detour_target_cache = DetourTargetFamilyCache::default();
    let mut full_no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut direct_probe_reachability_cache = Vec::new();

    let first = probe_reaches_adjacent_cell_with_definitions_no_detours_with_caches(
        &interior.point,
        &probe.point,
        &support,
        &[wall.clone()],
        &interior.planes,
        &probe.planes,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_nested_ordering_warmup_cache,
        &mut interior_box_axis_intervals,
        &mut no_step_cache,
        &mut halfspace_reports,
        &mut halfspace_seed_families,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut no_step_detour_target_cache,
        &mut full_no_detour_cache,
        &mut no_detour_cache,
        &mut direct_probe_reachability_cache,
        None,
    )
    .unwrap();
    let after_first = [
        affine_cache.len(),
        path_cache.len(),
        step_cache.len(),
        no_nested_ordering_warmup_cache.len(),
        interior_box_axis_intervals.len(),
        no_step_cache.len(),
        halfspace_reports.len(),
        halfspace_seed_families.len(),
        no_plane_replacement_cycle_guard_cache.len(),
        no_plane_replacement_cache.len(),
        no_step_detour_target_cache.len(),
        full_no_detour_cache.len(),
        no_detour_cache.len(),
        direct_probe_reachability_cache.len(),
    ];

    let second = probe_reaches_adjacent_cell_with_definitions_no_detours_with_caches(
        &interior.point,
        &probe.point,
        &support,
        &[wall],
        &interior.planes,
        &probe.planes,
        &mut affine_cache,
        &mut path_cache,
        &mut step_cache,
        &mut no_nested_ordering_warmup_cache,
        &mut interior_box_axis_intervals,
        &mut no_step_cache,
        &mut halfspace_reports,
        &mut halfspace_seed_families,
        &mut no_plane_replacement_cycle_guard_cache,
        &mut no_plane_replacement_cache,
        &mut no_step_detour_target_cache,
        &mut full_no_detour_cache,
        &mut no_detour_cache,
        &mut direct_probe_reachability_cache,
        None,
    )
    .unwrap();
    let after_second = [
        affine_cache.len(),
        path_cache.len(),
        step_cache.len(),
        no_nested_ordering_warmup_cache.len(),
        interior_box_axis_intervals.len(),
        no_step_cache.len(),
        halfspace_reports.len(),
        halfspace_seed_families.len(),
        no_plane_replacement_cycle_guard_cache.len(),
        no_plane_replacement_cache.len(),
        no_step_detour_target_cache.len(),
        full_no_detour_cache.len(),
        no_detour_cache.len(),
        direct_probe_reachability_cache.len(),
    ];

    assert_eq!(first, second);
    assert_eq!(full_no_detour_cache.len(), 1);
    assert_eq!(after_first, after_second);
}

#[test]
fn interior_box_axis_intervals_cache_reuses_core_leaf_wall_case_query() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let probe = bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
        .unwrap()
        .into_iter()
        .find(|probe| probe.side == Classification::Positive)
        .expect("leaf should have a positive-side probe");
    let mut interval_cache = InteriorBoxAxisIntervalsCache::default();

    let first = cached_interior_box_axis_intervals_with_surface_queries(
        &mut interval_cache,
        &interior.point,
        &probe.point,
        || {
            interior_box_axis_intervals_with_surface_queries(
                &interior.point,
                &probe.point,
                &[wall.clone()],
                &mut |edge_start, edge_end, polygon, axis| {
                    let start_class = classify_point(edge_start, &polygon.support)?;
                    let end_class = classify_point(edge_end, &polygon.support)?;
                    if start_class == Classification::On {
                        return Ok(Some(edge_start.clone()));
                    }
                    if end_class == Classification::On {
                        return Ok(Some(edge_end.clone()));
                    }
                    segment_plane_crossing(edge_start, edge_end, &polygon.support).and_then(
                        |crossing| {
                            if let Some(crossing) = crossing {
                                if !point_strictly_between_axis(
                                    &crossing, edge_start, edge_end, axis,
                                )? {
                                    return Ok(None);
                                }
                                Ok(Some(crossing))
                            } else {
                                Ok(None)
                            }
                        },
                    )
                },
                &mut |crossing, polygon| classify_point_in_polygon(crossing, polygon),
            )
        },
    )
    .unwrap();
    let after_first = interval_cache.len();

    let second = cached_interior_box_axis_intervals_with_surface_queries(
        &mut interval_cache,
        &interior.point,
        &probe.point,
        || {
            interior_box_axis_intervals_with_surface_queries(
                &interior.point,
                &probe.point,
                &[wall],
                &mut |edge_start, edge_end, polygon, axis| {
                    let start_class = classify_point(edge_start, &polygon.support)?;
                    let end_class = classify_point(edge_end, &polygon.support)?;
                    if start_class == Classification::On {
                        return Ok(Some(edge_start.clone()));
                    }
                    if end_class == Classification::On {
                        return Ok(Some(edge_end.clone()));
                    }
                    segment_plane_crossing(edge_start, edge_end, &polygon.support).and_then(
                        |crossing| {
                            if let Some(crossing) = crossing {
                                if !point_strictly_between_axis(
                                    &crossing, edge_start, edge_end, axis,
                                )? {
                                    return Ok(None);
                                }
                                Ok(Some(crossing))
                            } else {
                                Ok(None)
                            }
                        },
                    )
                },
                &mut |crossing, polygon| classify_point_in_polygon(crossing, polygon),
            )
        },
    )
    .unwrap();
    let after_second = interval_cache.len();

    assert_eq!(first, second);
    assert_eq!(after_first, 1);
    assert_eq!(after_first, after_second);
}

#[test]
fn strict_aabb_target_family_cache_reuses_core_leaf_wall_case_query() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let probe = bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
        .unwrap()
        .into_iter()
        .find(|probe| probe.side == Classification::Positive)
        .expect("leaf should have a positive-side probe");
    let strict_bounds = bounds_between_points(&interior.point, &probe.point).unwrap();
    let mut target_family_cache = StrictAabbTargetFamilyCache::default();
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();

    let first = cached_strict_aabb_target_families_with_seed_families(
        &mut target_family_cache,
        &strict_bounds,
        |bounds, halfspaces, report, local_unknown| {
            let report = match report {
                Some(report) => Some(report.clone()),
                None => cached_optional_halfspace_feasibility_report_with(
                    &mut halfspace_report_cache,
                    halfspaces,
                    local_unknown,
                )?,
            };
            cached_halfspace_cell_seed_families_from_optional_report_with(
                &mut halfspace_seed_family_cache,
                bounds,
                halfspaces,
                report.as_ref(),
                local_unknown,
            )
        },
    )
    .unwrap();
    let after_first = target_family_cache.len();

    let second = cached_strict_aabb_target_families_with_seed_families(
        &mut target_family_cache,
        &strict_bounds,
        |bounds, halfspaces, report, local_unknown| {
            let report = match report {
                Some(report) => Some(report.clone()),
                None => cached_optional_halfspace_feasibility_report_with(
                    &mut halfspace_report_cache,
                    halfspaces,
                    local_unknown,
                )?,
            };
            cached_halfspace_cell_seed_families_from_optional_report_with(
                &mut halfspace_seed_family_cache,
                bounds,
                halfspaces,
                report.as_ref(),
                local_unknown,
            )
        },
    )
    .unwrap();
    let after_second = target_family_cache.len();

    assert_eq!(first, second);
    assert_eq!(after_first, 1);
    assert_eq!(after_first, after_second);
}

#[test]
fn adjacent_normal_probes_preserve_family_uncertainty_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");

    let probes = adjacent_normal_probes(&interior, &support, &bounds, &[wall], true).unwrap();

    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn strict_normal_probe_targets_preserve_family_uncertainty_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let existing_probe =
        bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("leaf should have a positive-side probe");
    let corridor = bounds_between_points(&interior.point, &existing_probe.point).unwrap();

    let probes = strict_normal_probe_targets(
        &interior,
        &support,
        &corridor,
        Some(&interior.planes[0]),
        &existing_probe.point,
        true,
    )
    .unwrap();

    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn strict_normal_probe_direct_seed_phase_stays_certified_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let existing_probe =
        bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("leaf should have a positive-side probe");
    let corridor = bounds_between_points(&interior.point, &existing_probe.point).unwrap();

    let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
    halfspaces.push(support_side_halfspace(&support, true));
    halfspaces.push(normal_stop_halfspace(&support, &existing_probe.point, true));
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
    assert!(!saw_unknown);
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            &corridor,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )
        .unwrap();

    let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
        &interior,
        &support,
        &corridor,
        Some(&interior.planes[0]),
        &existing_probe.point,
        true,
        report.as_ref(),
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        |_seed| Ok(Vec::new()),
    )
    .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| !probe.uncertified_definition_fallback)
    );
}

#[test]
fn direct_normal_probe_seed_build_stays_certified_for_core_leaf_wall_case() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let support = wall.support.clone();
    let interior = certified_leaf_interior_points(&wall.support, &wall.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("leaf should have a replayable interior witness");
    let existing_probe =
        bounded_probes_from_interior(&interior, &support, &bounds, true, &[wall.clone()])
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("leaf should have a positive-side probe");
    let corridor = bounds_between_points(&interior.point, &existing_probe.point).unwrap();

    let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
    halfspaces.push(support_side_halfspace(&support, true));
    halfspaces.push(normal_stop_halfspace(&support, &existing_probe.point, true));
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
    assert!(!saw_unknown);
    let (seeds, _shifted_vertices, _shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            &corridor,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )
        .unwrap();

    let mut extra_planes = Vec::new();
    for definition in &interior.planes {
        for plane in &definition[1..] {
            if !extra_planes.iter().any(|existing| existing == plane) {
                extra_planes.push(plane.clone());
            }
        }
    }

    let built = seeds
        .iter()
        .filter_map(|seed| {
            build_probe_point(
                seed,
                &corridor,
                &support,
                &halfspaces,
                active_planes_from_optional_report(report.as_ref(), seed),
                &extra_planes,
                false,
            )
            .unwrap()
        })
        .collect::<Vec<_>>();

    assert!(!built.is_empty());
    assert!(
        built
            .iter()
            .any(|probe| !probe.uncertified_definition_fallback)
    );
}

#[test]
fn bounded_probe_family_collection_backtracks_after_uncertified_family() {
    let constrained_probe = ProbePoint {
        point: p(1, 1, 1),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let mut probes = Vec::new();
    let mut saw_unknown = false;

    extend_probe_families_backtracking_unknown(
        &mut probes,
        Err(HypermeshError::UnknownClassification),
        &mut saw_unknown,
    )
    .unwrap();
    extend_probe_families_backtracking_unknown(
        &mut probes,
        Ok(vec![constrained_probe.clone()]),
        &mut saw_unknown,
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, constrained_probe.point);
}

#[test]
fn bounded_probe_family_collection_reports_unknown_if_all_families_are_uncertified() {
    let mut probes = Vec::new();
    let mut saw_unknown = false;

    extend_probe_families_backtracking_unknown(
        &mut probes,
        Err(HypermeshError::UnknownClassification),
        &mut saw_unknown,
    )
    .unwrap();
    extend_probe_families_backtracking_unknown(
        &mut probes,
        Err(HypermeshError::UnknownClassification),
        &mut saw_unknown,
    )
    .unwrap();

    assert!(saw_unknown);
    assert!(probes.is_empty());
}

#[test]
fn bounded_probe_family_collection_tracks_unknown_after_uncertain_family_result() {
    let uncertain_probe = ProbePoint {
        point: p(1, 1, 1),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: true,
    };
    let certain_probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };
    let mut probes = Vec::new();
    let mut saw_unknown = false;

    extend_probe_families_backtracking_unknown(
        &mut probes,
        Ok(vec![uncertain_probe]),
        &mut saw_unknown,
    )
    .unwrap();
    extend_probe_families_backtracking_unknown(
        &mut probes,
        Ok(vec![certain_probe]),
        &mut saw_unknown,
    )
    .unwrap();

    let merged_unknown = saw_unknown
        || probes
            .iter()
            .any(|probe| probe.uncertified_definition_fallback);
    assert!(merged_unknown);
    assert_eq!(probes.len(), 2);
}

#[test]
fn bounded_probe_family_collection_keeps_certified_duplicate_state_certified() {
    let point = p(1, 1, 1);
    let mut probes = Vec::new();
    let mut saw_unknown = false;

    extend_probe_families_backtracking_unknown(
        &mut probes,
        Ok(vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&point)],
            uncertified_definition_fallback: true,
        }]),
        &mut saw_unknown,
    )
    .unwrap();
    extend_probe_families_backtracking_unknown(
        &mut probes,
        Ok(vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&point)],
            uncertified_definition_fallback: false,
        }]),
        &mut saw_unknown,
    )
    .unwrap();

    let merged_unknown = saw_unknown
        || probes
            .iter()
            .any(|probe| probe.uncertified_definition_fallback);
    assert!(!merged_unknown);
    assert_eq!(probes.len(), 1);
}

#[test]
fn leaf_probe_family_search_backtracks_after_uncertified_probe_family() {
    let first = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let second = InteriorLeafPoint {
        point: p(2, 2, 2),
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };
    let winning_probe = ProbePoint {
        point: p(3, 3, 3),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(3, 3, 3))],
        uncertified_definition_fallback: false,
    };

    let winding = search_leaf_probe_families(
        &[first.clone(), second.clone()],
        |point, _positive_side| {
            if point.point == first.point {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(vec![winning_probe.clone()])
            }
        },
        |point, _positive_side, _probe| {
            if point.point == second.point {
                Ok(Some(vec![1]))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(winding, Some(vec![1]));
}

#[test]
fn leaf_probe_family_search_backtracks_after_uncertified_probe_check() {
    let first = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let second = InteriorLeafPoint {
        point: p(2, 2, 2),
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };
    let probe = ProbePoint {
        point: p(3, 3, 3),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(3, 3, 3))],
        uncertified_definition_fallback: false,
    };

    let winding = search_leaf_probe_families(
        &[first.clone(), second.clone()],
        |_point, _positive_side| Ok(vec![probe.clone()]),
        |point, _positive_side, _probe| {
            if point.point == first.point {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(vec![2]))
            }
        },
    )
    .unwrap();

    assert_eq!(winding, Some(vec![2]));
}

#[test]
fn leaf_probe_family_search_reports_unknown_if_all_families_are_uncertified() {
    let point = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };

    let err = search_leaf_probe_families(
        &[point],
        |_point, _positive_side| Err(HypermeshError::UnknownClassification),
        |_point, _positive_side, _probe| Ok(Some(vec![1])),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn leaf_probe_family_search_reports_unknown_when_fallback_probe_is_rejected() {
    let point = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: true,
    };

    let err = search_leaf_probe_families(
        &[point],
        |_point, _positive_side| Ok(vec![probe.clone()]),
        |_point, _positive_side, _probe| Ok(None),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn leaf_probe_family_search_reports_unknown_when_fallback_interior_has_no_probes() {
    let point = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: true,
    };

    let err = search_leaf_probe_families(
        &[point],
        |_point, _positive_side| Ok(Vec::new()),
        |_point, _positive_side, _probe| Ok(Some(vec![1])),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn leaf_probe_family_search_accepts_fallback_probe_after_complete_proof() {
    let point = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let fallback_probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: true,
    };
    let certified_probe = ProbePoint {
        point: p(3, 3, 3),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(3, 3, 3))],
        uncertified_definition_fallback: false,
    };

    let winding = search_leaf_probe_families(
        &[point],
        |_point, positive_side| {
            if positive_side {
                Ok(vec![fallback_probe.clone(), certified_probe.clone()])
            } else {
                Ok(Vec::new())
            }
        },
        |_point, _positive_side, probe| {
            if probe.point == fallback_probe.point {
                Ok(Some(vec![11]))
            } else {
                Ok(Some(vec![13]))
            }
        },
    )
    .unwrap();

    assert_eq!(winding, Some(vec![11]));
}

#[test]
fn leaf_probe_family_search_accepts_only_fallback_probe_after_complete_proof() {
    let point = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let fallback_probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: true,
    };

    let winding = search_leaf_probe_families(
        &[point],
        |_point, _positive_side| Ok(vec![fallback_probe.clone()]),
        |_point, _positive_side, _probe| Ok(Some(vec![11])),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![11]));
}

#[test]
fn leaf_probe_family_search_accepts_fallback_interior_after_complete_proof() {
    let fallback_point = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: true,
    };
    let certified_point = InteriorLeafPoint {
        point: p(2, 2, 2),
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };
    let probe = ProbePoint {
        point: p(3, 3, 3),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(3, 3, 3))],
        uncertified_definition_fallback: false,
    };

    let winding = search_leaf_probe_families(
        &[fallback_point.clone(), certified_point.clone()],
        |_point, _positive_side| Ok(vec![probe.clone()]),
        |point, _positive_side, _probe| {
            if point.point == fallback_point.point {
                Ok(Some(vec![17]))
            } else {
                Ok(Some(vec![19]))
            }
        },
    )
    .unwrap();

    assert_eq!(winding, Some(vec![17]));
}

#[test]
fn leaf_probe_family_search_accepts_only_fallback_interior_after_complete_proof() {
    let fallback_point = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: true,
    };
    let probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };

    let winding = search_leaf_probe_families(
        &[fallback_point],
        |_point, _positive_side| Ok(vec![probe.clone()]),
        |_point, _positive_side, _probe| Ok(Some(vec![17])),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![17]));
}

#[test]
fn cached_probe_winding_reuses_equivalent_trace_across_probe_sides() {
    let definition = axis_plane_definition(&p(1, 2, 3));
    let positive = ProbePoint {
        point: p(1, 2, 3),
        side: Classification::Positive,
        planes: vec![definition.clone()],
        uncertified_definition_fallback: false,
    };
    let negative = ProbePoint {
        point: p(1, 2, 3),
        side: Classification::Negative,
        planes: vec![definition],
        uncertified_definition_fallback: false,
    };
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_probe_winding_with(&mut cache, &positive, || {
        calls += 1;
        Ok(vec![7])
    })
    .unwrap();
    let second = cached_probe_winding_with(&mut cache, &negative, || {
        calls += 1;
        Ok(vec![9])
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, vec![7]);
    assert_eq!(second, vec![7]);
}

#[test]
fn cached_probe_winding_reuses_permuted_definition_families() {
    let definition = axis_plane_definition(&p(1, 2, 3));
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let first = ProbePoint {
        point: p(1, 2, 3),
        side: Classification::Positive,
        planes: vec![definition],
        uncertified_definition_fallback: false,
    };
    let second = ProbePoint {
        point: p(1, 2, 3),
        side: Classification::Positive,
        planes: vec![permuted],
        uncertified_definition_fallback: false,
    };
    let mut cache = Vec::new();
    let mut calls = 0;

    let first_result = cached_probe_winding_with(&mut cache, &first, || {
        calls += 1;
        Ok(vec![5])
    })
    .unwrap();
    let second_result = cached_probe_winding_with(&mut cache, &second, || {
        calls += 1;
        Ok(vec![9])
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first_result, vec![5]);
    assert_eq!(second_result, vec![5]);
}

#[test]
fn cached_surface_and_probe_reachability_reuse_equivalent_queries() {
    let definition = axis_plane_definition(&p(1, 2, 3));
    let interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![axis_plane_definition(&p(0, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let probe = ProbePoint {
        point: p(1, 2, 3),
        side: Classification::Positive,
        planes: vec![definition],
        uncertified_definition_fallback: false,
    };
    let mut surface_cache = Vec::new();
    let mut reachability_cache = Vec::new();
    let mut surface_calls = 0;
    let mut reachability_calls = 0;

    let first_surface = cached_surface_query_with(&mut surface_cache, &probe.point, || {
        surface_calls += 1;
        Ok(false)
    })
    .unwrap();
    let second_surface = cached_surface_query_with(&mut surface_cache, &probe.point, || {
        surface_calls += 1;
        Ok(true)
    })
    .unwrap();
    let first_reachability =
        cached_probe_reachability_with(&mut reachability_cache, &interior, &probe, || {
            reachability_calls += 1;
            Ok(true)
        })
        .unwrap();
    let second_reachability =
        cached_probe_reachability_with(&mut reachability_cache, &interior, &probe, || {
            reachability_calls += 1;
            Ok(false)
        })
        .unwrap();

    assert_eq!(surface_calls, 1);
    assert!(!first_surface);
    assert!(!second_surface);
    assert_eq!(reachability_calls, 1);
    assert!(first_reachability);
    assert!(second_reachability);
}

#[test]
fn cached_bounded_probes_from_interior_reuse_equivalent_queries() {
    let interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![axis_plane_definition(&p(0, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let support = Plane::axis_aligned(0, r(0));
    let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
    let probe = ProbePoint {
        point: p(1, 0, 0),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(1, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_bounded_probes_from_interior_with(
        &mut cache,
        &interior,
        &support,
        &bounds,
        true,
        || {
            calls += 1;
            Ok(vec![probe.clone()])
        },
    )
    .unwrap();
    let second = cached_bounded_probes_from_interior_with(
        &mut cache,
        &interior,
        &support,
        &bounds,
        true,
        || {
            calls += 1;
            Ok(Vec::new())
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, vec![probe.clone()]);
    assert_eq!(second, vec![probe]);
}

#[test]
fn cached_adjacent_normal_probes_reuse_equivalent_queries() {
    let interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![axis_plane_definition(&p(0, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let support = Plane::axis_aligned(0, r(0));
    let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
    let probe = ProbePoint {
        point: p(1, 0, 0),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(1, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_adjacent_normal_probes_with(&mut cache, &interior, &support, &bounds, true, || {
            calls += 1;
            Ok(vec![probe.clone()])
        })
        .unwrap();
    let second =
        cached_adjacent_normal_probes_with(&mut cache, &interior, &support, &bounds, true, || {
            calls += 1;
            Ok(Vec::new())
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, vec![probe.clone()]);
    assert_eq!(second, vec![probe]);
}

#[test]
fn cached_adjacent_axis_probes_reuse_equivalent_queries() {
    let interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![axis_plane_definition(&p(0, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let support = Plane::axis_aligned(0, r(0));
    let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
    let probe = ProbePoint {
        point: p(0, 1, 0),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(0, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_adjacent_axis_probes_with(&mut cache, &interior, &support, &bounds, 1, true, || {
            calls += 1;
            Ok(vec![probe.clone()])
        })
        .unwrap();
    let second =
        cached_adjacent_axis_probes_with(&mut cache, &interior, &support, &bounds, 1, true, || {
            calls += 1;
            Ok(Vec::new())
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, vec![probe.clone()]);
    assert_eq!(second, vec![probe]);
}

#[test]
fn cached_halfspace_cell_seed_families_reuse_permuted_halfspaces() {
    let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let (report, mut first_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
    let mut cache = Vec::new();

    let first = cached_halfspace_cell_seed_families_from_optional_report_with(
        &mut cache,
        &bounds,
        &halfspaces,
        report.as_ref(),
        &mut first_unknown,
    )
    .unwrap();

    let mut permuted_halfspaces = halfspaces.clone();
    permuted_halfspaces.rotate_left(2);
    let (permuted_report, mut second_unknown) =
        optional_halfspace_feasibility_report(&permuted_halfspaces).unwrap();
    let second = cached_halfspace_cell_seed_families_from_optional_report_with(
        &mut cache,
        &bounds,
        &permuted_halfspaces,
        permuted_report.as_ref(),
        &mut second_unknown,
    )
    .unwrap();

    assert_eq!(first, second);
    assert_eq!(cache.len(), 1);
    assert_eq!(first_unknown, second_unknown);
}

#[test]
fn cached_optional_halfspace_feasibility_report_reuses_permuted_halfspaces() {
    let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut cache = Vec::new();
    let mut first_unknown = false;

    let first = cached_optional_halfspace_feasibility_report_with(
        &mut cache,
        &halfspaces,
        &mut first_unknown,
    )
    .unwrap();

    let mut permuted_halfspaces = halfspaces.clone();
    permuted_halfspaces.rotate_left(2);
    let mut second_unknown = false;
    let second = cached_optional_halfspace_feasibility_report_with(
        &mut cache,
        &permuted_halfspaces,
        &mut second_unknown,
    )
    .unwrap();

    assert_eq!(first, second);
    assert_eq!(cache.len(), 1);
    assert_eq!(first_unknown, second_unknown);
}

#[test]
fn cached_adjacent_normal_probe_stop_values_reuse_equivalent_query() {
    let mut cache = Vec::new();
    let interior = p(0, 0, 0);
    let direction = p(0, 0, 1);
    let support = Plane::new(p(0, 0, 1), Real::from(0));
    let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
    let mut calls = 0;

    let first = cached_adjacent_normal_probe_stop_values_with(
        &mut cache,
        &interior,
        &direction,
        &support,
        &bounds,
        || {
            calls += 1;
            Ok((vec![r(1), r(2)], true))
        },
    )
    .unwrap();
    let second = cached_adjacent_normal_probe_stop_values_with(
        &mut cache,
        &interior,
        &direction,
        &support,
        &bounds,
        || {
            calls += 1;
            Ok((vec![r(3)], false))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, (vec![r(1), r(2)], true));
    assert_eq!(second, first);
    assert_eq!(cache.len(), 1);
}

#[test]
fn cached_adjacent_axis_probe_stop_values_reuse_equivalent_query() {
    let mut cache = Vec::new();
    let interior = p(0, 0, 0);
    let bounds = Aabb::new(p(-1, -1, -1), p(1, 1, 1));
    let mut calls = 0;

    let first = cached_adjacent_axis_probe_stop_values_with(
        &mut cache,
        &interior,
        &bounds,
        2,
        true,
        || {
            calls += 1;
            Ok((vec![r(1), r(2)], true))
        },
    )
    .unwrap();
    let second = cached_adjacent_axis_probe_stop_values_with(
        &mut cache,
        &interior,
        &bounds,
        2,
        true,
        || {
            calls += 1;
            Ok((vec![r(3)], false))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, (vec![r(1), r(2)], true));
    assert_eq!(second, first);
    assert_eq!(cache.len(), 1);
}

#[test]
fn cached_probe_reachability_reuses_permuted_definition_families() {
    let interior_definition = axis_plane_definition(&p(0, 0, 0));
    let interior_permuted = [
        interior_definition[1].clone(),
        interior_definition[2].clone(),
        interior_definition[0].clone(),
    ];
    let probe_definition = axis_plane_definition(&p(1, 2, 3));
    let probe_permuted = [
        probe_definition[1].clone(),
        probe_definition[2].clone(),
        probe_definition[0].clone(),
    ];
    let first_interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![interior_definition],
        uncertified_definition_fallback: false,
    };
    let second_interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![interior_permuted],
        uncertified_definition_fallback: false,
    };
    let first_probe = ProbePoint {
        point: p(1, 2, 3),
        side: Classification::Positive,
        planes: vec![probe_definition],
        uncertified_definition_fallback: false,
    };
    let second_probe = ProbePoint {
        point: p(1, 2, 3),
        side: Classification::Positive,
        planes: vec![probe_permuted],
        uncertified_definition_fallback: false,
    };
    let mut cache = Vec::new();
    let mut calls = 0;

    let first_result =
        cached_probe_reachability_with(&mut cache, &first_interior, &first_probe, || {
            calls += 1;
            Ok(true)
        })
        .unwrap();
    let second_result =
        cached_probe_reachability_with(&mut cache, &second_interior, &second_probe, || {
            calls += 1;
            Ok(false)
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert!(first_result);
    assert!(second_result);
}

#[test]
fn cached_probe_reachability_reuses_in_progress_exact_state() {
    let interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![axis_plane_definition(&p(0, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let probe = ProbePoint {
        point: p(1, 0, 0),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(1, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let mut cache = vec![ProbeReachabilityCacheEntry {
        interior_point: interior.point.clone(),
        interior_planes: interior.planes.clone(),
        probe_point: probe.point.clone(),
        probe_planes: probe.planes.clone(),
        reachable: Err(HypermeshError::UnknownClassification),
    }];

    let result = cached_probe_reachability_with(&mut cache, &interior, &probe, || Ok(true));

    assert_eq!(result, Err(HypermeshError::UnknownClassification));
    assert_eq!(cache.len(), 1);
    assert_eq!(
        cache[0].reachable,
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn cached_direct_probe_reachability_reuses_identical_query() {
    let mut cache = Vec::new();
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let host_support = Plane::axis_aligned(2, r(0));
    let polygons = vec![make_triangle(
        &p(2, -1, -1),
        &p(2, 1, -1),
        &p(2, 0, 1),
        0,
        0,
    )];
    let mut calls = 0;

    let first = cached_direct_probe_reachability_with(
        &mut cache,
        &start,
        &end,
        &host_support,
        &polygons,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_direct_probe_reachability_with(
        &mut cache,
        &start,
        &end,
        &host_support,
        &polygons,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert!(first);
    assert!(second);
    assert_eq!(cache.len(), 1);
}

#[test]
fn cached_direct_probe_reachability_reuses_reversed_query() {
    let mut cache = Vec::new();
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let host_support = Plane::axis_aligned(2, r(0));
    let polygons = vec![make_triangle(
        &p(2, -1, -1),
        &p(2, 1, -1),
        &p(2, 0, 1),
        0,
        0,
    )];
    let mut calls = 0;

    let first = cached_direct_probe_reachability_with(
        &mut cache,
        &start,
        &end,
        &host_support,
        &polygons,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_direct_probe_reachability_with(
        &mut cache,
        &end,
        &start,
        &host_support,
        &polygons,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(calls, 1);
    assert_eq!(cache.len(), 1);
}

#[test]
fn cached_direct_probe_reachability_shares_identical_polygon_families() {
    let mut cache = Vec::new();
    let host_support = Plane::axis_aligned(2, r(0));
    let polygons = vec![make_triangle(
        &p(2, -1, -1),
        &p(2, 1, -1),
        &p(2, 0, 1),
        0,
        0,
    )];

    cached_direct_probe_reachability_with(
        &mut cache,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &host_support,
        &polygons,
        || Ok(true),
    )
    .unwrap();
    cached_direct_probe_reachability_with(
        &mut cache,
        &p(0, 1, 0),
        &p(1, 1, 0),
        &host_support,
        &polygons,
        || Ok(true),
    )
    .unwrap();

    assert_eq!(cache.len(), 2);
    assert!(std::sync::Arc::ptr_eq(
        &cache[0].polygons,
        &cache[1].polygons
    ));
}

#[test]
fn trace_axis_ordered_paths_reuse_equivalent_intermediate_surface_queries() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 0);
    let mut surface_cache = Vec::new();
    let mut query_calls = 0;

    let err = trace_axis_ordered_paths_with_surface_query(&start, &end, &[0], &[], |point| {
        cached_surface_query_with(&mut surface_cache, point, || {
            query_calls += 1;
            Ok(*point == p(1, 0, 0) || *point == p(0, 1, 0))
        })
    })
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert_eq!(query_calls, 2);
}

#[test]
fn trace_axis_ordered_paths_reuse_equivalent_segment_traces() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 0);
    let mut segment_cache = Vec::new();
    let mut trace_calls = 0;

    let err = trace_axis_ordered_paths_with_queries(
        &start,
        &end,
        &[0],
        &[],
        |_point| Ok(false),
        |current, next, axis, attempt, _polygons| {
            cached_axis_ordered_segment_trace_with(
                &mut segment_cache,
                current,
                next,
                axis,
                attempt,
                || {
                    trace_calls += 1;
                    Ok(TraceAxisSegmentResult {
                        winding: attempt.to_vec(),
                        valid: false,
                    })
                },
            )
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert_eq!(trace_calls, 2);
}

#[test]
fn trace_axis_ordered_paths_try_later_ordering_after_uncertified_surface_query() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 0);

    let winding = trace_axis_ordered_paths_with_queries(
        &start,
        &end,
        &[7],
        &[],
        |point| {
            if *point == p(1, 0, 0) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        },
        |_current, _next, _axis, attempt, _polygons| {
            Ok(TraceAxisSegmentResult {
                winding: attempt.to_vec(),
                valid: true,
            })
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn trace_axis_ordered_paths_try_later_ordering_after_boundary_surface_query() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 0);
    let polygon = make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0);

    let winding = trace_axis_ordered_paths_with_surface_query(
        &start,
        &end,
        &[7],
        std::slice::from_ref(&polygon),
        |point| point_lies_on_traced_surface(point, std::slice::from_ref(&polygon)),
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn trace_axis_ordered_paths_try_later_ordering_after_uncertified_segment_step() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 0);

    let winding = trace_axis_ordered_paths_with_queries(
        &start,
        &end,
        &[7],
        &[],
        |_point| Ok(false),
        |current, next, _axis, attempt, _polygons| {
            if *current == start && *next == p(1, 0, 0) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(TraceAxisSegmentResult {
                    winding: attempt.to_vec(),
                    valid: true,
                })
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn trace_axis_ordered_paths_reports_unknown_for_zero_length_surface_contact() {
    let start = p(0, 0, 0);

    let err = trace_axis_ordered_paths_with_queries(
        &start,
        &start,
        &[7],
        &[],
        |_point| Ok(true),
        |_current, _next, _axis, _attempt, _polygons| {
            panic!("zero-length trace should not issue a segment step")
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn trace_axis_ordered_paths_try_later_ordering_after_endpoint_surface_contact() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 0);
    let polygon = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

    let winding = trace_axis_ordered_paths_with_queries(
        &start,
        &end,
        &[7],
        std::slice::from_ref(&polygon),
        |_point| Ok(false),
        |current, next, axis, attempt, polygons| {
            trace_axis_segment(current, next, axis, attempt, polygons)
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn cached_definition_no_detour_trace_reuses_identical_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut cache = Vec::new();
    let mut trace_calls = 0;

    let first = cached_definition_no_detour_trace_with(
        &mut cache,
        &start,
        &end,
        &[7],
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(Some(vec![7]))
        },
    )
    .unwrap();
    let second = cached_definition_no_detour_trace_with(
        &mut cache,
        &start,
        &end,
        &[7],
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(Some(vec![7]))
        },
    )
    .unwrap();

    assert_eq!(first, Some(vec![7]));
    assert_eq!(second, Some(vec![7]));
    assert_eq!(trace_calls, 1);
}

#[test]
fn cached_definition_no_detour_trace_reuses_permuted_definition_families() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_a = axis_plane_definition(&start);
    let start_b = axis_plane_definition(&p(0, 1, 0));
    let end_a = axis_plane_definition(&end);
    let end_b = axis_plane_definition(&p(1, 1, 0));
    let mut cache = Vec::new();
    let mut trace_calls = 0;

    let first = cached_definition_no_detour_trace_with(
        &mut cache,
        &start,
        &end,
        &[7],
        &[start_a.clone(), start_b.clone()],
        &[end_a.clone(), end_b.clone()],
        || {
            trace_calls += 1;
            Ok(Some(vec![7]))
        },
    )
    .unwrap();
    let second = cached_definition_no_detour_trace_with(
        &mut cache,
        &start,
        &end,
        &[7],
        &[start_b, start_a],
        &[end_b, end_a],
        || {
            trace_calls += 1;
            Ok(Some(vec![7]))
        },
    )
    .unwrap();

    assert_eq!(first, Some(vec![7]));
    assert_eq!(second, Some(vec![7]));
    assert_eq!(trace_calls, 1);
}

#[test]
fn trace_segment_from_definitions_shared_query_caches_reuse_equivalent_calls() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();

    let first = trace_segment_from_definitions_with_caches(
        &start,
        &end,
        &[7],
        &[],
        &start_definitions,
        &end_definitions,
        &mut no_detour_cache,
        &mut detour_target_cache,
        None,
    )
    .unwrap();
    let no_detour_len = no_detour_cache.len();
    let detour_len = detour_target_cache.len();
    let second = trace_segment_from_definitions_with_caches(
        &start,
        &end,
        &[7],
        &[],
        &start_definitions,
        &end_definitions,
        &mut no_detour_cache,
        &mut detour_target_cache,
        None,
    )
    .unwrap();

    assert_eq!(first, vec![7]);
    assert_eq!(second, vec![7]);
    assert_eq!(no_detour_cache.len(), no_detour_len);
    assert_eq!(detour_target_cache.len(), detour_len);
}

#[test]
fn cached_definition_no_detour_reachability_reuses_identical_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut cache = DefinitionNoDetourReachabilityCache::default();
    let mut trace_calls = 0;

    let first = cached_definition_no_detour_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_definition_no_detour_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(trace_calls, 1);
}

#[test]
fn cached_definition_no_detour_reachability_reuses_permuted_definition_families() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_a = axis_plane_definition(&start);
    let start_b = axis_plane_definition(&p(0, 1, 0));
    let end_a = axis_plane_definition(&end);
    let end_b = axis_plane_definition(&p(1, 1, 0));
    let mut cache = DefinitionNoDetourReachabilityCache::default();
    let mut trace_calls = 0;

    let first = cached_definition_no_detour_reachability_with(
        &mut cache,
        &start,
        &end,
        &[start_a.clone(), start_b.clone()],
        &[end_a.clone(), end_b.clone()],
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_definition_no_detour_reachability_with(
        &mut cache,
        &start,
        &end,
        &[start_b, start_a],
        &[end_b, end_a],
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(trace_calls, 1);
}

#[test]
fn cached_definition_no_detour_reachability_reuses_reversed_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut cache = DefinitionNoDetourReachabilityCache::default();
    let mut trace_calls = 0;

    let first = cached_definition_no_detour_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_definition_no_detour_reachability_with(
        &mut cache,
        &end,
        &start,
        &end_definitions,
        &start_definitions,
        || {
            trace_calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(trace_calls, 1);
}

#[test]
fn cached_definition_no_detour_reachability_reuses_in_progress_exact_state() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut cache =
        DefinitionNoDetourReachabilityCache::from(vec![DefinitionNoDetourReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            result: Err(HypermeshError::UnknownClassification),
        }]);

    let result = cached_definition_no_detour_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || Ok(true),
    );

    assert_eq!(result, Err(HypermeshError::UnknownClassification));
    assert_eq!(cache.len(), 1);
    assert_eq!(
        cache.entries[0].result,
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn cached_definition_no_plane_replacement_reachability_reuses_identical_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut trace_calls = 0;

    let first = cached_definition_no_plane_replacement_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_definition_no_plane_replacement_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(trace_calls, 1);
}

#[test]
fn cached_definition_no_plane_replacement_reachability_reuses_permuted_definition_families() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_a = axis_plane_definition(&start);
    let start_b = axis_plane_definition(&p(0, 1, 0));
    let end_a = axis_plane_definition(&end);
    let end_b = axis_plane_definition(&p(1, 1, 0));
    let mut cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut trace_calls = 0;

    let first = cached_definition_no_plane_replacement_reachability_with(
        &mut cache,
        &start,
        &end,
        &[start_a.clone(), start_b.clone()],
        &[end_a.clone(), end_b.clone()],
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_definition_no_plane_replacement_reachability_with(
        &mut cache,
        &start,
        &end,
        &[start_b, start_a],
        &[end_b, end_a],
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(trace_calls, 1);
}

#[test]
fn cached_definition_no_plane_replacement_reachability_reuses_reversed_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut trace_calls = 0;

    let first = cached_definition_no_plane_replacement_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || {
            trace_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_definition_no_plane_replacement_reachability_with(
        &mut cache,
        &end,
        &start,
        &end_definitions,
        &start_definitions,
        || {
            trace_calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(trace_calls, 1);
}

#[test]
fn cached_definition_no_plane_replacement_reachability_reuses_in_progress_exact_state() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let mut cache = DefinitionNoPlaneReplacementReachabilityCache::from(vec![
        DefinitionNoPlaneReplacementReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            result: Err(HypermeshError::UnknownClassification),
        },
    ]);

    let result = cached_definition_no_plane_replacement_reachability_with(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        || Ok(true),
    );

    assert_eq!(result, Err(HypermeshError::UnknownClassification));
    assert_eq!(cache.len(), 1);
    assert_eq!(
        cache.entries[0].result,
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn cached_detour_target_family_reuses_identical_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let target = DetourTarget {
        point: p(0, 1, 0),
        definitions: vec![axis_plane_definition(&p(0, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let mut cache = DetourTargetFamilyCache::default();
    let mut build_calls = 0;

    let first = cached_detour_target_family_with(&mut cache, &start, &end, None, || {
        build_calls += 1;
        Ok(vec![target.clone()])
    })
    .unwrap();
    let second = cached_detour_target_family_with(&mut cache, &start, &end, None, || {
        build_calls += 1;
        Ok(vec![target.clone()])
    })
    .unwrap();

    assert_eq!(first, vec![target.clone()]);
    assert_eq!(second, vec![target]);
    assert_eq!(build_calls, 1);
}

#[test]
fn cached_detour_target_family_reuses_reversed_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let target = DetourTarget {
        point: p(0, 1, 0),
        definitions: vec![axis_plane_definition(&p(0, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let mut cache = DetourTargetFamilyCache::default();
    let mut build_calls = 0;

    let first = cached_detour_target_family_with(&mut cache, &start, &end, None, || {
        build_calls += 1;
        Ok(vec![target.clone()])
    })
    .unwrap();
    let second = cached_detour_target_family_with(&mut cache, &end, &start, None, || {
        build_calls += 1;
        Ok(vec![target.clone()])
    })
    .unwrap();

    assert_eq!(first, vec![target.clone()]);
    assert_eq!(second, vec![target]);
    assert_eq!(build_calls, 1);
}

#[test]
fn cached_detour_target_family_distinguishes_trace_bounds() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let target = DetourTarget {
        point: p(0, 1, 0),
        definitions: vec![axis_plane_definition(&p(0, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let first_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let second_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
    let mut cache = DetourTargetFamilyCache::default();
    let mut build_calls = 0;

    for bounds in [&first_bounds, &second_bounds, &first_bounds] {
        cached_detour_target_family_with(&mut cache, &start, &end, Some(bounds), || {
            build_calls += 1;
            Ok(vec![target.clone()])
        })
        .unwrap();
    }

    assert_eq!(build_calls, 2);
    assert_eq!(cache.len(), 2);
}

#[test]
fn detour_trace_cycle_guard_reuses_surface_queries_across_failed_branches() {
    let start = p(0, 0, 0);
    let shared = p(1, 0, 0);
    let outer_b = p(2, 0, 0);
    let outer_a = p(3, 0, 0);
    let end = p(4, 0, 0);
    let outer_targets = vec![
        DetourTarget {
            point: outer_a.clone(),
            definitions: vec![axis_plane_definition(&outer_a)],
            uncertified_definition_fallback: false,
        },
        DetourTarget {
            point: outer_b.clone(),
            definitions: vec![axis_plane_definition(&outer_b)],
            uncertified_definition_fallback: false,
        },
    ];
    let shared_target = DetourTarget {
        point: shared.clone(),
        definitions: vec![axis_plane_definition(&shared)],
        uncertified_definition_fallback: false,
    };
    let mut surface_cache = Vec::new();
    let mut query_calls = 0;

    let err = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
        &start,
        &end,
        &[0],
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut surface_cache,
        &mut |_point| {
            query_calls += 1;
            Ok(false)
        },
        &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
        &mut |from, to| {
            if *from == start && *to == end {
                Ok(outer_targets.clone())
            } else if *from == start && (*to == outer_a || *to == outer_b) {
                Ok(vec![shared_target.clone()])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert_eq!(query_calls, 3);
}

#[test]
fn normal_probe_is_clipped_before_intervening_surface() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let blocker = make_triangle(&p(6, 0, 0), &p(0, 6, 0), &p(0, 0, 6), 1, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let vertices = leaf.vertices().unwrap();
    let center = centroid(&vertices).unwrap().unwrap();
    let interior = shifted_edge_interior_points(&leaf, &center)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("shifted edge construction should retain defining planes");

    let probes = adjacent_normal_probes(
        &interior,
        &leaf.support,
        &bounds,
        std::slice::from_ref(&blocker),
        true,
    )
    .unwrap();
    let probe = probes
        .into_iter()
        .find(|probe| probe.side == Classification::Positive)
        .expect("normal corridor should contain a certified probe witness");

    assert!(!probe.planes.is_empty());
    for definition in &probe.planes {
        assert_eq!(affine_from_planes(definition).unwrap(), probe.point);
    }
    let start_value = leaf.support.expression_at_point(&interior.point);
    let probe_value = leaf.support.expression_at_point(&probe.point);
    let blocker_value = blocker.support.expression_at_point(&probe.point);
    assert!(compare_real(&probe_value, &start_value).unwrap().is_gt());
    assert!(compare_real(&blocker_value, &Real::zero()).unwrap().is_lt());
}

#[test]
fn adjacent_normal_probe_stop_values_backtrack_after_uncertified_crossing() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = p(1, 1, 1);
    let direction = support.normal.clone();
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior,
        &direction,
        &support,
        &bounds,
        &[first, second],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(if *point == p(3, 1, 1) {
                    PolygonPointLocation::Interior
                } else {
                    PolygonPointLocation::Outside
                })
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3)]);
}

#[test]
fn adjacent_normal_probe_marks_later_corridor_uncertain_after_uncertified_crossing() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let probes = adjacent_normal_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        true,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        |point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(if *point == p(3, 1, 1) {
                    PolygonPointLocation::Interior
                } else {
                    PolygonPointLocation::Outside
                })
            }
        },
        |corridor, stop_point| {
            if corridor.max.x == r(3) && *stop_point == p(3, 1, 1) {
                Ok(vec![ProbePoint {
                    point: p(2, 1, 1),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(&p(2, 1, 1))],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, p(2, 1, 1));
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn adjacent_normal_probe_reports_unknown_when_corridor_family_is_partially_uncertified_and_later_corridors_fail()
 {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let err = adjacent_normal_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        true,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        |point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(if *point == p(3, 1, 1) {
                    PolygonPointLocation::Interior
                } else {
                    PolygonPointLocation::Outside
                })
            }
        },
        |_corridor, _stop_point| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn adjacent_normal_probe_stop_values_retain_boundary_crossing_as_stop() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = p(1, 1, 1);
    let direction = support.normal.clone();
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior,
        &direction,
        &support,
        &bounds,
        &[first, second],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |_point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(1), r(2), r(3)]);
}

#[test]
fn adjacent_normal_probe_stop_values_treat_boundary_start_contact_as_unknown_and_keep_later_corridor()
 {
    let support = Plane::axis_aligned(0, r(0));
    let interior = p(1, 1, 1);
    let direction = support.normal.clone();
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior,
        &direction,
        &support,
        &bounds,
        &[first, second],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |_point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3)]);
}

#[test]
fn adjacent_normal_probe_stop_values_treat_endpoint_boundary_contact_as_unknown_and_keep_later_corridor()
 {
    let support = Plane::axis_aligned(0, r(0));
    let interior = p(1, 1, 1);
    let direction = support.normal.clone();
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior,
        &direction,
        &support,
        &bounds,
        &[first, second],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(4) {
                Ok(if *point == p(4, 1, 1) {
                    PolygonPointLocation::Boundary
                } else {
                    PolygonPointLocation::Outside
                })
            } else {
                Ok(if *point == p(3, 1, 1) {
                    PolygonPointLocation::Interior
                } else {
                    PolygonPointLocation::Outside
                })
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3)]);
}

#[test]
fn adjacent_normal_probe_stop_values_treat_bound_start_contact_as_unknown() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = p(4, 1, 1);
    let direction = support.normal.clone();
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior,
        &direction,
        &support,
        &bounds,
        &[],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |_point, _polygon| Ok(PolygonPointLocation::Outside),
    )
    .unwrap();

    assert!(saw_unknown);
    assert!(stop_values.is_empty());
}

#[test]
fn adjacent_normal_probe_reports_unknown_for_bound_start_contact() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(4, 1, 1),
        planes: vec![axis_plane_definition(&p(4, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let err = adjacent_normal_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[],
        true,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        |_point, _polygon| Ok(PolygonPointLocation::Outside),
        |_corridor, _stop_point| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn adjacent_normal_probe_marks_later_corridor_uncertain_after_boundary_start_contact() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let probes = adjacent_normal_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        true,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        |_point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
        |corridor, stop_point| {
            if corridor.max.x == r(3) && *stop_point == p(3, 1, 1) {
                Ok(vec![ProbePoint {
                    point: p(2, 1, 1),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(&p(2, 1, 1))],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, p(2, 1, 1));
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn adjacent_normal_probe_marks_later_corridor_uncertain_after_endpoint_boundary_contact() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let probes = adjacent_normal_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        true,
        |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        |point, polygon| {
            if polygon.vertices().unwrap()[0].x == r(4) {
                Ok(if *point == p(4, 1, 1) {
                    PolygonPointLocation::Boundary
                } else {
                    PolygonPointLocation::Outside
                })
            } else {
                Ok(if *point == p(3, 1, 1) {
                    PolygonPointLocation::Interior
                } else {
                    PolygonPointLocation::Outside
                })
            }
        },
        |corridor, stop_point| {
            if corridor.max.x == r(3) && *stop_point == p(3, 1, 1) {
                Ok(vec![ProbePoint {
                    point: p(2, 1, 1),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(&p(2, 1, 1))],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, p(2, 1, 1));
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn strict_normal_probe_targets_try_shifted_search_from_report_witness_seed() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let stop_point = p(3, 1, 1);
    let witness = p(1, 2, 2);
    let visited = std::cell::RefCell::new(Vec::new());

    let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
        &interior,
        &support,
        &corridor,
        None,
        &stop_point,
        true,
        Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [None, None, None],
        )),
        Vec::new(),
        vec![witness.clone()],
        Vec::new(),
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ShiftedHalfspaceWitness {
                point: p(2, 1, 1),
                families: vec![ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(3))],
                    active_planes: [Some(0), None, None],
                }],
                uncertified_definition_fallback: false,
            }])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness]);
    assert!(probes.iter().any(|probe| probe.point == p(2, 1, 1)));
}

#[test]
fn strict_normal_probe_targets_merge_same_point_certified_shifted_replay_definitions() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![[
            support.clone(),
            Plane::axis_aligned(2, r(1)),
            Plane::axis_aligned(2, r(1)),
        ]],
        uncertified_definition_fallback: false,
    };
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let stop_point = p(3, 1, 1);
    let witness = p(2, 1, 1);
    let visited = std::cell::RefCell::new(Vec::new());

    let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
        &interior,
        &support,
        &corridor,
        None,
        &stop_point,
        true,
        Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [None, None, None],
        )),
        vec![witness.clone()],
        Vec::new(),
        Vec::new(),
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ShiftedHalfspaceWitness {
                point: seed.clone(),
                families: vec![ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(1, false, r(2))],
                    active_planes: [Some(0), None, None],
                }],
                uncertified_definition_fallback: false,
            }])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness.clone()]);
    let probe = probes
        .iter()
        .find(|probe| probe.point == witness && probe.side == Classification::Positive)
        .expect("same-point shifted replay should keep the direct probe and enrich it");
    assert!(!probe.uncertified_definition_fallback);
    assert!(probe.planes.iter().any(|definition| {
        definition
            .iter()
            .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
    }));
}

#[test]
fn collect_normal_probe_targets_keeps_unrestricted_family_after_definition_hits() {
    let support = Plane::axis_aligned(2, r(0));
    let definition = [
        support.clone(),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let constrained_probe = ProbePoint {
        point: p(1, 1, 1),
        side: Classification::Positive,
        planes: vec![definition.clone()],
        uncertified_definition_fallback: false,
    };
    let unrestricted_probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };

    let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Ok(vec![constrained_probe.clone()]),
        None => Ok(vec![unrestricted_probe.clone()]),
    })
    .unwrap();

    assert_eq!(probes.len(), 2);
    assert!(
        probes
            .iter()
            .any(|probe| probe.point == constrained_probe.point)
    );
    assert!(
        probes
            .iter()
            .any(|probe| probe.point == unrestricted_probe.point)
    );
}

#[test]
fn unique_normal_probe_search_definitions_skip_duplicate_retained_pairs() {
    let support = Plane::axis_aligned(0, r(0));
    let axis_definition = axis_plane_definition(&p(1, 2, 3));
    let duplicate_first = [
        Plane::axis_aligned(0, r(7)),
        axis_definition[1].clone(),
        axis_definition[2].clone(),
    ];
    let swapped_pair = [
        Plane::axis_aligned(0, r(9)),
        axis_definition[2].clone(),
        axis_definition[1].clone(),
    ];

    let unique = unique_normal_probe_search_definitions(
        &[axis_definition.clone(), duplicate_first, swapped_pair],
        &support,
    )
    .unwrap();

    assert_eq!(unique.len(), 1);
    assert!(retained_plane_pairs_match_as_sets(
        &unique[0],
        &axis_definition
    ));
}

#[test]
fn collect_normal_probe_targets_merges_duplicate_unrestricted_probe_definitions() {
    let support = Plane::axis_aligned(2, r(0));
    let definition_probe = ProbePoint {
        point: p(1, 1, 1),
        side: Classification::Positive,
        planes: vec![[
            support.clone(),
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ]],
        uncertified_definition_fallback: false,
    };
    let extra_definition = [
        support,
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(2, r(1)),
    ];

    let probes = collect_normal_probe_targets(&[definition_probe.planes[0].clone()], |candidate| {
        match candidate {
            Some(_) => Ok(vec![definition_probe.clone()]),
            None => Ok(vec![ProbePoint {
                point: definition_probe.point.clone(),
                side: definition_probe.side,
                planes: vec![extra_definition.clone()],
                uncertified_definition_fallback: false,
            }]),
        }
    })
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].planes.len(), 2);
}

#[test]
fn collect_normal_probe_targets_skips_duplicate_definition_families() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let mut definition_calls = 0;
    let mut unrestricted_calls = 0;

    let probes =
        collect_normal_probe_targets(&[definition.clone(), definition.clone()], |candidate| {
            match candidate {
                Some(found_definition) => {
                    definition_calls += 1;
                    assert_eq!(found_definition, &definition);
                    Ok(vec![ProbePoint {
                        point: p(0, 0, 1),
                        side: Classification::Positive,
                        planes: vec![definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                }
                None => {
                    unrestricted_calls += 1;
                    Ok(Vec::new())
                }
            }
        })
        .unwrap();

    assert_eq!(definition_calls, 1);
    assert_eq!(unrestricted_calls, 1);
    assert_eq!(probes.len(), 1);
}

#[test]
fn collect_normal_probe_targets_skips_permuted_definition_families() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let permuted_definition = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let mut definition_calls = 0;
    let mut unrestricted_calls = 0;

    let probes =
        collect_normal_probe_targets(&[definition.clone(), permuted_definition], |candidate| {
            match candidate {
                Some(found_definition) => {
                    definition_calls += 1;
                    assert!(definition_planes_match_as_sets(
                        found_definition,
                        &definition
                    ));
                    Ok(vec![ProbePoint {
                        point: p(0, 0, 1),
                        side: Classification::Positive,
                        planes: vec![definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                }
                None => {
                    unrestricted_calls += 1;
                    Ok(Vec::new())
                }
            }
        })
        .unwrap();

    assert_eq!(definition_calls, 1);
    assert_eq!(unrestricted_calls, 1);
    assert_eq!(probes.len(), 1);
}

#[test]
fn collect_normal_probe_targets_backtracks_after_uncertified_definition() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let unrestricted_probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };

    let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Err(HypermeshError::UnknownClassification),
        None => Ok(vec![unrestricted_probe.clone()]),
    })
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, unrestricted_probe.point);
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn collect_normal_probe_targets_report_unknown_if_all_families_are_uncertified() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];

    let err = collect_normal_probe_targets(&[definition], |_candidate| {
        Err(HypermeshError::UnknownClassification)
    })
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn collect_normal_probe_targets_mark_later_probes_uncertain_after_uncertain_family_result() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];

    let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Ok(vec![ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: true,
        }]),
        None => Ok(vec![ProbePoint {
            point: p(3, 3, 3),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(3, 3, 3))],
            uncertified_definition_fallback: false,
        }]),
    })
    .unwrap();

    assert_eq!(probes.len(), 2);
    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn collect_normal_probe_targets_keeps_certified_duplicate_state_certified() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let point = p(2, 2, 2);
    let planes = vec![axis_plane_definition(&point)];

    let probes = collect_normal_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Ok(vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: planes.clone(),
            uncertified_definition_fallback: true,
        }]),
        None => Ok(vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: planes.clone(),
            uncertified_definition_fallback: false,
        }]),
    })
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert!(!probes[0].uncertified_definition_fallback);
}

#[test]
fn probe_point_build_collection_backtracks_after_uncertified_candidate() {
    let mut probes = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            if *candidate == first {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(ProbePoint {
                    point: candidate.clone(),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: false,
                }))
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, second);
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn probe_point_build_collection_marks_existing_probes_uncertain_after_later_unknown() {
    let mut probes = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            if *candidate == second {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(ProbePoint {
                    point: candidate.clone(),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: false,
                }))
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn probe_point_build_collection_marks_later_probes_uncertain_after_uncertain_candidate_result() {
    let mut probes = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            Ok(Some(ProbePoint {
                point: candidate.clone(),
                side: Classification::Positive,
                planes: vec![axis_plane_definition(candidate)],
                uncertified_definition_fallback: *candidate == first,
            }))
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 2);
    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn probe_point_build_collection_keeps_certified_duplicate_state_certified() {
    let mut probes = Vec::new();
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);

    extend_probe_point_builds_backtracking_unknown(&mut probes, [0, 1].iter(), |candidate| {
        Ok(Some(ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: vec![definition.clone()],
            uncertified_definition_fallback: *candidate == 0,
        }))
    })
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert!(!probes[0].uncertified_definition_fallback);
}

#[test]
fn probe_point_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let mut probes = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    let err = extend_probe_point_builds_backtracking_unknown(
        &mut probes,
        [first, second].iter(),
        |_candidate| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn adjacent_axis_probe_uses_corridor_witness_and_retains_definition() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };

    let probe = adjacent_axis_probes(&interior, &leaf.support, &bounds, &[], 0, true)
        .unwrap()
        .into_iter()
        .find(|probe| probe.side == Classification::Positive)
        .expect("axis corridor should contain a certified probe witness");

    assert_eq!(probe.side, Classification::Positive);
    assert!(!probe.planes.is_empty());
    for definition in &probe.planes {
        assert_eq!(affine_from_planes(definition).unwrap(), probe.point);
    }
    assert!(compare_real(&probe.point.x, &r(1)).unwrap().is_gt());
    assert!(compare_real(&probe.point.x, &r(4)).unwrap().is_lt());
    assert_eq!(probe.point.y, r(1));
    assert_eq!(probe.point.z, r(1));
}

#[test]
fn adjacent_axis_probe_stop_values_backtrack_after_uncertified_crossing() {
    let interior = p(1, 1, 1);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
        &interior,
        &bounds,
        &[first, second],
        0,
        true,
        &mut |_interior, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        &mut |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(3), r(4)]);
}

#[test]
fn adjacent_axis_probe_marks_later_corridor_uncertain_after_uncertified_crossing() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let probes = adjacent_axis_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        0,
        true,
        |_interior, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
        |corridor| {
            if corridor.max.x == r(3) {
                Ok(vec![ProbePoint {
                    point: p(2, 1, 1),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(&p(2, 1, 1))],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, p(2, 1, 1));
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn adjacent_axis_probe_reports_unknown_when_corridor_family_is_partially_uncertified_and_later_corridors_fail()
 {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let err = adjacent_axis_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        0,
        true,
        |_interior, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        |_crossing, _polygon| Ok(PolygonPointLocation::Interior),
        |_corridor| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn adjacent_axis_probe_stop_values_retain_boundary_crossing_as_stop() {
    let interior = p(1, 1, 1);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
        &interior,
        &bounds,
        &[first, second],
        0,
        true,
        &mut |_interior, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Ok(Some(p(2, 1, 1)))
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        &mut |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3), r(4)]);
}

#[test]
fn adjacent_axis_probe_stop_values_treat_endpoint_boundary_contact_as_unknown_and_keep_later_corridor()
 {
    let interior = p(1, 1, 1);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
        &interior,
        &bounds,
        &[first, second],
        0,
        true,
        &mut |_interior, endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(4) {
                Ok(Some(endpoint.clone()))
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        &mut |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(4) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(3), r(4)]);
}

#[test]
fn adjacent_axis_probe_stop_values_treat_start_boundary_contact_as_unknown_and_keep_later_corridor()
{
    let interior = p(1, 1, 1);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
        &interior,
        &bounds,
        &[first, second],
        0,
        true,
        &mut |interior, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(Some(interior.clone()))
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        &mut |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(3), r(4)]);
}

#[test]
fn adjacent_axis_probe_stop_values_treat_bound_start_contact_as_unknown() {
    let interior = p(4, 1, 1);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let (stop_values, saw_unknown) = adjacent_axis_probe_stop_values_with_queries(
        &interior,
        &bounds,
        &[],
        0,
        true,
        &mut |_interior, _endpoint, _polygon, _axis| Ok(None),
        &mut |_crossing, _polygon| Ok(PolygonPointLocation::Outside),
    )
    .unwrap();

    assert!(saw_unknown);
    assert!(stop_values.is_empty());
}

#[test]
fn adjacent_axis_probe_reports_unknown_for_bound_start_contact() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(4, 1, 1),
        planes: vec![axis_plane_definition(&p(4, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let err = adjacent_axis_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[],
        0,
        true,
        |_interior, _endpoint, _polygon, _axis| Ok(None),
        |_crossing, _polygon| Ok(PolygonPointLocation::Outside),
        |_corridor| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn adjacent_axis_probe_marks_later_corridor_uncertain_after_boundary_crossing() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let probes = adjacent_axis_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        0,
        true,
        |_interior, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Ok(Some(p(2, 1, 1)))
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(2) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
        |corridor| {
            if corridor.max.x == r(3) {
                Ok(vec![ProbePoint {
                    point: p(2, 1, 1),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(&p(2, 1, 1))],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, p(2, 1, 1));
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn adjacent_axis_probe_marks_later_corridor_uncertain_after_endpoint_boundary_contact() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(4, 0, 0), &p(4, 1, 0), &p(4, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let probes = adjacent_axis_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        0,
        true,
        |_interior, endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(4) {
                Ok(Some(endpoint.clone()))
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(4) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
        |corridor| {
            if corridor.max.x == r(3) {
                Ok(vec![ProbePoint {
                    point: p(2, 1, 1),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(&p(2, 1, 1))],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, p(2, 1, 1));
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn adjacent_axis_probe_marks_later_corridor_uncertain_after_boundary_start_contact() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 1);

    let probes = adjacent_axis_probes_with_queries(
        &interior,
        &support,
        &bounds,
        &[first, second],
        0,
        true,
        |interior, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(Some(interior.clone()))
            } else {
                Ok(Some(p(3, 1, 1)))
            }
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(PolygonPointLocation::Boundary)
            } else {
                Ok(PolygonPointLocation::Interior)
            }
        },
        |corridor| {
            if corridor.max.x == r(3) {
                Ok(vec![ProbePoint {
                    point: p(2, 1, 1),
                    side: Classification::Positive,
                    planes: vec![axis_plane_definition(&p(2, 1, 1))],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, p(2, 1, 1));
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn strict_axis_probe_targets_try_shifted_search_from_report_witness_seed() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![axis_plane_definition(&p(1, 1, 1))],
        uncertified_definition_fallback: false,
    };
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let witness = p(1, 2, 2);
    let visited = std::cell::RefCell::new(Vec::new());

    let probes = strict_axis_probe_targets_from_seed_families_with_tracking_unknown(
        &interior,
        &support,
        &corridor,
        0,
        true,
        None,
        Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [None, None, None],
        )),
        Vec::new(),
        vec![witness.clone()],
        Vec::new(),
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ShiftedHalfspaceWitness {
                point: p(2, 1, 1),
                families: vec![ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(0, false, r(3))],
                    active_planes: [Some(0), None, None],
                }],
                uncertified_definition_fallback: false,
            }])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness]);
    assert!(probes.iter().any(|probe| probe.point == p(2, 1, 1)));
}

#[test]
fn strict_axis_probe_targets_merge_same_point_certified_shifted_replay_definitions() {
    let support = Plane::axis_aligned(0, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 1),
        planes: vec![[
            support.clone(),
            Plane::axis_aligned(2, r(1)),
            Plane::axis_aligned(2, r(1)),
        ]],
        uncertified_definition_fallback: false,
    };
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let witness = p(2, 1, 1);
    let visited = std::cell::RefCell::new(Vec::new());

    let probes = strict_axis_probe_targets_from_seed_families_with_tracking_unknown(
        &interior,
        &support,
        &corridor,
        0,
        true,
        None,
        Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [None, None, None],
        )),
        vec![witness.clone()],
        Vec::new(),
        Vec::new(),
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ShiftedHalfspaceWitness {
                point: seed.clone(),
                families: vec![ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(1, false, r(2))],
                    active_planes: [Some(0), None, None],
                }],
                uncertified_definition_fallback: false,
            }])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness.clone()]);
    let probe = probes
        .iter()
        .find(|probe| probe.point == witness && probe.side == Classification::Positive)
        .expect("same-point shifted replay should keep the direct axis probe and enrich it");
    assert!(!probe.uncertified_definition_fallback);
    assert!(probe.planes.iter().any(|definition| {
        definition
            .iter()
            .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
    }));
}

#[test]
fn collect_axis_probe_targets_backtracks_after_uncertified_definition() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let unrestricted_probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };

    let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Err(HypermeshError::UnknownClassification),
        None => Ok(vec![unrestricted_probe.clone()]),
    })
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].point, unrestricted_probe.point);
    assert!(probes[0].uncertified_definition_fallback);
}

#[test]
fn collect_axis_probe_targets_report_unknown_if_all_families_are_uncertified() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];

    let err = collect_axis_probe_targets(&[definition], |_candidate| {
        Err(HypermeshError::UnknownClassification)
    })
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn collect_axis_probe_targets_mark_later_probes_uncertain_after_uncertain_family_result() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];

    let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Ok(vec![ProbePoint {
            point: p(2, 2, 2),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(2, 2, 2))],
            uncertified_definition_fallback: true,
        }]),
        None => Ok(vec![ProbePoint {
            point: p(3, 3, 3),
            side: Classification::Positive,
            planes: vec![axis_plane_definition(&p(3, 3, 3))],
            uncertified_definition_fallback: false,
        }]),
    })
    .unwrap();

    assert_eq!(probes.len(), 2);
    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn collect_axis_probe_targets_keeps_certified_duplicate_state_certified() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let point = p(2, 2, 2);
    let planes = vec![axis_plane_definition(&point)];

    let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Ok(vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: planes.clone(),
            uncertified_definition_fallback: true,
        }]),
        None => Ok(vec![ProbePoint {
            point: point.clone(),
            side: Classification::Positive,
            planes: planes.clone(),
            uncertified_definition_fallback: false,
        }]),
    })
    .unwrap();

    assert_eq!(probes.len(), 1);
    assert!(!probes[0].uncertified_definition_fallback);
}

#[test]
fn collect_axis_probe_targets_skips_duplicate_definition_families() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let mut definition_calls = 0;
    let mut unrestricted_calls = 0;

    let probes =
        collect_axis_probe_targets(&[definition.clone(), definition.clone()], |candidate| {
            match candidate {
                Some(found_definition) => {
                    definition_calls += 1;
                    assert_eq!(found_definition, &definition);
                    Ok(vec![ProbePoint {
                        point: p(1, 0, 0),
                        side: Classification::Positive,
                        planes: vec![definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                }
                None => {
                    unrestricted_calls += 1;
                    Ok(Vec::new())
                }
            }
        })
        .unwrap();

    assert_eq!(definition_calls, 1);
    assert_eq!(unrestricted_calls, 1);
    assert_eq!(probes.len(), 1);
}

#[test]
fn collect_axis_probe_targets_keeps_unrestricted_family_after_definition_hits() {
    let definition = [
        Plane::axis_aligned(2, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let constrained_probe = ProbePoint {
        point: p(1, 1, 1),
        side: Classification::Positive,
        planes: vec![definition.clone()],
        uncertified_definition_fallback: false,
    };
    let unrestricted_probe = ProbePoint {
        point: p(2, 2, 2),
        side: Classification::Positive,
        planes: vec![axis_plane_definition(&p(2, 2, 2))],
        uncertified_definition_fallback: false,
    };

    let probes = collect_axis_probe_targets(&[definition], |candidate| match candidate {
        Some(_) => Ok(vec![constrained_probe.clone()]),
        None => Ok(vec![unrestricted_probe.clone()]),
    })
    .unwrap();

    assert_eq!(probes.len(), 2);
    assert!(
        probes
            .iter()
            .any(|probe| probe.point == constrained_probe.point)
    );
    assert!(
        probes
            .iter()
            .any(|probe| probe.point == unrestricted_probe.point)
    );
}

#[test]
fn adjacent_axis_probe_preserves_retained_definition_when_axis_direction_allows() {
    let support = Plane::axis_aligned(2, r(0));
    let bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
    let retained = [
        support.clone(),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![retained.clone()],
        uncertified_definition_fallback: false,
    };

    let probe = adjacent_axis_probes(&interior, &support, &bounds, &[], 2, true)
        .unwrap()
        .into_iter()
        .find(|probe| {
            probe.side == Classification::Positive
                && probe
                    .planes
                    .iter()
                    .any(|planes| planes[1] == retained[1] && planes[2] == retained[2])
        })
        .expect("axis-direction probe should preserve retained axis-stable planes");

    assert_eq!(probe.point.x, r(1));
    assert_eq!(probe.point.y, r(1));
    assert!(compare_real(&probe.point.z, &r(0)).unwrap().is_gt());
}

#[test]
fn leaf_classification_uses_certified_slanted_normal_probe() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let ref_definitions = [axis_plane_definition(&p(0, 0, 0))];

    let winding = classify_leaf_polygon(
        &leaf.support,
        &leaf.edges,
        &p(0, 0, 0),
        &ref_definitions,
        &[0],
        std::slice::from_ref(&leaf),
        &bounds,
        &leaf.delta_w,
    )
    .unwrap();

    assert_eq!(winding, vec![-1]);
}

#[test]
fn leaf_classification_keeps_certified_direct_leaf_witness_after_invalid_active_replay() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let ref_point = p(0, 0, 0);
    let ref_definitions = [axis_plane_definition(&ref_point)];
    let interior = build_strict_leaf_point(
        &leaf,
        &p(1, 1, 1),
        &[
            limit_plane_from_plane(&leaf.support),
            axis_halfspace(0, false, r(1)),
        ],
        [Some(9), None, None],
        false,
    )
    .unwrap()
    .expect("direct leaf witness should still certify");

    assert!(!interior.uncertified_definition_fallback);

    let winding = classify_leaf_polygon_from_interior_points(
        std::slice::from_ref(&interior),
        &leaf.support,
        &ref_point,
        &ref_definitions,
        &[0],
        std::slice::from_ref(&leaf),
        &bounds,
        &leaf.delta_w,
    )
    .unwrap();

    assert_eq!(winding, vec![-1]);
}

#[test]
fn leaf_classification_certifies_fallback_marked_interior_after_complete_probe_proof() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let ref_point = p(0, 0, 0);
    let ref_definitions = [axis_plane_definition(&ref_point)];
    let mut interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .next()
        .expect("slanted leaf should have an interior witness");
    interior.uncertified_definition_fallback = true;

    let winding = classify_leaf_polygon_from_interior_points(
        std::slice::from_ref(&interior),
        &leaf.support,
        &ref_point,
        &ref_definitions,
        &[0],
        &[leaf.clone()],
        &bounds,
        &leaf.delta_w,
    )
    .unwrap();

    assert_eq!(winding, vec![-1]);
}

#[test]
fn positive_probe_traces_for_slanted_leaf_case() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let ref_point = p(0, 0, 0);
    let ref_definitions = [axis_plane_definition(&ref_point)];
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");
    let probe =
        bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[leaf.clone()])
            .unwrap()
            .into_iter()
            .find(|probe| probe.side == Classification::Positive)
            .expect("slanted leaf should have a positive-side probe");
    assert!(probe.uncertified_definition_fallback);

    assert!(
        probe_reaches_adjacent_cell_from_interior(
            &interior,
            &probe,
            &leaf.support,
            &[leaf.clone()],
        )
        .unwrap()
    );

    let winding =
        trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[leaf.clone()]).unwrap();

    assert_eq!(winding, vec![-1]);
}

#[test]
fn certified_leaf_interior_points_exist_for_slanted_leaf_case() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);

    let interior_points = certified_leaf_interior_points(&leaf.support, &leaf.edges).unwrap();

    assert!(!interior_points.is_empty());
    assert!(interior_points.iter().any(|point| !point.planes.is_empty()));
}

#[test]
fn bounded_probes_find_positive_probe_for_slanted_leaf_case() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");

    let probes =
        bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[leaf.clone()])
            .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
}

#[test]
fn bounded_probes_preserve_family_uncertainty_for_slanted_leaf_case() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");

    let probes =
        bounded_probes_from_interior(&interior, &leaf.support, &bounds, true, &[leaf.clone()])
            .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn adjacent_normal_probe_stop_values_exist_for_slanted_leaf_case() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");

    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &leaf.support.normal,
        &leaf.support,
        &bounds,
        std::slice::from_ref(&leaf),
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| classify_point_in_polygon(point, polygon),
    )
    .unwrap();

    assert!(!saw_unknown);
    assert!(!stop_values.is_empty());
    assert!(
        stop_values
            .iter()
            .all(|stop| { compare_real(stop, &Real::zero()).unwrap().is_gt() })
    );
}

#[test]
fn strict_normal_probe_targets_find_positive_probe_for_slanted_leaf_case() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");
    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &leaf.support.normal,
        &leaf.support,
        &bounds,
        &[leaf.clone()],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| classify_point_in_polygon(point, polygon),
    )
    .unwrap();

    assert!(!saw_unknown);
    let stop_t = stop_values[0].clone();
    let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
    let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

    let probes = strict_normal_probe_targets(
        &interior,
        &leaf.support,
        &corridor,
        Some(&interior.planes[0]),
        &stop_point,
        true,
    )
    .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
}

#[test]
fn strict_normal_probe_targets_preserve_family_uncertainty_for_slanted_leaf_case() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");
    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &leaf.support.normal,
        &leaf.support,
        &bounds,
        &[leaf.clone()],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| classify_point_in_polygon(point, polygon),
    )
    .unwrap();

    assert!(!saw_unknown);
    let stop_t = stop_values[0].clone();
    let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
    let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

    let probes = strict_normal_probe_targets(
        &interior,
        &leaf.support,
        &corridor,
        Some(&interior.planes[0]),
        &stop_point,
        true,
    )
    .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn adjacent_normal_probes_find_positive_probe_for_slanted_leaf_case() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");

    let probes =
        adjacent_normal_probes(&interior, &leaf.support, &bounds, &[leaf.clone()], true).unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
}

#[test]
fn adjacent_normal_probes_preserve_family_uncertainty_for_slanted_leaf_case() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");

    let probes =
        adjacent_normal_probes(&interior, &leaf.support, &bounds, &[leaf.clone()], true).unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
    assert!(
        probes
            .iter()
            .all(|probe| probe.uncertified_definition_fallback)
    );
}

#[test]
fn strict_normal_probe_targets_find_positive_probe_for_slanted_leaf_case_unrestricted() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");
    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &leaf.support.normal,
        &leaf.support,
        &bounds,
        &[leaf.clone()],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| classify_point_in_polygon(point, polygon),
    )
    .unwrap();

    assert!(!saw_unknown);
    let stop_t = stop_values[0].clone();
    let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
    let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

    let probes =
        strict_normal_probe_targets(&interior, &leaf.support, &corridor, None, &stop_point, true)
            .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
}

#[test]
fn strict_normal_probe_direct_seed_phase_finds_positive_probe_for_slanted_leaf_case_unrestricted() {
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");
    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &leaf.support.normal,
        &leaf.support,
        &bounds,
        &[leaf.clone()],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| classify_point_in_polygon(point, polygon),
    )
    .unwrap();

    assert!(!saw_unknown);
    let stop_t = stop_values[0].clone();
    let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
    let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

    let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
    halfspaces.push(support_side_halfspace(&leaf.support, true));
    halfspaces.push(normal_stop_halfspace(&leaf.support, &stop_point, true));
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
    assert!(!saw_unknown);
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            &corridor,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )
        .unwrap();

    let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
        &interior,
        &leaf.support,
        &corridor,
        None,
        &stop_point,
        true,
        report.as_ref(),
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        |_seed| Ok(Vec::new()),
    )
    .unwrap();

    assert!(
        probes
            .iter()
            .any(|probe| probe.side == Classification::Positive)
    );
}

#[test]
fn strict_normal_probe_direct_seed_phase_keeps_certified_probe_for_slanted_leaf_case_unrestricted()
{
    let mut leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    leaf.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let interior = certified_leaf_interior_points(&leaf.support, &leaf.edges)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("slanted leaf should have a replayable interior witness");
    let (stop_values, saw_unknown) = adjacent_normal_probe_stop_values_with_queries(
        &interior.point,
        &leaf.support.normal,
        &leaf.support,
        &bounds,
        &[leaf.clone()],
        &mut |_interior, direction, polygon| Ok(dot_direction(&polygon.support.normal, direction)),
        &mut |point, polygon| classify_point_in_polygon(point, polygon),
    )
    .unwrap();

    assert!(!saw_unknown);
    let stop_t = stop_values[0].clone();
    let stop_point = offset_point(&interior.point, &leaf.support.normal, &stop_t);
    let corridor = bounds_between_points(&interior.point, &stop_point).unwrap();

    let mut halfspaces = aabb_core_halfspaces(&corridor).unwrap();
    halfspaces.push(support_side_halfspace(&leaf.support, true));
    halfspaces.push(normal_stop_halfspace(&leaf.support, &stop_point, true));
    let (report, mut saw_unknown) = optional_halfspace_feasibility_report(&halfspaces).unwrap();
    assert!(!saw_unknown);
    let (seeds, shifted_vertices, shifted_geometry_seeds) =
        halfspace_cell_seed_families_from_optional_report(
            &corridor,
            &halfspaces,
            report.as_ref(),
            &mut saw_unknown,
        )
        .unwrap();

    let probes = strict_normal_probe_targets_from_seed_families_with_tracking_unknown(
        &interior,
        &leaf.support,
        &corridor,
        None,
        &stop_point,
        true,
        report.as_ref(),
        seeds,
        shifted_vertices,
        shifted_geometry_seeds,
        |_seed| Ok(Vec::new()),
    )
    .unwrap();

    assert!(probes.iter().any(|probe| {
        probe.side == Classification::Positive && !probe.uncertified_definition_fallback
    }));
}

#[test]
fn strict_leaf_cell_points_retain_replayable_planes() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let center = p(1, 1, 1);

    let interior = strict_leaf_cell_points(&leaf, &center)
        .unwrap()
        .into_iter()
        .find(|point| !point.planes.is_empty())
        .expect("strict leaf halfspaces should have a feasible witness");

    assert!(point_strictly_inside_leaf(&interior.point, &leaf).unwrap());
    assert!(!interior.planes.is_empty());
    let planes = &interior.planes[0];
    assert_eq!(affine_from_planes(planes).unwrap(), interior.point);
    assert_eq!(planes[0], leaf.support);
}

#[test]
fn strict_leaf_cell_points_include_shifted_leaf_vertices() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let center = p(1, 1, 1);
    let vertices = leaf.vertices().unwrap();
    let bounds = leaf_bounds(&vertices).unwrap();
    let half = (Real::one() / Real::from(2)).unwrap();
    let mut halfspaces = vec![
        limit_plane_from_plane(&leaf.support),
        limit_plane_from_plane(&leaf.support.inverted()),
    ];
    for edge in leaf.edges.iter() {
        let margin = edge.expression_at_point(&center);
        halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
            edge, &margin, &half,
        )));
    }

    let report = halfspace_feasibility_report(&halfspaces).unwrap();
    let report_witness = report.witness.clone();
    let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();
    let mut direct_points = Vec::new();
    for seed in seeds {
        let active_planes = if report_witness.as_ref().is_some_and(|point| point == &seed) {
            report.active_planes
        } else {
            [None, None, None]
        };
        if let Some(point) =
            build_strict_leaf_point(&leaf, &seed, &halfspaces, active_planes, false).unwrap()
        {
            direct_points.push(point.point);
        }
    }

    let interiors = strict_leaf_cell_points(&leaf, &center).unwrap();
    let shifted = interiors
        .iter()
        .find(|point| !direct_points.iter().any(|direct| direct == &point.point))
        .expect("shifted strict leaf witness family should extend direct seed points");

    assert!(!shifted.planes.is_empty());
}

#[test]
fn strict_leaf_cell_points_merge_same_point_certified_shifted_replay_definitions() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let center = p(1, 1, 1);
    let vertices = leaf.vertices().unwrap();
    let bounds = leaf_bounds(&vertices).unwrap();
    let half = (Real::one() / Real::from(2)).unwrap();
    let mut halfspaces = vec![
        limit_plane_from_plane(&leaf.support),
        limit_plane_from_plane(&leaf.support.inverted()),
    ];
    for edge in leaf.edges.iter() {
        let margin = edge.expression_at_point(&center);
        halfspaces.push(limit_plane_from_plane(&inward_shifted_edge_plane(
            edge, &margin, &half,
        )));
    }

    let report = halfspace_feasibility_report(&halfspaces).unwrap();
    let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();
    let witness = seeds[0].clone();
    let extra_definition = [
        leaf.support.clone(),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let visited = std::cell::RefCell::new(Vec::new());

    let interiors = strict_leaf_cell_points_from_seed_families_with_tracking_unknown(
        &leaf,
        &center,
        Some(&report),
        vec![witness.clone()],
        Vec::new(),
        Vec::new(),
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ShiftedHalfspaceWitness {
                point: seed.clone(),
                families: vec![ShiftedHalfspaceWitnessFamily {
                    halfspaces: vec![axis_halfspace(1, false, r(1))],
                    active_planes: [Some(0), None, None],
                }],
                uncertified_definition_fallback: false,
            }])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness.clone()]);
    let interior = interiors
        .iter()
        .find(|point| point.point == witness)
        .expect("same-point shifted replay should keep the direct strict leaf point and enrich it");
    assert!(!interior.uncertified_definition_fallback);
    assert!(
        interior
            .planes
            .iter()
            .any(|definition| { definition_planes_match_as_sets(definition, &extra_definition) })
    );
}

#[test]
fn normal_probe_extra_planes_only_keep_selected_definition_planes() {
    let support = Plane::axis_aligned(2, r(0));
    let first = [
        support.clone(),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let second = [
        support.clone(),
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(1, r(2)),
    ];
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![first.clone(), second.clone()],
        uncertified_definition_fallback: false,
    };

    let extra_planes = normal_probe_extra_planes(&interior, Some(&first));

    assert_eq!(extra_planes.len(), 2);
    assert!(extra_planes.iter().any(|plane| plane == &first[1]));
    assert!(extra_planes.iter().any(|plane| plane == &first[2]));
    assert!(
        extra_planes
            .iter()
            .all(|plane| plane != &second[1] && plane != &second[2])
    );
}

#[test]
fn normal_probe_extra_planes_leave_unrestricted_family_empty() {
    let support = Plane::axis_aligned(2, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![[
            support,
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
        ]],
        uncertified_definition_fallback: false,
    };

    assert!(normal_probe_extra_planes(&interior, None).is_empty());
}

#[test]
fn normal_probe_shifted_seed_families_keep_only_report_root_after_certified_direct_probe() {
    let report_witness = p(9, 9, 9);
    let direct_probe_point = p(1, 1, 1);
    let shifted_vertex = p(2, 2, 2);
    let shifted_geometry = p(3, 3, 3);

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            None,
            Some(&report_witness),
            std::slice::from_ref(&direct_probe_point),
            vec![direct_probe_point.clone()],
            vec![shifted_vertex],
            vec![shifted_geometry],
        );

    assert_eq!(strict_shift_seeds, vec![report_witness]);
    assert!(shifted_vertices.is_empty());
    assert!(shifted_geometry_seeds.is_empty());
}

#[test]
fn normal_probe_shifted_seed_families_fall_back_to_first_certified_probe_without_report() {
    let direct_probe_point = p(1, 1, 1);
    let shifted_vertex = p(2, 2, 2);
    let shifted_geometry = p(3, 3, 3);

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            None,
            None,
            std::slice::from_ref(&direct_probe_point),
            vec![direct_probe_point.clone()],
            vec![shifted_vertex],
            vec![shifted_geometry],
        );

    assert_eq!(strict_shift_seeds, vec![direct_probe_point]);
    assert!(shifted_vertices.is_empty());
    assert!(shifted_geometry_seeds.is_empty());
}

#[test]
fn normal_probe_shifted_seed_families_keep_raw_roots_without_certified_direct_probe() {
    let shifted_vertex = p(2, 2, 2);
    let shifted_geometry = p(3, 3, 3);

    let (strict_shift_seeds, shifted_vertices, shifted_geometry_seeds) =
        normal_probe_shifted_seed_families(
            None,
            None,
            &[],
            Vec::new(),
            vec![shifted_vertex.clone()],
            vec![shifted_geometry.clone()],
        );

    assert!(strict_shift_seeds.is_empty());
    assert_eq!(shifted_vertices, vec![shifted_vertex]);
    assert_eq!(shifted_geometry_seeds, vec![shifted_geometry]);
}

#[test]
fn strict_leaf_cell_points_return_only_strict_points() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let center = p(1, 1, 1);

    let interiors = strict_leaf_cell_points(&leaf, &center).unwrap();

    assert!(!interiors.is_empty());
    for interior in &interiors {
        assert!(point_strictly_inside_leaf(&interior.point, &leaf).unwrap());
    }
}

#[test]
fn strict_leaf_witness_points_include_shifted_leaf_vertices() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let vertices = leaf.vertices().unwrap();

    let interiors = strict_leaf_witness_points(&leaf, &vertices).unwrap();

    assert!(
        interiors
            .iter()
            .any(|point| point.point == Point3::new(q(1, 2), q(1, 2), r(2)))
    );
    assert!(interiors.iter().all(|point| !point.planes.is_empty()));
}

#[test]
fn strict_leaf_witness_points_extend_direct_family_with_stricter_leaf_cells() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let vertices = leaf.vertices().unwrap();
    let bounds = leaf_bounds(&vertices).unwrap();
    let halfspaces = leaf_halfspaces(&leaf);
    let report = halfspace_feasibility_report(&halfspaces).unwrap();
    let report_witness = report.witness.clone();
    let seeds = strict_halfspace_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    let mut direct_points = Vec::new();
    for seed in &seeds {
        let active_planes = if report_witness.as_ref().is_some_and(|point| point == seed) {
            report.active_planes
        } else {
            [None, None, None]
        };
        if let Some(point) =
            build_strict_leaf_point(&leaf, seed, &halfspaces, active_planes, false).unwrap()
        {
            direct_points.push(point.point);
        }
    }

    let mut stricter_points = Vec::new();
    for point in &direct_points {
        for stricter in strict_leaf_cell_points(&leaf, point).unwrap() {
            if !direct_points.iter().any(|direct| direct == &stricter.point) {
                stricter_points.push(stricter.point);
            }
        }
    }

    let interiors = strict_leaf_witness_points(&leaf, &vertices).unwrap();

    assert!(!stricter_points.is_empty());
    assert!(
        stricter_points
            .iter()
            .any(|point| interiors.iter().any(|interior| &interior.point == point))
    );
}

#[test]
fn strict_leaf_witness_points_merge_stricter_replay_definitions_with_family_uncertainty() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let vertices = leaf.vertices().unwrap();
    let witness = p(1, 1, 1);
    let extra_definition = [
        leaf.support.clone(),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];

    let interiors = strict_leaf_witness_points_with_seed_families_and_stricter_replay(
        &leaf,
        &vertices,
        &mut |_leaf, _vertices, _bounds, _halfspaces, _report| {
            Ok(LeafWitnessSeedFamilies {
                seeds: vec![witness.clone()],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: false,
            })
        },
        |_leaf, point| {
            Ok(vec![InteriorLeafPoint {
                point: point.clone(),
                planes: vec![axis_plane_definition(point), extra_definition.clone()],
                uncertified_definition_fallback: false,
            }])
        },
    )
    .unwrap();
    let merged = interiors
        .iter()
        .find(|point| point.point == witness)
        .expect("same-point stricter replay should survive witness aggregation");

    assert!(
        merged
            .planes
            .iter()
            .any(|candidate| { definition_planes_match_as_sets(candidate, &extra_definition) })
    );
    assert!(merged.uncertified_definition_fallback);
}

#[test]
fn strict_leaf_witness_points_try_shifted_search_from_report_witness_seed() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let vertices = leaf.vertices().unwrap();

    let interiors = strict_leaf_witness_points_with_seed_families(
        &leaf,
        &vertices,
        |_leaf, _vertices, _bounds, _halfspaces, _report| {
            Ok(LeafWitnessSeedFamilies {
                seeds: Vec::new(),
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: false,
            })
        },
    )
    .unwrap();

    assert!(!interiors.is_empty());
    assert!(
        interiors
            .iter()
            .any(|point| point.point == Point3::new(q(1, 2), q(1, 2), r(2)))
    );
}

#[test]
fn interior_leaf_point_collection_backtracks_after_uncertified_candidate() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            if *candidate == first {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(vec![InteriorLeafPoint {
                    point: candidate.clone(),
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: false,
                }])
            }
        },
    )
    .unwrap();

    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point, second);
    assert!(points[0].uncertified_definition_fallback);
}

#[test]
fn interior_leaf_point_collection_marks_existing_points_uncertain_after_later_unknown() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            if *candidate == second {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(vec![InteriorLeafPoint {
                    point: candidate.clone(),
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: false,
                }])
            }
        },
    )
    .unwrap();

    assert_eq!(points.len(), 1);
    assert!(points[0].uncertified_definition_fallback);
}

#[test]
fn interior_leaf_point_collection_marks_later_points_uncertain_after_uncertain_candidate_result() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            Ok(vec![InteriorLeafPoint {
                point: candidate.clone(),
                planes: vec![axis_plane_definition(candidate)],
                uncertified_definition_fallback: *candidate == first,
            }])
        },
    )
    .unwrap();

    assert_eq!(points.len(), 2);
    assert!(
        points
            .iter()
            .all(|point| point.uncertified_definition_fallback)
    );
}

#[test]
fn interior_leaf_point_collection_keeps_certified_duplicate_state_certified() {
    let mut points = Vec::new();
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);

    extend_interior_leaf_points_backtracking_unknown(&mut points, [0, 1].iter(), |candidate| {
        Ok(vec![InteriorLeafPoint {
            point: point.clone(),
            planes: vec![definition.clone()],
            uncertified_definition_fallback: *candidate == 0,
        }])
    })
    .unwrap();

    assert_eq!(points.len(), 1);
    assert!(!points[0].uncertified_definition_fallback);
}

#[test]
fn interior_leaf_point_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    let err = extend_interior_leaf_points_backtracking_unknown(
        &mut points,
        [first, second].iter(),
        |_candidate| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn leaf_point_build_collection_backtracks_after_uncertified_candidate() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            if *candidate == first {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(InteriorLeafPoint {
                    point: candidate.clone(),
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: false,
                }))
            }
        },
    )
    .unwrap();

    assert_eq!(points.len(), 1);
    assert_eq!(points[0].point, second);
    assert!(points[0].uncertified_definition_fallback);
}

#[test]
fn leaf_point_build_collection_marks_existing_points_uncertain_after_later_unknown() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            if *candidate == second {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(InteriorLeafPoint {
                    point: candidate.clone(),
                    planes: vec![axis_plane_definition(candidate)],
                    uncertified_definition_fallback: false,
                }))
            }
        },
    )
    .unwrap();

    assert_eq!(points.len(), 1);
    assert!(points[0].uncertified_definition_fallback);
}

#[test]
fn leaf_point_build_collection_marks_later_points_uncertain_after_uncertain_candidate_result() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        [first.clone(), second.clone()].iter(),
        |candidate| {
            Ok(Some(InteriorLeafPoint {
                point: candidate.clone(),
                planes: vec![axis_plane_definition(candidate)],
                uncertified_definition_fallback: *candidate == first,
            }))
        },
    )
    .unwrap();

    assert_eq!(points.len(), 2);
    assert!(
        points
            .iter()
            .all(|point| point.uncertified_definition_fallback)
    );
}

#[test]
fn leaf_point_build_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let mut points = Vec::new();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    let err = extend_leaf_point_builds_backtracking_unknown(
        &mut points,
        [first, second].iter(),
        |_candidate| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn certified_leaf_test_point_prefers_replayable_interior_witness() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let expected_points = interior_leaf_points(&leaf)
        .unwrap()
        .into_iter()
        .filter(|point| !point.planes.is_empty())
        .map(|point| point.point)
        .collect::<Vec<_>>();

    let point = certified_leaf_test_point(&leaf.support, &leaf.edges)
        .unwrap()
        .expect("triangle leaf should have a certified strict interior point")
        .to_affine_point()
        .unwrap();

    assert!(!expected_points.is_empty());
    assert!(expected_points.iter().any(|expected| expected == &point));
}

#[test]
fn interior_leaf_points_drop_naked_centroid_when_replayable_points_exist() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);

    let points = interior_leaf_points(&leaf).unwrap();

    assert!(!points.is_empty());
    assert!(points.iter().all(|point| !point.planes.is_empty()));
}

#[test]
fn leaf_interior_definitions_include_non_basis_active_halfspaces() {
    let witness = p(1, 1, 1);
    let support = Plane::axis_aligned(2, r(1));
    let halfspaces = vec![
        limit_plane_from_plane(&support),
        limit_plane_from_plane(&support.inverted()),
        LimitPlane3::new(p(1, 0, 0), r(-1)),
        LimitPlane3::new(p(0, 1, 0), r(-1)),
        LimitPlane3::new(p(1, 1, 1), r(-3)),
    ];

    let definitions = leaf_interior_definitions_from_active_halfspaces(
        &witness,
        &support,
        &halfspaces,
        [Some(0), Some(2), Some(3)],
    )
    .unwrap();

    assert!(definitions.definitions.iter().any(|definition| {
        definition[1..]
            .iter()
            .any(|plane| plane.normal == p(1, 1, 1))
    }));
    for definition in &definitions.definitions {
        assert_eq!(definition[0], support);
        assert_eq!(affine_from_planes(definition).unwrap(), witness);
    }
}

#[test]
fn strict_leaf_witness_retains_axis_definition_when_active_replay_fails() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let witness = p(1, 1, 1);
    let halfspaces = vec![limit_plane_from_plane(&leaf.support)];

    let point = build_strict_leaf_point(&leaf, &witness, &halfspaces, [Some(9), None, None], false)
        .unwrap()
        .expect("strict witness should still be retained");

    assert_eq!(point.point, witness);
    assert!(!point.uncertified_definition_fallback);
    assert!(point.planes.iter().any(|definition| {
        definition[0] == leaf.support
            && definition[1..]
                .iter()
                .filter(|plane| {
                    plane.normal == p(1, 0, 0)
                        || plane.normal == p(0, 1, 0)
                        || plane.normal == p(0, 0, 1)
                })
                .count()
                == 2
    }));
}

#[test]
fn strict_leaf_witness_preserves_inherited_uncertified_definition_fallback() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let witness = p(1, 1, 1);
    let halfspaces = vec![limit_plane_from_plane(&leaf.support)];

    let point = build_strict_leaf_point(&leaf, &witness, &halfspaces, [None, None, None], true)
        .unwrap()
        .expect("strict witness should still be retained");

    assert!(point.uncertified_definition_fallback);
}

#[test]
fn strict_leaf_witness_reports_unknown_for_leaf_boundary_contact() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let witness = p(3, 0, 0);
    let halfspaces = vec![limit_plane_from_plane(&leaf.support)];

    assert_eq!(
        build_strict_leaf_point(&leaf, &witness, &halfspaces, [None, None, None], false),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_leaf_witness_points_mark_surviving_points_uncertain_after_seed_family_unknown() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let vertices = leaf.vertices().unwrap();

    let points = strict_leaf_witness_points_with_seed_families(
        &leaf,
        &vertices,
        |_leaf, _vertices, _bounds, _halfspaces, _report| {
            Ok(LeafWitnessSeedFamilies {
                seeds: vec![p(1, 1, 1)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: true,
            })
        },
    )
    .unwrap();

    assert!(points.iter().any(|point| point.point == p(1, 1, 1)));
    assert!(
        points
            .iter()
            .all(|point| point.uncertified_definition_fallback)
    );
}

#[test]
fn strict_leaf_witness_points_mark_surviving_points_uncertain_after_boundary_seed_candidate() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let vertices = leaf.vertices().unwrap();

    let points = strict_leaf_witness_points_with_seed_families(
        &leaf,
        &vertices,
        |leaf, _vertices, _bounds, _halfspaces, _report| {
            let boundary_family = collect_strict_halfspace_seed_family(
                Ok(vec![p(3, 0, 0), p(1, 1, 1)]),
                |candidate| point_strictly_inside_leaf_or_unknown(candidate, leaf),
            )?;
            Ok(LeafWitnessSeedFamilies {
                seeds: boundary_family.seeds,
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: boundary_family.saw_unknown,
            })
        },
    )
    .unwrap();

    assert!(points.iter().any(|point| point.point == p(1, 1, 1)));
    assert!(
        points
            .iter()
            .all(|point| point.uncertified_definition_fallback)
    );
}

#[test]
fn leaf_witness_seed_family_gate_allows_shifted_seed_sources_after_unknown_direct_family() {
    assert!(!seed_family_search_failed_without_any_seed(
        &[],
        &[p(1, 1, 1)],
        &[],
        true,
    ));
    assert!(!seed_family_search_failed_without_any_seed(
        &[],
        &[],
        &[p(1, 1, 1)],
        true,
    ));
}

#[test]
fn strict_leaf_witness_from_shifted_witness_merges_definition_families() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let witness = ShiftedHalfspaceWitness {
        point: p(1, 1, 1),
        families: vec![
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(1))],
                active_planes: [Some(0), None, None],
            },
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(1, false, r(1))],
                active_planes: [Some(0), None, None],
            },
        ],
        uncertified_definition_fallback: false,
    };

    let point = build_strict_leaf_point_from_shifted_witness(&leaf, &witness)
        .unwrap()
        .expect("shifted witness should still certify a strict leaf point");

    assert!(point.planes.iter().any(|definition| {
        definition
            .iter()
            .any(|plane| plane.normal == p(1, 0, 0) && plane.offset == r(-1))
    }));
    assert!(point.planes.iter().any(|definition| {
        definition
            .iter()
            .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
    }));
}

#[test]
fn strict_leaf_witness_from_shifted_witness_reports_unknown_for_leaf_boundary_contact() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let witness = ShiftedHalfspaceWitness {
        point: p(3, 0, 0),
        families: vec![ShiftedHalfspaceWitnessFamily {
            halfspaces: vec![axis_halfspace(0, false, r(3))],
            active_planes: [Some(0), None, None],
        }],
        uncertified_definition_fallback: false,
    };

    assert_eq!(
        build_strict_leaf_point_from_shifted_witness(&leaf, &witness),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_leaf_witness_from_shifted_witness_stays_certified_when_one_family_is_singular() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let witness = ShiftedHalfspaceWitness {
        point: p(1, 1, 1),
        families: vec![
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![
                    limit_plane_from_plane(&leaf.support),
                    axis_halfspace(0, false, r(1)),
                ],
                active_planes: [Some(9), None, None],
            },
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(1, false, r(1))],
                active_planes: [Some(0), None, None],
            },
        ],
        uncertified_definition_fallback: false,
    };

    let point = build_strict_leaf_point_from_shifted_witness(&leaf, &witness)
        .unwrap()
        .expect("shifted witness should still certify a strict leaf point");

    assert_eq!(point.point, witness.point);
    assert!(!point.uncertified_definition_fallback);
    assert!(!point.planes.is_empty());
}

#[test]
fn strict_leaf_witness_keeps_certified_replay_after_invalid_active_index() {
    let leaf = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 0, 0);
    let witness = p(1, 1, 1);
    let halfspaces = vec![
        limit_plane_from_plane(&leaf.support),
        axis_halfspace(0, false, r(1)),
    ];

    let point = build_strict_leaf_point(&leaf, &witness, &halfspaces, [Some(9), None, None], false)
        .unwrap()
        .expect("strict witness should still be retained");

    assert_eq!(point.point, witness);
    assert!(!point.uncertified_definition_fallback);
    assert!(point.planes.iter().any(|definition| {
        definition[1..]
            .iter()
            .any(|plane| plane.normal == p(1, 0, 0) && plane.offset == r(-1))
    }));
}

#[test]
fn witness_active_planes_return_report_planes_only_for_matching_witness() {
    let report_witness = p(1, 2, 3);
    let active_planes = [Some(4), Some(5), None];

    assert_eq!(
        witness_active_planes(Some(&report_witness), active_planes, &report_witness),
        active_planes
    );
    assert_eq!(
        witness_active_planes(Some(&report_witness), active_planes, &p(9, 9, 9)),
        [None, None, None]
    );
}

#[test]
fn probe_definitions_include_non_basis_active_halfspaces() {
    let witness = p(1, 1, 1);
    let shifted_support = Plane::axis_aligned(2, r(1));
    let halfspaces = vec![
        LimitPlane3::new(p(1, 0, 0), r(-1)),
        LimitPlane3::new(p(0, 1, 0), r(-1)),
        LimitPlane3::new(p(1, 1, 1), r(-3)),
    ];

    let definitions = probe_definitions_from_active_halfspaces(
        &witness,
        &halfspaces,
        [Some(0), Some(1), None],
        &[shifted_support],
    )
    .unwrap();

    assert!(!definitions.saw_unknown);
    assert!(
        definitions
            .definitions
            .iter()
            .any(|definition| definition.iter().any(|plane| plane.normal == p(1, 1, 1)))
    );
    for definition in &definitions.definitions {
        assert_eq!(affine_from_planes(definition).unwrap(), witness);
    }
}

#[test]
fn probe_definitions_or_axis_falls_back_to_axis_definition() {
    let witness = p(1, 2, 3);

    let (definitions, used_fallback) =
        probe_definitions_or_axis(&witness, Err(HypermeshError::UnknownClassification)).unwrap();

    assert_eq!(definitions, vec![axis_plane_definition(&witness)]);
    assert!(used_fallback);
}

#[test]
fn strict_probe_witness_stays_certified_when_active_replay_is_singular() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = p(1, 1, 1);
    let halfspaces = vec![axis_halfspace(2, false, r(2))];

    let probe = build_probe_point(
        &witness,
        &corridor,
        &support,
        &halfspaces,
        [Some(9), None, None],
        &[],
        false,
    )
    .unwrap()
    .expect("strict probe witness should still be retained");

    assert_eq!(probe.point, witness);
    assert!(!probe.uncertified_definition_fallback);
    assert!(probe.planes.iter().any(|definition| {
        definition_planes_match_as_sets(definition, &axis_plane_definition(&probe.point))
    }));
}

#[test]
fn strict_probe_witness_preserves_inherited_uncertified_definition_fallback() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = p(1, 1, 1);
    let halfspaces = vec![axis_halfspace(2, false, r(2))];

    let probe = build_probe_point(
        &witness,
        &corridor,
        &support,
        &halfspaces,
        [None, None, None],
        &[],
        true,
    )
    .unwrap()
    .expect("strict probe witness should still be retained");

    assert!(probe.uncertified_definition_fallback);
}

#[test]
fn strict_probe_witness_reports_unknown_for_support_boundary_contact() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = p(1, 1, 0);
    let halfspaces = vec![axis_halfspace(0, false, r(2))];

    assert_eq!(
        build_probe_point(
            &witness,
            &corridor,
            &support,
            &halfspaces,
            [None, None, None],
            &[],
            false
        ),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_probe_witness_from_shifted_witness_merges_definition_families() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = ShiftedHalfspaceWitness {
        point: p(1, 1, 1),
        families: vec![
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(2))],
                active_planes: [Some(0), None, None],
            },
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(1, false, r(2))],
                active_planes: [Some(0), None, None],
            },
        ],
        uncertified_definition_fallback: false,
    };

    let probe = build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[])
        .unwrap()
        .expect("shifted witness should still certify a strict probe");

    assert!(probe.planes.iter().any(|definition| {
        definition
            .iter()
            .any(|plane| plane.normal == p(1, 0, 0) && plane.offset == r(-1))
    }));
    assert!(probe.planes.iter().any(|definition| {
        definition
            .iter()
            .any(|plane| plane.normal == p(0, 1, 0) && plane.offset == r(-1))
    }));
}

#[test]
fn strict_probe_witness_from_shifted_witness_reports_unknown_for_support_boundary_contact() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = ShiftedHalfspaceWitness {
        point: p(1, 1, 0),
        families: vec![ShiftedHalfspaceWitnessFamily {
            halfspaces: vec![axis_halfspace(0, false, r(2))],
            active_planes: [Some(0), None, None],
        }],
        uncertified_definition_fallback: false,
    };

    assert_eq!(
        build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_probe_witness_from_shifted_witness_stays_certified_when_one_family_is_singular() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = ShiftedHalfspaceWitness {
        point: p(1, 1, 1),
        families: vec![
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(2, false, r(2))],
                active_planes: [Some(9), None, None],
            },
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(2))],
                active_planes: [Some(0), None, None],
            },
        ],
        uncertified_definition_fallback: false,
    };

    let probe = build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[])
        .unwrap()
        .expect("shifted witness should still certify a strict probe");

    assert_eq!(probe.point, witness.point);
    assert!(!probe.uncertified_definition_fallback);
    assert!(!probe.planes.is_empty());
}

#[test]
fn strict_probe_witness_reports_unknown_for_halfspace_boundary_contact() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = p(1, 1, 1);
    let halfspaces = vec![axis_halfspace(2, false, r(1))];

    assert_eq!(
        build_probe_point(
            &witness,
            &corridor,
            &support,
            &halfspaces,
            [None, None, None],
            &[],
            false,
        ),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_axis_probe_witness_stays_certified_when_active_replay_is_singular() {
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![axis_plane_definition(&p(1, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let witness = p(2, 1, 1);
    let halfspaces = vec![axis_halfspace(0, false, r(3))];

    let probe = build_axis_probe_point(
        &witness,
        &interior,
        &corridor,
        &support,
        0,
        None,
        &halfspaces,
        [Some(9), None, None],
        false,
    )
    .unwrap()
    .expect("strict axis probe witness should still be retained");

    assert_eq!(probe.point, witness);
    assert!(!probe.uncertified_definition_fallback);
    assert!(probe.planes.iter().any(|definition| {
        definition_planes_match_as_sets(definition, &axis_plane_definition(&probe.point))
    }));
}

#[test]
fn strict_axis_probe_witness_preserves_inherited_uncertified_definition_fallback() {
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![axis_plane_definition(&p(1, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let witness = p(2, 1, 1);
    let halfspaces = vec![axis_halfspace(0, false, r(3))];

    let probe = build_axis_probe_point(
        &witness,
        &interior,
        &corridor,
        &support,
        0,
        None,
        &halfspaces,
        [None, None, None],
        true,
    )
    .unwrap()
    .expect("strict axis probe witness should still be retained");

    assert!(probe.uncertified_definition_fallback);
}

#[test]
fn strict_axis_probe_witness_reports_unknown_for_support_boundary_contact() {
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![axis_plane_definition(&p(1, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let witness = p(2, 1, 0);
    let halfspaces = vec![axis_halfspace(0, false, r(3))];

    assert_eq!(
        build_axis_probe_point(
            &witness,
            &interior,
            &corridor,
            &support,
            0,
            None,
            &halfspaces,
            [None, None, None],
            false,
        ),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_axis_probe_witness_from_shifted_witness_reports_unknown_for_support_boundary_contact() {
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![axis_plane_definition(&p(1, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let witness = ShiftedHalfspaceWitness {
        point: p(2, 1, 0),
        families: vec![ShiftedHalfspaceWitnessFamily {
            halfspaces: vec![axis_halfspace(0, false, r(3))],
            active_planes: [Some(0), None, None],
        }],
        uncertified_definition_fallback: false,
    };

    assert_eq!(
        build_axis_probe_point_from_shifted_witness(
            &witness, &interior, &corridor, &support, 0, None
        ),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_axis_probe_witness_from_shifted_witness_stays_certified_when_one_family_is_singular() {
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![axis_plane_definition(&p(1, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let witness = ShiftedHalfspaceWitness {
        point: p(2, 1, 1),
        families: vec![
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(0, false, r(3))],
                active_planes: [Some(9), None, None],
            },
            ShiftedHalfspaceWitnessFamily {
                halfspaces: vec![axis_halfspace(1, false, r(2))],
                active_planes: [Some(0), None, None],
            },
        ],
        uncertified_definition_fallback: false,
    };

    let probe = build_axis_probe_point_from_shifted_witness(
        &witness, &interior, &corridor, &support, 0, None,
    )
    .unwrap()
    .expect("shifted witness should still certify a strict axis probe");

    assert_eq!(probe.point, witness.point);
    assert!(!probe.uncertified_definition_fallback);
    assert!(!probe.planes.is_empty());
}

#[test]
fn strict_probe_witness_from_shifted_witness_reports_unknown_for_halfspace_boundary_contact() {
    let corridor = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let witness = ShiftedHalfspaceWitness {
        point: p(1, 1, 1),
        families: vec![ShiftedHalfspaceWitnessFamily {
            halfspaces: vec![axis_halfspace(0, false, r(1))],
            active_planes: [Some(0), None, None],
        }],
        uncertified_definition_fallback: false,
    };

    assert_eq!(
        build_probe_point_from_shifted_witness(&witness, &corridor, &support, &[]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn strict_axis_probe_witness_reports_unknown_for_halfspace_boundary_contact() {
    let corridor = Aabb::new(p(1, 0, 0), p(4, 3, 3));
    let support = Plane::axis_aligned(2, r(0));
    let interior = InteriorLeafPoint {
        point: p(1, 1, 0),
        planes: vec![axis_plane_definition(&p(1, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let witness = p(2, 1, 1);
    let halfspaces = vec![axis_halfspace(0, false, r(2))];

    assert_eq!(
        build_axis_probe_point(
            &witness,
            &interior,
            &corridor,
            &support,
            0,
            None,
            &halfspaces,
            [None, None, None],
            false,
        ),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn duplicate_probe_points_merge_plane_definitions() {
    let point = p(1, 1, 1);
    let first_definition = [
        Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let second_definition = [
        Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(2, r(1)),
    ];
    let mut probes = vec![ProbePoint {
        point: point.clone(),
        side: Classification::Positive,
        planes: vec![first_definition.clone()],
        uncertified_definition_fallback: false,
    }];

    push_unique_probe_point(
        &mut probes,
        ProbePoint {
            point,
            side: Classification::Positive,
            planes: vec![second_definition.clone()],
            uncertified_definition_fallback: false,
        },
    );
    push_unique_probe_point(
        &mut probes,
        ProbePoint {
            point: p(1, 1, 1),
            side: Classification::Positive,
            planes: vec![second_definition],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].planes.len(), 2);
}

#[test]
fn duplicate_probe_points_merge_permuted_plane_definitions() {
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let mut probes = vec![ProbePoint {
        point: point.clone(),
        side: Classification::Positive,
        planes: vec![definition],
        uncertified_definition_fallback: false,
    }];

    push_unique_probe_point(
        &mut probes,
        ProbePoint {
            point,
            side: Classification::Positive,
            planes: vec![permuted.clone()],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(probes.len(), 1);
    assert_eq!(probes[0].planes.len(), 1);
    assert!(definition_planes_match_as_sets(
        &probes[0].planes[0],
        &permuted
    ));
}

#[test]
fn duplicate_probe_points_prefer_certified_duplicate_definitions() {
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);
    let mut probes = vec![ProbePoint {
        point: point.clone(),
        side: Classification::Positive,
        planes: vec![definition.clone()],
        uncertified_definition_fallback: true,
    }];

    push_unique_probe_point(
        &mut probes,
        ProbePoint {
            point,
            side: Classification::Positive,
            planes: vec![definition],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(probes.len(), 1);
    assert!(!probes[0].uncertified_definition_fallback);
}

#[test]
fn duplicate_interior_points_merge_plane_definitions() {
    let point = p(1, 1, 1);
    let mut points = vec![InteriorLeafPoint {
        point: point.clone(),
        planes: Vec::new(),
        uncertified_definition_fallback: false,
    }];
    let first_definition = [
        Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(1)),
    ];
    let second_definition = [
        Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(2, r(1)),
    ];

    push_unique_interior_point(
        &mut points,
        InteriorLeafPoint {
            point: point.clone(),
            planes: vec![first_definition.clone()],
            uncertified_definition_fallback: false,
        },
    );
    push_unique_interior_point(
        &mut points,
        InteriorLeafPoint {
            point,
            planes: vec![second_definition.clone()],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(points.len(), 1);
    assert_eq!(points[0].planes, vec![first_definition, second_definition]);
}

#[test]
fn duplicate_interior_points_merge_permuted_plane_definitions() {
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let mut points = vec![InteriorLeafPoint {
        point: point.clone(),
        planes: vec![definition],
        uncertified_definition_fallback: false,
    }];

    push_unique_interior_point(
        &mut points,
        InteriorLeafPoint {
            point,
            planes: vec![permuted.clone()],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(points.len(), 1);
    assert_eq!(points[0].planes.len(), 1);
    assert!(definition_planes_match_as_sets(
        &points[0].planes[0],
        &permuted
    ));
}

#[test]
fn duplicate_interior_points_prefer_certified_duplicate_definitions() {
    let point = p(1, 1, 1);
    let definition = axis_plane_definition(&point);
    let mut points = vec![InteriorLeafPoint {
        point: point.clone(),
        planes: vec![definition.clone()],
        uncertified_definition_fallback: true,
    }];

    push_unique_interior_point(
        &mut points,
        InteriorLeafPoint {
            point,
            planes: vec![definition],
            uncertified_definition_fallback: false,
        },
    );

    assert_eq!(points.len(), 1);
    assert!(!points[0].uncertified_definition_fallback);
}

#[test]
fn plane_replacement_path_traces_certified_winding_steps() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let start = axis_plane_defined_point(&p(0, 0, 0));
    let end = axis_plane_defined_point(&p(2, 0, 0));

    let winding = trace_plane_replacement_path(&start.planes, &end.planes, &[0], &[wall]).unwrap();

    assert_eq!(winding, vec![-1]);
}

#[test]
fn retained_reference_definitions_try_later_plane_replacement_paths() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let invalid_start = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(2)),
    ];
    let valid_start = axis_plane_defined_point(&p(0, 0, 0));
    let end = axis_plane_defined_point(&p(2, 0, 0));

    let winding = trace_probe_from_reference_definitions(
        &p(0, 0, 0),
        &[invalid_start, valid_start.planes],
        &p(2, 0, 0),
        std::slice::from_ref(&end.planes),
        &[0],
        &[wall],
    )
    .unwrap();

    assert_eq!(winding, vec![-1]);
}

#[test]
fn retained_probe_definitions_try_later_plane_replacement_paths() {
    let ref_point = p(0, 0, 0);
    let ref_definitions = [[
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::from_coefficients(r(1), r(1), r(1), r(0)),
    ]];
    let invalid_probe_definition = [
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(0)),
    ];
    let probe = ProbePoint {
        point: p(2, 1, 0),
        side: Classification::Positive,
        planes: vec![invalid_probe_definition, axis_plane_definition(&p(2, 1, 0))],
        uncertified_definition_fallback: false,
    };
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];

    assert_eq!(
        trace_segment_without_detours(&ref_point, &probe.point, &[0], &[wall.clone()]),
        Err(HypermeshError::UnknownClassification)
    );

    let winding = trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap();

    assert_eq!(winding, vec![0]);
}

#[test]
fn retained_definition_segment_search_continues_after_uncertified_direct_family() {
    let ref_point = p(0, 0, 0);
    let ref_definitions = [[
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::from_coefficients(r(1), r(1), r(1), r(0)),
    ]];
    let invalid_probe_definition = [
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(0)),
    ];
    let probe_point = p(2, 1, 0);
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];

    assert_eq!(
        trace_segment_without_detours(&ref_point, &probe_point, &[0], &[wall.clone()]),
        Err(HypermeshError::UnknownClassification)
    );

    let winding = trace_segment_from_definitions_with_step_detoured_plane_replacement(
        &ref_point,
        &probe_point,
        &[0],
        &[wall],
        &ref_definitions,
        &[
            invalid_probe_definition,
            axis_plane_definition(&probe_point),
        ],
    )
    .unwrap();

    assert_eq!(winding, vec![0]);
}

#[test]
fn retained_plane_replacement_skips_mismatched_start_definition() {
    let start = p(0, 0, 0);
    let end = p(0, 1, 0);
    let mismatched_start = axis_plane_definition(&p(2, 0, 0));
    let end_definition = axis_plane_definition(&end);
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, 2, -2), &p(1, 0, 2), 0, 0);
    wall.delta_w = vec![1];
    let mut no_detour_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();

    let winding = trace_from_definition_sets_with_step_detoured_plane_replacement(
        &start,
        &[mismatched_start],
        &end,
        &[end_definition],
        &[0],
        &[wall],
        &mut no_detour_cache,
        &mut detour_target_cache,
        None,
    )
    .unwrap();

    assert_eq!(winding, vec![0]);
}

#[test]
fn definition_pair_trace_backtracks_after_uncertified_pair() {
    let start_unknown = axis_plane_definition(&p(0, 0, 0));
    let start_ok = axis_plane_definition(&p(1, 0, 0));
    let end = axis_plane_definition(&p(2, 0, 0));

    let traced = definition_pair_trace_backtracking_unknown(
        &[start_unknown.clone(), start_ok.clone()],
        std::slice::from_ref(&end),
        |start_definition, end_definition| {
            if start_definition == &start_unknown && end_definition == &end {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(vec![1])
            }
        },
    )
    .unwrap();

    assert_eq!(traced, Some(vec![1]));
}

#[test]
fn definition_pair_trace_reports_unknown_if_all_pairs_are_uncertified() {
    let start = axis_plane_definition(&p(0, 0, 0));
    let end = axis_plane_definition(&p(1, 0, 0));

    let err = definition_pair_trace_backtracking_unknown(
        std::slice::from_ref(&start),
        std::slice::from_ref(&end),
        |_start_definition, _end_definition| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn definition_pair_trace_search_skips_duplicate_definition_pairs() {
    let start_a = axis_plane_definition(&p(0, 0, 0));
    let start_b = axis_plane_definition(&p(1, 0, 0));
    let end = axis_plane_definition(&p(2, 0, 0));
    let mut trace_calls = 0;

    let traced = definition_pair_trace_backtracking_unknown(
        &[start_a.clone(), start_a.clone(), start_b.clone()],
        &[end.clone(), end.clone()],
        |start_definition, end_definition| {
            trace_calls += 1;
            if start_definition == &start_a && end_definition == &end {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(vec![1])
            }
        },
    )
    .unwrap();

    assert_eq!(traced, Some(vec![1]));
    assert_eq!(trace_calls, 2);
}

#[test]
fn definition_pair_trace_search_skips_permuted_definition_pairs() {
    let start_a = axis_plane_definition(&p(0, 0, 0));
    let start_a_permuted = [start_a[1].clone(), start_a[2].clone(), start_a[0].clone()];
    let start_b = axis_plane_definition(&p(1, 0, 0));
    let end = axis_plane_definition(&p(2, 0, 0));
    let end_permuted = [end[2].clone(), end[0].clone(), end[1].clone()];
    let mut trace_calls = 0;

    let traced = definition_pair_trace_backtracking_unknown(
        &[start_a.clone(), start_a_permuted, start_b.clone()],
        &[end.clone(), end_permuted],
        |start_definition, end_definition| {
            trace_calls += 1;
            if definition_planes_match_as_sets(start_definition, &start_a)
                && definition_planes_match_as_sets(end_definition, &end)
            {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(vec![1])
            }
        },
    )
    .unwrap();

    assert_eq!(traced, Some(vec![1]));
    assert_eq!(trace_calls, 2);
}

#[test]
fn detour_legs_retry_direct_paths_when_axis_order_fails() {
    let blockers = vec![
        make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0),
        make_triangle(&p(0, 1, 0), &p(1, 1, 0), &p(0, 2, 0), 0, 1),
        make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 0, 2),
    ];

    assert_eq!(
        trace_axis_ordered_paths(&p(0, 0, 0), &p(1, 1, 1), &[0], &blockers),
        Err(HypermeshError::UnknownClassification)
    );
    assert_eq!(
        trace_direct_segment(&p(0, 0, 0), &p(1, 1, 1), &[0], &blockers)
            .unwrap()
            .winding,
        vec![0]
    );

    let traced = trace_segment_via_detours_with_definitions_budget(
        &p(0, 0, 0),
        &p(2, 2, 2),
        &[0],
        &blockers,
        &[DetourTarget {
            point: p(1, 1, 1),
            definitions: vec![axis_plane_definition(&p(1, 1, 1))],
            uncertified_definition_fallback: false,
        }],
        &[axis_plane_definition(&p(0, 0, 0))],
        &[axis_plane_definition(&p(2, 2, 2))],
        1,
        &mut |start, end, winding, start_definitions, end_definitions| {
            trace_segment_with_definitions_no_detours(
                start,
                end,
                winding,
                &blockers,
                start_definitions,
                end_definitions,
            )
        },
        &mut |_start, _end| Ok(Vec::new()),
    )
    .unwrap();

    assert_eq!(traced, Some(vec![0]));
}

#[test]
fn detour_legs_retry_plane_replacement_from_detour_definitions() {
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];
    let detour = DetourTarget {
        point: p(2, 1, 0),
        definitions: vec![[
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        ]],
        uncertified_definition_fallback: false,
    };

    assert_eq!(
        trace_segment_without_detours(&p(0, 0, 0), &detour.point, &[0], &[wall.clone()])
            .unwrap_err(),
        HypermeshError::UnknownClassification
    );

    let traced = trace_segment_via_detours_with_definitions_budget(
        &p(0, 0, 0),
        &p(2, 2, 0),
        &[0],
        &[wall.clone()],
        &[detour],
        &[axis_plane_definition(&p(0, 0, 0))],
        &[axis_plane_definition(&p(2, 2, 0))],
        1,
        &mut |start, end, winding, start_definitions, end_definitions| {
            trace_segment_with_definitions_no_detours(
                start,
                end,
                winding,
                &[wall.clone()],
                start_definitions,
                end_definitions,
            )
        },
        &mut |_start, _end| Ok(Vec::new()),
    )
    .unwrap();

    assert_eq!(traced, Some(vec![0]));
}

#[test]
fn detour_legs_can_use_retained_start_definitions() {
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];
    let start = p(0, 0, 0);
    let end = p(2, 2, 0);
    let start_definitions = [[
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::from_coefficients(r(1), r(1), r(1), r(0)),
    ]];
    let detour = DetourTarget {
        point: p(2, 1, 0),
        definitions: vec![axis_plane_definition(&p(2, 1, 0))],
        uncertified_definition_fallback: false,
    };

    let without_retained_start = trace_segment_via_detours_with_definitions_budget(
        &start,
        &end,
        &[0],
        &[wall.clone()],
        std::slice::from_ref(&detour),
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        1,
        &mut |start, end, winding, start_definitions, end_definitions| {
            trace_segment_with_definitions_no_detours(
                start,
                end,
                winding,
                &[wall.clone()],
                start_definitions,
                end_definitions,
            )
        },
        &mut |_start, _end| Ok(Vec::new()),
    );
    assert_eq!(
        without_retained_start.unwrap_err(),
        HypermeshError::UnknownClassification
    );

    let with_retained_start = trace_segment_via_detours_with_definitions_budget(
        &start,
        &end,
        &[0],
        &[wall.clone()],
        &[detour],
        &start_definitions,
        &[axis_plane_definition(&end)],
        1,
        &mut |start, end, winding, start_definitions, end_definitions| {
            trace_segment_with_definitions_no_detours(
                start,
                end,
                winding,
                &[wall.clone()],
                start_definitions,
                end_definitions,
            )
        },
        &mut |_start, _end| Ok(Vec::new()),
    )
    .unwrap();

    assert_eq!(with_retained_start, Some(vec![0]));
}

#[test]
fn detour_search_continues_after_uncertified_no_detour_family() {
    let start = p(0, 0, 0);
    let detour_point = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour = DetourTarget {
        point: detour_point.clone(),
        definitions: vec![axis_plane_definition(&detour_point)],
        uncertified_definition_fallback: false,
    };

    let traced = trace_segment_from_definitions_with_budget_impl(
        &start,
        &end,
        &[0],
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        1,
        &mut |from, to, winding, _start_definitions, _end_definitions| {
            if *from == start && *to == end {
                Err(HypermeshError::UnknownClassification)
            } else if (*from == start && *to == detour_point)
                || (*from == detour_point && *to == end)
            {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        },
        &mut |from, to| {
            if *from == start && *to == end {
                Ok(vec![detour.clone()])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(traced, vec![0]);
}

#[test]
fn detour_search_reports_unknown_if_all_detours_are_uncertified() {
    let start = p(0, 0, 0);
    let detour_point = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour = DetourTarget {
        point: detour_point,
        definitions: vec![axis_plane_definition(&p(1, 0, 0))],
        uncertified_definition_fallback: false,
    };

    let err = trace_segment_via_detours_with_definitions_budget(
        &start,
        &end,
        &[0],
        &[],
        &[detour],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        1,
        &mut |_from, _to, _winding, _start_definitions, _end_definitions| {
            Err(HypermeshError::UnknownClassification)
        },
        &mut |_from, _to| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn detour_trace_reports_unknown_when_fallback_surface_detour_is_skipped() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let fallback_detour = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: true,
    };
    let polygons = vec![ConvexPolygon {
        support: Plane::axis_aligned(0, r(1)),
        edges: Vec::new().into(),
        mesh_index: 0,
        polygon_index: 0,
        delta_w: Vec::new(),
        approx_bounds: None,
    }];

    let err = trace_segment_via_detours_with_definitions_budget(
        &start,
        &end,
        &[0],
        &polygons,
        &[fallback_detour],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        1,
        &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
        &mut |_from, _to| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn detour_trace_reports_unknown_when_fallback_revisited_detour_is_skipped() {
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);
    let fallback_detour = DetourTarget {
        point: end.clone(),
        definitions: vec![axis_plane_definition(&end)],
        uncertified_definition_fallback: true,
    };

    let err = trace_segment_from_definitions_with_cycle_guard_impl(
        &start,
        &end,
        &[0],
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
        &mut |_from, _to| Ok(vec![fallback_detour.clone()]),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn detour_trace_cycle_guard_tries_later_detour_after_uncertified_surface_query() {
    let start = p(0, 0, 0);
    let first_detour = p(1, 0, 0);
    let second_detour = p(2, 0, 0);
    let end = p(3, 0, 0);
    let mut surface_cache = Vec::new();

    let winding = trace_segment_via_detours_with_cycle_guard_with_surface_query(
        &start,
        &end,
        &[0],
        &[],
        &[
            DetourTarget {
                point: first_detour.clone(),
                definitions: vec![axis_plane_definition(&first_detour)],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: second_detour.clone(),
                definitions: vec![axis_plane_definition(&second_detour)],
                uncertified_definition_fallback: false,
            },
        ],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut surface_cache,
        &mut |point| {
            if *point == first_detour {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        },
        &mut |_from, _to, winding, _start_definitions, _end_definitions| Ok(Some(winding.to_vec())),
        &mut |_from, _to| Ok(Vec::new()),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![0]));
}

#[test]
fn detour_trace_cycle_guard_allows_same_point_definition_transition_at_start() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = axis_plane_definition(&end);
    let lifted_start_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
    ];
    let winding = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
        &start,
        &end,
        &[5],
        &[],
        std::slice::from_ref(&start_definition),
        std::slice::from_ref(&end_definition),
        &initial_visited_definition_points(
            &start,
            std::slice::from_ref(&start_definition),
            &end,
            std::slice::from_ref(&end_definition),
        ),
        &mut Vec::new(),
        &mut |_point| Ok(false),
        &mut |from, to, winding, start_definitions, end_definitions| {
            if *from == start
                && *to == end
                && start_definitions == std::slice::from_ref(&start_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(None)
            } else if *from == start
                && *to == start
                && start_definitions == std::slice::from_ref(&start_definition)
                && end_definitions == std::slice::from_ref(&lifted_start_definition)
            {
                Ok(Some(winding.to_vec()))
            } else if *from == start
                && *to == end
                && start_definitions == std::slice::from_ref(&lifted_start_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(Some(vec![7]))
            } else {
                Ok(None)
            }
        },
        &mut |from, to| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: start.clone(),
                    definitions: vec![lifted_start_definition.clone()],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn detour_trace_cycle_guard_allows_same_point_definition_transition_on_surface() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = axis_plane_definition(&end);
    let lifted_start_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
    ];

    let winding = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
        &start,
        &end,
        &[5],
        &[],
        std::slice::from_ref(&start_definition),
        std::slice::from_ref(&end_definition),
        &initial_visited_definition_points(
            &start,
            std::slice::from_ref(&start_definition),
            &end,
            std::slice::from_ref(&end_definition),
        ),
        &mut Vec::new(),
        &mut |point| Ok(*point == start),
        &mut |from, to, winding, start_definitions, end_definitions| {
            if *from == start
                && *to == end
                && start_definitions == std::slice::from_ref(&start_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(None)
            } else if *from == start
                && *to == start
                && start_definitions == std::slice::from_ref(&start_definition)
                && end_definitions == std::slice::from_ref(&lifted_start_definition)
            {
                Ok(Some(winding.to_vec()))
            } else if *from == start
                && *to == end
                && start_definitions == std::slice::from_ref(&lifted_start_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(Some(vec![7]))
            } else {
                Ok(None)
            }
        },
        &mut |from, to| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: start.clone(),
                    definitions: vec![lifted_start_definition.clone()],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn detour_trace_cycle_guard_allows_revisiting_point_with_new_definitions() {
    let start = p(0, 0, 0);
    let shared = p(1, 0, 0);
    let mid = p(2, 0, 0);
    let end = p(3, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let shared_definition = axis_plane_definition(&shared);
    let mid_definition = axis_plane_definition(&mid);
    let end_definition = axis_plane_definition(&end);
    let lifted_shared_definition = [
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(-1)),
    ];

    let winding = trace_segment_from_definitions_with_cycle_guard_impl_with_surface_query(
        &start,
        &end,
        &[5],
        &[],
        std::slice::from_ref(&start_definition),
        std::slice::from_ref(&end_definition),
        &initial_visited_definition_points(
            &start,
            std::slice::from_ref(&start_definition),
            &end,
            std::slice::from_ref(&end_definition),
        ),
        &mut Vec::new(),
        &mut |_point| Ok(false),
        &mut |from, to, winding, start_definitions, end_definitions| {
            if *from == start
                && *to == end
                && start_definitions == std::slice::from_ref(&start_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(None)
            } else if *from == start
                && *to == shared
                && start_definitions == std::slice::from_ref(&start_definition)
                && end_definitions == std::slice::from_ref(&shared_definition)
            {
                Ok(Some(winding.to_vec()))
            } else if *from == shared
                && *to == end
                && start_definitions == std::slice::from_ref(&shared_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(None)
            } else if *from == shared
                && *to == mid
                && start_definitions == std::slice::from_ref(&shared_definition)
                && end_definitions == std::slice::from_ref(&mid_definition)
            {
                Ok(Some(winding.to_vec()))
            } else if *from == mid
                && *to == end
                && start_definitions == std::slice::from_ref(&mid_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(None)
            } else if *from == mid
                && *to == shared
                && start_definitions == std::slice::from_ref(&mid_definition)
                && end_definitions == std::slice::from_ref(&lifted_shared_definition)
            {
                Ok(Some(winding.to_vec()))
            } else if *from == shared
                && *to == end
                && start_definitions == std::slice::from_ref(&lifted_shared_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Ok(Some(vec![7]))
            } else {
                Ok(None)
            }
        },
        &mut |from, to| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: shared.clone(),
                    definitions: vec![shared_definition.clone()],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == shared && *to == end {
                Ok(vec![DetourTarget {
                    point: mid.clone(),
                    definitions: vec![mid_definition.clone()],
                    uncertified_definition_fallback: false,
                }])
            } else if *from == mid && *to == end {
                Ok(vec![DetourTarget {
                    point: shared.clone(),
                    definitions: vec![lifted_shared_definition.clone()],
                    uncertified_definition_fallback: false,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn detour_trace_cycle_guard_accepts_fallback_detour_after_both_legs_succeed() {
    let start = p(0, 0, 0);
    let fallback_detour = p(1, 0, 0);
    let certified_detour = p(2, 0, 0);
    let end = p(3, 0, 0);
    let mut surface_cache = Vec::new();

    let winding = trace_segment_via_detours_with_cycle_guard_with_surface_query(
        &start,
        &end,
        &[0],
        &[],
        &[
            DetourTarget {
                point: fallback_detour.clone(),
                definitions: vec![axis_plane_definition(&fallback_detour)],
                uncertified_definition_fallback: true,
            },
            DetourTarget {
                point: certified_detour.clone(),
                definitions: vec![axis_plane_definition(&certified_detour)],
                uncertified_definition_fallback: false,
            },
        ],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |_from, to, winding, _start_definitions, _end_definitions| {
            Ok(Some(vec![if *to == fallback_detour {
                winding[0] + 1
            } else {
                winding[0] + 2
            }]))
        },
        &mut |_from, _to| Ok(Vec::new()),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![3]));
}

#[test]
fn detour_trace_cycle_guard_accepts_only_fallback_detour_after_both_legs_succeed() {
    let start = p(0, 0, 0);
    let fallback_detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let mut surface_cache = Vec::new();

    let winding = trace_segment_via_detours_with_cycle_guard_with_surface_query(
        &start,
        &end,
        &[0],
        &[],
        &[DetourTarget {
            point: fallback_detour.clone(),
            definitions: vec![axis_plane_definition(&fallback_detour)],
            uncertified_definition_fallback: true,
        }],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |_from, _to, winding, _start_definitions, _end_definitions| Ok(Some(winding.to_vec())),
        &mut |_from, _to| Ok(Vec::new()),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![0]));
}

#[test]
fn detour_trace_cycle_guard_tries_later_detour_after_boundary_surface_query() {
    let start = p(0, 0, 0);
    let first_detour = p(1, 0, 0);
    let second_detour = p(2, 0, 1);
    let end = p(3, 0, 0);
    let polygon = make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0);
    let mut surface_cache = Vec::new();

    let winding = trace_segment_via_detours_with_cycle_guard_with_surface_query(
        &start,
        &end,
        &[0],
        std::slice::from_ref(&polygon),
        &[
            DetourTarget {
                point: first_detour.clone(),
                definitions: vec![axis_plane_definition(&first_detour)],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: second_detour.clone(),
                definitions: vec![axis_plane_definition(&second_detour)],
                uncertified_definition_fallback: false,
            },
        ],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut surface_cache,
        &mut |point| point_lies_on_traced_surface(point, std::slice::from_ref(&polygon)),
        &mut |_from, _to, winding, _start_definitions, _end_definitions| Ok(Some(winding.to_vec())),
        &mut |_from, _to| Ok(Vec::new()),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![0]));
}

#[test]
fn point_lies_on_traced_surface_reports_unknown_for_boundary_contact() {
    let polygon = make_triangle(&p(1, 0, 0), &p(2, 0, 0), &p(1, 1, 0), 0, 0);

    assert_eq!(
        point_lies_on_traced_surface(&p(1, 0, 0), std::slice::from_ref(&polygon)),
        Err(HypermeshError::UnknownClassification)
    );
    assert!(!point_lies_on_traced_surface(&p(3, 3, 0), &[polygon]).unwrap());
}

#[test]
fn detour_trace_cycle_guard_reports_unknown_when_surface_query_is_uncertified_and_later_detours_fail()
 {
    let start = p(0, 0, 0);
    let first_detour = p(1, 0, 0);
    let second_detour = p(2, 0, 0);
    let end = p(3, 0, 0);
    let mut surface_cache = Vec::new();

    let err = trace_segment_via_detours_with_cycle_guard_with_surface_query(
        &start,
        &end,
        &[0],
        &[],
        &[
            DetourTarget {
                point: first_detour.clone(),
                definitions: vec![axis_plane_definition(&first_detour)],
                uncertified_definition_fallback: false,
            },
            DetourTarget {
                point: second_detour.clone(),
                definitions: vec![axis_plane_definition(&second_detour)],
                uncertified_definition_fallback: false,
            },
        ],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut surface_cache,
        &mut |point| {
            if *point == first_detour {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        },
        &mut |_from, _to, _winding, _start_definitions, _end_definitions| Ok(None),
        &mut |_from, _to| Ok(Vec::new()),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn axis_defined_probes_retry_plane_replacement_from_reference_definitions() {
    let ref_point = p(0, 0, 0);
    let ref_definitions = [[
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::from_coefficients(r(1), r(1), r(1), r(0)),
    ]];
    let probe = ProbePoint {
        point: p(2, 1, 0),
        side: Classification::Positive,
        planes: Vec::new(),
        uncertified_definition_fallback: false,
    };
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];

    assert_eq!(
        trace_segment_without_detours(&ref_point, &probe.point, &[0], &[wall.clone()]),
        Err(HypermeshError::UnknownClassification)
    );

    let winding = trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap();

    assert_eq!(winding, vec![0]);
}

#[test]
fn probe_reachability_retries_plane_replacement_from_retained_definitions() {
    let host_support = Plane::axis_aligned(2, r(0));
    let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]],
        uncertified_definition_fallback: false,
    };
    let probe = ProbePoint {
        point: p(2, 1, 1),
        side: Classification::Positive,
        planes: vec![[
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-4)),
        ]],
        uncertified_definition_fallback: false,
    };

    assert_eq!(
        probe_reaches_adjacent_cell(
            &interior.point,
            &probe.point,
            &host_support,
            std::slice::from_ref(&blocker),
        ),
        Err(HypermeshError::UnknownClassification)
    );
    assert!(
        probe_reaches_adjacent_cell_from_interior(&interior, &probe, &host_support, &[blocker],)
            .unwrap()
    );
}

#[test]
fn probe_reaches_adjacent_cell_reports_unknown_for_boundary_crossing() {
    let host_support = Plane::axis_aligned(2, r(0));
    let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);

    assert_eq!(
        probe_reaches_adjacent_cell(&p(0, 0, 0), &p(2, 0, 0), &host_support, &[blocker]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn probe_polyline_classifies_internal_surface_vertex_from_incident_sides() {
    let host_support = Plane::axis_aligned(2, r(-10));
    let wall = make_triangle(&p(-4, 4, -4), &p(4, -4, -4), &p(0, 0, 4), 0, 0);

    assert!(
        !probe_polyline_reaches_adjacent_cell(
            &[p(-1, 0, 0), p(0, 0, 0), p(0, 1, 0)],
            &host_support,
            std::slice::from_ref(&wall),
        )
        .unwrap()
    );
    assert!(
        probe_polyline_reaches_adjacent_cell(
            &[p(-1, 0, 0), p(0, 0, 0), p(-1, 0, 1)],
            &host_support,
            &[wall],
        )
        .unwrap()
    );
}

#[test]
fn no_step_plane_replacement_classifies_axis_path_vertex_crossings_as_blocked() {
    let host_support = Plane::axis_aligned(2, r(5));
    let wall = make_triangle(&p(5, 1, 1), &p(5, 5, 9), &p(4, 5, 4), 0, 1);
    let start = Point3::new(q(5983, 1350), q(1787, 450), q(2431, 675));
    let end = Point3::new(q(6523, 1500), q(21217, 5400), q(271, 75));

    assert!(
        !plane_replacement_path_reaches_adjacent_cell_without_step_detours(
            &axis_plane_definition(&start),
            &axis_plane_definition(&end),
            &host_support,
            &[wall],
        )
        .unwrap()
    );
}

#[test]
fn probe_reaches_adjacent_cell_accepts_zero_length_clear_point() {
    let host_support = Plane::axis_aligned(2, r(0));

    assert!(probe_reaches_adjacent_cell(&p(1, 1, 1), &p(1, 1, 1), &host_support, &[]).unwrap());
}

#[test]
fn probe_reaches_adjacent_cell_reports_unknown_for_zero_length_surface_contact() {
    let host_support = Plane::axis_aligned(2, r(0));
    let blocker = make_triangle(&p(1, 0, 0), &p(1, -1, 1), &p(1, 1, 1), 0, 0);

    assert_eq!(
        probe_reaches_adjacent_cell(&p(1, 0, 0), &p(1, 0, 0), &host_support, &[blocker]),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn probe_reachability_definition_search_continues_after_uncertified_direct_check() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 1);

    assert!(
        probe_reaches_adjacent_cell_with_definition_search(
            &start,
            &end,
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            || Err(HypermeshError::UnknownClassification),
            |_start_definition, _end_definition| Ok(true),
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_definition_search_skips_mismatched_endpoint_definitions() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 1);
    let mismatched_start = axis_plane_definition(&p(2, 0, 0));
    let mut replacement_calls = 0;

    assert!(
        probe_reaches_adjacent_cell_with_definition_search(
            &start,
            &end,
            &[mismatched_start],
            &[axis_plane_definition(&end)],
            || Ok(false),
            |start_definition, end_definition| {
                replacement_calls += 1;
                assert_eq!(affine_from_planes(start_definition).unwrap(), start);
                assert_eq!(affine_from_planes(end_definition).unwrap(), end);
                Ok(true)
            },
        )
        .unwrap()
    );
    assert_eq!(replacement_calls, 1);
}

#[test]
fn probe_reachability_definition_search_continues_after_boundary_direct_check() {
    let host_support = Plane::axis_aligned(2, r(0));
    let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);

    assert!(
        probe_reaches_adjacent_cell_with_definition_search(
            &start,
            &end,
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            || {
                probe_reaches_adjacent_cell(
                    &start,
                    &end,
                    &host_support,
                    std::slice::from_ref(&blocker),
                )
            },
            |_start_definition, _end_definition| Ok(true),
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_definition_search_reports_unknown_when_direct_check_is_uncertified_and_replacements_fail()
 {
    let start = p(0, 0, 0);
    let end = p(1, 1, 1);

    let err = probe_reaches_adjacent_cell_with_definition_search(
        &start,
        &end,
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        || Err(HypermeshError::UnknownClassification),
        |_start_definition, _end_definition| Ok(false),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_reachability_definition_search_preferring_precheck_short_circuits_true_pair() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 1);
    let start_defs = [
        axis_plane_definition(&start),
        [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ],
    ];
    let end_defs = [
        axis_plane_definition(&end),
        [
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        ],
    ];
    let mut replacement_calls = 0;

    let reaches = probe_reaches_adjacent_cell_with_definition_search_preferring_precheck(
        &start,
        &end,
        &start_defs,
        &end_defs,
        || Ok(false),
        |start_definition, end_definition| {
            if definition_planes_match_as_sets(start_definition, &start_defs[1])
                && definition_planes_match_as_sets(end_definition, &end_defs[1])
            {
                Ok(true)
            } else {
                Ok(false)
            }
        },
        |_start_definition, _end_definition| {
            replacement_calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(reaches);
    assert_eq!(replacement_calls, 0);
}

#[test]
fn probe_reachability_definition_search_preferring_precheck_prioritizes_unknown_pairs() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 1);
    let start_defs = [
        axis_plane_definition(&start),
        [
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ],
    ];
    let end_defs = [
        axis_plane_definition(&end),
        [
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
        ],
    ];
    let mut seen_pairs = Vec::new();

    let reaches = probe_reaches_adjacent_cell_with_definition_search_preferring_precheck(
        &start,
        &end,
        &start_defs,
        &end_defs,
        || Ok(false),
        |start_definition, end_definition| {
            if definition_planes_match_as_sets(start_definition, &start_defs[1])
                && definition_planes_match_as_sets(end_definition, &end_defs[0])
            {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        },
        |start_definition, end_definition| {
            let start_index = if definition_planes_match_as_sets(start_definition, &start_defs[0]) {
                0
            } else {
                1
            };
            let end_index = if definition_planes_match_as_sets(end_definition, &end_defs[0]) {
                0
            } else {
                1
            };
            seen_pairs.push((start_index, end_index));
            Ok(start_index == 1 && end_index == 0)
        },
    )
    .unwrap();

    assert!(reaches);
    assert_eq!(seen_pairs.first().copied(), Some((1, 0)));
}

#[test]
fn probe_step_detour_helper_retries_lower_definition_trace() {
    let host_support = Plane::axis_aligned(2, r(0));
    let blocker = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let interior = InteriorLeafPoint {
        point: p(0, 0, 0),
        planes: vec![[
            Plane::axis_aligned(0, r(0)),
            Plane::axis_aligned(1, r(0)),
            Plane::from_coefficients(r(1), r(1), r(1), r(0)),
        ]],
        uncertified_definition_fallback: false,
    };
    let probe = ProbePoint {
        point: p(2, 1, 1),
        side: Classification::Positive,
        planes: vec![[
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(1)),
            Plane::from_coefficients(r(1), r(1), r(1), r(-4)),
        ]],
        uncertified_definition_fallback: false,
    };

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions(
            &interior.point,
            &probe.point,
            &host_support,
            &[blocker],
            &interior.planes,
            &probe.planes,
        )
        .unwrap()
    );
}

#[test]
fn probe_step_detour_runtime_allows_three_nested_detours() {
    let start = p(0, 0, 0);
    let detour_a = p(1, 0, 0);
    let detour_b = p(2, 0, 0);
    let detour_c = p(3, 0, 0);
    let end = p(4, 0, 0);
    let start_definitions = [axis_plane_definition(&start)];
    let end_definitions = [axis_plane_definition(&end)];
    let detour_a_definitions = [axis_plane_definition(&detour_a)];
    let detour_b_definitions = [axis_plane_definition(&detour_b)];
    let detour_c_definitions = [axis_plane_definition(&detour_c)];
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         start_definitions_arg: &[[Plane; 3]],
         end_definitions_arg: &[[Plane; 3]]| {
            Ok((*from == start
                && *to == detour_a
                && start_definitions_arg == start_definitions
                && end_definitions_arg == detour_a_definitions)
                || (*from == detour_a
                    && *to == detour_b
                    && start_definitions_arg == detour_a_definitions
                    && end_definitions_arg == detour_b_definitions)
                || (*from == detour_b
                    && *to == detour_c
                    && start_definitions_arg == detour_b_definitions
                    && end_definitions_arg == detour_c_definitions)
                || (*from == detour_c
                    && *to == end
                    && start_definitions_arg == detour_c_definitions
                    && end_definitions_arg == end_definitions))
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![DetourTarget {
                point: detour_c.clone(),
                definitions: vec![axis_plane_definition(&detour_c)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == start && *to == detour_c {
            Ok(vec![DetourTarget {
                point: detour_b.clone(),
                definitions: vec![axis_plane_definition(&detour_b)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == start && *to == detour_b {
            Ok(vec![DetourTarget {
                point: detour_a.clone(),
                definitions: vec![axis_plane_definition(&detour_a)],
                uncertified_definition_fallback: false,
            }])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(&start, &start_definitions, &end, &end_definitions,),
            &start_definitions,
            &end_definitions,
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );

    let mut surface_cache = Vec::new();
    assert!(
        probe_reaches_adjacent_cell_with_detours_breadth_first_with_surface_query(
            &start,
            &end,
            &start_definitions,
            &end_definitions,
            &[],
            &mut surface_cache,
            &mut |_point| Ok(false),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn breadth_first_probe_detours_try_shallow_sibling_before_deeper_branch() {
    let start = p(0, 0, 0);
    let good_inner = p(1, 0, 0);
    let good_outer = p(2, 0, 0);
    let bad_deep = p(7, 0, 0);
    let bad_inner = p(8, 0, 0);
    let bad_outer = p(9, 0, 0);
    let end = p(10, 0, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let mut expanded_bad_deep = false;
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            Ok((*from == start && *to == good_inner)
                || (*from == good_inner && *to == good_outer)
                || (*from == good_outer && *to == end)
                || (*from == bad_inner && *to == bad_outer)
                || (*from == bad_outer && *to == end))
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        let points = if *from == start && *to == end {
            vec![bad_outer.clone(), good_outer.clone()]
        } else if *from == start && *to == bad_outer {
            vec![bad_inner.clone()]
        } else if *from == start && *to == bad_inner {
            vec![bad_deep.clone()]
        } else if *from == start && *to == bad_deep {
            expanded_bad_deep = true;
            Vec::new()
        } else if *from == start && *to == good_outer {
            vec![good_inner.clone()]
        } else {
            Vec::new()
        };
        Ok(points
            .into_iter()
            .map(|point| DetourTarget {
                definitions: definitions(&point),
                point,
                uncertified_definition_fallback: false,
            })
            .collect())
    };
    let mut surface_cache = Vec::new();

    assert!(
        probe_reaches_adjacent_cell_with_detours_breadth_first_with_surface_query(
            &start,
            &end,
            &definitions(&start),
            &definitions(&end),
            &[],
            &mut surface_cache,
            &mut |_point| Ok(false),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
    assert!(!expanded_bad_deep);
}

#[test]
fn breadth_first_probe_detours_accept_fallback_only_complete_path() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let mut surface_cache = Vec::new();

    let reaches = probe_reaches_adjacent_cell_with_detours_breadth_first_with_surface_query(
        &start,
        &end,
        &definitions(&start),
        &definitions(&end),
        &[],
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |from, to, _start_definitions, _end_definitions| {
            Ok((*from == start && *to == detour) || (*from == detour && *to == end))
        },
        &mut |from, to| {
            if *from == start && *to == end {
                Ok(vec![DetourTarget {
                    point: detour.clone(),
                    definitions: definitions(&detour),
                    uncertified_definition_fallback: true,
                }])
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert!(reaches);
}

#[test]
fn breadth_first_probe_detours_preserve_unknown_after_failed_earlier_batch() {
    let start = p(0, 0, 0);
    let failed_detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let mut surface_cache = Vec::new();

    let err = probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
        &start,
        &end,
        &definitions(&start),
        &definitions(&end),
        &[],
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |_from, _to, _start_definitions, _end_definitions| Ok(false),
        &mut |from, to, batch_index| {
            if *from == start && *to == end {
                match batch_index {
                    0 => Ok(Some(vec![DetourTarget {
                        point: failed_detour.clone(),
                        definitions: definitions(&failed_detour),
                        uncertified_definition_fallback: false,
                    }])),
                    _ => Err(HypermeshError::UnknownClassification),
                }
            } else {
                Ok(None)
            }
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn breadth_first_probe_detours_prune_repeated_arrangement_cell() {
    let start = p(-2, -2, 0);
    let repeated_cell = p(-1, -1, 0);
    let end = p(2, 2, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let arrangement_planes = [Plane::axis_aligned(0, r(0)), Plane::axis_aligned(1, r(0))];
    let mut detour_calls = 0;
    let mut surface_cache = Vec::new();

    let reaches = probe_reaches_adjacent_cell_with_detours_breadth_first_with_surface_query(
        &start,
        &end,
        &definitions(&start),
        &definitions(&end),
        &arrangement_planes,
        &mut surface_cache,
        &mut |_point| panic!("repeated-cell target should be pruned before surface query"),
        &mut |_from, _to, _start_definitions, _end_definitions| Ok(false),
        &mut |_from, _to| {
            detour_calls += 1;
            Ok(vec![DetourTarget {
                point: repeated_cell.clone(),
                definitions: definitions(&repeated_cell),
                uncertified_definition_fallback: false,
            }])
        },
    )
    .unwrap();

    assert!(!reaches);
    assert_eq!(detour_calls, 1);
}

#[test]
fn breadth_first_probe_detours_do_not_reenqueue_cell_from_later_batch() {
    let start = p(-2, -2, 0);
    let first = p(-2, 2, 0);
    let duplicate = p(-1, 1, 0);
    let end = p(2, 2, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let arrangement_planes = [Plane::axis_aligned(0, r(0)), Plane::axis_aligned(1, r(0))];
    let mut traced_duplicate = false;
    let mut surface_cache = Vec::new();

    let reaches = probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
        &start,
        &end,
        &definitions(&start),
        &definitions(&end),
        &arrangement_planes,
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |from, to, _start_definitions, _end_definitions| {
            traced_duplicate |= *from == duplicate || *to == duplicate;
            Ok(false)
        },
        &mut |from, to, batch_index| {
            if *from == start && *to == end {
                Ok(match batch_index {
                    0 => Some(vec![DetourTarget {
                        point: first.clone(),
                        definitions: definitions(&first),
                        uncertified_definition_fallback: false,
                    }]),
                    1 => Some(vec![DetourTarget {
                        point: duplicate.clone(),
                        definitions: definitions(&duplicate),
                        uncertified_definition_fallback: false,
                    }]),
                    _ => None,
                })
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert!(!reaches);
    assert!(!traced_duplicate);
}

#[test]
fn breadth_first_probe_detours_resume_later_batch_before_deeper_path() {
    let start = p(0, 0, 0);
    let good = p(1, 0, 0);
    let bad_deep = p(7, 0, 0);
    let bad_inner = p(8, 0, 0);
    let bad_outer = p(9, 0, 0);
    let end = p(10, 0, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let target = |point: &Point3| DetourTarget {
        point: point.clone(),
        definitions: definitions(point),
        uncertified_definition_fallback: false,
    };
    let mut expanded_bad_deep = false;
    let mut surface_cache = Vec::new();

    assert!(
        probe_reaches_adjacent_cell_with_detour_batches_breadth_first_with_surface_query(
            &start,
            &end,
            &definitions(&start),
            &definitions(&end),
            &[],
            &mut surface_cache,
            &mut |_point| Ok(false),
            &mut |from, to, _start_definitions, _end_definitions| {
                Ok((*from == start && *to == good)
                    || (*from == good && *to == end)
                    || (*from == bad_inner && *to == bad_outer)
                    || (*from == bad_outer && *to == end))
            },
            &mut |from, to, batch_index| {
                if *from == start && *to == end {
                    Ok(match batch_index {
                        0 => Some(vec![target(&bad_outer)]),
                        1 => Some(vec![target(&good)]),
                        _ => None,
                    })
                } else if *from == start && *to == bad_outer {
                    Ok(match batch_index {
                        0 => Some(vec![target(&bad_inner)]),
                        _ => None,
                    })
                } else if *from == start && *to == bad_inner {
                    Ok(match batch_index {
                        0 => Some(vec![target(&bad_deep)]),
                        _ => None,
                    })
                } else if *from == start && *to == bad_deep {
                    expanded_bad_deep = true;
                    Ok(None)
                } else {
                    Ok(None)
                }
            },
        )
        .unwrap()
    );
    assert!(!expanded_bad_deep);
}

#[test]
fn breadth_first_trace_detours_propagate_winding_and_resume_later_batch() {
    let start = p(0, 0, 0);
    let good = p(1, 0, 0);
    let bad_deep = p(7, 0, 0);
    let bad_inner = p(8, 0, 0);
    let bad_outer = p(9, 0, 0);
    let end = p(10, 0, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let target = |point: &Point3| DetourTarget {
        point: point.clone(),
        definitions: definitions(point),
        uncertified_definition_fallback: false,
    };
    let mut expanded_bad_deep = false;
    let mut surface_cache = Vec::new();

    let winding = trace_segment_with_detour_batches_breadth_first_with_surface_query(
        &start,
        &end,
        &[0],
        &definitions(&start),
        &definitions(&end),
        &[],
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |from, to, winding, _start_definitions, _end_definitions| {
            if *from == start && *to == good && winding == [0] {
                Ok(Some(vec![1]))
            } else if *from == good && *to == end && winding == [1] {
                Ok(Some(vec![2]))
            } else {
                Ok(None)
            }
        },
        &mut |from, to, batch_index| {
            if *from == start && *to == end {
                Ok(match batch_index {
                    0 => Some(vec![target(&bad_outer)]),
                    1 => Some(vec![target(&good)]),
                    _ => None,
                })
            } else if *from == start && *to == bad_outer {
                Ok(match batch_index {
                    0 => Some(vec![target(&bad_inner)]),
                    _ => None,
                })
            } else if *from == start && *to == bad_inner {
                Ok(match batch_index {
                    0 => Some(vec![target(&bad_deep)]),
                    _ => None,
                })
            } else if *from == start && *to == bad_deep {
                expanded_bad_deep = true;
                Ok(None)
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![2]);
    assert!(!expanded_bad_deep);
}

#[test]
fn breadth_first_trace_detours_keep_later_distinct_arrangement_cell_path() {
    let start = p(-2, -2, 0);
    let first_bad = p(-2, 2, 0);
    let duplicate_bad = p(-1, 1, 0);
    let good = p(2, -2, 0);
    let end = p(2, 2, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let target = |point: &Point3| DetourTarget {
        point: point.clone(),
        definitions: definitions(point),
        uncertified_definition_fallback: false,
    };
    let arrangement_planes = [Plane::axis_aligned(0, r(0)), Plane::axis_aligned(1, r(0))];
    let mut traced_duplicate = false;
    let mut surface_cache = Vec::new();

    let winding = trace_segment_with_detour_batches_breadth_first_with_surface_query(
        &start,
        &end,
        &[0],
        &definitions(&start),
        &definitions(&end),
        &arrangement_planes,
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |from, to, winding, _start_definitions, _end_definitions| {
            if *from == duplicate_bad || *to == duplicate_bad {
                traced_duplicate = true;
            }
            if *from == start && *to == good && winding == [0] {
                Ok(Some(vec![1]))
            } else if *from == good && *to == end && winding == [1] {
                Ok(Some(vec![2]))
            } else {
                Ok(None)
            }
        },
        &mut |from, to, batch_index| {
            if *from == start && *to == end && batch_index == 0 {
                Ok(Some(vec![
                    target(&first_bad),
                    target(&duplicate_bad),
                    target(&good),
                ]))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![2]);
    assert!(!traced_duplicate);
}

#[test]
fn breadth_first_trace_detours_do_not_reenqueue_cell_from_later_batch() {
    let start = p(-2, -2, 0);
    let first = p(-2, 2, 0);
    let duplicate = p(-1, 1, 0);
    let end = p(2, 2, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let arrangement_planes = [Plane::axis_aligned(0, r(0)), Plane::axis_aligned(1, r(0))];
    let mut traced_duplicate = false;
    let mut surface_cache = Vec::new();

    let err = trace_segment_with_detour_batches_breadth_first_with_surface_query(
        &start,
        &end,
        &[0],
        &definitions(&start),
        &definitions(&end),
        &arrangement_planes,
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |from, to, _winding, _start_definitions, _end_definitions| {
            traced_duplicate |= *from == duplicate || *to == duplicate;
            Ok(None)
        },
        &mut |from, to, batch_index| {
            if *from == start && *to == end {
                Ok(match batch_index {
                    0 => Some(vec![DetourTarget {
                        point: first.clone(),
                        definitions: definitions(&first),
                        uncertified_definition_fallback: false,
                    }]),
                    1 => Some(vec![DetourTarget {
                        point: duplicate.clone(),
                        definitions: definitions(&duplicate),
                        uncertified_definition_fallback: false,
                    }]),
                    _ => None,
                })
            } else {
                Ok(None)
            }
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert!(!traced_duplicate);
}

#[test]
fn breadth_first_trace_detours_accept_fallback_only_complete_path() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let definitions = |point: &Point3| vec![axis_plane_definition(point)];
    let mut surface_cache = Vec::new();

    let winding = trace_segment_with_detour_batches_breadth_first_with_surface_query(
        &start,
        &end,
        &[0],
        &definitions(&start),
        &definitions(&end),
        &[],
        &mut surface_cache,
        &mut |_point| Ok(false),
        &mut |from, to, winding, _start_definitions, _end_definitions| {
            if (*from == start && *to == detour) || (*from == detour && *to == end) {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        },
        &mut |from, to, batch_index| {
            if *from == start && *to == end && batch_index == 0 {
                Ok(Some(vec![DetourTarget {
                    point: detour.clone(),
                    definitions: definitions(&detour),
                    uncertified_definition_fallback: true,
                }]))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![0]);
}

#[test]
fn probe_reachability_uses_geometry_seeded_arrangement_detour_replacement_leg() {
    let host_support = Plane::axis_aligned(2, r(0));
    let start = p(0, 0, 0);
    let end = p(4, 4, 4);
    let mut blockers = vec![
        make_triangle(&p(4, 0, 0), &p(5, 0, 0), &p(4, 1, 0), 0, 0),
        make_triangle(&p(0, 4, 0), &p(1, 4, 0), &p(0, 5, 0), 0, 1),
        make_triangle(&p(0, 0, 4), &p(1, 0, 4), &p(0, 1, 4), 0, 2),
    ];

    for (index, x) in [q(4, 3), r(2), q(8, 3)].into_iter().enumerate() {
        blockers.push(make_triangle(
            &px(x.clone(), -1, -1),
            &px(x.clone(), 5, -1),
            &px(x, 2, 5),
            0,
            3 + index as isize,
        ));
    }

    assert!(!probe_reaches_adjacent_cell(&start, &end, &host_support, &blockers).unwrap());
    assert!(
        probe_reaches_adjacent_cell_via_progressive_detours(
            &start,
            &end,
            &host_support,
            &blockers,
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
        )
        .unwrap()
    );
}

#[test]
fn recursive_probe_reachability_budget_retries_detour_legs() {
    let start = p(0, 0, 0);
    let inner = p(1, 0, 0);
    let outer = p(2, 0, 0);
    let end = p(3, 0, 0);
    let outer_target = DetourTarget {
        point: outer.clone(),
        definitions: vec![axis_plane_definition(&outer)],
        uncertified_definition_fallback: false,
    };
    let inner_target = DetourTarget {
        point: inner.clone(),
        definitions: vec![axis_plane_definition(&inner)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            Ok((*from == start && *to == inner)
                || (*from == inner && *to == outer)
                || (*from == outer && *to == end))
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![outer_target.clone()])
        } else if *from == start && *to == outer {
            Ok(vec![inner_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        !probe_reaches_adjacent_cell_with_definitions_budget_impl(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );

    assert!(
        probe_reaches_adjacent_cell_with_definitions_budget_impl(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            2,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_runtime_allows_three_nested_detours() {
    let start = p(0, 0, 0);
    let detour_a = p(1, 0, 0);
    let detour_b = p(2, 0, 0);
    let detour_c = p(3, 0, 0);
    let end = p(4, 0, 0);
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            Ok((*from == start && *to == detour_a)
                || (*from == detour_a && *to == detour_b)
                || (*from == detour_b && *to == detour_c)
                || (*from == detour_c && *to == end))
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![DetourTarget {
                point: detour_c.clone(),
                definitions: vec![axis_plane_definition(&detour_c)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == start && *to == detour_c {
            Ok(vec![DetourTarget {
                point: detour_b.clone(),
                definitions: vec![axis_plane_definition(&detour_b)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == start && *to == detour_b {
            Ok(vec![DetourTarget {
                point: detour_a.clone(),
                definitions: vec![axis_plane_definition(&detour_a)],
                uncertified_definition_fallback: false,
            }])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        probe_reaches_adjacent_cell_with_cycle_guard_impl(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn probe_step_detour_cycle_guard_reports_unknown_when_fallback_detour_has_no_path() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: true,
    };
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| Ok(false);
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let err = probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
        &start,
        &end,
        &[],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &DefinitionNoPlaneReplacementReachabilityCache::default(),
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_step_detour_cycle_guard_reports_unknown_when_fallback_surface_detour_is_skipped() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: true,
    };
    let polygons = vec![ConvexPolygon {
        support: Plane::axis_aligned(0, r(1)),
        edges: Vec::new().into(),
        mesh_index: 0,
        polygon_index: 0,
        delta_w: Vec::new(),
        approx_bounds: None,
    }];
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| Ok(false);
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let err = probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
        &start,
        &end,
        &polygons,
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &DefinitionNoPlaneReplacementReachabilityCache::default(),
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_step_detour_cycle_guard_reports_unknown_when_fallback_revisited_detour_is_skipped() {
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: end.clone(),
        definitions: vec![axis_plane_definition(&end)],
        uncertified_definition_fallback: true,
    };
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| Ok(false);
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let err = probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl(
        &start,
        &end,
        &[],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &DefinitionNoPlaneReplacementReachabilityCache::default(),
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_step_detour_cycle_guard_tries_later_detour_after_uncertified_surface_query() {
    let start = p(0, 0, 0);
    let first_detour = p(1, 0, 0);
    let second_detour = p(2, 0, 0);
    let end = p(3, 0, 0);
    let mut surface_cache = Vec::new();

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &mut surface_cache,
            &mut |point| {
                if *point == first_detour {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut |_from, _to, _start_definitions, _end_definitions| Ok(true),
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![
                        DetourTarget {
                            point: first_detour.clone(),
                            definitions: vec![axis_plane_definition(&first_detour)],
                            uncertified_definition_fallback: false,
                        },
                        DetourTarget {
                            point: second_detour.clone(),
                            definitions: vec![axis_plane_definition(&second_detour)],
                            uncertified_definition_fallback: false,
                        },
                    ])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_step_detour_cycle_guard_allows_same_point_definition_transition_at_start() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = axis_plane_definition(&end);
    let lifted_start_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
    ];

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut |from, to, start_definitions, end_definitions| {
                Ok(
                    (*from == start
                        && *to == start
                        && start_definitions == std::slice::from_ref(&start_definition)
                        && end_definitions == std::slice::from_ref(&lifted_start_definition))
                        || (*from == start
                            && *to == end
                            && start_definitions
                                == std::slice::from_ref(&lifted_start_definition)
                            && end_definitions == std::slice::from_ref(&end_definition)),
                )
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: start.clone(),
                        definitions: vec![lifted_start_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_step_detour_cycle_guard_allows_same_point_definition_transition_on_surface() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = axis_plane_definition(&end);
    let lifted_start_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
    ];

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &mut Vec::new(),
            &mut |point| Ok(*point == start),
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut |from, to, start_definitions, end_definitions| {
                Ok(
                    (*from == start
                        && *to == start
                        && start_definitions == std::slice::from_ref(&start_definition)
                        && end_definitions == std::slice::from_ref(&lifted_start_definition))
                        || (*from == start
                            && *to == end
                            && start_definitions
                                == std::slice::from_ref(&lifted_start_definition)
                            && end_definitions == std::slice::from_ref(&end_definition)),
                )
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: start.clone(),
                        definitions: vec![lifted_start_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_step_detour_cycle_guard_allows_revisiting_point_with_new_definitions() {
    let start = p(0, 0, 0);
    let shared = p(1, 0, 0);
    let mid = p(2, 0, 0);
    let end = p(3, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let shared_definition = axis_plane_definition(&shared);
    let mid_definition = axis_plane_definition(&mid);
    let end_definition = axis_plane_definition(&end);
    let lifted_shared_definition = [
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(-1)),
    ];

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut |from, to, start_definitions, end_definitions| {
                Ok(
                    (*from == start
                        && *to == shared
                        && start_definitions == std::slice::from_ref(&start_definition)
                        && end_definitions == std::slice::from_ref(&shared_definition))
                        || (*from == shared
                            && *to == mid
                            && start_definitions
                                == std::slice::from_ref(&shared_definition)
                            && end_definitions == std::slice::from_ref(&mid_definition))
                        || (*from == mid
                            && *to == shared
                            && start_definitions == std::slice::from_ref(&mid_definition)
                            && end_definitions
                                == std::slice::from_ref(&lifted_shared_definition))
                        || (*from == shared
                            && *to == end
                            && start_definitions
                                == std::slice::from_ref(&lifted_shared_definition)
                            && end_definitions == std::slice::from_ref(&end_definition)),
                )
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: shared.clone(),
                        definitions: vec![shared_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else if *from == shared && *to == end {
                    Ok(vec![DetourTarget {
                        point: mid.clone(),
                        definitions: vec![mid_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else if *from == mid && *to == end {
                    Ok(vec![DetourTarget {
                        point: shared.clone(),
                        definitions: vec![lifted_shared_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_step_detour_cycle_guard_accepts_first_fallback_detour_after_path_succeeds() {
    let start = p(0, 0, 0);
    let fallback_detour = p(1, 0, 0);
    let certified_detour = p(2, 0, 0);
    let end = p(3, 0, 0);

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut |from, to, _start_definitions, _end_definitions| {
                if *from == start && *to == end {
                    Ok(false)
                } else {
                    Ok(true)
                }
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![
                        DetourTarget {
                            point: fallback_detour.clone(),
                            definitions: vec![axis_plane_definition(&fallback_detour)],
                            uncertified_definition_fallback: true,
                        },
                        DetourTarget {
                            point: certified_detour.clone(),
                            definitions: vec![axis_plane_definition(&certified_detour)],
                            uncertified_definition_fallback: false,
                        },
                    ])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_step_detour_cycle_guard_accepts_only_fallback_detour_after_path_succeeds() {
    let start = p(0, 0, 0);
    let fallback_detour = p(1, 0, 0);
    let end = p(2, 0, 0);

    let reaches =
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut |from, to, _start_definitions, _end_definitions| {
                if *from == start && *to == end {
                    Ok(false)
                } else {
                    Ok(true)
                }
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: fallback_detour.clone(),
                        definitions: vec![axis_plane_definition(&fallback_detour)],
                        uncertified_definition_fallback: true,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap();

    assert!(reaches);
}

#[test]
fn probe_step_detour_cycle_guard_reuses_surface_queries_across_failed_branches() {
    let start = p(0, 0, 0);
    let shared = p(1, 0, 0);
    let outer_b = p(2, 0, 0);
    let outer_a = p(3, 0, 0);
    let end = p(4, 0, 0);
    let outer_targets = vec![
        DetourTarget {
            point: outer_a.clone(),
            definitions: vec![axis_plane_definition(&outer_a)],
            uncertified_definition_fallback: false,
        },
        DetourTarget {
            point: outer_b.clone(),
            definitions: vec![axis_plane_definition(&outer_b)],
            uncertified_definition_fallback: false,
        },
    ];
    let shared_target = DetourTarget {
        point: shared.clone(),
        definitions: vec![axis_plane_definition(&shared)],
        uncertified_definition_fallback: false,
    };
    let mut surface_cache = Vec::new();
    let mut query_calls = 0;

    assert!(
        !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &mut surface_cache,
            &mut |_point| {
                query_calls += 1;
                Ok(false)
            },
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut |_from, _to, _start_definitions, _end_definitions| Ok(false),
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(outer_targets.clone())
                } else if *from == start && (*to == outer_a || *to == outer_b) {
                    Ok(vec![shared_target.clone()])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
    assert_eq!(query_calls, 3);
}

#[test]
fn probe_step_detour_entry_reuses_no_detour_and_detour_family_queries() {
    let start = p(0, 0, 0);
    let shared = p(1, 0, 0);
    let outer_b = p(2, 0, 0);
    let outer_a = p(3, 0, 0);
    let end = p(4, 0, 0);
    let outer_targets = vec![
        DetourTarget {
            point: outer_a.clone(),
            definitions: vec![axis_plane_definition(&outer_a)],
            uncertified_definition_fallback: false,
        },
        DetourTarget {
            point: outer_b.clone(),
            definitions: vec![axis_plane_definition(&outer_b)],
            uncertified_definition_fallback: false,
        },
    ];
    let shared_target = DetourTarget {
        point: shared.clone(),
        definitions: vec![axis_plane_definition(&shared)],
        uncertified_definition_fallback: false,
    };
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut shared_no_detour_calls = 0;
    let mut shared_detour_family_calls = 0;

    assert!(
        !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &mut no_detour_cache,
            &mut no_plane_replacement_cycle_guard_cache,
            &mut no_plane_replacement_cache,
            &mut halfspace_report_cache,
            &mut halfspace_seed_family_cache,
            &mut detour_target_cache,
            &mut interior_box_axis_intervals,
            |from: &Point3, to: &Point3, _start_definitions, _end_definitions| {
                if *from == start && *to == shared {
                    shared_no_detour_calls += 1;
                }
                Ok(false)
            },
            |from, to| {
                if *from == start && *to == end {
                    Ok(outer_targets.clone())
                } else if *from == start && (*to == outer_a || *to == outer_b) {
                    Ok(vec![shared_target.clone()])
                } else if *from == start && *to == shared {
                    shared_detour_family_calls += 1;
                    Ok(Vec::new())
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
    assert_eq!(shared_no_detour_calls, 1);
    assert_eq!(shared_detour_family_calls, 1);
}

#[test]
fn probe_reachability_from_definitions_shared_query_caches_reuse_equivalent_calls() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = [axis_plane_definition(&start)];
    let end_definitions = [axis_plane_definition(&end)];
    let mut no_detour_cache = DefinitionNoDetourReachabilityCache::default();
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let mut no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::default();
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut trace_calls = 0;

    let first =
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
            &start,
            &end,
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut no_plane_replacement_cycle_guard_cache,
            &mut no_plane_replacement_cache,
            &mut halfspace_report_cache,
            &mut halfspace_seed_family_cache,
            &mut detour_target_cache,
            &mut interior_box_axis_intervals,
            |_from, _to, _start_definitions, _end_definitions| {
                trace_calls += 1;
                Ok(true)
            },
            |_from, _to| Ok(Vec::new()),
        )
        .unwrap();
    assert_eq!(trace_calls, 1);
    let no_detour_len = no_detour_cache.len();
    let no_plane_replacement_cycle_guard_len = no_plane_replacement_cycle_guard_cache.len();
    let no_plane_replacement_len = no_plane_replacement_cache.len();
    let detour_len = detour_target_cache.len();
    let second =
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_from_definitions_with(
            &start,
            &end,
            &[],
            &start_definitions,
            &end_definitions,
            &mut no_detour_cache,
            &mut no_plane_replacement_cycle_guard_cache,
            &mut no_plane_replacement_cache,
            &mut halfspace_report_cache,
            &mut halfspace_seed_family_cache,
            &mut detour_target_cache,
            &mut interior_box_axis_intervals,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
            |_from, _to| Ok(Vec::new()),
        )
        .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(trace_calls, 1);
    assert_eq!(no_detour_cache.len(), no_detour_len);
    assert_eq!(
        no_plane_replacement_cycle_guard_cache.len(),
        no_plane_replacement_cycle_guard_len
    );
    assert_eq!(no_plane_replacement_cache.len(), no_plane_replacement_len);
    assert_eq!(detour_target_cache.len(), detour_len);
}

#[test]
fn no_plane_cycle_guard_reuses_cached_whole_query_false_across_visited_points() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let mid = p(0, 1, 0);
    let start_definitions = [axis_plane_definition(&start)];
    let end_definitions = [axis_plane_definition(&end)];
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::from(vec![
        DefinitionNoPlaneReplacementReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.to_vec(),
            end_definitions: end_definitions.to_vec(),
            result: Ok(false),
        },
    ]);
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut surface_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let visited_points = vec![VisitedDefinitionPoint {
        point: mid,
        definitions: vec![axis_plane_definition(&p(0, 1, 1))],
    }];
    let mut trace_calls = 0;
    let mut detour_calls = 0;

    let result =
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
            &start,
            &end,
            &[],
            &visited_points,
            &start_definitions,
            &end_definitions,
            false,
            &mut no_plane_replacement_cycle_guard_cache,
            &no_plane_replacement_cache,
            &mut halfspace_report_cache,
            &mut halfspace_seed_family_cache,
            &mut strict_aabb_target_families,
            &mut interior_box_axis_intervals,
            &mut surface_cache,
            &mut |_point| Ok(false),
            &mut |_from, _to, _start_defs, _end_defs| {
                trace_calls += 1;
                Ok(true)
            },
            &mut detour_target_cache,
            &mut |_from, _to| {
                detour_calls += 1;
                Ok(Vec::new())
            },
        );

    assert_eq!(result, Ok(false));
    assert_eq!(trace_calls, 0);
    assert_eq!(detour_calls, 0);
}

#[test]
fn no_plane_cycle_guard_reuses_cached_whole_query_true_for_initial_visited_points() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = [axis_plane_definition(&start)];
    let end_definitions = [axis_plane_definition(&end)];
    let mut no_plane_replacement_cycle_guard_cache =
        DefinitionNoPlaneReplacementCycleGuardCache::default();
    let no_plane_replacement_cache = DefinitionNoPlaneReplacementReachabilityCache::from(vec![
        DefinitionNoPlaneReplacementReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.to_vec(),
            end_definitions: end_definitions.to_vec(),
            result: Ok(true),
        },
    ]);
    let mut halfspace_report_cache = Vec::new();
    let mut halfspace_seed_family_cache = Vec::new();
    let mut strict_aabb_target_families = StrictAabbTargetFamilyCache::default();
    let mut interior_box_axis_intervals = InteriorBoxAxisIntervalsCache::default();
    let mut surface_cache = Vec::new();
    let mut detour_target_cache = DetourTargetFamilyCache::default();
    let visited_points =
        initial_visited_definition_points(&start, &start_definitions, &end, &end_definitions);
    let mut trace_calls = 0;
    let mut detour_calls = 0;

    let result =
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query_mode(
            &start,
            &end,
            &[],
            &visited_points,
            &start_definitions,
            &end_definitions,
            false,
            &mut no_plane_replacement_cycle_guard_cache,
            &no_plane_replacement_cache,
            &mut halfspace_report_cache,
            &mut halfspace_seed_family_cache,
            &mut strict_aabb_target_families,
            &mut interior_box_axis_intervals,
            &mut surface_cache,
            &mut |_point| Ok(false),
            &mut |_from, _to, _start_defs, _end_defs| {
                trace_calls += 1;
                Ok(false)
            },
            &mut detour_target_cache,
            &mut |_from, _to| {
                detour_calls += 1;
                Ok(Vec::new())
            },
        );

    assert_eq!(result, Ok(true));
    assert_eq!(trace_calls, 0);
    assert_eq!(detour_calls, 0);
}

#[test]
fn definition_no_plane_replacement_cycle_guard_cache_reuses_false_for_superset_visited_points() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let extra = p(0, 2, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let extra_definitions = vec![axis_plane_definition(&extra)];
    let cached_visited = vec![VisitedDefinitionPoint {
        point: shared.clone(),
        definitions: shared_definitions.clone(),
    }];
    let current_visited = vec![
        cached_visited[0].clone(),
        VisitedDefinitionPoint {
            point: extra,
            definitions: extra_definitions,
        },
    ];
    let cache = DefinitionNoPlaneReplacementCycleGuardCache::from(vec![
        DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(false),
        },
    ]);

    let reused = cached_definition_no_plane_replacement_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &current_visited,
    );

    assert_eq!(reused, Some(Ok(false)));
}

#[test]
fn definition_no_plane_replacement_cycle_guard_cache_reuses_true_for_subset_visited_points() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let extra = p(0, 2, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let extra_definitions = vec![axis_plane_definition(&extra)];
    let current_visited = vec![VisitedDefinitionPoint {
        point: shared.clone(),
        definitions: shared_definitions.clone(),
    }];
    let cached_visited = vec![
        current_visited[0].clone(),
        VisitedDefinitionPoint {
            point: extra,
            definitions: extra_definitions,
        },
    ];
    let cache = DefinitionNoPlaneReplacementCycleGuardCache::from(vec![
        DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(true),
        },
    ]);

    let reused = cached_definition_no_plane_replacement_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &current_visited,
    );

    assert_eq!(reused, Some(Ok(true)));
}

#[test]
fn definition_no_plane_replacement_cycle_guard_cache_reuses_reversed_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let visited_points = vec![
        VisitedDefinitionPoint {
            point: start.clone(),
            definitions: start_definitions.clone(),
        },
        VisitedDefinitionPoint {
            point: end.clone(),
            definitions: end_definitions.clone(),
        },
    ];
    let cache = DefinitionNoPlaneReplacementCycleGuardCache::from(vec![
        DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: visited_points.clone(),
            result: Ok(true),
        },
    ]);

    let reused = cached_definition_no_plane_replacement_cycle_guard_result(
        &cache,
        &end,
        &start,
        &end_definitions,
        &start_definitions,
        &visited_points,
    );

    assert_eq!(reused, Some(Ok(true)));
}

#[test]
fn definition_no_plane_replacement_cycle_guard_cache_ignores_redundant_current_endpoints() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let cache = DefinitionNoPlaneReplacementCycleGuardCache::from(vec![
        DefinitionNoPlaneReplacementCycleGuardCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: vec![VisitedDefinitionPoint {
                point: shared.clone(),
                definitions: shared_definitions.clone(),
            }],
            result: Ok(true),
        },
    ]);
    let current_visited = vec![
        VisitedDefinitionPoint {
            point: start.clone(),
            definitions: start_definitions.clone(),
        },
        VisitedDefinitionPoint {
            point: end.clone(),
            definitions: end_definitions.clone(),
        },
        VisitedDefinitionPoint {
            point: shared,
            definitions: shared_definitions,
        },
    ];

    let reused = cached_definition_no_plane_replacement_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &current_visited,
    );

    assert_eq!(reused, Some(Ok(true)));
}

#[test]
fn definition_no_plane_replacement_cycle_guard_cache_reuses_in_progress_exact_state() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let visited_points = vec![VisitedDefinitionPoint {
        point: shared,
        definitions: shared_definitions,
    }];
    let mut cache = DefinitionNoPlaneReplacementCycleGuardCache::default();
    let index = begin_definition_no_plane_replacement_cycle_guard_result(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &visited_points,
    );

    let reused = cached_definition_no_plane_replacement_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &visited_points,
    );

    assert_eq!(reused, Some(Err(HypermeshError::UnknownClassification)));
    assert_eq!(
        cache.entries[index].result,
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn definition_cycle_guard_cache_reuses_identical_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let visited_points = vec![VisitedDefinitionPoint {
        point: shared.clone(),
        definitions: shared_definitions,
    }];
    let cache = DefinitionCycleGuardReachabilityCache::from(vec![
        DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: visited_points.clone(),
            result: Ok(true),
        },
    ]);

    let reused = cached_definition_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &visited_points,
    );

    assert_eq!(reused, Some(Ok(true)));
}

#[test]
fn definition_cycle_guard_cache_reuses_reversed_query() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let visited_points = vec![VisitedDefinitionPoint {
        point: shared.clone(),
        definitions: vec![axis_plane_definition(&shared)],
    }];
    let cache = DefinitionCycleGuardReachabilityCache::from(vec![
        DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: visited_points.clone(),
            result: Ok(true),
        },
    ]);

    let reused = cached_definition_cycle_guard_result(
        &cache,
        &end,
        &start,
        &end_definitions,
        &start_definitions,
        &visited_points,
    );

    assert_eq!(reused, Some(Ok(true)));
}

#[test]
fn definition_cycle_guard_cache_ignores_redundant_current_endpoints() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let cache = DefinitionCycleGuardReachabilityCache::from(vec![
        DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: vec![VisitedDefinitionPoint {
                point: shared.clone(),
                definitions: shared_definitions.clone(),
            }],
            result: Err(HypermeshError::UnknownClassification),
        },
    ]);
    let current_visited = vec![
        VisitedDefinitionPoint {
            point: start.clone(),
            definitions: start_definitions.clone(),
        },
        VisitedDefinitionPoint {
            point: end.clone(),
            definitions: end_definitions.clone(),
        },
        VisitedDefinitionPoint {
            point: shared,
            definitions: shared_definitions,
        },
    ];

    let reused = cached_definition_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &current_visited,
    );

    assert_eq!(reused, Some(Err(HypermeshError::UnknownClassification)));
}

#[test]
fn definition_cycle_guard_cache_reuses_in_progress_exact_state() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let visited_points = vec![VisitedDefinitionPoint {
        point: shared,
        definitions: shared_definitions,
    }];
    let mut cache = DefinitionCycleGuardReachabilityCache::default();
    let index = begin_definition_cycle_guard_result(
        &mut cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &visited_points,
    );

    let reused = cached_definition_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &visited_points,
    );

    assert_eq!(reused, Some(Err(HypermeshError::UnknownClassification)));
    assert_eq!(
        cache.entries[index].result,
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn definition_cycle_guard_cache_reuses_false_for_superset_visited_points() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let extra = p(0, 2, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let extra_definitions = vec![axis_plane_definition(&extra)];
    let cached_visited = vec![VisitedDefinitionPoint {
        point: shared.clone(),
        definitions: shared_definitions.clone(),
    }];
    let current_visited = vec![
        cached_visited[0].clone(),
        VisitedDefinitionPoint {
            point: extra,
            definitions: extra_definitions,
        },
    ];
    let cache = DefinitionCycleGuardReachabilityCache::from(vec![
        DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(false),
        },
    ]);

    let reused = cached_definition_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &current_visited,
    );

    assert_eq!(reused, Some(Ok(false)));
}

#[test]
fn definition_cycle_guard_cache_reuses_true_for_subset_visited_points() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let shared = p(0, 1, 0);
    let extra = p(0, 2, 0);
    let start_definitions = vec![axis_plane_definition(&start)];
    let end_definitions = vec![axis_plane_definition(&end)];
    let shared_definitions = vec![axis_plane_definition(&shared)];
    let extra_definitions = vec![axis_plane_definition(&extra)];
    let current_visited = vec![VisitedDefinitionPoint {
        point: shared.clone(),
        definitions: shared_definitions.clone(),
    }];
    let cached_visited = vec![
        current_visited[0].clone(),
        VisitedDefinitionPoint {
            point: extra,
            definitions: extra_definitions,
        },
    ];
    let cache = DefinitionCycleGuardReachabilityCache::from(vec![
        DefinitionCycleGuardReachabilityCacheEntry {
            start: start.clone(),
            end: end.clone(),
            start_definitions: start_definitions.clone(),
            end_definitions: end_definitions.clone(),
            visited_points: cached_visited,
            result: Ok(true),
        },
    ]);

    let reused = cached_definition_cycle_guard_result(
        &cache,
        &start,
        &end,
        &start_definitions,
        &end_definitions,
        &current_visited,
    );

    assert_eq!(reused, Some(Ok(true)));
}

#[test]
fn ordered_interior_points_for_probe_search_prefers_axis_aligned_definition_planes() {
    let slanted = Plane {
        normal: Point3::new(r(2), r(3), r(5)),
        offset: r(7),
    };
    let more_slanted = Plane {
        normal: Point3::new(r(7), r(11), r(13)),
        offset: r(17),
    };
    let most_axis_aligned = InteriorLeafPoint {
        point: p(3, 0, 0),
        planes: vec![axis_plane_definition(&p(3, 0, 0))],
        uncertified_definition_fallback: false,
    };
    let partly_axis_aligned = InteriorLeafPoint {
        point: p(2, 0, 0),
        planes: vec![[
            Plane::axis_aligned(2, r(1)),
            slanted.clone(),
            more_slanted.clone(),
        ]],
        uncertified_definition_fallback: false,
    };
    let non_axis_aligned = InteriorLeafPoint {
        point: p(1, 0, 0),
        planes: vec![[slanted, more_slanted.clone(), more_slanted]],
        uncertified_definition_fallback: false,
    };

    let points = [
        non_axis_aligned.clone(),
        partly_axis_aligned.clone(),
        most_axis_aligned.clone(),
    ];
    let ordered = ordered_interior_points_for_probe_search(&points);

    assert_eq!(ordered[0].point, most_axis_aligned.point);
    assert_eq!(ordered[1].point, partly_axis_aligned.point);
    assert_eq!(ordered[2].point, non_axis_aligned.point);
}

#[test]
fn ordered_interior_points_for_probe_search_with_support_prefers_retained_definition_points_in_root_host_fixture()
 {
    use crate::mesh::prepare_input;
    use crate::polygon::ConvexPolygon;

    fn tetra_from_face_and_apex(a: Point3, b: Point3, c: Point3, apex: Point3) -> crate::InputMesh {
        crate::InputMesh::new(
            vec![a, b, c, apex],
            vec![
                crate::Triangle::new(0, 2, 1),
                crate::Triangle::new(0, 1, 3),
                crate::Triangle::new(0, 3, 2),
                crate::Triangle::new(1, 2, 3),
            ],
        )
    }

    fn face_at(
        polygons: &[ConvexPolygon],
        mesh_index: isize,
        polygon_index: isize,
    ) -> ConvexPolygon {
        polygons
            .iter()
            .find(|polygon| {
                polygon.mesh_index == mesh_index && polygon.polygon_index == polygon_index
            })
            .unwrap()
            .clone()
    }

    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let polygons = soup.polygons.clone();
    let host = face_at(&polygons, 0, 1);
    let intersections = polygons
        .iter()
        .enumerate()
        .filter_map(|(index, polygon)| {
            if polygon.mesh_index == host.mesh_index && polygon.polygon_index == host.polygon_index
            {
                return None;
            }
            let intersection =
                crate::intersection::intersect_polygons(&host, polygon, index).ok()?;
            Some(intersection)
        })
        .collect::<Vec<_>>();
    let bsp_leaves =
        crate::subdivision::build_host_bsp_leaves(&host, &polygons, &intersections).unwrap();

    let mut checked = 0;
    for (leaf_index, leaf) in bsp_leaves.iter().enumerate() {
        if leaf.edges.len() < 3 {
            continue;
        }
        let Ok((interior_points, _)) =
            crate::subdivision::certify_bsp_leaf_and_delta_w(&host, &leaf.edges, &polygons)
        else {
            continue;
        };
        let ordered =
            ordered_interior_points_for_probe_search_with_support(&interior_points, &host.support)
                .unwrap();
        let ordered_indices = ordered
            .iter()
            .map(|ordered_point| {
                interior_points
                    .iter()
                    .position(|point| point.point == ordered_point.point)
                    .unwrap()
            })
            .collect::<Vec<_>>();

        match leaf_index {
            1 => {
                assert_eq!(ordered_indices[0], 2);
                checked += 1;
            }
            2 => {
                assert_eq!(ordered_indices[0], 0);
                checked += 1;
            }
            _ => {}
        }
    }

    assert_eq!(checked, 2);
}

#[test]
fn probe_reachability_backtracks_after_uncertified_detour_leg() {
    let start = p(0, 0, 0);
    let blocked = p(1, 0, 0);
    let good = p(2, 0, 0);
    let end = p(3, 0, 0);
    let blocked_target = DetourTarget {
        point: blocked.clone(),
        definitions: vec![axis_plane_definition(&blocked)],
        uncertified_definition_fallback: false,
    };
    let good_target = DetourTarget {
        point: good.clone(),
        definitions: vec![axis_plane_definition(&good)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            if *from == start && *to == blocked {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok((*from == start && *to == good) || (*from == good && *to == end))
            }
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![blocked_target.clone(), good_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        probe_reaches_adjacent_cell_via_detours_with_budget(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_continues_after_uncertified_no_detour_family() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            if *from == start && *to == end {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok((*from == start && *to == detour) || (*from == detour && *to == end))
            }
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        probe_reaches_adjacent_cell_with_definitions_budget_impl(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_reports_unknown_when_no_detour_family_is_uncertified_and_detours_fail() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            if *from == start && *to == end {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let err = probe_reaches_adjacent_cell_with_definitions_budget_impl(
        &start,
        &end,
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        1,
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_reachability_reports_unknown_if_all_detours_are_uncertified() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| { Err(HypermeshError::UnknownClassification) };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert_eq!(
        probe_reaches_adjacent_cell_via_detours_with_budget(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        ),
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn probe_reachability_reports_unknown_when_fallback_detour_has_no_path() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: true,
    };
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| Ok(false);
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let err = probe_reaches_adjacent_cell_via_detours_with_budget(
        &start,
        &end,
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        1,
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_reachability_reports_unknown_when_fallback_surface_detour_is_skipped() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: true,
    };
    let polygons = vec![ConvexPolygon {
        support: Plane::axis_aligned(0, r(1)),
        edges: Vec::new().into(),
        mesh_index: 0,
        polygon_index: 0,
        delta_w: Vec::new(),
        approx_bounds: None,
    }];
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| Ok(false);
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let err = probe_reaches_adjacent_cell_via_detours_with_budget(
        &start,
        &end,
        &polygons,
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        1,
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_reachability_reports_unknown_when_fallback_revisited_detour_is_skipped() {
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: end.clone(),
        definitions: vec![axis_plane_definition(&end)],
        uncertified_definition_fallback: true,
    };
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| Ok(false);
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let err = probe_reaches_adjacent_cell_via_detours_with_cycle_guard(
        &start,
        &end,
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_reachability_cycle_guard_tries_detours_after_uncertified_direct_check() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour.clone(),
        definitions: vec![axis_plane_definition(&detour)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            if *from == start && *to == end {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(true)
            }
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_cycle_guard_reports_unknown_when_direct_check_is_uncertified_and_no_detour_succeeds()
 {
    let start = p(0, 0, 0);
    let end = p(2, 0, 0);
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| { Err(HypermeshError::UnknownClassification) };
    let mut detours_for = |_from: &Point3, _to: &Point3| Ok(Vec::new());

    let err =
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &DefinitionNoPlaneReplacementReachabilityCache::default(),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn probe_reachability_cycle_guard_tries_later_detour_after_uncertified_surface_query() {
    let start = p(0, 0, 0);
    let first_detour = p(1, 0, 0);
    let second_detour = p(2, 0, 0);
    let end = p(3, 0, 0);
    let mut surface_cache = Vec::new();

    assert!(
        probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &mut surface_cache,
            &mut |point| {
                if *point == first_detour {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(false)
                }
            },
            &mut |_from, _to, _start_definitions, _end_definitions| Ok(true),
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![
                        DetourTarget {
                            point: first_detour.clone(),
                            definitions: vec![axis_plane_definition(&first_detour)],
                            uncertified_definition_fallback: false,
                        },
                        DetourTarget {
                            point: second_detour.clone(),
                            definitions: vec![axis_plane_definition(&second_detour)],
                            uncertified_definition_fallback: false,
                        },
                    ])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_cycle_guard_allows_same_point_definition_transition_at_start() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = axis_plane_definition(&end);
    let lifted_start_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
    ];

    assert!(
        probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &mut |from, to, start_definitions, end_definitions| {
                Ok((*from == start
                    && *to == start
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&lifted_start_definition))
                    || (*from == start
                        && *to == end
                        && start_definitions == std::slice::from_ref(&lifted_start_definition)
                        && end_definitions == std::slice::from_ref(&end_definition)))
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: start.clone(),
                        definitions: vec![lifted_start_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_cycle_guard_allows_same_point_definition_transition_on_surface() {
    let start = p(0, 0, 0);
    let end = p(1, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = axis_plane_definition(&end);
    let lifted_start_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(0)),
    ];

    assert!(
        probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            &mut Vec::new(),
            &mut |point| Ok(*point == start),
            &mut |from, to, start_definitions, end_definitions| {
                Ok((*from == start
                    && *to == start
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&lifted_start_definition))
                    || (*from == start
                        && *to == end
                        && start_definitions == std::slice::from_ref(&lifted_start_definition)
                        && end_definitions == std::slice::from_ref(&end_definition)))
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: start.clone(),
                        definitions: vec![lifted_start_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_cycle_guard_allows_revisiting_point_with_new_definitions() {
    let start = p(0, 0, 0);
    let shared = p(1, 0, 0);
    let mid = p(2, 0, 0);
    let end = p(3, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let shared_definition = axis_plane_definition(&shared);
    let mid_definition = axis_plane_definition(&mid);
    let end_definition = axis_plane_definition(&end);
    let lifted_shared_definition = [
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(1, r(0)),
        Plane::new(Point3::new(r(1), r(1), r(1)), r(-1)),
    ];

    assert!(
        probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            std::slice::from_ref(&start_definition),
            std::slice::from_ref(&end_definition),
            &initial_visited_definition_points(
                &start,
                std::slice::from_ref(&start_definition),
                &end,
                std::slice::from_ref(&end_definition),
            ),
            &mut Vec::new(),
            &mut |_point| Ok(false),
            &mut |from, to, start_definitions, end_definitions| {
                Ok((*from == start
                    && *to == shared
                    && start_definitions == std::slice::from_ref(&start_definition)
                    && end_definitions == std::slice::from_ref(&shared_definition))
                    || (*from == shared
                        && *to == mid
                        && start_definitions == std::slice::from_ref(&shared_definition)
                        && end_definitions == std::slice::from_ref(&mid_definition))
                    || (*from == mid
                        && *to == shared
                        && start_definitions == std::slice::from_ref(&mid_definition)
                        && end_definitions == std::slice::from_ref(&lifted_shared_definition))
                    || (*from == shared
                        && *to == end
                        && start_definitions == std::slice::from_ref(&lifted_shared_definition)
                        && end_definitions == std::slice::from_ref(&end_definition)))
            },
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(vec![DetourTarget {
                        point: shared.clone(),
                        definitions: vec![shared_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else if *from == shared && *to == end {
                    Ok(vec![DetourTarget {
                        point: mid.clone(),
                        definitions: vec![mid_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else if *from == mid && *to == end {
                    Ok(vec![DetourTarget {
                        point: shared.clone(),
                        definitions: vec![lifted_shared_definition.clone()],
                        uncertified_definition_fallback: false,
                    }])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_reachability_cycle_guard_reuses_surface_queries_across_failed_branches() {
    let start = p(0, 0, 0);
    let shared = p(1, 0, 0);
    let outer_b = p(2, 0, 0);
    let outer_a = p(3, 0, 0);
    let end = p(4, 0, 0);
    let outer_targets = vec![
        DetourTarget {
            point: outer_a.clone(),
            definitions: vec![axis_plane_definition(&outer_a)],
            uncertified_definition_fallback: false,
        },
        DetourTarget {
            point: outer_b.clone(),
            definitions: vec![axis_plane_definition(&outer_b)],
            uncertified_definition_fallback: false,
        },
    ];
    let shared_target = DetourTarget {
        point: shared.clone(),
        definitions: vec![axis_plane_definition(&shared)],
        uncertified_definition_fallback: false,
    };
    let mut surface_cache = Vec::new();
    let mut query_calls = 0;

    assert!(
        !probe_reaches_adjacent_cell_with_cycle_guard_impl_with_surface_query(
            &start,
            &end,
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            &initial_visited_definition_points(
                &start,
                &[axis_plane_definition(&start)],
                &end,
                &[axis_plane_definition(&end)],
            ),
            &mut surface_cache,
            &mut |_point| {
                query_calls += 1;
                Ok(false)
            },
            &mut |_from, _to, _start_definitions, _end_definitions| Ok(false),
            &mut |from, to| {
                if *from == start && *to == end {
                    Ok(outer_targets.clone())
                } else if *from == start && (*to == outer_a || *to == outer_b) {
                    Ok(vec![shared_target.clone()])
                } else {
                    Ok(Vec::new())
                }
            },
        )
        .unwrap()
    );
    assert_eq!(query_calls, 3);
}

#[test]
fn probe_plane_replacement_step_detour_budget_uses_single_detour() {
    let start = p(0, 0, 0);
    let detour_point = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour_point.clone(),
        definitions: vec![axis_plane_definition(&detour_point)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours = |from: &Point3, to: &Point3| {
        Ok((*from == start && *to == detour_point) || (*from == detour_point && *to == end))
    };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        !probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            &start,
            &end,
            &[],
            0,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );

    assert!(
        probe_reaches_adjacent_cell_with_detours_without_plane_replacement_impl(
            &start,
            &end,
            &[],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn probe_winding_plane_replacement_step_detour_budget_uses_single_detour() {
    let start = p(0, 0, 0);
    let detour_point = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour_point.clone(),
        definitions: vec![axis_plane_definition(&detour_point)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours = |from: &Point3, to: &Point3, winding: &[i32]| {
        if (*from == start && *to == detour_point) || (*from == detour_point && *to == end) {
            Ok(Some(winding.to_vec()))
        } else {
            Ok(None)
        }
    };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert_eq!(
        trace_segment_with_detours_without_plane_replacement_impl(
            &start,
            &end,
            &[0],
            &[],
            0,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap(),
        None
    );

    assert_eq!(
        trace_segment_with_detours_without_plane_replacement_impl(
            &start,
            &end,
            &[0],
            &[],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap(),
        Some(vec![0])
    );
}

#[test]
fn no_detour_segment_search_backtracks_after_uncertified_direct_family() {
    let start = p(0, 0, 0);
    let detour_point = p(1, 0, 0);
    let end = p(2, 0, 0);
    let detour_target = DetourTarget {
        point: detour_point.clone(),
        definitions: vec![axis_plane_definition(&detour_point)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours = |from: &Point3, to: &Point3, winding: &[i32]| {
        if *from == start && *to == end {
            Err(HypermeshError::UnknownClassification)
        } else if (*from == start && *to == detour_point) || (*from == detour_point && *to == end) {
            Ok(Some(winding.to_vec()))
        } else {
            Ok(None)
        }
    };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert_eq!(
        trace_segment_with_detours_without_plane_replacement_impl(
            &start,
            &end,
            &[0],
            &[],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap(),
        Some(vec![0])
    );
}

#[test]
fn probe_plane_replacement_step_detours_preserve_intermediate_definitions() {
    let start = p(0, 0, 0);
    let detour_point = p(1, 0, 0);
    let end = p(2, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = [
        Plane::from_coefficients(r(1), r(1), r(1), r(-2)),
        Plane::axis_aligned(1, r(0)),
        Plane::axis_aligned(2, r(0)),
    ];
    let detour_definition = [
        Plane::from_coefficients(r(1), r(1), r(1), r(-1)),
        Plane::axis_aligned(1, r(0)),
        Plane::axis_aligned(2, r(0)),
    ];
    let detour_target = DetourTarget {
        point: detour_point.clone(),
        definitions: vec![detour_definition.clone()],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         start_definitions: &[[Plane; 3]],
         end_definitions: &[[Plane; 3]]| {
            Ok((*from == start
                && *to == detour_point
                && start_definitions == [start_definition.clone()]
                && end_definitions == [detour_definition.clone()])
                || (*from == detour_point
                    && *to == end
                    && start_definitions == [detour_definition.clone()]
                    && end_definitions == [end_definition.clone()]))
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![detour_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();

    assert!(
        plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &mut affine_cache,
            &mut step_cache,
            |from, to, start_definitions, end_definitions| {
                probe_reaches_adjacent_cell_with_definitions_budget_impl(
                    from,
                    to,
                    &[],
                    start_definitions,
                    end_definitions,
                    1,
                    &mut trace_without_detours,
                    &mut detours_for,
                )
            },
        )
        .unwrap()
    );
}

#[test]
fn probe_plane_replacement_reachability_surfaces_uncertified_intermediate_orderings() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = [
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(2, r(0)),
    ];
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();

    let err = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        &start_definition,
        &end_definition,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &mut affine_cache,
        &mut step_cache,
        |_from, _to, _start_definitions, _end_definitions| Ok(false),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn plane_replacement_reachability_step_reuses_equivalent_steps_across_orderings() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 0, 0));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut step_calls = 0;

    assert!(
        !plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &mut affine_cache,
            &mut step_cache,
            |_from, _to, _start_definitions, _end_definitions| {
                step_calls += 1;
                Ok(false)
            },
        )
        .unwrap()
    );

    assert_eq!(step_calls, 1);
}

#[test]
fn plane_replacement_reachability_tries_later_ordering_after_uncertified_step() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 1, 0));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();

    assert!(
        plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
            &start_definition,
            &end_definition,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &mut affine_cache,
            &mut step_cache,
            |from, to, _start_definitions, _end_definitions| {
                if *from == p(0, 0, 0) && *to == p(1, 0, 0) {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(true)
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn ordered_axis_orderings_by_no_step_precheck_prefers_more_direct_prefixes() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 1, 1));
    let mut affine_cache = PlaneReplacementAffineCache::default();

    let ordered = ordered_axis_orderings_by_no_step_precheck_with(
        &start_definition,
        &end_definition,
        &mut affine_cache,
        |_current, _next, current_planes, next_planes| {
            let changed_axis = (0..3)
                .find(|axis| current_planes[*axis] != next_planes[*axis])
                .unwrap();
            match changed_axis {
                2 => Ok(true),
                1 => Err(HypermeshError::UnknownClassification),
                0 => Ok(false),
                _ => unreachable!(),
            }
        },
    )
    .unwrap();

    assert_eq!(ordered[0], [2, 1, 0]);
    assert_eq!(ordered[1], [2, 0, 1]);
}

#[test]
fn plane_replacement_reachability_step_reuses_permuted_plane_sets() {
    let current_planes = axis_plane_definition(&p(0, 0, 0));
    let next_planes = axis_plane_definition(&p(1, 0, 0));
    let permuted_current = [
        current_planes[1].clone(),
        current_planes[2].clone(),
        current_planes[0].clone(),
    ];
    let permuted_next = [
        next_planes[1].clone(),
        next_planes[2].clone(),
        next_planes[0].clone(),
    ];
    let mut cache = PlaneReplacementReachabilityStepCache::default();
    let mut calls = 0;

    let first = cached_plane_replacement_reachability_step_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &current_planes,
        &next_planes,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_plane_replacement_reachability_step_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &permuted_current,
        &permuted_next,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert!(first);
    assert!(second);
}

#[test]
fn plane_replacement_reachability_step_cache_distinguishes_modes() {
    let current_planes = axis_plane_definition(&p(0, 0, 0));
    let next_planes = axis_plane_definition(&p(1, 0, 0));
    let mut cache = PlaneReplacementReachabilityStepCache::default();
    let mut calls = 0;

    let first = cached_plane_replacement_reachability_step_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &current_planes,
        &next_planes,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_plane_replacement_reachability_step_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &current_planes,
        &next_planes,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(!second);
    assert_eq!(calls, 2);
}

#[test]
fn plane_replacement_reachability_step_reuses_reversed_query() {
    let current_planes = axis_plane_definition(&p(0, 0, 0));
    let next_planes = axis_plane_definition(&p(1, 0, 0));
    let mut cache = PlaneReplacementReachabilityStepCache::default();
    let mut calls = 0;

    let first = cached_plane_replacement_reachability_step_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &current_planes,
        &next_planes,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_plane_replacement_reachability_step_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &p(1, 0, 0),
        &p(0, 0, 0),
        &next_planes,
        &current_planes,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(calls, 1);
}

#[test]
fn plane_replacement_reachability_step_reuses_in_progress_exact_state() {
    let current_planes = axis_plane_definition(&p(0, 0, 0));
    let next_planes = axis_plane_definition(&p(1, 0, 0));
    let mut cache = PlaneReplacementReachabilityStepCache {
        entries: vec![PlaneReplacementReachabilityStepCacheEntry {
            mode: PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            current_point: p(0, 0, 0),
            next_point: p(1, 0, 0),
            current_planes,
            next_planes,
            result: Err(HypermeshError::UnknownClassification),
        }],
        buckets: vec![PlaneReplacementReachabilityStepBucket {
            mode: PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            current_point: p(0, 0, 0),
            next_point: p(1, 0, 0),
            current_planes: axis_plane_definition(&p(0, 0, 0)),
            next_planes: axis_plane_definition(&p(1, 0, 0)),
            indices: vec![0],
        }],
    };

    let result = cached_plane_replacement_reachability_step_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &axis_plane_definition(&p(0, 0, 0)),
        &axis_plane_definition(&p(1, 0, 0)),
        || Ok(true),
    );

    assert_eq!(result, Err(HypermeshError::UnknownClassification));
    assert_eq!(cache.len(), 1);
    assert_eq!(
        cache.entries[0].result,
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn plane_replacement_reachability_path_reuses_permuted_plane_sets() {
    let start_planes = axis_plane_definition(&p(0, 0, 0));
    let end_planes = axis_plane_definition(&p(1, 0, 0));
    let permuted_start = [
        start_planes[1].clone(),
        start_planes[2].clone(),
        start_planes[0].clone(),
    ];
    let permuted_end = [
        end_planes[1].clone(),
        end_planes[2].clone(),
        end_planes[0].clone(),
    ];
    let mut cache = PlaneReplacementReachabilityPathCache::default();
    let mut calls = 0;

    let first = cached_plane_replacement_reachability_path_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        &start_planes,
        &end_planes,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_plane_replacement_reachability_path_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        &permuted_start,
        &permuted_end,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert!(first);
    assert!(second);
}

#[test]
fn plane_replacement_reachability_path_cache_distinguishes_modes() {
    let start_planes = axis_plane_definition(&p(0, 0, 0));
    let end_planes = axis_plane_definition(&p(1, 0, 0));
    let mut cache = PlaneReplacementReachabilityPathCache::default();
    let mut calls = 0;

    let first = cached_plane_replacement_reachability_path_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        &start_planes,
        &end_planes,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_plane_replacement_reachability_path_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &start_planes,
        &end_planes,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(!second);
    assert_eq!(calls, 2);
}

#[test]
fn plane_replacement_reachability_path_reuses_reversed_query() {
    let start_planes = axis_plane_definition(&p(0, 0, 0));
    let end_planes = axis_plane_definition(&p(1, 0, 0));
    let mut cache = PlaneReplacementReachabilityPathCache::default();
    let mut calls = 0;

    let first = cached_plane_replacement_reachability_path_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &start_planes,
        &end_planes,
        || {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_plane_replacement_reachability_path_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &end_planes,
        &start_planes,
        || {
            calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(calls, 1);
}

#[test]
fn plane_replacement_reachability_path_reuses_in_progress_exact_state() {
    let start_planes = axis_plane_definition(&p(0, 0, 0));
    let end_planes = axis_plane_definition(&p(1, 0, 0));
    let mut cache = PlaneReplacementReachabilityPathCache {
        entries: vec![PlaneReplacementReachabilityPathCacheEntry {
            mode: PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            start_planes,
            end_planes,
            result: Err(HypermeshError::UnknownClassification),
        }],
        buckets: vec![PlaneReplacementReachabilityPathBucket {
            mode: PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            start_planes: axis_plane_definition(&p(0, 0, 0)),
            end_planes: axis_plane_definition(&p(1, 0, 0)),
            indices: vec![0],
        }],
    };

    let result = cached_plane_replacement_reachability_path_with(
        &mut cache,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &axis_plane_definition(&p(0, 0, 0)),
        &axis_plane_definition(&p(1, 0, 0)),
        || Ok(true),
    );

    assert_eq!(result, Err(HypermeshError::UnknownClassification));
    assert_eq!(cache.len(), 1);
    assert_eq!(
        cache.entries[0].result,
        Err(HypermeshError::UnknownClassification)
    );
}

#[test]
fn plane_replacement_no_nested_ordering_warmup_reuses_cached_local_warm_state() {
    let start_planes = axis_plane_definition(&p(0, 0, 0));
    let end_planes = axis_plane_definition(&p(1, 0, 0));
    let affine_entry = PlaneReplacementAffineCacheEntry {
        planes: start_planes.clone(),
        point: Ok(p(0, 0, 0)),
    };
    let path_entry = PlaneReplacementReachabilityPathCacheEntry {
        mode: PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        start_planes: start_planes.clone(),
        end_planes: end_planes.clone(),
        result: Ok(true),
    };
    let step_entry = PlaneReplacementReachabilityStepCacheEntry {
        mode: PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        current_point: p(0, 0, 0),
        next_point: p(1, 0, 0),
        current_planes: start_planes.clone(),
        next_planes: end_planes.clone(),
        result: Ok(true),
    };
    let mut cache = PlaneReplacementNoNestedOrderingWarmupCache::default();
    let mut first_affine = PlaneReplacementAffineCache::default();
    let mut first_path = PlaneReplacementReachabilityPathCache::default();
    let mut first_step = PlaneReplacementReachabilityStepCache::default();
    let mut calls = 0;

    let first = cached_plane_replacement_no_nested_ordering_warmup_with(
        &mut cache,
        &start_planes,
        &end_planes,
        &mut first_affine,
        &mut first_path,
        &mut first_step,
        |affine, path, step| {
            calls += 1;
            affine.entries.push(affine_entry.clone());
            path.entries.push(path_entry.clone());
            push_plane_replacement_reachability_path_bucket_entry(
                &mut path.buckets,
                path_entry.mode,
                &path_entry.start_planes,
                &path_entry.end_planes,
                path.entries.len() - 1,
            );
            step.entries.push(step_entry.clone());
            push_plane_replacement_reachability_step_bucket_entry(
                &mut step.buckets,
                step_entry.mode,
                &step_entry.current_point,
                &step_entry.next_point,
                &step_entry.current_planes,
                &step_entry.next_planes,
                step.entries.len() - 1,
            );
            Ok(vec![[0, 1, 2]])
        },
    )
    .unwrap();

    let mut second_affine = PlaneReplacementAffineCache::default();
    let mut second_path = PlaneReplacementReachabilityPathCache::default();
    let mut second_step = PlaneReplacementReachabilityStepCache::default();
    let second = cached_plane_replacement_no_nested_ordering_warmup_with(
        &mut cache,
        &start_planes,
        &end_planes,
        &mut second_affine,
        &mut second_path,
        &mut second_step,
        |_, _, _| {
            calls += 1;
            Ok(vec![[2, 1, 0]])
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
    assert_eq!(first_affine, second_affine);
    assert_eq!(first_path, second_path);
    assert_eq!(first_step, second_step);
}

#[test]
fn probe_hot_leaf_probe_family_breakdown() {
    use crate::mesh::prepare_input;
    use crate::polygon::ConvexPolygon;

    fn tetra_from_face_and_apex(a: Point3, b: Point3, c: Point3, apex: Point3) -> crate::InputMesh {
        crate::InputMesh::new(
            vec![a, b, c, apex],
            vec![
                crate::Triangle::new(0, 2, 1),
                crate::Triangle::new(0, 1, 3),
                crate::Triangle::new(0, 3, 2),
                crate::Triangle::new(1, 2, 3),
            ],
        )
    }

    fn face_at(
        polygons: &[ConvexPolygon],
        mesh_index: isize,
        polygon_index: isize,
    ) -> ConvexPolygon {
        polygons
            .iter()
            .find(|polygon| {
                polygon.mesh_index == mesh_index && polygon.polygon_index == polygon_index
            })
            .unwrap()
            .clone()
    }

    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let polygons = vec![
        face_at(&soup.polygons, 1, 4),
        face_at(&soup.polygons, 1, 5),
        face_at(&soup.polygons, 1, 7),
        face_at(&soup.polygons, 2, 8),
        face_at(&soup.polygons, 2, 11),
    ];
    let bounds = Aabb::new(p(1, 1, 1), p(9, 9, 9));
    let ref_point = p(0, 5, 5);
    let ref_definitions = vec![axis_plane_definition(&ref_point)];
    let ref_wnv = vec![0; soup.num_meshes];

    let host = &polygons[0];
    let intersections = polygons
        .iter()
        .enumerate()
        .filter_map(|(index, polygon)| {
            if index == 0 {
                return None;
            }
            let intersection =
                crate::intersection::intersect_polygons(host, polygon, index).ok()?;
            Some(intersection)
        })
        .collect::<Vec<_>>();
    let bsp_leaves =
        crate::subdivision::build_host_bsp_leaves(host, &polygons, &intersections).unwrap();
    let (leaf, interior_points, effective_delta_w) = bsp_leaves
        .iter()
        .filter_map(|leaf| {
            crate::subdivision::certify_bsp_leaf_and_delta_w(host, &leaf.edges, &polygons)
                .ok()
                .map(|(interior_points, effective_delta_w)| {
                    (leaf, interior_points, effective_delta_w)
                })
        })
        .max_by_key(|(leaf, _, _)| leaf.edges.len())
        .unwrap();
    let interior = interior_points[0].clone();

    let normal_probes =
        adjacent_normal_probes(&interior, &host.support, &bounds, &polygons, true).unwrap();

    let mut axis_probe_counts = Vec::new();
    for axis in probe_axes(&host.support).unwrap() {
        let normal_sign =
            crate::geometry::classify_real(axis_ref(&host.support.normal, axis)).unwrap();
        if normal_sign == Classification::On {
            continue;
        }
        let direction_positive = normal_sign == Classification::Positive;
        let probes = adjacent_axis_probes(
            &interior,
            &host.support,
            &bounds,
            &polygons,
            axis,
            direction_positive,
        )
        .unwrap();
        axis_probe_counts.push((axis, probes.len()));
    }

    let winding = classify_leaf_polygon_from_interior_points(
        &interior_points,
        &host.support,
        &ref_point,
        &ref_definitions,
        &ref_wnv,
        &polygons,
        &bounds,
        &effective_delta_w,
    )
    .unwrap();
    assert_eq!(leaf.edges.len(), 4);
    assert!(!normal_probes.is_empty());
    assert!(!axis_probe_counts.is_empty());
    assert!(axis_probe_counts.iter().all(|(_, count)| *count > 0));
    assert_eq!(winding, vec![0, 0, 0]);
}

#[test]
fn plane_replacement_reachability_shared_caches_reuse_equivalent_path_across_calls() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 0, 0));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut step_calls = 0;

    let first = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        &start_definition,
        &end_definition,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &mut affine_cache,
        &mut step_cache,
        |_from, _to, _start_definitions, _end_definitions| {
            step_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        &start_definition,
        &end_definition,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &mut affine_cache,
        &mut step_cache,
        |_from, _to, _start_definitions, _end_definitions| {
            step_calls += 1;
            Ok(true)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(step_calls, 1);
}

#[test]
fn bounded_plane_replacement_reachability_adapts_when_every_raw_order_leaves_bounds() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 1);
    let start_definition = axis_plane_definition(&start);
    let end_definition = [
        Plane::from_coefficients(r(1), r(2), r(0), r(-3)),
        Plane::from_coefficients(r(0), r(1), r(2), r(-3)),
        Plane::from_coefficients(r(2), r(0), r(1), r(-3)),
    ];
    assert_eq!(affine_from_planes(&end_definition).unwrap(), end);
    let bounds = Aabb::new(start.clone(), end.clone());
    for plane_index in 0..3 {
        let mut first_step = start_definition.clone();
        first_step[plane_index] = end_definition[plane_index].clone();
        assert!(
            !bounds
                .contains_point(&affine_from_planes(&first_step).unwrap())
                .unwrap()
        );
    }
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut traced_steps = Vec::new();

    let reaches =
        plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
            &AXIS_ORDERINGS,
            &start_definition,
            &end_definition,
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
            &mut affine_cache,
            &mut step_cache,
            Some(&bounds),
            |current, next, current_definitions, next_definitions| {
                assert!(bounds.contains_point(current).unwrap());
                assert!(bounds.contains_point(next).unwrap());
                assert_eq!(current_definitions.len(), 1);
                assert_eq!(next_definitions.len(), 1);
                assert_eq!(
                    affine_from_planes(&current_definitions[0]).unwrap(),
                    *current
                );
                assert_eq!(affine_from_planes(&next_definitions[0]).unwrap(), *next);
                traced_steps.push((current.clone(), next.clone()));
                Ok(true)
            },
        )
        .unwrap();

    assert!(reaches);
    assert_eq!(traced_steps.first().unwrap().0, start);
    assert_eq!(traced_steps.last().unwrap().1, end);
    assert!(traced_steps.len() >= 2);
}

#[test]
fn bounded_plane_replacement_reachability_does_not_adapt_outside_endpoint() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(2, 0, 0));
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut traced_steps = 0;

    let err = plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
        &AXIS_ORDERINGS,
        &start_definition,
        &end_definition,
        PlaneReplacementReachabilityStepMode::WithoutStepDetours,
        &mut affine_cache,
        &mut step_cache,
        Some(&bounds),
        |_current, _next, _current_definitions, _next_definitions| {
            traced_steps += 1;
            Ok(true)
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert_eq!(traced_steps, 0);
}

#[test]
fn no_step_ordering_precheck_warms_shared_affine_cache_for_step_trace() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 2, 3));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let ordered = ordered_axis_orderings_by_no_step_precheck_with(
        &start_definition,
        &end_definition,
        &mut affine_cache,
        |_from, _to, _start_definitions, _end_definitions| Ok(true),
    )
    .unwrap();
    let affine_len_after_precheck = affine_cache.len();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();

    let reaches =
        plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
            &ordered,
            &start_definition,
            &end_definition,
            PlaneReplacementReachabilityStepMode::WithoutStepDetours,
            &mut affine_cache,
            &mut step_cache,
            None,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
        )
        .unwrap();

    assert!(reaches);
    assert_eq!(affine_cache.len(), affine_len_after_precheck);
}

#[test]
fn no_nested_ordering_precheck_warms_shared_affine_cache_for_step_trace() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 2, 3));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut no_step_affine_cache = PlaneReplacementAffineCache::default();
    let mut no_step_path_cache = PlaneReplacementReachabilityPathCache::default();
    let mut no_step_step_cache = PlaneReplacementReachabilityStepCache::default();
    let mut warmup_cache = PlaneReplacementNoNestedOrderingWarmupCache::default();
    let ordered = cached_plane_replacement_no_nested_ordering_warmup_with(
        &mut warmup_cache,
        &start_definition,
        &end_definition,
        &mut no_step_affine_cache,
        &mut no_step_path_cache,
        &mut no_step_step_cache,
        |_, _, _| {
            ordered_axis_orderings_by_no_step_precheck_with(
                &start_definition,
                &end_definition,
                &mut affine_cache,
                |_from, _to, _start_definitions, _end_definitions| Ok(true),
            )
        },
    )
    .unwrap();
    let affine_len_after_precheck = affine_cache.len();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();

    let reaches =
        plane_replacement_path_reaches_adjacent_cell_with_step_detours_for_orderings_impl(
            &ordered,
            &start_definition,
            &end_definition,
            PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
            &mut affine_cache,
            &mut step_cache,
            None,
            |_from, _to, _start_definitions, _end_definitions| Ok(true),
        )
        .unwrap();

    assert!(reaches);
    assert_eq!(affine_cache.len(), affine_len_after_precheck);
}

#[test]
fn plane_replacement_reachability_reports_unknown_for_same_point_uncertified_step() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(2, r(0)),
        Plane::from_coefficients(r(1), r(1), r(0), r(0)),
    ];
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = PlaneReplacementReachabilityStepCache::default();

    let err = plane_replacement_path_reaches_adjacent_cell_with_step_detours_impl(
        &start_definition,
        &end_definition,
        PlaneReplacementReachabilityStepMode::WithoutNestedPlaneReplacement,
        &mut affine_cache,
        &mut step_cache,
        |from, to, start_definitions, end_definitions| {
            if *from == p(0, 0, 0)
                && *to == p(0, 0, 0)
                && start_definitions == std::slice::from_ref(&start_definition)
                && end_definitions == std::slice::from_ref(&end_definition)
            {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(true)
            }
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn definition_pair_reachability_backtracks_after_uncertified_pair() {
    let start_unknown = axis_plane_definition(&p(0, 0, 0));
    let start_ok = axis_plane_definition(&p(1, 0, 0));
    let end = axis_plane_definition(&p(2, 0, 0));

    assert!(
        definition_pair_reachability_backtracking_unknown(
            &[start_unknown.clone(), start_ok.clone()],
            std::slice::from_ref(&end),
            |start_definition, end_definition| {
                if start_definition == &start_unknown && end_definition == &end {
                    Err(HypermeshError::UnknownClassification)
                } else {
                    Ok(start_definition == &start_ok && end_definition == &end)
                }
            },
        )
        .unwrap()
    );
}

#[test]
fn definition_pair_reachability_reports_unknown_if_all_pairs_are_uncertified() {
    let start = axis_plane_definition(&p(0, 0, 0));
    let end = axis_plane_definition(&p(1, 0, 0));

    let err = definition_pair_reachability_backtracking_unknown(
        std::slice::from_ref(&start),
        std::slice::from_ref(&end),
        |_start_definition, _end_definition| Err(HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn definition_pair_reachability_search_skips_duplicate_definition_pairs() {
    let start_a = axis_plane_definition(&p(0, 0, 0));
    let start_b = axis_plane_definition(&p(1, 0, 0));
    let end = axis_plane_definition(&p(2, 0, 0));
    let mut reachability_calls = 0;

    let reaches = definition_pair_reachability_backtracking_unknown(
        &[start_a.clone(), start_a.clone(), start_b.clone()],
        &[end.clone(), end.clone()],
        |start_definition, end_definition| {
            reachability_calls += 1;
            if start_definition == &start_a && end_definition == &end {
                Ok(false)
            } else {
                Ok(start_definition == &start_b && end_definition == &end)
            }
        },
    )
    .unwrap();

    assert!(reaches);
    assert_eq!(reachability_calls, 2);
}

#[test]
fn definition_pair_reachability_search_skips_permuted_definition_pairs() {
    let start_a = axis_plane_definition(&p(0, 0, 0));
    let start_a_permuted = [start_a[2].clone(), start_a[0].clone(), start_a[1].clone()];
    let start_b = axis_plane_definition(&p(1, 0, 0));
    let end = axis_plane_definition(&p(2, 0, 0));
    let end_permuted = [end[1].clone(), end[2].clone(), end[0].clone()];
    let mut reachability_calls = 0;

    let reaches = definition_pair_reachability_backtracking_unknown(
        &[start_a.clone(), start_a_permuted, start_b.clone()],
        &[end.clone(), end_permuted],
        |start_definition, end_definition| {
            reachability_calls += 1;
            if definition_planes_match_as_sets(start_definition, &start_a)
                && definition_planes_match_as_sets(end_definition, &end)
            {
                Ok(false)
            } else {
                Ok(definition_planes_match_as_sets(start_definition, &start_b)
                    && definition_planes_match_as_sets(end_definition, &end))
            }
        },
    )
    .unwrap();

    assert!(reaches);
    assert_eq!(reachability_calls, 2);
}

#[test]
fn plane_replacement_step_detours_preserve_intermediate_definitions() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = [
        Plane::from_coefficients(r(1), r(1), r(1), r(-1)),
        Plane::axis_aligned(1, r(0)),
        Plane::axis_aligned(2, r(0)),
    ];
    let expected_start_definitions = vec![start_definition.clone()];
    let expected_end_definitions = vec![end_definition.clone()];
    let expected_start = p(0, 0, 0);
    let expected_end = p(1, 0, 0);
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();

    let winding = trace_plane_replacement_path_with_step_detours_impl(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
        None,
        |current, next, attempt, _polygons, current_definitions, next_definitions| {
            if *current == expected_start
                && *next == expected_end
                && current_definitions == expected_start_definitions.as_slice()
                && next_definitions == expected_end_definitions.as_slice()
            {
                Ok(Some(attempt.to_vec()))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn bounded_plane_replacement_adapts_outside_intermediate_orderings() {
    let start = p(0, 0, 0);
    let start_definition = axis_plane_definition(&start);
    let end_definition = [
        Plane::from_coefficients(r(1), r(1), r(0), r(-2)),
        Plane::axis_aligned(1, r(1)),
        Plane::axis_aligned(2, r(1)),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    let mut traced_steps = Vec::new();

    let winding = trace_plane_replacement_path_with_tracer_and_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
        Some(&bounds),
        |current, next, _current_planes, _next_planes, attempt, _polygons| {
            assert!(bounds.contains_point(current).unwrap());
            assert!(bounds.contains_point(next).unwrap());
            traced_steps.push((current.clone(), next.clone()));
            Ok(Some(attempt.to_vec()))
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
    assert_eq!(traced_steps[0], (start, p(1, 0, 0)));
    assert!(!traced_steps.iter().any(|(_, next)| *next == p(2, 0, 0)));
}

#[test]
fn bounded_plane_replacement_adapts_when_every_raw_order_leaves_bounds() {
    let start = p(0, 0, 0);
    let end = p(1, 1, 1);
    let start_definition = axis_plane_definition(&start);
    let end_definition = [
        Plane::from_coefficients(r(1), r(2), r(0), r(-3)),
        Plane::from_coefficients(r(0), r(1), r(2), r(-3)),
        Plane::from_coefficients(r(2), r(0), r(1), r(-3)),
    ];
    assert_eq!(affine_from_planes(&end_definition).unwrap(), end);
    let bounds = Aabb::new(start.clone(), end.clone());
    for plane_index in 0..3 {
        let mut first_step = start_definition.clone();
        first_step[plane_index] = end_definition[plane_index].clone();
        assert!(
            !bounds
                .contains_point(&affine_from_planes(&first_step).unwrap())
                .unwrap()
        );
    }
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    let mut traced_steps = Vec::new();

    let winding = trace_plane_replacement_path_with_tracer_and_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
        Some(&bounds),
        |current, next, current_planes, next_planes, attempt, _polygons| {
            assert!(bounds.contains_point(current).unwrap());
            assert!(bounds.contains_point(next).unwrap());
            assert_eq!(affine_from_planes(current_planes).unwrap(), *current);
            assert_eq!(affine_from_planes(next_planes).unwrap(), *next);
            traced_steps.push((current.clone(), next.clone()));
            Ok(Some(attempt.to_vec()))
        },
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
    assert_eq!(traced_steps.first().unwrap().0, start);
    assert_eq!(traced_steps.last().unwrap().1, end);
    assert!(traced_steps.len() >= 2);
}

#[test]
fn bounded_plane_replacement_does_not_adapt_outside_endpoint() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(2, 0, 0));
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    let mut traced_steps = 0;

    let err = trace_plane_replacement_path_with_tracer_and_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
        Some(&bounds),
        |_current, _next, _current_planes, _next_planes, attempt, _polygons| {
            traced_steps += 1;
            Ok(Some(attempt.to_vec()))
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert_eq!(traced_steps, 0);
}

#[test]
fn plane_replacement_tracer_shared_caches_reuse_equivalent_path_across_calls() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 0, 0));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    let mut step_calls = 0;

    let first = trace_plane_replacement_path_with_tracer_and_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
        None,
        |_current, _next, _current_planes, _next_planes, attempt, _polygons| {
            step_calls += 1;
            Ok(Some(attempt.to_vec()))
        },
    )
    .unwrap();
    let second = trace_plane_replacement_path_with_tracer_and_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
        None,
        |_current, _next, _current_planes, _next_planes, attempt, _polygons| {
            step_calls += 1;
            Ok(Some(attempt.to_vec()))
        },
    )
    .unwrap();

    assert_eq!(first, vec![7]);
    assert_eq!(second, vec![7]);
    assert_eq!(step_calls, 1);
}

#[test]
fn plane_replacement_tracer_reports_unknown_for_same_point_uncertified_step() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(2, r(0)),
        Plane::from_coefficients(r(1), r(1), r(0), r(0)),
    ];
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();

    let err = trace_plane_replacement_path_with_tracer_and_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
        None,
        |current, next, current_planes, next_planes, _attempt, _polygons| {
            if *current == p(0, 0, 0)
                && *next == p(0, 0, 0)
                && current_planes == &start_definition
                && next_planes == &end_definition
            {
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(vec![7]))
            }
        },
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn plane_replacement_no_detour_shared_caches_reuse_equivalent_path_across_calls() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 0, 0));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();

    let first = trace_plane_replacement_path_without_detours_with_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
    )
    .unwrap();
    let affine_len = affine_cache.len();
    let step_len = step_cache.len();
    let second = trace_plane_replacement_path_without_detours_with_caches(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        &mut affine_cache,
        &mut step_cache,
    )
    .unwrap();

    assert_eq!(first, vec![7]);
    assert_eq!(second, vec![7]);
    assert_eq!(affine_cache.len(), affine_len);
    assert_eq!(step_cache.len(), step_len);
}

#[test]
fn plane_replacement_step_tracer_backtracks_after_uncertified_step() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 1, 0));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    let mut first_call = true;

    let winding = trace_plane_replacement_path_with_tracer(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        |_current, _next, _current_planes, _next_planes, attempt, _polygons| {
            if first_call {
                first_call = false;
                Err(HypermeshError::UnknownClassification)
            } else {
                Ok(Some(attempt.to_vec()))
            }
        },
        &mut affine_cache,
        &mut step_cache,
    )
    .unwrap();

    assert_eq!(winding, vec![7]);
}

#[test]
fn plane_replacement_step_tracer_reuses_equivalent_steps_across_orderings() {
    let start_definition = axis_plane_definition(&p(0, 0, 0));
    let end_definition = axis_plane_definition(&p(1, 0, 0));
    let mut affine_cache = PlaneReplacementAffineCache::default();
    let mut step_cache = Vec::new();
    let mut step_calls = 0;

    let err = trace_plane_replacement_path_with_tracer(
        &start_definition,
        &end_definition,
        &[7],
        &[],
        |_current, _next, _current_planes, _next_planes, _attempt, _polygons| {
            step_calls += 1;
            Ok(None)
        },
        &mut affine_cache,
        &mut step_cache,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
    assert_eq!(step_calls, 1);
}

#[test]
fn plane_replacement_step_tracer_reuses_permuted_plane_sets() {
    let current_planes = axis_plane_definition(&p(0, 0, 0));
    let next_planes = axis_plane_definition(&p(1, 0, 0));
    let permuted_current = [
        current_planes[1].clone(),
        current_planes[2].clone(),
        current_planes[0].clone(),
    ];
    let permuted_next = [
        next_planes[1].clone(),
        next_planes[2].clone(),
        next_planes[0].clone(),
    ];
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_plane_replacement_step_with(
        &mut cache,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &current_planes,
        &next_planes,
        &[7],
        || {
            calls += 1;
            Ok(Some(vec![7]))
        },
    )
    .unwrap();
    let second = cached_plane_replacement_step_with(
        &mut cache,
        &p(0, 0, 0),
        &p(1, 0, 0),
        &permuted_current,
        &permuted_next,
        &[7],
        || {
            calls += 1;
            Ok(Some(vec![9]))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, Some(vec![7]));
    assert_eq!(second, Some(vec![7]));
}

#[test]
fn cached_affine_from_planes_reuses_identical_plane_set() {
    let planes = axis_plane_definition(&p(1, 2, 3));
    let mut cache = PlaneReplacementAffineCache::default();
    let mut calls = 0;

    let first = cached_affine_from_planes_with(&mut cache, &planes, || {
        calls += 1;
        affine_from_planes(&planes)
    })
    .unwrap();
    let second = cached_affine_from_planes_with(&mut cache, &planes, || {
        calls += 1;
        affine_from_planes(&planes)
    })
    .unwrap();

    assert_eq!(first, p(1, 2, 3));
    assert_eq!(second, first);
    assert_eq!(calls, 1);
}

#[test]
fn cached_affine_from_planes_reuses_permuted_plane_set() {
    let planes = axis_plane_definition(&p(1, 2, 3));
    let permuted = [planes[1].clone(), planes[2].clone(), planes[0].clone()];
    let mut cache = PlaneReplacementAffineCache::default();
    let mut calls = 0;

    let first = cached_affine_from_planes_with(&mut cache, &planes, || {
        calls += 1;
        affine_from_planes(&planes)
    })
    .unwrap();
    let second = cached_affine_from_planes_with(&mut cache, &permuted, || {
        calls += 1;
        affine_from_planes(&permuted)
    })
    .unwrap();

    assert_eq!(first, p(1, 2, 3));
    assert_eq!(second, first);
    assert_eq!(calls, 1);
}

#[test]
fn endpoint_definition_family_drops_only_known_mismatches() {
    let point = p(1, 2, 3);
    let matching = axis_plane_definition(&point);
    let mismatched = axis_plane_definition(&p(4, 5, 6));
    let singular = [
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(2, r(3)),
    ];

    let family = endpoint_definition_family(
        &point,
        &[mismatched.clone(), singular.clone(), matching.clone()],
    )
    .unwrap();

    assert_eq!(family.definitions, vec![matching]);
    assert!(family.saw_unknown);
    assert!(!family.definitions.contains(&mismatched));
    assert!(!family.definitions.contains(&singular));
}

#[test]
fn recursive_detour_budget_retries_detour_legs() {
    let start = p(0, 0, 0);
    let inner = p(1, 0, 0);
    let outer = p(2, 0, 0);
    let end = p(3, 0, 0);
    let outer_target = DetourTarget {
        point: outer.clone(),
        definitions: vec![axis_plane_definition(&outer)],
        uncertified_definition_fallback: false,
    };
    let inner_target = DetourTarget {
        point: inner.clone(),
        definitions: vec![axis_plane_definition(&inner)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         winding: &[i32],
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            if (*from == start && *to == inner)
                || (*from == inner && *to == outer)
                || (*from == outer && *to == end)
            {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![outer_target.clone()])
        } else if *from == start && *to == outer {
            Ok(vec![inner_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert_eq!(
        trace_segment_from_definitions_with_budget_impl(
            &start,
            &end,
            &[0],
            &[],
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            1,
            &mut trace_without_detours,
            &mut detours_for,
        ),
        Err(HypermeshError::UnknownClassification)
    );

    let with_nested = trace_segment_from_definitions_with_budget_impl(
        &start,
        &end,
        &[0],
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        2,
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap();
    assert_eq!(with_nested, vec![0]);
}

#[test]
fn trace_segment_from_definitions_runtime_allows_three_nested_detours() {
    let start = p(0, 0, 0);
    let detour_a = p(1, 0, 0);
    let detour_b = p(2, 0, 0);
    let detour_c = p(3, 0, 0);
    let end = p(4, 0, 0);
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         winding: &[i32],
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            if (*from == start && *to == detour_a)
                || (*from == detour_a && *to == detour_b)
                || (*from == detour_b && *to == detour_c)
                || (*from == detour_c && *to == end)
            {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![DetourTarget {
                point: detour_c.clone(),
                definitions: vec![axis_plane_definition(&detour_c)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == start && *to == detour_c {
            Ok(vec![DetourTarget {
                point: detour_b.clone(),
                definitions: vec![axis_plane_definition(&detour_b)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == start && *to == detour_b {
            Ok(vec![DetourTarget {
                point: detour_a.clone(),
                definitions: vec![axis_plane_definition(&detour_a)],
                uncertified_definition_fallback: false,
            }])
        } else {
            Ok(Vec::new())
        }
    };

    let traced = trace_segment_from_definitions_with_cycle_guard_impl(
        &start,
        &end,
        &[0],
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap();

    assert_eq!(traced, vec![0]);
}

#[test]
fn trace_segment_from_definitions_cycle_guard_skips_revisited_path_points() {
    let start = p(0, 0, 0);
    let detour = p(1, 0, 0);
    let end = p(2, 0, 0);
    let mut trace_without_detours =
        |_from: &Point3,
         _to: &Point3,
         _winding: &[i32],
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| { Ok(None) };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![DetourTarget {
                point: detour.clone(),
                definitions: vec![axis_plane_definition(&detour)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == detour && *to == end {
            Ok(vec![DetourTarget {
                point: start.clone(),
                definitions: vec![axis_plane_definition(&start)],
                uncertified_definition_fallback: false,
            }])
        } else if *from == detour && *to == start {
            Ok(vec![DetourTarget {
                point: end.clone(),
                definitions: vec![axis_plane_definition(&end)],
                uncertified_definition_fallback: false,
            }])
        } else {
            Ok(Vec::new())
        }
    };

    let err = trace_segment_from_definitions_with_cycle_guard_impl(
        &start,
        &end,
        &[0],
        &[],
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        &initial_visited_definition_points(
            &start,
            &[axis_plane_definition(&start)],
            &end,
            &[axis_plane_definition(&end)],
        ),
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn detour_recursion_limit_scales_with_local_polygon_count() {
    let polygons = vec![
        make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
        make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 0, 1),
        make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 0, 2),
    ];

    assert_eq!(detour_recursion_limit(&[]), 2);
    assert_eq!(detour_recursion_limit(&polygons), 3);
}

#[test]
fn plane_replacement_step_detour_limit_scales_with_local_polygon_count() {
    let polygons = vec![
        make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
        make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 0, 1),
    ];

    assert_eq!(plane_replacement_step_detour_limit(&[]), 1);
    assert_eq!(plane_replacement_step_detour_limit(&polygons), 2);
}

#[test]
fn polygon_scaled_detour_budget_allows_two_nested_detours() {
    let start = p(0, 0, 0);
    let inner = p(1, 0, 0);
    let outer = p(2, 0, 0);
    let end = p(3, 0, 0);
    let polygons = vec![
        make_triangle(&p(0, 10, 0), &p(1, 10, 0), &p(0, 11, 0), 0, 0),
        make_triangle(&p(0, 10, 1), &p(1, 10, 1), &p(0, 11, 1), 0, 1),
        make_triangle(&p(0, 10, 2), &p(1, 10, 2), &p(0, 11, 2), 0, 2),
    ];
    let outer_target = DetourTarget {
        point: outer.clone(),
        definitions: vec![axis_plane_definition(&outer)],
        uncertified_definition_fallback: false,
    };
    let inner_target = DetourTarget {
        point: inner.clone(),
        definitions: vec![axis_plane_definition(&inner)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         winding: &[i32],
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            if (*from == start && *to == inner)
                || (*from == inner && *to == outer)
                || (*from == outer && *to == end)
            {
                Ok(Some(winding.to_vec()))
            } else {
                Ok(None)
            }
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![outer_target.clone()])
        } else if *from == start && *to == outer {
            Ok(vec![inner_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    let traced = trace_segment_from_definitions_with_budget_impl(
        &start,
        &end,
        &[0],
        &polygons,
        &[axis_plane_definition(&start)],
        &[axis_plane_definition(&end)],
        detour_recursion_limit(&polygons),
        &mut trace_without_detours,
        &mut detours_for,
    )
    .unwrap();

    assert_eq!(traced, vec![0]);
}

#[test]
fn polygon_scaled_probe_step_detour_budget_allows_two_nested_detours() {
    let start = p(0, 0, 0);
    let inner = p(1, 0, 0);
    let outer = p(2, 0, 0);
    let end = p(3, 0, 0);
    let polygons = vec![
        make_triangle(&p(0, 10, 0), &p(1, 10, 0), &p(0, 11, 0), 0, 0),
        make_triangle(&p(0, 10, 1), &p(1, 10, 1), &p(0, 11, 1), 0, 1),
    ];
    let outer_target = DetourTarget {
        point: outer.clone(),
        definitions: vec![axis_plane_definition(&outer)],
        uncertified_definition_fallback: false,
    };
    let inner_target = DetourTarget {
        point: inner.clone(),
        definitions: vec![axis_plane_definition(&inner)],
        uncertified_definition_fallback: false,
    };
    let mut trace_without_detours =
        |from: &Point3,
         to: &Point3,
         _start_definitions: &[[Plane; 3]],
         _end_definitions: &[[Plane; 3]]| {
            Ok((*from == start && *to == inner)
                || (*from == inner && *to == outer)
                || (*from == outer && *to == end))
        };
    let mut detours_for = |from: &Point3, to: &Point3| {
        if *from == start && *to == end {
            Ok(vec![outer_target.clone()])
        } else if *from == start && *to == outer {
            Ok(vec![inner_target.clone()])
        } else {
            Ok(Vec::new())
        }
    };

    assert!(
        probe_reaches_adjacent_cell_with_definitions_budget_impl(
            &start,
            &end,
            &polygons,
            &[axis_plane_definition(&start)],
            &[axis_plane_definition(&end)],
            plane_replacement_step_detour_limit(&polygons),
            &mut trace_without_detours,
            &mut detours_for,
        )
        .unwrap()
    );
}

#[test]
fn probe_fallback_retries_axis_start_after_retained_definitions_fail() {
    let ref_point = p(0, 0, 0);
    let invalid_ref_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(2)),
    ];
    let valid_probe_definition = [
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(1, r(1)),
        Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
    ];
    let probe = ProbePoint {
        point: p(2, 1, 0),
        side: Classification::Positive,
        planes: vec![valid_probe_definition],
        uncertified_definition_fallback: false,
    };
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];

    assert_eq!(
        trace_segment_without_detours(&ref_point, &probe.point, &[0], &[wall.clone()]),
        Err(HypermeshError::UnknownClassification)
    );

    let winding =
        trace_probe_winding(&ref_point, &[invalid_ref_definition], &probe, &[0], &[wall]).unwrap();

    assert_eq!(winding, vec![0]);
}

#[test]
fn probe_winding_reports_unknown_if_all_definition_paths_are_uncertified() {
    let ref_point = p(0, 0, 0);
    let ref_definitions = [[
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(2)),
    ]];
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];
    let probe = ProbePoint {
        point: p(2, 1, 0),
        side: Classification::Positive,
        planes: Vec::new(),
        uncertified_definition_fallback: false,
    };

    assert_eq!(
        trace_segment_without_detours(&ref_point, &probe.point, &[0], &[wall.clone()]),
        Err(HypermeshError::UnknownClassification)
    );

    let err = trace_probe_winding(&ref_point, &ref_definitions, &probe, &[0], &[wall]).unwrap_err();

    assert_eq!(err, HypermeshError::UnknownClassification);
}
