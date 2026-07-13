use super::*;
use crate::geometry::Plane;
use crate::intersection::OverlapInfo;
use crate::mesh::{OutputVertex, PolygonSoup, prepare_input};
use crate::operations::{EmberConfig, boolean_operation};
use crate::output::{BooleanResult, TriangleSoup, triangulate_and_resolve_certified};
use crate::polygon::make_triangle;
use crate::winding::{BooleanOp, make_indicator};
use crate::{InputMesh, Triangle};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

#[test]
fn unlimited_depth_budget_never_preempts_the_finite_split_basis() {
    assert!(!subdivision_depth_budget_reached(0, usize::MAX));
    assert!(!subdivision_depth_budget_reached(usize::MAX, usize::MAX));
    assert!(subdivision_depth_budget_reached(7, 7));
    assert!(subdivision_depth_budget_reached(8, 7));
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

#[test]
fn reference_target_clones_share_definition_families() {
    let target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let cloned = target.clone();

    assert!(Arc::ptr_eq(&target.definitions, &cloned.definitions));
}

#[test]
fn support_reference_context_clones_share_immutable_families() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let definitions = axis_defs(&p(0, 0, 1));
    let context = support_reference_cache_context_key(
        &p(0, 0, 1),
        &definitions,
        &[0],
        std::slice::from_ref(&polygon),
    );
    let cloned = context.clone();
    let polygon_context = support_reference_polygon_context_key_from_support_context(&context);

    assert!(Arc::ptr_eq(
        &context.old_ref_definitions,
        &cloned.old_ref_definitions
    ));
    assert!(Arc::ptr_eq(&context.old_wnv, &cloned.old_wnv));
    assert!(Arc::ptr_eq(&context.polygons, &cloned.polygons));
    assert!(Arc::ptr_eq(&context.polygons, &polygon_context.polygons));
}

fn assert_certified_reference_result(
    found: Option<(ReferenceTarget, Vec<i32>)>,
    expected_point: &Point3,
    expected_winding: &[i32],
) {
    let (target, winding) = found.expect("expected a certified reference");
    assert_eq!(&target.point, expected_point);
    assert_eq!(winding, expected_winding);
    assert!(!target.uncertified_definition_fallback);
    assert!(!target.definitions.is_empty());
    assert!(
        target
            .definitions
            .iter()
            .all(|definition| affine_from_planes(definition).as_ref() == Ok(expected_point))
    );
}

fn sample_segment_intersection(other_polygon_idx: usize) -> PairwiseIntersection {
    PairwiseIntersection {
        kind: PairwiseIntersectionType::Segment,
        segment: Some(IntersectionSegment {
            v0: p(0, 0, 0),
            v1: p(1, 0, 0),
            split_plane: Plane::axis_aligned(0, r(0)),
            other_polygon_idx,
        }),
        overlap: None,
    }
}

fn quadrilateral_reference_cell_fixture() -> (Aabb, Vec<LimitPlane3>, Point3) {
    let bounds = Aabb::new(p(0, 0, 0), p(5, 4, 0));
    let support = Plane::axis_aligned(2, r(0));
    let interior = Point3::new(q(9, 4), r(2), r(0));
    let vertices = [p(0, 0, 0), p(4, 0, 0), p(5, 4, 0), p(0, 4, 0)];
    let mut halfspaces = vec![
        LimitPlane3::new(support.normal.clone(), support.offset.clone()),
        LimitPlane3::new(
            support.inverted().normal.clone(),
            support.inverted().offset.clone(),
        ),
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
        halfspaces.push(LimitPlane3::new(
            edge_plane.normal.clone(),
            edge_plane.offset.clone(),
        ));
    }

    (bounds, halfspaces, Point3::new(q(5, 2), r(2), r(0)))
}

fn px(x: Real, y: i32, z: i32) -> Point3 {
    Point3::new(x, r(y), r(z))
}

fn axis_defs(point: &Point3) -> Vec<[Plane; 3]> {
    vec![axis_plane_definition(point)]
}

fn tetra_from_face_and_apex(a: Point3, b: Point3, c: Point3, apex: Point3) -> InputMesh {
    InputMesh::new(
        vec![a, b, c, apex],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

fn axis_face_polygon(polygons: &[ConvexPolygon], axis: usize, value: i32) -> ConvexPolygon {
    polygons
        .iter()
        .find(|polygon| {
            compare_real(axis_ref(&polygon.support.normal, axis), &Real::zero())
                .unwrap()
                .is_gt()
                && polygon
                    .vertices()
                    .unwrap()
                    .iter()
                    .all(|vertex| axis_ref(vertex, axis) == &r(value))
        })
        .cloned()
        .expect("expected axis-aligned support face in prepared mesh soup")
}

#[test]
fn cached_leaf_classification_reuses_rotated_edge_cycles() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let mut rotated_edges = polygon.edges[1..].to_vec();
    rotated_edges.push(polygon.edges[0].clone());
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_leaf_classification_with(
        &mut cache,
        None,
        &polygon.support,
        &polygon.edges,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(vec![7])
        },
    )
    .unwrap();
    let second = cached_leaf_classification_with(
        &mut cache,
        None,
        &polygon.support,
        &rotated_edges,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(vec![9])
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, vec![7]);
    assert_eq!(second, vec![7]);
}

#[test]
fn cached_leaf_classification_distinguishes_leaf_context() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let left_context = Arc::new(LeafClassificationCacheContextKey {
        polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        polygons: vec![polygon.clone()],
        bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
        ref_point: p(0, 0, -1),
        ref_definitions: vec![axis_plane_definition(&p(0, 0, -1))],
        ref_wnv: vec![0],
    });
    let right_context = Arc::new(LeafClassificationCacheContextKey {
        polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        polygons: vec![polygon.clone()],
        bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
        ref_point: p(0, 0, 1),
        ref_definitions: vec![axis_plane_definition(&p(0, 0, 1))],
        ref_wnv: vec![0],
    });
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_leaf_classification_with(
        &mut cache,
        Some(&left_context),
        &polygon.support,
        &polygon.edges,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(vec![7])
        },
    )
    .unwrap();
    assert!(Arc::ptr_eq(
        cache[0].context.as_ref().unwrap(),
        &left_context
    ));
    let second = cached_leaf_classification_with(
        &mut cache,
        Some(&right_context),
        &polygon.support,
        &polygon.edges,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(vec![9])
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_eq!(first, vec![7]);
    assert_eq!(second, vec![9]);
}

#[test]
fn cached_leaf_point_classification_reuses_identical_state() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let point = certified_leaf_interior_points(&polygon.support, &polygon.edges)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_leaf_point_classification_with(
        &mut cache,
        None,
        &polygon.support,
        &point,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(LeafPointClassificationState {
                winding: Some(vec![7]),
                saw_unknown: false,
            })
        },
    )
    .unwrap();
    let second = cached_leaf_point_classification_with(
        &mut cache,
        None,
        &polygon.support,
        &point,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(LeafPointClassificationState {
                winding: Some(vec![9]),
                saw_unknown: true,
            })
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(
        first,
        LeafPointClassificationState {
            winding: Some(vec![7]),
            saw_unknown: false,
        }
    );
    assert_eq!(second, first);
}

#[test]
fn cached_leaf_point_classification_distinguishes_context() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let point = certified_leaf_interior_points(&polygon.support, &polygon.edges)
        .unwrap()
        .into_iter()
        .next()
        .unwrap();
    let left_context = Arc::new(LeafClassificationCacheContextKey {
        polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        polygons: vec![polygon.clone()],
        bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
        ref_point: p(0, 0, -1),
        ref_definitions: vec![axis_plane_definition(&p(0, 0, -1))],
        ref_wnv: vec![0],
    });
    let right_context = Arc::new(LeafClassificationCacheContextKey {
        polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        polygons: vec![polygon.clone()],
        bounds: Aabb::new(p(0, 0, 0), p(2, 2, 0)),
        ref_point: p(0, 0, 1),
        ref_definitions: vec![axis_plane_definition(&p(0, 0, 1))],
        ref_wnv: vec![0],
    });
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_leaf_point_classification_with(
        &mut cache,
        Some(&left_context),
        &polygon.support,
        &point,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(LeafPointClassificationState {
                winding: Some(vec![7]),
                saw_unknown: false,
            })
        },
    )
    .unwrap();
    let second = cached_leaf_point_classification_with(
        &mut cache,
        Some(&right_context),
        &polygon.support,
        &point,
        &polygon.delta_w,
        || {
            calls += 1;
            Ok(LeafPointClassificationState {
                winding: Some(vec![9]),
                saw_unknown: false,
            })
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_eq!(first.winding, Some(vec![7]));
    assert_eq!(second.winding, Some(vec![9]));
}

#[test]
fn cached_bsp_leaf_certification_reuses_permuted_polygon_families() {
    let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    host.delta_w = vec![1, 0];
    let mut cutter = make_triangle(&p(2, 0, 0), &p(0, 0, 0), &p(2, -1, 0), 1, 0);
    cutter.delta_w = vec![0, 1];
    let cache = RefCell::new(Vec::new());

    let first_polygons = vec![host.clone(), cutter.clone()];
    let first_intersections = pairwise_intersections_by_polygon(&first_polygons).unwrap();
    let first = cached_bsp_leaf_certification_with(
        &cache,
        &host,
        &host.edges,
        &first_polygons,
        &first_intersections[0],
    )
    .unwrap();

    let second_polygons = vec![cutter, host.clone()];
    let second_intersections = pairwise_intersections_by_polygon(&second_polygons).unwrap();
    let second = cached_bsp_leaf_certification_with(
        &cache,
        &host,
        &host.edges,
        &second_polygons,
        &second_intersections[1],
    )
    .unwrap();

    assert_eq!(first, second);
    assert!(Arc::ptr_eq(&first, &second));
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn bsp_leaf_certification_candidate_indices_use_host_segment_and_overlap_only() {
    let host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let segment_other = make_triangle(&p(1, 0, -1), &p(1, 1, 1), &p(1, 2, -1), 1, 0);
    let overlap_other = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 1, 0), 2, 0);
    let skipped_other = make_triangle(&p(0, 0, 1), &p(2, 0, 1), &p(0, 2, 1), 3, 0);
    let polygons = vec![
        host.clone(),
        segment_other.clone(),
        overlap_other.clone(),
        skipped_other,
    ];
    let intersections = pairwise_intersections_by_polygon(&polygons).unwrap();

    let indices =
        bsp_leaf_certification_candidate_indices(&host, &polygons, Some(&intersections[0]))
            .unwrap();

    assert_eq!(indices, vec![1, 2]);
}

#[test]
fn cached_host_bsp_leaves_reuse_permuted_polygon_families() {
    let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    host.delta_w = vec![1, 0];
    let mut cutter = make_triangle(&p(2, 0, 0), &p(0, 0, 0), &p(2, -1, 0), 1, 0);
    cutter.delta_w = vec![0, 1];
    let cache = RefCell::new(Vec::new());

    let first_polygons = vec![host.clone(), cutter.clone()];
    let first_intersections = pairwise_intersections_by_polygon(&first_polygons).unwrap();
    let first =
        cached_host_bsp_leaves_with(&cache, &host, &first_polygons, &first_intersections[0])
            .unwrap();

    let second_polygons = vec![cutter, host.clone()];
    let second_intersections = pairwise_intersections_by_polygon(&second_polygons).unwrap();
    let second =
        cached_host_bsp_leaves_with(&cache, &host, &second_polygons, &second_intersections[1])
            .unwrap();

    assert_eq!(first, second);
    assert!(Arc::ptr_eq(&first, &second));
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn bsp_leaf_edge_cycle_dedupe_skips_rotated_duplicates() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let mut rotated_edges = polygon.edges[1..].to_vec();
    rotated_edges.push(polygon.edges[0].clone());
    let mut seen = Vec::new();

    assert!(take_new_bsp_leaf_edge_cycle(&mut seen, &polygon.edges));
    assert!(!take_new_bsp_leaf_edge_cycle(&mut seen, &rotated_edges));
    assert_eq!(seen, vec![polygon.edges.as_ref().clone()]);
}

fn vertex_key(vertex: &OutputVertex) -> [String; 3] {
    [
        vertex.x.to_string(),
        vertex.y.to_string(),
        vertex.z.to_string(),
    ]
}

fn sorted_triangle_key(soup: &TriangleSoup, triangle: [usize; 3]) -> [[String; 3]; 3] {
    let mut keys = [
        vertex_key(&soup.vertices[triangle[0]]),
        vertex_key(&soup.vertices[triangle[1]]),
        vertex_key(&soup.vertices[triangle[2]]),
    ];
    keys.sort();
    keys
}

fn assert_same_shape(left: &TriangleSoup, right: &TriangleSoup) {
    let left_faces = left
        .triangles
        .iter()
        .map(|triangle| sorted_triangle_key(left, *triangle))
        .collect::<std::collections::BTreeSet<_>>();
    let right_faces = right
        .triangles
        .iter()
        .map(|triangle| sorted_triangle_key(right, *triangle))
        .collect::<std::collections::BTreeSet<_>>();
    assert_eq!(left_faces, right_faces);
}

fn definition_uses_non_axis_plane(definition: &[Plane; 3]) -> bool {
    definition.iter().any(|plane| {
        plane.normal != p(1, 0, 0) && plane.normal != p(0, 1, 0) && plane.normal != p(0, 0, 1)
    })
}

#[test]
fn can_split_any_certified_positive_extent() {
    let bounds = Aabb::new(p(0, 0, 0), p(1, 0, 0));

    assert!(can_split_bounds(&bounds).unwrap());
}

#[test]
fn select_subdivision_split_prefers_interior_arrangement_gap() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![
        make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0),
        make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0),
    ];

    let (axis, value) = select_subdivision_split(&bounds, &polygons).unwrap();

    assert_eq!(axis, 0);
    assert_eq!(value, q(3, 2));
}

#[test]
fn select_subdivision_split_prefers_nonempty_arrangement_gap() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 0, 0)];

    let (axis, value) = select_subdivision_split(&bounds, &polygons).unwrap();

    assert_eq!(axis, 1);
    assert_eq!(value, r(1));
}

#[test]
fn select_subdivision_split_can_use_intersection_segment_coordinates() {
    let bounds = Aabb::new(p(-3, 0, -1), p(3, 4, 1));
    let horizontal =
        crate::polygon::make_quad(&p(-3, 0, 0), &p(3, 0, 0), &p(3, 4, 0), &p(-3, 4, 0), 0, 0);
    let vertical = make_triangle(&p(-2, 2, -1), &p(2, 2, -1), &p(1, 2, 1), 1, 0);

    let candidates =
        intersection_split_candidates(&bounds, &[horizontal.clone(), vertical.clone()], 0).unwrap();

    assert_eq!(candidates, vec![q(-1, 2), q(3, 2)]);
    let vertex_candidates =
        arrangement_split_candidates(&bounds, &[horizontal, vertical], 0).unwrap();
    assert!(!vertex_candidates.iter().any(|(_, value)| *value == q(1, 2)));
}

#[test]
fn arrangement_split_candidates_from_axis_values_matches_direct_query() {
    let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
    let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![horizontal, vertical];

    let direct = arrangement_split_candidates(&bounds, &polygons, 0).unwrap();
    let axis_values = polygon_axis_values(&polygons).unwrap();
    let cached =
        arrangement_split_candidates_from_axis_values(&bounds, &axis_values[0], 0).unwrap();

    assert_eq!(direct, cached);
}

#[test]
fn cached_polygon_axis_values_reuse_permuted_polygon_families() {
    let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
    let cache = RefCell::new(Vec::new());

    let first =
        cached_polygon_axis_values_with(&cache, &[polygon_a.clone(), polygon_b.clone()]).unwrap();
    let second = cached_polygon_axis_values_with(&cache, &[polygon_b, polygon_a]).unwrap();

    assert_eq!(first, second);
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn cached_polygon_axis_values_memoize_current_equivalent_state() {
    let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
    let cache = RefCell::new(Vec::new());

    cached_polygon_axis_values_with(&cache, &[polygon_a.clone(), polygon_b.clone()]).unwrap();
    cached_polygon_axis_values_with(&cache, &[polygon_b, polygon_a]).unwrap();

    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn subdivision_has_no_split_without_interior_arrangement_events() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![
        make_triangle(&p(0, 0, 0), &p(10, 0, 0), &p(0, 0, 4), 0, 0),
        make_triangle(&p(0, 4, 0), &p(10, 4, 0), &p(0, 4, 4), 1, 0),
    ];

    let splits = ordered_subdivision_splits(&bounds, &polygons).unwrap();

    assert!(splits.is_empty());
}

#[test]
fn cached_subdivision_has_no_split_without_interior_arrangement_events() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![
        make_triangle(&p(0, 0, 0), &p(10, 0, 0), &p(0, 0, 4), 0, 0),
        make_triangle(&p(0, 4, 0), &p(10, 4, 0), &p(0, 4, 4), 1, 0),
    ];
    let caches = SubdivisionRuntimeCaches::default();

    let splits = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &bounds,
        &polygons,
    )
    .unwrap();

    assert!(splits.is_empty());
}

#[test]
fn descendant_splits_only_use_the_cached_root_event_basis() {
    let root_bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let root_polygons = vec![
        make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0),
        make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0),
    ];
    let caches = SubdivisionRuntimeCaches::default();
    let root_basis = cached_root_split_basis_with(
        &caches.split_candidates,
        &caches.polygon_axis_values,
        &caches.pairwise_intersections,
        &root_bounds,
        &root_polygons,
    )
    .unwrap();
    let root_axis_value_cache_len = caches.polygon_axis_values.borrow().len();
    let descendant_bounds = root_bounds.right_half(0, q(3, 2));
    let descendant_polygons = vec![make_triangle(&p(3, 0, 0), &p(4, 2, 0), &p(3, 0, 2), 0, 0)];
    let descendant_axis_values = polygon_axis_values(&descendant_polygons).unwrap();
    let descendant_local_splits = arrangement_split_candidates_from_axis_values(
        &descendant_bounds,
        &descendant_axis_values[0],
        0,
    )
    .unwrap();

    assert!(
        descendant_local_splits
            .iter()
            .any(|(_, value)| *value == q(7, 2))
    );

    let attempts = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &descendant_bounds,
        &descendant_polygons,
    )
    .unwrap();

    assert!(!attempts.is_empty());
    assert_eq!(
        caches.polygon_axis_values.borrow().len(),
        root_axis_value_cache_len
    );
    assert!(attempts.iter().all(|attempt| {
        root_basis
            .iter()
            .any(|split| split.axis == attempt.axis && split.value == attempt.value)
    }));
    assert!(
        !attempts
            .iter()
            .any(|attempt| attempt.axis == 0 && attempt.value == q(7, 2))
    );
    assert_eq!(
        caches.split_candidates.borrow().root_basis,
        Some(Ok(root_basis))
    );
}

#[test]
fn each_root_split_plane_is_removed_from_both_child_branches() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let root_basis = vec![
        RootSplitPlane {
            axis: 0,
            value: r(2),
            source: SplitSource::Arrangement,
        },
        RootSplitPlane {
            axis: 0,
            value: r(6),
            source: SplitSource::Intersection,
        },
        RootSplitPlane {
            axis: 1,
            value: r(4),
            source: SplitSource::Arrangement,
        },
        RootSplitPlane {
            axis: 2,
            value: r(8),
            source: SplitSource::Intersection,
        },
    ];

    for selected in &root_basis {
        let child_bounds = [
            bounds.left_half(selected.axis, selected.value.clone()),
            bounds.right_half(selected.axis, selected.value.clone()),
        ];
        for child_bounds in child_bounds {
            let remaining = root_basis
                .iter()
                .filter(|split| {
                    split_value_is_strictly_inside_bounds(&child_bounds, split.axis, &split.value)
                        .unwrap()
                })
                .collect::<Vec<_>>();
            assert!(remaining.len() < root_basis.len());
            assert!(!remaining.contains(&selected));
        }
    }
}

#[test]
fn subdivision_exhausts_arrangement_splits_before_depth_budget() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![
        make_triangle(&p(0, 0, 0), &p(10, 0, 0), &p(0, 0, 4), 0, 0),
        make_triangle(&p(0, 4, 0), &p(10, 4, 0), &p(0, 4, 4), 1, 0),
    ];
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let caches = SubdivisionRuntimeCaches::default();
    let mut output = Vec::new();
    let mut leaf_calls = 0;

    let err = subdivide_into_inner_with(
        SubdivisionTask::new(polygons, bounds, p(-1, -1, -1), vec![0]),
        &indicator,
        SubdivisionConfig { max_depth: 0 },
        None,
        &mut output,
        &mut |_task, _indicator, _output| {
            leaf_calls += 1;
            Err(crate::error::HypermeshError::UnknownClassification)
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    assert_eq!(leaf_calls, 1);
    assert!(output.is_empty());
}

#[test]
fn certified_root_leaf_preempts_available_arrangement_splits() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![
        make_triangle(&p(0, 0, 0), &p(10, 0, 0), &p(0, 0, 4), 0, 0),
        make_triangle(&p(0, 4, 0), &p(10, 4, 0), &p(0, 4, 4), 1, 0),
    ];
    let emitted = ClassifiedPolygon::new(polygons[0].clone(), 1);
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let caches = SubdivisionRuntimeCaches::default();
    let mut output = Vec::new();
    let mut leaf_calls = 0;

    subdivide_into_inner_with(
        SubdivisionTask::new(polygons, bounds, p(-1, -1, -1), vec![0]),
        &indicator,
        SubdivisionConfig::default(),
        None,
        &mut output,
        &mut |task, _indicator, local_output| {
            leaf_calls += 1;
            assert_eq!(task.depth, 0);
            local_output.push(emitted.clone());
            Ok(LeafProcessingStats {
                polygon_count: 2,
                certified_complete: true,
                ..LeafProcessingStats::default()
            })
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap();

    assert_eq!(leaf_calls, 1);
    assert_eq!(output, vec![emitted]);
    assert!(caches.split_candidates.borrow().entries.is_empty());
    assert!(caches.split_child_partitions.borrow().is_empty());
    assert!(caches.child_reference.borrow().is_empty());
}

#[test]
fn intersection_split_candidates_can_beat_arrangement_improvement() {
    let mut best_axis = 0;
    let mut best_value = r(5);
    let mut best_counts = (6, 0, 9, 1, 3, 2);

    consider_split_candidates(
        &mut best_axis,
        &mut best_value,
        &mut best_counts,
        0,
        [r(4)],
        |_value| Ok((5, 0, 8, 1, 2, 1)),
    )
    .unwrap();

    assert_eq!(best_axis, 0);
    assert_eq!(best_value, r(4));
    assert_eq!(best_counts, (5, 0, 8, 1, 2, 1));

    consider_split_candidates(
        &mut best_axis,
        &mut best_value,
        &mut best_counts,
        1,
        [r(2)],
        |_value| Ok((4, 0, 4, 0, 0, 0)),
    )
    .unwrap();

    assert_eq!(best_axis, 1);
    assert_eq!(best_value, r(2));
    assert_eq!(best_counts, (4, 0, 4, 0, 0, 0));
}

#[test]
fn intersection_split_sources_win_arrangement_ties() {
    let mut candidates = vec![
        SplitCandidate {
            axis: 1,
            value: r(2),
            counts: (4, 0, 4, 0, 1, 0),
            source: SplitSource::Arrangement,
        },
        SplitCandidate {
            axis: 2,
            value: r(1),
            counts: (4, 0, 4, 0, 1, 0),
            source: SplitSource::Intersection,
        },
    ];

    candidates.sort_by(|left, right| {
        left.counts
            .cmp(&right.counts)
            .then_with(|| left.source.cmp(&right.source))
    });

    assert_eq!(
        candidates
            .into_iter()
            .map(|candidate| (candidate.axis, candidate.value, candidate.source))
            .collect::<Vec<_>>(),
        vec![
            (2, r(1), SplitSource::Intersection),
            (1, r(2), SplitSource::Arrangement),
        ]
    );
}

#[test]
fn duplicate_arrangement_split_candidate_promotes_to_intersection_source() {
    let polygons = vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)];
    let mut candidates = vec![SplitCandidate {
        axis: 0,
        value: r(5),
        counts: (1, 0, 2, 0, 0, 0),
        source: SplitSource::Arrangement,
    }];

    push_split_candidate(
        &mut candidates,
        &polygons,
        0,
        r(5),
        SplitSource::Intersection,
    )
    .unwrap();
    assert_eq!(candidates.len(), 1);
    assert_eq!(candidates[0].source, SplitSource::Intersection);
}

#[test]
fn split_ranking_penalizes_empty_child_splits() {
    let mut best_axis = 0;
    let mut best_value = r(5);
    let mut best_counts = (4, 0, 6, 0, 2, 0);

    consider_split_candidates(
        &mut best_axis,
        &mut best_value,
        &mut best_counts,
        1,
        [r(1)],
        |_value| Ok((4, 1, 4, 0, 0, 4)),
    )
    .unwrap();

    assert_eq!(best_axis, 0);
    assert_eq!(best_value, r(5));
    assert_eq!(best_counts, (4, 0, 6, 0, 2, 0));
}

#[test]
fn split_ranking_prefers_lower_total_child_count_on_max_count_tie() {
    let mut best_axis = 0;
    let mut best_value = r(5);
    let mut best_counts = (4, 0, 8, 0, 2, 1);

    consider_split_candidates(
        &mut best_axis,
        &mut best_value,
        &mut best_counts,
        1,
        [r(2)],
        |_value| Ok((4, 0, 6, 0, 2, 1)),
    )
    .unwrap();

    assert_eq!(best_axis, 1);
    assert_eq!(best_value, r(2));
    assert_eq!(best_counts, (4, 0, 6, 0, 2, 1));
}

#[test]
fn split_ranking_prefers_lower_child_imbalance_on_count_tie() {
    let mut best_axis = 0;
    let mut best_value = r(5);
    let mut best_counts = (4, 0, 6, 0, 2, 5);

    consider_split_candidates(
        &mut best_axis,
        &mut best_value,
        &mut best_counts,
        1,
        [r(2)],
        |_value| Ok((4, 0, 6, 0, 2, 1)),
    )
    .unwrap();

    assert_eq!(best_axis, 1);
    assert_eq!(best_value, r(2));
    assert_eq!(best_counts, (4, 0, 6, 0, 2, 1));
}

#[test]
fn split_ranking_prefers_candidates_without_unchanged_parent_children() {
    let mut best_axis = 0;
    let mut best_value = r(5);
    let mut best_counts = (4, 0, 6, 1, 2, 1);

    consider_split_candidates(
        &mut best_axis,
        &mut best_value,
        &mut best_counts,
        1,
        [r(2)],
        |_value| Ok((4, 0, 6, 0, 2, 1)),
    )
    .unwrap();

    assert_eq!(best_axis, 1);
    assert_eq!(best_value, r(2));
    assert_eq!(best_counts, (4, 0, 6, 0, 2, 1));
}

#[test]
fn ordered_subdivision_splits_rank_best_candidate_first() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![
        make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0),
        make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0),
    ];

    let ordered = ordered_subdivision_splits(&bounds, &polygons).unwrap();

    assert!(!ordered.is_empty());
    assert_eq!(ordered[0], (0, q(3, 2)));
}

#[test]
fn intersection_split_candidates_from_segments_matches_direct_query() {
    let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
    let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![horizontal, vertical];

    let direct = intersection_split_candidates(&bounds, &polygons, 0).unwrap();
    let segments = split_intersection_segments(&polygons).unwrap();
    let cached = intersection_split_candidates_from_segments(&bounds, &segments, 0).unwrap();

    assert_eq!(direct, cached);
}

#[test]
fn split_intersection_segments_with_pairwise_cache_matches_direct_query() {
    let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
    let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
    let polygons = vec![horizontal, vertical];
    let cache = RefCell::new(Vec::<PairwiseIntersectionsCacheEntry>::new());

    let direct = split_intersection_segments(&polygons).unwrap();
    let cached = split_intersection_segments_with_pairwise_cache(&cache, &polygons).unwrap();

    assert_eq!(direct, cached);
}

#[test]
fn cached_ordered_subdivision_splits_reuse_permuted_polygon_families() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
    let axis_value_cache = RefCell::new(Vec::new());
    let cache = RefCell::new(SplitCandidatesCache::default());
    let fanout_cache = RefCell::new(Vec::new());
    let partition_cache = RefCell::new(Vec::new());
    let pairwise_cache = RefCell::new(Vec::new());

    let first = cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &cache,
        &fanout_cache,
        &partition_cache,
        &RefCell::new(Vec::new()),
        &pairwise_cache,
        &bounds,
        &[polygon_a.clone(), polygon_b.clone()],
    )
    .unwrap();
    let second = cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &cache,
        &fanout_cache,
        &partition_cache,
        &RefCell::new(Vec::new()),
        &pairwise_cache,
        &bounds,
        &[polygon_b, polygon_a],
    )
    .unwrap();

    assert_eq!(
        first
            .iter()
            .map(|candidate| (candidate.axis, candidate.value.clone()))
            .collect::<Vec<_>>(),
        second
            .iter()
            .map(|candidate| (candidate.axis, candidate.value.clone()))
            .collect::<Vec<_>>()
    );
    assert_eq!(cache.borrow().entries.len(), 2);
}

#[test]
fn cached_unique_subdivision_split_attempt_count_reuses_equivalent_child_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
    let mut cache = Vec::new();
    let calls = std::cell::Cell::new(0);

    let first = cached_unique_subdivision_split_attempt_count_with_query(
        &mut cache,
        &bounds,
        &[polygon_a.clone(), polygon_b.clone()],
        || {
            calls.set(calls.get() + 1);
            Ok(3)
        },
    )
    .unwrap();
    let second = cached_unique_subdivision_split_attempt_count_with_query(
        &mut cache,
        &bounds,
        &[polygon_b, polygon_a],
        || {
            calls.set(calls.get() + 1);
            Ok(9)
        },
    )
    .unwrap();

    assert_eq!(first, 3);
    assert_eq!(second, 3);
    assert_eq!(calls.get(), 1);
    assert_eq!(cache.len(), 2);
}

#[test]
fn cached_ordered_subdivision_splits_memoize_current_equivalent_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
    let axis_value_cache = RefCell::new(Vec::new());
    let cache = RefCell::new(SplitCandidatesCache::default());
    let fanout_cache = RefCell::new(Vec::new());
    let partition_cache = RefCell::new(Vec::new());
    let pairwise_cache = RefCell::new(Vec::new());
    let polygon_bounds_cache = RefCell::new(Vec::new());

    cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &cache,
        &fanout_cache,
        &partition_cache,
        &polygon_bounds_cache,
        &pairwise_cache,
        &bounds,
        &[polygon_a.clone(), polygon_b.clone()],
    )
    .unwrap();
    cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &cache,
        &fanout_cache,
        &partition_cache,
        &polygon_bounds_cache,
        &pairwise_cache,
        &bounds,
        &[polygon_b, polygon_a],
    )
    .unwrap();

    assert_eq!(cache.borrow().entries.len(), 2);
}

#[test]
fn cached_ordered_subdivision_splits_cache_distinguishes_bounds_even_when_results_match() {
    let polygon = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let axis_value_cache = RefCell::new(Vec::new());
    let cache = RefCell::new(SplitCandidatesCache::default());
    let fanout_cache = RefCell::new(Vec::new());
    let partition_cache = RefCell::new(Vec::new());
    let pairwise_cache = RefCell::new(Vec::new());
    let first_bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let second_bounds = Aabb::new(p(0, 0, 0), p(8, 4, 4));

    let first = cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &cache,
        &fanout_cache,
        &partition_cache,
        &RefCell::new(Vec::new()),
        &pairwise_cache,
        &first_bounds,
        std::slice::from_ref(&polygon),
    )
    .unwrap();
    let second = cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &cache,
        &fanout_cache,
        &partition_cache,
        &RefCell::new(Vec::new()),
        &pairwise_cache,
        &second_bounds,
        &[polygon],
    )
    .unwrap();

    assert_eq!(
        first
            .iter()
            .map(|candidate| (candidate.axis, candidate.value.clone()))
            .collect::<Vec<_>>(),
        second
            .iter()
            .map(|candidate| (candidate.axis, candidate.value.clone()))
            .collect::<Vec<_>>()
    );
    assert_eq!(cache.borrow().entries.len(), 2);
}

#[test]
fn cached_ordered_subdivision_splits_populate_partition_cache() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 4, 4));
    let polygons = vec![
        make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0),
        make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0),
    ];
    let axis_value_cache = RefCell::new(Vec::new());
    let split_cache = RefCell::new(SplitCandidatesCache::default());
    let fanout_cache = RefCell::new(Vec::new());
    let partition_cache = RefCell::new(Vec::new());
    let polygon_bounds_cache = RefCell::new(Vec::new());
    let pairwise_cache = RefCell::new(Vec::new());

    let ordered = cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &split_cache,
        &fanout_cache,
        &partition_cache,
        &polygon_bounds_cache,
        &pairwise_cache,
        &bounds,
        &polygons,
    )
    .unwrap();
    let cached_partition_count = partition_cache.borrow().len();

    assert!(!ordered.is_empty());
    assert!(cached_partition_count > 0);

    let axis = ordered[0].axis;
    let value = &ordered[0].value;
    let _partition =
        cached_split_child_partition_with(&partition_cache, &polygons, axis, value).unwrap();

    assert_eq!(partition_cache.borrow().len(), cached_partition_count);
}

#[test]
fn cached_ordered_subdivision_splits_dedupes_equivalent_child_partitions() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygon = make_triangle(&p(1, 1, 1), &p(2, 1, 1), &p(1, 2, 1), 0, 0);
    let polygons = vec![polygon];
    let axis_value_cache = RefCell::new(Vec::new());
    let split_cache = RefCell::new(SplitCandidatesCache::default());
    let fanout_cache = RefCell::new(Vec::new());
    let partition_cache = RefCell::new(Vec::new());
    let polygon_bounds_cache = RefCell::new(Vec::new());
    let pairwise_cache = RefCell::new(Vec::new());

    let raw = ordered_subdivision_splits(&bounds, &polygons).unwrap();
    let deduped = cached_ordered_subdivision_splits_with(
        &axis_value_cache,
        &split_cache,
        &fanout_cache,
        &partition_cache,
        &polygon_bounds_cache,
        &pairwise_cache,
        &bounds,
        &polygons,
    )
    .unwrap();

    assert!(deduped.len() < raw.len());
}

#[test]
fn cached_split_child_partition_reuses_permuted_polygon_families() {
    let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
    let cache = RefCell::new(Vec::new());

    let first = cached_split_child_partition_with(
        &cache,
        &[polygon_a.clone(), polygon_b.clone()],
        0,
        &r(3),
    )
    .unwrap();
    let second =
        cached_split_child_partition_with(&cache, &[polygon_b, polygon_a], 0, &r(3)).unwrap();

    assert_eq!(first, second);
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn cached_split_child_partition_memoizes_current_equivalent_state() {
    let polygon_a = make_triangle(&p(1, 0, 0), &p(1, 2, 0), &p(1, 0, 2), 0, 0);
    let polygon_b = make_triangle(&p(2, 0, 0), &p(2, 2, 0), &p(2, 0, 2), 1, 0);
    let cache = RefCell::new(Vec::new());

    cached_split_child_partition_with(&cache, &[polygon_a.clone(), polygon_b.clone()], 0, &r(3))
        .unwrap();
    cached_split_child_partition_with(&cache, &[polygon_b, polygon_a], 0, &r(3)).unwrap();

    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn ordered_subdivision_split_search_backtracks_after_unknown_candidate() {
    let candidates = vec![(0, r(1)), (1, r(2))];
    let mut visited = Vec::new();

    let found = try_ordered_subdivision_splits(&candidates, |axis, value| {
        visited.push((axis, value.clone()));
        if axis == 0 {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            Ok((axis, value.clone()))
        }
    })
    .unwrap();

    assert_eq!(visited, candidates);
    assert_eq!(found, (1, r(2)));
}

#[test]
fn ordered_subdivision_split_search_keeps_strongest_failure() {
    let candidates = vec![(0, r(1)), (1, r(2)), (2, r(3))];

    let err = try_ordered_subdivision_splits(&candidates, |axis, _value| match axis {
        0 => Err::<(usize, Real), crate::error::HypermeshError>(
            crate::error::HypermeshError::UnknownClassification,
        ),
        1 => Err::<(usize, Real), crate::error::HypermeshError>(
            crate::error::HypermeshError::ReferencePropagationFailed,
        ),
        _ => Err::<(usize, Real), crate::error::HypermeshError>(
            crate::error::HypermeshError::SubdivisionDepthLimit {
                depth: 7,
                polygon_count: 11,
            },
        ),
    })
    .unwrap_err();

    assert_eq!(
        err,
        crate::error::HypermeshError::SubdivisionDepthLimit {
            depth: 7,
            polygon_count: 11,
        }
    );
}

#[test]
fn cannot_split_zero_extent_bounds() {
    let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));

    assert!(!can_split_bounds(&bounds).unwrap());
}

#[test]
fn point_strictly_inside_bounds_rejects_positive_extent_boundary() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    assert!(!point_strictly_inside_bounds(&p(0, 2, 2), &bounds).unwrap());
    assert!(point_strictly_inside_bounds(&p(2, 2, 2), &bounds).unwrap());
}

#[test]
fn point_strictly_inside_bounds_accepts_zero_extent_axis_on_plane() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 0, 4));

    assert!(point_strictly_inside_bounds(&p(2, 0, 2), &bounds).unwrap());
    assert!(!point_strictly_inside_bounds(&p(2, 1, 2), &bounds).unwrap());
}

#[test]
fn projected_reference_targets_preserve_strict_inherited_axes() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let targets = projected_reference_targets(&p(0, 2, 5), &bounds).unwrap();

    assert!(!targets.is_empty());
    for target in &targets {
        assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
        assert_eq!(target.point.y, r(2));
    }
}

#[test]
fn compute_new_reference_uses_projected_target_family() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let (point, definitions, winding) =
        compute_new_reference(&p(0, 2, 5), &axis_defs(&p(0, 2, 5)), &[0], &bounds, &[]).unwrap();

    assert!(point_strictly_inside_bounds(&point, &bounds).unwrap());
    assert!(!definitions.is_empty());
    assert_eq!(winding, vec![0]);
}

#[test]
fn compute_new_reference_replaces_uncertified_inherited_definitions() {
    let old_ref = p(2, 2, 2);
    let mismatched = axis_plane_definition(&p(3, 2, 2));
    let singular = [
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(0, r(3)),
        Plane::axis_aligned(2, r(2)),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let (point, definitions, winding) =
        compute_new_reference(&old_ref, &[mismatched, singular], &[7], &bounds, &[]).unwrap();

    assert_eq!(point, old_ref);
    assert_eq!(definitions, axis_defs(&point));
    assert_eq!(winding, vec![7]);
}

#[test]
fn certified_reference_definitions_keep_unique_matching_non_axis_triples() {
    let point = p(2, 2, 2);
    let matching = [
        Plane::from_coefficients(r(1), r(1), r(0), r(-4)),
        Plane::axis_aligned(1, r(2)),
        Plane::axis_aligned(2, r(2)),
    ];
    let permuted = [
        matching[2].clone(),
        matching[0].clone(),
        matching[1].clone(),
    ];
    let mismatched = axis_plane_definition(&p(3, 2, 2));
    let singular = [
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(0, r(3)),
        Plane::axis_aligned(2, r(2)),
    ];

    let definitions = certified_reference_definitions(
        &point,
        &[mismatched, matching.clone(), singular, permuted],
    );

    assert_eq!(definitions, vec![matching]);
}

#[test]
fn compute_new_reference_skips_projected_search_after_support_hit() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();

    let polygons = vec![
        axis_face_polygon(&soup.polygons, 0, 5),
        axis_face_polygon(&soup.polygons, 1, 5),
        axis_face_polygon(&soup.polygons, 2, 5),
    ];
    let old_ref = p(0, 5, 5);
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let old_defs = axis_defs(&old_ref);
    let old_wnv = vec![0; soup.num_meshes];

    let support = support_plane_cell_reference_with_query_caches(
        &old_ref,
        &old_defs,
        &old_wnv,
        &bounds,
        &polygons,
        &mut SupportReferenceQueryCaches::default(),
    )
    .unwrap()
    .expect("support search should find a witness");

    let mut query_caches = SupportReferenceQueryCaches::default();
    let (point, definitions, winding) = compute_new_reference_with_query_caches(
        &old_ref,
        &old_defs,
        &old_wnv,
        &bounds,
        &polygons,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(point, support.0.point);
    assert_eq!(&definitions, support.0.definitions.as_ref());
    assert_eq!(winding, support.1);
    assert!(query_caches.projected_reference_result_cache.is_empty());
    assert!(query_caches.projected_root_cache.is_empty());
}

#[test]
fn alternate_support_reference_matches_general_boolean_results() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let refs = [x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()];
    for op in [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::SymmetricDifference,
    ] {
        let indicator = make_indicator(op, soup.num_meshes);
        let classified = subdivide(
            SubdivisionTask::new(
                soup.polygons.clone(),
                Aabb::new(p(0, 0, 0), p(10, 10, 10)),
                p(0, 5, 5),
                vec![0; soup.num_meshes],
            ),
            &indicator,
            SubdivisionConfig { max_depth: 4 },
        )
        .unwrap_or_else(|err| panic!("alternate {op:?} failed: {err:?}"));

        let alternate_result = BooleanResult::from_classified(
            PolygonSoup {
                polygons: Vec::new(),
                bounds: soup.bounds.clone(),
                num_meshes: soup.num_meshes,
            },
            classified,
        );
        let alternate_soup = triangulate_and_resolve_certified(&alternate_result)
            .unwrap_or_else(|err| panic!("alternate triangulation {op:?} failed: {err:?}"));

        let general_result = boolean_operation(&refs, op, EmberConfig { max_depth: 4 })
            .unwrap_or_else(|err| panic!("general {op:?} failed: {err:?}"));
        let general_soup = triangulate_and_resolve_certified(&general_result)
            .unwrap_or_else(|err| panic!("general triangulation {op:?} failed: {err:?}"));

        assert_same_shape(&alternate_soup, &general_soup);
    }
}

#[test]
fn ordered_subdivision_splits_keep_lower_child_load_ahead_of_downstream_fanout() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let caches = SubdivisionRuntimeCaches::default();
    let root_task = contract_task_to_polygon_family_bounds_if_tighter(
        &SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        ),
        &caches,
    )
    .unwrap()
    .unwrap_or_else(|| {
        SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        )
    });

    let root_attempts = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &root_task.bounds,
        &root_task.polygons,
    )
    .unwrap();
    let hot_root_attempt = root_attempts
        .into_iter()
        .find(|attempt| {
            let mut sizes = [attempt.left_polys.len(), attempt.right_polys.len()];
            sizes.sort_unstable();
            sizes == [6, 10]
        })
        .unwrap();
    let hot_child = ordered_split_attempt_children(
        &root_task.polygons,
        hot_root_attempt.left_polys,
        hot_root_attempt.left_bounds,
        hot_root_attempt.right_polys,
        hot_root_attempt.right_bounds,
    )
    .into_iter()
    .find(|child| child.polygons.len() == 10)
    .unwrap();
    let (hot_ref, hot_defs, hot_wnv) =
        propagate_child_reference(&root_task, &hot_child.polygons, &hot_child.bounds, &caches)
            .unwrap();
    let hot_task = SubdivisionTask {
        polygons: hot_child.polygons,
        bounds: hot_child.bounds,
        ref_point: hot_ref,
        ref_definitions: hot_defs,
        ref_wnv: hot_wnv,
        depth: root_task.depth + 1,
    };

    let hot_attempts = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &hot_task.bounds,
        &hot_task.polygons,
    )
    .unwrap();
    let first = hot_attempts.first().unwrap();
    let mut first_sizes = [first.left_polys.len(), first.right_polys.len()];
    first_sizes.sort_unstable();

    assert_eq!(first_sizes, [7, 8]);
}

#[test]
fn full_soup_hot_fragment_classifies_with_positive_normal_probe() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let caches = SubdivisionRuntimeCaches::default();
    let root_task = contract_task_to_polygon_family_bounds_if_tighter(
        &SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        ),
        &caches,
    )
    .unwrap()
    .unwrap_or_else(|| {
        SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        )
    });

    let root_attempts = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &root_task.bounds,
        &root_task.polygons,
    )
    .unwrap();
    let root_attempt = root_attempts
        .into_iter()
        .find(|attempt| {
            let mut sizes = [attempt.left_polys.len(), attempt.right_polys.len()];
            sizes.sort_unstable();
            sizes == [6, 10]
        })
        .unwrap();
    let root_children = ordered_split_attempt_children(
        &root_task.polygons,
        root_attempt.left_polys,
        root_attempt.left_bounds,
        root_attempt.right_polys,
        root_attempt.right_bounds,
    );
    let hot_child = root_children
        .into_iter()
        .find(|child| child.polygons.len() == 10)
        .unwrap();

    let (hot_ref, hot_defs, hot_wnv) =
        propagate_child_reference(&root_task, &hot_child.polygons, &hot_child.bounds, &caches)
            .unwrap();
    let hot_task = SubdivisionTask {
        polygons: hot_child.polygons,
        bounds: hot_child.bounds,
        ref_point: hot_ref,
        ref_definitions: hot_defs,
        ref_wnv: hot_wnv,
        depth: root_task.depth + 1,
    };

    let hot_attempts = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &hot_task.bounds,
        &hot_task.polygons,
    )
    .unwrap();
    let hot_attempt = hot_attempts
        .iter()
        .find(|attempt| {
            let mut sizes = [attempt.left_polys.len(), attempt.right_polys.len()];
            sizes.sort_unstable();
            sizes == [5, 9]
        })
        .unwrap();
    let hot_split_children = ordered_split_attempt_children(
        &hot_task.polygons,
        hot_attempt.left_polys.clone(),
        hot_attempt.left_bounds.clone(),
        hot_attempt.right_polys.clone(),
        hot_attempt.right_bounds.clone(),
    );
    for child in hot_split_children {
        let (child_ref, child_defs, child_wnv) =
            propagate_child_reference(&hot_task, &child.polygons, &child.bounds, &caches).unwrap();
        let child_task = SubdivisionTask {
            polygons: child.polygons,
            bounds: child.bounds,
            ref_point: child_ref,
            ref_definitions: child_defs,
            ref_wnv: child_wnv,
            depth: hot_task.depth + 1,
        };
        if child_task.polygons.len() == 5 {
            let intersections = pairwise_intersections_by_polygon(&child_task.polygons).unwrap();
            for index in ordered_leaf_polygon_indices_by_intersections(&intersections) {
                let polygon = &child_task.polygons[index];
                if polygon.mesh_index != 0
                    || polygon.polygon_index != 3
                    || intersections[index].is_empty()
                {
                    continue;
                }
                let bsp_leaves =
                    build_host_bsp_leaves(polygon, &child_task.polygons, &intersections[index])
                        .unwrap();
                for leaf in bsp_leaves {
                    if leaf.edges.len() != 4 {
                        continue;
                    }
                    let Ok((interior_points, effective_delta_w)) =
                        certify_bsp_leaf_and_delta_w_with_host_intersections(
                            polygon,
                            &leaf.edges,
                            &child_task.polygons,
                            Some(&intersections[index]),
                        )
                    else {
                        continue;
                    };
                    if interior_points.len() != 4 || effective_delta_w != vec![1, 0, 0] {
                        continue;
                    }

                    let winding = crate::segment_trace::classify_leaf_polygon_from_interior_points_with_probe_query_caches(
                        std::slice::from_ref(&interior_points[0]),
                        &polygon.support,
                        &child_task.ref_point,
                        &child_task.ref_definitions,
                        &child_task.ref_wnv,
                        &child_task.polygons,
                        &child_task.bounds,
                        &effective_delta_w,
                        &mut LeafProbeQueryCaches::default(),
                    )
                    .unwrap();
                    assert_eq!(winding, vec![0, 0, 1]);
                    return;
                }
            }
        }
    }

    panic!("failed to find the full-soup hot fragment regression target");
}

#[test]
fn full_soup_root_host_nine_leaf_one_point_zero_classifies() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let caches = SubdivisionRuntimeCaches::default();
    let root_task = contract_task_to_polygon_family_bounds_if_tighter(
        &SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        ),
        &caches,
    )
    .unwrap()
    .unwrap_or_else(|| {
        SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        )
    });

    let root_attempts = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &root_task.bounds,
        &root_task.polygons,
    )
    .unwrap();
    let root_attempt = root_attempts
        .into_iter()
        .find(|attempt| {
            let mut sizes = [attempt.left_polys.len(), attempt.right_polys.len()];
            sizes.sort_unstable();
            sizes == [6, 10]
        })
        .unwrap();
    let root_children = ordered_split_attempt_children(
        &root_task.polygons,
        root_attempt.left_polys,
        root_attempt.left_bounds,
        root_attempt.right_polys,
        root_attempt.right_bounds,
    );
    let hot_child = root_children
        .into_iter()
        .find(|child| child.polygons.len() == 10)
        .unwrap();

    let (hot_ref, hot_defs, hot_wnv) =
        propagate_child_reference(&root_task, &hot_child.polygons, &hot_child.bounds, &caches)
            .unwrap();
    let hot_task = SubdivisionTask {
        polygons: hot_child.polygons,
        bounds: hot_child.bounds,
        ref_point: hot_ref,
        ref_definitions: hot_defs,
        ref_wnv: hot_wnv,
        depth: root_task.depth + 1,
    };

    let hot_attempts = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &hot_task.bounds,
        &hot_task.polygons,
    )
    .unwrap();
    let hot_attempt = hot_attempts
        .iter()
        .find(|attempt| {
            let mut sizes = [attempt.left_polys.len(), attempt.right_polys.len()];
            sizes.sort_unstable();
            sizes == [5, 9]
        })
        .unwrap();
    let hot_split_children = ordered_split_attempt_children(
        &hot_task.polygons,
        hot_attempt.left_polys.clone(),
        hot_attempt.left_bounds.clone(),
        hot_attempt.right_polys.clone(),
        hot_attempt.right_bounds.clone(),
    );

    for child in hot_split_children {
        let (child_ref, child_defs, child_wnv) =
            propagate_child_reference(&hot_task, &child.polygons, &child.bounds, &caches).unwrap();
        let child_task = SubdivisionTask {
            polygons: child.polygons,
            bounds: child.bounds,
            ref_point: child_ref,
            ref_definitions: child_defs,
            ref_wnv: child_wnv,
            depth: hot_task.depth + 1,
        };
        if child_task.polygons.len() != 9 {
            continue;
        }

        let intersections = pairwise_intersections_by_polygon(&child_task.polygons).unwrap();
        for index in ordered_leaf_polygon_indices_by_intersections(&intersections) {
            let polygon = &child_task.polygons[index];
            if polygon.mesh_index != 2
                || polygon.polygon_index != 9
                || intersections[index].is_empty()
            {
                continue;
            }
            let bsp_leaves =
                build_host_bsp_leaves(polygon, &child_task.polygons, &intersections[index])
                    .unwrap();
            for (leaf_index, leaf) in bsp_leaves.iter().enumerate() {
                let Ok((interior_points, effective_delta_w)) =
                    certify_bsp_leaf_and_delta_w_with_host_intersections(
                        polygon,
                        &leaf.edges,
                        &child_task.polygons,
                        Some(&intersections[index]),
                    )
                else {
                    continue;
                };
                if leaf_index != 1
                    || interior_points.len() != 3
                    || effective_delta_w != vec![0, 0, 1]
                {
                    continue;
                }

                let winding = crate::segment_trace::classify_leaf_polygon_from_interior_points_with_probe_query_caches(
                    std::slice::from_ref(&interior_points[0]),
                    &polygon.support,
                    &child_task.ref_point,
                    &child_task.ref_definitions,
                    &child_task.ref_wnv,
                    &child_task.polygons,
                    &child_task.bounds,
                    &effective_delta_w,
                    &mut LeafProbeQueryCaches::default(),
                )
                .unwrap();
                assert_eq!(leaf.edges.len(), 3);
                assert_eq!(winding, vec![1, 0, 0]);
                return;
            }
        }
    }

    panic!("failed to find root host9 leaf1 point0 regression target");
}

#[test]
fn ordered_reference_search_polygons_prefers_bounds_overlaps() {
    let overlapping = make_triangle(&p(1, 1, 1), &p(3, 1, 1), &p(1, 3, 1), 10, 0);
    let disjoint = make_triangle(&p(8, 8, 8), &p(9, 8, 8), &p(8, 9, 8), 20, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let ordered = ordered_reference_search_polygons(&[disjoint, overlapping.clone()], &bounds);

    assert_eq!(ordered[0].mesh_index, overlapping.mesh_index);
    assert_eq!(ordered[0].polygon_index, overlapping.polygon_index);
}

#[test]
fn projected_support_plane_cell_reference_certifies_interior_target_after_boundary_seed() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    let found = support_plane_cell_reference_with_halfspaces(
        &p(0, 2, 5),
        &axis_defs(&p(0, 2, 5)),
        &[0],
        &bounds,
        &[],
        projected_reference_halfspaces(&p(0, 2, 5), &bounds).unwrap(),
    )
    .unwrap();

    assert_certified_reference_result(found, &p(2, 2, 2), &[0]);
}

#[test]
fn projected_reference_search_tries_projected_support_before_escape() {
    use std::cell::RefCell;

    let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
    let support_target = ReferenceTarget::axis_defined(p(2, 2, 3));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        std::slice::from_ref(&projected),
        std::slice::from_ref(&projected),
        || {
            calls.borrow_mut().push("projected_support");
            Ok(Some((support_target.clone(), vec![7])))
        },
        |target| {
            calls.borrow_mut().push("direct");
            assert_eq!(target, &projected);
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("axis_escape");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("tight_escape");
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some((support_target, vec![7])));
    assert_eq!(*calls.borrow(), vec!["direct", "projected_support"]);
}

#[test]
fn projected_reference_search_lazy_escape_skips_escape_build_after_projected_support_hit() {
    let support_target = ReferenceTarget::axis_defined(p(2, 2, 3));
    let mut escape_builds = 0;

    let found = search_projected_reference_families_lazy_escape(
        &[],
        || Ok(Some((support_target.clone(), vec![7]))),
        |_target| unreachable!("no direct projected targets"),
        || {
            escape_builds += 1;
            Ok(vec![ReferenceTarget::axis_defined(p(3, 2, 3))])
        },
        |_target| unreachable!("escape search should not run"),
        |_target| unreachable!("escape search should not run"),
    )
    .unwrap();

    assert_eq!(found, Some((support_target, vec![7])));
    assert_eq!(escape_builds, 0);
}

#[test]
fn projected_reference_search_lazy_escape_builds_escape_targets_after_support_miss() {
    let escape_target = ReferenceTarget::axis_defined(p(3, 2, 3));
    let mut escape_builds = 0;

    let found = search_projected_reference_families_lazy_escape(
        &[],
        || Ok(None),
        |target| {
            assert_eq!(target, &escape_target);
            Ok(Some(vec![9]))
        },
        || {
            escape_builds += 1;
            Ok(vec![escape_target.clone()])
        },
        |_target| unreachable!("direct escape trace should already succeed"),
        |_target| unreachable!("direct escape trace should already succeed"),
    )
    .unwrap();

    assert_eq!(found, Some((escape_target, vec![9])));
    assert_eq!(escape_builds, 1);
}

#[test]
fn projected_reference_search_backtracks_after_uncertified_projected_support() {
    use std::cell::RefCell;

    let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
    let axis_target = ReferenceTarget::axis_defined(p(3, 2, 3));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        std::slice::from_ref(&projected),
        std::slice::from_ref(&projected),
        || {
            calls.borrow_mut().push("projected_support");
            Err(crate::error::HypermeshError::UnknownClassification)
        },
        |target| {
            calls.borrow_mut().push("direct");
            assert_eq!(target, &projected);
            Ok(None)
        },
        |target| {
            calls.borrow_mut().push("axis_escape");
            assert_eq!(target, &projected);
            Ok(Some((axis_target.clone(), vec![11])))
        },
        |_target| {
            calls.borrow_mut().push("tight_escape");
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some((axis_target, vec![11])));
    assert_eq!(
        *calls.borrow(),
        vec!["direct", "projected_support", "axis_escape"]
    );
}

#[test]
fn projected_reference_search_skips_duplicate_escape_direct_trace() {
    use std::cell::RefCell;

    let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        std::slice::from_ref(&projected),
        std::slice::from_ref(&projected),
        || {
            calls.borrow_mut().push("projected_support");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("direct");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("axis_escape");
            Ok(None)
        },
        |target| {
            calls.borrow_mut().push("tight_escape");
            Ok(Some((target.clone(), vec![31])))
        },
    )
    .unwrap();

    assert_eq!(found, Some((projected, vec![31])));
    assert_eq!(
        *calls.borrow(),
        vec!["direct", "projected_support", "axis_escape", "tight_escape"]
    );
}

#[test]
fn projected_reference_search_skips_duplicate_escape_direct_trace_for_permuted_definitions() {
    use std::cell::RefCell;

    let point = p(1, 2, 3);
    let definition = axis_defs(&point)[0].clone();
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let projected = ReferenceTarget::with_definitions(point.clone(), vec![definition]);
    let escape_target = ReferenceTarget::with_definitions(point, vec![permuted]);
    let axis_target = ReferenceTarget::axis_defined(p(2, 2, 4));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        std::slice::from_ref(&projected),
        std::slice::from_ref(&escape_target),
        || {
            calls.borrow_mut().push("projected_support");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("direct");
            Ok(None)
        },
        |target| {
            calls.borrow_mut().push("axis_escape");
            assert!(reference_targets_match_for_trace_cache(
                target,
                &escape_target
            ));
            Ok(Some((axis_target.clone(), vec![37])))
        },
        |_target| {
            calls.borrow_mut().push("tight_escape");
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some((axis_target, vec![37])));
    assert_eq!(
        *calls.borrow(),
        vec!["direct", "projected_support", "axis_escape"]
    );
}

#[test]
fn projected_reference_search_skips_duplicate_escape_direct_trace_for_fallback_duplicate() {
    use std::cell::RefCell;

    let point = p(1, 2, 3);
    let projected = ReferenceTarget::axis_defined(point.clone());
    let escape_target = ReferenceTarget::axis_defined_fallback(point);
    let axis_target = ReferenceTarget::axis_defined(p(2, 2, 4));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        std::slice::from_ref(&projected),
        std::slice::from_ref(&escape_target),
        || {
            calls.borrow_mut().push("projected_support");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("direct");
            Ok(None)
        },
        |target| {
            calls.borrow_mut().push("axis_escape");
            assert!(reference_targets_match_for_trace_cache(
                target,
                &escape_target
            ));
            Ok(Some((axis_target.clone(), vec![41])))
        },
        |_target| {
            calls.borrow_mut().push("tight_escape");
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some((axis_target, vec![41])));
    assert_eq!(
        *calls.borrow(),
        vec!["direct", "projected_support", "axis_escape"]
    );
}

#[test]
fn projected_reference_search_still_tries_projected_support_without_targets() {
    use std::cell::RefCell;

    let support_target = ReferenceTarget::axis_defined(p(2, 2, 3));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        &[],
        &[],
        || {
            calls.borrow_mut().push("projected_support");
            Ok(Some((support_target.clone(), vec![13])))
        },
        |_target| {
            calls.borrow_mut().push("direct");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("axis_escape");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("tight_escape");
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some((support_target, vec![13])));
    assert_eq!(*calls.borrow(), vec!["projected_support"]);
}

#[test]
fn projected_reference_search_uses_escape_targets_without_direct_targets() {
    use std::cell::RefCell;

    let escape_target = ReferenceTarget::axis_defined(p(2, 2, 2));
    let axis_target = ReferenceTarget::axis_defined(p(1, 2, 4));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || {
            calls.borrow_mut().push("projected_support");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("direct");
            Ok(None)
        },
        |target| {
            calls.borrow_mut().push("axis_escape");
            assert_eq!(target, &escape_target);
            Ok(Some((axis_target.clone(), vec![17])))
        },
        |_target| {
            calls.borrow_mut().push("tight_escape");
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some((axis_target, vec![17])));
    assert_eq!(
        *calls.borrow(),
        vec!["projected_support", "direct", "axis_escape"]
    );
}

#[test]
fn projected_reference_search_tries_direct_escape_targets_before_axis_escape() {
    use std::cell::RefCell;

    let escape_target = ReferenceTarget::axis_defined(p(2, 2, 2));
    let calls = RefCell::new(Vec::new());

    let found = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || {
            calls.borrow_mut().push("projected_support");
            Ok(None)
        },
        |target| {
            calls.borrow_mut().push("direct");
            assert_eq!(target, &escape_target);
            Ok(Some(vec![23]))
        },
        |_target| {
            calls.borrow_mut().push("axis_escape");
            Ok(None)
        },
        |_target| {
            calls.borrow_mut().push("tight_escape");
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some((escape_target, vec![23])));
    assert_eq!(*calls.borrow(), vec!["projected_support", "direct"]);
}

#[test]
fn projected_reference_search_reports_unknown_if_all_families_are_uncertified() {
    let projected = ReferenceTarget::axis_defined(p(1, 2, 3));
    let err = search_projected_reference_families(
        std::slice::from_ref(&projected),
        std::slice::from_ref(&projected),
        || Err(crate::error::HypermeshError::UnknownClassification),
        |_target| Err(crate::error::HypermeshError::UnknownClassification),
        |_target| Err(crate::error::HypermeshError::UnknownClassification),
        |_target| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projected_reference_search_reports_unknown_when_fallback_target_cannot_trace() {
    let projected = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
    let err = search_projected_reference_families(
        std::slice::from_ref(&projected),
        &[],
        || Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projected_reference_search_certifies_fallback_projected_target_after_trace_succeeds() {
    let fallback = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
    let certified = ReferenceTarget::axis_defined(p(2, 2, 3));

    let found = search_projected_reference_families(
        &[fallback.clone(), certified.clone()],
        &[],
        || Ok(None),
        |target| {
            if target == &fallback {
                Ok(Some(vec![41]))
            } else {
                Ok(Some(vec![43]))
            }
        },
        |_target| Ok(None),
        |_target| Ok(None),
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_certifies_only_fallback_projected_target_after_trace() {
    let fallback = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));

    let found = search_projected_reference_families(
        std::slice::from_ref(&fallback),
        &[],
        || Ok(None),
        |_target| Ok(Some(vec![41])),
        |_target| Ok(None),
        |_target| Ok(None),
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_certifies_fallback_projected_support_success() {
    let escape_target = ReferenceTarget::axis_defined(p(4, 2, 3));
    let found = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || {
            Ok(Some((
                ReferenceTarget::axis_defined_fallback(p(1, 2, 3)),
                vec![41],
            )))
        },
        |_target| Ok(None),
        |_target| Ok(None),
        |_target| Ok(Some((ReferenceTarget::axis_defined(p(5, 2, 3)), vec![43]))),
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_certifies_only_fallback_projected_support_success() {
    let found = search_projected_reference_families(
        &[],
        &[],
        || {
            Ok(Some((
                ReferenceTarget::axis_defined_fallback(p(1, 2, 3)),
                vec![41],
            )))
        },
        |_target| Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_reports_unknown_when_fallback_escape_target_has_no_escape_path() {
    let escape_target = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
    let err = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projected_reference_search_certifies_fallback_axis_escape_success() {
    let escape_target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let found = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || Ok(None),
        |_target| Ok(None),
        |_target| {
            Ok(Some((
                ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                vec![41],
            )))
        },
        |_target| Ok(Some((ReferenceTarget::axis_defined(p(3, 2, 3)), vec![43]))),
    )
    .unwrap();

    assert_certified_reference_result(found, &p(2, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_certifies_only_fallback_axis_escape_success() {
    let escape_target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let found = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || Ok(None),
        |_target| Ok(None),
        |_target| {
            Ok(Some((
                ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                vec![41],
            )))
        },
        |_target| Ok(None),
    )
    .unwrap();

    assert_certified_reference_result(found, &p(2, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_accepts_later_tight_escape_after_fallback_escape_axis_failure() {
    let escape_target = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
    let found = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
        |_target| Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 3)), vec![41]))),
    )
    .unwrap();

    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 2, 3)), vec![41]))
    );
}

#[test]
fn projected_reference_search_certifies_only_fallback_tight_escape_success() {
    let escape_target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let found = search_projected_reference_families(
        &[],
        std::slice::from_ref(&escape_target),
        || Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
        |_target| {
            Ok(Some((
                ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                vec![41],
            )))
        },
    )
    .unwrap();

    assert_certified_reference_result(found, &p(2, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_certifies_first_fallback_tight_escape_success() {
    let first_escape = ReferenceTarget::axis_defined(p(1, 2, 3));
    let second_escape = ReferenceTarget::axis_defined(p(4, 2, 3));
    let found = search_projected_reference_families(
        &[],
        &[first_escape.clone(), second_escape.clone()],
        || Ok(None),
        |_target| Ok(None),
        |_target| Ok(None),
        |target| {
            if target == &first_escape {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(2, 2, 3)),
                    vec![41],
                )))
            } else {
                Ok(Some((ReferenceTarget::axis_defined(p(5, 2, 3)), vec![43])))
            }
        },
    )
    .unwrap();

    assert_certified_reference_result(found, &p(2, 2, 3), &[41]);
}

#[test]
fn projected_reference_search_or_none_skips_uncertified_local_search() {
    let target = ReferenceTarget::axis_defined(p(1, 2, 3));
    assert_eq!(
        projected_reference_search_or_none(Err(
            crate::error::HypermeshError::UnknownClassification
        ))
        .unwrap(),
        None
    );
    assert_eq!(
        projected_reference_search_or_none(Ok(Some((target.clone(), vec![29])))).unwrap(),
        Some((target, vec![29]))
    );
    assert_eq!(
        projected_reference_search_or_none(Err(
            crate::error::HypermeshError::ReferencePropagationFailed
        )),
        Err(crate::error::HypermeshError::ReferencePropagationFailed)
    );
}

#[test]
fn projected_reference_search_or_none_tracking_sets_unknown_flag() {
    let target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let mut saw_unknown = false;

    assert_eq!(
        projected_reference_search_or_none_tracking_unknown(
            Err(crate::error::HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap(),
        None
    );
    assert!(saw_unknown);

    saw_unknown = false;
    assert_eq!(
        projected_reference_search_or_none_tracking_unknown(
            Ok(Some((target.clone(), vec![29]))),
            &mut saw_unknown,
        )
        .unwrap(),
        Some((target, vec![29]))
    );
    assert!(!saw_unknown);
}

#[test]
fn projected_reference_escape_targets_use_certified_projected_cell_family() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = projected_reference_halfspaces(&p(-2, 2, 7), &bounds).unwrap();

    let targets = projected_reference_escape_targets(&bounds, &halfspaces, &[]).unwrap();

    assert!(targets.len() > 1);
    assert!(targets.iter().any(|target| target.point == p(2, 2, 2)));
    assert!(
        targets
            .iter()
            .find(|target| target.point == p(2, 2, 2))
            .is_some_and(|target| target.definitions.as_ref() != &axis_defs(&target.point))
    );
    for target in &targets {
        assert_eq!(axis_ref(&target.point, 1), &r(2));
        assert!(point_satisfies_halfspaces(&target.point, &halfspaces).unwrap());
    }
}

#[test]
fn projected_reference_escape_targets_extend_direct_projected_targets() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = projected_reference_halfspaces(&p(-2, 2, 7), &bounds).unwrap();
    let direct = ReferenceTarget::axis_defined(p(2, 2, 2));

    let targets =
        projected_reference_escape_targets(&bounds, &halfspaces, std::slice::from_ref(&direct))
            .unwrap();

    assert!(targets.iter().any(|target| target.point == direct.point));
    assert!(targets.len() > 1);
}

#[test]
fn projected_reference_escape_targets_include_direct_strict_seed_targets() {
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

    let targets =
        projected_reference_escape_targets_from_report(&bounds, &halfspaces, &[], &report).unwrap();

    assert!(
        targets
            .iter()
            .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
    );
    assert!(
        targets
            .iter()
            .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
            .is_some_and(|target| !target.definitions.is_empty())
    );
}

#[test]
fn reference_target_collection_backtracks_after_uncertified_candidate() {
    let mut targets = Vec::new();

    extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |candidate| {
        if candidate == 0 {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
        }
    })
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(1, 2, 3));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn reference_target_collection_marks_later_targets_uncertain_after_uncertain_candidate_result() {
    let mut targets = Vec::new();

    extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |candidate| {
        if candidate == 0 {
            Ok(vec![ReferenceTarget {
                point: p(1, 2, 3),
                definitions: vec![axis_plane_definition(&p(1, 2, 3))].into(),
                uncertified_definition_fallback: true,
            }])
        } else {
            Ok(vec![ReferenceTarget::axis_defined(p(2, 3, 4))])
        }
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
fn reference_target_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let mut targets = Vec::new();

    let err = extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |_candidate| {
        Err(crate::error::HypermeshError::UnknownClassification)
    })
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_target_collection_keeps_certified_duplicate_state_certified() {
    let mut targets = Vec::new();
    let point = p(1, 2, 3);
    let definition = axis_plane_definition(&point);

    extend_reference_targets_backtracking_unknown(&mut targets, [0, 1], |candidate| {
        if candidate == 0 {
            Ok(vec![ReferenceTarget {
                point: point.clone(),
                definitions: vec![definition.clone()].into(),
                uncertified_definition_fallback: true,
            }])
        } else {
            Ok(vec![ReferenceTarget::with_definitions(
                point.clone(),
                vec![definition.clone()],
            )])
        }
    })
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn reference_target_family_search_backtracks_after_uncertified_earlier_family() {
    let mut targets = Vec::new();

    extend_reference_target_families_backtracking_unknown(
        &mut targets,
        [
            Err(crate::error::HypermeshError::UnknownClassification),
            Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))]),
        ],
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(1, 2, 3));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn reference_target_family_search_tracks_unknown_after_later_certified_family() {
    let mut targets = Vec::new();

    let saw_unknown = extend_reference_target_families_collect_unknown(
        &mut targets,
        [
            Err(crate::error::HypermeshError::UnknownClassification),
            Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))]),
        ],
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets, vec![ReferenceTarget::axis_defined(p(1, 2, 3))]);
}

#[test]
fn reference_target_family_search_tracks_unknown_after_uncertain_family_result() {
    let mut targets = Vec::new();

    let saw_unknown = extend_reference_target_families_collect_unknown(
        &mut targets,
        [
            Ok(vec![ReferenceTarget {
                point: p(1, 2, 3),
                definitions: vec![axis_plane_definition(&p(1, 2, 3))].into(),
                uncertified_definition_fallback: true,
            }]),
            Ok(vec![ReferenceTarget::axis_defined(p(2, 3, 4))]),
        ],
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets.len(), 2);
    assert!(targets[0].uncertified_definition_fallback);
    assert!(!targets[1].uncertified_definition_fallback);
}

#[test]
fn reference_target_family_search_ignores_redundant_fallback_duplicate() {
    let mut targets = Vec::new();
    let point = p(1, 2, 3);
    let definition = axis_plane_definition(&point);

    let saw_unknown = extend_reference_target_families_collect_unknown(
        &mut targets,
        [
            Ok(vec![ReferenceTarget {
                point: point.clone(),
                definitions: vec![definition.clone()].into(),
                uncertified_definition_fallback: true,
            }]),
            Ok(vec![ReferenceTarget::with_definitions(
                point.clone(),
                vec![definition.clone()],
            )]),
        ],
    )
    .unwrap();

    assert!(!saw_unknown);
    assert_eq!(targets.len(), 1);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn reference_target_family_search_marks_later_targets_uncertain_after_uncertain_family_result() {
    let mut targets = Vec::new();

    extend_reference_target_families_backtracking_unknown(
        &mut targets,
        [
            Ok(vec![ReferenceTarget {
                point: p(1, 2, 3),
                definitions: vec![axis_plane_definition(&p(1, 2, 3))].into(),
                uncertified_definition_fallback: true,
            }]),
            Ok(vec![ReferenceTarget::axis_defined(p(2, 3, 4))]),
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
fn reference_target_family_search_reports_unknown_if_all_families_are_uncertified() {
    let mut targets = Vec::new();

    let err = extend_reference_target_families_backtracking_unknown(
        &mut targets,
        [
            Err(crate::error::HypermeshError::UnknownClassification),
            Err(crate::error::HypermeshError::UnknownClassification),
        ],
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_target_family_from_witness_reports_unknown_for_uncertified_witness() {
    let err = reference_target_family_from_witness(
        Some(&p(1, 2, 3)),
        |_candidate| Err(crate::error::HypermeshError::UnknownClassification),
        |_candidate| Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9)))),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_target_family_from_witness_returns_direct_target_when_certified() {
    let targets = reference_target_family_from_witness(
        Some(&p(1, 2, 3)),
        |_candidate| Ok(true),
        |candidate| Ok(Some(ReferenceTarget::axis_defined(candidate.clone()))),
    )
    .unwrap();

    assert_eq!(targets, vec![ReferenceTarget::axis_defined(p(1, 2, 3))]);
}

#[test]
fn reference_target_family_from_witness_reports_unknown_for_boundary_reference_witness() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let err = reference_target_family_from_witness(
        Some(&p(0, 2, 2)),
        |candidate| {
            point_strictly_inside_reference_halfspace_cell_or_unknown(
                candidate,
                &bounds,
                &halfspaces,
            )
        },
        |candidate| Ok(Some(ReferenceTarget::axis_defined(candidate.clone()))),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn strict_projected_target_family_tracking_preserves_empty_unknown_result() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut saw_unknown = false;

    let targets = strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
        &bounds,
        &halfspaces,
        None,
        Vec::new(),
        vec![p(1, 1, 1)],
        Vec::new(),
        &mut saw_unknown,
        |_seed| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap();

    assert!(targets.is_empty());
    assert!(saw_unknown);
}

#[test]
fn projected_escape_target_family_tracking_preserves_unknown_with_existing_targets() {
    let projected_targets = vec![ReferenceTarget::axis_defined(p(0, 0, 0))];
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &[],
        &projected_targets,
        None,
        Vec::new(),
        vec![p(1, 1, 1)],
        Vec::new(),
        &mut saw_unknown,
        |_seed| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets.len(), projected_targets.len());
    assert_eq!(targets[0].point, projected_targets[0].point);
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn projected_escape_target_family_tracking_marks_surviving_targets_uncertain_after_boundary_report_witness()
 {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(0, 2, 2), [None, None, None]);
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &halfspaces,
        &[],
        Some(&report),
        vec![p(1, 1, 1)],
        Vec::new(),
        Vec::new(),
        &mut saw_unknown,
        |seed| Ok(vec![ReferenceTarget::axis_defined(seed.clone())]),
    )
    .unwrap();

    assert!(saw_unknown);
    assert!(targets.iter().any(|target| target.point == p(1, 1, 1)));
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn projected_escape_target_family_tracking_marks_surviving_targets_uncertain_after_fallback_family()
{
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &[],
        &[],
        None,
        Vec::new(),
        vec![first.clone(), second.clone()],
        Vec::new(),
        &mut saw_unknown,
        |seed| {
            Ok(vec![if *seed == first {
                ReferenceTarget::axis_defined_fallback(seed.clone())
            } else {
                ReferenceTarget::axis_defined(seed.clone())
            }])
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets.len(), 2);
    assert!(targets.iter().any(|target| target.point == first));
    assert!(targets.iter().any(|target| target.point == second));
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn projected_escape_target_family_tracking_ignores_redundant_fallback_duplicate() {
    let point = p(1, 2, 3);
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &[],
        &[ReferenceTarget::axis_defined(point.clone())],
        None,
        Vec::new(),
        vec![point.clone()],
        Vec::new(),
        &mut saw_unknown,
        |seed| Ok(vec![ReferenceTarget::axis_defined_fallback(seed.clone())]),
    )
    .unwrap();

    assert!(!saw_unknown);
    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, point);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn projected_escape_target_family_tries_shifted_search_from_report_witness_seed() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 2, 3), [None, None, None]);
    let visited = std::cell::RefCell::new(Vec::new());
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &halfspaces,
        &[],
        Some(&report),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        &mut saw_unknown,
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![p(1, 2, 3)]);
    assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
    assert!(!saw_unknown);
}

#[test]
fn strict_projected_target_family_tracking_marks_surviving_targets_uncertain_after_unknown() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut saw_unknown = false;

    let targets = strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
        &bounds,
        &halfspaces,
        None,
        vec![first.clone(), second.clone()],
        Vec::new(),
        Vec::new(),
        &mut saw_unknown,
        |seed| {
            if *seed == second {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert!(!targets.is_empty());
    assert!(targets.iter().any(|target| target.point == first));
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn strict_projected_target_family_marks_surviving_targets_uncertain_after_unknown() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);

    let targets = strict_projected_cell_targets_from_seed_families_with(
        &bounds,
        &halfspaces,
        None,
        vec![first.clone(), second.clone()],
        Vec::new(),
        Vec::new(),
        |seed| {
            if *seed == second {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Vec::new())
            }
        },
    )
    .unwrap();

    assert!(!targets.is_empty());
    assert!(targets.iter().any(|target| target.point == first));
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn shifted_projected_target_family_marks_surviving_targets_uncertain_after_boundary_report_witness()
{
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let families = ShiftedProjectedCellFamilies {
        shifted: halfspaces.clone(),
        report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            p(0, 2, 2),
            [None, None, None],
        )),
        saw_unknown: false,
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let targets =
        shifted_projected_cell_targets_from_families(&bounds, &halfspaces, &families).unwrap();

    assert!(!targets.is_empty());
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn shifted_projected_target_family_prefers_certified_report_witness_duplicate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 2, 3);
    let families = ShiftedProjectedCellFamilies {
        shifted: halfspaces.clone(),
        report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [Some(9), None, None],
        )),
        saw_unknown: false,
        strict_seeds: Vec::new(),
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let targets =
        shifted_projected_cell_targets_from_families(&bounds, &halfspaces, &families).unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, witness);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn shifted_projected_target_family_marks_surviving_targets_uncertain_after_unknown() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let families = ShiftedProjectedCellFamilies {
        shifted: halfspaces.clone(),
        report: None,
        saw_unknown: true,
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let targets =
        shifted_projected_cell_targets_from_families(&bounds, &halfspaces, &families).unwrap();

    assert!(!targets.is_empty());
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn projected_escape_family_marks_surviving_targets_uncertain_after_unknown() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let families = ShiftedProjectedCellFamilies {
        shifted: halfspaces.clone(),
        report: None,
        saw_unknown: true,
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let targets = projected_escape_targets_from_families(&halfspaces, &families).unwrap();

    assert!(!targets.is_empty());
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn projected_escape_family_prefers_certified_report_witness_duplicate() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let witness = p(1, 2, 3);
    let families = ShiftedProjectedCellFamilies {
        shifted: halfspaces.clone(),
        report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [Some(9), None, None],
        )),
        saw_unknown: false,
        strict_seeds: Vec::new(),
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let targets = projected_escape_targets_from_families(&halfspaces, &families).unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, witness);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn projected_escape_family_marks_surviving_targets_uncertain_after_boundary_report_witness() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let families = ShiftedProjectedCellFamilies {
        shifted: halfspaces.clone(),
        report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            p(0, 2, 2),
            [None, None, None],
        )),
        saw_unknown: false,
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let targets = projected_escape_targets_from_families(&halfspaces, &families).unwrap();

    assert!(!targets.is_empty());
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn deferred_projected_escape_direct_targets_backtrack_after_uncertified_seed() {
    let strict_seeds = vec![p(1, 2, 3), p(1, 2, 4)];
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(4)),
        axis_halfspace(2, false, r(4)),
    ];

    let targets = deferred_projected_escape_direct_targets_with_contains(
        &strict_seeds,
        None,
        &halfspaces,
        |seed, _halfspaces| {
            if seed == &p(1, 2, 3) {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(true)
            }
        },
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(1, 2, 4));
    assert!(!targets[0].definitions.is_empty());
}

#[test]
fn deferred_projected_escape_direct_targets_mark_later_target_uncertain_after_boundary_seed() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();

    let targets =
        deferred_projected_escape_direct_targets(&[p(0, 2, 2), p(1, 2, 2)], None, &halfspaces)
            .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(1, 2, 2));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn deferred_projected_escape_direct_targets_report_unknown_for_boundary_seed() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();

    let err =
        deferred_projected_escape_direct_targets(&[p(0, 2, 2)], None, &halfspaces).unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn deferred_projected_escape_direct_targets_report_unknown_if_all_seeds_are_uncertified() {
    let strict_seeds = vec![p(1, 2, 3), p(1, 2, 4)];
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(4)),
        axis_halfspace(2, false, r(4)),
    ];

    let err = deferred_projected_escape_direct_targets_with_contains(
        &strict_seeds,
        None,
        &halfspaces,
        |_seed, _halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn deferred_direct_reference_targets_backtrack_after_uncertified_seed() {
    let first = p(1, 2, 3);
    let second = p(1, 2, 4);
    let mut saw_unknown = false;

    let targets = deferred_direct_reference_targets_from_strict_seeds_with(
        &[first.clone(), second.clone()],
        None,
        &mut saw_unknown,
        |seed| {
            if *seed == first {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Some(ReferenceTarget::axis_defined(seed.clone())))
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, second);
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn deferred_direct_reference_targets_track_unknown_if_all_seeds_are_uncertified() {
    let first = p(1, 2, 3);
    let second = p(1, 2, 4);
    let mut saw_unknown = false;

    let targets = deferred_direct_reference_targets_from_strict_seeds_with(
        &[first, second],
        None,
        &mut saw_unknown,
        |_seed| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap();

    assert!(targets.is_empty());
    assert!(saw_unknown);
}

#[test]
fn deferred_direct_reference_targets_do_not_mark_unknown_for_fallback_results() {
    let mut saw_unknown = false;

    let targets = deferred_direct_reference_targets_from_strict_seeds_with(
        &[p(1, 2, 3)],
        None,
        &mut saw_unknown,
        |seed| Ok(Some(ReferenceTarget::axis_defined_fallback(seed.clone()))),
    )
    .unwrap();

    assert!(!saw_unknown);
    assert_eq!(targets.len(), 1);
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn strict_projected_target_family_tries_shifted_search_from_report_witness_seed() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 2, 3);
    let visited = std::cell::RefCell::new(Vec::new());
    let mut saw_unknown = false;

    let targets = strict_projected_cell_targets_from_seed_families_with_tracking_unknown(
        &bounds,
        &halfspaces,
        Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [None, None, None],
        )),
        Vec::new(),
        vec![witness.clone()],
        Vec::new(),
        &mut saw_unknown,
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness]);
    assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
}

#[test]
fn strict_support_target_family_tries_shifted_search_from_report_witness_seed() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 2, 3);
    let visited = std::cell::RefCell::new(Vec::new());
    let mut saw_unknown = false;

    let targets = strict_support_cell_targets_from_seed_families_with_tracking_unknown(
        &bounds,
        &halfspaces,
        Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [None, None, None],
        )),
        Vec::new(),
        vec![witness.clone()],
        Vec::new(),
        &mut saw_unknown,
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness]);
    assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
}

#[test]
fn point3_family_search_backtracks_after_uncertified_earlier_family() {
    let mut points = Vec::new();

    extend_point3_families_backtracking_unknown(
        &mut points,
        [
            Err(crate::error::HypermeshError::UnknownClassification),
            Ok(Point3FamilyState {
                points: vec![p(1, 2, 3)],
                saw_unknown: false,
            }),
        ],
    )
    .unwrap();

    assert_eq!(points, vec![p(1, 2, 3)]);
}

#[test]
fn point3_family_search_tracks_unknown_after_uncertain_family_result() {
    let mut points = Vec::new();

    let saw_unknown = extend_point3_families_collect_unknown(
        &mut points,
        [
            Ok(Point3FamilyState {
                points: vec![p(1, 2, 3)],
                saw_unknown: true,
            }),
            Ok(Point3FamilyState {
                points: vec![p(2, 3, 4)],
                saw_unknown: false,
            }),
        ],
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(points, vec![p(1, 2, 3), p(2, 3, 4)]);
}

#[test]
fn point3_family_search_reports_unknown_if_all_families_are_uncertified() {
    let mut points = Vec::new();

    let err = extend_point3_families_backtracking_unknown(
        &mut points,
        [
            Err(crate::error::HypermeshError::UnknownClassification),
            Err(crate::error::HypermeshError::UnknownClassification),
        ],
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn collect_point3_family_tracks_unknown_after_later_strict_point() {
    let candidates = [p(1, 2, 3), p(2, 3, 4)];
    let family = collect_point3_family(&candidates, |candidate| {
        if *candidate == p(1, 2, 3) {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            Ok(true)
        }
    })
    .unwrap();

    assert_eq!(family.points, vec![p(2, 3, 4)]);
    assert!(family.saw_unknown);
}

#[test]
fn collect_point3_family_tracks_unknown_after_reference_boundary_candidate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let candidates = [p(0, 2, 2), p(1, 1, 1)];
    let family = collect_point3_family(&candidates, |candidate| {
        point_strictly_inside_reference_halfspace_cell_or_unknown(candidate, &bounds, &halfspaces)
    })
    .unwrap();

    assert_eq!(family.points, vec![p(1, 1, 1)]);
    assert!(family.saw_unknown);
}

#[test]
fn reference_target_family_or_empty_skips_uncertified_family() {
    let target = ReferenceTarget::axis_defined(p(1, 2, 3));

    assert_eq!(
        reference_target_family_or_empty(Err(crate::error::HypermeshError::UnknownClassification))
            .unwrap(),
        Vec::<ReferenceTarget>::new()
    );
    assert_eq!(
        reference_target_family_or_empty(Ok(vec![target.clone()])).unwrap(),
        vec![target]
    );
    assert_eq!(
        reference_target_family_or_empty(Err(
            crate::error::HypermeshError::ReferencePropagationFailed
        )),
        Err(crate::error::HypermeshError::ReferencePropagationFailed)
    );
}

#[test]
fn reference_target_family_or_empty_tracking_sets_unknown_flag() {
    let target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let mut saw_unknown = false;

    assert_eq!(
        reference_target_family_or_empty_tracking_unknown(
            Err(crate::error::HypermeshError::UnknownClassification),
            &mut saw_unknown,
        )
        .unwrap(),
        Vec::<ReferenceTarget>::new()
    );
    assert!(saw_unknown);

    saw_unknown = false;
    assert_eq!(
        reference_target_family_or_empty_tracking_unknown(
            Ok(vec![target.clone()]),
            &mut saw_unknown
        )
        .unwrap(),
        vec![target]
    );
    assert!(!saw_unknown);
}

#[test]
fn reference_result_or_error_prefers_support_after_uncertified_projected_search() {
    let projected_unknown = true;
    let support_target = ReferenceTarget::axis_defined(p(4, 5, 6));

    let (point, definitions, winding) = reference_result_or_error(
        None,
        Some((support_target.clone(), vec![11])),
        projected_unknown,
    )
    .unwrap();

    assert_eq!(point, support_target.point);
    assert_eq!(&definitions, support_target.definitions.as_ref());
    assert_eq!(winding, vec![11]);
}

#[test]
fn reference_result_or_error_drops_mismatched_target_definitions() {
    let point = p(4, 5, 6);
    let target =
        ReferenceTarget::with_definitions(point.clone(), vec![axis_plane_definition(&p(7, 8, 9))]);

    let result = reference_result_or_error(Some((target, vec![11])), None, false).unwrap();

    assert_eq!(result, (point.clone(), axis_defs(&point), vec![11]));
}

#[test]
fn reference_result_with_support_fallback_skips_support_search_after_projected_hit() {
    let projected_target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let mut support_calls = 0;

    let (point, definitions, winding) = reference_result_with_support_fallback(
        Some((projected_target.clone(), vec![17])),
        false,
        || {
            support_calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(support_calls, 0);
    assert_eq!(point, projected_target.point);
    assert_eq!(&definitions, projected_target.definitions.as_ref());
    assert_eq!(winding, vec![17]);
}

#[test]
fn reference_result_with_support_fallback_uses_support_search_when_projected_missing() {
    let support_target = ReferenceTarget::axis_defined(p(4, 5, 6));
    let mut support_calls = 0;

    let (point, definitions, winding) = reference_result_with_support_fallback(None, true, || {
        support_calls += 1;
        Ok(Some((support_target.clone(), vec![11])))
    })
    .unwrap();

    assert_eq!(support_calls, 1);
    assert_eq!(point, support_target.point);
    assert_eq!(&definitions, support_target.definitions.as_ref());
    assert_eq!(winding, vec![11]);
}

#[test]
fn reference_result_or_error_reports_unknown_after_uncertified_projected_search() {
    let err = reference_result_or_error(None, None, true).unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_result_or_error_reports_reference_failure_when_all_families_are_certified_absent() {
    let err = reference_result_or_error(None, None, false).unwrap_err();

    assert_eq!(
        err,
        crate::error::HypermeshError::ReferencePropagationFailed
    );
}

#[test]
fn certified_leaf_output_helper_runs_leaf_attempt_once() {
    let task = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(1, 1, 1)),
        p(0, 0, 0),
        vec![0],
    );
    let indicator = crate::winding::make_indicator(BooleanOp::Union, 1);
    let mut attempts = 0;

    let output =
        certified_leaf_output_if_complete_with(&task, &indicator, |_task, _indicator, _output| {
            attempts += 1;
            Err(crate::error::HypermeshError::UnknownClassification)
        })
        .unwrap();

    assert_eq!(attempts, 1);
    assert_eq!(output, None);
}

#[test]
fn unsplittable_subdivision_runs_leaf_processor_once() {
    let task = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(0, 0, 0)),
        p(0, 0, 0),
        vec![0],
    );
    let indicator = crate::winding::make_indicator(BooleanOp::Union, 1);
    let mut attempts = 0;
    let mut output = Vec::new();
    let caches = SubdivisionRuntimeCaches::default();

    let err = subdivide_into_inner_with(
        task,
        &indicator,
        SubdivisionConfig { max_depth: 0 },
        None,
        &mut output,
        &mut |_task, _indicator, _output| {
            attempts += 1;
            Err(crate::error::HypermeshError::UnknownClassification)
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    assert_eq!(attempts, 1);
    assert!(output.is_empty());
}

#[test]
fn recursive_child_bounds_contract_unchanged_polygon_family() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let parent_bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let child_bounds = parent_bounds.left_half(0, r(5));

    let tightened = recursive_child_bounds(
        std::slice::from_ref(&polygon),
        std::slice::from_ref(&polygon),
        &child_bounds,
    )
    .unwrap();

    assert_eq!(tightened, Aabb::new(p(0, 0, 0), p(1, 1, 0)));
}

#[test]
fn recursive_child_bounds_contracts_permuted_unchanged_polygon_family() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let parent_bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let child_bounds = parent_bounds.left_half(0, r(5));

    let tightened = recursive_child_bounds(
        &[polygon_a.clone(), polygon_b.clone()],
        &[polygon_b, polygon_a],
        &child_bounds,
    )
    .unwrap();

    assert_eq!(tightened, Aabb::new(p(0, 0, 0), p(1, 1, 1)));
}

#[test]
fn recursive_child_bounds_contract_changed_polygon_family() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let clipped = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 0, 1), 0, 0);
    let parent_bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let child_bounds = parent_bounds.left_half(0, r(5));

    let tightened =
        recursive_child_bounds(&[polygon_a, polygon_b], &[clipped], &child_bounds).unwrap();

    assert_eq!(tightened, Aabb::new(p(0, 0, 0), p(1, 0, 1)));
}

#[test]
fn contract_task_to_polygon_family_bounds_if_tighter_returns_contracted_task() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let polygons = vec![
        axis_face_polygon(&soup.polygons, 0, 5),
        axis_face_polygon(&soup.polygons, 1, 5),
        axis_face_polygon(&soup.polygons, 2, 5),
    ];
    let task = SubdivisionTask::new(
        polygons.clone(),
        Aabb::new(p(0, 0, 0), p(10, 10, 10)),
        p(0, 5, 5),
        vec![0; soup.num_meshes],
    );
    let caches = SubdivisionRuntimeCaches::default();

    let contracted = contract_task_to_polygon_family_bounds_if_tighter(&task, &caches)
        .unwrap()
        .expect("expected tighter polygon-family bounds");

    assert_eq!(contracted.bounds, Aabb::new(p(1, 1, 1), p(9, 9, 9)));
    assert_eq!(contracted.polygons, polygons);
    assert_eq!(contracted.ref_wnv, task.ref_wnv);
    assert_eq!(contracted.depth, task.depth);
}

#[test]
fn contract_task_to_polygon_family_bounds_if_tighter_skips_already_tight_task() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let polygons = vec![
        axis_face_polygon(&soup.polygons, 0, 5),
        axis_face_polygon(&soup.polygons, 1, 5),
        axis_face_polygon(&soup.polygons, 2, 5),
    ];
    let task = SubdivisionTask::new(
        polygons,
        Aabb::new(p(1, 1, 1), p(9, 9, 9)),
        p(5, 5, 5),
        vec![0; soup.num_meshes],
    );
    let caches = SubdivisionRuntimeCaches::default();

    assert_eq!(
        contract_task_to_polygon_family_bounds_if_tighter(&task, &caches).unwrap(),
        None
    );
}

#[test]
fn contract_task_to_polygon_family_bounds_if_tighter_never_expands_task() {
    let task = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(0, 0, 0)),
        p(0, 0, 0),
        vec![0],
    );
    let caches = SubdivisionRuntimeCaches::default();

    assert_eq!(
        contract_task_to_polygon_family_bounds_if_tighter(&task, &caches).unwrap(),
        None
    );
}

#[test]
fn contract_task_to_polygon_family_bounds_if_tighter_reuses_cached_subdivision_reference() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let polygons = vec![
        axis_face_polygon(&soup.polygons, 0, 5),
        axis_face_polygon(&soup.polygons, 1, 5),
        axis_face_polygon(&soup.polygons, 2, 5),
    ];
    let contracted_bounds = Aabb::new(p(1, 1, 1), p(9, 9, 9));
    let cached_ref = Point3::new(q(13, 5), q(21, 5), q(21, 5));
    let task = SubdivisionTask::new(
        polygons.clone(),
        Aabb::new(p(0, 0, 0), p(10, 10, 10)),
        p(0, 5, 5),
        vec![0; soup.num_meshes],
    );
    let caches = SubdivisionRuntimeCaches::default();
    caches
        .child_subdivision
        .borrow_mut()
        .push(ChildSubdivisionCacheEntry {
            polygon_profile: polygon_family_profile(&polygons),
            task: SubdivisionTask::new(
                polygons.clone(),
                contracted_bounds.clone(),
                cached_ref.clone(),
                vec![0; soup.num_meshes],
            ),
            result: Ok(vec![]),
        });

    let contracted = contract_task_to_polygon_family_bounds_if_tighter(&task, &caches)
        .unwrap()
        .expect("expected tighter polygon-family bounds");

    assert_eq!(contracted.bounds, contracted_bounds);
    assert_eq!(contracted.ref_point, cached_ref);
    assert_eq!(contracted.ref_wnv, task.ref_wnv);
    assert!(caches.child_reference.borrow().is_empty());
}

#[test]
fn subdivide_into_inner_with_reuses_cached_contracted_task_result() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let polygons = vec![
        axis_face_polygon(&soup.polygons, 0, 5),
        axis_face_polygon(&soup.polygons, 1, 5),
        axis_face_polygon(&soup.polygons, 2, 5),
    ];
    let task = SubdivisionTask::new(
        polygons.clone(),
        Aabb::new(p(0, 0, 0), p(10, 10, 10)),
        p(0, 5, 5),
        vec![0; soup.num_meshes],
    );
    let contracted_task = SubdivisionTask::new(
        polygons,
        Aabb::new(p(1, 1, 1), p(9, 9, 9)),
        Point3::new(q(13, 5), q(21, 5), q(21, 5)),
        vec![0; soup.num_meshes],
    );
    let cached_output = vec![ClassifiedPolygon::new(
        make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 1),
        1,
    )];
    let indicator = crate::winding::make_indicator(BooleanOp::Union, 1);
    let caches = SubdivisionRuntimeCaches::default();
    caches
        .child_subdivision
        .borrow_mut()
        .push(ChildSubdivisionCacheEntry {
            polygon_profile: polygon_family_profile(&contracted_task.polygons),
            task: contracted_task,
            result: Ok(cached_output.clone()),
        });
    let mut process_leaf_calls = 0;
    let mut output = Vec::new();

    subdivide_into_inner_with(
        task,
        &indicator,
        SubdivisionConfig { max_depth: 0 },
        None,
        &mut output,
        &mut |_task, _indicator, _output| {
            process_leaf_calls += 1;
            Err(crate::error::HypermeshError::UnknownClassification)
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap();

    assert_eq!(process_leaf_calls, 0);
    assert_eq!(output, cached_output);
}

#[test]
fn ordered_split_attempt_children_prefers_changed_family_before_unchanged_parent_copy() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let parent = vec![polygon_a.clone(), polygon_b.clone()];
    let children = ordered_split_attempt_children(
        &parent,
        parent.clone(),
        Some(Aabb::new(p(0, 0, 0), p(1, 1, 1))),
        vec![polygon_a.clone()],
        Some(Aabb::new(p(0, 0, 0), p(1, 1, 0))),
    );

    assert_eq!(children.len(), 2);
    assert_eq!(children[0].polygons, vec![polygon_a]);
    assert!(!children[0].unchanged_from_parent);
    assert_eq!(children[1].polygons, parent);
    assert!(children[1].unchanged_from_parent);
}

#[test]
fn ordered_split_attempt_children_prefers_smaller_changed_child_family() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let polygon_c = make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 2, 0);
    let parent = vec![polygon_a.clone(), polygon_b.clone(), polygon_c.clone()];
    let children = ordered_split_attempt_children(
        &parent,
        vec![polygon_a.clone(), polygon_b.clone()],
        Some(Aabb::new(p(0, 0, 0), p(1, 1, 1))),
        vec![polygon_c.clone()],
        Some(Aabb::new(p(0, 0, 2), p(1, 1, 2))),
    );

    assert_eq!(children.len(), 2);
    assert_eq!(children[0].polygons, vec![polygon_c]);
    assert_eq!(children[1].polygons, vec![polygon_a, polygon_b]);
    assert!(!children[0].unchanged_from_parent);
    assert!(!children[1].unchanged_from_parent);
}

#[test]
fn ordered_leaf_polygon_indices_prefers_intersecting_hosts_before_direct_hosts() {
    let intersections = vec![
        vec![sample_segment_intersection(1)],
        vec![],
        vec![
            sample_segment_intersection(2),
            sample_segment_intersection(3),
        ],
    ];

    let indices = ordered_leaf_polygon_indices_by_intersections(&intersections);

    assert_eq!(indices, vec![2, 0, 1]);
}

#[test]
fn ordered_leaf_polygon_indices_keeps_original_order_for_equal_intersection_counts() {
    let intersections = vec![
        vec![sample_segment_intersection(1)],
        vec![sample_segment_intersection(2)],
        vec![],
    ];

    let indices = ordered_leaf_polygon_indices_by_intersections(&intersections);

    assert_eq!(indices, vec![0, 1, 2]);
}

#[test]
fn ordered_bsp_leaf_indices_prefers_larger_edge_cycles_first() {
    let leaves = vec![
        BspLeaf {
            edges: vec![
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::axis_aligned(2, r(0)),
            ],
            enabled: true,
        },
        BspLeaf {
            edges: vec![
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::axis_aligned(2, r(0)),
                Plane::axis_aligned(0, r(1)),
            ],
            enabled: true,
        },
        BspLeaf {
            edges: vec![
                Plane::axis_aligned(0, r(0)),
                Plane::axis_aligned(1, r(0)),
                Plane::axis_aligned(2, r(0)),
            ],
            enabled: true,
        },
    ];

    let indices = ordered_bsp_leaf_indices_by_complexity(&leaves);

    assert_eq!(indices, vec![1, 0, 2]);
}

#[test]
fn split_attempt_recursive_room_key_prefers_lower_dimensional_children() {
    let line_bounds = Aabb::new(p(0, 0, 0), p(1, 0, 0));
    let slab_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let volume_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));

    let flatter = RankedSplitAttempt {
        axis: 0,
        value: r(1),
        counts: (4, 0, 6, 0, 1, 0),
        source: SplitSource::Arrangement,
        left_polys: Vec::new(),
        left_bounds: Some(line_bounds),
        right_polys: Vec::new(),
        right_bounds: Some(slab_bounds.clone()),
    };
    let deeper = RankedSplitAttempt {
        axis: 1,
        value: r(2),
        counts: (4, 0, 6, 0, 1, 0),
        source: SplitSource::Arrangement,
        left_polys: Vec::new(),
        left_bounds: Some(slab_bounds),
        right_polys: Vec::new(),
        right_bounds: Some(volume_bounds),
    };

    assert!(split_attempt_recursive_room_key(&flatter) < split_attempt_recursive_room_key(&deeper));
}

#[test]
fn split_attempt_fanout_order_keeps_child_load_ahead_of_downstream_fanout() {
    let lower_child_load = RankedSplitAttempt {
        axis: 0,
        value: r(2),
        counts: (12, 0, 24, 0, 0, 0),
        source: SplitSource::Arrangement,
        left_polys: Vec::new(),
        left_bounds: None,
        right_polys: Vec::new(),
        right_bounds: None,
    };
    let lower_fanout = RankedSplitAttempt {
        axis: 1,
        value: r(4),
        counts: (14, 0, 24, 0, 0, 4),
        ..lower_child_load.clone()
    };

    assert!(
        split_attempt_fanout_order_key(&lower_child_load, (9, 18, 0))
            < split_attempt_fanout_order_key(&lower_fanout, (1, 2, 0))
    );

    let same_child_load = RankedSplitAttempt {
        axis: 2,
        value: r(3),
        ..lower_child_load.clone()
    };
    assert!(
        split_attempt_fanout_order_key(&same_child_load, (1, 2, 0))
            < split_attempt_fanout_order_key(&lower_child_load, (9, 18, 0))
    );
}

#[test]
fn split_attempt_strict_reduction_requires_every_child_family_to_shrink() {
    let reducing = RankedSplitAttempt {
        axis: 0,
        value: r(1),
        counts: (4, 0, 7, 0, 1, 1),
        source: SplitSource::Arrangement,
        left_polys: Vec::new(),
        left_bounds: None,
        right_polys: Vec::new(),
        right_bounds: None,
    };
    let retaining = RankedSplitAttempt {
        counts: (5, 0, 8, 0, 2, 2),
        ..reducing.clone()
    };

    assert!(split_attempt_strictly_reduces_polygon_family(&reducing, 5));
    assert!(!split_attempt_strictly_reduces_polygon_family(
        &retaining, 5
    ));
}

#[test]
fn preferred_split_partition_preserves_deferred_rank_order() {
    let attempt = |axis, max_child_count| RankedSplitAttempt {
        axis,
        value: r(axis as i32 + 1),
        counts: (max_child_count, 0, max_child_count + 2, 0, 1, 1),
        source: SplitSource::Arrangement,
        left_polys: Vec::new(),
        left_bounds: None,
        right_polys: Vec::new(),
        right_bounds: None,
    };
    let first = attempt(0, 5);
    let preferred = attempt(1, 4);
    let last = attempt(2, 3);

    let (selected, deferred) = partition_preferred_subdivision_split(
        vec![first.clone(), preferred.clone(), last.clone()],
        5,
    );

    assert_eq!(selected, Some(preferred));
    assert_eq!(deferred, vec![first, last]);
}

#[test]
fn split_child_matches_parent_geometry_requires_same_bounds_and_family() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let parent = vec![polygon_a.clone(), polygon_b.clone()];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    assert!(split_child_matches_parent_geometry(
        &parent,
        &bounds,
        &[polygon_b, polygon_a],
        &bounds,
    ));
}

#[test]
fn split_child_matches_parent_geometry_rejects_tighter_bounds() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let parent = vec![polygon_a.clone(), polygon_b.clone()];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let tighter_bounds = Aabb::new(p(0, 0, 0), p(2, 4, 4));

    assert!(!split_child_matches_parent_geometry(
        &parent,
        &bounds,
        &[polygon_b, polygon_a],
        &tighter_bounds,
    ));
}

#[test]
fn cached_polygon_family_bounds_reuses_permuted_polygon_families() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_polygon_family_bounds_with(
        &cache,
        &[polygon_a.clone(), polygon_b.clone()],
        |_polygons| {
            calls.set(calls.get() + 1);
            Ok(Aabb::new(p(0, 0, 0), p(1, 1, 1)))
        },
    )
    .unwrap();
    let second = cached_polygon_family_bounds_with(&cache, &[polygon_b, polygon_a], |_polygons| {
        calls.set(calls.get() + 1);
        Ok(Aabb::new(p(0, 0, 0), p(9, 9, 9)))
    })
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn cached_polygon_family_bounds_memoizes_current_equivalent_state() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let cache = RefCell::new(Vec::new());

    cached_polygon_family_bounds_with(&cache, &[polygon_a.clone(), polygon_b.clone()], |_p| {
        Ok(Aabb::new(p(0, 0, 0), p(1, 1, 1)))
    })
    .unwrap();
    cached_polygon_family_bounds_with(&cache, &[polygon_b, polygon_a], |_p| {
        Ok(Aabb::new(p(0, 0, 0), p(9, 9, 9)))
    })
    .unwrap();

    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn cached_pairwise_intersections_reuse_identical_polygon_sequence() {
    let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
    let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
    let polygons = vec![horizontal, vertical];
    let cache = RefCell::new(Vec::new());

    let first = cached_pairwise_intersections_by_polygon_with(&cache, &polygons).unwrap();
    let second = cached_pairwise_intersections_by_polygon_with(&cache, &polygons).unwrap();

    assert_eq!(first, second);
    assert!(Arc::ptr_eq(&first, &second));
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn cached_pairwise_intersections_reuse_permuted_polygon_sequence() {
    let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
    let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
    let first_polygons = vec![horizontal.clone(), vertical.clone()];
    let second_polygons = vec![vertical, horizontal];
    let cache = RefCell::new(Vec::new());

    let first = cached_pairwise_intersections_by_polygon_with(&cache, &first_polygons).unwrap();
    let second = cached_pairwise_intersections_by_polygon_with(&cache, &second_polygons).unwrap();
    let direct = pairwise_intersections_by_polygon(&second_polygons).unwrap();

    assert_eq!(first.len(), 2);
    assert_eq!(second.as_ref(), &direct);
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn cached_pairwise_intersections_memoize_current_equivalent_state() {
    let horizontal = make_triangle(&p(2, 1, 0), &p(8, 1, 0), &p(5, 1, 4), 0, 0);
    let vertical = make_triangle(&p(5, 0, 1), &p(5, 4, 1), &p(5, 2, 4), 1, 0);
    let first_polygons = vec![horizontal.clone(), vertical.clone()];
    let second_polygons = vec![vertical, horizontal];
    let cache = RefCell::new(Vec::new());

    cached_pairwise_intersections_by_polygon_with(&cache, &first_polygons).unwrap();
    cached_pairwise_intersections_by_polygon_with(&cache, &second_polygons).unwrap();

    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn cached_reference_halfspace_containment_reuses_permuted_halfspaces() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let point = p(2, 2, 2);
    let left = vec![
        axis_halfspace(0, false, r(0)),
        axis_halfspace(1, false, r(0)),
    ];
    let right = vec![
        axis_halfspace(1, false, r(0)),
        axis_halfspace(0, false, r(0)),
    ];
    let cache = RefCell::new(Vec::<ReferenceHalfspaceContainmentCacheEntry>::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_reference_halfspace_containment_with(
        &mut cache.borrow_mut(),
        &bounds,
        &point,
        &left,
        |_point, _bounds, _halfspaces| {
            calls.set(calls.get() + 1);
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_reference_halfspace_containment_with(
        &mut cache.borrow_mut(),
        &bounds,
        &point,
        &right,
        |_point, _bounds, _halfspaces| {
            calls.set(calls.get() + 1);
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(calls.get(), 1);
}

#[test]
fn cached_pure_halfspace_containment_reuses_permuted_halfspaces() {
    let point = p(2, 2, 2);
    let left = vec![
        axis_halfspace(0, false, r(0)),
        axis_halfspace(1, false, r(0)),
    ];
    let right = vec![
        axis_halfspace(1, false, r(0)),
        axis_halfspace(0, false, r(0)),
    ];
    let cache = RefCell::new(Vec::<ReferencePureHalfspaceContainmentCacheEntry>::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_pure_halfspace_containment_with(
        &mut cache.borrow_mut(),
        &point,
        &left,
        |_point, _halfspaces| {
            calls.set(calls.get() + 1);
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_pure_halfspace_containment_with(
        &mut cache.borrow_mut(),
        &point,
        &right,
        |_point, _halfspaces| {
            calls.set(calls.get() + 1);
            Ok(false)
        },
    )
    .unwrap();

    assert!(first);
    assert!(second);
    assert_eq!(calls.get(), 1);
}

#[test]
fn subdivision_child_partition_dedupe_skips_duplicate_contracted_unchanged_branch() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let parent_bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let left_x = recursive_child_bounds(
        std::slice::from_ref(&polygon),
        std::slice::from_ref(&polygon),
        &parent_bounds.left_half(0, r(5)),
    )
    .unwrap();
    let left_y = recursive_child_bounds(
        std::slice::from_ref(&polygon),
        std::slice::from_ref(&polygon),
        &parent_bounds.left_half(1, r(5)),
    )
    .unwrap();
    let mut seen = Vec::new();

    assert!(take_new_subdivision_child_partition(
        &mut seen,
        std::slice::from_ref(&polygon),
        Some(&left_x),
        &[],
        None,
    ));
    assert!(!take_new_subdivision_child_partition(
        &mut seen,
        std::slice::from_ref(&polygon),
        Some(&left_y),
        &[],
        None,
    ));
}

#[test]
fn subdivision_child_partition_dedupe_keeps_distinct_nonempty_bounds() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let mut seen = Vec::new();
    let left_a = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let left_b = Aabb::new(p(0, 0, 0), p(2, 1, 0));

    assert!(take_new_subdivision_child_partition(
        &mut seen,
        std::slice::from_ref(&polygon),
        Some(&left_a),
        &[],
        None,
    ));
    assert!(take_new_subdivision_child_partition(
        &mut seen,
        std::slice::from_ref(&polygon),
        Some(&left_b),
        &[],
        None,
    ));
}

#[test]
fn cached_child_reference_reuses_identical_child_state() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);
    let old_ref = p(0, 0, 0);
    let old_ref_definitions = axis_defs(&old_ref);
    let old_wnv = vec![0];

    let first = cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
        },
    )
    .unwrap();
    let second = cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(9, 9, 9), axis_defs(&p(9, 9, 9)), vec![99]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
}

#[test]
fn cached_child_reference_stores_only_certified_result_definitions() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let cache = RefCell::new(Vec::new());
    let old_ref = p(0, 0, 0);
    let point = p(1, 2, 3);

    let result = cached_child_reference_with(
        &cache,
        &old_ref,
        &axis_defs(&old_ref),
        &[0],
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            Ok((
                point.clone(),
                vec![axis_plane_definition(&p(9, 9, 9))],
                vec![7],
            ))
        },
    )
    .unwrap();

    assert_eq!(result, (point.clone(), axis_defs(&point), vec![7]));
    assert_eq!(cache.borrow()[0].result, Ok(result));
}

#[test]
fn cached_child_reference_reuses_permuted_parent_definition_families() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);
    let old_ref = p(1, 2, 3);
    let definition = axis_defs(&old_ref)[0].clone();
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let old_wnv = vec![0];

    let first = cached_child_reference_with(
        &cache,
        &old_ref,
        std::slice::from_ref(&definition),
        &old_wnv,
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![9]))
        },
    )
    .unwrap();
    let second = cached_child_reference_with(
        &cache,
        &old_ref,
        std::slice::from_ref(&permuted),
        &old_wnv,
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(7, 8, 9), axis_defs(&p(7, 8, 9)), vec![11]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
}

#[test]
fn cached_child_reference_memoizes_current_equivalent_state() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let cache = RefCell::new(Vec::new());
    let old_ref = p(0, 0, 0);
    let old_ref_definitions = axis_defs(&old_ref);
    let old_wnv = vec![0];

    cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        &[polygon_a.clone(), polygon_b.clone()],
        &bounds,
        || Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7])),
    )
    .unwrap();
    cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        &[polygon_b, polygon_a],
        &bounds,
        || Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8])),
    )
    .unwrap();

    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn cached_child_reference_prefers_newest_exact_alias_state() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let old_ref = p(0, 0, 0);
    let old_ref_definitions = axis_defs(&old_ref);
    let old_wnv = vec![0];
    let cache = RefCell::new(vec![
        ChildReferenceCacheEntry {
            source_polygon_profile: polygon_family_profile(&[polygon_a.clone(), polygon_b.clone()]),
            source_polygons: vec![polygon_a.clone(), polygon_b.clone()],
            bounds: bounds.clone(),
            old_ref: old_ref.clone(),
            old_ref_definitions: old_ref_definitions.clone(),
            old_wnv: old_wnv.clone(),
            result: Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7])),
        },
        ChildReferenceCacheEntry {
            source_polygon_profile: polygon_family_profile(&[polygon_b.clone(), polygon_a.clone()]),
            source_polygons: vec![polygon_b.clone(), polygon_a.clone()],
            bounds: bounds.clone(),
            old_ref: old_ref.clone(),
            old_ref_definitions: old_ref_definitions.clone(),
            old_wnv: old_wnv.clone(),
            result: Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![9])),
        },
    ]);

    let result = cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        &[polygon_b, polygon_a],
        &bounds,
        || unreachable!(),
    )
    .unwrap();

    assert_eq!(result, (p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![9]));
}

#[test]
fn cached_child_reference_keeps_distinct_child_bounds_separate() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let bounds_a = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let bounds_b = Aabb::new(p(0, 0, 0), p(2, 1, 0));
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);
    let old_ref = p(0, 0, 0);
    let old_ref_definitions = axis_defs(&old_ref);
    let old_wnv = vec![0];

    cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        std::slice::from_ref(&polygon),
        &bounds_a,
        || {
            calls.set(calls.get() + 1);
            Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
        },
    )
    .unwrap();
    cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        std::slice::from_ref(&polygon),
        &bounds_b,
        || {
            calls.set(calls.get() + 1);
            Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 2);
}

#[test]
fn reusable_child_reference_if_certified_reuses_parent_reference() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let task = SubdivisionTask::new(
        vec![polygon.clone()],
        Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        p(1, 1, 1),
        vec![0],
    );
    let child_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
    let cache = RefCell::new(Vec::new());
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_if_certified(
        &cache,
        &task,
        std::slice::from_ref(&polygon),
        &child_bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((
            task.ref_point.clone(),
            task.ref_definitions.clone(),
            task.ref_wnv.clone(),
        ))
    );
    assert_eq!(query_caches.validity_cache.len(), 1);
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn reusable_child_reference_if_certified_reuses_changed_child_family_when_point_stays_valid() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let other = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 1, 0);
    let task = SubdivisionTask::new(
        vec![polygon],
        Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        p(1, 1, 1),
        vec![0],
    );
    let child_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
    let cache = RefCell::new(Vec::new());
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_if_certified(
        &cache,
        &task,
        std::slice::from_ref(&other),
        &child_bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((
            task.ref_point.clone(),
            task.ref_definitions.clone(),
            task.ref_wnv.clone(),
        ))
    );
    assert_eq!(query_caches.validity_cache.len(), 1);
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn reusable_child_reference_if_certified_skips_invalid_point() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let task = SubdivisionTask::new(
        vec![polygon.clone()],
        Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        p(0, 0, 0),
        vec![0],
    );
    let child_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
    let cache = RefCell::new(Vec::new());
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_if_certified(
        &cache,
        &task,
        std::slice::from_ref(&polygon),
        &child_bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(query_caches.validity_cache.len(), 1);
    assert!(cache.borrow().is_empty());
}

#[test]
fn reusable_child_reference_if_certified_memoizes_current_state() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let task = SubdivisionTask::new(
        vec![polygon.clone()],
        Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        p(1, 1, 1),
        vec![0],
    );
    let child_bounds = Aabb::new(p(0, 0, 0), p(2, 2, 2));
    let cache = RefCell::new(Vec::new());
    let mut query_caches = SupportReferenceQueryCaches::default();
    let calls = std::cell::Cell::new(0);

    let reused = reusable_child_reference_if_certified(
        &cache,
        &task,
        std::slice::from_ref(&polygon),
        &child_bounds,
        &mut query_caches,
    )
    .unwrap()
    .unwrap();

    let cached = cached_child_reference_with(
        &cache,
        &task.ref_point,
        &task.ref_definitions,
        &task.ref_wnv,
        std::slice::from_ref(&polygon),
        &child_bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(9, 9, 9), axis_defs(&p(9, 9, 9)), vec![9]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 0);
    assert_eq!(cached, reused);
}

#[test]
fn propagate_child_reference_prefers_direct_result_before_equivalent_cached_reuse() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let root_task = contract_task_to_polygon_family_bounds_if_tighter(
        &SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        ),
        &SubdivisionRuntimeCaches::default(),
    )
    .unwrap()
    .unwrap();
    let caches = SubdivisionRuntimeCaches::default();
    let root_attempt = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &root_task.bounds,
        &root_task.polygons,
    )
    .unwrap()
    .into_iter()
    .next()
    .unwrap();
    let child = ordered_split_attempt_children(
        &root_task.polygons,
        root_attempt.left_polys,
        root_attempt.left_bounds,
        root_attempt.right_polys,
        root_attempt.right_bounds,
    )
    .into_iter()
    .nth(1)
    .unwrap();
    let source_polygons = ordered_reference_search_polygons(&root_task.polygons, &child.bounds);
    let direct = compute_new_reference_with_query_caches(
        &root_task.ref_point,
        &root_task.ref_definitions,
        &root_task.ref_wnv,
        &child.bounds,
        &source_polygons,
        &mut SupportReferenceQueryCaches::default(),
    )
    .unwrap();
    let mut permuted_source_polygons = source_polygons.clone();
    permuted_source_polygons.reverse();
    caches
        .child_reference
        .borrow_mut()
        .push(ChildReferenceCacheEntry {
            source_polygon_profile: polygon_family_profile(&permuted_source_polygons),
            source_polygons: permuted_source_polygons,
            bounds: child.bounds.clone(),
            old_ref: root_task.ref_point.clone(),
            old_ref_definitions: root_task.ref_definitions.clone(),
            old_wnv: root_task.ref_wnv.clone(),
            result: Ok((
                direct.0.clone(),
                direct.1.clone(),
                vec![9; direct.2.len().max(1)],
            )),
        });

    let propagated =
        propagate_child_reference(&root_task, &child.polygons, &child.bounds, &caches).unwrap();

    assert_eq!(propagated, direct);
}

#[test]
fn propagate_child_reference_prefers_direct_result_before_exact_cached_hit() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let root_task = contract_task_to_polygon_family_bounds_if_tighter(
        &SubdivisionTask::new(
            soup.polygons.clone(),
            Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        ),
        &SubdivisionRuntimeCaches::default(),
    )
    .unwrap()
    .unwrap();
    let caches = SubdivisionRuntimeCaches::default();
    let root_attempt = cached_ordered_subdivision_splits_with(
        &caches.polygon_axis_values,
        &caches.split_candidates,
        &caches.split_child_fanout_counts,
        &caches.split_child_partitions,
        &caches.polygon_family_bounds,
        &caches.pairwise_intersections,
        &root_task.bounds,
        &root_task.polygons,
    )
    .unwrap()
    .into_iter()
    .next()
    .unwrap();
    let child = ordered_split_attempt_children(
        &root_task.polygons,
        root_attempt.left_polys,
        root_attempt.left_bounds,
        root_attempt.right_polys,
        root_attempt.right_bounds,
    )
    .into_iter()
    .nth(1)
    .unwrap();
    let source_polygons = ordered_reference_search_polygons(&root_task.polygons, &child.bounds);
    let direct = compute_new_reference_with_query_caches(
        &root_task.ref_point,
        &root_task.ref_definitions,
        &root_task.ref_wnv,
        &child.bounds,
        &source_polygons,
        &mut SupportReferenceQueryCaches::default(),
    )
    .unwrap();
    caches
        .child_reference
        .borrow_mut()
        .push(ChildReferenceCacheEntry {
            source_polygon_profile: polygon_family_profile(&source_polygons),
            source_polygons: source_polygons.clone(),
            bounds: child.bounds.clone(),
            old_ref: root_task.ref_point.clone(),
            old_ref_definitions: root_task.ref_definitions.clone(),
            old_wnv: root_task.ref_wnv.clone(),
            result: Ok((
                direct.0.clone(),
                direct.1.clone(),
                vec![9; direct.2.len().max(1)],
            )),
        });

    let propagated =
        propagate_child_reference(&root_task, &child.polygons, &child.bounds, &caches).unwrap();

    assert_eq!(propagated, direct);
}

#[test]
fn reusable_child_reference_from_cached_trace_if_certified_reuses_cached_target() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_point = p(2, 1, 1);
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: bounds.clone(),
        old_ref: p(1, 1, 1),
        old_ref_definitions: axis_defs(&p(1, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_from_cached_trace_if_certified(
        &cache,
        &cached_point,
        &axis_defs(&cached_point),
        &[0],
        std::slice::from_ref(&polygon),
        &bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((cached_point.clone(), axis_defs(&cached_point), vec![0]))
    );
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_reference_from_cached_trace_if_certified_reuses_cached_target_across_tighter_bounds()
 {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let cached_point = p(2, 1, 1);
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: cached_bounds,
        old_ref: p(1, 1, 1),
        old_ref_definitions: axis_defs(&p(1, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_from_cached_trace_if_certified(
        &cache,
        &cached_point,
        &axis_defs(&cached_point),
        &[0],
        std::slice::from_ref(&polygon),
        &query_bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((cached_point.clone(), axis_defs(&cached_point), vec![0]))
    );
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_reference_from_cached_trace_reuses_cached_target_across_parent_winding() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_point = p(2, 1, 1);
    let query_wnv = vec![7];
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: bounds.clone(),
        old_ref: p(1, 1, 1),
        old_ref_definitions: axis_defs(&p(1, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_from_cached_trace_if_certified(
        &cache,
        &cached_point,
        &axis_defs(&cached_point),
        &query_wnv,
        std::slice::from_ref(&polygon),
        &bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((
            cached_point.clone(),
            axis_defs(&cached_point),
            query_wnv.clone()
        ))
    );
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_reference_from_cached_result_if_certified_reuses_cached_target() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_point = p(2, 1, 1);
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: bounds.clone(),
        old_ref: p(1, 1, 1),
        old_ref_definitions: axis_defs(&p(1, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_from_cached_result_if_certified(
        &cache,
        &cached_point,
        &axis_defs(&cached_point),
        &[0],
        std::slice::from_ref(&polygon),
        &bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((cached_point.clone(), axis_defs(&cached_point), vec![0]))
    );
    assert!(query_caches.trace_cache.is_empty());
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_reference_from_cached_result_if_certified_reuses_cached_target_across_tighter_bounds()
 {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let cached_point = p(2, 1, 1);
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: cached_bounds,
        old_ref: p(1, 1, 1),
        old_ref_definitions: axis_defs(&p(1, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_from_cached_result_if_certified(
        &cache,
        &cached_point,
        &axis_defs(&cached_point),
        &[0],
        std::slice::from_ref(&polygon),
        &query_bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((cached_point.clone(), axis_defs(&cached_point), vec![0]))
    );
    assert!(query_caches.trace_cache.is_empty());
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_reference_from_cached_result_if_certified_skips_invalid_cached_target() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_point = p(0, 0, 0);
    let query_point = p(1, 1, 1);
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: bounds.clone(),
        old_ref: p(2, 1, 1),
        old_ref_definitions: axis_defs(&p(2, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_from_cached_result_if_certified(
        &cache,
        &query_point,
        &axis_defs(&query_point),
        &[0],
        std::slice::from_ref(&polygon),
        &bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert!(query_caches.trace_cache.is_empty());
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn reusable_child_reference_from_cached_trace_if_certified_skips_invalid_cached_target() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_point = p(0, 0, 0);
    let query_point = p(1, 1, 1);
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: bounds.clone(),
        old_ref: p(2, 1, 1),
        old_ref_definitions: axis_defs(&p(2, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_reference_from_cached_trace_if_certified(
        &cache,
        &query_point,
        &axis_defs(&query_point),
        &[0],
        std::slice::from_ref(&polygon),
        &bounds,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(reused, None);
}

#[test]
fn reusable_child_reference_from_cached_trace_if_certified_memoizes_current_state() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_point = p(2, 1, 1);
    let query_point = cached_point.clone();
    let cache = RefCell::new(vec![ChildReferenceCacheEntry {
        source_polygon_profile: polygon_family_profile(std::slice::from_ref(&polygon)),
        source_polygons: vec![polygon.clone()],
        bounds: bounds.clone(),
        old_ref: p(1, 1, 1),
        old_ref_definitions: axis_defs(&p(1, 1, 1)),
        old_wnv: vec![0],
        result: Ok((cached_point.clone(), axis_defs(&cached_point), vec![0])),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();
    let calls = std::cell::Cell::new(0);

    let reused = reusable_child_reference_from_cached_trace_if_certified(
        &cache,
        &query_point,
        &axis_defs(&query_point),
        &[0],
        std::slice::from_ref(&polygon),
        &bounds,
        &mut query_caches,
    )
    .unwrap()
    .unwrap();

    let cached = cached_child_reference_with(
        &cache,
        &query_point,
        &axis_defs(&query_point),
        &[0],
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(9, 9, 9), axis_defs(&p(9, 9, 9)), vec![9]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 0);
    assert_eq!(cached, reused);
}

#[test]
fn process_split_attempt_child_backtracks_on_identical_recursive_state() {
    let polygon = make_triangle(&p(1, 1, 0), &p(3, 1, 0), &p(1, 3, 0), 0, 0);
    let task = SubdivisionTask::new(
        vec![polygon.clone()],
        Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        p(2, 2, 2),
        vec![0],
    );
    let indicator = make_indicator(BooleanOp::Union, 1);
    let caches = SubdivisionRuntimeCaches::default();
    let mut candidate_output = Vec::new();
    let mut candidate_buckets = ClassifiedPolygonBucketState::new();

    let err = process_split_attempt_child(
        &task,
        vec![polygon],
        task.bounds.clone(),
        &indicator,
        SubdivisionConfig { max_depth: 4 },
        Some(BooleanOp::Union),
        &mut candidate_output,
        &mut candidate_buckets,
        &mut |_task, _indicator, _output| {
            panic!("identical recursive child state should backtrack before leaf processing")
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap_err();

    assert_eq!(
        err,
        crate::error::HypermeshError::ReferencePropagationFailed
    );
    assert!(candidate_output.is_empty());
}

#[test]
fn subdivision_child_partition_dedupe_skips_permuted_polygon_order() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let left_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let mut seen = Vec::new();

    assert!(take_new_subdivision_child_partition(
        &mut seen,
        &[polygon_a.clone(), polygon_b.clone()],
        Some(&left_bounds),
        &[],
        None,
    ));
    assert!(!take_new_subdivision_child_partition(
        &mut seen,
        &[polygon_b, polygon_a],
        Some(&left_bounds),
        &[],
        None,
    ));
}

#[test]
fn subdivision_child_partition_dedupe_skips_swapped_equivalent_children() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let left_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let right_bounds = Aabb::new(p(0, 0, 1), p(1, 1, 1));
    let mut seen = Vec::new();

    assert!(take_new_subdivision_child_partition(
        &mut seen,
        std::slice::from_ref(&polygon_a),
        Some(&left_bounds),
        std::slice::from_ref(&polygon_b),
        Some(&right_bounds),
    ));
    assert!(!take_new_subdivision_child_partition(
        &mut seen,
        std::slice::from_ref(&polygon_b),
        Some(&right_bounds),
        std::slice::from_ref(&polygon_a),
        Some(&left_bounds),
    ));
}

#[test]
fn cached_child_reference_keeps_distinct_parent_reference_states_separate() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);
    let old_ref_a = p(0, 0, 0);
    let old_ref_b = p(9, 9, 9);
    let old_ref_definitions_a = axis_defs(&old_ref_a);
    let old_ref_definitions_b = axis_defs(&old_ref_b);
    let old_wnv_a = vec![0];
    let old_wnv_b = vec![1];

    cached_child_reference_with(
        &cache,
        &old_ref_a,
        &old_ref_definitions_a,
        &old_wnv_a,
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
        },
    )
    .unwrap();
    cached_child_reference_with(
        &cache,
        &old_ref_b,
        &old_ref_definitions_b,
        &old_wnv_b,
        std::slice::from_ref(&polygon),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 2);
}

#[test]
fn cached_child_reference_keeps_distinct_source_polygon_families_separate() {
    let source_polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let source_polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 0));
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);
    let old_ref = p(0, 0, 0);
    let old_ref_definitions = axis_defs(&old_ref);
    let old_wnv = vec![0];

    cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        std::slice::from_ref(&source_polygon_a),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
        },
    )
    .unwrap();
    cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        std::slice::from_ref(&source_polygon_b),
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 2);
}

#[test]
fn cached_child_reference_reuses_permuted_source_polygon_families() {
    let source_polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let source_polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);
    let old_ref = p(0, 0, 0);
    let old_ref_definitions = axis_defs(&old_ref);
    let old_wnv = vec![0];

    let first = cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        &[source_polygon_a.clone(), source_polygon_b.clone()],
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(1, 2, 3), axis_defs(&p(1, 2, 3)), vec![7]))
        },
    )
    .unwrap();
    let second = cached_child_reference_with(
        &cache,
        &old_ref,
        &old_ref_definitions,
        &old_wnv,
        &[source_polygon_b, source_polygon_a],
        &bounds,
        || {
            calls.set(calls.get() + 1);
            Ok((p(4, 5, 6), axis_defs(&p(4, 5, 6)), vec![8]))
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
}

#[test]
fn cached_child_subdivision_reuses_identical_child_task() {
    let task = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(1, 1, 0)),
        p(0, 0, 0),
        vec![0],
    );
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_child_subdivision_with(&cache, &task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )])
    })
    .unwrap();
    let second = cached_child_subdivision_with(&cache, &task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
            1,
        )])
    })
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
}

#[test]
fn cached_child_subdivision_keeps_distinct_child_tasks_separate() {
    let task_a = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(1, 1, 0)),
        p(0, 0, 0),
        vec![0],
    );
    let task_b = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(2, 2, 0)),
        p(0, 0, 0),
        vec![0],
    );
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    cached_child_subdivision_with(&cache, &task_a, || {
        calls.set(calls.get() + 1);
        Ok(Vec::new())
    })
    .unwrap();
    cached_child_subdivision_with(&cache, &task_b, || {
        calls.set(calls.get() + 1);
        Ok(Vec::new())
    })
    .unwrap();

    assert_eq!(calls.get(), 2);
}

#[test]
fn cached_child_subdivision_reuses_permuted_parent_definition_families() {
    let mut task = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(1, 1, 0)),
        p(1, 2, 3),
        vec![0],
    );
    let definition = axis_defs(&task.ref_point)[0].clone();
    task.ref_definitions = vec![definition.clone()];
    let mut permuted_task = task.clone();
    permuted_task.ref_definitions = vec![[
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ]];
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_child_subdivision_with(&cache, &task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )])
    })
    .unwrap();
    let second = cached_child_subdivision_with(&cache, &permuted_task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
            1,
        )])
    })
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
}

#[test]
fn cached_child_subdivision_reuses_permuted_polygon_families() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let task = SubdivisionTask::new(
        vec![polygon_a.clone(), polygon_b.clone()],
        Aabb::new(p(0, 0, 0), p(1, 1, 1)),
        p(0, 0, 0),
        vec![0],
    );
    let mut permuted_task = task.clone();
    permuted_task.polygons = vec![polygon_b, polygon_a];
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_child_subdivision_with(&cache, &task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )])
    })
    .unwrap();
    let second = cached_child_subdivision_with(&cache, &permuted_task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
            1,
        )])
    })
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
}

#[test]
fn cached_child_subdivision_memoizes_current_equivalent_task_state() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let task = SubdivisionTask::new(
        vec![polygon_a.clone(), polygon_b.clone()],
        Aabb::new(p(0, 0, 0), p(1, 1, 1)),
        p(0, 0, 0),
        vec![0],
    );
    let mut permuted_task = task.clone();
    permuted_task.polygons = vec![polygon_b, polygon_a];
    let cache = RefCell::new(Vec::new());

    cached_child_subdivision_with(&cache, &task, || {
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )])
    })
    .unwrap();
    cached_child_subdivision_with(&cache, &permuted_task, || {
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
            1,
        )])
    })
    .unwrap();

    assert!(
        cache
            .borrow()
            .iter()
            .any(|existing| existing.task == permuted_task)
    );
}

#[test]
fn cached_child_subdivision_prefers_newest_exact_alias_state() {
    let polygon_a = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let polygon_b = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    let task = SubdivisionTask::new(
        vec![polygon_b.clone(), polygon_a.clone()],
        Aabb::new(p(0, 0, 0), p(1, 1, 1)),
        p(0, 0, 0),
        vec![0],
    );
    let older_task = SubdivisionTask::new(
        vec![polygon_a, polygon_b],
        task.bounds.clone(),
        task.ref_point.clone(),
        task.ref_wnv.clone(),
    );
    let older_result = vec![ClassifiedPolygon::new(
        make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
        1,
    )];
    let newer_result = vec![ClassifiedPolygon::new(
        make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
        1,
    )];
    let cache = RefCell::new(vec![
        ChildSubdivisionCacheEntry {
            polygon_profile: polygon_family_profile(&older_task.polygons),
            task: older_task,
            result: Ok(older_result),
        },
        ChildSubdivisionCacheEntry {
            polygon_profile: polygon_family_profile(&task.polygons),
            task: task.clone(),
            result: Ok(newer_result.clone()),
        },
    ]);

    let result = cached_child_subdivision_with(&cache, &task, || unreachable!()).unwrap();

    assert_eq!(result, newer_result);
}

#[test]
fn cached_child_subdivision_reuses_deeper_success_for_shallower_equivalent_task() {
    let mut deeper_task = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(1, 1, 0)),
        p(0, 0, 0),
        vec![0],
    );
    deeper_task.depth = 3;
    let mut shallower_task = deeper_task.clone();
    shallower_task.depth = 1;
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_child_subdivision_with(&cache, &deeper_task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )])
    })
    .unwrap();
    let second = cached_child_subdivision_with(&cache, &shallower_task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
            1,
        )])
    })
    .unwrap();

    assert_eq!(calls.get(), 1);
    assert_eq!(first, second);
}

#[test]
fn reusable_child_subdivision_if_certified_reuses_changed_reference_state() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let existing_task =
        SubdivisionTask::new(vec![polygon.clone()], bounds.clone(), p(1, 1, 1), vec![0]);
    let query_task = SubdivisionTask::new(vec![polygon], bounds, p(2, 1, 1), vec![0]);
    let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&existing_task.polygons),
        task: existing_task,
        result: Ok(vec![]),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused =
        reusable_child_subdivision_if_certified(&cache, &query_task, &mut query_caches).unwrap();

    assert_eq!(reused, Some(vec![]));
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_subdivision_if_certified_skips_invalid_reference_state() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let existing_task =
        SubdivisionTask::new(vec![polygon.clone()], bounds.clone(), p(1, 1, 1), vec![0]);
    let query_task = SubdivisionTask::new(vec![polygon], bounds, p(0, 0, 0), vec![0]);
    let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&existing_task.polygons),
        task: existing_task,
        result: Ok(vec![]),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused =
        reusable_child_subdivision_if_certified(&cache, &query_task, &mut query_caches).unwrap();

    assert_eq!(reused, None);
}

#[test]
fn reusable_child_subdivision_if_certified_memoizes_current_task_state() {
    let polygon = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let existing_task =
        SubdivisionTask::new(vec![polygon.clone()], bounds.clone(), p(1, 1, 1), vec![0]);
    let query_task = SubdivisionTask::new(vec![polygon], bounds, p(2, 1, 1), vec![0]);
    let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&existing_task.polygons),
        task: existing_task,
        result: Ok(vec![]),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();
    let calls = std::cell::Cell::new(0);

    let reused = reusable_child_subdivision_if_certified(&cache, &query_task, &mut query_caches)
        .unwrap()
        .unwrap();

    let cached = cached_child_subdivision_with(&cache, &query_task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )])
    })
    .unwrap();

    assert_eq!(calls.get(), 0);
    assert_eq!(cached, reused);
}

#[test]
fn reusable_child_subdivision_if_certified_reuses_result_across_tighter_bounds() {
    let polygon = make_triangle(&p(2, 2, 0), &p(4, 2, 0), &p(2, 4, 0), 0, 0);
    let existing_task = SubdivisionTask::new(
        vec![polygon.clone()],
        Aabb::new(p(0, 0, 0), p(6, 6, 6)),
        p(4, 4, 1),
        vec![0],
    );
    let query_task = SubdivisionTask::new(
        vec![polygon],
        Aabb::new(p(1, 1, 0), p(5, 5, 6)),
        p(4, 3, 1),
        vec![0],
    );
    let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&existing_task.polygons),
        task: existing_task,
        result: Ok(vec![]),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused =
        reusable_child_subdivision_if_certified(&cache, &query_task, &mut query_caches).unwrap();

    assert_eq!(reused, Some(vec![]));
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_subdivision_if_certified_skips_cached_result_invalid_for_tighter_bounds() {
    let polygon = make_triangle(&p(2, 2, 0), &p(4, 2, 0), &p(2, 4, 0), 0, 0);
    let existing_task = SubdivisionTask::new(
        vec![polygon.clone()],
        Aabb::new(p(0, 0, 0), p(6, 6, 6)),
        p(1, 1, 1),
        vec![0],
    );
    let query_task = SubdivisionTask::new(
        vec![polygon],
        Aabb::new(p(2, 2, 0), p(5, 5, 6)),
        p(4, 3, 1),
        vec![0],
    );
    let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&existing_task.polygons),
        task: existing_task,
        result: Ok(vec![]),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused =
        reusable_child_subdivision_if_certified(&cache, &query_task, &mut query_caches).unwrap();

    assert_eq!(reused, None);
}

#[test]
fn reusable_child_subdivision_from_cached_trace_if_certified_reuses_cached_result_across_parent_winding()
 {
    let mut polygon = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
    polygon.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, -2), p(4, 4, 2));
    let query_task =
        SubdivisionTask::new(vec![polygon.clone()], bounds.clone(), p(1, 1, 1), vec![0]);
    let existing_point = p(1, 1, -1);
    let existing_definitions = vec![axis_plane_definition(&existing_point)];
    let existing_wnv = trace_reference_target_from_validated_bounds(
        &query_task.ref_point,
        &query_task.ref_definitions,
        &query_task.ref_wnv,
        &query_task.bounds,
        &query_task.polygons,
        &ReferenceTarget::with_definitions(existing_point.clone(), existing_definitions.clone()),
    )
    .unwrap()
    .unwrap();
    let existing_task = SubdivisionTask {
        polygons: query_task.polygons.clone(),
        bounds: query_task.bounds.clone(),
        ref_point: existing_point,
        ref_definitions: existing_definitions,
        ref_wnv: existing_wnv,
        depth: query_task.depth,
    };
    let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&existing_task.polygons),
        task: existing_task,
        result: Ok(vec![]),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_subdivision_from_cached_trace_if_certified(
        &cache,
        &query_task,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(reused, Some(vec![]));
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_child_subdivision_from_cached_trace_if_certified_skips_mismatched_cached_winding() {
    let mut polygon = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
    polygon.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, -2), p(4, 4, 2));
    let query_task =
        SubdivisionTask::new(vec![polygon.clone()], bounds.clone(), p(1, 1, 1), vec![0]);
    let existing_point = p(1, 1, -1);
    let existing_task = SubdivisionTask {
        polygons: query_task.polygons.clone(),
        bounds: query_task.bounds.clone(),
        ref_point: existing_point.clone(),
        ref_definitions: vec![axis_plane_definition(&existing_point)],
        ref_wnv: vec![9],
        depth: query_task.depth,
    };
    let cache = RefCell::new(vec![ChildSubdivisionCacheEntry {
        polygon_profile: polygon_family_profile(&existing_task.polygons),
        task: existing_task,
        result: Ok(vec![]),
    }]);
    let mut query_caches = SupportReferenceQueryCaches::default();

    let reused = reusable_child_subdivision_from_cached_trace_if_certified(
        &cache,
        &query_task,
        &mut query_caches,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn cached_child_subdivision_keeps_shallower_and_deeper_successes_separate() {
    let mut shallower_task = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(1, 1, 0)),
        p(0, 0, 0),
        vec![0],
    );
    shallower_task.depth = 1;
    let mut deeper_task = shallower_task.clone();
    deeper_task.depth = 3;
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let first = cached_child_subdivision_with(&cache, &shallower_task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0),
            1,
        )])
    })
    .unwrap();
    let second = cached_child_subdivision_with(&cache, &deeper_task, || {
        calls.set(calls.get() + 1);
        Ok(vec![ClassifiedPolygon::new(
            make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 1),
            1,
        )])
    })
    .unwrap();

    assert_eq!(calls.get(), 2);
    assert_ne!(first, second);
}

#[test]
fn cached_child_subdivision_allows_nested_shared_cache_queries() {
    let task_a = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(1, 1, 0)),
        p(0, 0, 0),
        vec![0],
    );
    let task_b = SubdivisionTask::new(
        vec![make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0)],
        Aabb::new(p(0, 0, 0), p(2, 2, 0)),
        p(0, 0, 0),
        vec![0],
    );
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    cached_child_subdivision_with(&cache, &task_a, || {
        calls.set(calls.get() + 1);
        cached_child_subdivision_with(&cache, &task_b, || {
            calls.set(calls.get() + 1);
            Ok(Vec::new())
        })?;
        Ok(Vec::new())
    })
    .unwrap();

    cached_child_subdivision_with(&cache, &task_b, || {
        calls.set(calls.get() + 100);
        Ok(Vec::new())
    })
    .unwrap();

    assert_eq!(calls.get(), 2);
}

#[test]
fn support_target_collection_backtracks_after_uncertified_candidate() {
    let mut targets = Vec::new();

    extend_reference_targets_backtracking_unknown(
        &mut targets,
        [p(0, 0, 0), p(1, 2, 3)],
        |candidate| {
            if candidate == p(0, 0, 0) {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(vec![ReferenceTarget::axis_defined(candidate)])
            }
        },
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, p(1, 2, 3));
    assert!(targets[0].uncertified_definition_fallback);
}

#[test]
fn shifted_support_target_family_marks_surviving_targets_uncertain_after_boundary_report_witness() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let families = ShiftedSupportCellFamilies {
        shifted: halfspaces.clone(),
        report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            p(0, 2, 2),
            [None, None, None],
        )),
        saw_unknown: false,
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let cache = std::cell::RefCell::new(Vec::<ReferenceWitnessTargetCacheEntry>::new());
    let strict_contains_cache =
        std::cell::RefCell::new(Vec::<ReferenceHalfspaceContainmentCacheEntry>::new());
    let targets = shifted_support_cell_targets_from_families(
        &bounds,
        &halfspaces,
        &families,
        &cache,
        &strict_contains_cache,
    )
    .unwrap();

    assert!(!targets.is_empty());
    assert!(
        targets
            .iter()
            .all(|target| target.uncertified_definition_fallback)
    );
}

#[test]
fn shifted_support_target_family_prefers_certified_report_witness_duplicate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 2, 3);
    let families = ShiftedSupportCellFamilies {
        shifted: halfspaces.clone(),
        report: Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            witness.clone(),
            [Some(9), None, None],
        )),
        saw_unknown: false,
        strict_seeds: Vec::new(),
        shifted_vertices: Vec::new(),
        shifted_geometry_seeds: Vec::new(),
    };

    let cache = std::cell::RefCell::new(Vec::<ReferenceWitnessTargetCacheEntry>::new());
    let strict_contains_cache =
        std::cell::RefCell::new(Vec::<ReferenceHalfspaceContainmentCacheEntry>::new());
    let targets = shifted_support_cell_targets_from_families(
        &bounds,
        &halfspaces,
        &families,
        &cache,
        &strict_contains_cache,
    )
    .unwrap();

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].point, witness);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn reference_target_from_halfspace_witness_retains_axis_definition_when_active_definitions_fail() {
    let halfspaces = vec![axis_halfspace(0, false, r(1))];

    let target =
        reference_target_from_halfspace_witness(&p(1, 2, 3), &halfspaces, [Some(9), None, None])
            .unwrap();

    let target = target.expect("witness target should still be retained");
    assert_eq!(target.point, p(1, 2, 3));
    assert!(target.uncertified_definition_fallback);
    assert!(
        target
            .definitions
            .iter()
            .any(|definition| definition == &axis_plane_definition(&p(1, 2, 3)))
    );
}

#[test]
fn reference_target_from_halfspace_witness_salvages_coincident_halfspaces_after_invalid_active_index()
 {
    let witness = p(1, 2, 3);
    let halfspaces = vec![
        axis_halfspace(0, false, r(1)),
        LimitPlane3::new(p(1, 1, 1), r(-6)),
    ];

    let target =
        reference_target_from_halfspace_witness(&witness, &halfspaces, [Some(9), None, None])
            .unwrap()
            .expect("witness target should still be retained");

    assert_eq!(target.point, witness);
    assert!(target.uncertified_definition_fallback);
    assert!(target.definitions.iter().any(|definition| {
        definition
            .iter()
            .any(|plane| plane.normal == p(1, 1, 1) && plane.offset == r(-6))
    }));
}

#[test]
fn cached_reference_target_from_halfspace_witness_reuses_permuted_active_state() {
    let witness = p(1, 2, 3);
    let halfspaces = vec![
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, false, r(3)),
    ];
    let permuted = vec![
        halfspaces[2].clone(),
        halfspaces[0].clone(),
        halfspaces[1].clone(),
    ];
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_reference_target_from_halfspace_witness_with(
        &mut cache,
        &witness,
        &halfspaces,
        [Some(0), Some(1), None],
        || {
            calls += 1;
            Ok(Some(ReferenceTarget::axis_defined(witness.clone())))
        },
    )
    .unwrap();
    let second = cached_reference_target_from_halfspace_witness_with(
        &mut cache,
        &witness,
        &permuted,
        [Some(1), Some(2), None],
        || {
            calls += 1;
            Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9))))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_reference_target_from_halfspace_witness_distinguishes_active_state() {
    let witness = p(1, 2, 3);
    let halfspaces = vec![
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, false, r(3)),
    ];
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_reference_target_from_halfspace_witness_with(
        &mut cache,
        &witness,
        &halfspaces,
        [Some(0), None, None],
        || {
            calls += 1;
            Ok(Some(ReferenceTarget::axis_defined(witness.clone())))
        },
    )
    .unwrap();
    let second = cached_reference_target_from_halfspace_witness_with(
        &mut cache,
        &witness,
        &halfspaces,
        [Some(1), None, None],
        || {
            calls += 1;
            Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9))))
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_ne!(first, second);
}

#[test]
fn valid_reference_rejects_local_surface_points() {
    let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    assert!(!is_valid_reference_for_bounds(&p(2, 2, 1), &bounds, &[wall]).unwrap());
}

#[test]
fn surface_reference_normalization_selects_matching_positive_side_winding() {
    let mesh = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    let polygon = soup
        .polygons
        .iter()
        .find(|polygon| {
            classify_point_in_local_polygon(&p(1, 3, 3), polygon).unwrap()
                == LocalPolygonPointLocation::Interior
        })
        .unwrap()
        .clone();
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let (point, definitions, winding) =
        normalize_surface_reference(&p(1, 3, 3), &[-1], &bounds, &soup.polygons)
            .unwrap()
            .expect("surface interior should have a positive-side departure");

    assert_eq!(
        crate::geometry::classify_point(&point, &polygon.support).unwrap(),
        Classification::Positive
    );
    assert_eq!(winding, vec![-1]);
    assert_eq!(affine_from_planes(&definitions[0]).unwrap(), point);
    assert!(is_certified_valid_reference_for_bounds(&point, &bounds, &soup.polygons).unwrap());
}

#[test]
fn surface_reference_normalization_selects_matching_negative_side_winding() {
    let mesh = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    let polygon = soup
        .polygons
        .iter()
        .find(|polygon| {
            classify_point_in_local_polygon(&p(1, 3, 3), polygon).unwrap()
                == LocalPolygonPointLocation::Interior
        })
        .unwrap()
        .clone();
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let (point, _definitions, winding) =
        normalize_surface_reference(&p(1, 3, 3), &[0], &bounds, &soup.polygons)
            .unwrap()
            .expect("surface interior should depart on the side with room");

    assert_eq!(
        crate::geometry::classify_point(&point, &polygon.support).unwrap(),
        Classification::Negative
    );
    assert_eq!(winding, vec![0]);
    assert!(is_certified_valid_reference_for_bounds(&point, &bounds, &soup.polygons).unwrap());
}

#[test]
fn surface_reference_normalization_finds_matching_vertex_cell() {
    let mesh = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let (point, definitions, winding) =
        normalize_surface_reference(&p(1, 1, 1), &[0], &bounds, &soup.polygons)
            .unwrap()
            .expect("closed vertex contact should find the matching arrangement cell");

    assert_eq!(winding, vec![0]);
    assert_eq!(affine_from_planes(&definitions[0]).unwrap(), point);
    assert!(is_certified_valid_reference_for_bounds(&point, &bounds, &soup.polygons).unwrap());
}

#[test]
fn surface_reference_normalization_finds_matching_edge_cell() {
    let mesh = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let (point, _definitions, winding) =
        normalize_surface_reference(&p(1, 2, 1), &[-1], &bounds, &soup.polygons)
            .unwrap()
            .expect("closed edge contact should find the matching arrangement cell");

    assert_eq!(winding, vec![-1]);
    assert!(is_certified_valid_reference_for_bounds(&point, &bounds, &soup.polygons).unwrap());
}

#[test]
fn surface_reference_normalization_finds_matching_noncoplanar_surface_cell() {
    let left = tetra_from_face_and_apex(p(2, 0, 0), p(2, 6, 0), p(2, 3, 6), p(0, 3, 2));
    let right = tetra_from_face_and_apex(p(0, 3, 0), p(6, 3, 0), p(3, 3, 6), p(3, 5, 2));
    let soup = prepare_input(&[left.as_ref(), right.as_ref()]).unwrap();
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let (point, _definitions, winding) =
        normalize_surface_reference(&p(2, 3, 2), &[0, 0], &bounds, &soup.polygons)
            .unwrap()
            .expect("crossing surface contact should find the matching arrangement cell");

    assert_eq!(winding, vec![0, 0]);
    assert!(is_certified_valid_reference_for_bounds(&point, &bounds, &soup.polygons).unwrap());
}

#[test]
fn surface_reference_normalization_exhausts_adjacent_cells_for_impossible_winding() {
    let mesh = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let err = normalize_surface_reference(&p(1, 1, 1), &[7], &bounds, &soup.polygons).unwrap_err();

    assert_eq!(
        err,
        crate::error::HypermeshError::ReferencePropagationFailed
    );
}

#[test]
fn surface_reference_closure_allows_non_manifold_edge_valence() {
    let polygon = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    let polygons = vec![
        polygon.clone(),
        polygon.clone(),
        polygon.inverted(),
        polygon.inverted(),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    assert!(polygon_family_is_closed_within_bounds(&polygons, &bounds, 1).unwrap());
}

#[test]
fn surface_reference_closure_rejects_unbalanced_non_manifold_edge_valence() {
    let polygon = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    let polygons = vec![polygon.clone(), polygon.clone(), polygon];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    assert!(!polygon_family_is_closed_within_bounds(&polygons, &bounds, 1).unwrap());
}

#[test]
fn surface_reference_closure_rejects_boundary_edges() {
    let polygon = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    assert!(!polygon_family_is_closed_within_bounds(&[polygon], &bounds, 1).unwrap());
}

#[test]
fn surface_reference_closure_rejects_missing_winding_component_mesh() {
    let polygon = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    let polygons = vec![polygon.clone(), polygon];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    assert!(!polygon_family_is_closed_within_bounds(&polygons, &bounds, 2).unwrap());
}

#[test]
fn certified_reference_validity_reports_unknown_for_local_surface_boundary_point() {
    let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    assert_eq!(
        is_certified_valid_reference_for_bounds(&p(2, 1, 2), &bounds, &[wall]),
        Err(crate::error::HypermeshError::UnknownClassification)
    );
}

#[test]
fn compute_new_reference_reports_unknown_after_boundary_inherited_reference_if_search_exhausts() {
    let mut wall = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let old_ref = p(0, 0, 0);
    let bounds = Aabb::new(old_ref.clone(), old_ref.clone());

    let err =
        compute_new_reference(&old_ref, &axis_defs(&old_ref), &[0], &bounds, &[wall]).unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn trace_reference_target_rejects_invalid_targets() {
    let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));

    assert_eq!(
        trace_reference_target(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[0],
            &bounds,
            &[wall.clone()],
            &ReferenceTarget::axis_defined(p(2, 2, 1))
        )
        .unwrap(),
        None
    );
    assert_eq!(
        trace_reference_target(
            &p(-1, -1, -1),
            &axis_defs(&p(-1, -1, -1)),
            &[0],
            &bounds,
            &[wall],
            &ReferenceTarget::axis_defined(p(5, 2, 2))
        )
        .unwrap(),
        None
    );
}

#[test]
fn reference_trace_bounds_expand_only_to_inherited_reference() {
    let bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));

    let expanded = reference_trace_bounds(&p(-2, 1, 3), &bounds).unwrap();
    let exterior = exterior_reference_point(&bounds).unwrap();
    let exterior_expanded = reference_trace_bounds(&exterior, &bounds).unwrap();

    assert_eq!(expanded, Aabb::new(p(-2, 0, 0), p(1, 1, 3)));
    assert_eq!(exterior_expanded, Aabb::new(p(0, 0, 0), p(2, 2, 2)));
}

#[test]
fn trace_reference_target_uses_bounded_detour_for_valid_target() {
    let ref_point = p(0, 0, 0);
    let target_point = p(2, 1, 0);
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, -1, -1), p(3, 2, 1));

    assert_eq!(
        crate::segment_trace::trace_segment_without_detours(
            &ref_point,
            &target_point,
            &[0],
            &[wall.clone()],
        ),
        Err(crate::error::HypermeshError::UnknownClassification)
    );

    let winding = trace_reference_target(
        &ref_point,
        &axis_defs(&ref_point),
        &[0],
        &bounds,
        &[wall],
        &ReferenceTarget::axis_defined(target_point),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![0]));
}

#[test]
fn trace_reference_target_retries_axis_plane_replacement_definitions() {
    let ref_point = p(0, 0, 0);
    let target_point = p(2, 1, 0);
    let ref_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(1, r(0)),
        Plane::from_coefficients(r(1), r(1), r(1), r(0)),
    ];
    let invalid_definition = [
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(0)),
    ];
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, -1, -1), p(3, 2, 1));

    assert_eq!(
        crate::segment_trace::trace_segment_without_detours(
            &ref_point,
            &target_point,
            &[0],
            &[wall.clone()],
        ),
        Err(crate::error::HypermeshError::UnknownClassification)
    );

    let winding = trace_reference_target(
        &ref_point,
        &[ref_definition],
        &[0],
        &bounds,
        &[wall],
        &ReferenceTarget::with_definitions(target_point, vec![invalid_definition]),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![0]));
}

#[test]
fn trace_reference_target_retries_axis_start_after_retained_definitions_fail() {
    let ref_point = p(0, 0, 0);
    let target_point = p(2, 1, 0);
    let invalid_ref_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(0, r(2)),
    ];
    let valid_target_definition = [
        Plane::axis_aligned(0, r(2)),
        Plane::axis_aligned(1, r(1)),
        Plane::from_coefficients(r(1), r(1), r(1), r(-3)),
    ];
    let mut wall = make_triangle(&p(1, -2, -2), &p(1, -2, 0), &p(1, 1, 0), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, -1, -1), p(3, 2, 1));

    assert_eq!(
        crate::segment_trace::trace_segment_without_detours(
            &ref_point,
            &target_point,
            &[0],
            &[wall.clone()],
        ),
        Err(crate::error::HypermeshError::UnknownClassification)
    );

    let winding = trace_reference_target(
        &ref_point,
        &[invalid_ref_definition],
        &[0],
        &bounds,
        &[wall],
        &ReferenceTarget::with_definitions(target_point, vec![valid_target_definition]),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![0]));
}

#[test]
fn trace_reference_target_uses_detour_on_plane_replacement_step() {
    let ref_point = p(0, 0, 0);
    let target_point = p(4, 0, 0);
    let ref_definition = [
        Plane::axis_aligned(0, r(0)),
        Plane::from_coefficients(r(-1), r(1), r(0), r(0)),
        Plane::from_coefficients(r(-1), r(0), r(1), r(0)),
    ];
    let target_definition = [
        Plane::from_coefficients(r(1), r(1), r(0), r(-4)),
        Plane::axis_aligned(1, r(0)),
        Plane::axis_aligned(2, r(0)),
    ];
    let mut blockers = vec![
        make_triangle(&p(2, 0, 0), &p(3, 0, 0), &p(2, 1, 0), 0, 0),
        make_triangle(&p(0, 2, 0), &p(1, 2, 0), &p(0, 3, 0), 0, 1),
        make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 0, 2),
    ];
    for (index, x) in [q(2, 3), r(1), q(4, 3)].into_iter().enumerate() {
        blockers.push(make_triangle(
            &px(x.clone(), -1, -1),
            &px(x.clone(), 3, -1),
            &px(x, 1, 3),
            0,
            3 + index as isize,
        ));
    }
    let bounds = Aabb::new(p(0, -1, -1), p(5, 3, 5));

    assert_eq!(
        crate::segment_trace::trace_segment_without_detours(
            &ref_point,
            &target_point,
            &[0],
            &blockers,
        ),
        Err(crate::error::HypermeshError::UnknownClassification)
    );

    let winding = trace_reference_target(
        &ref_point,
        &[ref_definition],
        &[0],
        &bounds,
        &blockers,
        &ReferenceTarget::with_definitions(target_point, vec![target_definition]),
    )
    .unwrap();

    assert_eq!(winding, Some(vec![0]));
}

#[test]
fn projection_escape_bounds_stop_at_nearest_axis_surfaces() {
    let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    left.delta_w = vec![1];
    let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    right.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let escape = projection_escape_bounds(&p(1, 3, 3), &bounds, &[left, right])
        .unwrap()
        .expect("parallel walls should define a strict projection escape box");

    assert_eq!(escape.min.x, r(0));
    assert_eq!(escape.max.x, r(4));
    assert_eq!(escape.min.y, r(0));
    assert_eq!(escape.max.y, r(6));
    assert_eq!(escape.min.z, r(0));
    assert_eq!(escape.max.z, r(6));
}

#[test]
fn projection_escape_bounds_family_includes_later_exact_boxes() {
    let mut x_wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
    x_wall.delta_w = vec![1];
    let mut y_wall = make_triangle(&p(0, 5, 0), &p(6, 5, 0), &p(0, 5, 6), 0, 1);
    y_wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let family = projection_escape_bounds_family(&p(1, 3, 3), &bounds, &[x_wall, y_wall]).unwrap();

    assert!(family.len() >= 4);
    assert_eq!(family[0], Aabb::new(p(0, 0, 0), p(4, 5, 6)));
    assert!(
        family
            .iter()
            .any(|bounds| *bounds == Aabb::new(p(0, 0, 0), p(6, 5, 6)))
    );
    assert!(
        family
            .iter()
            .any(|bounds| *bounds == Aabb::new(p(0, 0, 0), p(4, 6, 6)))
    );
}

#[test]
fn projection_escape_bounds_family_backtracks_after_uncertified_candidate_box() {
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
    ];

    let (family, saw_unknown) =
        projection_escape_bounds_family_from_axis_options_with_extents(&axis_options, |bounds| {
            if *bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(true)
            }
        })
        .unwrap();

    assert!(saw_unknown);
    assert_eq!(family, vec![Aabb::new(p(0, 0, 0), p(2, 1, 1))]);
}

#[test]
fn escaped_reference_axis_stop_values_backtrack_after_uncertified_crossing() {
    let projected = p(0, 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
        &projected,
        &bounds,
        &[first, second],
        0,
        true,
        |_projected, _endpoint, polygon, _axis| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Some(p(2, 0, 0)))
            }
        },
        |_crossing, _polygon| Ok(LocalPolygonPointLocation::Interior),
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3)]);
}

#[test]
fn escaped_reference_axis_stop_values_treat_boundary_crossing_as_unknown_and_keep_later_corridor() {
    let projected = p(0, 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
    let first = make_triangle(&p(1, 0, 0), &p(1, 1, 0), &p(1, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
        &projected,
        &bounds,
        &[first, second],
        0,
        true,
        |_projected, _endpoint, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            Ok(Some(Point3::new(x, r(0), r(0))))
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(1) {
                Ok(LocalPolygonPointLocation::Boundary)
            } else {
                Ok(LocalPolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3)]);
}

#[test]
fn escaped_reference_axis_stop_values_treat_endpoint_boundary_contact_as_unknown_and_keep_later_corridor()
 {
    let projected = p(0, 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
    let first = make_triangle(&p(3, 0, 0), &p(3, 1, 0), &p(3, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
        &projected,
        &bounds,
        &[first, second],
        0,
        true,
        |_projected, endpoint, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            if x == r(3) {
                Ok(Some(endpoint.clone()))
            } else {
                Ok(Some(Point3::new(x, r(0), r(0))))
            }
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(3) {
                Ok(LocalPolygonPointLocation::Boundary)
            } else {
                Ok(LocalPolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3)]);
}

#[test]
fn escaped_reference_axis_stop_values_treat_start_boundary_contact_as_unknown_and_keep_later_corridor()
 {
    let projected = p(0, 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));
    let first = make_triangle(&p(0, 0, 0), &p(0, 1, 0), &p(0, 0, 1), 0, 0);
    let second = make_triangle(&p(2, 0, 0), &p(2, 1, 0), &p(2, 0, 1), 0, 1);

    let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
        &projected,
        &bounds,
        &[first, second],
        0,
        true,
        |projected, _endpoint, polygon, _axis| {
            let x = polygon.vertices().unwrap()[0].x.clone();
            if x == r(0) {
                Ok(Some(projected.clone()))
            } else {
                Ok(Some(Point3::new(x, r(0), r(0))))
            }
        },
        |_crossing, polygon| {
            if polygon.vertices().unwrap()[0].x == r(0) {
                Ok(LocalPolygonPointLocation::Boundary)
            } else {
                Ok(LocalPolygonPointLocation::Interior)
            }
        },
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(stop_values, vec![r(2), r(3)]);
}

#[test]
fn escaped_reference_axis_stop_values_treat_bound_start_contact_as_unknown() {
    let projected = p(3, 0, 0);
    let bounds = Aabb::new(p(0, 0, 0), p(3, 1, 1));

    let (stop_values, saw_unknown) = escaped_reference_axis_stop_values_with_queries(
        &projected,
        &bounds,
        &[],
        0,
        true,
        |_projected, _endpoint, _polygon, _axis| Ok(None),
        |_crossing, _polygon| Ok(LocalPolygonPointLocation::Outside),
    )
    .unwrap();

    assert!(saw_unknown);
    assert!(stop_values.is_empty());
}

#[test]
fn projection_axis_escape_reference_certifies_fallback_corridor_witness_after_trace() {
    let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    left.delta_w = vec![1];
    let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    right.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let found = projection_axis_escape_reference(
        &p(-1, 3, 3),
        &axis_defs(&p(-1, 3, 3)),
        &[0],
        &p(1, 3, 3),
        &bounds,
        &[left, right],
    )
    .unwrap();

    assert_certified_reference_result(found, &Point3::new(q(5, 2), r(3), r(3)), &[-1]);
}

#[test]
fn cached_projection_escape_axis_options_reuses_projected_target_point() {
    let projected = p(1, 3, 3);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_projection_escape_axis_options_with(
        &mut cache,
        &projected,
        &bounds,
        &polygons,
        || {
            calls += 1;
            Ok(vec![(vec![r(0)], vec![r(4)]); 3])
        },
    )
    .unwrap();
    let second = cached_projection_escape_axis_options_with(
        &mut cache,
        &projected,
        &bounds,
        &polygons,
        || {
            calls += 1;
            Ok(vec![(vec![r(0)], vec![r(6)]); 3])
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn support_reference_query_caches_reuse_identical_halfspace_queries() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut query_caches = SupportReferenceQueryCaches::default();
    let mut report_calls = 0;
    let mut feasible_calls = 0;

    let first_report =
        cached_halfspace_report_with(&mut query_caches.report_cache, &halfspaces, |_halfspaces| {
            report_calls += 1;
            Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(1, 1, 1),
                [None, None, None],
            )))
        })
        .unwrap();
    let first_feasible = cached_halfspace_feasibility_with(
        &mut query_caches.feasible_cache,
        &halfspaces,
        |_halfspaces| {
            feasible_calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second_report =
        cached_halfspace_report_with(&mut query_caches.report_cache, &halfspaces, |_halfspaces| {
            report_calls += 1;
            Ok(None)
        })
        .unwrap();
    let second_feasible = cached_halfspace_feasibility_with(
        &mut query_caches.feasible_cache,
        &halfspaces,
        |_halfspaces| {
            feasible_calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert_eq!(report_calls, 1);
    assert_eq!(feasible_calls, 1);
    assert_eq!(first_report, second_report);
    assert_eq!(first_feasible, second_feasible);
}

#[test]
fn support_reference_query_caches_reuse_report_for_feasibility() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut query_caches = SupportReferenceQueryCaches::default();
    let mut report_calls = 0;
    let mut feasible_calls = 0;

    let report =
        cached_halfspace_report_with(&mut query_caches.report_cache, &halfspaces, |_halfspaces| {
            report_calls += 1;
            Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(1, 1, 1),
                [None, None, None],
            )))
        })
        .unwrap();
    let feasible = cached_halfspace_feasibility_with_report_cache(
        &mut query_caches.report_cache,
        &mut query_caches.feasible_cache,
        &halfspaces,
        |_halfspaces| {
            report_calls += 1;
            Ok(None)
        },
        |_halfspaces| {
            feasible_calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert!(report.is_some());
    assert!(feasible);
    assert_eq!(report_calls, 1);
    assert_eq!(feasible_calls, 0);
}

#[test]
fn support_reference_query_caches_prime_report_from_feasibility() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut query_caches = SupportReferenceQueryCaches::default();
    let mut report_calls = 0;
    let mut feasible_calls = 0;

    let feasible = cached_halfspace_feasibility_with_report_cache(
        &mut query_caches.report_cache,
        &mut query_caches.feasible_cache,
        &halfspaces,
        |_halfspaces| {
            report_calls += 1;
            Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
                p(1, 1, 1),
                [None, None, None],
            )))
        },
        |_halfspaces| {
            feasible_calls += 1;
            Ok(false)
        },
    )
    .unwrap();
    let report =
        cached_halfspace_report_with(&mut query_caches.report_cache, &halfspaces, |_halfspaces| {
            report_calls += 1;
            Ok(None)
        })
        .unwrap();

    assert!(feasible);
    assert!(report.is_some());
    assert_eq!(report_calls, 1);
    assert_eq!(feasible_calls, 0);
}

#[test]
fn support_reference_query_caches_prime_projected_root_report_for_later_support_queries() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let old_ref = p(-1, 2, 2);
    let halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
    let projected_root =
        projected_root_reference_families(&bounds, &halfspaces, &mut Vec::new()).unwrap();
    let mut query_caches = SupportReferenceQueryCaches::default();
    let mut report_calls = 0;
    let mut feasible_calls = 0;

    prime_support_reference_query_caches_with_known_halfspace_report(
        &mut query_caches,
        &halfspaces,
        projected_root.report.as_ref(),
        projected_root.saw_unknown,
    );

    let report =
        cached_halfspace_report_with(&mut query_caches.report_cache, &halfspaces, |_halfspaces| {
            report_calls += 1;
            Ok(None)
        })
        .unwrap();
    let feasible = cached_halfspace_feasibility_with_report_cache(
        &mut query_caches.report_cache,
        &mut query_caches.feasible_cache,
        &halfspaces,
        |_halfspaces| {
            report_calls += 1;
            Ok(None)
        },
        |_halfspaces| {
            feasible_calls += 1;
            Ok(false)
        },
    )
    .unwrap();

    assert_eq!(report, projected_root.report);
    assert!(feasible);
    assert_eq!(report_calls, 0);
    assert_eq!(feasible_calls, 0);
}

#[test]
fn cached_projected_root_reference_families_reuse_permuted_halfspace_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let old_ref = p(-1, 2, 2);
    let halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(2);
    let mut caches = SupportReferenceQueryCaches::default();

    let first =
        cached_projected_root_reference_families_with(&bounds, &halfspaces, &mut caches).unwrap();
    let second =
        cached_projected_root_reference_families_with(&bounds, &permuted, &mut caches).unwrap();

    assert_eq!(first.report, second.report);
    assert_eq!(first.projected_targets, second.projected_targets);
    assert_eq!(
        first.projected_escape_seed_families,
        second.projected_escape_seed_families
    );
    assert_eq!(first.saw_unknown, second.saw_unknown);
    assert_eq!(caches.projected_root_cache.len(), 1);
}

#[test]
fn cached_support_cell_seed_geometry_reuses_identical_halfspaces() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = projected_reference_halfspaces(&p(-1, 2, 2), &bounds).unwrap();
    let mut cache = Vec::new();
    let mut centroid_subset_seed_cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_cell_seed_geometry_with(&mut cache, &halfspaces, || {
        calls += 1;
        support_cell_seed_geometry_state(&halfspaces, &mut centroid_subset_seed_cache)
    })
    .unwrap();
    let second = cached_support_cell_seed_geometry_with(&mut cache, &halfspaces, || {
        calls += 1;
        Ok(SupportCellSeedGeometryState {
            shifted_vertices: vec![p(9, 9, 9)],
            shifted_geometry_seeds: vec![p(8, 8, 8)],
            saw_unknown: false,
        })
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_cell_seed_geometry_reuses_permuted_halfspaces() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = projected_reference_halfspaces(&p(-1, 2, 2), &bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let mut cache = Vec::new();
    let mut centroid_subset_seed_cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_cell_seed_geometry_with(&mut cache, &halfspaces, || {
        calls += 1;
        support_cell_seed_geometry_state(&halfspaces, &mut centroid_subset_seed_cache)
    })
    .unwrap();
    let second = cached_support_cell_seed_geometry_with(&mut cache, &permuted, || {
        calls += 1;
        Ok(SupportCellSeedGeometryState {
            shifted_vertices: vec![p(9, 9, 9)],
            shifted_geometry_seeds: vec![p(8, 8, 8)],
            saw_unknown: false,
        })
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_point3_centroid_subset_family_reuses_permuted_vertices() {
    let first_vertices = vec![p(0, 0, 0), p(2, 0, 0), p(0, 2, 0)];
    let second_vertices = vec![p(0, 2, 0), p(0, 0, 0), p(2, 0, 0)];
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_point3_centroid_subset_family_from_vertices_with(
        &mut cache,
        &first_vertices,
        || {
            calls += 1;
            point3_centroid_subset_family_from_vertices(&first_vertices)
        },
    )
    .unwrap();
    let second = cached_point3_centroid_subset_family_from_vertices_with(
        &mut cache,
        &second_vertices,
        || {
            calls += 1;
            Ok(Point3FamilyState {
                points: vec![p(9, 9, 9)],
                saw_unknown: true,
            })
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn support_reference_query_caches_prime_unknown_report_for_later_support_queries() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut query_caches = SupportReferenceQueryCaches::default();
    let mut report_calls = 0;
    let mut feasible_calls = 0;

    prime_support_reference_query_caches_with_known_halfspace_report(
        &mut query_caches,
        &halfspaces,
        None,
        true,
    );

    let report_err =
        cached_halfspace_report_with(&mut query_caches.report_cache, &halfspaces, |_halfspaces| {
            report_calls += 1;
            Ok(None)
        })
        .unwrap_err();
    let feasible_err = cached_halfspace_feasibility_with_report_cache(
        &mut query_caches.report_cache,
        &mut query_caches.feasible_cache,
        &halfspaces,
        |_halfspaces| {
            report_calls += 1;
            Ok(None)
        },
        |_halfspaces| {
            feasible_calls += 1;
            Ok(false)
        },
    )
    .unwrap_err();

    assert_eq!(
        report_err,
        crate::error::HypermeshError::UnknownClassification
    );
    assert_eq!(
        feasible_err,
        crate::error::HypermeshError::UnknownClassification
    );
    assert_eq!(report_calls, 0);
    assert_eq!(feasible_calls, 0);
}

#[test]
fn support_reference_query_caches_reset_preserves_shareable_caches() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let point = p(1, 1, 1);
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let context = support_reference_cache_context_key(
        &point,
        &[axis_plane_definition(&point)],
        &[0],
        &[support_only_polygon(Plane::axis_aligned(0, r(2)))],
    );
    let mut query_caches = SupportReferenceQueryCaches::default();

    query_caches.report_cache.push(HalfspaceReportCacheEntry {
        halfspaces: halfspaces.clone(),
        report: Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            point.clone(),
            [None, None, None],
        ))),
    });
    query_caches
        .seed_geometry_cache
        .push(SupportCellSeedGeometryCacheEntry {
            halfspaces: halfspaces.clone(),
            geometry: Ok(SupportCellSeedGeometryState {
                shifted_vertices: vec![point.clone()],
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: false,
            }),
        });
    query_caches
        .centroid_subset_seed_cache
        .push(Point3CentroidSubsetFamilyCacheEntry {
            vertices: vec![point.clone()],
            family: Ok(Point3FamilyState {
                points: Vec::new(),
                saw_unknown: false,
            }),
        });
    query_caches
        .reference_witness_cache
        .get_mut()
        .push(ReferenceWitnessTargetCacheEntry {
            point: point.clone(),
            halfspaces: halfspaces.clone(),
            active_planes: [None, None, None],
            target: Ok(Some(ReferenceTarget::axis_defined(point.clone()))),
        });
    query_caches
        .trace_cache
        .push(ReferenceTargetTraceCacheEntry {
            context: Some(context.clone()),
            target: ReferenceTarget::axis_defined(point.clone()),
            winding: Ok(Some(vec![0])),
        });
    query_caches
        .validity_cache
        .push(ReferenceBoundsValidityCacheEntry {
            context: Some(support_reference_polygon_context_key_from_support_context(
                &context,
            )),
            bounds: bounds.clone(),
            point: point.clone(),
            is_valid: Ok(true),
        });
    query_caches
        .support_surface_cache
        .push(SupportSurfaceCacheEntry {
            context: Some(support_reference_polygon_context_key_from_support_context(
                &context,
            )),
            point: point.clone(),
            on_support_surface: Ok(false),
        });
    query_caches
        .accept_cache
        .get_mut()
        .push(SupportReferenceAcceptCacheEntry {
            context: Some(context.clone()),
            bounds: bounds.clone(),
            halfspaces: halfspaces.clone(),
            report: None,
            accepted: Ok(None),
        });
    query_caches
        .search_cache
        .get_mut()
        .push(SupportPlaneCellSearchCacheEntry {
            context: Some(context.clone()),
            bounds: bounds.clone(),
            polygon_index: 0,
            halfspaces: halfspaces.clone(),
            result: Ok(None::<(ReferenceTarget, Vec<i32>)>),
        });
    query_caches
        .support_reference_result_cache
        .push(SupportReferenceResultCacheEntry {
            context: context.clone(),
            bounds: bounds.clone(),
            halfspaces: halfspaces.clone(),
            result: Ok(None),
        });
    query_caches
        .projected_reference_result_cache
        .push(ProjectedReferenceResultCacheEntry {
            context,
            bounds: bounds.clone(),
            halfspaces: halfspaces.clone(),
            result: Ok(None),
        });

    query_caches.reset_per_reference_call_caches();

    assert_eq!(query_caches.report_cache.len(), 1);
    assert_eq!(query_caches.seed_geometry_cache.len(), 1);
    assert_eq!(query_caches.centroid_subset_seed_cache.len(), 1);
    assert_eq!(query_caches.reference_witness_cache.get_mut().len(), 1);
    assert!(query_caches.support_seed_family_cache.is_empty());
    assert!(query_caches.support_direct_target_cache.is_empty());
    assert!(query_caches.projected_root_cache.is_empty());
    assert!(
        query_caches
            .projection_escape_axis_options_cache
            .get_mut()
            .is_empty()
    );
    assert!(
        query_caches
            .projection_escape_search_cache
            .get_mut()
            .is_empty()
    );
    assert!(query_caches.shifted_projected_family_cache.is_empty());
    assert!(query_caches.shifted_support_family_cache.is_empty());
    assert!(query_caches.strict_contains_cache.get_mut().is_empty());
    assert!(
        query_caches
            .pure_halfspace_contains_cache
            .get_mut()
            .is_empty()
    );
    assert!(query_caches.trace_cache.is_empty());
    assert!(query_caches.validity_cache.is_empty());
    assert!(query_caches.support_surface_cache.is_empty());
    assert!(query_caches.target_cache.get_mut().is_empty());
    assert!(query_caches.accept_cache.get_mut().is_empty());
    assert!(query_caches.projected_reference_result_cache.is_empty());
    assert!(query_caches.support_reference_result_cache.is_empty());
    assert!(query_caches.search_cache.get_mut().is_empty());
}

#[test]
fn cached_projection_escape_axis_options_state_reuses_projected_target_point() {
    let projected = p(1, 3, 3);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_projection_escape_axis_options_state_with(
        &mut cache,
        &projected,
        &bounds,
        &polygons,
        || {
            calls += 1;
            Ok(ProjectionEscapeAxisOptionsState {
                axis_options: vec![(vec![r(0)], vec![r(4)]); 3],
                saw_unknown: true,
            })
        },
    )
    .unwrap();
    let second = cached_projection_escape_axis_options_state_with(
        &mut cache,
        &projected,
        &bounds,
        &polygons,
        || {
            calls += 1;
            Ok(ProjectionEscapeAxisOptionsState {
                axis_options: vec![(vec![r(0)], vec![r(6)]); 3],
                saw_unknown: false,
            })
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_halfspace_report_reuses_identical_state() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_halfspace_report_with(&mut cache, &halfspaces, |_halfspaces| {
        calls += 1;
        Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            p(1, 1, 1),
            [None, None, None],
        )))
    })
    .unwrap();
    let second = cached_halfspace_report_with(&mut cache, &halfspaces, |_halfspaces| {
        calls += 1;
        Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            p(2, 2, 2),
            [Some(0), None, None],
        )))
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_halfspace_report_reuses_permuted_state() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_halfspace_report_with(&mut cache, &halfspaces, |_halfspaces| {
        calls += 1;
        Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            p(1, 1, 1),
            [None, None, None],
        )))
    })
    .unwrap();
    let second = cached_halfspace_report_with(&mut cache, &permuted, |_halfspaces| {
        calls += 1;
        Ok(Some(hyperlimit::HalfspaceFeasibilityReport::feasible(
            p(2, 2, 2),
            [Some(0), None, None],
        )))
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_halfspace_feasibility_reuses_identical_state() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_halfspace_feasibility_with(&mut cache, &halfspaces, |_halfspaces| {
        calls += 1;
        Ok(true)
    })
    .unwrap();
    let second = cached_halfspace_feasibility_with(&mut cache, &halfspaces, |_halfspaces| {
        calls += 1;
        Ok(false)
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_halfspace_feasibility_reuses_permuted_state() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_halfspace_feasibility_with(&mut cache, &halfspaces, |_halfspaces| {
        calls += 1;
        Ok(true)
    })
    .unwrap();
    let second = cached_halfspace_feasibility_with(&mut cache, &permuted, |_halfspaces| {
        calls += 1;
        Ok(false)
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_reference_target_trace_reuses_identical_target() {
    let target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_reference_target_trace_with(&mut cache, &target, |_target| {
        calls += 1;
        Ok(Some(vec![17]))
    })
    .unwrap();
    let second = cached_reference_target_trace_with(&mut cache, &target, |_target| {
        calls += 1;
        Ok(Some(vec![99]))
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_reference_target_trace_distinguishes_reference_context() {
    let point = p(1, 2, 3);
    let target = ReferenceTarget::axis_defined(point.clone());
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let left_old_ref = p(0, 0, 0);
    let left_context = support_reference_cache_context_key(
        &left_old_ref,
        &[axis_plane_definition(&left_old_ref)],
        &[0],
        &polygons,
    );
    let right_old_ref = p(1, 0, 0);
    let right_context = support_reference_cache_context_key(
        &right_old_ref,
        &[axis_plane_definition(&right_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_reference_target_trace_with_context(
        &mut cache,
        Some(&left_context),
        &target,
        |_target| {
            calls += 1;
            Ok(Some(vec![17]))
        },
    )
    .unwrap();
    let second = cached_reference_target_trace_with_context(
        &mut cache,
        Some(&right_context),
        &target,
        |_target| {
            calls += 1;
            Ok(Some(vec![23]))
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_eq!(first, Some(vec![17]));
    assert_eq!(second, Some(vec![23]));
}

#[test]
fn cached_reference_bounds_validity_reuses_identical_point() {
    let point = p(1, 2, 3);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_reference_bounds_validity_with(&mut cache, &bounds, &point, |_point| {
        calls += 1;
        Ok(true)
    })
    .unwrap();
    let second = cached_reference_bounds_validity_with(&mut cache, &bounds, &point, |_point| {
        calls += 1;
        Ok(false)
    })
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_reference_bounds_validity_keeps_distinct_bounds_separate() {
    let point = p(1, 2, 3);
    let bounds_a = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let bounds_b = Aabb::new(p(0, 0, 0), p(5, 4, 4));
    let mut cache = Vec::new();
    let mut calls = 0;

    cached_reference_bounds_validity_with(&mut cache, &bounds_a, &point, |_point| {
        calls += 1;
        Ok(true)
    })
    .unwrap();
    cached_reference_bounds_validity_with(&mut cache, &bounds_b, &point, |_point| {
        calls += 1;
        Ok(true)
    })
    .unwrap();

    assert_eq!(calls, 2);
}

#[test]
fn cached_reference_bounds_validity_reuses_same_polygon_context() {
    let point = p(1, 2, 3);
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let left_old_ref = p(0, 0, 0);
    let left_context = support_reference_cache_context_key(
        &left_old_ref,
        &[axis_plane_definition(&left_old_ref)],
        &[0],
        &polygons,
    );
    let right_old_ref = p(1, 0, 0);
    let right_context = support_reference_cache_context_key(
        &right_old_ref,
        &[axis_plane_definition(&right_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_reference_bounds_validity_with_context(
        &mut cache,
        Some(&left_context),
        &bounds,
        &point,
        |_point| {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_reference_bounds_validity_with_context(
        &mut cache,
        Some(&right_context),
        &bounds,
        &point,
        |_point| {
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
fn cached_support_surface_query_reuses_same_polygon_context() {
    let point = p(2, 1, 1);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let left_old_ref = p(0, 0, 0);
    let left_context = support_reference_cache_context_key(
        &left_old_ref,
        &[axis_plane_definition(&left_old_ref)],
        &[0],
        &polygons,
    );
    let right_old_ref = p(1, 0, 0);
    let right_context = support_reference_cache_context_key(
        &right_old_ref,
        &[axis_plane_definition(&right_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_surface_query_with_context(
        &mut cache,
        Some(&left_context),
        &point,
        |_point| {
            calls += 1;
            Ok(true)
        },
    )
    .unwrap();
    let second = cached_support_surface_query_with_context(
        &mut cache,
        Some(&right_context),
        &point,
        |_point| {
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
fn projected_reference_trace_helper_reuses_point_validity_and_full_target_trace() {
    use std::cell::Cell;

    let first = ReferenceTarget::axis_defined(p(1, 2, 3));
    let second = ReferenceTarget::axis_defined_fallback(p(1, 2, 3));
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();
    let validity_calls = Cell::new(0);
    let trace_calls = Cell::new(0);

    let first_result = trace_projected_reference_target_with_queries(
        &mut validity_cache,
        &mut trace_cache,
        None,
        &bounds,
        &first,
        |_point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(true)
        },
        |target| {
            trace_calls.set(trace_calls.get() + 1);
            Ok(Some(vec![if target.uncertified_definition_fallback {
                2
            } else {
                1
            }]))
        },
    )
    .unwrap();
    let second_result = trace_projected_reference_target_with_queries(
        &mut validity_cache,
        &mut trace_cache,
        None,
        &bounds,
        &second,
        |_point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(true)
        },
        |target| {
            trace_calls.set(trace_calls.get() + 1);
            Ok(Some(vec![if target.uncertified_definition_fallback {
                2
            } else {
                1
            }]))
        },
    )
    .unwrap();
    let third_result = trace_projected_reference_target_with_queries(
        &mut validity_cache,
        &mut trace_cache,
        None,
        &bounds,
        &first,
        |_point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(false)
        },
        |_target| {
            trace_calls.set(trace_calls.get() + 1);
            Ok(Some(vec![99]))
        },
    )
    .unwrap();

    assert_eq!(validity_calls.get(), 1);
    assert_eq!(trace_calls.get(), 1);
    assert_eq!(first_result, Some(vec![1]));
    assert_eq!(second_result, Some(vec![1]));
    assert_eq!(third_result, Some(vec![1]));
}

#[test]
fn projected_reference_trace_helper_distinguishes_reference_context() {
    use std::cell::Cell;

    let target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let left_old_ref = p(0, 0, 0);
    let left_context = support_reference_cache_context_key(
        &left_old_ref,
        &[axis_plane_definition(&left_old_ref)],
        &[0],
        &polygons,
    );
    let right_old_ref = p(1, 0, 0);
    let right_context = support_reference_cache_context_key(
        &right_old_ref,
        &[axis_plane_definition(&right_old_ref)],
        &[0],
        &polygons,
    );
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();
    let validity_calls = Cell::new(0);
    let trace_calls = Cell::new(0);

    let first_result = trace_projected_reference_target_with_queries(
        &mut validity_cache,
        &mut trace_cache,
        Some(&left_context),
        &bounds,
        &target,
        |_point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(true)
        },
        |_target| {
            trace_calls.set(trace_calls.get() + 1);
            Ok(Some(vec![7]))
        },
    )
    .unwrap();
    let second_result = trace_projected_reference_target_with_queries(
        &mut validity_cache,
        &mut trace_cache,
        Some(&right_context),
        &bounds,
        &target,
        |_point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(true)
        },
        |_target| {
            trace_calls.set(trace_calls.get() + 1);
            Ok(Some(vec![9]))
        },
    )
    .unwrap();

    assert_eq!(validity_calls.get(), 1);
    assert_eq!(trace_calls.get(), 2);
    assert_eq!(first_result, Some(vec![7]));
    assert_eq!(second_result, Some(vec![9]));
}

#[test]
fn cached_reference_target_trace_reuses_certified_and_fallback_duplicates() {
    use std::cell::Cell;

    let point = p(1, 2, 3);
    let target = ReferenceTarget::axis_defined(point.clone());
    let fallback = ReferenceTarget::axis_defined_fallback(point);
    let mut trace_cache = Vec::new();
    let calls = Cell::new(0);

    let first = cached_reference_target_trace_with(&mut trace_cache, &fallback, |_target| {
        calls.set(calls.get() + 1);
        Ok(Some(vec![7]))
    })
    .unwrap();
    let second = cached_reference_target_trace_with(&mut trace_cache, &target, |_target| {
        calls.set(calls.get() + 1);
        Ok(Some(vec![9]))
    })
    .unwrap();

    assert_eq!(first, Some(vec![7]));
    assert_eq!(second, Some(vec![7]));
    assert_eq!(calls.get(), 1);
}

#[test]
fn cached_reference_target_trace_reuses_permuted_definition_families() {
    use std::cell::Cell;

    let point = p(1, 2, 3);
    let definition = axis_defs(&point)[0].clone();
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let first = ReferenceTarget::with_definitions(point.clone(), vec![definition.clone()]);
    let second = ReferenceTarget::with_definitions(point, vec![permuted]);
    let mut trace_cache = Vec::new();
    let calls = Cell::new(0);

    let first_result = cached_reference_target_trace_with(&mut trace_cache, &first, |_target| {
        calls.set(calls.get() + 1);
        Ok(Some(vec![7]))
    })
    .unwrap();
    let second_result = cached_reference_target_trace_with(&mut trace_cache, &second, |_target| {
        calls.set(calls.get() + 1);
        Ok(Some(vec![9]))
    })
    .unwrap();

    assert_eq!(first_result, Some(vec![7]));
    assert_eq!(second_result, Some(vec![7]));
    assert_eq!(calls.get(), 1);
}

#[test]
fn push_unique_reference_target_merges_permuted_definitions() {
    let point = p(1, 2, 3);
    let definition = axis_defs(&point)[0].clone();
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let mut targets = vec![ReferenceTarget::with_definitions(
        point.clone(),
        vec![definition.clone()],
    )];

    push_unique_reference_target(
        &mut targets,
        ReferenceTarget::with_definitions(point, vec![permuted.clone()]),
    );

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].definitions.len(), 1);
    assert!(reference_definition_planes_match_as_sets(
        &targets[0].definitions[0],
        &permuted
    ));
}

#[test]
fn push_unique_reference_target_prefers_certified_duplicate_definitions() {
    let point = p(1, 2, 3);
    let definition = axis_plane_definition(&point);
    let mut targets = vec![ReferenceTarget {
        point: point.clone(),
        definitions: vec![definition.clone()].into(),
        uncertified_definition_fallback: true,
    }];

    push_unique_reference_target(
        &mut targets,
        ReferenceTarget::with_definitions(point, vec![definition]),
    );

    assert_eq!(targets.len(), 1);
    assert!(!targets[0].uncertified_definition_fallback);
}

#[test]
fn push_verified_definition_merges_permuted_definitions() {
    let witness = p(1, 2, 3);
    let definition = axis_defs(&witness)[0].clone();
    let permuted = [
        definition[1].clone(),
        definition[2].clone(),
        definition[0].clone(),
    ];
    let mut definitions = vec![definition.clone()];

    let mut saw_unknown = false;
    push_verified_definition(
        &mut definitions,
        permuted.clone(),
        &witness,
        &mut saw_unknown,
    )
    .unwrap();

    assert_eq!(definitions.len(), 1);
    assert!(!saw_unknown);
    assert!(reference_definition_planes_match_as_sets(
        &definitions[0],
        &permuted
    ));
}

#[test]
fn projected_and_support_reference_traces_share_validity_and_trace_caches() {
    use std::cell::Cell;

    let target = ReferenceTarget::axis_defined(p(1, 2, 3));
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let validity_calls = Cell::new(0);
    let trace_calls = Cell::new(0);
    let mut query_caches = SupportReferenceQueryCaches::default();
    let mut surface_cache = Vec::new();

    let projected = {
        let SupportReferenceQueryCaches {
            validity_cache,
            trace_cache,
            ..
        } = &mut query_caches;
        trace_projected_reference_target_with_queries(
            validity_cache,
            trace_cache,
            None,
            &bounds,
            &target,
            |_point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(true)
            },
            |_target| {
                trace_calls.set(trace_calls.get() + 1);
                Ok(Some(vec![7]))
            },
        )
        .unwrap()
    };

    let support = {
        let SupportReferenceQueryCaches {
            validity_cache,
            trace_cache,
            ..
        } = &mut query_caches;
        trace_reference_targets_backtracking_unknown_with_query_caches(
            vec![target],
            &mut surface_cache,
            validity_cache,
            None,
            &bounds,
            &mut |_point| Ok(false),
            &mut |_point| {
                validity_calls.set(validity_calls.get() + 1);
                Ok(true)
            },
            |target| {
                cached_reference_target_trace_with(trace_cache, target, |_target| {
                    trace_calls.set(trace_calls.get() + 1);
                    Ok(Some(vec![99]))
                })
            },
        )
        .unwrap()
    };

    assert_eq!(projected, Some(vec![7]));
    assert_eq!(
        support,
        Some((ReferenceTarget::axis_defined(p(1, 2, 3)), vec![7]))
    );
    assert_eq!(validity_calls.get(), 1);
    assert_eq!(trace_calls.get(), 1);
}

#[test]
fn support_reference_target_trace_shortcut_skips_full_target_build_after_certified_report_witness()
{
    use std::cell::Cell;

    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 1, 1);
    let report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), [None, None, None]);
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();
    let build_calls = Cell::new(0);

    let found = trace_support_reference_targets_with_report_shortcut(
        &bounds,
        &halfspaces,
        Some(&report),
        &reference_witness_cache,
        &strict_contains_cache,
        &mut surface_cache,
        &mut validity_cache,
        None,
        &mut |_point| Ok(false),
        &mut |_point| Ok(true),
        || Ok((Vec::new(), false)),
        || {
            build_calls.set(build_calls.get() + 1);
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
        |_target| Ok(Some(vec![7])),
    )
    .unwrap()
    .expect("certified report witness should short-circuit support target search");

    assert_eq!(build_calls.get(), 0);
    assert_eq!(found.0.point, witness);
    assert_eq!(found.1, vec![7]);
}

#[test]
fn support_reference_target_trace_shortcut_falls_through_after_uncertified_report_witness() {
    use std::cell::Cell;

    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 1, 1);
    let report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), [None, None, None]);
    let later_target = ReferenceTarget::axis_defined(p(2, 2, 2));
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();
    let build_calls = Cell::new(0);

    let found = trace_support_reference_targets_with_report_shortcut(
        &bounds,
        &halfspaces,
        Some(&report),
        &reference_witness_cache,
        &strict_contains_cache,
        &mut surface_cache,
        &mut validity_cache,
        None,
        &mut |_point| Ok(false),
        &mut |_point| Ok(true),
        || Ok((Vec::new(), false)),
        || {
            build_calls.set(build_calls.get() + 1);
            Ok(vec![later_target.clone()])
        },
        |target| {
            if target.point == witness {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Some(vec![5]))
            }
        },
    )
    .unwrap()
    .expect("later certified support target should survive uncertified report witness");

    assert_eq!(build_calls.get(), 1);
    assert_eq!(found, (later_target, vec![5]));
}

#[test]
fn support_reference_target_trace_shortcut_skips_full_target_build_after_certified_direct_target() {
    use std::cell::Cell;

    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let direct_target = ReferenceTarget::axis_defined(p(2, 2, 2));
    let reference_witness_cache = std::cell::RefCell::new(Vec::new());
    let strict_contains_cache = std::cell::RefCell::new(Vec::new());
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();
    let build_calls = Cell::new(0);

    let found = trace_support_reference_targets_with_report_shortcut(
        &bounds,
        &halfspaces,
        None,
        &reference_witness_cache,
        &strict_contains_cache,
        &mut surface_cache,
        &mut validity_cache,
        None,
        &mut |_point| Ok(false),
        &mut |_point| Ok(true),
        || Ok((vec![direct_target.clone()], false)),
        || {
            build_calls.set(build_calls.get() + 1);
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
        |_target| Ok(Some(vec![11])),
    )
    .unwrap()
    .expect("certified direct support target should short-circuit full target build");

    assert_eq!(build_calls.get(), 0);
    assert_eq!(found, (direct_target, vec![11]));
}

#[test]
fn cached_support_target_family_reuses_identical_state_and_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_target_family_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
        },
    )
    .unwrap();
    let second = cached_support_target_family_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_cell_seed_families_reuse_identical_state_and_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;
    let expected = SupportCellSeedFamiliesState {
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: vec![p(1, 1, 1)],
        shifted_geometry_seeds: Vec::new(),
        saw_unknown: false,
    };

    let first = cached_support_cell_seed_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&report),
        || {
            calls += 1;
            Ok(expected.clone())
        },
    )
    .unwrap();
    let second = cached_support_cell_seed_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&report),
        || {
            calls += 1;
            Ok(SupportCellSeedFamiliesState {
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: true,
            })
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first.as_ref(), &expected);
    assert!(Arc::ptr_eq(&first, &second));
}

#[test]
fn cached_support_direct_reference_targets_reuse_identical_state_and_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;
    let expected = (vec![ReferenceTarget::axis_defined(p(1, 2, 3))], false);

    let first = cached_support_direct_reference_targets_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&report),
        || {
            calls += 1;
            Ok(expected.clone())
        },
    )
    .unwrap();
    let second = cached_support_direct_reference_targets_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&report),
        || {
            calls += 1;
            Ok((vec![ReferenceTarget::axis_defined(p(9, 9, 9))], true))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, expected);
    assert_eq!(first, second);
}

#[test]
fn cached_support_cell_seed_families_reuse_none_and_infeasible_reports() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let infeasible = hyperlimit::HalfspaceFeasibilityReport::infeasible(Some(
        hyperlimit::HalfspaceInfeasibilityCertificate {
            active_planes: [Some(0), Some(1), None, None],
            multipliers: [r(1), r(2), r(0), r(0)],
            offset_sum: r(3),
        },
    ));
    let mut cache = Vec::new();
    let mut calls = 0;
    let expected = SupportCellSeedFamiliesState {
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: vec![p(2, 2, 2)],
        shifted_geometry_seeds: vec![p(3, 3, 3)],
        saw_unknown: false,
    };

    let first =
        cached_support_cell_seed_families_with(&mut cache, &bounds, &halfspaces, None, || {
            calls += 1;
            Ok(expected.clone())
        })
        .unwrap();
    let second = cached_support_cell_seed_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&infeasible),
        || {
            calls += 1;
            Ok(SupportCellSeedFamiliesState {
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: true,
            })
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first.as_ref(), &expected);
    assert!(Arc::ptr_eq(&first, &second));
}

#[test]
fn cached_support_cell_seed_families_reuse_same_witness_different_active_planes() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 1, 1);
    let left =
        hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), [Some(0), None, None]);
    let right = hyperlimit::HalfspaceFeasibilityReport::feasible(witness, [Some(1), None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;
    let expected = SupportCellSeedFamiliesState {
        strict_seeds: vec![p(1, 1, 1)],
        shifted_vertices: vec![p(2, 2, 2)],
        shifted_geometry_seeds: vec![p(3, 3, 3)],
        saw_unknown: false,
    };

    let first = cached_support_cell_seed_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&left),
        || {
            calls += 1;
            Ok(expected.clone())
        },
    )
    .unwrap();
    let second = cached_support_cell_seed_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&right),
        || {
            calls += 1;
            Ok(SupportCellSeedFamiliesState {
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
                saw_unknown: true,
            })
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first.as_ref(), &expected);
    assert!(Arc::ptr_eq(&first, &second));
}

#[test]
fn cached_support_direct_reference_targets_reuse_none_and_infeasible_reports() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let infeasible = hyperlimit::HalfspaceFeasibilityReport::infeasible(None);
    let mut cache = Vec::new();
    let mut calls = 0;
    let expected = (vec![ReferenceTarget::axis_defined(p(1, 2, 3))], false);

    let first = cached_support_direct_reference_targets_with(
        &mut cache,
        &bounds,
        &halfspaces,
        None,
        || {
            calls += 1;
            Ok(expected.clone())
        },
    )
    .unwrap();
    let second = cached_support_direct_reference_targets_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&infeasible),
        || {
            calls += 1;
            Ok((vec![ReferenceTarget::axis_defined(p(9, 9, 9))], true))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, expected);
    assert_eq!(first, second);
}

#[test]
fn cached_support_direct_reference_targets_reuse_same_witness_different_active_planes() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let witness = p(1, 1, 1);
    let left =
        hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), [Some(0), None, None]);
    let right = hyperlimit::HalfspaceFeasibilityReport::feasible(witness, [Some(1), None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;
    let expected = (vec![ReferenceTarget::axis_defined(p(1, 2, 3))], false);

    let first = cached_support_direct_reference_targets_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&left),
        || {
            calls += 1;
            Ok(expected.clone())
        },
    )
    .unwrap();
    let second = cached_support_direct_reference_targets_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&right),
        || {
            calls += 1;
            Ok((vec![ReferenceTarget::axis_defined(p(9, 9, 9))], true))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, expected);
    assert_eq!(first, second);
}

#[test]
fn cached_support_target_family_reuses_permuted_state_and_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_target_family_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
        },
    )
    .unwrap();
    let second = cached_support_target_family_with(
        &mut cache,
        &bounds,
        &permuted,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_target_family_reuses_permuted_state_and_permuted_report_indices() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let witness = p(1, 1, 1);
    let left_active = [Some(0), Some(1), Some(2)];
    let right_active = left_active.map(|index| {
        index.map(|index| {
            permuted
                .iter()
                .position(|plane| plane == &halfspaces[index])
                .unwrap()
        })
    });
    let left_report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), left_active);
    let right_report = hyperlimit::HalfspaceFeasibilityReport::feasible(witness, right_active);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_target_family_with(
        &mut cache,
        &bounds,
        &halfspaces,
        Some(&left_report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(vec![ReferenceTarget::axis_defined(p(1, 2, 3))])
        },
    )
    .unwrap();
    let second = cached_support_target_family_with(
        &mut cache,
        &bounds,
        &permuted,
        Some(&right_report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(vec![ReferenceTarget::axis_defined(p(9, 9, 9))])
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_reference_accept_reuses_identical_state_and_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((
                ReferenceTarget::axis_defined(bounds.min.clone()),
                vec![23],
            )))
        },
    )
    .unwrap();
    let second = cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_reference_accept_reuses_permuted_state_and_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((
                ReferenceTarget::axis_defined(bounds.min.clone()),
                vec![23],
            )))
        },
    )
    .unwrap();
    let second = cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &permuted,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_reference_accept_memoizes_current_equivalent_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let mut cache = Vec::new();

    cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            Ok(Some((
                ReferenceTarget::axis_defined(bounds.min.clone()),
                vec![23],
            )))
        },
    )
    .unwrap();
    cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &permuted,
        Some(&report),
        |_halfspaces, _report| Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99]))),
    )
    .unwrap();

    assert_eq!(cache.len(), 2);
    assert!(cache.iter().any(|entry| {
        entry.context.is_none()
            && entry.bounds == bounds
            && entry.halfspaces == permuted
            && entry.report.as_ref() == Some(&report)
    }));
}

#[test]
fn cached_support_reference_accept_reuses_permuted_state_and_permuted_report_indices() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let witness = p(1, 1, 1);
    let left_active = [Some(0), Some(1), Some(2)];
    let right_active = left_active.map(|index| {
        index.map(|index| {
            permuted
                .iter()
                .position(|plane| plane == &halfspaces[index])
                .unwrap()
        })
    });
    let left_report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(witness.clone(), left_active);
    let right_report = hyperlimit::HalfspaceFeasibilityReport::feasible(witness, right_active);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &halfspaces,
        Some(&left_report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((
                ReferenceTarget::axis_defined(bounds.min.clone()),
                vec![23],
            )))
        },
    )
    .unwrap();
    let second = cached_support_reference_accept_with(
        &mut cache,
        None,
        &bounds,
        &permuted,
        Some(&right_report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_reference_accept_distinguishes_reference_context() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let left_old_ref = p(0, 0, 0);
    let left_context = support_reference_cache_context_key(
        &left_old_ref,
        &[axis_plane_definition(&left_old_ref)],
        &[0],
        &polygons,
    );
    let right_old_ref = p(1, 0, 0);
    let right_context = support_reference_cache_context_key(
        &right_old_ref,
        &[axis_plane_definition(&right_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_reference_accept_with(
        &mut cache,
        Some(&left_context),
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(0, 0, 0)), vec![23])))
        },
    )
    .unwrap();
    let second = cached_support_reference_accept_with(
        &mut cache,
        Some(&right_context),
        &bounds,
        &halfspaces,
        Some(&report),
        |_halfspaces, _report| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(1, 0, 0)), vec![24])))
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_ne!(first, second);
}

#[test]
fn reusable_support_reference_accept_if_certified_reuses_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_context = support_reference_cache_context_key(
        &query_old_ref,
        &[axis_plane_definition(&query_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = vec![SupportReferenceAcceptCacheEntry {
        context: Some(cached_context),
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        report: Some(report.clone()),
        accepted: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_support_reference_accept_if_certified(
        &mut cache,
        &query_context,
        &bounds,
        &halfspaces,
        Some(&report),
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))
    );
    assert_eq!(cache.len(), 2);
    assert!(cache.iter().any(|entry| {
        entry
            .context
            .as_ref()
            .is_some_and(|context| context.old_ref == query_old_ref)
            && matches!(
                &entry.accepted,
                Ok(Some((target, winding)))
                    if *target == ReferenceTarget::axis_defined(p(1, 1, 1))
                        && *winding == vec![23]
            )
    }));
}

#[test]
fn reusable_support_reference_accept_if_certified_reuses_cached_target_across_tighter_bounds() {
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let halfspaces = aabb_core_halfspaces(&cached_bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_context = support_reference_cache_context_key(
        &query_old_ref,
        &[axis_plane_definition(&query_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = vec![SupportReferenceAcceptCacheEntry {
        context: Some(cached_context),
        bounds: cached_bounds,
        halfspaces: halfspaces.clone(),
        report: Some(report.clone()),
        accepted: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_support_reference_accept_if_certified(
        &mut cache,
        &query_context,
        &query_bounds,
        &halfspaces,
        Some(&report),
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))
    );
    assert_eq!(cache.len(), 2);
}

#[test]
fn reusable_support_reference_accept_if_certified_skips_invalid_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_context = support_reference_cache_context_key(
        &query_old_ref,
        &[axis_plane_definition(&query_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = vec![SupportReferenceAcceptCacheEntry {
        context: Some(cached_context),
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        report: Some(report.clone()),
        accepted: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_support_reference_accept_if_certified(
        &mut cache,
        &query_context,
        &bounds,
        &halfspaces,
        Some(&report),
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.len(), 1);
}

#[test]
fn reusable_support_reference_accept_from_cached_trace_if_certified_reuses_cached_target_across_parent_winding()
 {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let polygons = Vec::new();
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let query_context =
        support_reference_cache_context_key(&query_old_ref, &query_definitions, &[7], &polygons);
    let mut cache = vec![SupportReferenceAcceptCacheEntry {
        context: Some(cached_context),
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        report: Some(report.clone()),
        accepted: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_support_reference_accept_from_cached_trace_if_certified(
        &mut cache,
        &query_context,
        &bounds,
        &halfspaces,
        Some(&report),
        &mut validity_cache,
        &mut trace_cache,
        &query_old_ref,
        &query_definitions,
        &[7],
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![7]))
    );
    assert_eq!(cache.len(), 2);
    assert!(cache.iter().any(|entry| {
        entry.context.as_ref().is_some_and(|context| {
            context.old_ref == query_old_ref && context.old_wnv.as_slice() == [7]
        }) && matches!(
            &entry.accepted,
            Ok(Some((target, winding)))
                if *target == ReferenceTarget::axis_defined(p(1, 1, 1))
                    && *winding == vec![7]
        )
    }));
}

#[test]
fn reusable_support_reference_accept_from_cached_trace_if_certified_skips_invalid_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report = hyperlimit::HalfspaceFeasibilityReport::feasible(p(1, 1, 1), [None, None, None]);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let query_context =
        support_reference_cache_context_key(&query_old_ref, &query_definitions, &[7], &polygons);
    let mut cache = vec![SupportReferenceAcceptCacheEntry {
        context: Some(cached_context),
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        report: Some(report.clone()),
        accepted: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_support_reference_accept_from_cached_trace_if_certified(
        &mut cache,
        &query_context,
        &bounds,
        &halfspaces,
        Some(&report),
        &mut validity_cache,
        &mut trace_cache,
        &query_old_ref,
        &query_definitions,
        &[7],
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.len(), 1);
}

#[test]
fn cached_support_reference_result_reuses_identical_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let old_ref = p(0, 0, 0);
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_support_reference_result_with(&mut cache, &context, &bounds, &halfspaces, || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23])))
        })
        .unwrap();
    let second =
        cached_support_reference_result_with(&mut cache, &context, &bounds, &halfspaces, || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(3, 3, 3)), vec![24])))
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_reference_result_reuses_permuted_halfspaces() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let old_ref = p(0, 0, 0);
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_support_reference_result_with(&mut cache, &context, &bounds, &halfspaces, || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23])))
        })
        .unwrap();
    let second =
        cached_support_reference_result_with(&mut cache, &context, &bounds, &permuted, || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(3, 3, 3)), vec![24])))
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_reference_result_memoizes_current_equivalent_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let old_ref = p(0, 0, 0);
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();

    cached_support_reference_result_with(&mut cache, &context, &bounds, &halfspaces, || {
        Ok(Some((
            ReferenceTarget::axis_defined(bounds.min.clone()),
            vec![23],
        )))
    })
    .unwrap();
    cached_support_reference_result_with(&mut cache, &context, &bounds, &permuted, || {
        Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
    })
    .unwrap();

    assert_eq!(cache.len(), 2);
    assert!(cache.iter().any(|entry| entry.context == context
        && entry.bounds == bounds
        && entry.halfspaces == permuted));
}

#[test]
fn cached_support_reference_result_distinguishes_reference_context() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let left_old_ref = p(0, 0, 0);
    let left_context = support_reference_cache_context_key(
        &left_old_ref,
        &[axis_plane_definition(&left_old_ref)],
        &[0],
        &polygons,
    );
    let right_old_ref = p(1, 0, 0);
    let right_context = support_reference_cache_context_key(
        &right_old_ref,
        &[axis_plane_definition(&right_old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_support_reference_result_with(
        &mut cache,
        &left_context,
        &bounds,
        &halfspaces,
        || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23])))
        },
    )
    .unwrap();
    let second = cached_support_reference_result_with(
        &mut cache,
        &right_context,
        &bounds,
        &halfspaces,
        || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(3, 3, 3)), vec![24])))
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_ne!(first, second);
}

#[test]
fn cached_projected_reference_result_reuses_identical_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let old_ref = p(0, 2, 5);
    let halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
    let polygons = Vec::new();
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_projected_reference_result_with(&mut cache, &context, &bounds, &halfspaces, || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23])))
        })
        .unwrap();
    let second =
        cached_projected_reference_result_with(&mut cache, &context, &bounds, &halfspaces, || {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(3, 3, 3)), vec![24])))
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn reusable_projected_reference_result_if_certified_reuses_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_old_ref = p(0, 2, 5);
    let halfspaces = projected_reference_halfspaces(&cached_old_ref, &bounds).unwrap();
    let polygons = Vec::new();
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(0, 1, 5);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![ProjectedReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_projected_reference_result_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[0],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))
    );
    assert_eq!(cache.len(), 2);
}

#[test]
fn reusable_projected_reference_result_if_certified_reuses_cached_target_across_tighter_bounds() {
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let cached_old_ref = p(0, 2, 5);
    let halfspaces = projected_reference_halfspaces(&cached_old_ref, &cached_bounds).unwrap();
    let polygons = Vec::new();
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(0, 1, 5);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![ProjectedReferenceResultCacheEntry {
        context: cached_context,
        bounds: cached_bounds,
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_projected_reference_result_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[0],
        &query_bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))
    );
    assert_eq!(cache.len(), 2);
}

#[test]
fn reusable_projected_reference_result_if_certified_skips_invalid_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_old_ref = p(0, 2, 5);
    let halfspaces = projected_reference_halfspaces(&cached_old_ref, &bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(0, 1, 5);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![ProjectedReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_projected_reference_result_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[0],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.len(), 1);
}

#[test]
fn reusable_projected_reference_result_from_cached_trace_if_certified_reuses_cached_target_across_parent_winding()
 {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let old_ref = p(0, 2, 5);
    let query_definitions = vec![axis_plane_definition(&old_ref)];
    let halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
    let polygons = Vec::new();
    let cached_context =
        support_reference_cache_context_key(&old_ref, &query_definitions, &[0], &polygons);
    let mut cache = vec![ProjectedReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_projected_reference_result_from_cached_trace_if_certified(
        &mut cache,
        &old_ref,
        &query_definitions,
        &[7],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
        &mut trace_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![7]))
    );
    assert_eq!(cache.len(), 2);
}

#[test]
fn reusable_projected_reference_result_from_cached_trace_if_certified_reuses_cached_target_across_tighter_bounds()
 {
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let old_ref = p(0, 2, 5);
    let query_definitions = vec![axis_plane_definition(&old_ref)];
    let halfspaces = projected_reference_halfspaces(&old_ref, &cached_bounds).unwrap();
    let polygons = Vec::new();
    let cached_context =
        support_reference_cache_context_key(&old_ref, &query_definitions, &[0], &polygons);
    let mut cache = vec![ProjectedReferenceResultCacheEntry {
        context: cached_context,
        bounds: cached_bounds,
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_projected_reference_result_from_cached_trace_if_certified(
        &mut cache,
        &old_ref,
        &query_definitions,
        &[7],
        &query_bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
        &mut trace_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![7]))
    );
    assert_eq!(cache.len(), 2);
}

#[test]
fn reusable_projected_reference_result_from_cached_trace_if_certified_skips_invalid_cached_target()
{
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let old_ref = p(0, 2, 5);
    let query_definitions = vec![axis_plane_definition(&old_ref)];
    let halfspaces = projected_reference_halfspaces(&old_ref, &bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_context =
        support_reference_cache_context_key(&old_ref, &query_definitions, &[0], &polygons);
    let mut cache = vec![ProjectedReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_projected_reference_result_from_cached_trace_if_certified(
        &mut cache,
        &old_ref,
        &query_definitions,
        &[7],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
        &mut trace_cache,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.len(), 1);
}

#[test]
fn reusable_support_reference_result_if_certified_reuses_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![SupportReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_support_reference_result_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[0],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))
    );
    assert_eq!(cache.len(), 2);
    assert!(cache.iter().any(|entry| {
        entry.context.old_ref == query_old_ref
            && matches!(
                &entry.result,
                Ok(Some((target, winding)))
                    if *target == ReferenceTarget::axis_defined(p(1, 1, 1))
                        && *winding == vec![23]
            )
    }));
}

#[test]
fn reusable_support_reference_result_if_certified_reuses_cached_target_across_tighter_bounds() {
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let halfspaces = aabb_core_halfspaces(&cached_bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![SupportReferenceResultCacheEntry {
        context: cached_context,
        bounds: cached_bounds,
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_support_reference_result_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[0],
        &query_bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))
    );
    assert_eq!(cache.len(), 2);
}

#[test]
fn reusable_support_reference_result_if_certified_skips_invalid_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![SupportReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();

    let reused = reusable_support_reference_result_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[0],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.len(), 1);
}

#[test]
fn reusable_support_reference_result_from_cached_trace_if_certified_reuses_cached_target_across_parent_winding()
 {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = Vec::new();
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![SupportReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_support_reference_result_from_cached_trace_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[7],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
        &mut trace_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![7]))
    );
    assert_eq!(cache.len(), 2);
    assert!(cache.iter().any(|entry| {
        entry.context.old_ref == query_old_ref
            && entry.context.old_wnv.as_slice() == [7]
            && matches!(
                &entry.result,
                Ok(Some((target, winding)))
                    if *target == ReferenceTarget::axis_defined(p(1, 1, 1))
                        && *winding == vec![7]
            )
    }));
}

#[test]
fn reusable_support_reference_result_from_cached_trace_if_certified_skips_invalid_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let mut cache = vec![SupportReferenceResultCacheEntry {
        context: cached_context,
        bounds: bounds.clone(),
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }];
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_support_reference_result_from_cached_trace_if_certified(
        &mut cache,
        &query_old_ref,
        &query_definitions,
        &[7],
        &bounds,
        &polygons,
        &halfspaces,
        &mut validity_cache,
        &mut trace_cache,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.len(), 1);
}

#[test]
fn cached_support_plane_cell_search_reuses_identical_state_and_index() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let cache = std::cell::RefCell::new(Vec::new());
    let mut calls = 0;

    let first = cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        halfspaces.clone(),
        || {
            calls += 1;
            Ok(Some(17))
        },
    )
    .unwrap();
    let second = cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        halfspaces,
        || {
            calls += 1;
            Ok(Some(99))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_plane_cell_search_reuses_same_preferred_order() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let cache = std::cell::RefCell::new(Vec::new());
    let mut calls = 0;
    let support = Plane::axis_aligned(0, r(2));
    let first_order = support_side_search_order(Some(&p(1, 1, 1)), &support);
    let second_order = support_side_search_order(Some(&p(1, 3, 3)), &support);

    assert_eq!(first_order, [false, true]);
    assert_eq!(first_order, second_order);

    let first = cached_support_plane_cell_search_with(
        &cache,
        None,
        first_order,
        &bounds,
        3,
        halfspaces.clone(),
        || {
            calls += 1;
            Ok(Some(17))
        },
    )
    .unwrap();
    let second = cached_support_plane_cell_search_with(
        &cache,
        None,
        second_order,
        &bounds,
        3,
        halfspaces,
        || {
            calls += 1;
            Ok(Some(99))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_plane_cell_search_reuses_opposite_preferred_order() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let cache = std::cell::RefCell::new(Vec::new());
    let mut calls = 0;

    let first = cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        halfspaces.clone(),
        || {
            calls += 1;
            Ok(Some(17))
        },
    )
    .unwrap();
    let second = cached_support_plane_cell_search_with(
        &cache,
        None,
        [true, false],
        &bounds,
        3,
        halfspaces,
        || {
            calls += 1;
            Ok(Some(99))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, Some(17));
    assert_eq!(second, Some(17));
}

#[test]
fn optional_halfspace_reports_match_permuted_infeasibility_certificates() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let left_active = [Some(0), Some(1), None, None];
    let right_active = left_active.map(|index| {
        index.map(|index| {
            permuted
                .iter()
                .position(|plane| plane == &halfspaces[index])
                .unwrap()
        })
    });
    let left = hyperlimit::HalfspaceFeasibilityReport::infeasible(Some(
        hyperlimit::HalfspaceInfeasibilityCertificate {
            active_planes: left_active,
            multipliers: [r(1), r(2), r(0), r(0)],
            offset_sum: r(3),
        },
    ));
    let right = hyperlimit::HalfspaceFeasibilityReport::infeasible(Some(
        hyperlimit::HalfspaceInfeasibilityCertificate {
            active_planes: right_active,
            multipliers: [r(1), r(2), r(0), r(0)],
            offset_sum: r(3),
        },
    ));

    assert!(optional_halfspace_reports_match_for_cache(
        &halfspaces,
        Some(&left),
        &permuted,
        Some(&right),
    ));
}

#[test]
fn cached_support_plane_cell_search_reuses_permuted_state_and_index() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let cache = std::cell::RefCell::new(Vec::new());
    let mut calls = 0;

    let first = cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        halfspaces,
        || {
            calls += 1;
            Ok(Some(17))
        },
    )
    .unwrap();
    let second = cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        permuted,
        || {
            calls += 1;
            Ok(Some(99))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_support_plane_cell_search_memoizes_current_equivalent_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let cache = std::cell::RefCell::new(Vec::<SupportPlaneCellSearchCacheEntry<i32>>::new());

    cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        halfspaces,
        || Ok(Some(17)),
    )
    .unwrap();
    cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        permuted.clone(),
        || Ok(Some(99)),
    )
    .unwrap();

    let cache = cache.borrow();
    assert_eq!(cache.len(), 2);
    assert!(cache.iter().any(|entry| {
        entry.context.is_none()
            && entry.bounds == bounds
            && entry.polygon_index == 3
            && entry.halfspaces == permuted
    }));
}

#[test]
fn cached_support_plane_cell_search_prefers_newest_exact_alias_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let cache = std::cell::RefCell::new(vec![
        SupportPlaneCellSearchCacheEntry {
            context: None,
            bounds: bounds.clone(),
            polygon_index: 3,
            halfspaces: permuted,
            result: Ok(Some(17)),
        },
        SupportPlaneCellSearchCacheEntry {
            context: None,
            bounds: bounds.clone(),
            polygon_index: 3,
            halfspaces: halfspaces.clone(),
            result: Ok(Some(29)),
        },
    ]);

    let result = cached_support_plane_cell_search_with(
        &cache,
        None,
        [false, true],
        &bounds,
        3,
        halfspaces,
        || unreachable!(),
    )
    .unwrap();

    assert_eq!(result, Some(29));
}

#[test]
fn support_plane_cell_search_cache_reuses_same_normalized_polygon_index() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
    let polygons = vec![polygon.clone()];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    halfspaces.push(support_side_halfspace(&polygon.support, false));
    let cache =
        std::cell::RefCell::new(Vec::<SupportPlaneCellSearchCacheEntry<ReferenceTarget>>::new());
    let mut report_calls = 0;
    let mut accept_calls = 0;

    let first = support_plane_cell_search_with_queries_cached(
        None,
        Some(&p(0, 0, 0)),
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |_halfspaces| {
            report_calls += 1;
            Ok(None)
        },
        &mut |_halfspaces| Ok(true),
        &mut |_halfspaces, _report| {
            accept_calls += 1;
            Ok(None)
        },
        &cache,
    )
    .unwrap();
    let second = support_plane_cell_search_with_queries_cached(
        None,
        Some(&p(9, 9, 9)),
        &bounds,
        &polygons,
        1,
        &mut halfspaces,
        &mut |_halfspaces| {
            report_calls += 1;
            Ok(None)
        },
        &mut |_halfspaces| Ok(true),
        &mut |_halfspaces, _report| {
            accept_calls += 1;
            Ok(None)
        },
        &cache,
    )
    .unwrap();

    assert_eq!(first, None);
    assert_eq!(second, None);
    assert_eq!(report_calls, 1);
    assert_eq!(accept_calls, 1);
}

#[test]
fn cached_support_plane_cell_search_distinguishes_reference_context() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let left_old_ref = p(0, 0, 0);
    let left_context = support_reference_cache_context_key(
        &left_old_ref,
        &[axis_plane_definition(&left_old_ref)],
        &[0],
        &polygons,
    );
    let right_old_ref = p(1, 0, 0);
    let right_context = support_reference_cache_context_key(
        &right_old_ref,
        &[axis_plane_definition(&right_old_ref)],
        &[0],
        &polygons,
    );
    let cache =
        std::cell::RefCell::new(Vec::<SupportPlaneCellSearchCacheEntry<ReferenceTarget>>::new());
    let mut calls = 0;

    let first = cached_support_plane_cell_search_with(
        &cache,
        Some(&left_context),
        [false, true],
        &bounds,
        0,
        halfspaces.clone(),
        || {
            calls += 1;
            Ok(Some(ReferenceTarget::axis_defined(p(0, 0, 0))))
        },
    )
    .unwrap();
    let second = cached_support_plane_cell_search_with(
        &cache,
        Some(&right_context),
        [false, true],
        &bounds,
        0,
        halfspaces,
        || {
            calls += 1;
            Ok(Some(ReferenceTarget::axis_defined(p(1, 0, 0))))
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_ne!(first, second);
}

#[test]
fn reusable_support_plane_cell_search_result_if_certified_reuses_cached_target_across_tighter_bounds()
 {
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let halfspaces = aabb_core_halfspaces(&cached_bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let old_ref = p(0, 0, 0);
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let cache = std::cell::RefCell::new(vec![SupportPlaneCellSearchCacheEntry {
        context: Some(context.clone()),
        bounds: cached_bounds,
        polygon_index: 0,
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }]);
    let mut validity_cache = Vec::new();

    let reused = reusable_support_plane_cell_search_result_if_certified(
        &cache,
        &context,
        &query_bounds,
        0,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))
    );
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_support_plane_cell_search_result_if_certified_skips_invalid_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let old_ref = p(0, 0, 0);
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let cache = std::cell::RefCell::new(vec![SupportPlaneCellSearchCacheEntry {
        context: Some(context.clone()),
        bounds: bounds.clone(),
        polygon_index: 0,
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }]);
    let mut validity_cache = Vec::new();

    let reused = reusable_support_plane_cell_search_result_if_certified(
        &cache,
        &context,
        &bounds,
        0,
        &halfspaces,
        &mut validity_cache,
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn reusable_support_plane_cell_search_result_from_cached_trace_if_certified_reuses_cached_target_across_parent_winding()
 {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = Vec::new();
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let query_context =
        support_reference_cache_context_key(&query_old_ref, &query_definitions, &[7], &polygons);
    let cache = std::cell::RefCell::new(vec![SupportPlaneCellSearchCacheEntry {
        context: Some(cached_context),
        bounds: bounds.clone(),
        polygon_index: 0,
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![23]))),
    }]);
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_support_plane_cell_search_result_from_cached_trace_if_certified(
        &cache,
        &query_context,
        &bounds,
        0,
        &halfspaces,
        &mut validity_cache,
        &mut trace_cache,
        &query_old_ref,
        &query_definitions,
        &[7],
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![7]))
    );
    assert_eq!(cache.borrow().len(), 2);
}

#[test]
fn reusable_support_plane_cell_search_result_from_cached_trace_if_certified_skips_invalid_cached_target()
 {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_old_ref = p(0, 0, 0);
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_old_ref = p(1, 0, 0);
    let query_definitions = vec![axis_plane_definition(&query_old_ref)];
    let query_context =
        support_reference_cache_context_key(&query_old_ref, &query_definitions, &[7], &polygons);
    let cache = std::cell::RefCell::new(vec![SupportPlaneCellSearchCacheEntry {
        context: Some(cached_context),
        bounds: bounds.clone(),
        polygon_index: 0,
        halfspaces: halfspaces.clone(),
        result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![23]))),
    }]);
    let mut validity_cache = Vec::new();
    let mut trace_cache = Vec::new();

    let reused = reusable_support_plane_cell_search_result_from_cached_trace_if_certified(
        &cache,
        &query_context,
        &bounds,
        0,
        &halfspaces,
        &mut validity_cache,
        &mut trace_cache,
        &query_old_ref,
        &query_definitions,
        &[7],
    )
    .unwrap();

    assert_eq!(reused, None);
    assert_eq!(cache.borrow().len(), 1);
}

#[test]
fn cached_shifted_projected_cell_families_reuse_identical_seed() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let seed = p(1, 2, 3);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_shifted_projected_cell_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        &seed,
        || {
            calls += 1;
            Ok(None)
        },
    )
    .unwrap();
    let second = cached_shifted_projected_cell_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        &seed,
        || {
            calls += 1;
            Ok(Some(ShiftedProjectedCellFamilies {
                shifted: Vec::new(),
                report: None,
                saw_unknown: false,
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
            }))
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_shifted_projected_cell_families_reuse_permuted_halfspace_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let seed = p(1, 2, 3);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_shifted_projected_cell_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        &seed,
        || {
            calls += 1;
            Ok(None)
        },
    )
    .unwrap();
    let second =
        cached_shifted_projected_cell_families_with(&mut cache, &bounds, &permuted, &seed, || {
            calls += 1;
            Ok(Some(ShiftedProjectedCellFamilies {
                shifted: Vec::new(),
                report: None,
                saw_unknown: false,
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
            }))
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_shifted_projected_cell_families_distinguish_halfspace_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let alternate_halfspaces = {
        let mut alternate = halfspaces.clone();
        alternate.push(support_side_halfspace(&Plane::axis_aligned(0, r(2)), false));
        alternate
    };
    let seed = p(1, 2, 3);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first = cached_shifted_projected_cell_families_with(
        &mut cache,
        &bounds,
        &halfspaces,
        &seed,
        || {
            calls += 1;
            Ok(None)
        },
    )
    .unwrap();
    let second = cached_shifted_projected_cell_families_with(
        &mut cache,
        &bounds,
        &alternate_halfspaces,
        &seed,
        || {
            calls += 1;
            Ok(Some(ShiftedProjectedCellFamilies {
                shifted: Vec::new(),
                report: None,
                saw_unknown: false,
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
            }))
        },
    )
    .unwrap();

    assert_eq!(calls, 2);
    assert_eq!(first, None);
    assert!(second.is_some());
}

#[test]
fn cached_shifted_support_cell_families_reuse_identical_seed_and_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let seed = p(1, 2, 3);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_shifted_support_cell_families_with(&mut cache, &bounds, &halfspaces, &seed, || {
            calls += 1;
            Ok(None)
        })
        .unwrap();
    let second =
        cached_shifted_support_cell_families_with(&mut cache, &bounds, &halfspaces, &seed, || {
            calls += 1;
            Ok(Some(ShiftedSupportCellFamilies {
                shifted: Vec::new(),
                report: None,
                saw_unknown: false,
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
            }))
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_shifted_support_cell_families_reuse_permuted_halfspace_state() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut permuted = halfspaces.clone();
    permuted.rotate_left(1);
    let seed = p(1, 2, 3);
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_shifted_support_cell_families_with(&mut cache, &bounds, &halfspaces, &seed, || {
            calls += 1;
            Ok(None)
        })
        .unwrap();
    let second =
        cached_shifted_support_cell_families_with(&mut cache, &bounds, &permuted, &seed, || {
            calls += 1;
            Ok(Some(ShiftedSupportCellFamilies {
                shifted: Vec::new(),
                report: None,
                saw_unknown: false,
                strict_seeds: vec![p(9, 9, 9)],
                shifted_vertices: Vec::new(),
                shifted_geometry_seeds: Vec::new(),
            }))
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_reference_escape_search_reuses_identical_escape_bounds() {
    let bounds = Aabb::new(p(1, 2, 3), p(4, 5, 6));
    let old_ref = p(0, 0, 0);
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &[support_only_polygon(Plane::axis_aligned(0, r(2)))],
    );
    let mut cache = Vec::new();
    let mut calls = 0;

    let first =
        cached_reference_escape_search_with(&mut cache, &context, &bounds, |escape_bounds| {
            calls += 1;
            Ok(Some((
                ReferenceTarget::axis_defined(escape_bounds.min.clone()),
                vec![11],
            )))
        })
        .unwrap();
    let second =
        cached_reference_escape_search_with(&mut cache, &context, &bounds, |_escape_bounds| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
        })
        .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(first, second);
}

#[test]
fn cached_reference_escape_search_memoizes_current_equivalent_state() {
    let bounds = Aabb::new(p(1, 2, 3), p(4, 5, 6));
    let old_ref = p(0, 0, 0);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let definition_a = axis_plane_definition(&old_ref);
    let definition_b = axis_plane_definition(&p(1, 0, 0));
    let context = support_reference_cache_context_key(
        &old_ref,
        &[definition_a.clone(), definition_b.clone()],
        &[0],
        &polygons,
    );
    let permuted_context = support_reference_cache_context_key(
        &old_ref,
        &[definition_b, definition_a],
        &[0],
        &polygons,
    );
    let mut cache = Vec::new();

    cached_reference_escape_search_with(&mut cache, &context, &bounds, |escape_bounds| {
        Ok(Some((
            ReferenceTarget::axis_defined(escape_bounds.min.clone()),
            vec![11],
        )))
    })
    .unwrap();
    cached_reference_escape_search_with(&mut cache, &permuted_context, &bounds, |_escape_bounds| {
        Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
    })
    .unwrap();

    assert_eq!(cache.len(), 2);
    assert!(
        cache
            .iter()
            .any(|entry| entry.context == permuted_context && entry.bounds == bounds)
    );
}

#[test]
fn cached_reference_escape_search_prefers_newest_exact_alias_state() {
    let bounds = Aabb::new(p(1, 2, 3), p(4, 5, 6));
    let old_ref = p(0, 0, 0);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let definition_a = axis_plane_definition(&old_ref);
    let definition_b = axis_plane_definition(&p(1, 0, 0));
    let exact_context = support_reference_cache_context_key(
        &old_ref,
        &[definition_a.clone(), definition_b.clone()],
        &[0],
        &polygons,
    );
    let equivalent_context = support_reference_cache_context_key(
        &old_ref,
        &[definition_b, definition_a],
        &[0],
        &polygons,
    );
    let mut cache = vec![
        ProjectionEscapeSearchCacheEntry {
            context: equivalent_context,
            bounds: bounds.clone(),
            result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![11]))),
        },
        ProjectionEscapeSearchCacheEntry {
            context: exact_context.clone(),
            bounds: bounds.clone(),
            result: Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![13]))),
        },
    ];

    let result = cached_reference_escape_search_with(
        &mut cache,
        &exact_context,
        &bounds,
        |_| unreachable!(),
    )
    .unwrap();

    assert_eq!(
        result,
        Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![13]))
    );
}

#[test]
fn cached_reference_escape_search_in_query_caches_reuses_cached_target_across_tighter_bounds() {
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));
    let old_ref = p(0, 0, 0);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let mut query_caches = SupportReferenceQueryCaches::default();
    query_caches
        .projection_escape_search_cache
        .borrow_mut()
        .push(ProjectionEscapeSearchCacheEntry {
            context: context.clone(),
            bounds: cached_bounds,
            result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![11]))),
        });
    let mut calls = 0;

    let reused = cached_reference_escape_search_in_query_caches(
        &mut query_caches,
        &context,
        &query_bounds,
        |_escape_bounds, _query_caches| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![11]))
    );
    assert_eq!(calls, 0);
    assert_eq!(
        query_caches.projection_escape_search_cache.borrow().len(),
        2
    );
}

#[test]
fn cached_reference_escape_search_in_query_caches_skips_invalid_cached_target() {
    let cached_bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let query_bounds = Aabb::new(p(0, 0, 0), p(1, 1, 1));
    let old_ref = p(0, 0, 0);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let context = support_reference_cache_context_key(
        &old_ref,
        &[axis_plane_definition(&old_ref)],
        &[0],
        &polygons,
    );
    let mut query_caches = SupportReferenceQueryCaches::default();
    query_caches
        .projection_escape_search_cache
        .borrow_mut()
        .push(ProjectionEscapeSearchCacheEntry {
            context: context.clone(),
            bounds: cached_bounds,
            result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![11]))),
        });
    let mut calls = 0;

    let reused = cached_reference_escape_search_in_query_caches(
        &mut query_caches,
        &context,
        &query_bounds,
        |_escape_bounds, _query_caches| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![99]))
    );
    assert_eq!(calls, 1);
}

#[test]
fn cached_reference_escape_search_in_query_caches_reuses_cached_target_across_parent_winding() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_old_ref = p(0, 0, 0);
    let query_old_ref = p(1, 0, 0);
    let polygons = Vec::new();
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_context = support_reference_cache_context_key(
        &query_old_ref,
        &[axis_plane_definition(&query_old_ref)],
        &[7],
        &polygons,
    );
    let mut query_caches = SupportReferenceQueryCaches::default();
    query_caches
        .projection_escape_search_cache
        .borrow_mut()
        .push(ProjectionEscapeSearchCacheEntry {
            context: cached_context,
            bounds: bounds.clone(),
            result: Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![11]))),
        });
    let mut calls = 0;

    let reused = cached_reference_escape_search_in_query_caches(
        &mut query_caches,
        &query_context,
        &bounds,
        |_escape_bounds, _query_caches| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(9, 9, 9)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![7]))
    );
    assert_eq!(calls, 0);
    assert_eq!(
        query_caches.projection_escape_search_cache.borrow().len(),
        2
    );
}

#[test]
fn cached_reference_escape_search_in_query_caches_skips_retrace_for_invalid_cached_target() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let cached_old_ref = p(0, 0, 0);
    let query_old_ref = p(1, 0, 0);
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let cached_context = support_reference_cache_context_key(
        &cached_old_ref,
        &[axis_plane_definition(&cached_old_ref)],
        &[0],
        &polygons,
    );
    let query_context = support_reference_cache_context_key(
        &query_old_ref,
        &[axis_plane_definition(&query_old_ref)],
        &[7],
        &polygons,
    );
    let mut query_caches = SupportReferenceQueryCaches::default();
    query_caches
        .projection_escape_search_cache
        .borrow_mut()
        .push(ProjectionEscapeSearchCacheEntry {
            context: cached_context,
            bounds: bounds.clone(),
            result: Ok(Some((ReferenceTarget::axis_defined(p(2, 1, 1)), vec![11]))),
        });
    let mut calls = 0;

    let reused = cached_reference_escape_search_in_query_caches(
        &mut query_caches,
        &query_context,
        &bounds,
        |_escape_bounds, _query_caches| {
            calls += 1;
            Ok(Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![99])))
        },
    )
    .unwrap();

    assert_eq!(
        reused,
        Some((ReferenceTarget::axis_defined(p(1, 1, 1)), vec![99]))
    );
    assert_eq!(calls, 1);
}

#[test]
fn projection_axis_escape_stop_values_include_later_bound_corridor() {
    let mut wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let stops = escaped_reference_axis_stop_values(&p(1, 3, 3), &bounds, &[wall], 0, true).unwrap();

    assert_eq!(stops, vec![r(4), r(6)]);
}

#[test]
fn projection_axis_escape_stop_values_report_unknown_for_bound_start_contact() {
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let err = escaped_reference_axis_stop_values(&p(6, 3, 3), &bounds, &[], 0, true).unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projection_axis_escape_reference_backtracks_after_empty_nearer_corridor() {
    let mut wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
    let mut searched_corridors = Vec::new();

    let found =
        projection_axis_escape_reference_with_search(&p(1, 3, 3), &bounds, &[wall], |corridor| {
            searched_corridors.push(corridor.clone());
            if corridor.max.x == r(4) {
                Ok(None)
            } else if corridor.max.x == r(6) {
                Ok(Some((ReferenceTarget::axis_defined(p(5, 3, 3)), vec![9])))
            } else {
                Ok(None)
            }
        })
        .unwrap();

    assert!(
        searched_corridors
            .iter()
            .any(|corridor| corridor.max.x == r(4) && corridor.min.x == r(1))
    );
    assert!(
        searched_corridors
            .iter()
            .any(|corridor| corridor.max.x == r(6) && corridor.min.x == r(1))
    );
    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(5, 3, 3)), vec![9]))
    );
}

#[test]
fn projection_axis_escape_reference_backtracks_after_uncertified_corridor() {
    let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    left.delta_w = vec![1];
    let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    right.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
    let mut attempts = 0;

    let found = projection_axis_escape_reference_with_search(
        &p(1, 3, 3),
        &bounds,
        &[left, right],
        |_corridor| {
            attempts += 1;
            if attempts == 1 {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![7])))
            }
        },
    )
    .unwrap();

    assert!(attempts >= 2);
    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![7]))
    );
}

#[test]
fn projection_axis_escape_reference_accepts_later_corridor_after_endpoint_boundary_contact() {
    let mut boundary = make_triangle(&p(6, 3, 3), &p(6, 5, 3), &p(6, 3, 5), 0, 0);
    boundary.delta_w = vec![1];
    let mut interior = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    interior.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
    let mut searched_corridors = Vec::new();

    let found = projection_axis_escape_reference_with_search(
        &p(1, 3, 3),
        &bounds,
        &[boundary, interior],
        |corridor| {
            searched_corridors.push(corridor.clone());
            if corridor.max.x == r(4) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![31])))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert!(
        searched_corridors
            .iter()
            .any(|corridor| corridor.max.x == r(4) && corridor.min.x == r(1))
    );
    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![31]))
    );
}

#[test]
fn projection_axis_escape_reference_accepts_later_corridor_after_boundary_start_contact() {
    let mut boundary = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    boundary.delta_w = vec![1];
    let mut interior = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    interior.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
    let mut searched_corridors = Vec::new();

    let found = projection_axis_escape_reference_with_search(
        &p(1, 3, 3),
        &bounds,
        &[boundary, interior],
        |corridor| {
            searched_corridors.push(corridor.clone());
            if corridor.max.x == r(4) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![41])))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert!(
        searched_corridors
            .iter()
            .any(|corridor| corridor.max.x == r(4) && corridor.min.x == r(1))
    );
    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![41]))
    );
}

#[test]
fn projection_axis_escape_reference_reports_unknown_when_corridor_family_is_partially_uncertified_and_search_fails()
 {
    let projected = p(1, 3, 3);
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(6)]),
        (vec![r(0)], vec![r(6)]),
    ];

    let err = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
        &projected,
        &axis_options,
        true,
        |_corridor| Ok(None),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projection_axis_escape_reference_accepts_later_corridor_after_uncertified_family_candidate() {
    let projected = p(1, 3, 3);
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(6)]),
        (vec![r(0)], vec![r(6)]),
    ];

    let found = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
        &projected,
        &axis_options,
        true,
        |corridor| {
            if corridor.max.x == r(2) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 3, 3)), vec![13])))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 3, 3)), vec![13]))
    );
}

#[test]
fn projection_axis_escape_reference_certifies_fallback_corridor_success() {
    let projected = p(1, 3, 3);
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(6)]),
        (vec![r(0)], vec![r(6)]),
    ];

    let found = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
        &projected,
        &axis_options,
        false,
        |corridor| {
            if corridor.max.x == r(1) {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(1, 3, 3)),
                    vec![11],
                )))
            } else if corridor.max.x == r(2) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 3, 3)), vec![13])))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 3, 3), &[11]);
}

#[test]
fn projection_axis_escape_reference_certifies_only_fallback_corridor_success() {
    let projected = p(1, 3, 3);
    let axis_options = vec![
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(6)]),
        (vec![r(0)], vec![r(6)]),
    ];

    let found = projection_axis_escape_reference_with_search_and_axis_options_tracking_unknown(
        &projected,
        &axis_options,
        false,
        |corridor| {
            if corridor.max.x == r(1) {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(1, 3, 3)),
                    vec![11],
                )))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 3, 3), &[11]);
}

#[test]
fn projection_axis_escape_reference_reports_unknown_if_all_corridors_are_uncertified() {
    let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    left.delta_w = vec![1];
    let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    right.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let err = projection_axis_escape_reference_with_search(
        &p(1, 3, 3),
        &bounds,
        &[left, right],
        |_corridor| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projection_escape_reference_backtracks_after_uncertified_tight_box() {
    let mut x_wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
    x_wall.delta_w = vec![1];
    let mut y_wall = make_triangle(&p(0, 5, 0), &p(6, 5, 0), &p(0, 5, 6), 0, 1);
    y_wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let found = projection_escape_reference_with_search(
        &p(1, 3, 3),
        &bounds,
        &[x_wall, y_wall],
        |escape_bounds| {
            if *escape_bounds == Aabb::new(p(0, 0, 0), p(4, 6, 6)) {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else if *escape_bounds == Aabb::new(p(0, 0, 0), p(6, 5, 6)) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![5])))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 2, 2)), vec![5]))
    );
}

#[test]
fn projection_escape_reference_reports_unknown_when_box_family_is_partially_uncertified_and_search_fails()
 {
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

    let err = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
        &axis_options,
        &bounds,
        false,
        |_escape_bounds| Ok(None),
        |axis_options, saw_unknown| {
            let (family, family_unknown) =
                projection_escape_bounds_family_from_axis_options_with_extents(
                    axis_options,
                    |escape_bounds| {
                        if *escape_bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                            Err(crate::error::HypermeshError::UnknownClassification)
                        } else {
                            Ok(true)
                        }
                    },
                )?;
            *saw_unknown |= family_unknown;
            Ok(family)
        },
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projection_escape_reference_accepts_later_box_after_uncertified_family_candidate() {
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

    let found = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
        &axis_options,
        &bounds,
        false,
        |escape_bounds| {
            if *escape_bounds == Aabb::new(p(0, 0, 0), p(2, 1, 1)) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![3])))
            } else {
                Ok(None)
            }
        },
        |axis_options, saw_unknown| {
            let (family, family_unknown) =
                projection_escape_bounds_family_from_axis_options_with_extents(
                    axis_options,
                    |escape_bounds| {
                        if *escape_bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                            Err(crate::error::HypermeshError::UnknownClassification)
                        } else {
                            Ok(true)
                        }
                    },
                )?;
            *saw_unknown |= family_unknown;
            Ok(family)
        },
    )
    .unwrap();

    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![3]))
    );
}

#[test]
fn projection_escape_reference_certifies_fallback_box_success() {
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

    let found = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
        &axis_options,
        &bounds,
        false,
        |escape_bounds| {
            if *escape_bounds == Aabb::new(p(0, 0, 0), p(1, 1, 1)) {
                Ok(Some((
                    ReferenceTarget::axis_defined_fallback(p(1, 1, 1)),
                    vec![7],
                )))
            } else if *escape_bounds == Aabb::new(p(0, 0, 0), p(2, 1, 1)) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![9])))
            } else {
                Ok(None)
            }
        },
        |axis_options, saw_unknown| {
            let (family, family_unknown) =
                projection_escape_bounds_family_from_axis_options_with_extents(
                    axis_options,
                    |_escape_bounds| Ok(true),
                )?;
            *saw_unknown |= family_unknown;
            Ok(family)
        },
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 1, 1), &[7]);
}

#[test]
fn projection_escape_reference_certifies_only_fallback_box_success() {
    let axis_options = vec![
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

    let found = projection_escape_reference_with_search_and_axis_options_and_bounds_family(
        &axis_options,
        &bounds,
        false,
        |_escape_bounds| {
            Ok(Some((
                ReferenceTarget::axis_defined_fallback(p(1, 1, 1)),
                vec![7],
            )))
        },
        |axis_options, saw_unknown| {
            let (family, family_unknown) =
                projection_escape_bounds_family_from_axis_options_with_extents(
                    axis_options,
                    |_escape_bounds| Ok(true),
                )?;
            *saw_unknown |= family_unknown;
            Ok(family)
        },
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 1, 1), &[7]);
}

#[test]
fn projection_escape_reference_reports_unknown_when_axis_option_family_is_partially_uncertified_and_box_search_fails()
 {
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

    let err = projection_escape_reference_with_search_and_axis_options_tracking_unknown(
        &axis_options,
        &bounds,
        true,
        |_escape_bounds| Ok(None),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projection_escape_reference_accepts_later_box_after_uncertified_axis_option_family_candidate() {
    let axis_options = vec![
        (vec![r(0)], vec![r(1), r(2)]),
        (vec![r(0)], vec![r(1)]),
        (vec![r(0)], vec![r(1)]),
    ];
    let bounds = Aabb::new(p(0, 0, 0), p(3, 3, 3));

    let found = projection_escape_reference_with_search_and_axis_options_tracking_unknown(
        &axis_options,
        &bounds,
        true,
        |escape_bounds| {
            if *escape_bounds == Aabb::new(p(0, 0, 0), p(2, 1, 1)) {
                Ok(Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![19])))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(2, 0, 0)), vec![19]))
    );
}

#[test]
fn projection_escape_reference_reports_unknown_if_all_boxes_are_uncertified() {
    let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    left.delta_w = vec![1];
    let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    right.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));

    let err = projection_escape_reference_with_search(
        &p(1, 3, 3),
        &bounds,
        &[left, right],
        |_escape_bounds| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn projection_escape_reference_backtracks_after_empty_tighter_box() {
    let mut x_wall = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 0);
    x_wall.delta_w = vec![1];
    let mut y_wall = make_triangle(&p(0, 5, 0), &p(6, 5, 0), &p(0, 5, 6), 0, 1);
    y_wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(6, 6, 6));
    let mut searched_boxes = Vec::new();

    let found = projection_escape_reference_with_search(
        &p(1, 3, 3),
        &bounds,
        &[x_wall, y_wall],
        |escape_bounds| {
            searched_boxes.push(escape_bounds.clone());
            if *escape_bounds == Aabb::new(p(0, 0, 0), p(4, 5, 6)) {
                Ok(None)
            } else if *escape_bounds == Aabb::new(p(0, 0, 0), p(6, 5, 6)) {
                Ok(Some((ReferenceTarget::axis_defined(p(5, 4, 3)), vec![11])))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert!(searched_boxes.contains(&Aabb::new(p(0, 0, 0), p(4, 5, 6))));
    assert!(searched_boxes.contains(&Aabb::new(p(0, 0, 0), p(6, 5, 6))));
    assert_eq!(
        found,
        Some((ReferenceTarget::axis_defined(p(5, 4, 3)), vec![11]))
    );
}

#[test]
fn support_plane_cell_finds_target_when_midpoint_is_blocked() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(5))),
        support_only_polygon(Plane::axis_aligned(1, r(5))),
        support_only_polygon(Plane::axis_aligned(2, r(5))),
        support_only_polygon(Plane::from_coefficients(r(1), r(1), r(1), r(-15))),
    ];

    assert!(point_lies_on_any_support_plane(&p(5, 5, 5), &polygons).unwrap());

    let target = support_plane_cell_target(&bounds, &polygons)
        .unwrap()
        .expect("strict support cell should have a feasible witness");

    assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
    assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
    assert!(
        target
            .definitions
            .iter()
            .any(|definition| affine_from_planes(definition).unwrap() == target.point)
    );
}

#[test]
fn point_lies_on_any_support_plane_reports_unknown_for_boundary_contact() {
    let polygon = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);

    let err = point_lies_on_any_support_plane(&p(2, 0, 0), &[polygon]).unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn point_lies_on_any_support_plane_ignores_coplanar_points_outside_polygon() {
    let polygon = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);

    assert!(!point_lies_on_any_support_plane(&p(5, 5, 0), &[polygon]).unwrap());
}

#[test]
fn support_plane_cell_search_accepts_current_cell_before_full_side_assignment() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(2))),
        support_only_polygon(Plane::axis_aligned(1, r(2))),
    ];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();
    let mut calls = 0;

    let found = support_plane_cell_search_from(
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces, _report| {
            calls += 1;
            if calls == 1 {
                assert_eq!(halfspaces.len(), root_halfspace_count);
                Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
            } else {
                panic!(
                    "search should have accepted the current feasible support cell before \
                     exhausting later polygon branches"
                );
            }
        },
    )
    .unwrap();

    assert_eq!(calls, 1);
    assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
}

#[test]
fn support_plane_cell_search_backtracks_after_uncertified_current_cell() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(2))),
        support_only_polygon(Plane::axis_aligned(1, r(2))),
    ];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();
    let mut call_halfspace_counts = Vec::new();
    let mut calls = 0;

    let found = support_plane_cell_search_from(
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces, _report| {
            calls += 1;
            call_halfspace_counts.push(halfspaces.len());
            if calls == 1 {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
            }
        },
    )
    .unwrap();

    assert!(calls >= 2);
    assert_eq!(call_halfspace_counts[0], root_halfspace_count);
    assert!(
        call_halfspace_counts[1..]
            .iter()
            .any(|count| *count >= root_halfspace_count)
    );
    assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
}

#[test]
fn support_plane_cell_search_backtracks_after_uncertified_current_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(2))),
        support_only_polygon(Plane::axis_aligned(1, r(2))),
    ];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();
    let mut accept_counts = Vec::new();

    let found = support_plane_cell_search_with_queries(
        None,
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces| {
            if halfspaces.len() == root_halfspace_count {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                halfspace_system_report(halfspaces)
            }
        },
        &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
        &mut |halfspaces, report| {
            assert!(report.is_none());
            accept_counts.push(halfspaces.len());
            Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
        },
    )
    .unwrap();

    assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
    assert!(accept_counts.contains(&root_halfspace_count));
}

#[test]
fn support_plane_cell_search_backtracks_after_uncertified_branch_feasibility() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();
    let mut accepted_counts = Vec::new();

    let found = support_plane_cell_search_with_queries(
        None,
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces| halfspace_system_report(halfspaces),
        &mut |halfspaces| {
            if halfspaces.len() == root_halfspace_count + 1 {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                halfspace_system_is_feasible(halfspaces)
            }
        },
        &mut |halfspaces, _report| {
            accepted_counts.push(halfspaces.len());
            if halfspaces.len() == root_halfspace_count + 1 {
                Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
    assert!(accepted_counts.contains(&(root_halfspace_count + 1)));
}

#[test]
fn support_plane_cell_search_accepts_current_cell_without_certified_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(2))),
        support_only_polygon(Plane::axis_aligned(1, r(2))),
    ];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();
    let mut accepted_counts = Vec::new();

    let found = support_plane_cell_search_with_queries(
        None,
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces| {
            if halfspaces.len() == root_halfspace_count {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                halfspace_system_report(halfspaces)
            }
        },
        &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
        &mut |halfspaces, report| {
            accepted_counts.push((halfspaces.len(), report.is_some()));
            if halfspaces.len() == root_halfspace_count {
                Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
    assert!(
        accepted_counts
            .iter()
            .any(|(count, had_report)| *count == root_halfspace_count && !had_report)
    );
}

#[test]
fn support_plane_cell_search_skips_current_report_after_direct_accept() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(2))),
        support_only_polygon(Plane::axis_aligned(1, r(2))),
    ];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();
    let mut accepted_counts = Vec::new();

    let found = support_plane_cell_search_with_queries(
        None,
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces| {
            if halfspaces.len() == root_halfspace_count {
                panic!("root report query should be skipped after direct support accept");
            }
            halfspace_system_report(halfspaces)
        },
        &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
        &mut |halfspaces, report| {
            accepted_counts.push((halfspaces.len(), report.is_some()));
            if halfspaces.len() == root_halfspace_count && report.is_none() {
                Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))))
            } else {
                Ok(None)
            }
        },
    )
    .unwrap();

    assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
    assert_eq!(accepted_counts, vec![(root_halfspace_count, false)]);
}

#[test]
fn support_plane_cell_search_reports_unknown_if_current_report_and_branches_are_uncertified() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let err = support_plane_cell_search_with_queries(
        None,
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |_halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
        &mut |_halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
        &mut |_halfspaces, _report| {
            Err::<Option<ReferenceTarget>, _>(crate::error::HypermeshError::UnknownClassification)
        },
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn support_plane_cell_search_prefers_reference_side_first() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();
    let mut accepted_branch = None;

    let found = support_plane_cell_search_with_queries(
        Some(&p(1, 1, 1)),
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces| halfspace_system_report(halfspaces),
        &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
        &mut |halfspaces, _report| {
            if halfspaces.len() == root_halfspace_count + 1 {
                accepted_branch = Some(
                    halfspaces.last().unwrap()
                        == &support_side_halfspace(&polygons[0].support, false),
                );
                return Ok(Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
            }
            Ok(None)
        },
    )
    .unwrap();

    assert_eq!(found, Some(ReferenceTarget::axis_defined(p(1, 1, 1))));
    assert_eq!(accepted_branch, Some(true));
}

#[test]
fn support_plane_cell_search_skips_duplicate_support_halfspace_branches() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(2))),
        support_only_polygon(Plane::axis_aligned(0, r(2))),
    ];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let repeated_branch = support_side_halfspace(&polygons[0].support, false);
    let mut duplicate_branch_count_seen = false;

    let found = support_plane_cell_search_with_queries(
        Some(&p(1, 1, 1)),
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |halfspaces| halfspace_system_report(halfspaces),
        &mut |halfspaces| {
            let repeated_count = halfspaces
                .iter()
                .filter(|halfspace| *halfspace == &repeated_branch)
                .count();
            if repeated_count > 1 {
                duplicate_branch_count_seen = true;
            }
            halfspace_system_is_feasible(halfspaces)
        },
        &mut |_halfspaces, _report| Ok(None::<ReferenceTarget>),
    )
    .unwrap();

    assert_eq!(found, None);
    assert!(!duplicate_branch_count_seen);
}

#[test]
fn support_plane_cell_search_skips_already_fixed_support_plane_states() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(2))),
        support_only_polygon(Plane::axis_aligned(0, r(2))),
    ];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    halfspaces.push(support_side_halfspace(&polygons[0].support, false));
    let mut report_calls = 0;
    let mut accept_calls = 0;

    let found = support_plane_cell_search_with_queries(
        Some(&p(1, 1, 1)),
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |_halfspaces| {
            report_calls += 1;
            Ok(None)
        },
        &mut |_halfspaces| Ok(true),
        &mut |_halfspaces, _report| {
            accept_calls += 1;
            Ok(None::<ReferenceTarget>)
        },
    )
    .unwrap();

    assert_eq!(found, None);
    assert_eq!(report_calls, 1);
    assert_eq!(accept_calls, 1);
}

#[test]
fn support_plane_cell_search_skips_opposite_support_halfspace_branches() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    halfspaces.push(support_side_halfspace(&polygon.support, false));
    let opposite_branch = support_side_halfspace(&polygon.support, true);
    let mut opposite_branch_count_seen = false;

    let found = support_plane_cell_search_with_queries(
        Some(&p(1, 1, 1)),
        &bounds,
        &[polygon],
        0,
        &mut halfspaces,
        &mut |halfspaces| halfspace_system_report(halfspaces),
        &mut |halfspaces| {
            let opposite_count = halfspaces
                .iter()
                .filter(|halfspace| *halfspace == &opposite_branch)
                .count();
            if opposite_count > 0 {
                opposite_branch_count_seen = true;
            }
            halfspace_system_is_feasible(halfspaces)
        },
        &mut |_halfspaces, _report| Ok(None::<ReferenceTarget>),
    )
    .unwrap();

    assert_eq!(found, None);
    assert!(!opposite_branch_count_seen);
}

#[test]
fn support_plane_cell_search_skips_surface_forcing_halfspace_states() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
    let polygons = vec![polygon.clone()];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    halfspaces.push(support_side_halfspace(&polygon.support, false));
    halfspaces.push(support_side_halfspace(&polygon.support, true));
    let mut report_calls = 0;
    let mut accept_calls = 0;

    let found = support_plane_cell_search_with_queries(
        Some(&p(1, 1, 1)),
        &bounds,
        &polygons,
        0,
        &mut halfspaces,
        &mut |_halfspaces| {
            report_calls += 1;
            Ok(None)
        },
        &mut |_halfspaces| Ok(true),
        &mut |_halfspaces, _report| {
            accept_calls += 1;
            Ok(None::<ReferenceTarget>)
        },
    )
    .unwrap();

    assert_eq!(found, None);
    assert_eq!(report_calls, 0);
    assert_eq!(accept_calls, 0);
}

#[test]
fn support_plane_cell_reference_accepts_current_cell_without_certified_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let old_ref = p(-1, 1, 1);
    let old_defs = axis_defs(&old_ref);
    let old_wnv = vec![0];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();

    let found = support_plane_cell_reference_with_queries(
        &old_ref,
        &old_defs,
        &old_wnv,
        &bounds,
        &polygons,
        &mut halfspaces,
        &mut |halfspaces| {
            if halfspaces.len() == root_halfspace_count {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                halfspace_system_report(halfspaces)
            }
        },
        &mut |halfspaces| halfspace_system_is_feasible(halfspaces),
    )
    .unwrap()
    .expect("current support cell should be usable without a certified report");

    assert!(point_strictly_inside_bounds(&found.0.point, &bounds).unwrap());
    assert!(!point_lies_on_any_support_plane(&found.0.point, &polygons).unwrap());
}

#[test]
fn support_plane_cell_reference_backtracks_after_uncertified_initial_feasibility_check() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(2)))];
    let old_ref = p(-1, 1, 1);
    let old_defs = axis_defs(&old_ref);
    let old_wnv = vec![0];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let root_halfspace_count = halfspaces.len();

    let found = support_plane_cell_reference_with_queries(
        &old_ref,
        &old_defs,
        &old_wnv,
        &bounds,
        &polygons,
        &mut halfspaces,
        &mut |halfspaces| {
            if halfspaces.len() == root_halfspace_count {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                halfspace_system_report(halfspaces)
            }
        },
        &mut |halfspaces| {
            if halfspaces.len() == root_halfspace_count {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                halfspace_system_is_feasible(halfspaces)
            }
        },
    )
    .unwrap();

    assert!(found.is_some());
}

#[test]
fn support_plane_cell_reference_reports_unknown_if_initial_feasibility_and_search_fail() {
    let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(0)))];
    let old_ref = p(-1, 0, 0);
    let old_defs = axis_defs(&old_ref);
    let old_wnv = vec![0];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let err = support_plane_cell_reference_with_queries(
        &old_ref,
        &old_defs,
        &old_wnv,
        &bounds,
        &polygons,
        &mut halfspaces,
        &mut |_halfspaces| Ok(None),
        &mut |_halfspaces| Err(crate::error::HypermeshError::UnknownClassification),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn duplicate_reference_targets_merge_definitions() {
    let point = p(1, 2, 3);
    let mut targets = vec![ReferenceTarget::axis_defined(point.clone())];
    let slanted_definition = [
        Plane::from_coefficients(r(1), r(1), r(0), r(-3)),
        Plane::axis_aligned(0, r(1)),
        Plane::axis_aligned(2, r(3)),
    ];

    push_unique_reference_target(
        &mut targets,
        ReferenceTarget::with_definitions(point, vec![slanted_definition.clone()]),
    );
    push_unique_reference_target(
        &mut targets,
        ReferenceTarget::with_definitions(p(1, 2, 3), vec![slanted_definition]),
    );

    assert_eq!(targets.len(), 1);
    assert_eq!(targets[0].definitions.len(), 2);
    assert!(
        targets[0]
            .definitions
            .iter()
            .any(definition_uses_non_axis_plane)
    );
}

#[test]
fn take_new_point_family_preserves_first_occurrence_order() {
    let mut seen = vec![p(0, 0, 0)];
    let fresh = take_new_point_family(
        vec![p(1, 0, 0), p(0, 0, 0), p(2, 0, 0), p(1, 0, 0)],
        &mut seen,
    );

    assert_eq!(fresh, vec![p(1, 0, 0), p(2, 0, 0)]);
    assert_eq!(seen, vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)]);
}

#[test]
fn shifted_target_seed_families_preserve_direct_report_witness_and_skip_later_duplicates() {
    let witness = p(1, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        dedupe_shifted_target_seed_families(
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
fn shifted_target_seed_families_with_report_seed_promote_report_witness_to_shifted_root() {
    let witness = p(1, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        shifted_target_seed_families_with_report_seed(
            Some(&witness),
            Vec::new(),
            vec![witness.clone(), p(2, 1, 1)],
            vec![witness.clone(), p(3, 1, 1)],
        );

    assert_eq!(strict_seeds, vec![witness]);
    assert_eq!(shifted_vertices, vec![p(2, 1, 1)]);
    assert_eq!(shifted_geometry_seeds, vec![p(3, 1, 1)]);
}

#[test]
fn support_shifted_target_seed_families_keep_one_strict_root_after_certified_direct_target() {
    let witness = p(1, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        support_shifted_target_seed_families(
            Some(&witness),
            vec![witness.clone(), p(2, 1, 1)],
            vec![p(3, 1, 1)],
            vec![p(4, 1, 1)],
            &[ReferenceTarget::axis_defined(p(2, 1, 1))],
        );

    assert_eq!(strict_seeds, vec![witness]);
    assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
    assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
}

#[test]
fn support_shifted_target_seed_families_fall_back_to_first_certified_direct_target() {
    let direct_target_point = p(2, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        support_shifted_target_seed_families(
            None,
            vec![direct_target_point.clone(), p(5, 1, 1)],
            vec![p(3, 1, 1)],
            vec![p(4, 1, 1)],
            &[ReferenceTarget::axis_defined(direct_target_point.clone())],
        );

    assert_eq!(strict_seeds, vec![direct_target_point]);
    assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
    assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
}

#[test]
fn support_shifted_target_seed_families_keep_full_strict_family_without_certified_direct_target() {
    let strict_seed = p(2, 1, 1);
    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        support_shifted_target_seed_families(
            None,
            vec![strict_seed.clone()],
            vec![p(3, 1, 1)],
            vec![p(4, 1, 1)],
            &[ReferenceTarget::axis_defined_fallback(p(7, 7, 7))],
        );

    assert_eq!(strict_seeds, vec![strict_seed]);
    assert_eq!(shifted_vertices, vec![p(3, 1, 1)]);
    assert_eq!(shifted_geometry_seeds, vec![p(4, 1, 1)]);
}

#[test]
fn point_seed_family_search_failure_allows_later_shifted_seeds_after_unknown_strict_family() {
    assert!(!point_seed_family_search_failed_without_any_seed(
        &[],
        &[p(1, 1, 1)],
        &[],
        true,
    ));
    assert!(!point_seed_family_search_failed_without_any_seed(
        &[],
        &[],
        &[p(2, 2, 2)],
        true,
    ));
}

#[test]
fn point_seed_family_search_failure_reports_unknown_only_when_every_seed_family_is_empty() {
    assert!(point_seed_family_search_failed_without_any_seed(
        &[],
        &[],
        &[],
        true,
    ));
    assert!(!point_seed_family_search_failed_without_any_seed(
        &[p(3, 3, 3)],
        &[],
        &[],
        true,
    ));
}

#[test]
fn projected_escape_target_family_keeps_same_point_report_witness_definitions() {
    let point = p(1, 2, 3);
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(3)),
        axis_halfspace(2, false, r(3)),
        LimitPlane3::new(p(1, 1, 1), r(-6)),
        LimitPlane3::new(p(-1, -1, -1), r(6)),
    ];
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &halfspaces,
        &[ReferenceTarget::axis_defined(point.clone())],
        Some(&hyperlimit::HalfspaceFeasibilityReport::feasible(
            point.clone(),
            [None, None, None],
        )),
        Vec::new(),
        Vec::new(),
        Vec::new(),
        &mut saw_unknown,
        |_seed| Ok(Vec::new()),
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets.len(), 1);
    assert!(targets[0].uncertified_definition_fallback);
    assert!(
        targets[0]
            .definitions
            .iter()
            .any(definition_uses_non_axis_plane)
    );
}

#[test]
fn projected_escape_target_family_keeps_same_point_direct_seed_definitions() {
    let point = p(1, 2, 3);
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(3)),
        axis_halfspace(2, false, r(3)),
        LimitPlane3::new(p(1, 1, 1), r(-6)),
        LimitPlane3::new(p(-1, -1, -1), r(6)),
    ];
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &halfspaces,
        &[ReferenceTarget::axis_defined(point.clone())],
        None,
        vec![point],
        Vec::new(),
        Vec::new(),
        &mut saw_unknown,
        |_seed| Ok(Vec::new()),
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets.len(), 1);
    assert!(targets[0].uncertified_definition_fallback);
    assert!(
        targets[0]
            .definitions
            .iter()
            .any(definition_uses_non_axis_plane)
    );
}

#[test]
fn projected_escape_target_family_keeps_same_point_report_witness_direct_seed_definitions() {
    let point = p(1, 2, 3);
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(3)),
        axis_halfspace(2, false, r(3)),
        LimitPlane3::new(p(1, 1, 1), r(-6)),
        LimitPlane3::new(p(-1, -1, -1), r(6)),
    ];
    let report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(point.clone(), [None, None, None]);
    let mut saw_unknown = false;

    let targets = projected_reference_escape_targets_from_seed_families_with_tracking_unknown(
        &halfspaces,
        &[ReferenceTarget::axis_defined(point.clone())],
        Some(&report),
        vec![point],
        Vec::new(),
        Vec::new(),
        &mut saw_unknown,
        |_seed| Ok(Vec::new()),
    )
    .unwrap();

    assert!(saw_unknown);
    assert_eq!(targets.len(), 1);
    assert!(targets[0].uncertified_definition_fallback);
    assert!(
        targets[0]
            .definitions
            .iter()
            .any(definition_uses_non_axis_plane)
    );
}

#[test]
fn shifted_projected_escape_target_family_search_skips_duplicate_seed_sources() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut targets = Vec::new();
    let visited = std::cell::RefCell::new(Vec::new());

    collect_shifted_projected_escape_target_families(
        &mut targets,
        None,
        vec![first.clone(), second.clone()],
        vec![second.clone(), first.clone()],
        Vec::new(),
        |_candidate| Ok(true),
        |_candidate| Ok(None),
        |candidate| {
            visited.borrow_mut().push(candidate.clone());
            Ok(Some(ReferenceTarget::axis_defined(candidate.clone())))
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![first.clone(), second.clone()]);
    assert_eq!(
        targets
            .into_iter()
            .map(|target| target.point)
            .collect::<Vec<_>>(),
        vec![first, second]
    );
}

#[test]
fn shifted_projected_escape_target_family_search_promotes_report_witness_to_shifted_root() {
    let witness = p(1, 2, 3);
    let mut targets = Vec::new();
    let visited = std::cell::RefCell::new(Vec::new());

    collect_shifted_projected_escape_target_families(
        &mut targets,
        Some(&witness),
        Vec::new(),
        vec![witness.clone()],
        Vec::new(),
        |_candidate| Ok(true),
        |_candidate| Ok(None),
        |candidate| {
            visited.borrow_mut().push(candidate.clone());
            Ok(Some(ReferenceTarget::axis_defined(p(9, 9, 9))))
        },
    )
    .unwrap();

    assert_eq!(visited.into_inner(), vec![witness]);
    assert!(targets.iter().any(|target| target.point == p(9, 9, 9)));
}

#[test]
fn shifted_projected_escape_target_family_search_backtracks_after_uncertified_earlier_family() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut targets = Vec::new();

    collect_shifted_projected_escape_target_families(
        &mut targets,
        None,
        vec![first.clone()],
        vec![first, second.clone()],
        Vec::new(),
        |_candidate| Ok(true),
        |_candidate| Ok(None),
        |candidate| {
            if *candidate == p(2, 2, 2) {
                Ok(Some(ReferenceTarget::axis_defined(candidate.clone())))
            } else {
                Err(crate::error::HypermeshError::UnknownClassification)
            }
        },
    )
    .unwrap();

    assert_eq!(
        targets
            .into_iter()
            .map(|target| target.point)
            .collect::<Vec<_>>(),
        vec![second]
    );
}

#[test]
fn winding_reachability_prunes_difference_when_other_mesh_cannot_reach_zero() {
    let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    first.delta_w = vec![0, 1];
    let mut second = make_triangle(&p(0, 0, 1), &p(1, 0, 1), &p(0, 1, 1), 1, 0);
    second.delta_w = vec![0, 1];

    assert!(
        can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 3], &[first, second])
            .unwrap()
    );
}

#[test]
fn winding_reachability_keeps_difference_when_other_mesh_can_reach_zero() {
    let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    first.delta_w = vec![0, 1];

    assert!(
        !can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &[first]).unwrap()
    );
}

#[test]
fn winding_reachability_prunes_correlated_difference_when_zero_is_not_jointly_reachable() {
    let mut correlated = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    correlated.delta_w = vec![1, 1];

    assert!(
        can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &[correlated]).unwrap()
    );
}

#[test]
fn cached_winding_reachability_reuses_transition_multiset_across_polygon_geometry() {
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let mut first = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    first.delta_w = vec![1, 1];
    let mut second = make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 1, 0);
    second.delta_w = vec![0, -1];
    let first_polygons = vec![first.clone(), second.clone()];

    let mut third = make_triangle(&p(3, 0, 0), &p(4, 0, 0), &p(3, 1, 0), 2, 0);
    third.delta_w = vec![0, -1];
    let mut fourth = make_triangle(&p(3, 0, 2), &p(4, 0, 2), &p(3, 1, 2), 3, 0);
    fourth.delta_w = vec![1, 1];
    let second_polygons = vec![third, fourth];

    let first_result = cached_winding_reachability_with(
        &cache,
        BooleanOp::Difference,
        &[1, 1],
        &first_polygons,
        || {
            calls.set(calls.get() + 1);
            can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &first_polygons)
        },
    )
    .unwrap();
    let second_result = cached_winding_reachability_with(
        &cache,
        BooleanOp::Difference,
        &[1, 1],
        &second_polygons,
        || {
            calls.set(calls.get() + 1);
            can_discard_by_winding_reachability(BooleanOp::Difference, &[1, 1], &second_polygons)
        },
    )
    .unwrap();

    assert_eq!(first_result, second_result);
    assert_eq!(calls.get(), 1);
}

#[test]
fn cached_winding_reachability_distinguishes_reference_winding_context() {
    let cache = RefCell::new(Vec::new());
    let calls = std::cell::Cell::new(0);

    let mut first_polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    first_polygon.delta_w = vec![0, 1];
    let mut second_polygon = first_polygon.clone();
    second_polygon.mesh_index = 1;

    cached_winding_reachability_with(
        &cache,
        BooleanOp::Difference,
        &[1, 3],
        &[first_polygon.clone()],
        || {
            calls.set(calls.get() + 1);
            can_discard_by_winding_reachability(
                BooleanOp::Difference,
                &[1, 3],
                &[first_polygon.clone()],
            )
        },
    )
    .unwrap();
    cached_winding_reachability_with(
        &cache,
        BooleanOp::Difference,
        &[1, 1],
        &[second_polygon.clone()],
        || {
            calls.set(calls.get() + 1);
            can_discard_by_winding_reachability(
                BooleanOp::Difference,
                &[1, 1],
                &[second_polygon.clone()],
            )
        },
    )
    .unwrap();

    assert_eq!(calls.get(), 2);
}

#[test]
fn support_plane_cell_target_finds_strict_point_in_closed_feasible_cell() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygons = vec![
        support_only_polygon(Plane::from_coefficients(r(-1), r(0), r(0), q(7, 2))),
        support_only_polygon(Plane::from_coefficients(r(1), r(0), r(0), q(-13, 2))),
        support_only_polygon(Plane::axis_aligned(0, r(5))),
    ];

    let target = support_plane_cell_target(&bounds, &polygons)
        .unwrap()
        .expect("closed feasible support cell should produce a strict interior point");

    assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
    assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
    assert!(compare_real(&target.point.x, &q(7, 2)).unwrap().is_gt());
    assert!(compare_real(&target.point.x, &q(13, 2)).unwrap().is_lt());
    assert!(
        target
            .definitions
            .iter()
            .any(|definition| affine_from_planes(definition).unwrap() == target.point)
    );
}

#[test]
fn support_plane_cell_search_backtracks_after_leaf_rejection() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(5)))];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut rejected_first_leaf = false;
    let mut accept = |_halfspaces: &[LimitPlane3],
                      report: Option<hyperlimit::HalfspaceFeasibilityReport>|
     -> HypermeshResult<Option<Point3>> {
        let Some(report) = report else {
            return Ok(None);
        };
        let Some(witness) = report.witness else {
            return Ok(None);
        };
        if compare_real(&witness.x, &r(5))?.is_lt() {
            rejected_first_leaf = true;
            return Ok(None);
        }
        Ok(Some(witness))
    };

    let target =
        support_plane_cell_search_from(&bounds, &polygons, 0, &mut halfspaces, &mut accept)
            .unwrap()
            .expect("search should continue after the first accepted leaf rejects");

    assert!(rejected_first_leaf);
    assert!(compare_real(&target.x, &r(5)).unwrap().is_gt());
}

#[test]
fn support_plane_cell_search_backtracks_after_uncertified_leaf() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(5)))];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut rejected_first_leaf = false;
    let mut accept = |_halfspaces: &[LimitPlane3],
                      report: Option<hyperlimit::HalfspaceFeasibilityReport>|
     -> HypermeshResult<Option<Point3>> {
        let Some(report) = report else {
            return Ok(None);
        };
        let Some(witness) = report.witness else {
            return Ok(None);
        };
        if compare_real(&witness.x, &r(5))?.is_lt() {
            rejected_first_leaf = true;
            return Err(crate::error::HypermeshError::UnknownClassification);
        }
        Ok(Some(witness))
    };

    let target =
        support_plane_cell_search_from(&bounds, &polygons, 0, &mut halfspaces, &mut accept)
            .unwrap()
            .expect("search should continue after an uncertified leaf branch");

    assert!(rejected_first_leaf);
    assert!(compare_real(&target.x, &r(5)).unwrap().is_gt());
}

#[test]
fn support_plane_cell_search_reports_unknown_if_all_branches_are_uncertified() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(5)))];
    let mut halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut accept = |_halfspaces: &[LimitPlane3],
                      _report: Option<hyperlimit::HalfspaceFeasibilityReport>|
     -> HypermeshResult<Option<Point3>> {
        Err(crate::error::HypermeshError::UnknownClassification)
    };

    let err = support_plane_cell_search_from(&bounds, &polygons, 0, &mut halfspaces, &mut accept)
        .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn support_plane_cell_reference_traces_certified_winding() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(5))),
        support_only_polygon(Plane::axis_aligned(1, r(5))),
        support_only_polygon(Plane::axis_aligned(2, r(5))),
    ];

    let (target, winding) = support_plane_cell_reference(
        &p(-1, -1, -1),
        &axis_defs(&p(-1, -1, -1)),
        &[7],
        &bounds,
        &polygons,
    )
    .unwrap()
    .expect("strict support cell target should trace from old reference");

    assert_eq!(winding, vec![7]);
    assert!(point_strictly_inside_bounds(&target.point, &bounds).unwrap());
    assert!(!point_lies_on_any_support_plane(&target.point, &polygons).unwrap());
    assert!(!target.definitions.is_empty());
}

#[test]
fn support_plane_cell_reference_returns_exact_definitions() {
    let bounds = Aabb::new(p(0, 0, 0), p(10, 10, 10));
    let polygons = vec![
        support_only_polygon(Plane::axis_aligned(0, r(5))),
        support_only_polygon(Plane::axis_aligned(1, r(5))),
        support_only_polygon(Plane::axis_aligned(2, r(5))),
        support_only_polygon(Plane::from_coefficients(r(1), r(1), r(1), r(-15))),
    ];

    let (target, winding) = support_plane_cell_reference(
        &p(-1, -1, -1),
        &axis_defs(&p(-1, -1, -1)),
        &[3],
        &bounds,
        &polygons,
    )
    .unwrap()
    .expect("support-cell witness should be traceable");

    assert_eq!(winding, vec![3]);
    assert!(
        target
            .definitions
            .iter()
            .any(|definition| affine_from_planes(definition).unwrap() == target.point)
    );
    for definition in target.definitions.iter() {
        assert_eq!(affine_from_planes(definition).unwrap(), target.point);
    }
}

#[test]
fn reference_target_trace_search_backtracks_after_uncertified_target() {
    let first = ReferenceTarget::axis_defined(p(1, 1, 1));
    let second = ReferenceTarget::axis_defined(p(2, 2, 2));

    let found = trace_reference_targets_backtracking_unknown(
        vec![first.clone(), second.clone()],
        &[],
        |target| {
            if target == &first {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(Some(vec![31]))
            }
        },
    )
    .unwrap();

    assert_eq!(found, Some((second, vec![31])));
}

#[test]
fn reference_target_trace_search_reports_unknown_if_all_targets_are_uncertified() {
    let first = ReferenceTarget::axis_defined(p(1, 1, 1));
    let second = ReferenceTarget::axis_defined(p(2, 2, 2));

    let err = trace_reference_targets_backtracking_unknown(vec![first, second], &[], |_target| {
        Err(crate::error::HypermeshError::UnknownClassification)
    })
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_target_trace_search_certifies_fallback_target_after_trace_succeeds() {
    let fallback = ReferenceTarget::axis_defined_fallback(p(1, 1, 1));
    let certified = ReferenceTarget::axis_defined(p(2, 2, 2));

    let found = trace_reference_targets_backtracking_unknown(
        vec![fallback.clone(), certified.clone()],
        &[],
        |target| {
            if target == &fallback {
                Ok(Some(vec![31]))
            } else {
                Ok(Some(vec![37]))
            }
        },
    )
    .unwrap();

    assert_certified_reference_result(found, &p(1, 1, 1), &[31]);
}

#[test]
fn reference_target_trace_search_certifies_only_fallback_target_after_trace() {
    let fallback = ReferenceTarget::axis_defined_fallback(p(1, 1, 1));

    let found = trace_reference_targets_backtracking_unknown(vec![fallback], &[], |_target| {
        Ok(Some(vec![31]))
    })
    .unwrap();

    assert_certified_reference_result(found, &p(1, 1, 1), &[31]);
}

#[test]
fn reference_target_trace_search_skips_support_surface_targets_before_trace() {
    let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
    let surface = ReferenceTarget::axis_defined(p(2, 1, 1));
    let interior = ReferenceTarget::axis_defined(p(1, 1, 1));
    let mut trace_calls = 0;

    let found = trace_reference_targets_backtracking_unknown(
        vec![surface, interior.clone()],
        &[polygon],
        |target| {
            trace_calls += 1;
            assert_eq!(target, &interior);
            Ok(Some(vec![13]))
        },
    )
    .unwrap();

    assert_eq!(trace_calls, 1);
    assert_eq!(found, Some((interior, vec![13])));
}

#[test]
fn reference_target_trace_search_tries_later_target_after_boundary_support_surface_contact() {
    let polygon = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 0, 4), 0, 0);
    let boundary = ReferenceTarget::axis_defined(p(2, 0, 2));
    let interior = ReferenceTarget::axis_defined(p(1, 1, 1));
    let mut trace_calls = 0;

    let found = trace_reference_targets_backtracking_unknown(
        vec![boundary, interior.clone()],
        &[polygon],
        |target| {
            trace_calls += 1;
            assert_eq!(target, &interior);
            Ok(Some(vec![29]))
        },
    )
    .unwrap();

    assert_eq!(trace_calls, 1);
    assert_eq!(found, Some((interior, vec![29])));
}

#[test]
fn reference_target_trace_search_reports_unknown_when_boundary_support_surface_contact_blocks_only_target()
 {
    let polygon = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 0, 4), 0, 0);
    let boundary = ReferenceTarget::axis_defined(p(2, 0, 2));

    let err = trace_reference_targets_backtracking_unknown(vec![boundary], &[polygon], |_target| {
        Ok(Some(vec![29]))
    })
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_target_trace_search_reports_unknown_when_fallback_surface_target_is_skipped() {
    let polygon = support_only_polygon(Plane::axis_aligned(0, r(2)));
    let surface = ReferenceTarget::axis_defined_fallback(p(2, 1, 1));

    let err = trace_reference_targets_backtracking_unknown(vec![surface], &[polygon], |_target| {
        Ok(Some(vec![13]))
    })
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_target_trace_search_reuses_equivalent_support_surface_queries() {
    let first = ReferenceTarget::with_definitions(
        p(2, 1, 1),
        vec![[
            Plane::axis_aligned(0, r(2)),
            Plane::axis_aligned(1, r(1)),
            Plane::axis_aligned(2, r(1)),
        ]],
    );
    let second = ReferenceTarget::axis_defined(p(2, 1, 1));
    let surface_calls = std::cell::Cell::new(0);
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    let found = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![first, second],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |point| {
            surface_calls.set(surface_calls.get() + 1);
            Ok(*point == p(2, 1, 1))
        },
        &mut |_point| Ok(true),
        |_target| Ok(Some(vec![17])),
    )
    .unwrap();

    assert_eq!(found, None);
    assert_eq!(surface_calls.get(), 1);
}

#[test]
fn reference_target_trace_search_reuses_reference_validity_queries_after_surface_passes() {
    let first = ReferenceTarget::with_definitions(
        p(1, 1, 1),
        vec![[
            Plane::axis_aligned(0, r(1)),
            Plane::axis_aligned(1, r(1)),
            Plane::axis_aligned(2, r(1)),
        ]],
    );
    let second = ReferenceTarget::axis_defined(p(1, 1, 1));
    let validity_calls = std::cell::Cell::new(0);
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    let found = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![first, second],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |_point| Ok(false),
        &mut |point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(*point == p(1, 1, 1))
        },
        |_target| Ok(None),
    )
    .unwrap();

    assert_eq!(found, None);
    assert_eq!(validity_calls.get(), 1);
}

#[test]
fn reference_target_trace_search_reuses_reference_validity_queries_across_calls() {
    let target = ReferenceTarget::axis_defined(p(1, 1, 1));
    let validity_calls = std::cell::Cell::new(0);
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    let first = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![target.clone()],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |_point| Ok(false),
        &mut |point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(*point == p(1, 1, 1))
        },
        |_target| Ok(None),
    )
    .unwrap();

    let second = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![target],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |_point| Ok(false),
        &mut |point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(*point == p(1, 1, 1))
        },
        |_target| Ok(None),
    )
    .unwrap();

    assert_eq!(first, None);
    assert_eq!(second, None);
    assert_eq!(validity_calls.get(), 1);
}

#[test]
fn reference_target_trace_search_keeps_distinct_reference_validity_bounds_separate() {
    let target = ReferenceTarget::axis_defined(p(1, 1, 1));
    let validity_calls = std::cell::Cell::new(0);
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![target.clone()],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |_point| Ok(false),
        &mut |point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(*point == p(1, 1, 1))
        },
        |_target| Ok(None),
    )
    .unwrap();

    trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![target],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(5, 4, 4)),
        &mut |_point| Ok(false),
        &mut |point| {
            validity_calls.set(validity_calls.get() + 1);
            Ok(*point == p(1, 1, 1))
        },
        |_target| Ok(None),
    )
    .unwrap();

    assert_eq!(validity_calls.get(), 2);
}

#[test]
fn reference_target_trace_search_reuses_support_surface_queries_across_calls() {
    let target = ReferenceTarget::axis_defined(p(2, 1, 1));
    let surface_calls = std::cell::Cell::new(0);
    let mut surface_cache = Vec::new();

    let first = trace_reference_targets_backtracking_unknown_with_surface_cache(
        vec![target.clone()],
        &mut surface_cache,
        &mut |point| {
            surface_calls.set(surface_calls.get() + 1);
            Ok(*point == p(2, 1, 1))
        },
        |_target| Ok(Some(vec![17])),
    )
    .unwrap();

    let second = trace_reference_targets_backtracking_unknown_with_surface_cache(
        vec![target],
        &mut surface_cache,
        &mut |point| {
            surface_calls.set(surface_calls.get() + 1);
            Ok(*point == p(2, 1, 1))
        },
        |_target| Ok(Some(vec![17])),
    )
    .unwrap();

    assert_eq!(first, None);
    assert_eq!(second, None);
    assert_eq!(surface_calls.get(), 1);
}

#[test]
fn reference_target_trace_search_tries_later_target_after_uncertified_surface_query() {
    let first = ReferenceTarget::axis_defined(p(1, 1, 1));
    let second = ReferenceTarget::axis_defined(p(2, 2, 2));
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    let found = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![first.clone(), second.clone()],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |point| {
            if *point == first.point {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        },
        &mut |_point| Ok(true),
        |target| {
            assert_eq!(target, &second);
            Ok(Some(vec![23]))
        },
    )
    .unwrap();

    assert_eq!(found, Some((second, vec![23])));
}

#[test]
fn reference_target_trace_search_reports_unknown_when_surface_query_is_uncertified_and_later_targets_fail()
 {
    let first = ReferenceTarget::axis_defined(p(1, 1, 1));
    let second = ReferenceTarget::axis_defined(p(2, 2, 2));
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    let err = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![first.clone(), second],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |point| {
            if *point == first.point {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(false)
            }
        },
        &mut |_point| Ok(true),
        |_target| Ok(None),
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn reference_target_trace_search_tries_later_target_after_uncertified_reference_validity_query() {
    let first = ReferenceTarget::axis_defined(p(1, 1, 1));
    let second = ReferenceTarget::axis_defined(p(2, 2, 2));
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    let found = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![first.clone(), second.clone()],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &Aabb::new(p(0, 0, 0), p(4, 4, 4)),
        &mut |_point| Ok(false),
        &mut |point| {
            if *point == first.point {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(true)
            }
        },
        |target| {
            assert_eq!(target, &second);
            Ok(Some(vec![29]))
        },
    )
    .unwrap();

    assert_eq!(found, Some((second, vec![29])));
}

#[test]
fn reference_target_trace_search_tries_later_target_after_boundary_local_surface_validity_query() {
    let first = ReferenceTarget::axis_defined(p(2, 1, 2));
    let second = ReferenceTarget::axis_defined(p(1, 1, 1));
    let mut wall = make_triangle(&p(2, 0, 0), &p(2, 4, 0), &p(2, 2, 4), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let mut surface_cache = Vec::new();
    let mut validity_cache = Vec::new();

    let found = trace_reference_targets_backtracking_unknown_with_query_caches(
        vec![first, second.clone()],
        &mut surface_cache,
        &mut validity_cache,
        None,
        &bounds,
        &mut |_point| Ok(false),
        &mut |point| is_certified_valid_reference_for_bounds(point, &bounds, &[wall.clone()]),
        |target| {
            assert_eq!(target, &second);
            Ok(Some(vec![31]))
        },
    )
    .unwrap();

    assert_eq!(found, Some((second, vec![31])));
}

#[test]
fn unique_overlap_edge_planes_preserve_first_occurrence_and_skip_inverted_duplicates() {
    let x0 = Plane::axis_aligned(0, r(0));
    let y0 = Plane::axis_aligned(1, r(0));
    let y1 = Plane::axis_aligned(1, r(1));
    let support = Plane::axis_aligned(2, r(0));
    let intersections = vec![
        PairwiseIntersection {
            kind: PairwiseIntersectionType::Overlap,
            segment: None,
            overlap: Some(OverlapInfo {
                other_polygon_idx: 0,
                other_edges: vec![x0.clone(), y0.clone()],
                other_support: support.clone(),
            }),
        },
        PairwiseIntersection {
            kind: PairwiseIntersectionType::Overlap,
            segment: None,
            overlap: Some(OverlapInfo {
                other_polygon_idx: 1,
                other_edges: vec![x0.inverted(), y1.clone()],
                other_support: support,
            }),
        },
    ];

    assert_eq!(unique_overlap_edge_planes(&intersections), vec![x0, y0, y1]);
}

#[test]
fn support_cell_targets_include_direct_strict_feasibility_witness() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let direct = p(2, 1, 3);
    let report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(direct.clone(), [None, None, None]);
    let seeds = strict_support_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();
    let mut saw_unknown = false;

    let targets = deferred_direct_reference_targets_from_strict_seeds(
        &seeds,
        Some(&direct),
        &halfspaces,
        &mut saw_unknown,
    )
    .unwrap();

    assert!(!saw_unknown);
    assert!(targets.iter().any(|target| target.point == direct));
    assert!(
        targets
            .iter()
            .find(|target| target.point == direct)
            .is_some_and(|target| !target.definitions.is_empty())
    );
}

#[test]
fn strict_projected_cell_seeds_include_strict_feasible_vertices() {
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

    let seeds = strict_projected_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    assert_eq!(seeds, vec![Point3::new(r(1), r(2), r(3))]);
}

#[test]
fn strict_projected_cell_seeds_include_strict_geometry_seeds() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(Point3::origin(), [None, None, None]);
    let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
    let tetra_center = p(1, 1, 1);

    let seeds = strict_projected_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    assert!(point_strictly_inside_projected_cell(&triangle_center, &bounds, &halfspaces).unwrap());
    assert!(point_strictly_inside_projected_cell(&tetra_center, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &triangle_center));
    assert!(seeds.iter().any(|seed| seed == &tetra_center));
}

#[test]
fn projected_cell_seed_families_track_unknown_after_boundary_vertex_candidate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut saw_unknown = false;

    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        projected_cell_seed_families_from_optional_report(
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
fn strict_projected_cell_seeds_include_strict_geometry_seeds_without_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
    let tetra_center = p(1, 1, 1);

    let seeds =
        strict_projected_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(point_strictly_inside_projected_cell(&triangle_center, &bounds, &halfspaces).unwrap());
    assert!(point_strictly_inside_projected_cell(&tetra_center, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &triangle_center));
    assert!(seeds.iter().any(|seed| seed == &tetra_center));
}

#[test]
fn strict_projected_cell_seeds_include_strict_edge_midpoints() {
    let (bounds, halfspaces, midpoint) = quadrilateral_reference_cell_fixture();

    let seeds =
        strict_projected_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(point_strictly_inside_projected_cell(&midpoint, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &midpoint));
}

#[test]
fn strict_projected_cell_seeds_include_strict_five_vertex_centroids() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let five_vertex_center = Point3::new(q(8, 5), q(8, 5), q(8, 5));

    let seeds =
        strict_projected_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(
        point_strictly_inside_projected_cell(&five_vertex_center, &bounds, &halfspaces).unwrap()
    );
    assert!(seeds.iter().any(|seed| seed == &five_vertex_center));
}

#[test]
fn report_free_projected_cell_prefers_canonical_all_vertex_centroid() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let seeds =
        strict_projected_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert_eq!(seeds.first(), Some(&p(2, 2, 2)));
    assert!(point_strictly_inside_projected_cell(&seeds[0], &bounds, &halfspaces).unwrap());
}

#[test]
fn point3_seed_collection_backtracks_after_uncertified_candidate() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut points = Vec::new();

    extend_point3_backtracking_unknown(
        &mut points,
        vec![first.clone(), second.clone()],
        |candidate| {
            if candidate == &first {
                Err(crate::error::HypermeshError::UnknownClassification)
            } else {
                Ok(candidate == &second)
            }
        },
    )
    .unwrap();

    assert_eq!(points, vec![second]);
}

#[test]
fn point3_seed_collection_reports_unknown_if_all_candidates_are_uncertified() {
    let first = p(1, 1, 1);
    let second = p(2, 2, 2);
    let mut points = Vec::new();

    let err = extend_point3_backtracking_unknown(&mut points, vec![first, second], |_candidate| {
        Err(crate::error::HypermeshError::UnknownClassification)
    })
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn feasible_support_cell_vertices_backtrack_after_uncertified_candidate() {
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

    let vertices = feasible_support_cell_vertices_with_contains(&halfspaces, |point, _| {
        if point == &first {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            Ok(point == &second)
        }
    })
    .unwrap();

    assert_eq!(vertices, vec![second]);
}

#[test]
fn feasible_support_cell_vertex_family_tracks_unknown_after_later_vertex() {
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

    let family = feasible_support_cell_vertex_family_with_contains(&halfspaces, |point, _| {
        if point == &first {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            Ok(point == &second)
        }
    })
    .unwrap();

    assert_eq!(family.points, vec![second]);
    assert!(family.saw_unknown);
}

#[test]
fn feasible_support_cell_vertices_report_unknown_if_all_candidates_are_uncertified() {
    let halfspaces = vec![
        axis_halfspace(0, true, r(0)),
        axis_halfspace(0, false, r(0)),
        axis_halfspace(1, true, r(0)),
        axis_halfspace(1, false, r(0)),
        axis_halfspace(2, true, r(0)),
        axis_halfspace(2, false, r(1)),
    ];

    let err = feasible_support_cell_vertices_with_contains(&halfspaces, |_point, _| {
        Err(crate::error::HypermeshError::UnknownClassification)
    })
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn support_cell_geometry_seed_candidates_from_vertices_matches_direct_query() {
    let halfspaces = aabb_core_halfspaces(&Aabb::new(p(0, 0, 0), p(4, 4, 4))).unwrap();
    let vertices = feasible_support_cell_vertices(&halfspaces).unwrap();

    let from_vertices = support_cell_geometry_seed_candidates_from_vertices(&vertices).unwrap();
    let direct = support_cell_geometry_seed_candidates(&halfspaces).unwrap();

    assert_eq!(from_vertices, direct);
}

#[test]
fn point3_centroid_subset_family_tracks_unknown_after_later_centroid() {
    let vertices = vec![p(0, 0, 0), p(2, 0, 0), p(4, 0, 0)];
    let blocked_subset = vec![vertices[0].clone(), vertices[1].clone()];

    let family = point3_centroid_subset_family_from_vertices_with(&vertices, |subset| {
        if subset == blocked_subset.as_slice() {
            Err(crate::error::HypermeshError::UnknownClassification)
        } else {
            point3_centroid(subset)
        }
    })
    .unwrap();

    assert!(family.saw_unknown);
    assert!(!family.points.is_empty());
}

#[test]
fn strict_projected_cell_targets_include_direct_strict_seed_targets() {
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

    let targets = strict_projected_cell_targets(&bounds, &halfspaces, &report).unwrap();

    assert!(
        targets
            .iter()
            .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
    );
    assert!(
        targets
            .iter()
            .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
            .is_some_and(|target| !target.definitions.is_empty())
    );
}

#[test]
fn strict_projected_cell_targets_include_direct_strict_seed_targets_without_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(3)),
        axis_halfspace(2, false, r(3)),
    ];

    let targets =
        strict_projected_cell_targets_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(
        targets
            .iter()
            .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
    );
    assert!(
        targets
            .iter()
            .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
            .is_some_and(|target| !target.definitions.is_empty())
    );
}

#[test]
fn strict_support_cell_seeds_include_strict_feasible_vertices() {
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

    let seeds = strict_support_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    assert_eq!(seeds, vec![Point3::new(r(1), r(2), r(3))]);
}

#[test]
fn strict_support_cell_seeds_include_strict_geometry_seeds() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let report =
        hyperlimit::HalfspaceFeasibilityReport::feasible(Point3::origin(), [None, None, None]);
    let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
    let tetra_center = p(1, 1, 1);

    let seeds = strict_support_cell_seeds_from_report(&bounds, &halfspaces, &report).unwrap();

    assert!(point_strictly_inside_support_cell(&triangle_center, &bounds, &halfspaces).unwrap());
    assert!(point_strictly_inside_support_cell(&tetra_center, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &triangle_center));
    assert!(seeds.iter().any(|seed| seed == &tetra_center));
}

#[test]
fn support_cell_seed_families_track_unknown_after_boundary_vertex_candidate() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let mut saw_unknown = false;

    let (strict_seeds, shifted_vertices, shifted_geometry_seeds) =
        support_cell_seed_families_from_optional_report(
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
fn strict_support_cell_seeds_include_strict_geometry_seeds_without_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let triangle_center = Point3::new(q(4, 3), q(4, 3), q(8, 3));
    let tetra_center = p(1, 1, 1);

    let seeds = strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(point_strictly_inside_support_cell(&triangle_center, &bounds, &halfspaces).unwrap());
    assert!(point_strictly_inside_support_cell(&tetra_center, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &triangle_center));
    assert!(seeds.iter().any(|seed| seed == &tetra_center));
}

#[test]
fn strict_support_cell_seeds_include_strict_edge_midpoints() {
    let (bounds, halfspaces, midpoint) = quadrilateral_reference_cell_fixture();

    let seeds = strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(point_strictly_inside_support_cell(&midpoint, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &midpoint));
}

#[test]
fn strict_support_cell_seeds_include_strict_five_vertex_centroids() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let five_vertex_center = Point3::new(q(8, 5), q(8, 5), q(8, 5));

    let seeds = strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert!(point_strictly_inside_support_cell(&five_vertex_center, &bounds, &halfspaces).unwrap());
    assert!(seeds.iter().any(|seed| seed == &five_vertex_center));
}

#[test]
fn report_free_support_cell_prefers_canonical_all_vertex_centroid() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let seeds = strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();

    assert_eq!(seeds.first(), Some(&p(2, 2, 2)));
    assert!(point_strictly_inside_support_cell(&seeds[0], &bounds, &halfspaces).unwrap());
}

#[test]
fn support_cell_targets_try_each_deferred_shift_seed_family() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let strict_a = p(1, 1, 1);
    let strict_b = p(2, 2, 2);
    let shifted_vertex = p(3, 1, 1);
    let shifted_geometry = p(1, 3, 1);
    let visited = std::cell::RefCell::new(Vec::new());
    let mut saw_unknown = false;

    let targets = strict_support_cell_targets_from_seed_families_with_tracking_unknown(
        &bounds,
        &halfspaces,
        None,
        vec![strict_a.clone(), strict_b.clone()],
        vec![shifted_vertex.clone()],
        vec![shifted_geometry.clone()],
        &mut saw_unknown,
        |seed| {
            visited.borrow_mut().push(seed.clone());
            Ok(vec![ReferenceTarget::axis_defined(seed.clone())])
        },
    )
    .unwrap();

    assert!(!saw_unknown);
    assert_eq!(
        visited.into_inner(),
        vec![strict_a, strict_b, shifted_vertex, shifted_geometry]
    );
    assert_eq!(targets.len(), 4);
}

#[test]
fn support_cell_targets_include_direct_strict_feasibility_witness_without_report() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = vec![
        axis_halfspace(0, true, r(1)),
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, true, r(2)),
        axis_halfspace(1, false, r(2)),
        axis_halfspace(2, true, r(3)),
        axis_halfspace(2, false, r(3)),
    ];
    let seeds = strict_support_cell_seeds_from_optional_report(&bounds, &halfspaces, None).unwrap();
    let mut saw_unknown = false;

    let targets = deferred_direct_reference_targets_from_strict_seeds(
        &seeds,
        None,
        &halfspaces,
        &mut saw_unknown,
    )
    .unwrap();

    assert!(!saw_unknown);
    assert!(
        targets
            .iter()
            .any(|target| target.point == Point3::new(r(1), r(2), r(3)))
    );
    assert!(
        targets
            .iter()
            .find(|target| target.point == Point3::new(r(1), r(2), r(3)))
            .is_some_and(|target| !target.definitions.is_empty())
    );
}

#[test]
fn shifted_support_cell_targets_try_all_shifted_strict_seeds() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let targets =
        shifted_support_cell_targets_from_seed(&bounds, &halfspaces, &p(2, 1, 3)).unwrap();

    assert!(
        targets
            .iter()
            .any(|target| { target.point == Point3::new(r(1), q(1, 2), q(3, 2)) })
    );
    assert!(targets.iter().all(|target| !target.definitions.is_empty()));
}

#[test]
fn shifted_projected_cell_targets_from_geometry_seed_return_targets() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let targets =
        shifted_projected_cell_targets_from_seed(&bounds, &halfspaces, &p(1, 1, 1)).unwrap();

    assert!(!targets.is_empty());
    assert!(targets.iter().all(|target| {
        point_strictly_inside_projected_cell(&target.point, &bounds, &halfspaces).unwrap()
    }));
}

#[test]
fn shifted_support_cell_targets_from_geometry_seed_return_targets() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();

    let targets =
        shifted_support_cell_targets_from_seed(&bounds, &halfspaces, &p(1, 1, 1)).unwrap();

    assert!(!targets.is_empty());
    assert!(targets.iter().all(|target| {
        point_strictly_inside_support_cell(&target.point, &bounds, &halfspaces).unwrap()
    }));
}

#[test]
fn support_cell_targets_include_shifted_targets_without_centroid_seed() {
    let bounds = Aabb::new(p(0, 0, 0), p(4, 4, 4));
    let halfspaces = aabb_core_halfspaces(&bounds).unwrap();
    let direct = p(2, 1, 3);

    let targets = shifted_support_cell_targets_from_seed(&bounds, &halfspaces, &direct).unwrap();

    assert!(
        targets
            .iter()
            .filter(|target| target.point != direct)
            .any(|target| !target.definitions.is_empty())
    );
}

#[test]
fn support_reference_definitions_include_non_basis_active_halfspaces() {
    let witness = p(1, 1, 1);
    let halfspaces = vec![
        axis_halfspace(0, false, r(1)),
        axis_halfspace(1, false, r(1)),
        axis_halfspace(2, false, r(1)),
        LimitPlane3::new(p(1, 1, 1), r(-3)),
    ];

    let definitions = reference_definitions_from_active_halfspaces(
        &witness,
        &halfspaces,
        [Some(0), Some(1), Some(2)],
    )
    .unwrap();

    assert!(
        definitions
            .definitions
            .iter()
            .any(definition_uses_non_axis_plane)
    );
    for definition in &definitions.definitions {
        assert_eq!(affine_from_planes(definition).unwrap(), witness);
    }
}

#[test]
fn reference_propagation_reports_unknown_for_uncertain_exhausted_construction() {
    let bounds = Aabb::new(p(0, 0, 0), p(0, 0, 0));
    let polygons = vec![support_only_polygon(Plane::axis_aligned(0, r(0)))];

    let err = compute_new_reference(
        &p(-1, -1, -1),
        &axis_defs(&p(-1, -1, -1)),
        &[0],
        &bounds,
        &polygons,
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
}

#[test]
fn subdivide_into_keeps_output_unchanged_on_uncertified_failure() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let sentinel = ClassifiedPolygon::new(
        make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
        1,
    );
    let mut output = vec![sentinel.clone()];

    let err = subdivide_into(
        SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
        &indicator,
        SubdivisionConfig { max_depth: 0 },
        &mut output,
    )
    .unwrap_err();

    assert_eq!(
        err,
        crate::error::HypermeshError::SubdivisionDepthLimit {
            depth: 0,
            polygon_count: 1
        }
    );
    assert_eq!(output, vec![sentinel]);
}

#[test]
fn unsplittable_task_requires_certified_leaf_completion() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(1, 0, 0), p(1, 0, 0));
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let mut output = Vec::new();
    let emitted = ClassifiedPolygon::new(wall.clone(), 1);
    let caches = SubdivisionRuntimeCaches::default();

    let err = subdivide_into_inner_with(
        SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
        &indicator,
        SubdivisionConfig { max_depth: 4 },
        None,
        &mut output,
        &mut |_task, _indicator, out| {
            out.push(emitted.clone());
            Ok(LeafProcessingStats {
                polygon_count: 1,
                certified_complete: false,
                ..LeafProcessingStats::default()
            })
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    assert!(output.is_empty());
}

#[test]
fn unsplittable_task_accepts_certified_leaf_completion() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(1, 0, 0), p(1, 0, 0));
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let mut output = Vec::new();
    let emitted = ClassifiedPolygon::new(wall.clone(), 1);
    let caches = SubdivisionRuntimeCaches::default();

    subdivide_into_inner_with(
        SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
        &indicator,
        SubdivisionConfig { max_depth: 4 },
        None,
        &mut output,
        &mut |_task, _indicator, out| {
            out.push(emitted.clone());
            Ok(LeafProcessingStats {
                polygon_count: 1,
                certified_complete: true,
                ..LeafProcessingStats::default()
            })
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap();

    assert_eq!(output, vec![emitted]);
}

#[test]
fn subdivision_normalizes_reference_definitions_before_leaf_processing() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(1, 0, 0), p(1, 0, 0));
    let ref_point = p(0, 0, 0);
    let mut task = SubdivisionTask::new(vec![wall], bounds, ref_point.clone(), vec![0]);
    task.ref_definitions = vec![axis_plane_definition(&p(9, 9, 9))];
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let caches = SubdivisionRuntimeCaches::default();
    let mut output = Vec::new();

    subdivide_into_inner_with(
        task,
        &indicator,
        SubdivisionConfig { max_depth: 4 },
        None,
        &mut output,
        &mut |task, _indicator, _output| {
            assert_eq!(task.ref_definitions, axis_defs(&ref_point));
            Ok(LeafProcessingStats {
                polygon_count: 1,
                certified_complete: true,
                ..LeafProcessingStats::default()
            })
        },
        &caches,
        &caches.winding_reachability,
    )
    .unwrap();
}

#[test]
fn subdivision_keeps_splitting_after_uncertified_leaf_failure() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let sentinel = ClassifiedPolygon::new(
        make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
        1,
    );
    let mut output = vec![sentinel.clone()];

    let err = subdivide_into(
        SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0]),
        &indicator,
        SubdivisionConfig { max_depth: 0 },
        &mut output,
    )
    .unwrap_err();

    assert_eq!(
        err,
        crate::error::HypermeshError::SubdivisionDepthLimit {
            depth: 0,
            polygon_count: 1
        }
    );
    assert_eq!(output, vec![sentinel]);
}

#[test]
fn operation_subdivision_discards_fixed_difference_outside_region() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 1, 0);
    wall.delta_w = vec![0, 1];
    let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Difference, 2);

    let output = subdivide_for_operation(
        SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0, 0]),
        &indicator,
        SubdivisionConfig { max_depth: 0 },
        BooleanOp::Difference,
    )
    .unwrap();

    assert!(output.is_empty());
}

#[test]
fn operation_subdivision_keeps_potential_difference_region() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1, 0];
    let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Difference, 2);

    let err = subdivide_for_operation(
        SubdivisionTask::new(vec![wall], bounds, p(0, 0, 0), vec![0, 0]),
        &indicator,
        SubdivisionConfig { max_depth: 0 },
        BooleanOp::Difference,
    )
    .unwrap_err();

    assert_eq!(
        err,
        crate::error::HypermeshError::SubdivisionDepthLimit {
            depth: 0,
            polygon_count: 1
        }
    );
}

#[test]
fn process_leaf_into_keeps_output_unchanged_on_uncertified_failure() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = Aabb::new(p(1, -1, -1), p(1, 1, 1));
    let indicator = crate::winding::make_indicator(crate::winding::BooleanOp::Union, 1);
    let sentinel = ClassifiedPolygon::new(
        make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 99),
        1,
    );
    let mut output = vec![sentinel.clone()];

    let err = process_leaf_into(
        &[wall],
        &bounds,
        &p(0, 0, 0),
        &axis_defs(&p(0, 0, 0)),
        &[0],
        &indicator,
        &mut output,
    )
    .unwrap_err();

    assert_eq!(err, crate::error::HypermeshError::UnknownClassification);
    assert_eq!(output, vec![sentinel]);
}

#[test]
fn bsp_leaf_certification_rejects_unsplit_interior_segment() {
    let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    host.delta_w = vec![1, 0];
    let mut cutter = make_triangle(&p(1, 0, -1), &p(1, 0, 1), &p(1, 2, 0), 1, 0);
    cutter.delta_w = vec![0, 1];
    let polygons = vec![host.clone(), cutter];

    assert!(
        !certify_bsp_leaf_has_no_interior_intersections(&host, &host.edges, &polygons).unwrap()
    );
}

#[test]
fn bsp_leaf_certification_rejects_boundary_ambiguous_overlap() {
    let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 1, 0);
    host.delta_w = vec![0, 1];
    let mut overlap = make_triangle(
        &p(0, 0, 0),
        &Point3::new(q(4, 3), r(0), r(0)),
        &Point3::new(r(0), q(4, 3), r(0)),
        0,
        0,
    );
    overlap.delta_w = vec![1, 0];
    let polygons = vec![host.clone(), overlap];

    assert!(
        !certify_bsp_leaf_has_no_interior_intersections(&host, &host.edges, &polygons).unwrap()
    );
}

#[test]
fn segment_interval_witness_finds_strict_overlap_when_midpoint_is_on_boundary() {
    let left = make_triangle(&p(1, -1, 0), &p(3, -1, 0), &p(1, 1, 0), 0, 0);
    let right = make_triangle(&p(0, -2, 0), &p(4, -2, 0), &p(0, 2, 0), 1, 0);

    assert!(
        segment_has_strict_interior_point_in_both(&p(0, 0, 0), &p(2, 0, 0), &left, &right).unwrap()
    );
}

fn support_only_polygon(support: Plane) -> ConvexPolygon {
    ConvexPolygon {
        support,
        edges: Vec::new().into(),
        mesh_index: 0,
        polygon_index: 0,
        delta_w: Vec::new(),
        approx_bounds: None,
    }
}
