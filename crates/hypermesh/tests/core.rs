use hyperlattice::{Point3, Real};
use hypermesh::bvh::bounds_overlap;
use hypermesh::clip::{ClipSide, clip_polygon};
use hypermesh::{
    BooleanOp, Classification, ClassifiedPolygon, EmberConfig, HypermeshError, Plane, PolygonSoup,
    SubdivisionConfig, SubdivisionTask, Triangle, WindingPair, boolean_operation,
    boolean_operation_refs, classify_leaf_polygon, classify_point, classify_polygon_output,
    extract_output, find_probe_point, intersect_polygons, make_indicator, make_quad, make_triangle,
    prepare_input, prepare_input_meshes, process_leaf, process_leaf_into, subdivide,
    trace_axis_segment, trace_segment, triangulate_and_resolve, triangulate_and_resolve_certified,
    triangulate_output,
};

fn r(value: i32) -> Real {
    value.into()
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn cube_mesh(min: i32, max: i32) -> hypermesh::InputMesh {
    let mut mesh = hypermesh::InputMesh::new(
        vec![
            p(min, min, min),
            p(max, min, min),
            p(max, max, min),
            p(min, max, min),
            p(min, min, max),
            p(max, min, max),
            p(max, max, max),
            p(min, max, max),
        ],
        vec![
            Triangle::new(4, 5, 6),
            Triangle::new(4, 6, 7),
            Triangle::new(0, 3, 2),
            Triangle::new(0, 2, 1),
            Triangle::new(1, 2, 6),
            Triangle::new(1, 6, 5),
            Triangle::new(0, 4, 7),
            Triangle::new(0, 7, 3),
            Triangle::new(3, 7, 6),
            Triangle::new(3, 6, 2),
            Triangle::new(0, 1, 5),
            Triangle::new(0, 5, 4),
        ],
    );
    mesh.nsi = true;
    mesh.nnc = true;
    mesh
}

fn ov(x: i32, y: i32, z: i32) -> hypermesh::OutputVertex {
    hypermesh::OutputVertex {
        x: r(x),
        y: r(y),
        z: r(z),
    }
}

fn assert_triangle_soup_within_bounds(
    soup: &hypermesh::TriangleSoup,
    min: i32,
    max: i32,
) -> hypermesh::HypermeshResult<()> {
    let bounds = hypermesh::Aabb::new(p(min, min, min), p(max, max, max));
    for vertex in &soup.vertices {
        assert!(
            bounds.contains_point(&Point3::new(
                vertex.x.clone(),
                vertex.y.clone(),
                vertex.z.clone(),
            ))?,
            "vertex ({}, {}, {}) is outside [{}, {}]^3",
            vertex.x,
            vertex.y,
            vertex.z,
            min,
            max
        );
    }
    Ok(())
}

fn vertex_axis(vertex: &hypermesh::OutputVertex, axis: usize) -> &Real {
    match axis {
        0 => &vertex.x,
        1 => &vertex.y,
        2 => &vertex.z,
        _ => unreachable!("axis must be in 0..3"),
    }
}

fn assert_triangle_soup_on_cube_boundary(soup: &hypermesh::TriangleSoup, min: i32, max: i32) {
    assert!(!soup.triangles.is_empty());
    let min = r(min);
    let max = r(max);

    for triangle in &soup.triangles {
        let vertices = [
            &soup.vertices[triangle[0]],
            &soup.vertices[triangle[1]],
            &soup.vertices[triangle[2]],
        ];
        let on_boundary = (0..3).any(|axis| {
            vertices
                .iter()
                .all(|vertex| vertex_axis(vertex, axis) == &min)
                || vertices
                    .iter()
                    .all(|vertex| vertex_axis(vertex, axis) == &max)
        });
        assert!(on_boundary);
    }
}

#[test]
fn winding_indicators_match_boolean_semantics() {
    let union = make_indicator(BooleanOp::Union, 2);
    let intersection = make_indicator(BooleanOp::Intersection, 2);
    let difference = make_indicator(BooleanOp::Difference, 2);
    let xor = make_indicator(BooleanOp::SymmetricDifference, 2);

    assert!(union(&[1, 0]));
    assert!(!union(&[0, 0]));
    assert!(intersection(&[1, 1]));
    assert!(!intersection(&[1, 0]));
    assert!(difference(&[1, 0]));
    assert!(!difference(&[1, 1]));
    assert!(xor(&[1, 0]));
    assert!(!xor(&[1, 1]));

    assert_eq!(classify_polygon_output(&[0, 0], &[1, 0], &union), 1);
    assert_eq!(classify_polygon_output(&[1, 0], &[0, 0], &union), -1);
    assert_eq!(classify_polygon_output(&[1, 0], &[1, 1], &union), 0);
}

#[test]
fn triangle_plane_and_vertices_are_exact_reals() {
    let tri = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    assert!(tri.is_valid());
    assert_eq!(tri.vertex_count(), 3);
    assert_eq!(
        classify_point(&p(0, 0, 1), &tri.support).unwrap(),
        Classification::Positive
    );

    let vertices = tri.vertices().unwrap();
    assert!(vertices.contains(&p(0, 0, 0)));
    assert!(vertices.contains(&p(1, 0, 0)));
    assert!(vertices.contains(&p(0, 1, 0)));
}

#[test]
fn borrowed_prepare_input_builds_polygon_soup() {
    let positions = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
    let triangles = vec![Triangle::new(0, 1, 2)];

    let soup = prepare_input(&positions, &triangles).unwrap();
    assert_eq!(soup.num_meshes, 1);
    assert_eq!(soup.polygons.len(), 1);
    assert_eq!(soup.polygons[0].delta_w, vec![1]);
    assert_eq!(soup.polygons[0].mesh_index, 0);
}

#[test]
fn owned_prepare_input_delegates_to_borrowed_api() {
    let mesh = hypermesh::InputMesh::new(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)],
        vec![Triangle::new(0, 1, 2)],
    );

    let borrowed = prepare_input(&mesh.positions, &mesh.triangles).unwrap();
    let owned = prepare_input_meshes(&[mesh]).unwrap();
    assert_eq!(owned.polygons, borrowed.polygons);
}

#[test]
fn clipping_triangle_against_axis_plane_splits_both_sides() {
    let tri = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let split = Plane::axis_aligned(0, r(1));
    let clipped = clip_polygon(&tri, &split).unwrap();

    assert_eq!(clipped.side, ClipSide::Both);
    assert!(clipped.left.vertex_count() >= 3);
    assert!(clipped.right.vertex_count() >= 3);
}

#[test]
fn intersecting_non_coplanar_triangles_produce_segment() {
    let xy = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let yz = make_triangle(&p(0, 0, -1), &p(0, 0, 1), &p(0, 2, 0), 1, 0);

    let intersection = intersect_polygons(&xy, &yz, 1).unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::PairwiseIntersectionType::Segment
    );
    let segment = intersection.segment.unwrap();
    assert_eq!(segment.other_polygon_idx, 1);
    assert!(
        [segment.v0, segment.v1]
            .into_iter()
            .all(|point| point.x.definitely_zero())
    );
}

#[test]
fn coplanar_overlapping_triangles_report_overlap() {
    let a = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let b = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 1, 0);

    let intersection = intersect_polygons(&a, &b, 3).unwrap();
    assert_eq!(
        intersection.kind,
        hypermesh::PairwiseIntersectionType::Overlap
    );
    assert_eq!(intersection.overlap.unwrap().other_polygon_idx, 3);
}

#[test]
fn coplanar_crossing_quads_report_overlap_without_contained_vertices() {
    let horizontal = make_quad(&p(-2, -1, 0), &p(2, -1, 0), &p(2, 1, 0), &p(-2, 1, 0), 0, 0);
    let vertical = make_quad(&p(-1, -2, 0), &p(1, -2, 0), &p(1, 2, 0), &p(-1, 2, 0), 1, 0);

    let intersection = intersect_polygons(&horizontal, &vertical, 7).unwrap();

    assert_eq!(
        intersection.kind,
        hypermesh::PairwiseIntersectionType::Overlap
    );
    assert_eq!(intersection.overlap.unwrap().other_polygon_idx, 7);
}

#[test]
fn boolean_operation_refs_validates_before_shortcuts() {
    let empty = hypermesh::MeshRef {
        positions: &[],
        triangles: &[],
        nsi: false,
        nnc: false,
    };
    assert!(matches!(
        boolean_operation_refs(&[empty], BooleanOp::Union, EmberConfig::default()),
        Err(hypermesh::HypermeshError::EmptyInput)
    ));

    let positions = vec![p(0, 0, 0), p(1, 0, 0)];
    let triangles = vec![Triangle::new(0, 1, 2)];
    let invalid = hypermesh::MeshRef {
        positions: &positions,
        triangles: &triangles,
        nsi: false,
        nnc: false,
    };
    assert!(matches!(
        boolean_operation_refs(&[invalid], BooleanOp::Union, EmberConfig::default()),
        Err(hypermesh::HypermeshError::VertexIndexOutOfBounds {
            index: 2,
            vertex_count: 2
        })
    ));
}

#[test]
fn local_bsp_splits_leaf_with_intersection_segment() {
    let host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    let cutter = make_triangle(&p(1, 0, -1), &p(1, 0, 1), &p(1, 2, 0), 1, 0);
    let segment = intersect_polygons(&host, &cutter, 1)
        .unwrap()
        .segment
        .unwrap();

    let mut bsp = hypermesh::LocalBsp::new(&host);
    bsp.add_segment(&segment).unwrap();
    let leaves = bsp.collect_leaves();

    assert_eq!(leaves.len(), 2);
    assert_eq!(bsp.node_count(), 3);
    assert!(leaves.iter().all(|leaf| leaf.edges.len() >= 3));
}

#[test]
fn local_bsp_overlap_can_disable_higher_index_duplicate_leaf() {
    let lower = make_triangle(&p(0, 0, 0), &p(4, 0, 0), &p(0, 4, 0), 0, 0);
    let higher = make_triangle(&p(1, 1, 0), &p(2, 1, 0), &p(1, 2, 0), 1, 2);
    let intersection = intersect_polygons(&higher, &lower, 0).unwrap();
    let overlap = intersection.overlap.as_ref().unwrap();

    let mut bsp = hypermesh::LocalBsp::new(&higher);
    bsp.add_overlap(&lower, overlap).unwrap();

    assert!(bsp.collect_leaves().is_empty());
}

#[test]
fn output_extraction_and_triangulation_use_real_vertices() {
    let positions = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
    let triangles = vec![Triangle::new(0, 1, 2)];
    let soup = prepare_input(&positions, &triangles).unwrap();
    let result = hypermesh::BooleanResult::new(soup, vec![1]);

    let polygons = extract_output(&result).unwrap();
    assert_eq!(polygons.len(), 1);
    assert_eq!(polygons[0].vertices.len(), 3);
    assert!(polygons[0].vertices.iter().any(|vertex| vertex.x == r(1)));

    let triangulated = triangulate_output(&result).unwrap();
    assert_eq!(triangulated.vertices.len(), 3);
    assert_eq!(triangulated.triangles, vec![[0, 1, 2]]);
}

#[test]
fn exact_bvh_reports_overlapping_polygon_bounds() {
    let left = vec![
        make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0),
        make_triangle(&p(10, 0, 0), &p(11, 0, 0), &p(10, 1, 0), 0, 1),
    ];
    let right = vec![make_triangle(&p(1, 0, 0), &p(3, 0, 0), &p(1, 2, 0), 1, 0)];

    let left_bvh = hypermesh::ExactBvh::build(&left).unwrap();
    let right_bvh = hypermesh::ExactBvh::build(&right).unwrap();
    let mut pairs = Vec::new();
    left_bvh
        .intersect_pairs(&right_bvh, |left, right| pairs.push((left, right)))
        .unwrap();

    assert_eq!(pairs, vec![(0, 0)]);
    assert!(
        bounds_overlap(
            left[0].approx_bounds.as_ref().unwrap(),
            right[0].approx_bounds.as_ref().unwrap()
        )
        .unwrap()
    );
}

#[test]
fn trace_axis_segment_accumulates_exact_winding_crossing() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];

    let traced = trace_axis_segment(&p(0, 0, 0), &p(2, 0, 0), 0, &[0], &[wall]).unwrap();

    assert!(traced.valid);
    assert_eq!(traced.winding, vec![-1]);
}

#[test]
fn trace_segment_uses_axis_orderings_for_l_path() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];

    let winding = trace_segment(&p(0, 0, 0), &p(2, 0, 0), &[0], &[wall]).unwrap();
    assert_eq!(winding, vec![-1]);
}

#[test]
fn trace_segment_rejects_surface_intermediate_points() {
    let blockers = vec![
        make_triangle(&p(2, 0, 0), &p(3, 0, 0), &p(2, 1, 0), 0, 0),
        make_triangle(&p(0, 2, 0), &p(1, 2, 0), &p(0, 3, 0), 0, 1),
        make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 0, 2),
    ];

    let err = trace_segment(&p(0, 0, 0), &p(2, 2, 2), &[0], &blockers).unwrap_err();
    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn leaf_classification_traces_to_probe_and_returns_winding_vector() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = hypermesh::Aabb::new(p(-2, -2, -2), p(3, 3, 3));

    let probe = find_probe_point(&wall).unwrap().unwrap();
    assert_ne!(probe.1, Classification::On);

    let winding = classify_leaf_polygon(
        &wall.support,
        &wall.edges,
        &p(0, 0, 0),
        &[0],
        &[wall.clone()],
        &bounds,
        &wall.delta_w,
    )
    .unwrap();
    assert_eq!(winding.len(), 1);
}

#[test]
fn leaf_classification_reports_unknown_when_no_valid_probe_path_exists() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = hypermesh::Aabb::new(p(1, -1, -1), p(1, 1, 1));

    let err = classify_leaf_polygon(
        &wall.support,
        &wall.edges,
        &p(0, 0, 0),
        &[0],
        &[wall.clone()],
        &bounds,
        &wall.delta_w,
    )
    .unwrap_err();
    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn process_leaf_classifies_direct_nsi_polygon_slice() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    wall.no_self_intersections = true;
    wall.no_nested_components = true;
    let bounds = hypermesh::Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let union = make_indicator(BooleanOp::Union, 1);

    let output = process_leaf(&[wall], &bounds, &p(0, 0, 0), &[0], &union).unwrap();

    assert_eq!(output.len(), 1);
    assert_ne!(output[0].classification, 0);
    assert!(!output[0].is_bsp_fragment);
    assert!(output[0].winding.is_some());
}

#[test]
fn process_leaf_uses_bsp_for_intersecting_cross_mesh_polygons() {
    let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    host.delta_w = vec![1, 0];
    let mut cutter = make_triangle(&p(1, 0, -1), &p(1, 0, 1), &p(1, 2, 0), 1, 1);
    cutter.delta_w = vec![0, 1];
    let polygons = vec![host, cutter];
    let bounds = hypermesh::Aabb::new(p(-1, -1, -2), p(3, 3, 2));
    let union = make_indicator(BooleanOp::Union, 2);
    let mut output = Vec::new();

    let stats = process_leaf_into(
        &polygons,
        &bounds,
        &p(-1, -1, -1),
        &[0, 0],
        &union,
        &mut output,
    )
    .unwrap();

    assert_eq!(stats.polygon_count, 2);
    assert!(stats.intersection_count >= 2);
    assert!(stats.bsp_fragment_count > 0);
    assert!(output.iter().any(|polygon| polygon.is_bsp_fragment));
}

#[test]
fn boolean_operation_refs_runs_leaf_pipeline_from_borrowed_meshes() {
    let positions = vec![p(1, -1, -1), p(1, 1, -1), p(1, 0, 1)];
    let triangles = vec![Triangle::new(0, 1, 2)];
    let mesh = hypermesh::MeshRef {
        positions: &positions,
        triangles: &triangles,
        nsi: true,
        nnc: true,
    };

    let result = boolean_operation_refs(&[mesh], BooleanOp::Union, EmberConfig::default()).unwrap();

    assert_eq!(result.classifications.len(), result.output.polygons.len());
    assert!(!result.output.polygons.is_empty());
}

#[test]
fn boolean_operation_owned_delegates_to_borrowed_pipeline() {
    let mut mesh = hypermesh::InputMesh::new(
        vec![p(1, -1, -1), p(1, 1, -1), p(1, 0, 1)],
        vec![Triangle::new(0, 1, 2)],
    );
    mesh.nsi = true;
    mesh.nnc = true;

    let result = boolean_operation(&[mesh], BooleanOp::Union, EmberConfig::default()).unwrap();

    assert!(!result.output.polygons.is_empty());
    assert!(
        result
            .classifications
            .iter()
            .all(|classification| *classification != 0)
    );
}

#[test]
fn boolean_operation_reports_unknown_when_max_depth_forces_oversized_leaf() {
    let mesh = cube_mesh(0, 2);
    let soup = prepare_input_meshes(&[mesh]).unwrap();
    let indicator = make_indicator(BooleanOp::Union, soup.num_meshes);
    let config = SubdivisionConfig {
        leaf_threshold: 0,
        max_depth: 0,
        use_early_termination: false,
    };

    assert!(matches!(
        subdivide(
            SubdivisionTask::new(
                soup.polygons,
                hypermesh::Aabb::new(p(-1, -1, -1), p(3, 3, 3)),
                p(-1, -1, -1),
                vec![0; soup.num_meshes],
            ),
            &indicator,
            config,
        ),
        Err(HypermeshError::UnknownClassification)
    ));
}

#[test]
fn resolve_tjunctions_merges_duplicate_vertices_and_faces_exactly() {
    let soup = hypermesh::TriangleSoup {
        vertices: vec![ov(0, 0, 0), ov(1, 0, 0), ov(0, 1, 0), ov(1, 0, 0)],
        triangles: vec![[0, 1, 2], [0, 3, 2]],
    };

    let resolved = hypermesh::resolve_tjunctions(&soup).unwrap();

    assert_eq!(resolved.vertices.len(), 3);
    assert_eq!(resolved.triangles.len(), 1);
}

#[test]
fn certified_triangulation_rejects_open_output_without_repair() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let result = hypermesh::BooleanResult::new(
        hypermesh::PolygonSoup {
            polygons: vec![polygon],
            bounds: hypermesh::Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            num_meshes: 1,
        },
        vec![1],
    );

    let raw = triangulate_output(&result).unwrap();
    assert!(!hypermesh::triangle_soup_is_closed(&raw));

    let err = triangulate_and_resolve_certified(&result).unwrap_err();
    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn boolean_result_preserves_classified_winding_evidence() {
    let polygon = make_triangle(&p(0, 0, 0), &p(1, 0, 0), &p(0, 1, 0), 0, 0);
    let mut classified = ClassifiedPolygon::new(polygon, 1);
    classified.winding = Some(WindingPair {
        w_front: vec![0],
        w_back: vec![1],
    });

    let result = hypermesh::BooleanResult::from_classified(
        PolygonSoup {
            polygons: Vec::new(),
            bounds: hypermesh::Aabb::new(p(0, 0, 0), p(1, 1, 0)),
            num_meshes: 1,
        },
        vec![classified],
    );

    assert_eq!(result.winding_pairs.len(), 1);
    assert_eq!(
        result.winding_pairs[0],
        Some(WindingPair {
            w_front: vec![0],
            w_back: vec![1],
        })
    );
}

#[test]
fn resolve_tjunctions_splits_exact_boundary_tjunction() {
    let soup = hypermesh::TriangleSoup {
        vertices: vec![ov(0, 0, 0), ov(2, 0, 0), ov(0, 2, 0), ov(1, 0, 0)],
        triangles: vec![[0, 1, 2]],
    };

    let resolved = hypermesh::resolve_tjunctions(&soup).unwrap();

    assert_eq!(resolved.vertices.len(), 4);
    assert_eq!(resolved.triangles.len(), 2);
    assert!(
        resolved
            .triangles
            .iter()
            .any(|triangle| triangle.contains(&3))
    );
}

#[test]
fn disjoint_cube_booleans_have_expected_polygon_counts() {
    let cube_a = cube_mesh(0, 2);
    let cube_b = cube_mesh(4, 6);
    let config = EmberConfig {
        assume_nsi: true,
        assume_nnc: true,
        use_early_termination: false,
        ..EmberConfig::default()
    };

    let union = hypermesh::boolean_union(&cube_a, &cube_b, config).unwrap();
    assert_eq!(union.output.polygons.len(), 24);
    assert_eq!(triangulate_output(&union).unwrap().triangles.len(), 24);

    let intersection = hypermesh::boolean_intersection(&cube_a, &cube_b, config).unwrap();
    assert!(intersection.output.polygons.is_empty());

    let difference = hypermesh::boolean_difference(&cube_a, &cube_b, config).unwrap();
    assert_eq!(difference.output.polygons.len(), 12);
    assert_eq!(triangulate_output(&difference).unwrap().triangles.len(), 12);
}

#[test]
fn overlapping_cube_booleans_clip_and_resolve_exactly() {
    let cube_a = cube_mesh(0, 2);
    let cube_b = cube_mesh(1, 3);
    let config = EmberConfig {
        leaf_threshold: 1,
        max_depth: 6,
        assume_nsi: true,
        assume_nnc: true,
        use_early_termination: false,
        use_fast_paths: false,
    };

    let union = hypermesh::boolean_union(&cube_a, &cube_b, config).unwrap();
    let union_soup = triangulate_and_resolve(&union).unwrap();
    assert!(!union.output.polygons.is_empty());
    assert!(!union_soup.triangles.is_empty());
    assert_triangle_soup_within_bounds(&union_soup, 0, 3).unwrap();

    let intersection = hypermesh::boolean_intersection(&cube_a, &cube_b, config).unwrap();
    let intersection_soup = triangulate_and_resolve(&intersection).unwrap();
    assert!(intersection.output.polygons.len() >= 12);
    assert_triangle_soup_within_bounds(&intersection_soup, 1, 2).unwrap();
    assert_triangle_soup_on_cube_boundary(&intersection_soup, 1, 2);

    let difference = hypermesh::boolean_difference(&cube_a, &cube_b, config).unwrap();
    let difference_soup = triangulate_and_resolve(&difference).unwrap();
    assert!(!difference.output.polygons.is_empty());
    assert!(!difference_soup.triangles.is_empty());
    assert_triangle_soup_within_bounds(&difference_soup, 0, 2).unwrap();
}
