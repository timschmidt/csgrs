use hyperlattice::{Point3, Real};
use hypermesh::bvh::bounds_overlap;
use hypermesh::clip::{ClipSide, clip_polygon};
use hypermesh::{
    BooleanOp, Classification, EmberConfig, HypermeshError, MeshRef, Plane, SubdivisionConfig,
    SubdivisionTask, Triangle, boolean_operation, classify_leaf_polygon, classify_point,
    classify_polygon_output, intersect_polygons, make_indicator, make_quad, make_triangle,
    prepare_input, process_leaf_into, subdivide, trace_axis_segment, trace_segment,
    triangulate_and_resolve_certified,
};

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn axis_defs(point: &Point3) -> [[Plane; 3]; 1] {
    [[
        Plane::axis_aligned(0, point.x.clone()),
        Plane::axis_aligned(1, point.y.clone()),
        Plane::axis_aligned(2, point.z.clone()),
    ]]
}

fn px(x: Real, y: i32, z: i32) -> Point3 {
    Point3::new(x, r(y), r(z))
}

fn tetra_mesh() -> hypermesh::InputMesh {
    hypermesh::InputMesh::new(
        vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0), p(0, 0, 1)],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

fn tetra_from_face_and_apex(a: Point3, b: Point3, c: Point3, apex: Point3) -> hypermesh::InputMesh {
    hypermesh::InputMesh::new(
        vec![a, b, c, apex],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

fn prepared_axis_face(
    polygons: &[hypermesh::ConvexPolygon],
    axis: usize,
    value: i32,
) -> hypermesh::ConvexPolygon {
    polygons
        .iter()
        .find(|polygon| {
            polygon.vertices().unwrap().iter().all(|vertex| match axis {
                0 => vertex.x == r(value),
                1 => vertex.y == r(value),
                2 => vertex.z == r(value),
                _ => unreachable!("axis must be in 0..3"),
            })
        })
        .cloned()
        .expect("expected prepared polygon on requested axis-aligned face")
}

fn cube_mesh(min: i32, max: i32) -> hypermesh::InputMesh {
    hypermesh::InputMesh::new(
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
    )
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
    let mesh = tetra_mesh();

    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    assert_eq!(soup.num_meshes, 1);
    assert_eq!(soup.polygons.len(), 4);
    assert!(
        soup.polygons
            .iter()
            .all(|polygon| polygon.delta_w == vec![1] && polygon.mesh_index == 0)
    );
}

#[test]
fn prepare_input_rejects_empty_mesh_views() {
    assert!(matches!(
        prepare_input(&[]),
        Err(hypermesh::HypermeshError::EmptyInput)
    ));

    let positions = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
    let empty = MeshRef {
        positions: &positions,
        triangles: &[],
    };
    assert_eq!(
        prepare_input(&[empty]),
        Err(hypermesh::HypermeshError::EmptyMesh { mesh_index: 0 })
    );
}

#[test]
fn prepare_input_rejects_open_source_meshes() {
    let positions = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
    let triangles = [Triangle::new(0, 1, 2)];

    assert_eq!(
        prepare_input(&[MeshRef {
            positions: &positions,
            triangles: &triangles,
        }]),
        Err(hypermesh::HypermeshError::OpenInput {
            mesh_index: 0,
            boundary_edges: 3
        })
    );
}

#[test]
fn prepare_input_rejects_degenerate_source_triangles() {
    let repeated_positions = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
    let repeated = [Triangle::new(0, 0, 2)];
    assert_eq!(
        prepare_input(&[MeshRef {
            positions: &repeated_positions,
            triangles: &repeated,
        }]),
        Err(hypermesh::HypermeshError::DegenerateTriangle {
            mesh_index: 0,
            triangle_index: 0
        })
    );

    let collinear_positions = vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)];
    let collinear = [Triangle::new(0, 1, 2)];
    assert_eq!(
        prepare_input(&[MeshRef {
            positions: &collinear_positions,
            triangles: &collinear,
        }]),
        Err(hypermesh::HypermeshError::DegenerateTriangle {
            mesh_index: 0,
            triangle_index: 0
        })
    );
}

#[test]
fn prepare_input_accepts_owned_mesh_views() {
    let mesh = tetra_mesh();

    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    assert_eq!(soup.num_meshes, 1);
    assert_eq!(soup.polygons.len(), 4);
    assert!(
        soup.polygons
            .iter()
            .all(|polygon| polygon.delta_w == vec![1])
    );
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
fn coplanar_identical_quads_report_overlap_from_interior_witness() {
    let left = make_quad(&p(-2, -1, 0), &p(2, -1, 0), &p(2, 1, 0), &p(-2, 1, 0), 0, 0);
    let right = make_quad(&p(-2, -1, 0), &p(2, -1, 0), &p(2, 1, 0), &p(-2, 1, 0), 1, 0);

    let intersection = intersect_polygons(&left, &right, 11).unwrap();

    assert_eq!(
        intersection.kind,
        hypermesh::PairwiseIntersectionType::Overlap
    );
    assert_eq!(intersection.overlap.unwrap().other_polygon_idx, 11);
}

#[test]
fn boolean_operation_validates_before_general_path() {
    assert!(matches!(
        boolean_operation(&[], BooleanOp::Union, EmberConfig::default()),
        Err(hypermesh::HypermeshError::EmptyInput)
    ));

    let empty = hypermesh::MeshRef {
        positions: &[],
        triangles: &[],
    };
    assert_eq!(
        boolean_operation(&[empty], BooleanOp::Union, EmberConfig::default()),
        Err(hypermesh::HypermeshError::EmptyMesh { mesh_index: 0 })
    );

    let positions_only = vec![p(0, 0, 0), p(1, 0, 0), p(0, 1, 0)];
    let no_triangles = hypermesh::MeshRef {
        positions: &positions_only,
        triangles: &[],
    };
    assert_eq!(
        boolean_operation(&[no_triangles], BooleanOp::Union, EmberConfig::default()),
        Err(hypermesh::HypermeshError::EmptyMesh { mesh_index: 0 })
    );

    let degenerate_positions = vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)];
    let degenerate_triangles = vec![Triangle::new(0, 1, 2)];
    let degenerate = hypermesh::MeshRef {
        positions: &degenerate_positions,
        triangles: &degenerate_triangles,
    };
    assert_eq!(
        boolean_operation(&[degenerate], BooleanOp::Union, EmberConfig::default()),
        Err(hypermesh::HypermeshError::DegenerateTriangle {
            mesh_index: 0,
            triangle_index: 0
        })
    );

    let positions = vec![p(0, 0, 0), p(1, 0, 0)];
    let triangles = vec![Triangle::new(0, 1, 2)];
    let invalid = hypermesh::MeshRef {
        positions: &positions,
        triangles: &triangles,
    };
    assert!(matches!(
        boolean_operation(&[invalid], BooleanOp::Union, EmberConfig::default()),
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
fn trace_segment_uses_detour_when_axis_order_corners_hit_surfaces() {
    let blockers = vec![
        make_triangle(&p(2, 0, 0), &p(3, 0, 0), &p(2, 1, 0), 0, 0),
        make_triangle(&p(0, 2, 0), &p(1, 2, 0), &p(0, 3, 0), 0, 1),
        make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 0, 2),
    ];

    let winding = trace_segment(&p(0, 0, 0), &p(2, 2, 2), &[0], &blockers).unwrap();
    assert_eq!(winding, vec![0]);
}

#[test]
fn trace_segment_uses_direct_path_when_axis_order_corners_hit_surfaces() {
    let mut blockers = vec![
        make_triangle(&p(2, 0, 0), &p(3, 0, 0), &p(2, 1, 0), 0, 0),
        make_triangle(&p(0, 2, 0), &p(1, 2, 0), &p(0, 3, 0), 0, 1),
        make_triangle(&p(0, 0, 2), &p(1, 0, 2), &p(0, 1, 2), 0, 2),
    ];
    let mut diagonal_wall = make_triangle(&p(3, 0, 0), &p(0, 3, 0), &p(0, 0, 3), 1, 0);
    diagonal_wall.delta_w = vec![1];
    blockers.push(diagonal_wall);

    let winding = trace_segment(&p(0, 0, 0), &p(2, 2, 2), &[0], &blockers).unwrap();
    assert_eq!(winding, vec![-1]);
}

#[test]
fn trace_segment_uses_arrangement_detour_when_fixed_fraction_box_is_blocked() {
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

    let winding = trace_segment(&p(0, 0, 0), &p(4, 4, 4), &[0], &blockers).unwrap();
    assert_eq!(winding, vec![0]);
}

#[test]
fn leaf_classification_traces_to_probe_and_returns_winding_vector() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = hypermesh::Aabb::new(p(-2, -2, -2), p(3, 3, 3));

    let winding = classify_leaf_polygon(
        &wall.support,
        &wall.edges,
        &p(0, 0, 0),
        &axis_defs(&p(0, 0, 0)),
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
        &axis_defs(&p(0, 0, 0)),
        &[0],
        &[wall.clone()],
        &bounds,
        &wall.delta_w,
    )
    .unwrap_err();
    assert_eq!(err, HypermeshError::UnknownClassification);
}

#[test]
fn leaf_classification_places_probe_before_intervening_surface() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let mut blocker = make_triangle(&p(2, -10, -10), &p(2, 10, -10), &p(2, 0, 10), 1, 0);
    blocker.delta_w = vec![1];
    let bounds = hypermesh::Aabb::new(p(1, -2, -2), p(5, 2, 2));

    let winding = classify_leaf_polygon(
        &wall.support,
        &wall.edges,
        &p(0, 0, 0),
        &axis_defs(&p(0, 0, 0)),
        &[0],
        &[wall.clone(), blocker],
        &bounds,
        &wall.delta_w,
    )
    .unwrap();
    assert_eq!(winding, vec![-1]);
}

#[test]
fn process_leaf_classifies_direct_polygon_slice() {
    let mut wall = make_triangle(&p(1, -1, -1), &p(1, 1, -1), &p(1, 0, 1), 0, 0);
    wall.delta_w = vec![1];
    let bounds = hypermesh::Aabb::new(p(-2, -2, -2), p(3, 3, 3));
    let union = make_indicator(BooleanOp::Union, 1);

    let mut output = Vec::new();
    let stats = process_leaf_into(
        &[wall],
        &bounds,
        &p(0, 0, 0),
        &axis_defs(&p(0, 0, 0)),
        &[0],
        &union,
        &mut output,
    )
    .unwrap();

    assert!(stats.certified_complete);
    assert_eq!(stats.intersection_count, 0);
    assert_eq!(stats.direct_polygon_count, 1);
    assert_eq!(stats.bsp_leaf_count, 0);
    assert_eq!(output.len(), 1);
    assert_ne!(output[0].classification(), 0);
    assert!(!output[0].is_bsp_fragment());
    assert!(output[0].winding().is_some());
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
        &axis_defs(&p(-1, -1, -1)),
        &[0, 0],
        &union,
        &mut output,
    )
    .unwrap();

    assert_eq!(stats.polygon_count, 2);
    assert!(stats.intersection_count >= 2);
    assert!(stats.certified_complete);
    assert!(stats.bsp_leaf_count > 0);
    assert!(stats.bsp_fragment_count > 0);
    assert!(output.iter().any(|polygon| polygon.is_bsp_fragment()));
}

#[test]
fn process_leaf_uses_bsp_for_same_mesh_self_intersections() {
    let mut host = make_triangle(&p(0, 0, 0), &p(2, 0, 0), &p(0, 2, 0), 0, 0);
    host.delta_w = vec![1];
    let mut cutter = make_triangle(&p(1, 0, -1), &p(1, 0, 1), &p(1, 2, 0), 0, 1);
    cutter.delta_w = vec![1];
    let polygons = vec![host, cutter];
    let bounds = hypermesh::Aabb::new(p(-1, -1, -2), p(3, 3, 2));
    let union = make_indicator(BooleanOp::Union, 1);
    let mut output = Vec::new();

    let stats = process_leaf_into(
        &polygons,
        &bounds,
        &p(-1, -1, -1),
        &axis_defs(&p(-1, -1, -1)),
        &[0],
        &union,
        &mut output,
    )
    .unwrap();

    assert_eq!(stats.polygon_count, 2);
    assert!(stats.intersection_count >= 2);
    assert!(stats.certified_complete);
    assert!(stats.bsp_leaf_count > 0);
    assert!(stats.bsp_fragment_count > 0);
    assert!(output.iter().any(|polygon| polygon.is_bsp_fragment()));
}

#[test]
fn boolean_operation_runs_leaf_pipeline_from_borrowed_meshes() {
    let cube = cube_mesh(0, 2);
    let mesh_ref = cube.as_ref();
    let mesh = hypermesh::MeshRef {
        positions: mesh_ref.positions,
        triangles: mesh_ref.triangles,
    };

    let result = boolean_operation(&[mesh], BooleanOp::Union, EmberConfig::default()).unwrap();

    assert_eq!(
        result.classifications().len(),
        result.output().polygons.len()
    );
    assert!(!result.output().polygons.is_empty());
    assert!(result.winding_pairs().iter().all(Option::is_some));
}

#[test]
fn boolean_operation_rejects_open_input_before_general_path() {
    let positions = vec![p(1, -1, -1), p(1, 1, -1), p(1, 0, 1)];
    let triangles = vec![Triangle::new(0, 1, 2)];
    let mesh = hypermesh::MeshRef {
        positions: &positions,
        triangles: &triangles,
    };

    let err = boolean_operation(&[mesh], BooleanOp::Union, EmberConfig::default()).unwrap_err();

    assert_eq!(
        err,
        HypermeshError::OpenInput {
            mesh_index: 0,
            boundary_edges: 3
        }
    );
}

#[test]
fn boolean_operation_accepts_input_mesh_refs() {
    let mesh = cube_mesh(0, 2);

    let result =
        boolean_operation(&[mesh.as_ref()], BooleanOp::Union, EmberConfig::default()).unwrap();

    assert!(!result.output().polygons.is_empty());
    assert!(
        result
            .classifications()
            .iter()
            .all(|classification| *classification != 0)
    );
}

#[test]
fn subdivision_processes_certified_leaf_at_max_depth() {
    let mesh = cube_mesh(0, 2);
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    let indicator = make_indicator(BooleanOp::Union, soup.num_meshes);
    let num_meshes = soup.num_meshes;
    let config = SubdivisionConfig { max_depth: 0 };

    let output = subdivide(
        SubdivisionTask::new(
            soup.polygons,
            hypermesh::Aabb::new(p(-1, -1, -1), p(3, 3, 3)),
            p(-1, -1, -1),
            vec![0; num_meshes],
        ),
        &indicator,
        config,
    )
    .unwrap();

    assert_eq!(output.len(), 12);
    assert!(output.iter().all(|polygon| polygon.winding().is_some()));
}

#[test]
fn subdivision_accepts_certified_leaf_before_split() {
    let mesh = cube_mesh(0, 2);
    let soup = prepare_input(&[mesh.as_ref()]).unwrap();
    let indicator = make_indicator(BooleanOp::Union, soup.num_meshes);
    let num_meshes = soup.num_meshes;
    let config = SubdivisionConfig { max_depth: 1 };

    let output = subdivide(
        SubdivisionTask::new(
            soup.polygons,
            hypermesh::Aabb::new(p(-1, -1, -1), p(3, 3, 3)),
            p(-1, -1, -1),
            vec![0; num_meshes],
        ),
        &indicator,
        config,
    )
    .unwrap();

    assert_eq!(output.len(), 12);
    assert!(output.iter().all(|polygon| polygon.winding().is_some()));
}

#[test]
fn subdivision_escapes_projected_reference_on_surface() {
    let mut left = make_triangle(&p(1, 1, 1), &p(1, 5, 1), &p(1, 3, 5), 0, 0);
    left.delta_w = vec![1];
    let mut right = make_triangle(&p(4, 1, 1), &p(4, 5, 1), &p(4, 3, 5), 0, 1);
    right.delta_w = vec![1];
    let indicator = make_indicator(BooleanOp::Union, 1);
    let config = SubdivisionConfig { max_depth: 4 };

    let result = subdivide(
        SubdivisionTask::new(
            vec![left, right],
            hypermesh::Aabb::new(p(0, 0, 0), p(6, 6, 6)),
            p(1, 3, 3),
            vec![0],
        ),
        &indicator,
        config,
    );
    assert!(result.is_ok());
}

#[test]
fn subdivision_escapes_projected_reference_on_surface_for_closed_meshes() {
    let left = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let right = tetra_from_face_and_apex(p(4, 1, 1), p(4, 5, 1), p(4, 3, 5), p(5, 3, 2));
    let soup = prepare_input(&[left.as_ref(), right.as_ref()]).unwrap();
    let indicator = make_indicator(BooleanOp::Union, soup.num_meshes);
    let config = SubdivisionConfig { max_depth: 4 };

    let result = subdivide(
        SubdivisionTask::new(
            soup.polygons,
            hypermesh::Aabb::new(p(0, 0, 0), p(6, 6, 6)),
            p(1, 3, 3),
            vec![0; soup.num_meshes],
        ),
        &indicator,
        config,
    );

    assert!(result.is_ok());
}

#[test]
fn subdivision_projected_reference_surface_case_preserves_boolean_semantics_for_closed_meshes() {
    let left = tetra_from_face_and_apex(p(1, 1, 1), p(1, 5, 1), p(1, 3, 5), p(0, 3, 2));
    let right = tetra_from_face_and_apex(p(4, 1, 1), p(4, 5, 1), p(4, 3, 5), p(5, 3, 2));
    let soup = prepare_input(&[left.as_ref(), right.as_ref()]).unwrap();
    let bounds = hypermesh::Aabb::new(p(0, 0, 0), p(6, 6, 6));
    let ref_point = p(1, 3, 3);
    let ref_wnv = vec![0; soup.num_meshes];
    let cases = [
        (BooleanOp::Union, 8usize),
        (BooleanOp::Intersection, 0usize),
        (BooleanOp::Difference, 4usize),
        (BooleanOp::SymmetricDifference, 8usize),
    ];

    for (op, expected_count) in cases {
        let indicator = make_indicator(op, soup.num_meshes);
        let output = subdivide(
            SubdivisionTask::new(
                soup.polygons.clone(),
                bounds.clone(),
                ref_point.clone(),
                ref_wnv.clone(),
            ),
            &indicator,
            SubdivisionConfig { max_depth: 4 },
        )
        .unwrap_or_else(|err| panic!("{op:?} failed: {err:?}"));

        assert_eq!(output.len(), expected_count, "{op:?}");
        assert!(output.iter().all(|polygon| polygon.winding().is_some()));
    }
}

#[test]
fn subdivision_support_reference_fallback_on_prepared_closed_mesh_faces() {
    let x_mesh = tetra_from_face_and_apex(p(5, 1, 1), p(5, 5, 9), p(5, 9, 1), p(4, 5, 4));
    let y_mesh = tetra_from_face_and_apex(p(1, 5, 1), p(9, 5, 1), p(5, 5, 9), p(5, 4, 4));
    let z_mesh = tetra_from_face_and_apex(p(1, 1, 5), p(5, 9, 5), p(9, 1, 5), p(5, 4, 4));
    let soup = prepare_input(&[x_mesh.as_ref(), y_mesh.as_ref(), z_mesh.as_ref()]).unwrap();
    let polygons = vec![
        prepared_axis_face(&soup.polygons, 0, 5),
        prepared_axis_face(&soup.polygons, 1, 5),
        prepared_axis_face(&soup.polygons, 2, 5),
    ];

    let indicator = |_wnv: &[i32]| true;
    let output = subdivide(
        SubdivisionTask::new(
            polygons,
            hypermesh::Aabb::new(p(0, 0, 0), p(10, 10, 10)),
            p(0, 5, 5),
            vec![0; soup.num_meshes],
        ),
        &indicator,
        SubdivisionConfig { max_depth: 4 },
    )
    .unwrap();

    assert!(!output.is_empty());
    assert!(output.iter().all(|polygon| polygon.winding().is_some()));
}

#[test]
fn disjoint_cube_booleans_have_expected_polygon_counts() {
    let cube_a = cube_mesh(0, 2);
    let cube_b = cube_mesh(4, 6);
    let config = EmberConfig::default();

    let union = hypermesh::boolean_union(cube_a.as_ref(), cube_b.as_ref(), config).unwrap();
    assert_eq!(union.output().polygons.len(), 24);
    assert_eq!(
        triangulate_and_resolve_certified(&union)
            .unwrap()
            .triangles
            .len(),
        24
    );

    let intersection =
        hypermesh::boolean_intersection(cube_a.as_ref(), cube_b.as_ref(), config).unwrap();
    assert!(intersection.output().polygons.is_empty());

    let difference =
        hypermesh::boolean_difference(cube_a.as_ref(), cube_b.as_ref(), config).unwrap();
    assert_eq!(difference.output().polygons.len(), 12);
    assert_eq!(
        triangulate_and_resolve_certified(&difference)
            .unwrap()
            .triangles
            .len(),
        12
    );
}

#[test]
fn overlapping_cube_booleans_use_general_path() {
    let cube_a = cube_mesh(0, 2);
    let cube_b = cube_mesh(1, 3);
    let config = EmberConfig { max_depth: 6 };

    let union = hypermesh::boolean_union(cube_a.as_ref(), cube_b.as_ref(), config).unwrap();
    let union_soup = triangulate_and_resolve_certified(&union).unwrap();
    assert!(!union.output().polygons.is_empty());
    assert!(!union_soup.triangles.is_empty());
    assert_triangle_soup_within_bounds(&union_soup, 0, 3).unwrap();

    let intersection =
        hypermesh::boolean_intersection(cube_a.as_ref(), cube_b.as_ref(), config).unwrap();
    let intersection_soup = triangulate_and_resolve_certified(&intersection).unwrap();
    assert!(intersection.output().polygons.len() >= 12);
    assert_triangle_soup_within_bounds(&intersection_soup, 1, 2).unwrap();
    assert_triangle_soup_on_cube_boundary(&intersection_soup, 1, 2);

    let difference =
        hypermesh::boolean_difference(cube_a.as_ref(), cube_b.as_ref(), config).unwrap();
    let difference_soup = triangulate_and_resolve_certified(&difference).unwrap();
    assert!(!difference.output().polygons.is_empty());
    assert!(!difference_soup.triangles.is_empty());
    assert_triangle_soup_within_bounds(&difference_soup, 0, 2).unwrap();
}
