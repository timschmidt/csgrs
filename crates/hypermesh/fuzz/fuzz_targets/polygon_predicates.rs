#![no_main]

mod support;

use hypermesh::clip::{clip_polygon, clip_polygon_to_aabb};
use hypermesh::{
    Aabb, Classification, ConvexPolygon, MeshCertainty, MeshContext, PairwiseIntersection, Plane,
    Point3, PredicatePolicy, Real, classify_point, convex_quad, convex_triangle,
    intersect_polygons,
};
use libfuzzer_sys::fuzz_target;
use support::{CONTEXT, value};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn rectangle(context: &MeshContext, bounds: [i64; 4], reverse: bool) -> ConvexPolygon {
    let [x0, x1, y0, y1] = bounds;
    let polygon = convex_quad(
        context,
        &p(x0, y0, 0),
        &p(x1, y0, 0),
        &p(x1, y1, 0),
        &p(x0, y1, 0),
        0,
        0,
    )
    .unwrap()
    .into_value();
    if reverse { polygon.inverted() } else { polygon }
}

fn unordered_segment_matches(actual: [&Point3; 2], expected: [Point3; 2]) -> bool {
    (actual[0] == &expected[0] && actual[1] == &expected[1])
        || (actual[0] == &expected[1] && actual[1] == &expected[0])
}

fn assert_rectangle_intersection(
    context: &MeshContext,
    left: &ConvexPolygon,
    right: &ConvexPolygon,
    left_bounds: [i64; 4],
    right_bounds: [i64; 4],
) {
    const OTHER_POLYGON: usize = 73;
    let outcome = intersect_polygons(context, left, right, OTHER_POLYGON).unwrap();
    assert_eq!(outcome.certainty, MeshCertainty::Certified);

    let [ax0, ax1, ay0, ay1] = left_bounds;
    let [bx0, bx1, by0, by1] = right_bounds;
    let x0 = ax0.max(bx0);
    let x1 = ax1.min(bx1);
    let y0 = ay0.max(by0);
    let y1 = ay1.min(by1);
    match outcome.value {
        PairwiseIntersection::Disjoint => assert!(x0 > x1 || y0 > y1),
        PairwiseIntersection::CoplanarPoint(contact) => {
            assert_eq!(contact.other_polygon_idx, OTHER_POLYGON);
            assert_eq!((x0, y0), (x1, y1));
            assert_eq!(contact.point, p(x0, y0, 0));
        }
        PairwiseIntersection::CoplanarSegment(segment) => {
            assert_eq!(segment.other_polygon_idx, OTHER_POLYGON);
            let expected = if x0 == x1 {
                [p(x0, y0, 0), p(x0, y1, 0)]
            } else {
                assert_eq!(y0, y1);
                [p(x0, y0, 0), p(x1, y0, 0)]
            };
            assert!(unordered_segment_matches([&segment.v0, &segment.v1], expected));
        }
        PairwiseIntersection::CoplanarOverlap(overlap) => {
            assert_eq!(overlap.other_polygon_idx, OTHER_POLYGON);
            assert!(x0 < x1 && y0 < y1);
        }
        PairwiseIntersection::NonCoplanarPoint(_)
        | PairwiseIntersection::NonCoplanarSegment(_) => {
            panic!("coplanar rectangles produced a non-coplanar intersection")
        }
    }
}

fuzz_target!(|data: [u8; 8]| {
    let split_axis = usize::from(data[0] % 3);
    let split_value = i64::from(data[1] % 7) - 3;
    let query = p(
        i64::from(data[2] % 9) - 4,
        i64::from(data[3] % 9) - 4,
        i64::from(data[4] % 9) - 4,
    );
    let triangle = value(convex_triangle(
        &CONTEXT,
        &p(-4, -3, 0),
        &p(5, -3, 0),
        &p(-4, 6, 0),
        0,
        0,
    ))
    .unwrap();
    let quad = value(convex_quad(
        &CONTEXT,
        &p(-4, -3, 0),
        &p(5, -3, 0),
        &p(5, 6, 0),
        &p(-4, 6, 0),
        0,
        1,
    ))
    .unwrap();
    assert!(value(triangle.is_valid(&CONTEXT)).unwrap());
    assert!(value(quad.is_valid(&CONTEXT)).unwrap());
    assert_eq!(triangle.vertices(&CONTEXT).unwrap().into_value().len(), 3);
    assert_eq!(quad.vertices(&CONTEXT).unwrap().into_value().len(), 4);

    let split = Plane::axis_aligned(split_axis, r(split_value));
    let expression = split.expression_at_point(&query);
    let classification = value(classify_point(&CONTEXT, &query, &split)).unwrap();
    assert_eq!(
        classification == Classification::On,
        expression.definitely_zero()
    );
    assert_eq!(split.inverted().inverted(), split);
    assert_eq!(
        value(split.axis_split_value(&CONTEXT)).unwrap(),
        Some((split_axis, r(split_value)))
    );
    let _ = split.as_projective();

    let clipped = value(clip_polygon(&CONTEXT, &triangle, &split)).unwrap();
    assert!(clipped.left.vertex_count() == 0 || value(clipped.left.is_valid(&CONTEXT)).unwrap());
    assert!(clipped.right.vertex_count() == 0 || value(clipped.right.is_valid(&CONTEXT)).unwrap());

    let extent = i64::from(data[5] % 4) + 1;
    let bounds = Aabb::new(p(-extent, -extent, -1), p(extent, extent, 1));
    let _ = bounds.extent(split_axis);
    let _ = bounds.midpoint(split_axis);
    let _ = bounds.splitting_plane(split_axis);
    let _ = value(bounds.longest_axis(&CONTEXT)).unwrap();
    let _ = value(bounds.contains_point(&CONTEXT, &query)).unwrap();
    let left = bounds.left_half(split_axis, r(0));
    let right = bounds.right_half(split_axis, r(0));
    let left_boundary = [&left.max.x, &left.max.y, &left.max.z][split_axis];
    let right_boundary = [&right.min.x, &right.min.y, &right.min.z][split_axis];
    assert_eq!(left_boundary, right_boundary);

    let clipped_to_bounds = value(clip_polygon_to_aabb(&CONTEXT, &quad, &bounds)).unwrap();
    assert!(
        clipped_to_bounds.vertex_count() == 0
            || value(clipped_to_bounds.is_valid(&CONTEXT)).unwrap()
    );

    let vertex = triangle.vertex(usize::from(data[6] % 3));
    assert!(value(triangle.contains_point(&CONTEXT, &vertex)).unwrap());
    let _ = value(triangle.contains_point_strictly(&CONTEXT, &vertex)).unwrap();
    let inverted = triangle.inverted();
    assert!(value(inverted.is_valid(&CONTEXT)).unwrap());
    assert_eq!(
        inverted.inverted().vertices(&CONTEXT).unwrap().into_value(),
        triangle.vertices(&CONTEXT).unwrap().into_value()
    );

    let left_bounds = [
        i64::from(data[0] % 11) - 5,
        i64::from(data[0] % 11) - 4 + i64::from(data[1] % 5),
        i64::from(data[2] % 11) - 5,
        i64::from(data[2] % 11) - 4 + i64::from(data[3] % 5),
    ];
    let right_bounds = [
        i64::from(data[4] % 11) - 5,
        i64::from(data[4] % 11) - 4 + i64::from(data[5] % 5),
        i64::from(data[6] % 11) - 5,
        i64::from(data[6] % 11) - 4 + i64::from(data[7] % 5),
    ];
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let context = MeshContext::new(policy);
        let left = rectangle(&context, left_bounds, data[0] & 1 != 0);
        let right = rectangle(&context, right_bounds, data[1] & 1 != 0);
        if data[2] & 1 == 0 {
            assert_rectangle_intersection(&context, &left, &right, left_bounds, right_bounds);
        } else {
            assert_rectangle_intersection(&context, &right, &left, right_bounds, left_bounds);
        }
    }
});
