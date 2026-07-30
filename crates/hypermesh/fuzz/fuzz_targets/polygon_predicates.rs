#![no_main]

mod support;

use hypermesh::clip::{clip_polygon, clip_polygon_to_aabb};
use hypermesh::{
    Aabb, Classification, Plane, Point3, Real, classify_point, convex_quad, convex_triangle,
};
use libfuzzer_sys::fuzz_target;
use support::{CONTEXT, value};

fn r(value: i64) -> Real {
    Real::from(value)
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
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
});
