#![no_main]

use hypermesh::clip::{clip_polygon, clip_polygon_to_aabb};
use hypermesh::{
    Aabb, Classification, Plane, Point3, Real, classify_point, make_quad, make_triangle,
};
use libfuzzer_sys::fuzz_target;

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
    let triangle = make_triangle(&p(-4, -3, 0), &p(5, -3, 0), &p(-4, 6, 0), 0, 0);
    let quad = make_quad(&p(-4, -3, 0), &p(5, -3, 0), &p(5, 6, 0), &p(-4, 6, 0), 0, 1);
    assert!(triangle.is_valid());
    assert!(quad.is_valid());
    assert_eq!(triangle.vertices().unwrap().len(), 3);
    assert_eq!(quad.vertices().unwrap().len(), 4);

    let split = Plane::axis_aligned(split_axis, r(split_value));
    let expression = split.expression_at_point(&query);
    let classification = classify_point(&query, &split).unwrap();
    assert_eq!(
        classification == Classification::On,
        expression.definitely_zero()
    );
    assert_eq!(split.inverted().inverted(), split);
    assert_eq!(split.axis_split_value(), Some((split_axis, r(split_value))));
    let _ = split.as_projective();

    let clipped = clip_polygon(&triangle, &split).unwrap();
    assert!(clipped.left.vertex_count() == 0 || clipped.left.is_valid());
    assert!(clipped.right.vertex_count() == 0 || clipped.right.is_valid());

    let extent = i64::from(data[5] % 4) + 1;
    let bounds = Aabb::new(p(-extent, -extent, -1), p(extent, extent, 1));
    let _ = bounds.extent(split_axis);
    let _ = bounds.midpoint(split_axis);
    let _ = bounds.splitting_plane(split_axis);
    let _ = bounds.longest_axis().unwrap();
    let _ = bounds.contains_point(&query).unwrap();
    let left = bounds.left_half(split_axis, r(0));
    let right = bounds.right_half(split_axis, r(0));
    let left_boundary = [&left.max.x, &left.max.y, &left.max.z][split_axis];
    let right_boundary = [&right.min.x, &right.min.y, &right.min.z][split_axis];
    assert_eq!(left_boundary, right_boundary);

    let clipped_to_bounds = clip_polygon_to_aabb(&quad, &bounds).unwrap();
    assert!(clipped_to_bounds.vertex_count() == 0 || clipped_to_bounds.is_valid());

    let vertex = triangle.vertex(usize::from(data[6] % 3));
    assert!(triangle.contains_point(&vertex).unwrap());
    let _ = triangle.contains_point_strictly(&vertex).unwrap();
    let inverted = triangle.inverted();
    assert!(inverted.is_valid());
    assert_eq!(
        inverted.inverted().vertices().unwrap(),
        triangle.vertices().unwrap()
    );
});
