use bevy::math::Vec3;

pub(crate) fn triangle_cube_intersection(triangle: (Vec3, Vec3, Vec3), cube: (Vec3, Vec3)) -> bool {
    let (tv0, tv1, tv2) = triangle;
    let (cube_min, cube_max) = cube;

    // Calculate the bounding box of the triangle
    let tri_min = tv0.min(tv1).min(tv2);
    let tri_max = tv0.max(tv1).max(tv2);

    // Early exit: Check if the triangle's bounding box overlaps with the cube's bounding box
    if tri_max.x < cube_min.x
        || tri_min.x > cube_max.x
        || tri_max.y < cube_min.y
        || tri_min.y > cube_max.y
        || tri_max.z < cube_min.z
        || tri_min.z > cube_max.z
    {
        return false;
    }

    // Check if any of the triangle's vertices are inside the cube
    if point_in_cube(tv0, cube) || point_in_cube(tv1, cube) || point_in_cube(tv2, cube) {
        // println!("Triangle vertex inside cube");
        return true;
    }

    // Check if any of the cube's vertices are inside the triangle
    let cube_vertices = [
        Vec3::new(cube_min.x, cube_min.y, cube_min.z),
        Vec3::new(cube_max.x, cube_min.y, cube_min.z),
        Vec3::new(cube_max.x, cube_max.y, cube_min.z),
        Vec3::new(cube_min.x, cube_max.y, cube_min.z),
        Vec3::new(cube_min.x, cube_min.y, cube_max.z),
        Vec3::new(cube_max.x, cube_min.y, cube_max.z),
        Vec3::new(cube_max.x, cube_max.y, cube_max.z),
        Vec3::new(cube_min.x, cube_max.y, cube_max.z),
    ];

    for &vertex in &cube_vertices {
        if point_in_triangle(vertex, triangle) {
            // println!("Cube vertex inside triangle");
            return true;
        }
    }

    // Check if any of the edges of the triangle intersect with any of the edges of the cube
    let triangle_edges = [(tv0, tv1), (tv1, tv2), (tv2, tv0)];

    let cube_edges = [
        (
            Vec3::new(cube_min.x, cube_min.y, cube_min.z),
            Vec3::new(cube_max.x, cube_min.y, cube_min.z),
        ),
        (
            Vec3::new(cube_max.x, cube_min.y, cube_min.z),
            Vec3::new(cube_max.x, cube_max.y, cube_min.z),
        ),
        (
            Vec3::new(cube_max.x, cube_max.y, cube_min.z),
            Vec3::new(cube_min.x, cube_max.y, cube_min.z),
        ),
        (
            Vec3::new(cube_min.x, cube_max.y, cube_min.z),
            Vec3::new(cube_min.x, cube_min.y, cube_min.z),
        ),
        (
            Vec3::new(cube_min.x, cube_min.y, cube_max.z),
            Vec3::new(cube_max.x, cube_min.y, cube_max.z),
        ),
        (
            Vec3::new(cube_max.x, cube_min.y, cube_max.z),
            Vec3::new(cube_max.x, cube_max.y, cube_max.z),
        ),
        (
            Vec3::new(cube_max.x, cube_max.y, cube_max.z),
            Vec3::new(cube_min.x, cube_max.y, cube_max.z),
        ),
        (
            Vec3::new(cube_min.x, cube_max.y, cube_max.z),
            Vec3::new(cube_min.x, cube_min.y, cube_max.z),
        ),
        (
            Vec3::new(cube_min.x, cube_min.y, cube_min.z),
            Vec3::new(cube_min.x, cube_min.y, cube_max.z),
        ),
        (
            Vec3::new(cube_max.x, cube_min.y, cube_min.z),
            Vec3::new(cube_max.x, cube_min.y, cube_max.z),
        ),
        (
            Vec3::new(cube_max.x, cube_max.y, cube_min.z),
            Vec3::new(cube_max.x, cube_max.y, cube_max.z),
        ),
        (
            Vec3::new(cube_min.x, cube_max.y, cube_min.z),
            Vec3::new(cube_min.x, cube_max.y, cube_max.z),
        ),
    ];

    for &triangle_edge in &triangle_edges {
        for &cube_edge in &cube_edges {
            if edge_intersection(triangle_edge, cube_edge) {
                // println!("Edge intersection found");
                return true;
            }
        }
    }

    false
}

pub(crate) fn point_in_cube(point: Vec3, cube: (Vec3, Vec3)) -> bool {
    let (cube_min, cube_max) = cube;

    point.x >= cube_min.x
        && point.x <= cube_max.x
        && point.y >= cube_min.y
        && point.y <= cube_max.y
        && point.z >= cube_min.z
        && point.z <= cube_max.z
}

pub(crate) fn point_in_triangle(point: Vec3, triangle: (Vec3, Vec3, Vec3)) -> bool {
    let (tv0, tv1, tv2) = triangle;

    // Compute vectors for two edges of the triangle
    let edge1 = tv1 - tv0;
    let edge2 = tv2 - tv0;
    let point_vec = point - tv0;

    let normal2 = edge1.cross(edge2);

    // Compute the normal of the triangle using cross product of edge1 and edge2
    let normal = Vec3::new(
        edge1.y * edge2.z - edge1.z * edge2.y,
        edge1.z * edge2.x - edge1.x * edge2.z,
        edge1.x * edge2.y - edge1.y * edge2.x,
    );

    assert_eq!(normal, normal2);

    // Check if point is co-planar with the triangle by checking the dot product
    let dot = point_vec.dot(normal);
    if dot.abs() > 1e-6 {
        return false; // The point is not in the plane of the triangle
    }

    // Now that the point is in the same plane, compute barycentric coordinates
    let dot00 = edge1.x * edge1.x + edge1.y * edge1.y + edge1.z * edge1.z;
    let dot01 = edge1.x * edge2.x + edge1.y * edge2.y + edge1.z * edge2.z;
    let dot02 = edge1.x * point_vec.x + edge1.y * point_vec.y + edge1.z * point_vec.z;
    let dot11 = edge2.x * edge2.x + edge2.y * edge2.y + edge2.z * edge2.z;
    let dot12 = edge2.x * point_vec.x + edge2.y * point_vec.y + edge2.z * point_vec.z;

    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is inside the triangle using barycentric coordinates
    (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0)
}

pub(crate) fn edge_intersection(edge1: (Vec3, Vec3), edge2: (Vec3, Vec3)) -> bool {
    // x1 = e1v0
    // x2 = e1v1
    // x3 = e2v0
    // x4 = e2v1
    let (e1v0, e1v1) = edge1;
    let (e2v0, e2v1) = edge2;

    // Direction vectors
    let d1 = e1v1 - e1v0;
    let d2 = e2v1 - e2v0;

    // Vector between the first points of each edge
    let r = e2v0 - e1v0;

    // Cross product of direction vectors (used to detect parallelism)
    let cross_d1_d2 = d1.cross(d2);

    // If the cross product is zero, the edges are parallel
    if cross_d1_d2.x.abs() < f32::EPSILON
        && cross_d1_d2.y.abs() < f32::EPSILON
        && cross_d1_d2.z.abs() < f32::EPSILON
    {
        // println!("Edges are parallel");
        return false;
    }

    // Compute cross product of r and d2
    let cross_r_d2 = r.cross(d2);

    // Compute t1 using the cross products
    let t1 = (cross_r_d2.x * cross_d1_d2.x
        + cross_r_d2.y * cross_d1_d2.y
        + cross_r_d2.z * cross_d1_d2.z)
        / (cross_d1_d2.x * cross_d1_d2.x
            + cross_d1_d2.y * cross_d1_d2.y
            + cross_d1_d2.z * cross_d1_d2.z);

    // Calculate the intersection point for edge1
    let intersect1 = Vec3::new(e1v0.x + t1 * d1.x, e1v0.y + t1 * d1.y, e1v0.z + t1 * d1.z);

    // Check if this point lies within edge2
    let t2 = if d2.x != 0.0 {
        (intersect1.x - e2v0.x) / d2.x
    } else if d2.y != 0.0 {
        (intersect1.y - e2v0.y) / d2.y
    } else {
        (intersect1.z - e2v0.z) / d2.z
    };

    // Ensure both t1 and t2 are within the bounds [0, 1] of their respective edges
    let result = (0.0..=1.0).contains(&t1) && (0.0..=1.0).contains(&t2);
    // println!(
    //     "edge_intersection: edge1={:?}, edge2={:?}, t1={}, t2={}, result={}",
    //     edge1, edge2, t1, t2, result
    // );
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_in_cube() {
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

        // Inside the cube
        assert!(point_in_cube(Vec3::new(0.5, 0.5, 0.5), cube));

        // On the edges
        assert!(point_in_cube(Vec3::new(0.0, 0.5, 0.5), cube));
        assert!(point_in_cube(Vec3::new(1.0, 0.5, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 1.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.5, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.5, 1.0), cube));

        // On the faces
        assert!(point_in_cube(Vec3::new(0.0, 0.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(1.0, 1.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.5, 1.0, 1.0), cube));

        // At the vertices
        assert!(point_in_cube(Vec3::new(0.0, 0.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 0.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.0, 1.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.0, 0.0, 1.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 1.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 0.0, 1.0), cube));
        assert!(point_in_cube(Vec3::new(0.0, 1.0, 1.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 1.0, 1.0), cube));

        // Outside the cube
        assert!(!point_in_cube(Vec3::new(-0.1, 0.5, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(1.1, 0.5, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(0.5, -0.1, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(0.5, 1.1, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(0.5, 0.5, -0.1), cube));
        assert!(!point_in_cube(Vec3::new(0.5, 0.5, 1.1), cube));
    }

    #[test]
    fn test_point_in_triangle() {
        let triangle = (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );

        // Inside the triangle
        assert!(point_in_triangle(Vec3::new(0.25, 0.25, 0.0), triangle));

        // On the edges
        assert!(point_in_triangle(Vec3::new(0.5, 0.0, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(0.0, 0.5, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(0.5, 0.5, 0.0), triangle));

        // On the vertices
        assert!(point_in_triangle(Vec3::new(0.0, 0.0, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(1.0, 0.0, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(0.0, 1.0, 0.0), triangle));

        // Outside the triangle but in the same plane
        assert!(!point_in_triangle(Vec3::new(1.0, 1.0, 0.0), triangle));
        assert!(!point_in_triangle(Vec3::new(-0.1, 0.5, 0.0), triangle));
        assert!(!point_in_triangle(Vec3::new(0.5, -0.1, 0.0), triangle));
        assert!(!point_in_triangle(Vec3::new(0.5, 0.5, 0.0), triangle));

        // Outside the triangle and not in the same plane
        assert!(!point_in_triangle(Vec3::new(0.25, 0.25, 1.0), triangle));
        assert!(!point_in_triangle(Vec3::new(0.5, 0.5, 1.0), triangle));
        assert!(!point_in_triangle(Vec3::new(1.0, 1.0, 1.0), triangle));
    }

    #[test]
    fn test_edge_intersection() {
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        let edge2 = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 1.0));
        assert!(edge_intersection(edge1, edge2)); // should intersect

        let edge3 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge4 = (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
        assert!(!edge_intersection(edge3, edge4)); // should not intersect
    }

    #[test]
    fn test_triangle_cube_intersection() {
        let triangle = (
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(1.5, 0.5, 0.5),
            Vec3::new(0.5, 1.5, 0.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));

        let triangle2 = (
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(2.5, 1.5, 1.5),
            Vec3::new(1.5, 2.5, 1.5),
        );
        assert!(!triangle_cube_intersection(triangle2, cube));
    }
}
