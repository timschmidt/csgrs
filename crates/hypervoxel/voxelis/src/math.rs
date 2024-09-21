use bevy::math::Vec3;

pub(crate) fn triangle_cube_intersection(triangle: (Vec3, Vec3, Vec3), cube: (Vec3, Vec3)) -> bool {
    let (tv0, tv1, tv2) = triangle;
    let (cube_min, cube_max) = cube;

    // Calculate the bounding box of the triangle
    let tri_min = tv0.min(tv1).min(tv2);
    let tri_max = tv0.max(tv1).max(tv2);

    // Check if the bounding boxes overlap or touch
    let epsilon = 1e-5;
    if tri_max.x < cube_min.x - epsilon
        || tri_min.x > cube_max.x + epsilon
        || tri_max.y < cube_min.y - epsilon
        || tri_min.y > cube_max.y + epsilon
        || tri_max.z < cube_min.z - epsilon
        || tri_min.z > cube_max.z + epsilon
    {
        return false;
    }

    // Check if any of the triangle's vertices are inside or on the cube
    if point_in_or_on_cube(tv0, cube)
        || point_in_or_on_cube(tv1, cube)
        || point_in_or_on_cube(tv2, cube)
    {
        return true;
    }

    // Check if any of the cube's vertices are inside or on the triangle
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
        if point_in_or_on_triangle(vertex, triangle) {
            return true;
        }
    }

    // Check if any of the triangle's edges intersect with any of the cube's faces
    let triangle_edges = [(tv0, tv1), (tv1, tv2), (tv2, tv0)];
    let cube_faces = [
        (
            cube_vertices[0],
            cube_vertices[1],
            cube_vertices[2],
            cube_vertices[3],
        ), // Front
        (
            cube_vertices[4],
            cube_vertices[5],
            cube_vertices[6],
            cube_vertices[7],
        ), // Back
        (
            cube_vertices[0],
            cube_vertices[1],
            cube_vertices[5],
            cube_vertices[4],
        ), // Bottom
        (
            cube_vertices[2],
            cube_vertices[3],
            cube_vertices[7],
            cube_vertices[6],
        ), // Top
        (
            cube_vertices[0],
            cube_vertices[3],
            cube_vertices[7],
            cube_vertices[4],
        ), // Left
        (
            cube_vertices[1],
            cube_vertices[2],
            cube_vertices[6],
            cube_vertices[5],
        ), // Right
    ];

    for &edge in &triangle_edges {
        for &face in &cube_faces {
            if edge_quad_intersection(edge, face) {
                return true;
            }
        }
    }

    false
}

fn point_in_or_on_cube(point: Vec3, cube: (Vec3, Vec3)) -> bool {
    let (cube_min, cube_max) = cube;
    let epsilon = 1e-5;

    point.x >= cube_min.x - epsilon
        && point.x <= cube_max.x + epsilon
        && point.y >= cube_min.y - epsilon
        && point.y <= cube_max.y + epsilon
        && point.z >= cube_min.z - epsilon
        && point.z <= cube_max.z + epsilon
}

fn point_in_or_on_triangle(point: Vec3, triangle: (Vec3, Vec3, Vec3)) -> bool {
    let (v0, v1, v2) = triangle;
    let epsilon = 1e-5;

    // Check if the point is in the same plane as the triangle
    let normal = (v1 - v0).cross(v2 - v0);
    let distance_to_plane = normal.dot(point - v0);
    if distance_to_plane.abs() > epsilon {
        return false;
    }

    // Compute vectors
    let v0v1 = v1 - v0;
    let v0v2 = v2 - v0;
    let v0p = point - v0;

    // Compute dot products
    let dot00 = v0v1.dot(v0v1);
    let dot01 = v0v1.dot(v0v2);
    let dot02 = v0v1.dot(v0p);
    let dot11 = v0v2.dot(v0v2);
    let dot12 = v0v2.dot(v0p);

    // Compute barycentric coordinates
    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is in or on triangle
    (u >= -epsilon) && (v >= -epsilon) && (u + v <= 1.0 + epsilon)
}

fn edge_quad_intersection(edge: (Vec3, Vec3), quad: (Vec3, Vec3, Vec3, Vec3)) -> bool {
    // Check if the edge intersects with either triangle of the quad
    triangle_edge_intersection(edge, (quad.0, quad.1, quad.2))
        || triangle_edge_intersection(edge, (quad.0, quad.2, quad.3))
}

fn triangle_edge_intersection(edge: (Vec3, Vec3), triangle: (Vec3, Vec3, Vec3)) -> bool {
    let (e1, e2) = edge;
    let (t1, t2, t3) = triangle;

    // Compute the plane's normal and distance
    let edge_vec = e2 - e1;
    let normal = (t2 - t1).cross(t3 - t1).normalize();
    let d = -normal.dot(t1);

    // Compute the t value for the directed line ab intersecting the plane
    let t = -(normal.dot(e1) + d) / normal.dot(edge_vec);

    // If t is in [0..1], the edge intersects the plane
    if t >= 0.0 && t <= 1.0 {
        // Compute the intersection point
        let intersection = e1 + edge_vec * t;

        // Check if the intersection point is inside the triangle
        point_in_triangle(intersection, triangle)
    } else {
        false
    }
}

fn triangle_quad_intersection(
    triangle: (Vec3, Vec3, Vec3),
    quad: (Vec3, Vec3, Vec3, Vec3),
) -> bool {
    // Implement triangle-quad intersection test here
    // This can be done by splitting the quad into two triangles and testing against both
    triangle_triangle_intersection(triangle, (quad.0, quad.1, quad.2))
        || triangle_triangle_intersection(triangle, (quad.0, quad.2, quad.3))
}

fn triangle_triangle_intersection(tri1: (Vec3, Vec3, Vec3), tri2: (Vec3, Vec3, Vec3)) -> bool {
    // Implement triangle-triangle intersection test here
    // This can be done using the Möller–Trumbore intersection algorithm
    // For simplicity, we'll just check if any vertex of one triangle is inside the other
    point_in_triangle(tri1.0, tri2)
        || point_in_triangle(tri1.1, tri2)
        || point_in_triangle(tri1.2, tri2)
        || point_in_triangle(tri2.0, tri1)
        || point_in_triangle(tri2.1, tri1)
        || point_in_triangle(tri2.2, tri1)
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
    let (v0, v1, v2) = triangle;
    let epsilon = 1e-5;

    // Check if the point is in the same plane as the triangle
    let normal = (v1 - v0).cross(v2 - v0);
    let distance_to_plane = normal.dot(point - v0);
    if distance_to_plane.abs() > epsilon {
        return false;
    }

    // Compute vectors
    let v0v1 = v1 - v0;
    let v0v2 = v2 - v0;
    let v0p = point - v0;

    // Compute dot products
    let dot00 = v0v1.dot(v0v1);
    let dot01 = v0v1.dot(v0v2);
    let dot02 = v0v1.dot(v0p);
    let dot11 = v0v2.dot(v0v2);
    let dot12 = v0v2.dot(v0p);

    // Compute barycentric coordinates
    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is in triangle
    (u >= -epsilon) && (v >= -epsilon) && (u + v <= 1.0 + epsilon)
}

pub(crate) fn edge_intersection(edge1: (Vec3, Vec3), edge2: (Vec3, Vec3)) -> bool {
    let (e1v0, e1v1) = edge1;
    let (e2v0, e2v1) = edge2;

    // Direction vectors for both edges
    let d1 = e1v1 - e1v0;
    let d2 = e2v1 - e2v0;

    // Vector from one point of edge1 to one point of edge2
    let r = e2v0 - e1v0;

    // Cross product of direction vectors (to detect parallelism and coplanarity)
    let cross_d1_d2 = d1.cross(d2);
    let denom = cross_d1_d2.length_squared();

    // Relaxed epsilon for floating-point comparisons
    let epsilon = 1e-5;

    // Check if edges are parallel
    if denom < epsilon {
        // Edges are parallel, check if they are collinear (on the same line)
        let cross_r_d1 = r.cross(d1);
        if cross_r_d1.length_squared() > epsilon {
            return false; // Edges are parallel but not collinear
        }

        // Edges are collinear, check if they overlap
        let t0 = r.dot(d1) / d1.length_squared();
        let t1 = (r + d2).dot(d1) / d1.length_squared();
        return (0.0 - epsilon..=1.0 + epsilon).contains(&t0)
            || (0.0 - epsilon..=1.0 + epsilon).contains(&t1)
            || (t0 <= 0.0 && t1 >= 1.0)
            || (t1 <= 0.0 && t0 >= 1.0);
    }

    // Check for coplanarity
    let scalar_triple_product = r.dot(cross_d1_d2);
    if scalar_triple_product.abs() > epsilon * cross_d1_d2.length() {
        return false; // Edges are not coplanar
    }

    // Compute t1 and t2, the parameters for both edges
    let t1 = r.cross(d2).dot(cross_d1_d2) / denom;
    let t2 = r.cross(d1).dot(cross_d1_d2) / denom;

    // Check if the intersection point is within both edges
    let intersects = (0.0 - epsilon..=1.0 + epsilon).contains(&t1)
        && (0.0 - epsilon..=1.0 + epsilon).contains(&t2);

    println!("Intersection result: {}", intersects);
    intersects
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point_in_cube_inside_cube() {
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

        // Inside the cube
        assert!(point_in_cube(Vec3::new(0.5, 0.5, 0.5), cube));
    }

    #[test]
    fn test_point_in_cube_on_edges() {
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

        // On the edges
        assert!(point_in_cube(Vec3::new(0.0, 0.5, 0.5), cube));
        assert!(point_in_cube(Vec3::new(1.0, 0.5, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 1.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.5, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.5, 1.0), cube));
    }

    #[test]
    fn test_point_in_cube_on_faces() {
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

        // On the faces
        assert!(point_in_cube(Vec3::new(0.0, 0.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(1.0, 1.0, 0.5), cube));
        assert!(point_in_cube(Vec3::new(0.5, 0.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.5, 1.0, 1.0), cube));
    }

    #[test]
    fn test_point_in_cube_at_vertices() {
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

        // At the vertices
        assert!(point_in_cube(Vec3::new(0.0, 0.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 0.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.0, 1.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(0.0, 0.0, 1.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 1.0, 0.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 0.0, 1.0), cube));
        assert!(point_in_cube(Vec3::new(0.0, 1.0, 1.0), cube));
        assert!(point_in_cube(Vec3::new(1.0, 1.0, 1.0), cube));
    }

    #[test]
    fn test_point_in_cube_outside_cube() {
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

        // Outside the cube
        assert!(!point_in_cube(Vec3::new(-0.1, 0.5, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(1.1, 0.5, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(0.5, -0.1, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(0.5, 1.1, 0.5), cube));
        assert!(!point_in_cube(Vec3::new(0.5, 0.5, -0.1), cube));
        assert!(!point_in_cube(Vec3::new(0.5, 0.5, 1.1), cube));
    }

    #[test]
    fn test_point_in_triangle_inside_triangle() {
        let triangle = (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );

        // Inside the triangle
        assert!(point_in_triangle(Vec3::new(0.25, 0.25, 0.0), triangle));
    }

    #[test]
    fn test_point_in_triangle_on_edges() {
        let triangle = (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );

        // On the edges
        assert!(point_in_triangle(Vec3::new(0.5, 0.0, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(0.0, 0.5, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(0.5, 0.5, 0.0), triangle));
    }

    #[test]
    fn test_point_in_triangle_on_vertices() {
        let triangle = (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );

        // On the vertices
        assert!(point_in_triangle(Vec3::new(0.0, 0.0, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(1.0, 0.0, 0.0), triangle));
        assert!(point_in_triangle(Vec3::new(0.0, 1.0, 0.0), triangle));
    }

    #[test]
    fn test_point_in_triangle_outside_triangle_same_plane() {
        let triangle = (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );

        // Outside the triangle but in the same plane
        assert!(!point_in_triangle(Vec3::new(1.0, 1.0, 0.0), triangle));
        assert!(!point_in_triangle(Vec3::new(-0.1, 0.5, 0.0), triangle));
        assert!(!point_in_triangle(Vec3::new(0.5, -0.1, 0.0), triangle));
    }

    #[test]
    fn test_point_in_triangle_outside_triangle_not_same_plane() {
        let triangle = (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );

        // Outside the triangle and not in the same plane
        assert!(!point_in_triangle(Vec3::new(0.25, 0.25, 1.0), triangle));
        assert!(!point_in_triangle(Vec3::new(0.5, 0.5, 1.0), triangle));
        assert!(!point_in_triangle(Vec3::new(1.0, 1.0, 1.0), triangle));
    }

    #[test]
    fn test_edge_intersection_parallel() {
        // Parallel edges, no intersection
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge2 = (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
        assert!(!edge_intersection(edge1, edge2));
    }

    #[test]
    fn test_edge_intersection_coincident() {
        // Coincident edges, they overlap completely
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge2 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        assert!(edge_intersection(edge1, edge2));
    }

    #[test]
    fn test_edge_intersection_intersect_at_midpoint() {
        // Edges intersect at a point
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
        let edge2 = (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        assert!(edge_intersection(edge1, edge2)); // Should intersect at (0.5, 0.5, 0.0)
    }

    #[test]
    fn test_edge_intersection_intersect_at_endpoint() {
        // Edges intersect at an endpoint
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
        let edge2 = (Vec3::new(1.0, 1.0, 0.0), Vec3::new(2.0, 2.0, 0.0));
        assert!(edge_intersection(edge1, edge2)); // Intersects at (1.0, 1.0, 0.0)
    }

    #[test]
    fn test_edge_intersection_outside_bounds() {
        // Edges would intersect if extended, but not within their bounds
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.5, 0.5, 0.0));
        let edge2 = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 1.0, 0.0));
        assert!(!edge_intersection(edge1, edge2)); // No intersection within the bounds of the edges
    }

    #[test]
    fn test_edge_intersection_non_coplanar() {
        // Non-coplanar edges, no intersection
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge2 = (Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(!edge_intersection(edge1, edge2)); // Edges are not in the same plane
    }

    #[test]
    fn test_edge_intersection_partial_overlap() {
        // Partially overlapping edges
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge2 = (Vec3::new(0.5, 0.0, 0.0), Vec3::new(1.5, 0.0, 0.0));
        assert!(edge_intersection(edge1, edge2)); // Should overlap at (0.5, 0.0, 0.0) to (1.0, 0.0, 0.0)
    }

    #[test]
    fn test_edge_intersection_touch_at_single_point() {
        // Edges touching at a single point
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge2 = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
        assert!(edge_intersection(edge1, edge2)); // Should intersect at (1.0, 0.0, 0.0)
    }

    #[test]
    fn test_edge_intersection_identical() {
        // Identical edges
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge2 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        assert!(edge_intersection(edge1, edge2)); // Should return true because they are identical
    }

    #[test]
    fn test_edge_intersection_parallel_but_non_intersecting() {
        // Parallel but non-intersecting edges
        let edge1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let edge2 = (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
        assert!(!edge_intersection(edge1, edge2)); // Should return false because they are parallel but non-overlapping
    }

    #[test]
    fn test_triangle_completely_inside_cube() {
        let triangle = (
            Vec3::new(0.25, 0.25, 0.25),
            Vec3::new(0.75, 0.25, 0.25),
            Vec3::new(0.25, 0.75, 0.25),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_completely_outside_cube() {
        let triangle = (
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(2.5, 1.5, 1.5),
            Vec3::new(1.5, 2.5, 1.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(!triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_vertex_inside_cube() {
        let triangle = (
            Vec3::new(-0.5, 0.5, 0.5),
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(1.5, 0.5, 0.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_edge_intersects_cube() {
        let triangle = (
            Vec3::new(-0.5, 0.5, 0.5),
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(0.5, 1.5, 0.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_face_intersects_cube() {
        let triangle = (
            Vec3::new(0.5, 0.5, -0.5),
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(0.5, 1.5, 0.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_cube_vertex_inside_triangle() {
        let triangle = (
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
        );
        let cube = (Vec3::new(0.5, 0.5, 0.5), Vec3::new(1.5, 1.5, 1.5));
        assert!(triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_crosses_cube() {
        // Test case 1: Current triangle crossing the cube
        let triangle = (
            Vec3::new(-0.5, 0.5, 0.5),
            Vec3::new(1.5, 0.5, 0.5),
            Vec3::new(0.5, 2.0, 0.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));

        // Test case 2: Triangle completely outside and above the cube
        let triangle = (
            Vec3::new(0.5, 1.5, 1.5),
            Vec3::new(1.5, 1.5, 1.5),
            Vec3::new(1.0, 2.0, 1.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(!triangle_cube_intersection(triangle, cube));

        // Test case 3: Triangle with one vertex inside the cube
        let triangle = (
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(2.0, 2.0, 2.0),
            Vec3::new(1.5, 1.5, 2.0),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));

        // Test case 4: Triangle parallel to one face of the cube but fully outside
        let triangle = (
            Vec3::new(1.5, 0.5, 0.5),
            Vec3::new(2.5, 0.5, 0.5),
            Vec3::new(2.0, 1.5, 0.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(!triangle_cube_intersection(triangle, cube));

        // Test case 5: Triangle fully inside the cube
        let triangle = (
            Vec3::new(0.2, 0.2, 0.2),
            Vec3::new(0.8, 0.2, 0.2),
            Vec3::new(0.5, 0.8, 0.2),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_no_intersection_triangle_behind_cube() {
        let triangle = (
            Vec3::new(-1.5, -1.5, -1.5),
            Vec3::new(-1.0, -1.5, -1.5),
            Vec3::new(-1.5, -1.0, -1.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(!triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_parallel_to_cube_face_no_intersection() {
        let triangle = (
            Vec3::new(0.0, 0.0, 1.5),
            Vec3::new(1.0, 0.0, 1.5),
            Vec3::new(0.0, 1.0, 1.5),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(!triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_touching_cube_face() {
        let triangle = (
            Vec3::new(0.5, 0.5, 1.0),
            Vec3::new(0.75, 0.25, 1.0),
            Vec3::new(0.25, 0.75, 1.0),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(triangle_cube_intersection(triangle, cube));
    }

    #[test]
    fn test_triangle_overlapping_cube_face_no_intersection() {
        let triangle = (
            Vec3::new(0.5, 0.5, 2.0),
            Vec3::new(1.5, 0.5, 2.0),
            Vec3::new(0.5, 1.5, 2.0),
        );
        let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
        assert!(!triangle_cube_intersection(triangle, cube));
    }

    // #[test]
    // fn kurwa() {
    //     let triangle = (
    //         Vec3::new(2.1, 2.1, 0.0),
    //         Vec3::new(2.1, 0.0, 2.1),
    //         Vec3::new(2.1, 0.0, 0.0),
    //     );
    //     let cube = (Vec3::new(2.0, 1.0, 1.0), Vec3::new(3.0, 2.0, 2.0));
    //     assert!(triangle_cube_intersection(triangle, cube));
    // }
}
