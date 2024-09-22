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

    // Check if the triangle's plane intersects the cube
    let normal = (tv1 - tv0).cross(tv2 - tv0);
    let d = -normal.dot(tv0);

    let p1 = cube_min;
    let p2 = Vec3::new(cube_max.x, cube_min.y, cube_min.z);
    let p3 = Vec3::new(cube_max.x, cube_max.y, cube_min.z);
    let p4 = Vec3::new(cube_min.x, cube_max.y, cube_min.z);
    let p5 = Vec3::new(cube_min.x, cube_min.y, cube_max.z);
    let p6 = Vec3::new(cube_max.x, cube_min.y, cube_max.z);
    let p7 = cube_max;
    let p8 = Vec3::new(cube_min.x, cube_max.y, cube_max.z);

    let cube_points = [p1, p2, p3, p4, p5, p6, p7, p8];
    let sign = (normal.dot(cube_points[0]) + d).signum();

    for p in &cube_points[1..] {
        let new_sign = (normal.dot(*p) + d).signum();
        if new_sign != sign {
            return true; // The plane intersects the cube
        }
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
    triangle_edge_intersection(edge, (quad.0, quad.1, quad.2)) ||
    triangle_edge_intersection(edge, (quad.0, quad.2, quad.3)) ||

    // Check if any point of the edge is inside the quad
    point_in_quad(edge.0, quad) ||
    point_in_quad(edge.1, quad)
}

fn point_in_quad(point: Vec3, quad: (Vec3, Vec3, Vec3, Vec3)) -> bool {
    // Check if the point is in either triangle of the quad
    point_in_or_on_triangle(point, (quad.0, quad.1, quad.2))
        || point_in_or_on_triangle(point, (quad.0, quad.2, quad.3))
}

fn triangle_edge_intersection(edge: (Vec3, Vec3), triangle: (Vec3, Vec3, Vec3)) -> bool {
    let (e1, e2) = edge;
    let (t1, t2, t3) = triangle;
    let epsilon = 1e-5;

    // Check if any of the triangle vertices lie on the edge
    if point_on_line_segment(t1, edge)
        || point_on_line_segment(t2, edge)
        || point_on_line_segment(t3, edge)
    {
        return true;
    }

    // Check if either endpoint of the edge is inside or on the triangle
    if point_in_or_on_triangle(e1, triangle) || point_in_or_on_triangle(e2, triangle) {
        return true;
    }

    // Compute the triangle's normal
    let normal = (t2 - t1).cross(t3 - t1);

    // If the normal is zero, the triangle is degenerate (a line or point)
    if normal.length_squared() < epsilon {
        // Check if the degenerate triangle overlaps with the edge
        return line_segment_overlap(edge, (t1, t2))
            || line_segment_overlap(edge, (t2, t3))
            || line_segment_overlap(edge, (t3, t1));
    }

    let normal = normal.normalize();
    let d = -normal.dot(t1);

    // Compute the intersection point
    let edge_vec = e2 - e1;
    let denom = normal.dot(edge_vec);

    // If denom is zero, the edge is parallel to the triangle's plane
    if denom.abs() < epsilon {
        // The edge is parallel to the triangle's plane
        // Check if it's coplanar
        if (normal.dot(e1) + d).abs() < epsilon {
            // The edge is coplanar with the triangle
            // Check if it intersects with any of the triangle's edges
            return line_segment_overlap(edge, (t1, t2))
                || line_segment_overlap(edge, (t2, t3))
                || line_segment_overlap(edge, (t3, t1));
        }
        return false;
    }

    let t = -(normal.dot(e1) + d) / denom;

    // Check if the intersection point is on the edge
    if t < 0.0 || t > 1.0 {
        return false;
    }

    // Compute the intersection point
    let intersection = e1 + edge_vec * t;

    // Check if the intersection point is inside or on the triangle
    point_in_or_on_triangle(intersection, triangle)
}

fn point_on_line_segment(point: Vec3, segment: (Vec3, Vec3)) -> bool {
    let (a, b) = segment;
    let epsilon = 1e-5;

    let ab = b - a;
    let ap = point - a;

    // Check if the point is collinear with the line segment
    if ab.cross(ap).length_squared() > epsilon * epsilon {
        return false;
    }

    // Check if the point is within the bounds of the line segment
    let t = ap.dot(ab) / ab.length_squared();
    -epsilon <= t && t <= 1.0 + epsilon
}

fn line_segment_overlap(seg1: (Vec3, Vec3), seg2: (Vec3, Vec3)) -> bool {
    let (a, b) = seg1;
    let (c, d) = seg2;
    let epsilon = 1e-5;

    // Check if the segments are parallel
    let dir1 = (b - a).normalize();
    let dir2 = (d - c).normalize();
    if dir1.cross(dir2).length() > epsilon {
        return false; // Not parallel
    }

    // Project segment endpoints onto the first segment
    let ac = c - a;
    let ad = d - a;
    let ab = b - a;

    let t1 = ac.dot(ab) / ab.length_squared();
    let t2 = ad.dot(ab) / ab.length_squared();

    // Check for overlap
    let tmin = t1.min(t2);
    let tmax = t1.max(t2);

    tmin <= 1.0 + epsilon && tmax >= 0.0 - epsilon
}

#[cfg(test)]
mod tests {
    use super::*;

    mod test_point_in_or_on_cube {
        use super::*;

        #[test]
        fn test_inside_cube() {
            let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

            // Inside the cube
            assert!(point_in_or_on_cube(Vec3::new(0.5, 0.5, 0.5), cube));
        }

        #[test]
        fn test_on_edges() {
            let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

            // On the edges
            assert!(point_in_or_on_cube(Vec3::new(0.0, 0.5, 0.5), cube));
            assert!(point_in_or_on_cube(Vec3::new(1.0, 0.5, 0.5), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.5, 0.0, 0.5), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.5, 1.0, 0.5), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.5, 0.5, 0.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.5, 0.5, 1.0), cube));
        }

        #[test]
        fn test_on_faces() {
            let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

            // On the faces
            assert!(point_in_or_on_cube(Vec3::new(0.0, 0.0, 0.5), cube));
            assert!(point_in_or_on_cube(Vec3::new(1.0, 1.0, 0.5), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.5, 0.0, 0.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.5, 1.0, 1.0), cube));
        }

        #[test]
        fn test_at_vertices() {
            let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

            // At the vertices
            assert!(point_in_or_on_cube(Vec3::new(0.0, 0.0, 0.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(1.0, 0.0, 0.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.0, 1.0, 0.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.0, 0.0, 1.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(1.0, 1.0, 0.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(1.0, 0.0, 1.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(0.0, 1.0, 1.0), cube));
            assert!(point_in_or_on_cube(Vec3::new(1.0, 1.0, 1.0), cube));
        }

        #[test]
        fn test_outside_cube() {
            let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

            // Outside the cube
            assert!(!point_in_or_on_cube(Vec3::new(-0.1, 0.5, 0.5), cube));
            assert!(!point_in_or_on_cube(Vec3::new(1.1, 0.5, 0.5), cube));
            assert!(!point_in_or_on_cube(Vec3::new(0.5, -0.1, 0.5), cube));
            assert!(!point_in_or_on_cube(Vec3::new(0.5, 1.1, 0.5), cube));
            assert!(!point_in_or_on_cube(Vec3::new(0.5, 0.5, -0.1), cube));
            assert!(!point_in_or_on_cube(Vec3::new(0.5, 0.5, 1.1), cube));
        }
    }

    mod test_point_in_or_on_triangle {
        use super::*;

        #[test]
        fn test_inside_triangle() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // Inside the triangle
            assert!(point_in_or_on_triangle(
                Vec3::new(0.25, 0.25, 0.0),
                triangle
            ));
        }

        #[test]
        fn test_on_edges() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // On the edges
            assert!(point_in_or_on_triangle(Vec3::new(0.5, 0.0, 0.0), triangle));
            assert!(point_in_or_on_triangle(Vec3::new(0.0, 0.5, 0.0), triangle));
            assert!(point_in_or_on_triangle(Vec3::new(0.5, 0.5, 0.0), triangle));
        }

        #[test]
        fn test_on_vertices() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // On the vertices
            assert!(point_in_or_on_triangle(Vec3::new(0.0, 0.0, 0.0), triangle));
            assert!(point_in_or_on_triangle(Vec3::new(1.0, 0.0, 0.0), triangle));
            assert!(point_in_or_on_triangle(Vec3::new(0.0, 1.0, 0.0), triangle));
        }

        #[test]
        fn test_outside_triangle_same_plane() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // Outside the triangle but in the same plane
            assert!(!point_in_or_on_triangle(Vec3::new(1.0, 1.0, 0.0), triangle));
            assert!(!point_in_or_on_triangle(
                Vec3::new(-0.1, 0.5, 0.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(
                Vec3::new(0.5, -0.1, 0.0),
                triangle
            ));
        }

        #[test]
        fn test_outside_triangle_not_same_plane() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // Outside the triangle and not in the same plane
            assert!(!point_in_or_on_triangle(
                Vec3::new(0.25, 0.25, 1.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(Vec3::new(0.5, 0.5, 1.0), triangle));
            assert!(!point_in_or_on_triangle(Vec3::new(1.0, 1.0, 1.0), triangle));
        }
    }

    mod test_edge_quad_intersection {
        use super::*;

        #[test]
        fn test_edge_completely_inside_quad() {
            let edge = (Vec3::new(0.25, 0.25, 0.0), Vec3::new(0.75, 0.25, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_completely_outside_quad() {
            let edge = (Vec3::new(1.5, 1.5, 0.0), Vec3::new(2.0, 1.5, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_intersecting_quad_edge() {
            let edge = (Vec3::new(0.5, -0.5, 0.0), Vec3::new(0.5, 0.5, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_intersecting_quad_vertex() {
            let edge = (Vec3::new(-0.5, -0.5, 0.0), Vec3::new(0.0, 0.0, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_intersecting_quad_not_touching_vertices_or_edges() {
            let edge = (Vec3::new(0.5, 0.5, -0.5), Vec3::new(0.5, 0.5, 0.5));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_parallel_to_quad_outside() {
            let edge = (Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 1.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_coinciding_with_quad_edge() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_coinciding_with_quad_vertex() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }
    }

    mod test_triangle_edge_intersection {
        use super::*;

        #[test]
        fn test_triangle_completely_inside_edge() {
            let triangle = (
                Vec3::new(0.25, 0.25, 0.0),
                Vec3::new(0.75, 0.25, 0.0),
                Vec3::new(0.5, 0.75, 0.0),
            );
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_completely_outside_edge() {
            let triangle = (
                Vec3::new(1.5, 1.5, 0.0),
                Vec3::new(2.0, 1.5, 0.0),
                Vec3::new(1.75, 2.0, 0.0),
            );
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(!triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_intersecting_edge_vertex() {
            let triangle = (
                Vec3::new(-0.5, -0.5, 0.0),
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(-0.5, 0.5, 0.0),
            );
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_intersecting_edge_not_touching_vertices() {
            let triangle = (
                Vec3::new(0.5, -0.5, 0.0),
                Vec3::new(0.5, 0.5, 0.0),
                Vec3::new(1.5, 0.0, 0.0),
            );
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_parallel_to_edge_outside() {
            let triangle = (
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 1.0),
                Vec3::new(0.5, 1.0, 1.0),
            );
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(!triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_coinciding_with_edge() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.5, 0.0, 0.0),
            );
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(triangle_edge_intersection(edge, triangle));
        }
    }

    mod test_triangle_cube_intersection {
        use super::*;

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
        fn test_cube_above_triangle_no_intersection() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            let cube = (Vec3::new(0.5, 0.5, 0.5), Vec3::new(1.5, 1.5, 1.5));
            assert!(!triangle_cube_intersection(triangle, cube));
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
    }
}
