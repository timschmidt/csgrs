pub type Vec3 = bevy::math::DVec3;
pub type Freal = f64;

pub(crate) fn triangle_cube_intersection(triangle: (Vec3, Vec3, Vec3), cube: (Vec3, Vec3)) -> bool {
    let (tv0, tv1, tv2) = triangle;
    let (cube_min, cube_max) = cube;

    // Calculate the bounding box of the triangle
    let tri_min = tv0.min(tv1).min(tv2);
    let tri_max = tv0.max(tv1).max(tv2);

    // Check if the bounding boxes overlap or touch with precision handling
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

    // Define the vertices of the cube once for use throughout the function
    let cube_points = [
        Vec3::new(cube_min.x, cube_min.y, cube_min.z),
        Vec3::new(cube_max.x, cube_min.y, cube_min.z),
        Vec3::new(cube_max.x, cube_max.y, cube_min.z),
        Vec3::new(cube_min.x, cube_max.y, cube_min.z),
        Vec3::new(cube_min.x, cube_min.y, cube_max.z),
        Vec3::new(cube_max.x, cube_min.y, cube_max.z),
        Vec3::new(cube_max.x, cube_max.y, cube_max.z),
        Vec3::new(cube_min.x, cube_max.y, cube_max.z),
    ];
    let sign = (normal.dot(cube_points[0]) + d).signum();

    for p in &cube_points[1..] {
        let new_sign = (normal.dot(*p) + d).signum();
        // Handle near-zero cases to improve intersection detection accuracy
        if (normal.dot(*p) + d).abs() < epsilon {
            continue; // Skip further checks if a point is very close to the plane
        }
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

    for &vertex in &cube_points {
        if point_in_or_on_triangle(vertex, triangle) {
            return true;
        }
    }

    // Check if any of the triangle's edges intersect with any of the cube's faces
    let triangle_edges = [(tv0, tv1), (tv1, tv2), (tv2, tv0)];
    let cube_faces = [
        (
            cube_points[0],
            cube_points[1],
            cube_points[2],
            cube_points[3],
        ), // Front
        (
            cube_points[4],
            cube_points[5],
            cube_points[6],
            cube_points[7],
        ), // Back
        (
            cube_points[0],
            cube_points[1],
            cube_points[5],
            cube_points[4],
        ), // Bottom
        (
            cube_points[2],
            cube_points[3],
            cube_points[7],
            cube_points[6],
        ), // Top
        (
            cube_points[0],
            cube_points[3],
            cube_points[7],
            cube_points[4],
        ), // Left
        (
            cube_points[1],
            cube_points[2],
            cube_points[6],
            cube_points[5],
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

pub(crate) fn point_in_or_on_cube(point: Vec3, cube: (Vec3, Vec3)) -> bool {
    let (cube_min, cube_max) = cube;

    // Calculate a dynamic epsilon based on the size of the cube
    let cube_size = (cube_max - cube_min).length();
    let epsilon = cube_size * 1e-8; // Scale epsilon based on the size of the cube

    // Check for degenerate cube cases (collapsing into a plane, line, or point)
    if cube_size < 1e-8 {
        // Treat the cube as a single point in this case
        return (point - cube_min).length() < epsilon;
    }

    // Check if the point is inside or very close to the cube's boundaries
    point.x >= cube_min.x - epsilon
        && point.x <= cube_max.x + epsilon
        && point.y >= cube_min.y - epsilon
        && point.y <= cube_max.y + epsilon
        && point.z >= cube_min.z - epsilon
        && point.z <= cube_max.z + epsilon
}

pub(crate) fn point_in_or_on_triangle(point: Vec3, triangle: (Vec3, Vec3, Vec3)) -> bool {
    let (a, b, c) = triangle;

    // Calculate the area of the triangle to determine epsilon dynamically
    let v0 = b - a;
    let v1 = c - a;
    let normal = v0.cross(v1);
    let triangle_area = normal.length() * 0.5;
    let epsilon = triangle_area * 1e-5; // Scale epsilon based on the area of the triangle

    // Check if the triangle is degenerate (nearly zero area)
    if triangle_area < 1e-8 {
        // If degenerate, treat the triangle as a line or point and check proximity
        let distance_to_vertices = [
            (point - a).length(),
            (point - b).length(),
            (point - c).length(),
        ];
        return distance_to_vertices.iter().any(|&d| d < epsilon);
    }

    // Check if the point is in the same plane as the triangle
    let distance_to_plane = normal.dot(point - a);
    if distance_to_plane.abs() > epsilon {
        return false;
    }

    // Check if the point is inside the triangle using barycentric coordinates
    let v2 = point - a;
    let dot00 = v0.dot(v0);
    let dot01 = v0.dot(v1);
    let dot02 = v0.dot(v2);
    let dot11 = v1.dot(v1);
    let dot12 = v1.dot(v2);

    let inv_denom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    u >= -epsilon && v >= -epsilon && (u + v) <= 1.0 + epsilon
}

pub(crate) fn edge_quad_intersection(edge: (Vec3, Vec3), quad: (Vec3, Vec3, Vec3, Vec3)) -> bool {
    let (e1, e2) = edge;
    let (q1, q2, q3, q4) = quad;

    // Calculate average edge length to determine a dynamic epsilon, handling degenerate cases
    let edge_length = (e2 - e1).length();
    let quad_edge_lengths = [
        (q2 - q1).length(),
        (q3 - q2).length(),
        (q4 - q3).length(),
        (q1 - q4).length(),
    ];
    let avg_length = (edge_length + quad_edge_lengths.iter().sum::<Freal>()) / 5.0;
    let epsilon = avg_length * 1e-5; // Scale epsilon based on average edge length

    // Handle degenerate cases where the quad collapses into a line or point
    if avg_length < 1e-8 {
        // Treat the quad as a degenerate case, check overlap directly with line segments
        let degenerate_edges = [(q1, q2), (q2, q3), (q3, q4), (q4, q1)];
        for &degenerate_edge in &degenerate_edges {
            if line_segment_overlap(edge, degenerate_edge) {
                return true;
            }
        }
        return false; // No intersection in degenerate case
    }

    // Check if the edge intersects with either triangle of the quad
    if triangle_edge_intersection(edge, (q1, q2, q3))
        || triangle_edge_intersection(edge, (q1, q3, q4))
    {
        return true;
    }

    // Compute average normal for the potentially non-planar quad
    let normal1 = (q2 - q1).cross(q3 - q1).normalize();
    let normal2 = (q3 - q2).cross(q4 - q2).normalize();
    let normal3 = (q4 - q3).cross(q1 - q3).normalize();
    let normal4 = (q1 - q4).cross(q2 - q4).normalize();
    let avg_normal = (normal1 + normal2 + normal3 + normal4).normalize();

    let edge_vec = e2 - e1;
    let center = (q1 + q2 + q3 + q4) * 0.25;

    // Check if the edge is parallel to the quad's average plane
    let parallel_check = avg_normal.dot(edge_vec);
    if parallel_check.abs() < epsilon {
        // Check if the edge is coplanar with the quad
        let dist_to_plane = avg_normal.dot(e1 - center).abs();
        if dist_to_plane < epsilon {
            // Edge is coplanar with quad, check if it intersects the quad's boundaries
            let quad_edges = [(q1, q2), (q2, q3), (q3, q4), (q4, q1)];
            for &quad_edge in &quad_edges {
                if line_segment_overlap(edge, quad_edge) {
                    return true;
                }
            }

            // Check if the midpoint of the edge is within the quad as a secondary containment check
            if point_in_quad((e1 + e2) * 0.5, quad) {
                return true;
            }
        }
    } else {
        // Edge intersects the quad's plane, compute the intersection point
        let t = avg_normal.dot(center - e1) / parallel_check;
        if (0.0..=1.0).contains(&t) {
            let intersection = e1 + edge_vec * t;
            if point_in_quad(intersection, quad) {
                return true;
            }
        }
    }

    false
}

pub(crate) fn point_in_quad(point: Vec3, quad: (Vec3, Vec3, Vec3, Vec3)) -> bool {
    let (a, b, c, d) = quad;

    // Calculate edge lengths to determine a dynamic epsilon
    let length1 = (b - a).length();
    let length2 = (c - b).length();
    let length3 = (d - c).length();
    let length4 = (a - d).length();
    let avg_length = (length1 + length2 + length3 + length4) * 0.25;
    let epsilon = avg_length * 1e-5; // Scale epsilon based on average edge length

    // Compute normals for the potentially non-planar quad
    let normal1 = (b - a).cross(c - a).normalize();
    let normal2 = (c - b).cross(d - b).normalize();
    let normal3 = (d - c).cross(a - c).normalize();
    let normal4 = (a - d).cross(b - d).normalize();
    let avg_normal = (normal1 + normal2 + normal3 + normal4).normalize();

    // Compute the center of the quad
    let center = (a + b + c + d) * 0.25;

    // Project the point onto the average plane of the quad
    let dist_to_plane = avg_normal.dot(point - center);
    let projected_point = point - avg_normal * dist_to_plane;

    // Check if the point is too far from the quad's plane
    if dist_to_plane.abs() > epsilon {
        return false;
    }

    // Function to check if a point is on the right side of an edge
    let is_on_right_side = |v1: Vec3, v2: Vec3| -> bool {
        let edge = v2 - v1;
        let to_point = projected_point - v1;
        avg_normal.dot(edge.cross(to_point)) >= -epsilon
    };

    // Check if the projected point is on the right side of all edges
    is_on_right_side(a, b)
        && is_on_right_side(b, c)
        && is_on_right_side(c, d)
        && is_on_right_side(d, a)
}

pub(crate) fn triangle_edge_intersection(edge: (Vec3, Vec3), triangle: (Vec3, Vec3, Vec3)) -> bool {
    let (e1, e2) = edge;
    let (t1, t2, t3) = triangle;

    // Calculate average edge length to determine a dynamic epsilon
    let edge_length = (e2 - e1).length();
    let tri_edge1 = (t2 - t1).length();
    let tri_edge2 = (t3 - t2).length();
    let tri_edge3 = (t1 - t3).length();
    let avg_length = (edge_length + tri_edge1 + tri_edge2 + tri_edge3) / 4.0;
    let epsilon = avg_length * 1e-5; // Scale epsilon based on average length

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
    if !(0.0..=1.0).contains(&t) {
        return false;
    }

    // Compute the intersection point
    let intersection = e1 + edge_vec * t;

    // Check if the intersection point is inside or on the triangle
    point_in_or_on_triangle(intersection, triangle)
}

pub(crate) fn point_on_line_segment(point: Vec3, segment: (Vec3, Vec3)) -> bool {
    let (a, b) = segment;

    // Calculate the segment length and determine a dynamic epsilon
    let ab = b - a;
    let segment_length = ab.length();
    let epsilon = segment_length * 1e-5; // Scale epsilon based on segment length

    // Handle degenerate case: if the segment length is nearly zero, treat it as a point
    if segment_length < 1e-8 {
        return (point - a).length() < epsilon;
    }

    let ap = point - a;

    // Check if the point is collinear with the line segment
    if ab.cross(ap).length_squared() > epsilon * epsilon {
        return false;
    }

    // Check if the point is within the bounds of the line segment
    let t = ap.dot(ab) / ab.length_squared();
    -epsilon <= t && t <= 1.0 + epsilon
}

pub(crate) fn line_segment_overlap(seg1: (Vec3, Vec3), seg2: (Vec3, Vec3)) -> bool {
    let (a, b) = seg1;
    let (c, d) = seg2;

    // Calculate the lengths of both segments and determine a dynamic epsilon
    let length1 = (b - a).length();
    let length2 = (d - c).length();
    let max_length = length1.max(length2);
    let epsilon = max_length * 1e-7; // Scale epsilon based on the longest segment

    // Handle degenerate cases: segments with nearly zero length
    if length1 < 1e-8 {
        // First segment is a point, check if it lies on the second segment or coincides with the point
        return (b - a).length() < epsilon && point_on_line_segment(a, seg2);
    }
    if length2 < 1e-8 {
        // Second segment is a point, check if it lies on the first segment
        return point_on_line_segment(c, seg1);
    }

    // Check if the segments share an endpoint
    if (a - c).length() < epsilon
        || (a - d).length() < epsilon
        || (b - c).length() < epsilon
        || (b - d).length() < epsilon
    {
        return true;
    }

    // Check if the segments are parallel
    let dir1 = (b - a).normalize();
    let dir2 = (d - c).normalize();
    if dir1.cross(dir2).length_squared() < epsilon * epsilon {
        // Segments are parallel, check for collinearity
        let ac = c - a;
        if ac.cross(dir1).length_squared() > epsilon * epsilon {
            return false; // Parallel but not collinear
        }
        // Check for overlap on the same line
        let t1 = ac.dot(dir1);
        let t2 = (d - a).dot(dir1);
        let s1 = length1;
        let s2 = length2;
        return (0.0..=s1).contains(&t1) || (0.0..=s2).contains(&t2) || (t1 <= 0.0 && t2 >= s1);
    }

    // Not parallel, check for intersection
    let n = dir1.cross(dir2);
    let ac = c - a;

    let denom = n.length_squared();

    // If the cross product of direction vectors is near zero but not zero, check for skew
    if denom.abs() < epsilon {
        // Check if the segments are truly skew: they do not lie on the same plane
        let plane_check = dir1.cross(dir2).dot(ac).abs();
        if plane_check > epsilon {
            return false; // Segments are skew and do not intersect
        }
    }

    let t = ac.cross(dir2).dot(n) / denom;
    let u = ac.cross(dir1).dot(n) / denom;

    // Check if intersection parameters t and u are within segment bounds [0, 1]
    if !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&u) {
        return false; // Intersection parameters are outside valid range
    }

    // Calculate actual intersection points on each segment
    let intersection_point_seg1 = a + (b - a) * t;
    let intersection_point_seg2 = c + (d - c) * u;

    // Ensure intersection points lie within the segment bounds (not just extended lines)
    if !point_on_line_segment(intersection_point_seg1, seg1)
        || !point_on_line_segment(intersection_point_seg2, seg2)
    {
        return false; // Intersection points do not lie within the actual segments
    }

    // Strictly check if the intersection point lies within the exact bounds of both segments without expanding
    true
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

        #[test]
        fn test_very_close_to_cube_but_outside() {
            let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let epsilon = 1e-5;

            // Very close to the cube but outside
            assert!(!point_in_or_on_cube(
                Vec3::new(1.0 + epsilon, 0.5, 0.5),
                cube
            ));
            assert!(!point_in_or_on_cube(
                Vec3::new(0.5, 1.0 + epsilon, 0.5),
                cube
            ));
            assert!(!point_in_or_on_cube(
                Vec3::new(0.5, 0.5, 1.0 + epsilon),
                cube
            ));
        }

        #[test]
        fn test_at_center_of_cube() {
            let cube = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));

            // At the center of the cube
            assert!(point_in_or_on_cube(Vec3::new(0.5, 0.5, 0.5), cube));
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

        #[test]
        fn test_very_close_to_triangle_but_outside() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            let epsilon = 1e-5;

            // Very close to the triangle but outside
            assert!(!point_in_or_on_triangle(
                Vec3::new(1.0 + epsilon, 0.0, 0.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(
                Vec3::new(0.0, 1.0 + epsilon, 0.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(
                Vec3::new(-epsilon, -epsilon, 0.0),
                triangle
            ));
        }

        #[test]
        fn test_at_centroid_of_triangle() {
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // At the centroid of the triangle
            let centroid = (triangle.0 + triangle.1 + triangle.2) / 3.0;
            assert!(point_in_or_on_triangle(centroid, triangle));
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

        #[test]
        fn test_edge_parallel_to_quad_inside() {
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
        fn test_edge_coinciding_with_quad_face() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_intersecting_quad_at_multiple_points() {
            let edge = (Vec3::new(-0.5, 0.5, 0.0), Vec3::new(1.5, 0.5, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_very_close_to_quad_but_outside() {
            let edge = (Vec3::new(1.0 + 1e-4, 0.5, 0.0), Vec3::new(2.0, 0.5, 0.0));
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!edge_quad_intersection(edge, quad));
        }
    }

    mod test_point_in_quad {
        use super::*;

        #[test]
        fn test_point_inside_quad() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(Vec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_on_quad_edge() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(Vec3::new(0.5, 0.0, 0.0), quad));
            assert!(point_in_quad(Vec3::new(1.0, 0.5, 0.0), quad));
            assert!(point_in_quad(Vec3::new(0.5, 1.0, 0.0), quad));
            assert!(point_in_quad(Vec3::new(0.0, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_on_quad_vertex() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(Vec3::new(0.0, 0.0, 0.0), quad));
            assert!(point_in_quad(Vec3::new(1.0, 0.0, 0.0), quad));
            assert!(point_in_quad(Vec3::new(1.0, 1.0, 0.0), quad));
            assert!(point_in_quad(Vec3::new(0.0, 1.0, 0.0), quad));
        }

        #[test]
        fn test_point_outside_quad() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(Vec3::new(-0.5, 0.5, 0.0), quad));
            assert!(!point_in_quad(Vec3::new(1.5, 0.5, 0.0), quad));
            assert!(!point_in_quad(Vec3::new(0.5, -0.5, 0.0), quad));
            assert!(!point_in_quad(Vec3::new(0.5, 1.5, 0.0), quad));
        }

        #[test]
        fn test_point_in_non_planar_quad() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 1.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(Vec3::new(0.5, 0.5, 0.25), quad));
        }

        #[test]
        fn test_point_outside_non_planar_quad() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 1.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(Vec3::new(0.5, 0.5, 1.0), quad));
        }

        #[test]
        fn test_point_on_vertex() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(Vec3::new(0.5, 0.0, 0.0), quad));
        }

        #[test]
        fn test_point_outside_vertex() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(Vec3::new(-0.5, -0.5, 0.0), quad));
        }

        #[test]
        fn test_point_outside_edge_not_touching_vertex() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(Vec3::new(0.5, -0.5, 0.0), quad));
        }

        #[test]
        fn test_point_far_from_quad() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(Vec3::new(10.0, 10.0, 10.0), quad));
        }

        #[test]
        fn test_point_on_diagonal() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(Vec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_on_boundary_not_edge_or_vertex() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // Point on the boundary but not on the edges or vertices
            assert!(point_in_quad(Vec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_at_center_of_quad() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // Point exactly at the center of the quad
            assert!(point_in_quad(Vec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_very_close_to_quad_but_outside() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );

            // Point very close to the quad but outside (testing epsilon)
            assert!(!point_in_quad(Vec3::new(1.0 + 1e-4, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_very_close_to_quad_but_inside() {
            let quad = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            );
            let point = Vec3::new(1.0 - 1e-6, 0.5, 0.0);
            assert!(point_in_quad(point, quad));
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

        #[test]
        fn test_triangle_edge_coinciding_with_edge() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
            let triangle = (
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
                Vec3::new(0.5, 0.5, 0.0),
            );
            assert!(triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_edge_very_close_to_edge_but_not_intersecting() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
            let triangle = (
                Vec3::new(0.0, 0.0, 0.1),
                Vec3::new(1.0, 1.0, 0.1),
                Vec3::new(0.5, 0.5, 0.1),
            );
            assert!(!triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_edge_intersecting_at_multiple_points() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 0.0));
            let triangle = (
                Vec3::new(0.5, 0.5, 0.0),
                Vec3::new(1.5, 0.5, 0.0),
                Vec3::new(0.5, 1.5, 0.0),
            );
            assert!(triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_edge_intersecting_at_single_point() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
            let triangle = (
                Vec3::new(0.5, 0.5, 0.0),
                Vec3::new(1.5, 0.5, 0.0),
                Vec3::new(0.5, 1.5, 0.0),
            );
            assert!(triangle_edge_intersection(edge, triangle));
        }

        #[test]
        fn test_triangle_edge_parallel_to_edge_but_inside() {
            let edge = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 0.0));
            let triangle = (
                Vec3::new(0.5, 0.5, 0.0),
                Vec3::new(1.5, 0.5, 0.0),
                Vec3::new(0.5, 1.5, 0.0),
            );
            assert!(triangle_edge_intersection(edge, triangle));
        }
    }

    mod test_point_on_line_segment {
        use super::*;

        #[test]
        fn test_point_on_segment() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(point_on_line_segment(Vec3::new(0.5, 0.5, 0.5), segment));
        }

        #[test]
        fn test_point_at_segment_start() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(point_on_line_segment(Vec3::new(0.0, 0.0, 0.0), segment));
        }

        #[test]
        fn test_point_at_segment_end() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(point_on_line_segment(Vec3::new(1.0, 1.0, 1.0), segment));
        }

        #[test]
        fn test_point_not_on_segment() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(!point_on_line_segment(Vec3::new(0.5, 0.5, 0.0), segment));
        }

        #[test]
        fn test_point_beyond_segment_start() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(!point_on_line_segment(Vec3::new(-0.1, -0.1, -0.1), segment));
        }

        #[test]
        fn test_point_beyond_segment_end() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(!point_on_line_segment(Vec3::new(1.1, 1.1, 1.1), segment));
        }

        #[test]
        fn test_point_on_horizontal_segment() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(point_on_line_segment(Vec3::new(0.5, 0.0, 0.0), segment));
        }

        #[test]
        fn test_point_on_vertical_segment() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
            assert!(point_on_line_segment(Vec3::new(0.0, 0.5, 0.0), segment));
        }

        #[test]
        fn test_point_near_segment_within_epsilon() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(point_on_line_segment(
                Vec3::new(0.5, 0.5, 0.500001),
                segment
            ));
        }

        #[test]
        fn test_point_on_boundary() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(point_on_line_segment(Vec3::new(0.5, 0.5, 0.5), segment));
        }

        #[test]
        fn test_point_far_beyond_segment_end() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(!point_on_line_segment(Vec3::new(1.5, 1.5, 1.5), segment));
        }

        #[test]
        fn test_point_far_beyond_segment_start() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(!point_on_line_segment(Vec3::new(-1.5, -1.5, -1.5), segment));
        }

        #[test]
        fn test_point_at_midpoint_of_segment() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let point = Vec3::new(0.5, 0.5, 0.5);
            assert!(point_on_line_segment(point, segment));
        }

        #[test]
        fn test_point_very_close_to_segment_but_outside_epsilon() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let point = Vec3::new(0.5, 0.5, 0.5001); // Slightly outside epsilon
            assert!(!point_on_line_segment(point, segment));
        }

        #[test]
        fn test_point_exactly_at_epsilon_boundary() {
            let segment = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let point = Vec3::new(0.5, 0.5, 0.50001); // Exactly at epsilon boundary
            assert!(point_on_line_segment(point, segment));
        }
    }

    mod test_line_segment_overlap {
        use super::*;

        #[test]
        fn test_segments_overlap() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let seg2 = (Vec3::new(0.5, 0.5, 0.5), Vec3::new(1.5, 1.5, 1.5));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_touch_at_endpoint() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let seg2 = (Vec3::new(1.0, 1.0, 1.0), Vec3::new(2.0, 2.0, 2.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_do_not_overlap() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let seg2 = (Vec3::new(2.0, 2.0, 2.0), Vec3::new(3.0, 3.0, 3.0));
            assert!(!line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_parallel_no_overlap() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            let seg2 = (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
            assert!(!line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_on_same_line_no_overlap() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            let seg2 = (Vec3::new(2.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0));
            assert!(!line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_overlap_partially() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
            let seg2 = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segment_contained_within_other() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0));
            let seg2 = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_barely_touching() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            let seg2 = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_barely_missing() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            let seg2 = (Vec3::new(1.000001, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
            assert!(!line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_perpendicular_intersecting() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
            let seg2 = (Vec3::new(1.0, -1.0, 0.0), Vec3::new(1.0, 1.0, 0.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_perpendicular_not_intersecting() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            let seg2 = (Vec3::new(2.0, -1.0, 0.0), Vec3::new(2.0, 1.0, 0.0));
            assert!(!line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_parallel_overlap_endpoint() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0));
            let seg2 = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_same_start_point() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let seg2 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_same_end_point() {
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0));
            let seg2 = (Vec3::new(0.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_one_point() {
            let seg1 = (Vec3::new(1.0, 1.0, 1.0), Vec3::new(1.0, 1.0, 1.0));
            let seg2 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(2.0, 2.0, 2.0));
            assert!(line_segment_overlap(seg1, seg2));
        }

        #[test]
        fn test_segments_parallel_very_close() {
            let epsilon = 1e-5;
            let seg1 = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
            let seg2 = (Vec3::new(0.0, epsilon, 0.0), Vec3::new(1.0, epsilon, 0.0));
            assert!(!line_segment_overlap(seg1, seg2));
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
