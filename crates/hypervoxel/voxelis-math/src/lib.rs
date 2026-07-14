//! Floating-point overlap helpers used by the legacy Voxelis voxelizer.
//!
//! These functions deliberately use scale-dependent or fixed tolerances. They
//! are suitable for sampled occupancy, not exact topology or certification.

use glam::DVec3;

/// Tests a triangle against an axis-aligned cube using tolerant rejection and
/// feature-intersection checks.
///
/// `cube.0` and `cube.1` are expected to be component-wise minimum and maximum
/// corners. This is a floating-point sampling predicate and may conservatively
/// classify near-degenerate configurations as intersections.
pub fn triangle_cube_intersection(triangle: (DVec3, DVec3, DVec3), cube: (DVec3, DVec3)) -> bool {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("triangle_cube_intersection");

    let (tv0, tv1, tv2) = triangle;
    let (cube_min, cube_max) = cube;

    let tri_min = tv0.min(tv1).min(tv2);
    let tri_max = tv0.max(tv1).max(tv2);

    // Reject separated bounding boxes, retaining a small contact tolerance.
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

    // Treat a cube that straddles the triangle plane as a conservative hit.
    let normal = (tv1 - tv0).cross(tv2 - tv0);
    let d = -normal.dot(tv0);

    let cube_points = [
        DVec3::new(cube_min.x, cube_min.y, cube_min.z),
        DVec3::new(cube_max.x, cube_min.y, cube_min.z),
        DVec3::new(cube_max.x, cube_max.y, cube_min.z),
        DVec3::new(cube_min.x, cube_max.y, cube_min.z),
        DVec3::new(cube_min.x, cube_min.y, cube_max.z),
        DVec3::new(cube_max.x, cube_min.y, cube_max.z),
        DVec3::new(cube_max.x, cube_max.y, cube_max.z),
        DVec3::new(cube_min.x, cube_max.y, cube_max.z),
    ];
    let sign = (normal.dot(cube_points[0]) + d).signum();

    for p in &cube_points[1..] {
        let new_sign = (normal.dot(*p) + d).signum();
        if (normal.dot(*p) + d).abs() < epsilon {
            continue;
        }
        if new_sign != sign {
            return true;
        }
    }

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

    let triangle_edges = [(tv0, tv1), (tv1, tv2), (tv2, tv0)];
    let cube_faces = [
        // Front
        (
            cube_points[0],
            cube_points[1],
            cube_points[2],
            cube_points[3],
        ),
        // Back
        (
            cube_points[4],
            cube_points[5],
            cube_points[6],
            cube_points[7],
        ),
        // Bottom
        (
            cube_points[0],
            cube_points[1],
            cube_points[5],
            cube_points[4],
        ),
        // Top
        (
            cube_points[2],
            cube_points[3],
            cube_points[7],
            cube_points[6],
        ),
        // Left
        (
            cube_points[0],
            cube_points[3],
            cube_points[7],
            cube_points[4],
        ),
        // Right
        (
            cube_points[1],
            cube_points[2],
            cube_points[6],
            cube_points[5],
        ),
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

/// Tests whether `point` lies within or near an axis-aligned cube.
///
/// The boundary tolerance is `1e-8` times the cube diagonal. A cube with a
/// shorter diagonal is treated as a point.
pub fn point_in_or_on_cube(point: DVec3, cube: (DVec3, DVec3)) -> bool {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("point_in_or_on_cube");

    let (cube_min, cube_max) = cube;

    let cube_size = (cube_max - cube_min).length();
    let epsilon = cube_size * 1e-8;

    if cube_size < 1e-8 {
        return (point - cube_min).length() <= 1e-8;
    }

    point.x >= cube_min.x - epsilon
        && point.x <= cube_max.x + epsilon
        && point.y >= cube_min.y - epsilon
        && point.y <= cube_max.y + epsilon
        && point.z >= cube_min.z - epsilon
        && point.z <= cube_max.z + epsilon
}

/// Tests the barycentric projection of `point` against a triangle.
///
/// Degenerate triangles return `false`. This helper does not test that `point`
/// lies in the triangle's plane; callers that need 3D containment must perform
/// a coplanarity test first.
pub fn point_in_or_on_triangle(point: DVec3, triangle: (DVec3, DVec3, DVec3)) -> bool {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("point_in_or_on_triangle");

    let (a, b, c) = triangle;
    let v0 = b - a;
    let v1 = c - a;
    let v2 = point - a;

    let dot00 = v0.dot(v0);
    let dot01 = v0.dot(v1);
    let dot02 = v0.dot(v2);
    let dot11 = v1.dot(v1);
    let dot12 = v1.dot(v2);

    let denom = dot00 * dot11 - dot01 * dot01;
    if denom.abs() < 1e-8 {
        return false;
    }
    let inv_denom = 1.0 / denom;

    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    u >= 0.0 && v >= 0.0 && (u + v) <= 1.0
}

/// Tests whether a segment crosses the interior or boundary of a planar quad.
///
/// The quad is split along `(0, 2)`. Segments parallel or coplanar with the
/// quad plane return `false`.
pub fn edge_quad_intersection(edge: (DVec3, DVec3), quad: (DVec3, DVec3, DVec3, DVec3)) -> bool {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("edge_quad_intersection");

    let (e1, e2) = edge;

    let normal = (quad.1 - quad.0).cross(quad.2 - quad.0).normalize();
    let denom = normal.dot(e2 - e1);
    if denom.abs() < 1e-8 {
        return false;
    }

    let t = normal.dot(quad.0 - e1) / denom;
    if !(0.0..=1.0).contains(&t) {
        return false;
    }

    let intersection_point = e1 + t * (e2 - e1);

    point_in_quad(intersection_point, quad)
}

/// Tests the barycentric projection of `point` against either triangle of a quad.
///
/// This function does not independently verify coplanarity.
pub fn point_in_quad(point: DVec3, quad: (DVec3, DVec3, DVec3, DVec3)) -> bool {
    #[cfg(feature = "tracy")]
    let _span = tracy_client::span!("point_in_quad");

    let (a, b, c, d) = quad;

    point_in_or_on_triangle(point, (a, b, c)) || point_in_or_on_triangle(point, (a, c, d))
}

#[cfg(test)]
mod tests {
    use super::*;

    mod test_point_in_or_on_cube {
        use super::*;

        #[test]
        fn test_inside_cube() {
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));

            // Inside the cube
            assert!(point_in_or_on_cube(DVec3::new(0.5, 0.5, 0.5), cube));
        }

        #[test]
        fn test_on_edges() {
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));

            // On the edges
            assert!(point_in_or_on_cube(DVec3::new(0.0, 0.5, 0.5), cube));
            assert!(point_in_or_on_cube(DVec3::new(1.0, 0.5, 0.5), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.5, 0.0, 0.5), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.5, 1.0, 0.5), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.5, 0.5, 0.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.5, 0.5, 1.0), cube));
        }

        #[test]
        fn test_on_faces() {
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));

            // On the faces
            assert!(point_in_or_on_cube(DVec3::new(0.0, 0.0, 0.5), cube));
            assert!(point_in_or_on_cube(DVec3::new(1.0, 1.0, 0.5), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.5, 0.0, 0.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.5, 1.0, 1.0), cube));
        }

        #[test]
        fn test_at_vertices() {
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));

            // At the vertices
            assert!(point_in_or_on_cube(DVec3::new(0.0, 0.0, 0.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(1.0, 0.0, 0.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.0, 1.0, 0.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.0, 0.0, 1.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(1.0, 1.0, 0.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(1.0, 0.0, 1.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(0.0, 1.0, 1.0), cube));
            assert!(point_in_or_on_cube(DVec3::new(1.0, 1.0, 1.0), cube));
        }

        #[test]
        fn test_outside_cube() {
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));

            // Outside the cube
            assert!(!point_in_or_on_cube(DVec3::new(-0.1, 0.5, 0.5), cube));
            assert!(!point_in_or_on_cube(DVec3::new(1.1, 0.5, 0.5), cube));
            assert!(!point_in_or_on_cube(DVec3::new(0.5, -0.1, 0.5), cube));
            assert!(!point_in_or_on_cube(DVec3::new(0.5, 1.1, 0.5), cube));
            assert!(!point_in_or_on_cube(DVec3::new(0.5, 0.5, -0.1), cube));
            assert!(!point_in_or_on_cube(DVec3::new(0.5, 0.5, 1.1), cube));
        }

        #[test]
        fn test_very_close_to_cube_but_outside() {
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            let epsilon = 1e-5;

            // Very close to the cube but outside
            assert!(!point_in_or_on_cube(
                DVec3::new(1.0 + epsilon, 0.5, 0.5),
                cube
            ));
            assert!(!point_in_or_on_cube(
                DVec3::new(0.5, 1.0 + epsilon, 0.5),
                cube
            ));
            assert!(!point_in_or_on_cube(
                DVec3::new(0.5, 0.5, 1.0 + epsilon),
                cube
            ));
        }

        #[test]
        fn test_at_center_of_cube() {
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));

            // At the center of the cube
            assert!(point_in_or_on_cube(DVec3::new(0.5, 0.5, 0.5), cube));
        }
    }

    mod test_point_in_or_on_triangle {
        use super::*;

        #[test]
        fn test_inside_triangle() {
            let triangle = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // Inside the triangle
            assert!(point_in_or_on_triangle(
                DVec3::new(0.25, 0.25, 0.0),
                triangle
            ));
        }

        #[test]
        fn test_on_edges() {
            let triangle = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // On the edges
            assert!(point_in_or_on_triangle(DVec3::new(0.5, 0.0, 0.0), triangle));
            assert!(point_in_or_on_triangle(DVec3::new(0.0, 0.5, 0.0), triangle));
            assert!(point_in_or_on_triangle(DVec3::new(0.5, 0.5, 0.0), triangle));
        }

        #[test]
        fn test_on_vertices() {
            let triangle = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // On the vertices
            assert!(point_in_or_on_triangle(DVec3::new(0.0, 0.0, 0.0), triangle));
            assert!(point_in_or_on_triangle(DVec3::new(1.0, 0.0, 0.0), triangle));
            assert!(point_in_or_on_triangle(DVec3::new(0.0, 1.0, 0.0), triangle));
        }

        #[test]
        fn test_outside_triangle_same_plane() {
            let triangle = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // Outside the triangle but in the same plane
            assert!(!point_in_or_on_triangle(
                DVec3::new(1.0, 1.0, 0.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(
                DVec3::new(-0.1, 0.5, 0.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(
                DVec3::new(0.5, -0.1, 0.0),
                triangle
            ));
        }

        #[test]
        fn test_very_close_to_triangle_but_outside() {
            let triangle = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            let epsilon = 1e-5;

            // Very close to the triangle but outside
            assert!(!point_in_or_on_triangle(
                DVec3::new(1.0 + epsilon, 0.0, 0.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(
                DVec3::new(0.0, 1.0 + epsilon, 0.0),
                triangle
            ));
            assert!(!point_in_or_on_triangle(
                DVec3::new(-epsilon, -epsilon, 0.0),
                triangle
            ));
        }

        #[test]
        fn test_at_centroid_of_triangle() {
            let triangle = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // At the centroid of the triangle
            let centroid = (triangle.0 + triangle.1 + triangle.2) / 3.0;
            assert!(point_in_or_on_triangle(centroid, triangle));
        }
    }

    mod test_edge_quad_intersection {
        use super::*;

        #[test]
        fn test_edge_completely_outside_quad() {
            let edge = (DVec3::new(1.5, 1.5, 0.0), DVec3::new(2.0, 1.5, 0.0));
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_intersecting_quad_not_touching_vertices_or_edges() {
            let edge = (DVec3::new(0.5, 0.5, -0.5), DVec3::new(0.5, 0.5, 0.5));
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_parallel_to_quad_outside() {
            let edge = (DVec3::new(0.0, 0.0, 1.0), DVec3::new(1.0, 0.0, 1.0));
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!edge_quad_intersection(edge, quad));
        }

        #[test]
        fn test_edge_very_close_to_quad_but_outside() {
            let edge = (DVec3::new(1.0 + 1e-4, 0.5, 0.0), DVec3::new(2.0, 0.5, 0.0));
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!edge_quad_intersection(edge, quad));
        }
    }

    mod test_point_in_quad {
        use super::*;

        #[test]
        fn test_point_inside_quad() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(DVec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_on_quad_edge() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(DVec3::new(0.5, 0.0, 0.0), quad));
            assert!(point_in_quad(DVec3::new(1.0, 0.5, 0.0), quad));
            assert!(point_in_quad(DVec3::new(0.5, 1.0, 0.0), quad));
            assert!(point_in_quad(DVec3::new(0.0, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_on_quad_vertex() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(DVec3::new(0.0, 0.0, 0.0), quad));
            assert!(point_in_quad(DVec3::new(1.0, 0.0, 0.0), quad));
            assert!(point_in_quad(DVec3::new(1.0, 1.0, 0.0), quad));
            assert!(point_in_quad(DVec3::new(0.0, 1.0, 0.0), quad));
        }

        #[test]
        fn test_point_outside_quad() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(DVec3::new(-0.5, 0.5, 0.0), quad));
            assert!(!point_in_quad(DVec3::new(1.5, 0.5, 0.0), quad));
            assert!(!point_in_quad(DVec3::new(0.5, -0.5, 0.0), quad));
            assert!(!point_in_quad(DVec3::new(0.5, 1.5, 0.0), quad));
        }

        #[test]
        fn test_point_in_non_planar_quad() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 1.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(DVec3::new(0.5, 0.5, 0.25), quad));
        }

        #[test]
        fn test_point_outside_non_planar_quad() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 1.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(DVec3::new(0.5, 0.5, 1.0), quad));
        }

        #[test]
        fn test_point_on_vertex() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(DVec3::new(0.5, 0.0, 0.0), quad));
        }

        #[test]
        fn test_point_outside_vertex() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(DVec3::new(-0.5, -0.5, 0.0), quad));
        }

        #[test]
        fn test_point_outside_edge_not_touching_vertex() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(DVec3::new(0.5, -0.5, 0.0), quad));
        }

        #[test]
        fn test_point_far_from_quad() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(!point_in_quad(DVec3::new(10.0, 10.0, 10.0), quad));
        }

        #[test]
        fn test_point_on_diagonal() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            assert!(point_in_quad(DVec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_on_boundary_not_edge_or_vertex() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // Point on the boundary but not on the edges or vertices
            assert!(point_in_quad(DVec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_at_center_of_quad() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // Point exactly at the center of the quad
            assert!(point_in_quad(DVec3::new(0.5, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_very_close_to_quad_but_outside() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );

            // Point very close to the quad but outside (testing epsilon)
            assert!(!point_in_quad(DVec3::new(1.0 + 1e-4, 0.5, 0.0), quad));
        }

        #[test]
        fn test_point_very_close_to_quad_but_inside() {
            let quad = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(1.0, 1.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            let point = DVec3::new(1.0 - 1e-6, 0.5, 0.0);
            assert!(point_in_quad(point, quad));
        }
    }

    mod test_triangle_cube_intersection {
        use super::*;

        #[test]
        fn test_triangle_completely_inside_cube() {
            let triangle = (
                DVec3::new(0.25, 0.25, 0.25),
                DVec3::new(0.75, 0.25, 0.25),
                DVec3::new(0.25, 0.75, 0.25),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_completely_outside_cube() {
            let triangle = (
                DVec3::new(1.5, 1.5, 1.5),
                DVec3::new(2.5, 1.5, 1.5),
                DVec3::new(1.5, 2.5, 1.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(!triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_vertex_inside_cube() {
            let triangle = (
                DVec3::new(-0.5, 0.5, 0.5),
                DVec3::new(0.5, 0.5, 0.5),
                DVec3::new(1.5, 0.5, 0.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_edge_intersects_cube() {
            let triangle = (
                DVec3::new(-0.5, 0.5, 0.5),
                DVec3::new(0.5, 0.5, 0.5),
                DVec3::new(0.5, 1.5, 0.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_face_intersects_cube() {
            let triangle = (
                DVec3::new(0.5, 0.5, -0.5),
                DVec3::new(0.5, 0.5, 0.5),
                DVec3::new(0.5, 1.5, 0.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_cube_above_triangle_no_intersection() {
            let triangle = (
                DVec3::new(0.0, 0.0, 0.0),
                DVec3::new(1.0, 0.0, 0.0),
                DVec3::new(0.0, 1.0, 0.0),
            );
            let cube = (DVec3::new(0.5, 0.5, 0.5), DVec3::new(1.5, 1.5, 1.5));
            assert!(!triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_crosses_cube() {
            // Test case 1: Current triangle crossing the cube
            let triangle = (
                DVec3::new(-0.5, 0.5, 0.5),
                DVec3::new(1.5, 0.5, 0.5),
                DVec3::new(0.5, 2.0, 0.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));

            // Test case 2: Triangle completely outside and above the cube
            let triangle = (
                DVec3::new(0.5, 1.5, 1.5),
                DVec3::new(1.5, 1.5, 1.5),
                DVec3::new(1.0, 2.0, 1.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(!triangle_cube_intersection(triangle, cube));

            // Test case 3: Triangle with one vertex inside the cube
            let triangle = (
                DVec3::new(0.5, 0.5, 0.5),
                DVec3::new(2.0, 2.0, 2.0),
                DVec3::new(1.5, 1.5, 2.0),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));

            // Test case 4: Triangle parallel to one face of the cube but fully outside
            let triangle = (
                DVec3::new(1.5, 0.5, 0.5),
                DVec3::new(2.5, 0.5, 0.5),
                DVec3::new(2.0, 1.5, 0.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(!triangle_cube_intersection(triangle, cube));

            // Test case 5: Triangle fully inside the cube
            let triangle = (
                DVec3::new(0.2, 0.2, 0.2),
                DVec3::new(0.8, 0.2, 0.2),
                DVec3::new(0.5, 0.8, 0.2),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_no_intersection_triangle_behind_cube() {
            let triangle = (
                DVec3::new(-1.5, -1.5, -1.5),
                DVec3::new(-1.0, -1.5, -1.5),
                DVec3::new(-1.5, -1.0, -1.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(!triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_parallel_to_cube_face_no_intersection() {
            let triangle = (
                DVec3::new(0.0, 0.0, 1.5),
                DVec3::new(1.0, 0.0, 1.5),
                DVec3::new(0.0, 1.0, 1.5),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(!triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_touching_cube_face() {
            let triangle = (
                DVec3::new(0.5, 0.5, 1.0),
                DVec3::new(0.75, 0.25, 1.0),
                DVec3::new(0.25, 0.75, 1.0),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(triangle_cube_intersection(triangle, cube));
        }

        #[test]
        fn test_triangle_overlapping_cube_face_no_intersection() {
            let triangle = (
                DVec3::new(0.5, 0.5, 2.0),
                DVec3::new(1.5, 0.5, 2.0),
                DVec3::new(0.5, 1.5, 2.0),
            );
            let cube = (DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 1.0, 1.0));
            assert!(!triangle_cube_intersection(triangle, cube));
        }
    }
}
