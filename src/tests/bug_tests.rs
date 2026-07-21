//! Regression tests for previously reported bugs.

use super::support::*;

#[test]
fn test_contains_vertex() {
    let csg_cube = Mesh::<()>::cube(r(6.0), ());

    assert!(csg_cube.contains_vertex(&p3(3.0, 3.0, 3.0)));
    assert!(csg_cube.contains_vertex(&p3(1.0, 2.0, 5.9)));
    assert!(!csg_cube.contains_vertex(&p3(3.0, 3.0, 6.0)));
    assert!(!csg_cube.contains_vertex(&p3(3.0, 3.0, -6.0)));
    assert!(!csg_cube.contains_vertex(&p3(3.0, 3.0, 0.0)));
    assert!(csg_cube.contains_vertex(&p3(3.0, 3.0, 0.01)));

    assert!(csg_cube.contains_vertex(&p3(3.0, 3.0, 5.99999999)));
    assert!(csg_cube.contains_vertex(&p3(3.0, 3.0, 6.0 - 1e-11)));
    assert!(csg_cube.contains_vertex(&p3(3.0, 3.0, 6.0 - 1e-14)));
    assert!(csg_cube.contains_vertex(&p3(3.0, 3.0, 5.9 + 9e-9)));

    assert!(!csg_cube.contains_vertex(&p3(3.0, -3.0, 3.0)));
    assert!(!csg_cube.contains_vertex(&p3(3.0, -3.01, 3.0)));
    assert!(csg_cube.contains_vertex(&p3(0.01, 4.0, 3.0)));
    assert!(!csg_cube.contains_vertex(&p3(-0.01, 4.0, 3.0)));

    let csg_cube_hole = Mesh::<()>::cube(r(4.0), ());
    let cube_with_hole = csg_cube.difference(&csg_cube_hole);

    assert!(!cube_with_hole.contains_vertex(&p3(0.01, 4.0, 3.0)));
    assert!(cube_with_hole.contains_vertex(&p3(0.01, 4.01, 3.0)));
    assert!(!cube_with_hole.contains_vertex(&p3(-0.01, 4.0, 3.0)));
    assert!(cube_with_hole.contains_vertex(&p3(1.0, 2.0, 5.9)));
    assert!(!cube_with_hole.contains_vertex(&p3(3.0, 3.0, 6.0)));

    let csg_sphere = Mesh::<()>::sphere(r(6.0), 14, 14, ());

    assert!(csg_sphere.contains_vertex(&p3(1.0, 1.0, 1.0)));
    assert!(csg_sphere.contains_vertex(&p3(-1.0, -1.0, -1.0)));
    assert!(!csg_sphere.contains_vertex(&p3(1.0, 2.0, 5.9)));
    assert!(!csg_sphere.contains_vertex(&p3(3.0, 3.0, 6.0)));
    assert!(!csg_sphere.contains_vertex(&p3(3.0, 3.0, -6.0)));
    assert!(csg_sphere.contains_vertex(&p3(3.0, 3.0, 0.0)));
    assert!(!csg_sphere.contains_vertex(&p3(3.0, 3.0, -6.01)));
    assert!(csg_sphere.contains_vertex(&p3(3.0, 3.0, 0.01)));
}

#[test]
fn test_union_crash() {
    let items: [Mesh<()>; 2] = [
        Mesh::from_polygons(vec![
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(640.0, 0.0, 640.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 0.0, 128.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 0.0, 256.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1024.0, 0.0, 640.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(1024.0, 256.0, 640.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 256.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 256.0, 128.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(640.0, 256.0, 640.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(640.0, 0.0, 640.0),
                        normal: v3(0.9701425433158875, -0.0, 0.24253563582897186),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(640.0, 256.0, 640.0),
                        normal: v3(0.9701425433158875, -0.0, 0.24253563582897186),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 256.0, 128.0),
                        normal: v3(0.9701425433158875, -0.0, 0.24253563582897186),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 0.0, 128.0),
                        normal: v3(0.9701425433158875, -0.0, 0.24253563582897186),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(768.0, 0.0, 128.0),
                        normal: v3(-0.24253563582897186, 0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 256.0, 128.0),
                        normal: v3(-0.24253563582897186, 0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 256.0),
                        normal: v3(-0.24253563582897186, 0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 0.0, 256.0),
                        normal: v3(-0.24253563582897186, 0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(1280.0, 0.0, 256.0),
                        normal: v3(-0.8320503234863281, 0.0, -0.5547001957893372),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 256.0),
                        normal: v3(-0.8320503234863281, 0.0, -0.5547001957893372),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1024.0, 256.0, 640.0),
                        normal: v3(-0.8320503234863281, 0.0, -0.5547001957893372),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1024.0, 0.0, 640.0),
                        normal: v3(-0.8320503234863281, 0.0, -0.5547001957893372),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(1024.0, 0.0, 640.0),
                        normal: v3(0.0, 0.0, -1.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1024.0, 256.0, 640.0),
                        normal: v3(0.0, 0.0, -1.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(640.0, 256.0, 640.0),
                        normal: v3(0.0, 0.0, -1.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(640.0, 0.0, 640.0),
                        normal: v3(0.0, 0.0, -1.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
        ]),
        Mesh::from_polygons(vec![
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(896.0, 0.0, 768.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 0.0, 512.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 0.0, 384.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 0.0, 640.0),
                        normal: v3(0.0, -1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(1280.0, 256.0, 640.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 384.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 256.0, 512.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(896.0, 256.0, 768.0),
                        normal: v3(0.0, 1.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(896.0, 0.0, 768.0),
                        normal: v3(0.8944271802902222, 0.0, -0.4472135901451111),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(896.0, 256.0, 768.0),
                        normal: v3(0.8944271802902222, 0.0, -0.4472135901451111),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 256.0, 512.0),
                        normal: v3(0.8944271802902222, 0.0, -0.4472135901451111),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 0.0, 512.0),
                        normal: v3(0.8944271802902222, 0.0, -0.4472135901451111),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(768.0, 0.0, 512.0),
                        normal: v3(0.24253563582897186, -0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(768.0, 256.0, 512.0),
                        normal: v3(0.24253563582897186, -0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 384.0),
                        normal: v3(0.24253563582897186, -0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 0.0, 384.0),
                        normal: v3(0.24253563582897186, -0.0, 0.9701425433158875),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(1280.0, 0.0, 384.0),
                        normal: v3(-1.0, 0.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 384.0),
                        normal: v3(-1.0, 0.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 640.0),
                        normal: v3(-1.0, 0.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 0.0, 640.0),
                        normal: v3(-1.0, 0.0, 0.0),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
            Polygon::from_planar_vertices(
                vec![
                    Vertex {
                        position: p3(1280.0, 0.0, 640.0),
                        normal: v3(-0.3162277638912201, 0.0, -0.9486832618713379),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(1280.0, 256.0, 640.0),
                        normal: v3(-0.3162277638912201, 0.0, -0.9486832618713379),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(896.0, 256.0, 768.0),
                        normal: v3(-0.3162277638912201, 0.0, -0.9486832618713379),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                    Vertex {
                        position: p3(896.0, 0.0, 768.0),
                        normal: v3(-0.3162277638912201, 0.0, -0.9486832618713379),
                        position_id: crate::vertex::fresh_position_id(),
                        coordinate_ids: [
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                            crate::vertex::fresh_position_id(),
                        ],
                        ruled_line: None,
                        hull_candidate: true,
                    },
                ],
                (),
            ),
        ]),
    ];

    let combined = items[0].union(&items[1]);
    println!("{:?}", combined);
}
