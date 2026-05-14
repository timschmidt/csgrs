//! Regression tests for previously reported bugs.

use super::support::*;

#[test]
fn test_contains_vertex() {
    let csg_cube = Mesh::<()>::cube(6.0, None);

    assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    assert!(csg_cube.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 0.0)));
    assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 0.01)));

    #[cfg(feature = "f64")]
    {
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.99999999)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0 - 1e-11)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0 - 1e-14)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.9 + 9e-9)));
    }

    #[cfg(feature = "f32")]
    {
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.999999)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 6.0 - 1e-6)));
        assert!(csg_cube.contains_vertex(&Point3::new(3.0, 3.0, 5.9 + 9e-9)));
    }

    assert!(csg_cube.contains_vertex(&Point3::new(3.0, -3.0, 3.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(3.0, -3.01, 3.0)));
    assert!(csg_cube.contains_vertex(&Point3::new(0.01, 4.0, 3.0)));
    assert!(!csg_cube.contains_vertex(&Point3::new(-0.01, 4.0, 3.0)));

    let csg_cube_hole = Mesh::<()>::cube(4.0, None);
    let cube_with_hole = csg_cube.difference(&csg_cube_hole);

    assert!(!cube_with_hole.contains_vertex(&Point3::new(0.01, 4.0, 3.0)));
    assert!(cube_with_hole.contains_vertex(&Point3::new(0.01, 4.01, 3.0)));
    assert!(!cube_with_hole.contains_vertex(&Point3::new(-0.01, 4.0, 3.0)));
    assert!(cube_with_hole.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));
    assert!(!cube_with_hole.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));

    let csg_sphere = Mesh::<()>::sphere(6.0, 14, 14, None);

    assert!(csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 3.0)));
    assert!(csg_sphere.contains_vertex(&Point3::new(-3.0, -3.0, -3.0)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(1.0, 2.0, 5.9)));

    assert!(!csg_sphere.contains_vertex(&Point3::new(1.0, 1.0, 5.8)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(0.0, 3.0, 5.8)));
    assert!(csg_sphere.contains_vertex(&Point3::new(0.0, 0.0, 5.8)));

    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 6.0)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, -6.0)));
    assert!(csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 0.0)));

    assert!(csg_sphere.contains_vertex(&Point3::new(0.0, 0.0, -5.8)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, -5.8)));
    assert!(!csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, -6.01)));
    assert!(csg_sphere.contains_vertex(&Point3::new(3.0, 3.0, 0.01)));
}

#[test]
fn test_union_crash() {
    let items: [Mesh<()>; 2] = [
        Mesh::from_polygons(
            &[
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(640.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(768.0, 0.0, 128.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 256.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1024.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(1024.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 256.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(768.0, 256.0, 128.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(640.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(640.0, 0.0, 640.0),
                            normal: Vector3::new(
                                0.9701425433158875,
                                -0.0,
                                0.24253563582897186,
                            ),
                        },
                        Vertex {
                            position: Point3::new(640.0, 256.0, 640.0),
                            normal: Vector3::new(
                                0.9701425433158875,
                                -0.0,
                                0.24253563582897186,
                            ),
                        },
                        Vertex {
                            position: Point3::new(768.0, 256.0, 128.0),
                            normal: Vector3::new(
                                0.9701425433158875,
                                -0.0,
                                0.24253563582897186,
                            ),
                        },
                        Vertex {
                            position: Point3::new(768.0, 0.0, 128.0),
                            normal: Vector3::new(
                                0.9701425433158875,
                                -0.0,
                                0.24253563582897186,
                            ),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(768.0, 0.0, 128.0),
                            normal: Vector3::new(
                                -0.24253563582897186,
                                0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            position: Point3::new(768.0, 256.0, 128.0),
                            normal: Vector3::new(
                                -0.24253563582897186,
                                0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 256.0),
                            normal: Vector3::new(
                                -0.24253563582897186,
                                0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 256.0),
                            normal: Vector3::new(
                                -0.24253563582897186,
                                0.0,
                                0.9701425433158875,
                            ),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 256.0),
                            normal: Vector3::new(
                                -0.8320503234863281,
                                0.0,
                                -0.5547001957893372,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 256.0),
                            normal: Vector3::new(
                                -0.8320503234863281,
                                0.0,
                                -0.5547001957893372,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1024.0, 256.0, 640.0),
                            normal: Vector3::new(
                                -0.8320503234863281,
                                0.0,
                                -0.5547001957893372,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1024.0, 0.0, 640.0),
                            normal: Vector3::new(
                                -0.8320503234863281,
                                0.0,
                                -0.5547001957893372,
                            ),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(1024.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, 0.0, -1.0),
                        },
                        Vertex {
                            position: Point3::new(1024.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 0.0, -1.0),
                        },
                        Vertex {
                            position: Point3::new(640.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 0.0, -1.0),
                        },
                        Vertex {
                            position: Point3::new(640.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, 0.0, -1.0),
                        },
                    ],
                    None,
                ),
            ],
            None,
        ),
        Mesh::from_polygons(
            &[
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(896.0, 0.0, 768.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(768.0, 0.0, 512.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 384.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 384.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(768.0, 256.0, 512.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(896.0, 256.0, 768.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(896.0, 0.0, 768.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                        Vertex {
                            position: Point3::new(896.0, 256.0, 768.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                        Vertex {
                            position: Point3::new(768.0, 256.0, 512.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                        Vertex {
                            position: Point3::new(768.0, 0.0, 512.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(768.0, 0.0, 512.0),
                            normal: Vector3::new(
                                0.24253563582897186,
                                -0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            position: Point3::new(768.0, 256.0, 512.0),
                            normal: Vector3::new(
                                0.24253563582897186,
                                -0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 384.0),
                            normal: Vector3::new(
                                0.24253563582897186,
                                -0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 384.0),
                            normal: Vector3::new(
                                0.24253563582897186,
                                -0.0,
                                0.9701425433158875,
                            ),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 384.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 384.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 640.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 640.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            position: Point3::new(1280.0, 0.0, 640.0),
                            normal: Vector3::new(
                                -0.3162277638912201,
                                0.0,
                                -0.9486832618713379,
                            ),
                        },
                        Vertex {
                            position: Point3::new(1280.0, 256.0, 640.0),
                            normal: Vector3::new(
                                -0.3162277638912201,
                                0.0,
                                -0.9486832618713379,
                            ),
                        },
                        Vertex {
                            position: Point3::new(896.0, 256.0, 768.0),
                            normal: Vector3::new(
                                -0.3162277638912201,
                                0.0,
                                -0.9486832618713379,
                            ),
                        },
                        Vertex {
                            position: Point3::new(896.0, 0.0, 768.0),
                            normal: Vector3::new(
                                -0.3162277638912201,
                                0.0,
                                -0.9486832618713379,
                            ),
                        },
                    ],
                    None,
                ),
            ],
            None,
        ),
    ];

    let combined = items[0].union(&items[1]);
    println!("{:?}", combined);
}
