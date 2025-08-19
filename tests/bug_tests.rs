use csgrs::{
    mesh::{Mesh, polygon::Polygon, vertex::Vertex},
    traits::CSG,
};
use nalgebra::{Point3, Vector3};

#[test]
fn union_crash() {
    let items: [Mesh<()>; 2] = [
        Mesh::from_polygons(
            &[
                Polygon::new(
                    vec![
                        Vertex {
                            pos: Point3::new(640.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 0.0, 128.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 0.0, 256.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1024.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            pos: Point3::new(1024.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 256.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 256.0, 128.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(640.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            pos: Point3::new(640.0, 0.0, 640.0),
                            normal: Vector3::new(
                                0.9701425433158875,
                                -0.0,
                                0.24253563582897186,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(640.0, 256.0, 640.0),
                            normal: Vector3::new(
                                0.9701425433158875,
                                -0.0,
                                0.24253563582897186,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 256.0, 128.0),
                            normal: Vector3::new(
                                0.9701425433158875,
                                -0.0,
                                0.24253563582897186,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 0.0, 128.0),
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
                            pos: Point3::new(768.0, 0.0, 128.0),
                            normal: Vector3::new(
                                -0.24253563582897186,
                                0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 256.0, 128.0),
                            normal: Vector3::new(
                                -0.24253563582897186,
                                0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 256.0),
                            normal: Vector3::new(
                                -0.24253563582897186,
                                0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 0.0, 256.0),
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
                            pos: Point3::new(1280.0, 0.0, 256.0),
                            normal: Vector3::new(
                                -0.8320503234863281,
                                0.0,
                                -0.5547001957893372,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 256.0),
                            normal: Vector3::new(
                                -0.8320503234863281,
                                0.0,
                                -0.5547001957893372,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1024.0, 256.0, 640.0),
                            normal: Vector3::new(
                                -0.8320503234863281,
                                0.0,
                                -0.5547001957893372,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1024.0, 0.0, 640.0),
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
                            pos: Point3::new(1024.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, 0.0, -1.0),
                        },
                        Vertex {
                            pos: Point3::new(1024.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 0.0, -1.0),
                        },
                        Vertex {
                            pos: Point3::new(640.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 0.0, -1.0),
                        },
                        Vertex {
                            pos: Point3::new(640.0, 0.0, 640.0),
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
                            pos: Point3::new(896.0, 0.0, 768.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 0.0, 512.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 0.0, 384.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 0.0, 640.0),
                            normal: Vector3::new(0.0, -1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 640.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 384.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 256.0, 512.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(896.0, 256.0, 768.0),
                            normal: Vector3::new(0.0, 1.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            pos: Point3::new(896.0, 0.0, 768.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                        Vertex {
                            pos: Point3::new(896.0, 256.0, 768.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 256.0, 512.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 0.0, 512.0),
                            normal: Vector3::new(0.8944271802902222, 0.0, -0.4472135901451111),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            pos: Point3::new(768.0, 0.0, 512.0),
                            normal: Vector3::new(
                                0.24253563582897186,
                                -0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(768.0, 256.0, 512.0),
                            normal: Vector3::new(
                                0.24253563582897186,
                                -0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 384.0),
                            normal: Vector3::new(
                                0.24253563582897186,
                                -0.0,
                                0.9701425433158875,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 0.0, 384.0),
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
                            pos: Point3::new(1280.0, 0.0, 384.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 384.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 640.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 0.0, 640.0),
                            normal: Vector3::new(-1.0, 0.0, 0.0),
                        },
                    ],
                    None,
                ),
                Polygon::new(
                    vec![
                        Vertex {
                            pos: Point3::new(1280.0, 0.0, 640.0),
                            normal: Vector3::new(
                                -0.3162277638912201,
                                0.0,
                                -0.9486832618713379,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(1280.0, 256.0, 640.0),
                            normal: Vector3::new(
                                -0.3162277638912201,
                                0.0,
                                -0.9486832618713379,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(896.0, 256.0, 768.0),
                            normal: Vector3::new(
                                -0.3162277638912201,
                                0.0,
                                -0.9486832618713379,
                            ),
                        },
                        Vertex {
                            pos: Point3::new(896.0, 0.0, 768.0),
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
