//! Stress tests for adversarial geometry workloads.

use csgrs::csg::CSG;
use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::Mesh;
use csgrs::mesh::metaballs::MetaBall;
use csgrs::mesh::plane::Plane;
use csgrs::sketch::Sketch;
use nalgebra::{Point2, Point3, Vector3};
use std::panic::{AssertUnwindSafe, catch_unwind};

fn assert_mesh_finite<S: Clone + Send + Sync + std::fmt::Debug>(mesh: &Mesh<S>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.x.is_finite());
        assert!(vertex.normal.y.is_finite());
        assert!(vertex.normal.z.is_finite());
    }
}

fn assert_sketch_finite<S: Clone + Send + Sync + std::fmt::Debug>(sketch: &Sketch<S>) {
    for polygon in sketch.to_multipolygon().0 {
        for coord in &polygon.exterior().0 {
            assert!(coord.x.is_finite());
            assert!(coord.y.is_finite());
        }
        for ring in polygon.interiors() {
            for coord in &ring.0 {
                assert!(coord.x.is_finite());
                assert!(coord.y.is_finite());
            }
        }
    }
}

fn regular_ring(count: usize, radius: Real, wobble: Real) -> Vec<[Real; 2]> {
    (0..count)
        .map(|i| {
            let t = (i as Real) * std::f64::consts::TAU as Real / count as Real;
            let r = radius + wobble * ((i % 7) as Real - 3.0);
            [r * t.cos(), r * t.sin()]
        })
        .chain(std::iter::once([radius, 0.0]))
        .collect()
}

#[test]
fn adversarial_stress_bounded_boolean_chains_and_cache_reuse() {
    let mut left_fold = Mesh::<()>::new();
    let mut parts = Vec::new();
    for i in 0..16 {
        let cube = Mesh::cube(0.5, None).translate(i as Real * 0.45, 0.0, 0.0);
        left_fold = if i == 0 {
            cube.clone()
        } else {
            left_fold.union(&cube)
        };
        parts.push(cube);
    }

    let mut balanced = parts;
    while balanced.len() > 1 {
        let mut next = Vec::new();
        for pair in balanced.chunks(2) {
            if pair.len() == 2 {
                next.push(pair[0].union(&pair[1]));
            } else {
                next.push(pair[0].clone());
            }
        }
        balanced = next;
    }

    assert_mesh_finite(&left_fold);
    assert_mesh_finite(&balanced[0]);

    let before = left_fold.bounding_box();
    for _ in 0..64 {
        assert_eq!(before.mins, left_fold.bounding_box().mins);
        assert_eq!(before.maxs, left_fold.bounding_box().maxs);
    }
    let mut invalidated = left_fold.clone();
    invalidated.invalidate_bounding_box();
    assert_eq!(before.mins, invalidated.bounding_box().mins);
    assert_eq!(before.maxs, invalidated.bounding_box().maxs);
}

#[test]
fn adversarial_stress_high_vertex_triangulate_offset_sweep_and_slice() {
    for count in [100usize, 256, 512] {
        let points = regular_ring(count, 10.0, 0.01);
        let sketch = Sketch::<()>::polygon(&points, None);
        assert_sketch_finite(&sketch);
        let triangles = sketch.triangulate();
        for tri in triangles {
            for point in tri {
                assert!(point.x.is_finite());
                assert!(point.y.is_finite());
                assert!(point.z.is_finite());
            }
        }

        let offset = sketch.offset(0.05);
        assert_sketch_finite(&offset);
        let path = (0..128)
            .map(|i| {
                Point3::new(
                    (i as Real * 0.05).sin(),
                    (i as Real * 0.05).cos(),
                    i as Real * 0.02,
                )
            })
            .collect::<Vec<_>>();
        let swept = Sketch::<()>::circle(0.1, 12, None).sweep(&path);
        assert_mesh_finite(&swept);
        let slice = swept.slice(Plane::from_normal(Vector3::z(), 0.5));
        assert_sketch_finite(&slice);
    }
}

#[test]
fn adversarial_stress_sdf_tpms_and_metaball_resolution_ladder() {
    for resolution in [2usize, 4, 8, 12] {
        let sdf = Mesh::<()>::sdf(
            |p| p.coords.norm() - 0.75,
            (resolution, resolution, resolution),
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
            0.0,
            None,
        );
        assert_mesh_finite(&sdf);

        let cube = Mesh::<()>::cube(2.0, None);
        for mesh in [
            cube.gyroid(resolution, 1.0, 0.0, None),
            cube.schwarz_p(resolution, 1.0, 0.0, None),
            cube.schwarz_d(resolution, 1.0, 0.0, None),
        ] {
            assert_mesh_finite(&mesh);
        }

        let balls = (0..resolution.min(10))
            .map(|i| MetaBall::new(Point3::new(i as Real * 0.25, 0.0, 0.0), 0.5))
            .collect::<Vec<_>>();
        let mesh_balls = Mesh::<()>::metaballs(
            &balls,
            (resolution, resolution, resolution),
            0.5,
            0.1,
            None,
        );
        assert_mesh_finite(&mesh_balls);

        let sketch_balls = balls
            .iter()
            .map(|ball| (Point2::new(ball.center.x, ball.center.y), ball.radius))
            .collect::<Vec<_>>();
        let sketch =
            Sketch::<()>::metaballs(&sketch_balls, (resolution, resolution), 0.5, 0.1, None);
        assert_sketch_finite(&sketch);
    }
}

#[test]
fn adversarial_stress_concrete_operation_chains_are_contained() {
    let invalid = Sketch::<()>::polygon(
        &[[0.0, 0.0], [1.0, 1.0], [0.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
        None,
    );
    let invalid_chain = catch_unwind(AssertUnwindSafe(|| {
        invalid
            .offset(0.05)
            .offset(-0.05)
            .extrude(0.5)
            .union(&Mesh::cube(0.5, None))
            .triangulate()
    }));
    if let Ok(mesh) = invalid_chain {
        assert_mesh_finite(&mesh);
        let _ = mesh.to_stl_ascii("invalid_chain");
        let _ = mesh.to_obj("invalid_chain");
    }

    let scaled = Mesh::<()>::cube(1.0, None)
        .scale(0.0, 1.0, 1.0)
        .union(&Mesh::sphere(0.5, 8, 8, None))
        .triangulate();
    assert_mesh_finite(&scaled);
    let _ = catch_unwind(AssertUnwindSafe(|| scaled.mass_properties(1.0)));

    let tiny_huge = Mesh::<()>::cuboid(tolerance(), tolerance(), tolerance(), None)
        .translate(1.0e6, -1.0e6, 1.0e6)
        .center()
        .float();
    assert_mesh_finite(&tiny_huge);
}
