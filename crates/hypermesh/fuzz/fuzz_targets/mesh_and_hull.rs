#![no_main]

use hypermesh::{
    ExactGpuVertex, InputMesh, Point3, Real, Triangle, approximate_gpu_mesh_f32,
    approximate_gpu_mesh_f64, approximate_interleaved_gpu_mesh_f32,
    approximate_interleaved_gpu_mesh_f64, convex_hull, convex_hull_with_coplanar_groups,
    build_polygon_soup,
};
use libfuzzer_sys::fuzz_target;

fn r(value: i64) -> Real {
    Real::from(value)
}

fn p(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn tetrahedron() -> InputMesh {
    InputMesh::new(
        vec![p(0, 0, 0), p(3, 0, 0), p(0, 3, 0), p(0, 0, 3)],
        vec![
            Triangle::new(0, 2, 1),
            Triangle::new(0, 1, 3),
            Triangle::new(0, 3, 2),
            Triangle::new(1, 2, 3),
        ],
    )
}

fuzz_target!(|data: [u8; 24]| {
    let mut mesh = tetrahedron();
    if data[0] & 1 != 0 {
        mesh.positions.push(p(
            i64::from(data[1] % 7) - 3,
            i64::from(data[2] % 7) - 3,
            i64::from(data[3] % 7) - 3,
        ));
    }
    if data[0] & 2 != 0 {
        mesh.triangles.push(Triangle::new(
            usize::from(data[4] % 7),
            usize::from(data[5] % 7),
            usize::from(data[6] % 7),
        ));
    }
    if let Ok(mut prepared) = build_polygon_soup(&[mesh.as_ref()]) {
        assert_eq!(prepared.num_meshes, 1);
        assert!(prepared.polygons.iter().all(|polygon| polygon.is_valid()));
        prepared.compute_bounds_from_vertices().unwrap();
    }

    let mut points = vec![p(0, 0, 0), p(4, 0, 0), p(0, 4, 0), p(0, 0, 4)];
    let extra_count = usize::from(data[7] % 8);
    for index in 0..extra_count {
        let base = 8 + index * 2;
        points.push(p(
            i64::from(data[base] % 9) - 4,
            i64::from(data[base + 1] % 9) - 4,
            i64::from(data[(base + 8) % data.len()] % 9) - 4,
        ));
    }

    for hull in [
        convex_hull(&points),
        convex_hull_with_coplanar_groups(&points, &[]),
    ]
    .into_iter()
    .flatten()
    {
        assert!(hull.positions.len() >= 4);
        assert!(hull.triangles.iter().all(|triangle| {
            triangle
                .indices()
                .into_iter()
                .all(|index| index < hull.positions.len())
        }));
        let prepared = build_polygon_soup(&[hull.as_ref()]).unwrap();
        assert!(prepared.polygons.iter().all(|polygon| polygon.is_valid()));
    }

    let render_vertices = points
        .iter()
        .map(|point| -> ExactGpuVertex {
            (
                [point.x.clone(), point.y.clone(), point.z.clone()],
                [Real::zero(), Real::zero(), Real::one()],
            )
        })
        .collect::<Vec<_>>();
    let mut render_indices = data[..data.len() / 3 * 3]
        .iter()
        .map(|value| u32::from(*value) % render_vertices.len() as u32)
        .collect::<Vec<_>>();
    if data[0] & 4 != 0 {
        render_indices[0] = render_vertices.len() as u32;
    }

    let separate_f32 = approximate_gpu_mesh_f32(&render_vertices, &render_indices);
    let interleaved_f32 = approximate_interleaved_gpu_mesh_f32(&render_vertices, &render_indices);
    match (separate_f32, interleaved_f32) {
        (Ok(separate), Ok(interleaved)) => {
            assert_eq!(
                interleaved.vertices,
                separate
                    .positions
                    .into_iter()
                    .zip(separate.normals)
                    .collect::<Vec<_>>()
            );
            assert_eq!(interleaved.indices, separate.indices);
        }
        (Err(separate), Err(interleaved)) => assert_eq!(separate, interleaved),
        results => panic!("split/interleaved f32 results differ: {results:?}"),
    }

    let separate_f64 = approximate_gpu_mesh_f64(&render_vertices, &render_indices);
    let interleaved_f64 = approximate_interleaved_gpu_mesh_f64(&render_vertices, &render_indices);
    match (separate_f64, interleaved_f64) {
        (Ok(separate), Ok(interleaved)) => {
            assert_eq!(
                interleaved.vertices,
                separate
                    .positions
                    .into_iter()
                    .zip(separate.normals)
                    .collect::<Vec<_>>()
            );
            assert_eq!(interleaved.indices, separate.indices);
        }
        (Err(separate), Err(interleaved)) => assert_eq!(separate, interleaved),
        results => panic!("split/interleaved f64 results differ: {results:?}"),
    }
});
