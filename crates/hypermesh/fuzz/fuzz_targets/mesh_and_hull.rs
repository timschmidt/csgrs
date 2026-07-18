#![no_main]

use hypermesh::{
    InputMesh, Point3, Real, Triangle, convex_hull, convex_hull_with_coplanar_groups, prepare_input,
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
    if let Ok(mut prepared) = prepare_input(&[mesh.as_ref()]) {
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
        let prepared = prepare_input(&[hull.as_ref()]).unwrap();
        assert!(prepared.polygons.iter().all(|polygon| polygon.is_valid()));
    }
});
