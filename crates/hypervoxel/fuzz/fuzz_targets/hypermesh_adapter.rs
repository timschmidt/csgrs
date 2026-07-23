#![no_main]

use hypermesh::{InputMesh, Point3, Real, Triangle};
use hypervoxel::{PreparedExactTriangleSolidMesh, adapt_hypermesh_exact_solid};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: (u8, bool, bool)| {
    let (scale_raw, exact_source, closed_policy) = data;
    let scale = i64::from((scale_raw % 5) + 1);
    let idx = [0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
    let coordinate = |value: i64| {
        if exact_source {
            Real::from(value)
        } else {
            Real::try_from(value as f64).unwrap()
        }
    };
    let point = |x, y, z| Point3::new(coordinate(x), coordinate(y), coordinate(z));
    let triangles = if closed_policy {
        idx.chunks_exact(3)
            .map(|triangle| Triangle::new(triangle[0], triangle[1], triangle[2]))
            .collect()
    } else {
        vec![Triangle::new(0, 2, 1)]
    };
    let mesh = InputMesh::new(
        vec![
            point(0, 0, 0),
            point(scale, 0, 0),
            point(0, scale, 0),
            point(0, 0, scale),
        ],
        triangles,
    );

    let adapter = adapt_hypermesh_exact_solid(&mesh);
    if closed_policy {
        let prepared = PreparedExactTriangleSolidMesh::prepare(adapter.unwrap()).unwrap();
        assert!(prepared.report().exact_prepared_solid_ready);
    } else {
        assert!(adapter.is_err());
    }
});
