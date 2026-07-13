mod common;

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use hypermesh::{
    BooleanOp, EmberConfig, Point3, Real, boolean_operation, convex_hull, prepare_input,
    triangulate_and_resolve_certified,
};

fn bench_end_to_end(c: &mut Criterion) {
    let cubes = common::cube_pair();
    let nested_cubes = common::nested_cube_pair();
    let octahedra = common::octahedron_pair();

    c.bench_function("prepare_input/cube_pair", |b| {
        b.iter(|| {
            prepare_input(black_box(&[cubes[0].as_ref(), cubes[1].as_ref()]))
                .expect("benchmark mesh is valid")
        })
    });

    let mut group = c.benchmark_group("boolean_operation");
    group.sample_size(20);
    group.warm_up_time(Duration::from_secs(1));
    group.measurement_time(Duration::from_secs(4));
    for (name, meshes) in [
        ("cubes", &cubes),
        ("nested_cubes", &nested_cubes),
        ("octahedra", &octahedra),
    ] {
        for op in [
            BooleanOp::Union,
            BooleanOp::Intersection,
            BooleanOp::Difference,
            BooleanOp::SymmetricDifference,
        ] {
            group.bench_with_input(BenchmarkId::new(name, format!("{op:?}")), &op, |b, op| {
                b.iter(|| {
                    boolean_operation(
                        black_box(&[meshes[0].as_ref(), meshes[1].as_ref()]),
                        *op,
                        EmberConfig::default(),
                    )
                    .expect("benchmark boolean is certified")
                })
            });
        }
    }
    group.finish();

    let cube_union = boolean_operation(
        &[cubes[0].as_ref(), cubes[1].as_ref()],
        BooleanOp::Union,
        EmberConfig::default(),
    )
    .expect("benchmark boolean is certified");
    c.bench_function("output/cube_union_triangulate_certified", |b| {
        b.iter(|| {
            triangulate_and_resolve_certified(black_box(&cube_union))
                .expect("benchmark output is certified")
        })
    });

    let hull_points = (-8..=8)
        .flat_map(|x| {
            (-8..=8).flat_map(move |y| {
                (-8..=8).map(move |z| Point3::new(Real::from(x), Real::from(y), Real::from(z)))
            })
        })
        .collect::<Vec<_>>();
    c.bench_function("convex_hull/grid_4913", |b| {
        b.iter(|| convex_hull(black_box(&hull_points)).expect("point set spans 3D"))
    });

    let moment_curve = (-32_i64..32)
        .map(|t| Point3::new(Real::from(t), Real::from(t * t), Real::from(t * t * t)))
        .collect::<Vec<_>>();
    c.bench_function("convex_hull/moment_curve_64", |b| {
        b.iter(|| convex_hull(black_box(&moment_curve)).expect("point set spans 3D"))
    });
}

criterion_group!(benches, bench_end_to_end);
criterion_main!(benches);
