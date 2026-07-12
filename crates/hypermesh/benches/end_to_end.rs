mod common;

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use hypermesh::{
    BooleanOp, EmberConfig, boolean_operation, prepare_input, triangulate_and_resolve_certified,
};

fn bench_end_to_end(c: &mut Criterion) {
    let cubes = common::cube_pair();
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
    for (name, meshes) in [("cubes", &cubes), ("octahedra", &octahedra)] {
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
}

criterion_group!(benches, bench_end_to_end);
criterion_main!(benches);
