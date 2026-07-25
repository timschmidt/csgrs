#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod support;

use std::{hint::black_box, time::Duration};

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use support::{
    Operation, corpus, large_boolean_case, prepare, prepare_yeahright, run_boolmesh, run_hypermesh,
    run_manifold, summarize, to_boolmesh, to_hypermesh, to_manifold, to_three_d_asset,
    validate_with_tri_mesh, yeahright_boolean_case,
};

fn competitive(c: &mut Criterion) {
    let cases = corpus();
    let mut boolean_group = c.benchmark_group("competitive_boolean");
    boolean_group.sample_size(20);
    boolean_group.warm_up_time(Duration::from_secs(1));
    boolean_group.measurement_time(Duration::from_secs(4));

    for case in &cases {
        let inputs = prepare(case);
        for operation in Operation::ALL {
            let workload = format!("{}/{}", case.name, operation.name());
            boolean_group.bench_function(BenchmarkId::new("hypermesh", &workload), |benchmark| {
                benchmark.iter(|| run_hypermesh(black_box(&inputs.hypermesh), operation));
            });
            boolean_group.bench_function(BenchmarkId::new("boolmesh", &workload), |benchmark| {
                benchmark.iter(|| run_boolmesh(black_box(&inputs.boolmesh), operation));
            });
            boolean_group.bench_function(
                BenchmarkId::new("manifold-rust", &workload),
                |benchmark| {
                    benchmark.iter(|| run_manifold(black_box(&inputs.manifold), operation));
                },
            );
        }
    }
    boolean_group.finish();

    let large_case = large_boolean_case();
    let large_inputs = prepare(&large_case);
    let mut large_boolean_group = c.benchmark_group("competitive_large_boolean");
    large_boolean_group.sample_size(10);
    large_boolean_group.warm_up_time(Duration::from_secs(1));
    large_boolean_group.measurement_time(Duration::from_secs(5));
    for operation in Operation::ALL {
        let workload = format!("{}/{}", large_case.name, operation.name());
        large_boolean_group.bench_function(BenchmarkId::new("hypermesh", &workload), |benchmark| {
            benchmark.iter(|| run_hypermesh(black_box(&large_inputs.hypermesh), operation));
        });
        large_boolean_group.bench_function(BenchmarkId::new("boolmesh", &workload), |benchmark| {
            benchmark.iter(|| run_boolmesh(black_box(&large_inputs.boolmesh), operation));
        });
        large_boolean_group.bench_function(
            BenchmarkId::new("manifold-rust", &workload),
            |benchmark| {
                benchmark.iter(|| run_manifold(black_box(&large_inputs.manifold), operation));
            },
        );
    }
    large_boolean_group.finish();

    let yeahright_case = yeahright_boolean_case();
    let yeahright_inputs = prepare_yeahright(&yeahright_case);
    let mut yeahright_boolean_group = c.benchmark_group("competitive_yeahright_boolean");
    yeahright_boolean_group.sample_size(10);
    yeahright_boolean_group.warm_up_time(Duration::from_secs(1));
    yeahright_boolean_group.measurement_time(Duration::from_secs(5));
    for operation in Operation::ALL {
        let workload = format!("{}/{}", yeahright_case.name, operation.name());
        yeahright_boolean_group.bench_function(
            BenchmarkId::new("hypermesh", &workload),
            |benchmark| {
                benchmark.iter(|| run_hypermesh(black_box(&yeahright_inputs.hypermesh), operation));
            },
        );
        yeahright_boolean_group.bench_function(
            BenchmarkId::new("boolmesh", &workload),
            |benchmark| {
                benchmark.iter(|| run_boolmesh(black_box(&yeahright_inputs.boolmesh), operation));
            },
        );
        yeahright_boolean_group.bench_function(
            BenchmarkId::new("manifold-rust", &workload),
            |benchmark| {
                benchmark.iter(|| run_manifold(black_box(&yeahright_inputs.manifold), operation));
            },
        );
    }
    yeahright_boolean_group.finish();

    let fixture = &cases[0].left;
    let mut import_group = c.benchmark_group("competitive_mesh_import/box_12");
    import_group.bench_function("hypermesh", |benchmark| {
        benchmark.iter(|| to_hypermesh(black_box(fixture)));
    });
    import_group.bench_function("boolmesh", |benchmark| {
        benchmark.iter(|| to_boolmesh(black_box(fixture)));
    });
    import_group.bench_function("manifold-rust", |benchmark| {
        benchmark.iter(|| to_manifold(black_box(fixture)));
    });
    import_group.bench_function("tri-mesh", |benchmark| {
        benchmark.iter(|| {
            let asset = to_three_d_asset(black_box(fixture));
            tri_mesh::Mesh::new(&asset)
        });
    });
    import_group.finish();

    let mut large_import_group =
        c.benchmark_group("competitive_large_mesh_import/subdivided_box_3072");
    large_import_group.bench_function("hypermesh", |benchmark| {
        benchmark.iter(|| to_hypermesh(black_box(&large_case.left)));
    });
    large_import_group.bench_function("boolmesh", |benchmark| {
        benchmark.iter(|| to_boolmesh(black_box(&large_case.left)));
    });
    large_import_group.bench_function("manifold-rust", |benchmark| {
        benchmark.iter(|| to_manifold(black_box(&large_case.left)));
    });
    large_import_group.bench_function("tri-mesh", |benchmark| {
        benchmark.iter(|| {
            let asset = to_three_d_asset(black_box(&large_case.left));
            tri_mesh::Mesh::new(&asset)
        });
    });
    large_import_group.finish();

    let mut yeahright_import_group =
        c.benchmark_group("competitive_yeahright_mesh_import/subdivided_hull_4512");
    yeahright_import_group.bench_function("hypermesh", |benchmark| {
        benchmark.iter(|| to_hypermesh(black_box(&yeahright_case.left)));
    });
    yeahright_import_group.bench_function("boolmesh", |benchmark| {
        benchmark.iter(|| to_boolmesh(black_box(&yeahright_case.left)));
    });
    yeahright_import_group.bench_function("manifold-rust", |benchmark| {
        benchmark.iter(|| to_manifold(black_box(&yeahright_case.left)));
    });
    yeahright_import_group.bench_function("tri-mesh", |benchmark| {
        benchmark.iter(|| {
            let asset = to_three_d_asset(black_box(&yeahright_case.left));
            tri_mesh::Mesh::new(&asset)
        });
    });
    yeahright_import_group.finish();

    let hypermesh_output = run_hypermesh(&prepare(&cases[0]).hypermesh, Operation::Union);
    let mut topology_group = c.benchmark_group("competitive_topology/box_union");
    topology_group.bench_function("hypermesh_summary", |benchmark| {
        benchmark.iter(|| summarize(black_box(&hypermesh_output)));
    });
    topology_group.bench_function("tri-mesh_validate", |benchmark| {
        benchmark.iter(|| validate_with_tri_mesh(black_box(&hypermesh_output)));
    });
    topology_group.finish();
}

criterion_group!(benches, competitive);
criterion_main!(benches);
