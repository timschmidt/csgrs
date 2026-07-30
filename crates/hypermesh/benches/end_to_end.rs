mod common;

use std::hint::black_box;
use std::time::Duration;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use hypermesh::{
    BooleanOp, EmberConfig, ExactGpuMeshBuffers, MeshContext, Point3, PredicatePolicy, Real,
    TriangleMeshRef, approximate_gpu_mesh_f32, approximate_gpu_mesh_f64,
    approximate_interleaved_gpu_mesh_f32, approximate_interleaved_gpu_mesh_f64, boolean_mesh,
    boolean_operation, convex_hull, convex_hull_with_coplanar_groups,
    convex_hull_with_retained_facts, convex_quad, convex_triangle, extract_output, polygon_soup,
    triangulate_and_resolve_certified,
};

const CONTEXT: MeshContext = MeshContext::new(PredicatePolicy::APPROXIMATE_512);

fn curved_shell(segments: usize, stacks: usize) -> Vec<Point3> {
    let mut unique = Vec::with_capacity(segments * (stacks - 1) + 2);
    unique.push(Point3::new(Real::zero(), Real::one(), Real::zero()));
    for longitude in 0..segments {
        let theta = std::f64::consts::TAU * longitude as f64 / segments as f64;
        let (sin_theta, cos_theta) = theta.sin_cos();
        for latitude in 1..stacks {
            let phi = std::f64::consts::PI * latitude as f64 / stacks as f64;
            let (sin_phi, cos_phi) = phi.sin_cos();
            unique.push(Point3::new(
                Real::try_from(cos_theta * sin_phi).expect("finite shell coordinate"),
                Real::try_from(cos_phi).expect("finite shell coordinate"),
                Real::try_from(sin_theta * sin_phi).expect("finite shell coordinate"),
            ));
        }
    }
    unique.push(Point3::new(Real::zero(), -Real::one(), Real::zero()));

    let mut points = Vec::with_capacity(unique.len() * 6);
    for _ in 0..6 {
        points.extend(unique.iter().cloned());
    }
    points
}

fn bench_end_to_end(c: &mut Criterion) {
    let cubes = common::cube_pair();
    let subdivided_cubes = common::subdivided_cube_pair(4);
    let nested_cubes = common::nested_cube_pair();
    let octahedra = common::octahedron_pair();

    let p0 = Point3::new(Real::zero(), Real::zero(), Real::zero());
    let p1 = Point3::new(Real::one(), Real::zero(), Real::zero());
    let p2 = Point3::new(Real::one(), Real::one(), Real::zero());
    let p3 = Point3::new(Real::zero(), Real::one(), Real::zero());
    c.bench_function("convex_polygon/triangle_construction", |b| {
        b.iter(|| {
            convex_triangle(
                &CONTEXT,
                black_box(&p0),
                black_box(&p1),
                black_box(&p3),
                black_box(0),
                black_box(0),
            )
            .expect("benchmark triangle is valid")
            .into_value()
        })
    });
    c.bench_function("convex_polygon/quad_construction", |b| {
        b.iter(|| {
            convex_quad(
                &CONTEXT,
                black_box(&p0),
                black_box(&p1),
                black_box(&p2),
                black_box(&p3),
                black_box(0),
                black_box(0),
            )
            .expect("benchmark quad is valid")
            .into_value()
        })
    });
    c.bench_function("build_polygon_soup/cube_pair", |b| {
        b.iter(|| {
            polygon_soup(&CONTEXT, black_box(&[cubes[0].as_ref(), cubes[1].as_ref()]))
                .expect("benchmark mesh is valid")
                .into_value()
        })
    });
    c.bench_function("build_polygon_soup/subdivided_cube_pair_3072_each", |b| {
        b.iter(|| {
            polygon_soup(
                &CONTEXT,
                black_box(&[subdivided_cubes[0].as_ref(), subdivided_cubes[1].as_ref()]),
            )
            .expect("benchmark mesh is valid")
            .into_value()
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
                        &CONTEXT,
                        black_box(&[meshes[0].as_ref(), meshes[1].as_ref()]),
                        *op,
                        EmberConfig::default(),
                    )
                    .expect("benchmark boolean is certified")
                    .into_value()
                })
            });
        }
    }
    group.finish();

    let operations = [
        BooleanOp::Union,
        BooleanOp::Intersection,
        BooleanOp::Difference,
        BooleanOp::SymmetricDifference,
    ];
    let cube_refs = [
        TriangleMeshRef::new(&cubes[0].positions, &cubes[0].triangles),
        TriangleMeshRef::new(&cubes[1].positions, &cubes[1].triangles),
    ];
    let certified_cubes = cubes.clone().map(|mesh| mesh.with_certified_convexity());
    let certified_cube_refs = [certified_cubes[0].as_ref(), certified_cubes[1].as_ref()];
    let mut output_group = c.benchmark_group("boolean_immediate_output/cubes");
    output_group.sample_size(20);
    output_group.warm_up_time(Duration::from_secs(1));
    output_group.measurement_time(Duration::from_secs(4));
    for operation in operations {
        output_group.bench_with_input(
            BenchmarkId::new("polygon", format!("{operation:?}")),
            &operation,
            |b, operation| {
                b.iter(|| {
                    boolean_operation(
                        &CONTEXT,
                        black_box(&cube_refs),
                        *operation,
                        EmberConfig::default(),
                    )
                    .expect("cube Boolean is certified")
                    .into_value()
                })
            },
        );
        output_group.bench_with_input(
            BenchmarkId::new("boolean_mesh", format!("{operation:?}")),
            &operation,
            |b, operation| {
                b.iter(|| {
                    boolean_mesh(
                        &CONTEXT,
                        black_box(&cube_refs),
                        *operation,
                        EmberConfig::default(),
                    )
                    .expect("cube triangle soup is certified")
                    .into_value()
                })
            },
        );
        output_group.bench_with_input(
            BenchmarkId::new("certified_convex_boolean_mesh", format!("{operation:?}")),
            &operation,
            |b, operation| {
                b.iter(|| {
                    boolean_mesh(
                        &CONTEXT,
                        black_box(&certified_cube_refs),
                        *operation,
                        EmberConfig::default(),
                    )
                    .expect("certified-convex cube triangle soup is certified")
                    .into_value()
                })
            },
        );
    }
    output_group.finish();

    let nested_tools = common::nested_tool_cubes();
    let nested_tool_refs = nested_tools
        .iter()
        .map(|mesh| mesh.as_ref())
        .collect::<Vec<_>>();
    c.bench_function("boolean_operation/nested_tools_5/Difference", |b| {
        b.iter(|| {
            boolean_operation(
                &CONTEXT,
                black_box(&nested_tool_refs),
                BooleanOp::Difference,
                EmberConfig::default(),
            )
            .expect("benchmark variadic difference is certified")
            .into_value()
        })
    });

    let subdivided_cubes = common::subdivided_cube_pair(2);
    let mut large_group = c.benchmark_group("boolean_operation/subdivided_cubes_192");
    large_group.sample_size(10);
    large_group.warm_up_time(Duration::from_secs(1));
    large_group.measurement_time(Duration::from_secs(4));
    large_group.bench_function("Union", |b| {
        b.iter(|| {
            boolean_operation(
                &CONTEXT,
                black_box(&[subdivided_cubes[0].as_ref(), subdivided_cubes[1].as_ref()]),
                BooleanOp::Union,
                EmberConfig::default(),
            )
            .expect("subdivided cube benchmark is certified")
            .into_value()
        })
    });
    large_group.finish();

    let cube_union = boolean_operation(
        &CONTEXT,
        &[cubes[0].as_ref(), cubes[1].as_ref()],
        BooleanOp::Union,
        EmberConfig::default(),
    )
    .expect("benchmark boolean is certified")
    .into_value();
    c.bench_function("output/cube_union_triangulate_certified", |b| {
        b.iter(|| {
            triangulate_and_resolve_certified(&CONTEXT, black_box(&cube_union))
                .expect("benchmark output is certified")
                .into_value()
        })
    });
    c.bench_function("output/cube_union_extract_public_views", |b| {
        b.iter(|| {
            let result = black_box(&cube_union);
            let owned = extract_output(&CONTEXT, result)
                .expect("benchmark output extraction is certified")
                .into_value();
            let borrowed =
                hypermesh::output::extract_output_polygons(&CONTEXT, &result.output().polygons)
                    .expect("benchmark borrowed output extraction is certified")
                    .into_value();
            (owned, borrowed)
        })
    });

    let render_vertex = (
        [Real::from(1), Real::from(2), Real::from(3)],
        [Real::zero(), Real::zero(), Real::one()],
    );
    let render_buffers = ExactGpuMeshBuffers::from_triangles_with_capacity(
        16_128,
        (0..16_128).map(|_| {
            [
                render_vertex.clone(),
                render_vertex.clone(),
                render_vertex.clone(),
            ]
        }),
    )
    .expect("large render corpus fits u32 indices");
    let mut gpu_group = c.benchmark_group("gpu_export/vertices_48384");
    gpu_group.bench_function("separate_f32", |b| {
        b.iter(|| {
            approximate_gpu_mesh_f32(
                black_box(&render_buffers.vertices),
                black_box(&render_buffers.indices),
            )
            .expect("render corpus approximates to f32")
        })
    });
    gpu_group.bench_function("interleaved_f32", |b| {
        b.iter(|| {
            approximate_interleaved_gpu_mesh_f32(
                black_box(&render_buffers.vertices),
                black_box(&render_buffers.indices),
            )
            .expect("render corpus approximates to interleaved f32")
        })
    });
    gpu_group.bench_function("separate_f64", |b| {
        b.iter(|| {
            approximate_gpu_mesh_f64(
                black_box(&render_buffers.vertices),
                black_box(&render_buffers.indices),
            )
            .expect("render corpus approximates to f64")
        })
    });
    gpu_group.bench_function("interleaved_f64", |b| {
        b.iter(|| {
            approximate_interleaved_gpu_mesh_f64(
                black_box(&render_buffers.vertices),
                black_box(&render_buffers.indices),
            )
            .expect("render corpus approximates to interleaved f64")
        })
    });
    gpu_group.finish();

    let hull_points = (-8..=8)
        .flat_map(|x| {
            (-8..=8).flat_map(move |y| {
                (-8..=8).map(move |z| Point3::new(Real::from(x), Real::from(y), Real::from(z)))
            })
        })
        .collect::<Vec<_>>();
    c.bench_function("convex_hull/grid_4913", |b| {
        b.iter(|| {
            convex_hull(&CONTEXT, black_box(&hull_points))
                .expect("point set spans 3D")
                .into_value()
        })
    });

    let moment_curve = (-32_i64..32)
        .map(|t| Point3::new(Real::from(t), Real::from(t * t), Real::from(t * t * t)))
        .collect::<Vec<_>>();
    c.bench_function("convex_hull/moment_curve_64", |b| {
        b.iter(|| {
            convex_hull(&CONTEXT, black_box(&moment_curve))
                .expect("point set spans 3D")
                .into_value()
        })
    });

    let curved_shell = curved_shell(16, 8);
    c.bench_function("convex_hull/curved_shell_684", |b| {
        b.iter(|| {
            convex_hull(&CONTEXT, black_box(&curved_shell))
                .expect("point set spans 3D")
                .into_value()
        })
    });

    let retained_points = vec![
        Point3::new(Real::from(0), Real::from(0), Real::from(0)),
        Point3::new(Real::from(2), Real::from(0), Real::from(0)),
        Point3::new(Real::from(0), Real::from(2), Real::from(0)),
        Point3::new(Real::from(0), Real::from(0), Real::from(2)),
    ];
    let retained_coordinate_ids = vec![
        [0, 1, 2, 10, 20],
        [3, 4, 5, 10, 21],
        [6, 7, 8, 10, 22],
        [9, 10, 11, 10, 23],
    ];
    c.bench_function("convex_hull/tetra_coplanar_group_api", |b| {
        b.iter(|| {
            convex_hull_with_coplanar_groups(&CONTEXT, black_box(&retained_points), black_box(&[]))
                .expect("tetrahedron spans 3D")
                .into_value()
        })
    });
    c.bench_function("convex_hull/tetra_retained_facts_api", |b| {
        b.iter(|| {
            convex_hull_with_retained_facts(
                &CONTEXT,
                black_box(&retained_points),
                black_box(&[]),
                black_box(&retained_coordinate_ids),
            )
            .expect("retained tetrahedron spans 3D")
            .into_value()
        })
    });
}

criterion_group!(benches, bench_end_to_end);
criterion_main!(benches);
