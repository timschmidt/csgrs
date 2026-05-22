use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hyperlattice::{Matrix4, Vector3};
use hyperlimit::{Plane3, Point3};
use hyperreal::Real;
use hypersdf::{
    SdfCoordinate, SdfExpr, SdfGradientContourReport, SdfHandoffDomain, SdfPreviewGrid,
    SdfProjectionProposal, SdfProjectionProposalKind, SdfSamplingPrecision, SdfVoxelCellGrid,
    SdfVoxelGridSource, prepare,
};

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn bench_point_classification(c: &mut Criterion) {
    let plane = prepare(SdfExpr::plane(Plane3::new(p(0, 0, 1), r(-5))));
    let slab = prepare(SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(5)));
    let cylinder = prepare(SdfExpr::cylinder(
        SdfCoordinate::Z,
        p(0, 0, 0),
        r(100),
        r(5),
    ));
    let capsule = prepare(SdfExpr::capsule(SdfCoordinate::Z, p(0, 0, 0), r(100), r(5)));
    let torus = prepare(SdfExpr::torus(SdfCoordinate::Z, p(0, 0, 0), r(25), r(4)));
    let sphere = prepare(SdfExpr::sphere(p(0, 0, 0), r(100)));
    let aabb = prepare(SdfExpr::aabb(p(-10, -10, -10), p(10, 10, 10)));
    let rounded_aabb = prepare(SdfExpr::rounded_aabb(
        p(-10, -10, -10),
        p(10, 10, 10),
        r(25),
    ));
    let csg =
        prepare(SdfExpr::sphere(p(-4, 0, 0), r(25)).union(SdfExpr::sphere(p(4, 0, 0), r(25))));
    let translated = prepare(SdfExpr::sphere(p(0, 0, 0), r(100)).translate(p(10, 0, 0)));
    let swap_xy = Matrix4([
        [r(0), r(1), r(0), r(0)],
        [r(1), r(0), r(0), r(0)],
        [r(0), r(0), r(1), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let affine = prepare(
        SdfExpr::x()
            .affine_transform(swap_xy)
            .expect("invertible affine transform"),
    );
    let offset = prepare(SdfExpr::aabb(p(-10, -10, -10), p(10, 10, 10)).offset(r(2)));
    let coordinate = prepare(SdfExpr::x().offset(r(1)));
    let linear = prepare(SdfExpr::linear(Vector3([r(2), r(-3), r(5)]), r(-7)));
    let arithmetic = prepare(
        SdfExpr::x()
            .add_expr(SdfExpr::y())
            .mul_expr(SdfExpr::z().abs()),
    );
    let sqrt_expr = prepare(SdfExpr::x().mul_expr(SdfExpr::x()).sqrt());
    let trig_expr = prepare(SdfExpr::x().sin().add_expr(SdfExpr::y().cos()));
    let point = p(3, 4, 5);

    c.bench_function("hypersdf plane point classification", |b| {
        b.iter(|| plane.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf slab point classification", |b| {
        b.iter(|| slab.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf cylinder point classification", |b| {
        b.iter(|| cylinder.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf capsule point classification", |b| {
        b.iter(|| capsule.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf torus point classification", |b| {
        b.iter(|| torus.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf sphere point classification", |b| {
        b.iter(|| sphere.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf aabb point classification", |b| {
        b.iter(|| aabb.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf rounded aabb point classification", |b| {
        b.iter(|| rounded_aabb.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf csg union point classification", |b| {
        b.iter(|| csg.classify_point(black_box(&point)))
    });

    let points = [
        p(0, 0, 0),
        p(1, 2, 3),
        p(3, 4, 5),
        p(10, 10, 10),
        p(-4, 0, 0),
        p(4, 0, 0),
    ];
    c.bench_function("hypersdf prepared point batch classification", |b| {
        b.iter(|| csg.classify_points(black_box(points.iter())))
    });
    c.bench_function("hypersdf prepared point batch report", |b| {
        b.iter(|| csg.classify_points_report(black_box(points.iter())))
    });
    c.bench_function("hypersdf preview sample points f32", |b| {
        b.iter(|| csg.sample_points_preview(black_box(points.iter()), SdfSamplingPrecision::F32))
    });
    let grid = SdfPreviewGrid::new(p(-2, -2, -2), p(1, 1, 1), [4, 4, 4]);
    c.bench_function("hypersdf preview sample grid f32", |b| {
        b.iter(|| csg.sample_grid_preview(black_box(grid.clone()), SdfSamplingPrecision::F32))
    });
    c.bench_function("hypersdf mesh preview diagnostic", |b| {
        b.iter(|| csg.mesh_preview_from_grid(black_box(grid.clone()), SdfSamplingPrecision::F32))
    });
    let dual_affine = prepare(SdfExpr::x());
    let dual_grid = SdfPreviewGrid::new(p(-1, -1, -1), p(2, 2, 2), [2, 2, 2]);
    c.bench_function("hypersdf dual contouring exact affine report", |b| {
        b.iter(|| {
            dual_affine.dual_contouring_report_from_grid(
                black_box(dual_grid.clone()),
                SdfSamplingPrecision::F64,
            )
        })
    });
    let signed_grid_samples = dual_affine
        .sample_grid_preview(dual_grid.clone(), SdfSamplingPrecision::F64)
        .expect("bench signed grid");
    c.bench_function("hypersdf dual contouring signed-grid proposal", |b| {
        b.iter(|| {
            hypersdf::SdfDualContouringReport::from_signed_grid_samples(black_box(
                signed_grid_samples.clone(),
            ))
        })
    });
    let gradient_grid = SdfPreviewGrid::new(p(-2, 0, 0), p(3, 1, 1), [3, 3, 2]);
    c.bench_function("hypersdf gradient contouring projection report", |b| {
        b.iter(|| {
            dual_affine.gradient_contouring_report_from_grid(
                black_box(gradient_grid.clone()),
                SdfSamplingPrecision::F64,
            )
        })
    });
    let gradient_samples = dual_affine
        .sample_grid_preview(gradient_grid.clone(), SdfSamplingPrecision::F64)
        .expect("bench gradient contour samples");
    c.bench_function("hypersdf gradient contouring signed-grid report", |b| {
        b.iter(|| {
            SdfGradientContourReport::from_signed_grid_samples(black_box(gradient_samples.clone()))
        })
    });
    c.bench_function("hypersdf glsl preview export", |b| {
        b.iter(|| csg.export_glsl_preview(black_box("field"), SdfSamplingPrecision::F32))
    });
    let proposal = SdfProjectionProposal::new(
        "criterion-fixture",
        SdfProjectionProposalKind::ClosestPoint,
        p(10, 0, 0),
        p(5, 0, 0),
    );
    c.bench_function("hypersdf projection replay", |b| {
        b.iter(|| sphere.replay_projection_proposal(black_box(proposal.clone())))
    });
    let package = csg
        .handoff_package()
        .with_grid_samples(
            csg.sample_grid_preview(grid.clone(), SdfSamplingPrecision::F32)
                .expect("bench grid"),
        )
        .with_mesh_preview(
            csg.mesh_preview_from_grid(grid.clone(), SdfSamplingPrecision::F32)
                .expect("bench mesh preview"),
        )
        .with_shader_preview(csg.export_glsl_preview("field", SdfSamplingPrecision::F32))
        .with_projection_replay(sphere.replay_projection_proposal(proposal.clone()));
    c.bench_function("hypersdf handoff package mesh requirement", |b| {
        b.iter(|| package.require_domain(black_box(SdfHandoffDomain::MeshPreview)))
    });
    c.bench_function("hypersdf translated point classification", |b| {
        b.iter(|| translated.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf affine point classification", |b| {
        b.iter(|| affine.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf offset point classification", |b| {
        b.iter(|| offset.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf coordinate point classification", |b| {
        b.iter(|| coordinate.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf linear vector point classification", |b| {
        b.iter(|| linear.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf arithmetic point classification", |b| {
        b.iter(|| arithmetic.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf arithmetic point gradient", |b| {
        b.iter(|| arithmetic.gradient_point(black_box(&point)))
    });
    c.bench_function("hypersdf arithmetic point normal", |b| {
        b.iter(|| arithmetic.normal_point(black_box(&point)))
    });
    c.bench_function("hypersdf sqrt point classification", |b| {
        b.iter(|| sqrt_expr.classify_point(black_box(&point)))
    });
    c.bench_function("hypersdf trig point classification", |b| {
        b.iter(|| trig_expr.classify_point(black_box(&point)))
    });
}

fn bench_cell_classification(c: &mut Criterion) {
    let sphere = prepare(SdfExpr::sphere(p(0, 0, 0), r(10_000)));
    let slab = prepare(SdfExpr::slab(Plane3::new(p(0, 0, 1), r(0)), r(100)));
    let cylinder = prepare(SdfExpr::cylinder(
        SdfCoordinate::Z,
        p(0, 0, 0),
        r(10_000),
        r(100),
    ));
    let capsule = prepare(SdfExpr::capsule(
        SdfCoordinate::Z,
        p(0, 0, 0),
        r(10_000),
        r(100),
    ));
    let torus = prepare(SdfExpr::torus(
        SdfCoordinate::Z,
        p(0, 0, 0),
        r(10_000),
        r(100),
    ));
    let aabb = prepare(SdfExpr::aabb(p(-100, -100, -100), p(100, 100, 100)));
    let rounded_aabb = prepare(SdfExpr::rounded_aabb(
        p(-100, -100, -100),
        p(100, 100, 100),
        r(400),
    ));
    let swap_xy = Matrix4([
        [r(0), r(1), r(0), r(0)],
        [r(1), r(0), r(0), r(0)],
        [r(0), r(0), r(1), r(0)],
        [r(0), r(0), r(0), r(1)],
    ]);
    let affine = prepare(
        SdfExpr::x()
            .affine_transform(swap_xy)
            .expect("invertible affine transform"),
    );
    let linear = prepare(SdfExpr::linear(Vector3([r(2), r(-3), r(5)]), r(-7)));
    let arithmetic = prepare(
        SdfExpr::x()
            .add_expr(SdfExpr::constant(r(2)))
            .mul_expr(SdfExpr::y().sub_expr(SdfExpr::constant(r(1))))
            .abs(),
    );
    let sqrt_expr = prepare(SdfExpr::x().add_expr(SdfExpr::constant(r(100))).sqrt());
    let min = p(-10, -10, -10);
    let max = p(10, 10, 10);

    c.bench_function("hypersdf sphere cell classification", |b| {
        b.iter(|| sphere.classify_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf aabb cell classification", |b| {
        b.iter(|| aabb.classify_cell(black_box(&min), black_box(&max)))
    });

    let mins = [p(-10, -10, -10), p(90, 90, 90), p(200, 200, 200)];
    let maxs = [p(10, 10, 10), p(110, 110, 110), p(210, 210, 210)];
    c.bench_function("hypersdf prepared cell batch classification", |b| {
        b.iter(|| sphere.classify_cells(black_box(mins.iter().zip(maxs.iter()))))
    });
    c.bench_function("hypersdf prepared cell batch report", |b| {
        b.iter(|| sphere.classify_cells_report(black_box(mins.iter().zip(maxs.iter()))))
    });
    c.bench_function("hypersdf conservative cell handoff", |b| {
        b.iter(|| sphere.classify_cells_for_handoff(black_box(mins.iter().zip(maxs.iter()))))
    });
    let voxel_grid = SdfVoxelCellGrid::new(p(-100, -100, -100), p(50, 50, 50), [4, 4, 4])
        .with_source(SdfVoxelGridSource::new("bench:sphere", 1));
    c.bench_function("hypersdf hypervoxel grid handoff", |b| {
        b.iter(|| sphere.classify_voxel_grid_for_handoff(black_box(voxel_grid.clone())))
    });
    let voxel_report = sphere
        .classify_voxel_grid_for_handoff(voxel_grid)
        .expect("positive benchmark grid");
    let voxel_interchange = voxel_report.interchange_manifest();
    c.bench_function("hypersdf hypervoxel interchange manifest", |b| {
        b.iter(|| voxel_report.interchange_report(black_box(&voxel_interchange)))
    });
    c.bench_function("hypersdf sphere cell interval", |b| {
        b.iter(|| sphere.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf slab cell interval", |b| {
        b.iter(|| slab.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf cylinder cell interval", |b| {
        b.iter(|| cylinder.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf capsule cell interval", |b| {
        b.iter(|| capsule.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf torus cell interval", |b| {
        b.iter(|| torus.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf aabb cell interval", |b| {
        b.iter(|| aabb.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf rounded aabb cell interval", |b| {
        b.iter(|| rounded_aabb.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf affine cell interval", |b| {
        b.iter(|| affine.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf linear vector cell interval", |b| {
        b.iter(|| linear.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf arithmetic cell interval", |b| {
        b.iter(|| arithmetic.interval_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf arithmetic cell lipschitz", |b| {
        b.iter(|| arithmetic.lipschitz_cell(black_box(&min), black_box(&max)))
    });
    c.bench_function("hypersdf sqrt cell interval", |b| {
        b.iter(|| sqrt_expr.interval_cell(black_box(&min), black_box(&max)))
    });
}

criterion_group!(
    benches,
    bench_point_classification,
    bench_cell_classification
);
criterion_main!(benches);
