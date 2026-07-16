use criterion::{Criterion, black_box, criterion_group, criterion_main};
use hypergraphics::{
    Color3, ExactCamera, ExactMesh, ExactVertex, Point3, Primitive, Real, ScreenPoint, Viewport,
    grid_mesh,
};

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn bench_geometry(c: &mut Criterion) {
    let color = Color3::new(0.2, 0.4, 0.8).expect("finite color");
    let vertices = (0..600)
        .map(|index| ExactVertex::new(point(index, index % 17, index % 31), color))
        .collect();
    let mesh = ExactMesh::new(Primitive::Triangles, vertices);

    c.bench_function("hypergraphics export 600 render vertices64", |b| {
        b.iter(|| mesh.to_render_vertices64())
    });
    c.bench_function("hypergraphics export 600 flat xyz rgb f64", |b| {
        b.iter(|| mesh.to_xyz_rgb_f64())
    });
    c.bench_function("hypergraphics exact grid 201 by 201 lines", |b| {
        b.iter(|| grid_mesh(black_box(100), black_box(Real::from(5)), black_box(color)))
    });

    let orientation_mesh = ExactMesh::new(
        Primitive::Triangles,
        vec![
            ExactVertex::new(point(0, 0, 0), color),
            ExactVertex::new(point(1_000_000, 0, 1), color),
            ExactVertex::new(point(0, 1_000_000, 1), color),
        ],
    );
    let query = point(1, 1, 1);
    let prepared = orientation_mesh
        .prepare_triangle_orientation(0)
        .expect("triangle orientation package");
    c.bench_function("hypergraphics triangle orientation direct", |b| {
        b.iter(|| orientation_mesh.triangle_orientation_against(0, black_box(&query)))
    });
    c.bench_function("hypergraphics triangle orientation prepared plane", |b| {
        b.iter(|| prepared.classify_point(black_box(&query)))
    });
}

fn bench_camera(c: &mut Criterion) {
    let camera = ExactCamera::default();
    let viewport = Viewport::new(0.0, 0.0, 1920.0, 1080.0).expect("finite viewport");
    let projection = camera.projection64(viewport).expect("camera projection");
    let screen = ScreenPoint { x: 960.0, y: 540.0 };

    c.bench_function("hypergraphics repeated unproject z0", |b| {
        b.iter(|| {
            hypergraphics::camera::unproject_to_z0(
                black_box(&projection),
                black_box(viewport),
                black_box(screen),
            )
        })
    });
}

criterion_group!(benches, bench_geometry, bench_camera);
criterion_main!(benches);
