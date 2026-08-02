//! Exact scene operations across pairs of Hyperreal representations.

#![no_main]

use arbitrary::Arbitrary;
use hypergraphics::{
    Color3, ExactCamera, ExactMesh, ExactVertex, Point2, Point3, PredicatePolicy, Primitive,
    ScreenPoint, TriangleOrientation, TriangulationContext, Viewport, axes_mesh, grid_mesh,
    polygon_surface_mesh,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

#[derive(Arbitrary, Debug)]
struct SceneInput {
    representation_x: u8,
    representation_y: u8,
    raw_color: [u32; 3],
    display_color: [u8; 3],
    approximate_policy: bool,
    triangle_index: u8,
    mutated_vertex: u8,
    grid_half_steps: u8,
    viewport_width: u16,
    viewport_height: u16,
    screen_x: i16,
    screen_y: i16,
}

fuzz_target!(|input: SceneInput| {
    let values = representative_values();
    let tx = &values[usize::from(input.representation_x) % values.len()];
    let ty = &values[usize::from(input.representation_y) % values.len()];

    let raw_color = input.raw_color.map(f32::from_bits);
    assert_eq!(
        Color3::new(raw_color[0], raw_color[1], raw_color[2]).is_ok(),
        raw_color.iter().all(|channel| channel.is_finite())
    );
    let color = Color3::new(
        f32::from(input.display_color[0]) / 255.0,
        f32::from(input.display_color[1]) / 255.0,
        f32::from(input.display_color[2]) / 255.0,
    )
    .expect("finite color");
    let policy = if input.approximate_policy {
        PredicatePolicy::APPROXIMATE_512
    } else {
        PredicatePolicy::STRICT
    };

    let a = Point3::new(tx.clone(), ty.clone(), Real::zero());
    let b = Point3::new(tx + Real::one(), ty.clone(), Real::zero());
    let c = Point3::new(tx.clone(), ty + Real::one(), Real::zero());
    let query = Point3::new(tx.clone(), ty.clone(), Real::one());
    let mut mesh = ExactMesh::new(
        Primitive::Triangles,
        vec![
            ExactVertex::new(a, color),
            ExactVertex::new(b, color),
            ExactVertex::new(c, color),
        ],
    );
    assert_eq!(mesh.triangle_count(), 1);
    let first = mesh
        .triangle_orientation_against(0, &query, policy)
        .expect("triangle index");
    assert_ne!(first, TriangleOrientation::Unknown);
    assert_eq!(
        mesh.triangle_orientation_against(0, &query, policy)
            .expect("cached triangle"),
        first
    );
    if input.triangle_index != 0 {
        assert!(
            mesh.triangle_orientation_against(usize::from(input.triangle_index), &query, policy,)
                .is_err()
        );
    }
    assert_eq!(mesh.to_render_vertices64().expect("finite values").len(), 3);
    assert_eq!(mesh.to_xyz_rgb_f64().expect("finite values").len(), 18);

    let mutated_vertex = usize::from(input.mutated_vertex) % 3;
    mesh.vertices_mut()[mutated_vertex].position.z = Real::one();
    assert_ne!(
        mesh.triangle_orientation_against(0, &query, PredicatePolicy::STRICT)
            .expect("cache invalidated"),
        TriangleOrientation::Unknown
    );

    assert_eq!(
        axes_mesh(tx.clone(), ty.clone())
            .expect("exact axis values")
            .vertex_count(),
        6
    );
    let half_steps = u32::from(input.grid_half_steps % 8);
    assert_eq!(
        grid_mesh(half_steps, tx.clone(), color)
            .expect("small grid")
            .vertex_count(),
        usize::try_from((u64::from(half_steps) * 2 + 1) * 4).unwrap()
    );

    let width = f64::from(input.viewport_width) + 1.0;
    let height = f64::from(input.viewport_height) + 1.0;
    let viewport = Viewport::new(0.0, 0.0, width, height).expect("positive viewport");
    let projection = ExactCamera::default()
        .projection64(viewport, policy)
        .expect("default projection");
    let screen = ScreenPoint::new(f64::from(input.screen_x), f64::from(input.screen_y))
        .expect("finite screen point");
    if let Some(point) = hypergraphics::camera::unproject_to_z0(&projection, viewport, screen)
        .expect("conditioned unprojection")
    {
        let coordinates = point.to_array();
        assert!(coordinates.iter().all(|coordinate| coordinate.is_finite()));
        assert_eq!(coordinates[2], 0.0);
    }

    let polygon = [
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(Real::from(2), Real::zero()),
        Point2::new(Real::from(2), Real::from(2)),
        Point2::new(Real::zero(), Real::from(2)),
    ];
    let context = TriangulationContext::new(policy);
    let triangulated = polygon_surface_mesh(&context, &polygon, &[], Real::zero(), color)
        .expect("valid rectangle")
        .into_value();
    assert_eq!(triangulated.triangle_count(), 2);
});

fn representative_values() -> Vec<Real> {
    let pi_squared = &Real::pi() * &Real::pi();
    let values = vec![
        Real::new(Rational::fraction(3, 2).expect("valid rational")),
        Real::pi(),
        Real::e(),
        Real::new(Rational::new(2)).sqrt().expect("positive"),
        Real::new(Rational::new(3)).ln().expect("positive"),
        Real::new(Rational::fraction(1, 5).expect("valid rational")).sin_pi(),
        pi_squared * Real::e(),
        Real::new(Rational::one()).sin(),
    ];
    assert_eq!(
        values
            .iter()
            .map(|value| value.detailed_facts().symbolic.kind)
            .collect::<Vec<_>>(),
        vec![
            StructuralKind::ExactRational,
            StructuralKind::PiLike,
            StructuralKind::ExpLike,
            StructuralKind::SqrtLike,
            StructuralKind::LogLike,
            StructuralKind::TrigExact,
            StructuralKind::ProductConstant,
            StructuralKind::ComputableOpaque,
        ]
    );
    values
}
