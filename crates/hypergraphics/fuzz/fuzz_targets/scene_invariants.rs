//! Exact scene operations over every pair of Hyperreal representations.

#![no_main]

use hypergraphics::{
    Color3, ExactMesh, ExactVertex, Point3, Primitive, TriangleOrientation, axes_mesh, grid_mesh,
};
use hyperreal::{Rational, Real, StructuralKind};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let values = representative_values();
    let color = Color3::new(
        f32::from(data.first().copied().unwrap_or(0)) / 255.0,
        0.5,
        1.0,
    )
    .expect("finite color");

    for tx in &values {
        for ty in &values {
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
                .triangle_orientation_against(0, &query)
                .expect("triangle index");
            assert_ne!(first, TriangleOrientation::Unknown);
            assert_eq!(
                mesh.triangle_orientation_against(0, &query)
                    .expect("cached triangle"),
                first
            );
            assert_eq!(mesh.to_render_vertices64().expect("finite values").len(), 3);
            assert_eq!(mesh.to_xyz_rgb_f64().expect("finite values").len(), 18);

            mesh.vertices_mut()[0].position.z = Real::zero();
            assert_ne!(
                mesh.triangle_orientation_against(0, &query)
                    .expect("cache invalidated"),
                TriangleOrientation::Unknown
            );

            assert_eq!(
                axes_mesh(tx.clone(), ty.clone())
                    .expect("positive finite exact values")
                    .vertex_count(),
                6
            );
            assert_eq!(grid_mesh(1, tx.clone(), color).vertex_count(), 12);
        }
    }
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
