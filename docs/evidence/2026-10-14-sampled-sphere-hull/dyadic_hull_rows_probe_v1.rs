//! Exact dyadic control for the isolated hull selection repair.
use csgrs::{GeometryCertainty, GeometryContext, Real, solid};

fn main() {
    solid::cache::clear();
    let mesh = solid::cube(Real::from(8_u8));
    let cold = solid::convex_hull_with_context(&mesh, &GeometryContext::STRICT);
    let warm = solid::convex_hull_with_context(&mesh, &GeometryContext::STRICT);
    let replay = solid::convex_hull_with_context(&mesh, &GeometryContext::STRICT);
    for (phase, result) in [("cold", cold), ("warm", warm), ("replay", replay)] {
        let outcome = result.expect("dyadic cube hull");
        assert_eq!(outcome.certainty, GeometryCertainty::Certified);
        assert_eq!(outcome.value.positions.len(), 8);
        assert_eq!(outcome.value.triangles.len(), 12);
        assert!(outcome.value.is_closed_manifold());
        let order: Vec<_> = outcome
            .value
            .positions
            .iter()
            .map(|point| {
                mesh.positions
                    .iter()
                    .position(|source| source == point)
                    .unwrap()
            })
            .collect();
        println!("phase={phase} position_order={order:?}");
        println!("triangle_rows={:?}", outcome.value.triangles);
    }
    solid::cache::clear();
}
