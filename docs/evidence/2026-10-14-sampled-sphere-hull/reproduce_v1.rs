//! Original exact sampled coordinates, with no rationalization or display round trip.
use csgrs::{GeometryCertainty, GeometryContext, Real, solid};

fn main() {
    let case = std::env::args().nth(1).expect("a frozen case name");
    let (radius, segments, stacks) = match case.as_str() {
        "fifteen_degrees" => (8_u8, 24, 12),
        "original_medium" => (10, 32, 16),
        "fifth_turn" => (8, 10, 7),
        "seventh_turn" => (8, 7, 5),
        _ => panic!("unknown frozen case"),
    };
    solid::cache::clear();
    let mesh = solid::sphere(Real::from(radius), segments, stacks);
    assert_eq!(mesh.positions.len(), segments * (stacks - 1) + 2);
    assert_eq!(mesh.triangles.len(), 2 * segments * (stacks - 1));
    assert!(mesh.is_closed_manifold());
    println!(
        "input case={case} radius={radius} segments={segments} stacks={stacks} positions={} triangles={}",
        mesh.positions.len(),
        mesh.triangles.len()
    );
    match solid::convex_hull_with_context(&mesh, &GeometryContext::STRICT) {
        Ok(outcome) => {
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            let hull = outcome.value;
            assert_eq!(hull.positions.len(), mesh.positions.len());
            assert_eq!(hull.triangles.len(), 2 * hull.positions.len() - 4);
            assert!(hull.is_closed_manifold());
            assert!(mesh.positions.iter().all(|p| hull.positions.contains(p)));
            assert!(hull.positions.iter().all(|p| mesh.positions.contains(p)));
            println!(
                "result case={case} certified=true original_positions_preserved=true positions={} triangles={}",
                hull.positions.len(),
                hull.triangles.len()
            );
        },
        Err(error) => println!("result case={case} certified=false error={error:?}"),
    }
}
