//! Original sphere replay with the default allocator and no query instrumentation.
use csgrs::{GeometryCertainty, GeometryContext, Real, solid};
use std::hint::black_box;

fn main() {
    let case = std::env::args().nth(1).expect("frozen case name");
    let (radius, segments, stacks) = match case.as_str() {
        "fifteen_degrees" => (8_u8, 24, 12),
        "original_medium" => (10, 32, 16),
        "fifth_turn" => (8, 10, 7),
        "seventh_turn" => (8, 7, 5),
        _ => panic!("unknown frozen case"),
    };
    solid::cache::clear();
    println!("replay_case={case} repetitions=3 grammar_policy=default allocator=default");
    let mesh = solid::sphere(Real::from(radius), segments, stacks);
    // All three outputs stay alive. No validation, serialization, timing,
    // allocation accounting, or other query occurs between these calls.
    let cold = solid::convex_hull_with_context(black_box(&mesh), &GeometryContext::STRICT);
    let warm = solid::convex_hull_with_context(black_box(&mesh), &GeometryContext::STRICT);
    let replay = solid::convex_hull_with_context(black_box(&mesh), &GeometryContext::STRICT);
    assert_eq!(mesh.positions.len(), segments * (stacks - 1) + 2);
    assert_eq!(mesh.triangles.len(), 2 * segments * (stacks - 1));
    assert!(mesh.is_closed_manifold());
    for (phase, result) in [("cold", &cold), ("warm", &warm), ("replay", &replay)] {
        println!("witness_phase={phase}");
        println!(
            "input case={case} radius={radius} segments={segments} stacks={stacks} positions={} triangles={}",
            mesh.positions.len(),
            mesh.triangles.len()
        );
        match result {
            Ok(outcome) => {
                assert_eq!(outcome.certainty, GeometryCertainty::Certified);
                let hull = &outcome.value;
                assert_eq!(hull.positions.len(), mesh.positions.len());
                assert_eq!(hull.triangles.len(), 2 * hull.positions.len() - 4);
                assert!(hull.is_closed_manifold());
                assert!(mesh.positions.iter().all(|p| hull.positions.contains(p)));
                assert!(hull.positions.iter().all(|p| mesh.positions.contains(p)));
                let order: Vec<_> = hull
                    .positions
                    .iter()
                    .map(|point| {
                        mesh.positions
                            .iter()
                            .position(|source| source == point)
                            .unwrap()
                    })
                    .collect();
                println!("position_order={order:?}");
                println!("triangle_rows={:?}", hull.triangles);
                println!(
                    "result case={case} certified=true original_positions_preserved=true positions={} triangles={}",
                    hull.positions.len(),
                    hull.triangles.len()
                );
            },
            Err(error) => println!("result case={case} certified=false error={error:?}"),
        }
    }
    drop((cold, warm, replay, mesh));
    solid::cache::clear();
}
