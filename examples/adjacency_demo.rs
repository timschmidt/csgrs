//! Demonstrates native Hypermesh connectivity and Laplacian smoothing.

use csgrs::solid;
use hyperlattice::Real;

fn main() {
    let sphere = solid::sphere(r(1.0), 16, 8);
    let adjacency = sphere.adjacency().expect("sphere connectivity");

    let edge_uses = adjacency.iter().map(Vec::len).sum::<usize>();
    let min_valence = adjacency.iter().map(Vec::len).min().unwrap_or(0);
    let max_valence = adjacency.iter().map(Vec::len).max().unwrap_or(0);
    println!(
        "vertices={} triangles={} edge_uses={} valence={}..={}",
        sphere.positions.len(),
        sphere.triangles.len(),
        edge_uses,
        min_valence,
        max_valence,
    );

    let original = sphere.positions[0].clone();
    let weak = sphere
        .laplacian_smooth(&r(0.1), 1)
        .expect("valid sphere smoothing");
    let strong = sphere
        .laplacian_smooth(&r(0.3), 1)
        .expect("valid sphere smoothing");
    let weak_change = (&original - &weak.positions[0]).norm();
    let strong_change = (&original - &strong.positions[0]).norm();

    assert!(strong_change > weak_change);
    println!("weak_change={weak_change} strong_change={strong_change}");
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}
