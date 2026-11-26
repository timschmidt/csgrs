//! Demonstrates native Hypermesh connectivity and Laplacian smoothing.
//! The sphere is a finite indexed mesh. Printed edge uses count directed vertex
//! adjacency entries, not triangle-edge multiplicity. The displacement assertion
//! concerns vertex zero after one iteration; it does not establish global shape
//! quality, volume preservation or a general smoothing convergence result.

use csgrs::solid;
use hyperlattice::Real;

/// Prints adjacency statistics and compares one-step smoothing at two positive weights.
/// Both runs start from the original mesh. Native failures, missing vertex zero
/// or a failed displacement-order assertion panic; no files are written.
fn main() {
    let sphere = solid::sphere(1.0, 16, 8).expect("valid primitive fixture");
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

/// Promotes finite binary64 exactly, so `0.1` here is not the rational one tenth.
/// Panics if supplied a nonfinite value.
fn r(value: f64) -> Real {
    Real::try_from(value).expect("example values must be finite")
}
