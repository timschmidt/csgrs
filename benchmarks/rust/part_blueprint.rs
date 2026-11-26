//! Part blueprint extraction benchmark.
//!
//! This uses a simple wall-clock loop instead of an external benchmark
//! dependency so the bench target remains available in minimal checkouts.
//! Requires `attributed`. Constructs 64 translated cubes before timing, then
//! performs 512 front-view bounds-report extractions with hidden edges retained.
//! The printed total counts assembled and exploded edges; it is not a geometry
//! checksum or correctness oracle. No warmup, per-sample distribution, blocker/
//! certainty assertion or shared-harness environment controls are used here.

use std::time::Instant;

use csgrs::{
    AttributedMesh,
    parts::{
        AssemblyDocumentation, BlueprintProjection, CsgPartInterface, InstallationVector,
        PartMetadata, PartSource, blueprint_from_aabb_parts,
    },
    solid::{self, SolidExt},
};
use hyperlattice::{Real, Vector3};

/// Constructs an exact integer offset for benchmark installation metadata.
fn vector(x: i64, y: i64, z: i64) -> Vector3 {
    Vector3::new([Real::from(x), Real::from(y), Real::from(z)])
}

/// Authors an exact-CSG label and fixed +Z installation with the supplied explode offset.
/// The label is metadata, not a geometric validation of the associated cube.
fn metadata(handle: &str, offset: Vector3) -> PartMetadata {
    let mut interface = CsgPartInterface::exact_csg(
        "bench-family",
        handle,
        PartSource {
            family: "bench".into(),
            revision: "local".into(),
        },
    );
    interface.documentation = AssemblyDocumentation {
        installation: Some(
            InstallationVector::new(vector(0, 0, 1), offset, false, false)
                .expect("bench install direction is nonzero"),
        ),
        pose_hint: None,
        flags: Vec::new(),
    };
    PartMetadata::new(handle, interface)
}

/// Times report construction/destruction and edge-count accumulation after fixture setup.
/// Prints one millisecond duration for all 512 calls; does not isolate cold/warm
/// behavior or verify visibility against an independent expectation.
fn main() {
    let parts = (0..64)
        .map(|idx| {
            let geometry = solid::cube(Real::from(2))
                .expect("valid primitive fixture")
                .translated(
                    Real::from(idx % 8) * Real::from(3),
                    Real::from(idx / 8),
                    Real::from(idx),
                )
                .expect("valid transform fixture");
            AttributedMesh::from_uniform(
                geometry,
                metadata(&format!("p{idx}"), vector(i64::from(idx), 0, 8)),
            )
        })
        .collect::<Vec<_>>();

    let start = Instant::now();
    let mut edge_count = 0usize;
    for _ in 0..512 {
        let report = blueprint_from_aabb_parts(&parts, BlueprintProjection::Front, false);
        edge_count += report.assembled.edges.len() + report.exploded.edges.len();
    }
    println!(
        "part_blueprint_aabb edges={} elapsed_ms={}",
        edge_count,
        start.elapsed().as_millis()
    );
}
