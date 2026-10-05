//! Part blueprint extraction benchmark.
//!
//! This uses a simple wall-clock loop instead of an external benchmark
//! dependency so the bench target remains available in minimal checkouts.

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

fn vector(x: i64, y: i64, z: i64) -> Vector3 {
    Vector3::new([Real::from(x), Real::from(y), Real::from(z)])
}

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

fn main() {
    let parts = (0..64)
        .map(|idx| {
            let geometry = solid::cube(Real::from(2)).translated(
                Real::from(idx % 8) * Real::from(3),
                Real::from(idx / 8),
                Real::from(idx),
            );
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
