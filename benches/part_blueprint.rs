//! Compile-time benchmark target for part blueprint extraction.
//!
//! This uses a simple wall-clock loop instead of an external benchmark
//! dependency so the bench target remains available in minimal checkouts.

use std::time::Instant;

use csgrs::{
    csg::CSG,
    mesh::Mesh,
    parts::{
        AssemblyDocumentation, BlueprintProjection, CsgPartInterface, ExactVector3,
        InstallationVector, PartMetadata, PartSource, blueprint_from_aabb_parts,
    },
};
use hyperlattice::Real;

fn vec3(x: i64, y: i64, z: i64) -> ExactVector3 {
    ExactVector3::from_i64(x, y, z)
}

fn metadata(handle: &str, offset: ExactVector3) -> PartMetadata {
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
            InstallationVector::new(vec3(0, 0, 1), offset, false, false)
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
            Mesh::cube(
                Real::from(2),
                metadata(&format!("p{idx}"), vec3(i64::from(idx), 0, 8)),
            )
            .translate(
                Real::from(idx % 8) * Real::from(3),
                Real::from(idx / 8),
                Real::from(idx),
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
