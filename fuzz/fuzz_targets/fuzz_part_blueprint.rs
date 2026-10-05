//! Fuzz target for CSG part blueprint reports.

#![no_main]

use csgrs::solid::{self, SolidExt};
use csgrs::{
    AttributedMesh,
    parts::{
        AssemblyDocumentation, BlueprintEdgeStyle, BlueprintProjection, CsgPartInterface,
        InstallationVector, PartMetadata, PartSource, blueprint_from_aabb_parts,
    },
};
use hyperlattice::{Real, Vector3};
use libfuzzer_sys::fuzz_target;

fn real(value: i16) -> Real {
    Real::from(i64::from(value))
}

fn vector(x: i16, y: i16, z: i16) -> Vector3 {
    Vector3::new([real(x), real(y), real(z)])
}

fn metadata(handle: String, offset: Vector3, include_install: bool) -> PartMetadata {
    let mut interface = CsgPartInterface::exact_csg(
        "fuzz-family",
        handle.clone(),
        PartSource {
            family: "fuzz".into(),
            revision: "bytes".into(),
        },
    );
    interface.documentation = AssemblyDocumentation {
        installation: include_install.then(|| {
            InstallationVector::new(vector(0, 0, 1), offset, false, false)
                .expect("fixed fuzz install direction is nonzero")
        }),
        pose_hint: None,
        flags: Vec::new(),
    };
    PartMetadata::new(handle, interface)
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 6 {
        return;
    }

    let mut parts = Vec::new();
    for (idx, chunk) in bytes.chunks(6).take(12).enumerate() {
        let x = i16::from(chunk[0] as i8);
        let y = i16::from(chunk[1] as i8);
        let z = i16::from(chunk[2] as i8);
        let size = i16::from(chunk[3] % 16) + 1;
        let explode = vector(i16::from(chunk[4] as i8), 0, i16::from(chunk[5] as i8));
        let include_install = chunk[5] & 1 == 0;
        let mesh = AttributedMesh::from_uniform(
            solid::cube(real(size)).translated(real(x), real(y), real(z)),
            metadata(format!("p{idx}"), explode, include_install),
        );
        parts.push(mesh);
    }

    let projection = match bytes[0] % 3 {
        0 => BlueprintProjection::Front,
        1 => BlueprintProjection::Top,
        _ => BlueprintProjection::Right,
    };
    let report = blueprint_from_aabb_parts(&parts, projection, bytes[1] & 1 == 1);
    let minimum_edges = parts.len() * 4;
    assert!(report.assembled.edges.len() >= minimum_edges);
    assert!(report.exploded.edges.len() >= minimum_edges);
    for edge in report.assembled.edges.iter().chain(&report.exploded.edges) {
        assert!(!edge.part_handle.is_empty());
        assert!(!edge.evidence.notes.is_empty());
        assert!(matches!(
            edge.style,
            BlueprintEdgeStyle::Visible
                | BlueprintEdgeStyle::HiddenDashed
                | BlueprintEdgeStyle::Suppressed
                | BlueprintEdgeStyle::Unknown
                | BlueprintEdgeStyle::Guide
                | BlueprintEdgeStyle::Dimension
        ));
    }
});
