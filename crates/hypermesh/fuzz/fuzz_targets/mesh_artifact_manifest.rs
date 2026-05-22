#![no_main]

use hypermesh::exact::{
    ExactMesh, MeshArtifactBlocker, MeshArtifactFaceRecord, MeshArtifactManifest,
    MeshArtifactRole, MeshArtifactSourceKind, MeshArtifactVertexRecord, MeshCoordinateEvidence,
    MeshNumericAdapterContract, MeshTopologyEvidence, ValidationPolicy,
};
use libfuzzer_sys::fuzz_target;

fn value(data: &[u8], index: usize) -> i64 {
    i64::from(i16::from_le_bytes([data[index], data[index + 1]]))
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let mode = data[0] % 6;
    if mode == 0 {
        let scale = i64::from(data[1] % 12) + 1;
        let pos = [
            0,
            0,
            0,
            scale,
            0,
            0,
            0,
            scale,
            0,
            0,
            0,
            scale,
        ];
        let idx = [0, 2, 1, 0, 1, 3, 1, 2, 3, 2, 0, 3];
        let mesh = ExactMesh::from_i64_triangles(&pos, &idx).unwrap();
        let report = MeshArtifactManifest::from_exact_mesh(&mesh)
            .unwrap()
            .report();
        assert!(report.validation_handoff_ready);
        assert!(report.blockers.is_empty());
        return;
    }

    let vertex_count = usize::from(data[1] % 16);
    let face_count = usize::from(data[2] % 16);
    let source_version = u64::from(data[3] % 8);
    let mut vertices = (0..vertex_count)
        .map(|index| MeshArtifactVertexRecord {
            index: if mode == 1 && index == 0 {
                index.saturating_add(1)
            } else {
                index
            },
            coordinate_evidence: if mode == 2 {
                MeshCoordinateEvidence::LossyPrimitiveFloat
            } else if mode == 5 {
                MeshCoordinateEvidence::CertifiedDerivedExact
            } else {
                MeshCoordinateEvidence::CertifiedDerivedExact
            },
        })
        .collect::<Vec<_>>();
    if mode == 3 && !vertices.is_empty() {
        vertices.pop();
    }

    let mut faces = Vec::new();
    for face in 0..face_count {
        let base = 8 + face * 2;
        let a = if base + 1 < data.len() {
            usize::from(data[base] % 20)
        } else {
            0
        };
        let b = if base + 1 < data.len() {
            usize::from(data[base + 1] % 20)
        } else {
            0
        };
        let c = if vertex_count > 0 {
            (a + b + face) % vertex_count.max(1)
        } else {
            0
        };
        let vertices_for_face = if mode == 4 && face == 0 {
            vec![a, b]
        } else {
            vec![a, b, c]
        };
        faces.push(MeshArtifactFaceRecord {
            index: face,
            vertices: vertices_for_face,
            topology_evidence: if mode == 2 {
                MeshTopologyEvidence::PreviewOnly
            } else {
                MeshTopologyEvidence::DerivedExactSurfaceHandoff
            },
        });
    }

    let mut manifest = MeshArtifactManifest::new(
        if mode == 5 {
            MeshArtifactSourceKind::SdfSurfaceNetsPreview
        } else {
            MeshArtifactSourceKind::BrepTessellation
        },
        format!("fuzz artifact {}", value(data, 4)),
        source_version,
        if mode == 2 {
            MeshArtifactRole::Preview
        } else {
            MeshArtifactRole::DerivedHandoff
        },
        if mode == 2 {
            MeshNumericAdapterContract::preview(MeshCoordinateEvidence::LossyPrimitiveFloat)
        } else {
            MeshNumericAdapterContract::exact(MeshCoordinateEvidence::CertifiedDerivedExact)
        },
        vertices,
        faces,
    );
    if mode == 3 {
        manifest.declared_vertex_count = vertex_count;
        manifest.expected_source_version = Some(source_version.saturating_add(1));
    }
    let report = manifest.report();
    assert_eq!(report.validation_handoff_ready, report.blockers.is_empty());
    if mode == 5 {
        assert!(!report.validation_handoff_ready);
        assert!(
            report
                .blockers
                .contains(&MeshArtifactBlocker::PreviewOrExportSource)
        );
    }
    if report.validation_handoff_ready {
        assert!(report.coordinates_exact_replay_ready);
        assert!(report.topology_validation_replay_ready);
        assert!(report.source_current);
        assert!(!report.preview_only);
    }

    let _ = ExactMesh::from_i64_triangles_with_policy(&[], &[], ValidationPolicy::ALLOW_BOUNDARY);
});
