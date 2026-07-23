//! Fuzz target for metaball sampling boundaries and generated mesh finiteness.

#![no_main]

use csgrs::mesh::{
    Mesh,
    metaballs::{MetaBall, MetaballDiagnostics},
};
use hyperlattice::{Point3, Real};
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-100.0, 100.0))
}

fn assert_mesh_finite(mesh: &Mesh<()>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.0[0].is_finite());
        assert!(vertex.normal.0[1].is_finite());
        assert!(vertex.normal.0[2].is_finite());
    }
}

fn assert_diagnostics_consistent(diagnostics: &MetaballDiagnostics) {
    assert_eq!(
        diagnostics.sample_count,
        diagnostics.finite_sample_count + diagnostics.non_finite_sample_count
    );
    assert_eq!(
        diagnostics.sample_count,
        diagnostics.negative_sample_count
            + diagnostics.zero_sample_count
            + diagnostics.positive_sample_count
    );
    assert_eq!(diagnostics.surface_nets_index_count % 3, 0);
    assert_eq!(
        diagnostics.surface_nets_index_count / 3,
        diagnostics.emitted_triangle_count + diagnostics.skipped_non_finite_triangle_count
    );
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let ball_count = (bytes[idx % bytes.len()] as usize % 5) + 1;
    idx += 1;
    let resolution = ((bytes[idx % bytes.len()] as usize % 8) + 2).min(10);
    idx += 1;

    let mut balls = Vec::with_capacity(ball_count);
    for _ in 0..ball_count {
        let center = Point3::new(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        );
        let radius = decode_real(bytes, &mut idx);
        balls.push(MetaBall::new(center, radius));
    }
    if bytes[idx % bytes.len()] & 1 == 1 {
        balls.push(MetaBall::new(
            Point3::new(Real::zero(), Real::zero(), Real::zero()),
            real(1.0e-25),
        ));
    }

    let iso_value = decode_real(bytes, &mut idx);
    let padding = decode_real(bytes, &mut idx);
    let (mesh, diagnostics) = Mesh::<()>::metaballs_with_diagnostics(
        &balls,
        (resolution, resolution, resolution),
        iso_value,
        padding,
        (),
    );

    assert_mesh_finite(&mesh);
    assert_diagnostics_consistent(&diagnostics);
    assert_eq!(diagnostics.emitted_triangle_count, mesh.triangles().len());
});
