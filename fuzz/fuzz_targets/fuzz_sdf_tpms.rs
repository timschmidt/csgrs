//! Fuzz target for SDF and TPMS mesh generation.

#![no_main]

use csgrs::mesh::Mesh;
use csgrs::float_types::Real;
use libfuzzer_sys::fuzz_target;
use nalgebra::Point3;

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-100.0, 100.0) as Real
}

fn assert_mesh_finite(mesh: &Mesh<()>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.x.is_finite());
        assert!(vertex.normal.y.is_finite());
        assert!(vertex.normal.z.is_finite());
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let tag = bytes[idx] % 4;
    idx += 1;
    let res = (bytes[idx % bytes.len()] as usize % 10) + 1;
    idx += 1;
    let period = decode_real(bytes, &mut idx).abs().max(0.001);
    let iso = decode_real(bytes, &mut idx);
    let base = Mesh::cube(2.0, None);

    let mesh = match tag {
        0 => Mesh::sdf(
            |p: &Point3<Real>| {
                if p.x > 0.9 {
                    Real::NAN
                } else {
                    p.coords.norm() - 0.5
                }
            },
            (res, res, res),
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, 1.0),
            iso,
            None,
        ),
        1 => base.gyroid(res, period, iso, None),
        2 => base.schwarz_p(res, period, iso, None),
        _ => base.schwarz_d(res, period, iso, None),
    };

    assert_mesh_finite(&mesh);
});
