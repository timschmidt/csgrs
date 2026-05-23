//! Fuzz target for sketch extrusion, revolve, and sweep operations.

#![no_main]

use csgrs::csg::CSG;
use csgrs::mesh::Mesh;
use csgrs::sketch::Profile;
use hyperlattice::{Point3, Real, Vector3};
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn clamp_real(value: Real, min: f64, max: f64) -> Real {
    value.max(real(min)).min(real(max))
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-1.0e3, 1.0e3))
}

fn assert_mesh_finite<M: Clone + Send + Sync + std::fmt::Debug>(mesh: &Mesh<M>) {
    for vertex in mesh.vertices() {
        assert!(vertex.position.x.is_finite());
        assert!(vertex.position.y.is_finite());
        assert!(vertex.position.z.is_finite());
        assert!(vertex.normal.0[0].is_finite());
        assert!(vertex.normal.0[1].is_finite());
        assert!(vertex.normal.0[2].is_finite());
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }
    let mut idx = 0usize;
    let width = decode_real(bytes, &mut idx).abs().max(tolerance());
    let height = decode_real(bytes, &mut idx).abs().max(tolerance());
    let sketch: Profile<Option<()>> = Profile::rectangle(width.clone(), height, None);
    let tag = bytes[idx % bytes.len()] % 4;
    idx += 1;
    let mesh = match tag {
        0 => sketch.extrude(decode_real(bytes, &mut idx)),
        1 => sketch.extrude_vector(Vector3::from_xyz(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        )),
        2 => {
            let angle = clamp_real(decode_real(bytes, &mut idx), -720.0, 720.0);
            let segments = (bytes[idx % bytes.len()] as usize % 16) + 2;
            match sketch
                .translate(width, Real::zero(), Real::zero())
                .revolve(angle, segments)
            {
                Ok(mesh) => mesh,
                Err(_) => Mesh::empty(None),
            }
        },
        _ => {
            let mut path = Vec::new();
            let count = (bytes[idx % bytes.len()] as usize % 8) + 1;
            idx += 1;
            for _ in 0..count {
                path.push(Point3::new(
                    decode_real(bytes, &mut idx),
                    decode_real(bytes, &mut idx),
                    decode_real(bytes, &mut idx),
                ));
            }
            sketch.sweep(&path)
        },
    };

    assert_mesh_finite(&mesh);
});
