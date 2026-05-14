//! Fuzz target for sketch extrusion, revolve, and sweep operations.

#![no_main]

use csgrs::csg::CSG;
use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::Mesh;
use csgrs::sketch::Sketch;
use libfuzzer_sys::fuzz_target;
use nalgebra::{Point3, Vector3};

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-1.0e3, 1.0e3) as Real
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
    let width = decode_real(bytes, &mut idx).abs().max(tolerance());
    let height = decode_real(bytes, &mut idx).abs().max(tolerance());
    let sketch = Sketch::rectangle(width, height, None);
    let tag = bytes[idx % bytes.len()] % 4;
    idx += 1;
    let mesh = match tag {
        0 => sketch.extrude(decode_real(bytes, &mut idx)),
        1 => sketch.extrude_vector(Vector3::new(
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        )),
        2 => {
            let angle = decode_real(bytes, &mut idx).clamp(-720.0, 720.0);
            let segments = (bytes[idx % bytes.len()] as usize % 16) + 2;
            match sketch.translate(width, 0.0, 0.0).revolve(angle, segments) {
                Ok(mesh) => mesh,
                Err(_) => Mesh::new(),
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
