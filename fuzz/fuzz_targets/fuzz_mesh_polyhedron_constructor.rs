#![no_main]

use csgrs::mesh::Mesh;
use libfuzzer_sys::fuzz_target;

fn decode_real(bytes: &[u8], idx: &mut usize) -> csgrs::float_types::Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-1.0e3, 1.0e3) as csgrs::float_types::Real
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }
    let mut idx = 0usize;
    let point_count = (bytes[idx % bytes.len()] as usize % 16).min(bytes.len());
    idx += 1;
    let mut points = Vec::with_capacity(point_count);
    for _ in 0..point_count {
        points.push([
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
            decode_real(bytes, &mut idx),
        ]);
    }

    let mut owned_faces: Vec<Vec<usize>> = Vec::new();
    let face_count = bytes[idx % bytes.len()] as usize % 16;
    idx += 1;
    for _ in 0..face_count {
        let len = bytes[idx % bytes.len()] as usize % 8;
        idx += 1;
        let mut face = Vec::with_capacity(len);
        for _ in 0..len {
            face.push(bytes[idx % bytes.len()] as usize % (point_count + 4));
            idx += 1;
        }
        owned_faces.push(face);
    }
    let faces = owned_faces.iter().map(Vec::as_slice).collect::<Vec<_>>();

    if let Ok(mesh) = Mesh::<()>::polyhedron(&points, &faces, None) {
        for vertex in mesh.vertices() {
            assert!(vertex.position.x.is_finite());
            assert!(vertex.position.y.is_finite());
            assert!(vertex.position.z.is_finite());
            assert!(vertex.normal.x.is_finite());
            assert!(vertex.normal.y.is_finite());
            assert!(vertex.normal.z.is_finite());
        }
    }
});
