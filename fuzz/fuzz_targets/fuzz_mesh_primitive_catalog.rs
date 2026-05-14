#![no_main]

use csgrs::float_types::{Real, tolerance};
use csgrs::mesh::Mesh;
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
    let tag = bytes[idx] % 12;
    idx += 1;
    let a = decode_real(bytes, &mut idx);
    let b = decode_real(bytes, &mut idx);
    let c = decode_real(bytes, &mut idx);
    let segments = (bytes[idx % bytes.len()] as usize % 24) + 1;

    let mesh = match tag {
        0 => Mesh::cuboid(a, b, c, None),
        1 => Mesh::cube(a.abs().max(tolerance()), None),
        2 => Mesh::sphere(a.abs().max(tolerance()), segments, segments, None),
        3 => Mesh::cylinder(a, b, segments, None),
        4 => Mesh::frustum(a, b, c, segments, None),
        5 => Mesh::frustum_ptp(Point3::origin(), Point3::new(a, b, c), a, b, segments, None),
        6 => Mesh::ellipsoid(a, b, c, segments, segments, None),
        7 => Mesh::arrow(Point3::origin(), Vector3::new(a, b, c), segments, false, None),
        8 => Mesh::octahedron(a, None),
        9 => Mesh::icosahedron(a, None),
        10 => Mesh::torus(a, b, segments, segments, None),
        _ => Mesh::teardrop_cylinder(a, b, c, segments, None),
    };

    assert_mesh_finite(&mesh);
});
