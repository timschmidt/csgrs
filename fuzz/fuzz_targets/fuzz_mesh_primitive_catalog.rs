//! Fuzz target for mesh primitive constructors.

#![no_main]

use csgrs::mesh::Mesh;
use hyperlattice::{Point3, Real, Vector3};
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
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

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let tag = bytes[idx] % 15;
    idx += 1;
    let a = decode_real(bytes, &mut idx);
    let b = decode_real(bytes, &mut idx);
    let c = decode_real(bytes, &mut idx);
    let segments = (bytes[idx % bytes.len()] as usize % 24) + 1;
    idx += 1;
    let teeth = (bytes[idx % bytes.len()] as usize % 24) + 1;

    let mesh = match tag {
        0 => Mesh::cuboid(a, b, c, ()),
        1 => Mesh::cube(a.abs().max(tolerance()), ()),
        2 => Mesh::sphere(a.abs().max(tolerance()), segments, segments, ()),
        3 => Mesh::cylinder(a, b, segments, ()),
        4 => Mesh::frustum(a, b, c, segments, ()),
        5 => Mesh::frustum_ptp(
            Point3::origin(),
            Point3::new(a.clone(), b.clone(), c),
            a,
            b,
            segments,
            (),
        ),
        6 => Mesh::ellipsoid(a, b, c, segments, segments, ()),
        7 => Mesh::arrow(
            Point3::origin(),
            Vector3::from_xyz(a, b, c),
            segments,
            false,
            (),
        ),
        8 => Mesh::octahedron(a, ()),
        9 => Mesh::icosahedron(a, ()),
        10 => Mesh::torus(a, b, segments, segments, ()),
        11 => Mesh::teardrop_cylinder(a, b, c, segments, ()),
        12 => Mesh::spur_gear_involute(
            a,
            teeth,
            b.clone(),
            c.clone(),
            real(0.01) * b,
            segments,
            c,
            (),
        ),
        13 => Mesh::spur_gear_cycloid(a, teeth, teeth.saturating_add(1), b, segments, c, ()),
        _ => Mesh::helical_involute_gear(
            a,
            teeth,
            b.clone(),
            c.clone(),
            real(0.01) * b.clone(),
            segments,
            c,
            b,
            segments,
            (),
        ),
    };

    assert_mesh_finite(&mesh);
});
