//! Fuzz target for mesh primitive constructors.

#![no_main]

mod support;

use csgrs::solid;
use hyperlattice::{Point3, Real, Vector3};
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    hyperlimit::real_max(&value, &tolerance)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order")
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
    let positive_b = at_least_tolerance(b.abs());

    let mesh = match tag {
        0 => solid::cuboid(a, b, c),
        1 => solid::cube(at_least_tolerance(a.abs())),
        2 => solid::sphere(at_least_tolerance(a.abs()), segments, segments),
        3 => solid::cylinder(a, b, segments),
        4 => solid::frustum(a, b, c, segments),
        5 => solid::frustum_between(
            Point3::origin(),
            Point3::new(a.clone(), b.clone(), c),
            a,
            b,
            segments,
        ),
        6 => solid::ellipsoid(a, b, c, segments, segments),
        7 => solid::arrow(Point3::origin(), Vector3::from_xyz(a, b, c), segments, false),
        8 => solid::octahedron(a),
        9 => solid::icosahedron(a),
        10 => solid::torus(a, b, segments, segments),
        11 => solid::teardrop_cylinder(a, b, c, segments),
        12 => solid::spur_gear_involute(
            a,
            teeth,
            b.clone(),
            c.clone(),
            real(0.01) * b,
            segments,
            c,
        ),
        13 => solid::spur_gear_cycloid(a, teeth, positive_b, b, segments, c),
        _ => solid::helical_involute_gear(
            a,
            teeth,
            b.clone(),
            c.clone(),
            real(0.01) * b.clone(),
            segments,
            c,
            b,
            segments,
        ),
    };

    support::validate_triangle_mesh(&mesh, true);
});
