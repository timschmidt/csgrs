//! Fuzz target for curve polygon triangulation.

#![no_main]

use hyperlattice::Real;
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
    let value = i64::from_le_bytes(raw) as f64 / 1.0e9;
    real(value.clamp(-1.0e6, 1.0e6))
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 2 {
        return;
    }
    let mut idx = 1usize;
    let point_count = (bytes[0] as usize % 64) + 1;
    let mut points = Vec::with_capacity(point_count);
    for _ in 0..point_count {
        let x = decode_real(bytes, &mut idx);
        let y = decode_real(bytes, &mut idx);
        points.push([x, y]);
    }

    let region = csgrs::curve::polygon(&points);
    let Ok(mesh) =
        csgrs::curve::try_triangulate(&region, &csgrs::GeometryContext::STRICT)
    else {
        return;
    };
    let mesh = mesh.into_value();
    for point in mesh.positions.iter() {
        assert!(point.x.is_finite());
        assert!(point.y.is_finite());
        assert!(point.z.is_finite());
    }
});
