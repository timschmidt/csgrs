#![no_main]

use csgrs::float_types::Real;
use csgrs::sketch::Sketch;
use libfuzzer_sys::fuzz_target;

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e9;
    value.clamp(-1.0e6, 1.0e6) as Real
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

    let sketch: Sketch<()> = Sketch::polygon(&points, None);
    for triangle in sketch.triangulate() {
        for point in triangle {
            assert!(point.x.is_finite());
            assert!(point.y.is_finite());
            assert!(point.z.is_finite());
        }
    }
});
