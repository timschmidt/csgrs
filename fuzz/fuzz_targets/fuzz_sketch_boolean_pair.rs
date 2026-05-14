#![no_main]

use csgrs::csg::CSG;
use csgrs::float_types::{Real, tolerance};
use csgrs::sketch::Sketch;
use libfuzzer_sys::fuzz_target;

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let mut raw = [0u8; 8];
    for slot in &mut raw {
        *slot = bytes[*idx % bytes.len()];
        *idx += 1;
    }
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    value.clamp(-100.0, 100.0) as Real
}

fn assert_sketch_finite(sketch: &Sketch<()>) {
    for polygon in sketch.to_multipolygon().0 {
        for coord in &polygon.exterior().0 {
            assert!(coord.x.is_finite());
            assert!(coord.y.is_finite());
        }
        for ring in polygon.interiors() {
            for coord in &ring.0 {
                assert!(coord.x.is_finite());
                assert!(coord.y.is_finite());
            }
        }
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0usize;
    let a = Sketch::rectangle(
        decode_real(bytes, &mut idx).abs().max(tolerance()),
        decode_real(bytes, &mut idx).abs().max(tolerance()),
        None,
    );
    let b = Sketch::circle(
        decode_real(bytes, &mut idx).abs().max(tolerance()),
        (bytes[idx % bytes.len()] as usize % 32) + 3,
        None,
    )
    .translate(decode_real(bytes, &mut idx), decode_real(bytes, &mut idx), 0.0);
    let result = match bytes[idx % bytes.len()] % 4 {
        0 => a.union(&b),
        1 => a.difference(&b),
        2 => a.intersection(&b),
        _ => a.xor(&b),
    };
    assert_sketch_finite(&result);
});
