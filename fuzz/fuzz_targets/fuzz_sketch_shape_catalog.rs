//! Fuzz target for sketch shape constructors.

#![no_main]

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
    value.clamp(-1.0e3, 1.0e3) as Real
}

fn assert_sketch_finite<M: Clone + Send + Sync + std::fmt::Debug>(sketch: &Sketch<M>) {
    let rings = sketch.region_rings();
    for ring in rings.iter_all() {
        for point in ring {
            assert!(point[0].is_finite());
            assert!(point[1].is_finite());
        }
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }

    let mut idx = 0usize;
    let tag = bytes[idx] % 18;
    idx += 1;
    let a = decode_real(bytes, &mut idx);
    let b = decode_real(bytes, &mut idx);
    let c = decode_real(bytes, &mut idx);
    let segments = (bytes[idx % bytes.len()] as usize % 32) + 1;
    let positive_a = a.abs().max(tolerance());
    let positive_b = b.abs().max(tolerance());

    let sketch: Sketch<Option<()>> = match tag {
        0 => Sketch::rectangle(a, b, None),
        1 => Sketch::square(a, None),
        2 => Sketch::circle(a, segments, None),
        3 => Sketch::right_triangle(a, b, None),
        4 => Sketch::ellipse(a, b, segments, None),
        5 => Sketch::regular_ngon(segments, a, None),
        6 => Sketch::arrow(a, b, c, positive_b, None),
        7 => Sketch::trapezoid(a, b, c, 0.25, None),
        8 => Sketch::star(segments, a, b, None),
        9 => Sketch::rounded_rectangle(a, b, c, segments, None),
        10 => Sketch::squircle(a, b, segments, None),
        11 => Sketch::keyhole(a, b, c, segments, None),
        12 => Sketch::reuleaux(segments, a, segments, None),
        13 => Sketch::ring(a, b, segments, None),
        14 => Sketch::pie_slice(a, b, c, segments, None),
        15 => Sketch::heart(a, b, segments, None),
        16 => Sketch::crescent(positive_a, positive_b, c, segments, None),
        _ => Sketch::airfoil_naca4(a, b, 12.0, positive_a, segments.max(2), None),
    };

    assert_sketch_finite(&sketch);
});
