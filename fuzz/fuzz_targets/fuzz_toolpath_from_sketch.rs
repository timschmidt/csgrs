//! Fuzz target for toolpath generation from sketches.

#![no_main]

use csgrs::float_types::Real;
use csgrs::sketch::Sketch;
use csgrs::toolpath::gcode::Post;
use csgrs::toolpath::{cut2d_contours, Feeds, KerfSide, LeadInOut, MachineKind};
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

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 16 {
        return;
    }

    let mut idx = 0usize;
    let width = decode_real(bytes, &mut idx).abs().max(0.001);
    let height = decode_real(bytes, &mut idx).abs().max(0.001);
    let z = decode_real(bytes, &mut idx).clamp(-100.0, 100.0);
    let kerf = decode_real(bytes, &mut idx).clamp(-10.0, 10.0);

    let sketch: Sketch<()> = Sketch::rectangle(width, height, None);
    let feeds = Feeds {
        travel: 3000.0,
        xy: 1200.0,
        plunge: 300.0,
        rpm: Some(12000.0),
        power: Some(0.5),
        pierce_ms: Some(1),
    };
    let path = cut2d_contours(
        &sketch,
        z,
        kerf,
        KerfSide::Outside,
        Some(LeadInOut {
            length: 0.5,
            radius: 0.0,
        }),
        &feeds,
        MachineKind::Laser,
    );
    let gcode = Post {
        absolute_e: false,
        units_mm: true,
        z_safe: 5.0,
    }
    .write(&path);
    assert!(!gcode.contains("NaN"));
    assert!(!gcode.contains("inf"));
});
