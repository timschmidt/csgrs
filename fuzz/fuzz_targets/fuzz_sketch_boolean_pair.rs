//! Fuzz target for paired sketch boolean operations.

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

fn assert_sketch_finite<M: Clone + Send + Sync + std::fmt::Debug>(sketch: &Sketch<M>) {
    let profiles = sketch.region_profiles();
    for ring in profiles.iter().flat_map(|profile| {
        std::iter::once(profile.material().points())
            .chain(profile.holes().iter().map(|hole| hole.points()))
    }) {
        for point in ring {
            assert!(point[0].is_finite());
            assert!(point[1].is_finite());
        }
    }
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0usize;
    let a: Sketch<Option<()>> = Sketch::rectangle(
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
