//! Fuzz target for sketch shape constructors.

#![no_main]

use csgrs::float_types::{Real, tolerance};
use csgrs::sketch::Profile;
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

fn assert_sketch_finite<M: Clone + Send + Sync + std::fmt::Debug>(sketch: &Profile<M>) {
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

    let sketch: Profile<Option<()>> = match tag {
        0 => Profile::rectangle(a, b, None),
        1 => Profile::square(a, None),
        2 => Profile::circle(a, segments, None),
        3 => Profile::right_triangle(a, b, None),
        4 => Profile::ellipse(a, b, segments, None),
        5 => Profile::regular_ngon(segments, a, None),
        6 => Profile::arrow(a, b, c, positive_b, None),
        7 => Profile::trapezoid(a, b, c, 0.25, None),
        8 => Profile::star(segments, a, b, None),
        9 => Profile::rounded_rectangle(a, b, c, segments, None),
        10 => Profile::squircle(a, b, segments, None),
        11 => Profile::keyhole(a, b, c, segments, None),
        12 => Profile::reuleaux(segments, a, segments, None),
        13 => Profile::ring(a, b, segments, None),
        14 => Profile::pie_slice(a, b, c, segments, None),
        15 => Profile::heart(a, b, segments, None),
        16 => Profile::crescent(positive_a, positive_b, c, segments, None),
        _ => Profile::airfoil_naca4(a, b, 12.0, positive_a, segments.max(2), None),
    };

    assert_sketch_finite(&sketch);
});
