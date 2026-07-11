//! Fuzz target for sketch shape constructors.

#![no_main]

use csgrs::sketch::Profile;
use hyperlattice::Real;
use libfuzzer_sys::fuzz_target;

fn real(value: f64) -> Real {
    Real::try_from(value).expect("fuzz decoder clamps to finite values")
}

fn tolerance() -> Real {
    real(1.0e-9)
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    value.max(&tolerance).clone()
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

fn assert_sketch_finite(sketch: &Profile) {
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
    let tag = bytes[idx] % 22;
    idx += 1;
    let a = decode_real(bytes, &mut idx);
    let b = decode_real(bytes, &mut idx);
    let c = decode_real(bytes, &mut idx);
    let segments = (bytes[idx % bytes.len()] as usize % 32) + 1;
    idx += 1;
    let teeth = (bytes[idx % bytes.len()] as usize % 24) + 1;
    let positive_a = at_least_tolerance(a.abs());
    let positive_b = at_least_tolerance(b.abs());

    let sketch: Profile = match tag {
        0 => Profile::rectangle(a, b ),
        1 => Profile::square(a ),
        2 => Profile::circle(a, segments ),
        3 => Profile::right_triangle(a, b ),
        4 => Profile::ellipse(a, b, segments ),
        5 => Profile::regular_ngon(segments, a ),
        6 => Profile::arrow(a, b, c, positive_b ),
        7 => Profile::trapezoid(a, b, c, real(0.25) ),
        8 => Profile::star(segments, a, b ),
        9 => Profile::rounded_rectangle(a, b, c, segments ),
        10 => Profile::squircle(a, b, segments ),
        11 => Profile::keyhole(a, b, c, segments ),
        12 => Profile::reuleaux(segments, a, segments ),
        13 => Profile::ring(a, b, segments ),
        14 => Profile::pie_slice(a, b, c, segments ),
        15 => Profile::heart(a, b, segments ),
        16 => Profile::crescent(positive_a, positive_b, c, segments ),
        17 => Profile::airfoil_naca4(a, b, real(12.0), positive_a, segments.max(2) ),
        18 => Profile::involute_gear(a, teeth, b.clone(), c, real(0.01) * b, segments ),
        19 => Profile::cycloidal_gear(a, teeth, teeth.saturating_add(1), b, segments ),
        20 => Profile::involute_rack(a, teeth, b, c.clone(), real(0.01) * c ),
        _ => Profile::cycloidal_rack(a, teeth, positive_b, c, segments ),
    };

    assert_sketch_finite(&sketch);
});
