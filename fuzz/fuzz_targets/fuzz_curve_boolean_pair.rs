//! Fuzz target for paired curve boolean operations.

#![no_main]

use csgrs::curve::{self, CurveRegionExt};
use hypercurve::CurveRegion2;
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
    real(value.clamp(-100.0, 100.0))
}

fn assert_curve_finite(curve: &CurveRegion2) {
    let profiles = curve::finite_profiles(curve);
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
    let a = curve::rectangle(
        at_least_tolerance(decode_real(bytes, &mut idx).abs()),
        at_least_tolerance(decode_real(bytes, &mut idx).abs()),
    );
    let b = curve::circle(
        at_least_tolerance(decode_real(bytes, &mut idx).abs()),
        (bytes[idx % bytes.len()] as usize % 32) + 3,
    )
    .transformed_affine(
        &Real::one(),
        &Real::zero(),
        &Real::zero(),
        &Real::one(),
        &decode_real(bytes, &mut idx),
        &decode_real(bytes, &mut idx),
    )
    .unwrap_or_else(|_| curve::empty());
    let result = match bytes[idx % bytes.len()] % 4 {
        0 => a.try_union(&b),
        1 => a.try_difference(&b),
        2 => a.try_intersection(&b),
        _ => a.try_xor(&b),
    };
    if let Ok(result) = result {
        assert_curve_finite(&result);
    }
});
