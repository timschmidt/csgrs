//! Fuzz target for paired mesh boolean operations.

#![no_main]

mod support;

use csgrs::solid::{self, SolidExt};
use hyperlattice::Real;
use libfuzzer_sys::fuzz_target;

fn tolerance() -> Real {
    (Real::one() / Real::from(1_000_000_u64)).expect("one million is nonzero")
}

fn at_least_tolerance(value: Real) -> Real {
    let tolerance = tolerance();
    hyperlimit::real_max(&value, &tolerance)
        .value()
        .cloned()
        .expect("decoded rationals have decidable order")
}

fn decode_real(bytes: &[u8], idx: &mut usize) -> Real {
    let numerator_byte =
        bytes[*idx % bytes.len()].wrapping_add(idx.to_le_bytes()[0].wrapping_mul(37));
    *idx += 1;
    let denominator_byte =
        bytes[*idx % bytes.len()].wrapping_add(idx.to_le_bytes()[0].wrapping_mul(53));
    *idx += 1;
    let numerator = i16::from(numerator_byte % 33) - 16;
    let denominator = u16::from(denominator_byte % 8) + 1;
    (Real::from(numerator) / Real::from(denominator)).expect("denominator is positive")
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0usize;
    let size_a = at_least_tolerance(decode_real(bytes, &mut idx).abs());
    let size_b = at_least_tolerance(decode_real(bytes, &mut idx).abs());
    let segments = 4;
    idx += 1;
    let a = match bytes[idx % bytes.len()] % 3 {
        0 => solid::cube(size_a.clone()),
        1 => solid::octahedron(size_a.clone()),
        _ => solid::cylinder(size_a.clone(), size_b.clone(), segments),
    };
    idx += 1;
    let b = match bytes[idx % bytes.len()] % 3 {
        0 => solid::cube(size_b.clone()),
        1 => solid::octahedron(size_b.clone()),
        _ => csgrs::curve::extrude(
            &csgrs::curve::rectangle(
                size_b.clone(),
                (size_b.clone() / Real::from(2_u8)).expect("two is nonzero"),
            ),
            size_a,
        ),
    }
    .translated(
        decode_real(bytes, &mut idx),
        decode_real(bytes, &mut idx),
        decode_real(bytes, &mut idx),
    );
    support::validate_triangle_mesh(&a, true);
    support::validate_triangle_mesh(&b, true);
    let result = match bytes[idx % bytes.len()] % 4 {
        0 => a.try_union(&b),
        1 => a.try_difference(&b),
        2 => a.try_intersection(&b),
        _ => a.try_xor(&b),
    };
    if let Ok(result) = result {
        support::validate_triangle_mesh(&result, false);
    }
});
