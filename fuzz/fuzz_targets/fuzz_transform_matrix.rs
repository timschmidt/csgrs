//! Fuzz target for transform matrix inputs.

#![no_main]

use hyperlattice::{Matrix4, Real};
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
    let value = i64::from_le_bytes(raw) as f64 / 1.0e12;
    real(value.clamp(-1.0e6, 1.0e6))
}

fuzz_target!(|bytes: &[u8]| {
    if bytes.is_empty() {
        return;
    }
    let mut idx = 0;
    let mut values: [Real; 16] = std::array::from_fn(|_| Real::zero());
    for value in &mut values {
        *value = decode_real(bytes, &mut idx);
    }
    let Some(matrix) = Matrix4::from_row_slice(&values) else {
        return;
    };
    let mesh = csgrs::solid::cube(Real::one());
    let transformed = csgrs::solid::transform(&mesh, &matrix);

    for position in transformed.positions.iter() {
        assert!(position.x.is_finite());
        assert!(position.y.is_finite());
        assert!(position.z.is_finite());
    }
});
