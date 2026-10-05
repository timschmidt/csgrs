#![no_main]

mod support;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameterInterval, BezierParameterPolynomial, Classification,
    CurveContext, Real,
};
use libfuzzer_sys::fuzz_target;

fn real_from_byte(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn unit_from_byte(byte: u8) -> Real {
    (Real::from((byte % 17) as i32) / Real::from(16_i32)).unwrap()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 6 {
        return;
    }

    let policy = CurveContext::STRICT;
    let polynomial = match support::under_classified_result(&policy, || {
        BezierParameterPolynomial::try_new_power_basis(vec![
            real_from_byte(data[0]),
            real_from_byte(data[1]),
            real_from_byte(data[2]),
        ])
    }) {
        Ok(Classification::Decided(polynomial)) => polynomial,
        Ok(Classification::Uncertain(_)) | Err(_) => return,
    };

    let mut start = unit_from_byte(data[3]);
    let mut end = unit_from_byte(data[4]);
    if data[5] & 1 == 1 {
        std::mem::swap(&mut start, &mut end);
    }

    let interval = match support::under_classified_result(&policy, || {
        BezierParameterInterval::try_new(start, end)
    }) {
        Ok(Classification::Decided(interval)) => interval,
        Ok(Classification::Uncertain(_)) | Err(_) => return,
    };

    let _ =
        support::under_classified_result(&policy, || polynomial.root_count_in_interval(&interval));
    let _ = support::under_classified_result(&policy, || {
        BezierAlgebraicParameter2::try_isolate(polynomial, interval)
    });
});
