//! Fuzz exact Real arithmetic, fused kernels, facts, and serialization.

#![no_main]

use arbitrary::Arbitrary;
use hyperreal::{Rational, Real, RealSign, ZeroKnowledge};
use libfuzzer_sys::fuzz_target;

#[derive(Clone, Copy, Debug, Arbitrary)]
struct RawRational {
    numerator: i16,
    denominator: u8,
}

impl RawRational {
    fn rational(self) -> Rational {
        Rational::fraction(i64::from(self.numerator), u64::from(self.denominator) + 1)
            .expect("the generated denominator is positive")
    }

    fn real(self) -> Real {
        Real::new(self.rational())
    }
}

#[derive(Debug, Arbitrary)]
struct Input {
    values: [RawRational; 8],
}

fuzz_target!(|input: Input| {
    let values = input.values.map(RawRational::real);
    let a = &values[0];
    let b = &values[1];
    let c = &values[2];

    assert_eq!(
        a + b,
        Real::new(&input.values[0].rational() + &input.values[1].rational())
    );
    assert_eq!(
        a - b,
        Real::new(&input.values[0].rational() - &input.values[1].rational())
    );
    assert_eq!(
        a * b,
        Real::new(&input.values[0].rational() * &input.values[1].rational())
    );
    assert_eq!(a * &(b + c), (a * b) + (a * c));

    if !b.definitely_zero() {
        assert_eq!((a / b).expect("nonzero exact division") * b, a.clone());
    }

    let dot2 = Real::dot2_refs([a, b], [c, &values[3]]);
    assert_eq!(dot2, (a * c) + (b * &values[3]));
    let dot3 = Real::dot3_refs([a, b, c], [&values[3], &values[4], &values[5]]);
    assert_eq!(dot3, (a * &values[3]) + (b * &values[4]) + (c * &values[5]));

    let fused = Real::signed_product_sum([true, false], [[a, b], [c, &values[3]]]);
    assert_eq!(fused, (a * b) - (c * &values[3]));

    let (complex_re, complex_im) =
        Real::exact_rational_complex_product_known_exact([a, b], [c, &values[3]]);
    assert_eq!(complex_re, (a * c) - (b * &values[3]));
    assert_eq!(complex_im, (a * &values[3]) + (b * c));

    if !c.definitely_zero() || !values[3].definitely_zero() {
        let denominator = (c * c) + (&values[3] * &values[3]);
        let (quotient_re, quotient_im) =
            Real::exact_rational_complex_quotient_known_exact([a, b], [c, &values[3]])
                .expect("nonzero exact complex denominator");
        assert_eq!(
            quotient_re,
            (((a * c) + (b * &values[3])) / &denominator).expect("nonzero exact real quotient")
        );
        assert_eq!(
            quotient_im,
            (((b * c) - (a * &values[3])) / denominator).expect("nonzero exact imaginary quotient")
        );
    }

    let determinant =
        Real::certified_affine_det2_sign([a, b], [c, &values[3]], [&values[4], &values[5]]);
    if let Some(prepared) = Real::prepare_affine_det2_filter([a, b], [c, &values[3]]) {
        assert_eq!(prepared.sign([&values[4], &values[5]]), determinant);
    }

    let points = [
        [&values[0], &values[1], &values[2]],
        [&values[3], &values[4], &values[5]],
        [&values[6], &values[7], &values[0]],
    ];
    let rationals = input.values.map(RawRational::rational);
    let ab: [Rational; 3] =
        core::array::from_fn(|axis| rationals[axis + 3].clone() - &rationals[axis]);
    let ac = [
        &rationals[6] - &rationals[0],
        &rationals[7] - &rationals[1],
        &rationals[0] - &rationals[2],
    ];
    let cross = [
        Rational::signed_product_sum2([true, false], [[&ab[1], &ac[2]], [&ab[2], &ac[1]]]),
        Rational::signed_product_sum2([true, false], [[&ab[2], &ac[0]], [&ab[0], &ac[2]]]),
        Rational::signed_product_sum2([true, false], [[&ab[0], &ac[1]], [&ab[1], &ac[0]]]),
    ];
    let squares = cross.each_ref().map(|value| value * value);
    let mut axis = 0;
    for candidate in 1..3 {
        if squares[candidate] > squares[axis] {
            axis = candidate;
        }
    }
    let expected = if cross[axis].is_positive() {
        Some((axis, RealSign::Positive))
    } else if cross[axis].is_negative() {
        Some((axis, RealSign::Negative))
    } else {
        None
    };
    assert_eq!(
        Real::exact_rational_dominant_affine_cross_axis(points[0], points[1], points[2]),
        expected
    );

    for value in &values {
        let f64_first = value.clone();
        let expected_f32 = value.to_f32_lossy().map(f32::to_bits);
        assert_eq!(value.to_f32_lossy().map(f32::to_bits), expected_f32);
        let f64_value = f64_first.to_f64_lossy();
        assert_eq!(f64_first.to_f32_lossy().map(f32::to_bits), expected_f32);
        assert!(f64_value.is_none_or(f64::is_finite));
        assert!(expected_f32.is_none_or(|bits| f32::from_bits(bits).is_finite()));

        let facts = value.structural_facts();
        assert!(facts.exact_rational);
        assert_eq!(
            value.zero_status(),
            if value
                .exact_rational_ref()
                .expect("exact rational")
                .is_zero()
            {
                ZeroKnowledge::Zero
            } else {
                ZeroKnowledge::NonZero
            }
        );
        assert_eq!(
            Real::from_json(&value.to_json()).expect("JSON roundtrip"),
            *value
        );
        assert_eq!(
            Real::from_bytes(&value.to_bytes()).expect("CBOR roundtrip"),
            *value
        );
        let _ = value.certified_sign_until(-64);
        let _ = value.certified_cmp_until(a, -64);
        let _ = value.certified_dyadic_interval(-64);
        let _ = value.to_f64_exact_dyadic();
    }
});
