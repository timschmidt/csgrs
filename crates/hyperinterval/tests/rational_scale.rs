#![cfg(feature = "rational")]

use hyperinterval::RationalInterval;
use hyperreal::Rational;

#[test]
fn scalar_images_match_integer_endpoint_products_and_compose() {
    for lower in -4..=4 {
        for upper in lower..=4 {
            let source = RationalInterval::new(
                Rational::fraction(lower, 3).unwrap(),
                Rational::fraction(upper, 3).unwrap(),
            )
            .unwrap();
            for numerator in -5..=5 {
                let factor = Rational::fraction(numerator, 7).unwrap();
                let image = source.scale(&factor);
                let a = lower * numerator;
                let b = upper * numerator;
                let expected = RationalInterval::new(
                    Rational::fraction(a.min(b), 21).unwrap(),
                    Rational::fraction(a.max(b), 21).unwrap(),
                )
                .unwrap();
                assert_eq!(image, expected);
                if numerator != 0 {
                    let inverse =
                        Rational::fraction(7 * numerator.signum(), numerator.unsigned_abs())
                            .unwrap();
                    assert_eq!(image.scale(&inverse), source);
                }
            }
        }
    }
    for factor in [-7, 0, 5] {
        assert!(
            RationalInterval::empty()
                .scale(&Rational::new(factor))
                .is_empty()
        );
    }
}

#[test]
fn large_factor_preserves_exact_endpoints_through_replay_and_reuse() {
    // The factor exceeds both native word widths. Endpoint cancellation
    // leaves exact nonintegral results, including on a 32-bit target.
    let large: Rational = "340282366920938463463374607431768211457".parse().unwrap();
    let lower = Rational::new(-3) / &large;
    let upper = Rational::new(11) / &large;
    let source = RationalInterval::new(lower, upper).unwrap();
    let image = source.scale(&(-&large / Rational::new(7)));
    let expected = RationalInterval::new(
        Rational::fraction(-11, 7).unwrap(),
        Rational::fraction(3, 7).unwrap(),
    )
    .unwrap();
    assert_eq!(image, expected);
    #[cfg(feature = "serde")]
    let image: RationalInterval =
        serde_json::from_slice(&serde_json::to_vec(&image).unwrap()).unwrap();
    assert_eq!(
        image.scale(&Rational::new(-7)),
        RationalInterval::new(Rational::new(-3), Rational::new(11)).unwrap()
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn scaling_a_crossing_interval_uses_two_endpoint_products() {
    use hyperreal::dispatch_trace;

    let source = RationalInterval::new(
        Rational::fraction(-3, 7).unwrap(),
        Rational::fraction(11, 5).unwrap(),
    )
    .unwrap();
    let factor = Rational::fraction(17, 13).unwrap();
    for _ in 0..2 {
        dispatch_trace::reset();
        let image = dispatch_trace::with_recording(|| source.scale(&factor));
        assert_eq!(
            dispatch_trace::take_trace().operation_count("rational", "mul"),
            2
        );
        assert_eq!(
            image.into_bounds(),
            Some((
                Rational::fraction(-51, 91).unwrap(),
                Rational::fraction(187, 65).unwrap(),
            ))
        );
    }
}
