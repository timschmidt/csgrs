//! Public admission metadata for rational coordinates with a deferred reduction.
use hyperreal::{ExactDyadicLine2, Rational};
use num::BigInt;

#[test]
fn canonical_parts_need_no_reduction_regardless_of_size_or_sign() {
    for value in [
        Rational::from(0),
        Rational::from(-1),
        Rational::fraction(17, 31).unwrap(),
        Rational::from_bigint(BigInt::from(1) << 1024),
    ] {
        assert_eq!(value.reduction_input_bits(), 0);
    }
}

#[test]
fn generated_coordinate_width_is_available_before_and_after_reduction() {
    for span in [6., 281_474_976_710_662.] {
        for x in [span, -span] {
            let line = ExactDyadicLine2::from_f64([0., 0.], [x, span]).unwrap();
            let (_, point) = line.intersection_point_f64([0., 3.], [x, 3.]).unwrap();
            let value = point[0].exact_rational_ref().unwrap();
            let alias = value.clone();
            let probe = || {
                let (_, numerator, denominator) = value.ratio_parts();
                let width = numerator.bits().max(denominator.bits());
                assert_eq!(width, if span == 6. { 7 } else { 98 });
                assert_eq!(value.reduction_input_bits(), width);
                assert_eq!(alias.reduction_input_bits(), width);
            };
            let reduce = || {
                assert_eq!(alias.numerator(), &3_u8.into());
                assert_eq!(alias.denominator(), &1_u8.into());
                assert_eq!(alias.is_negative(), x < 0.);
            };
            #[cfg(feature = "dispatch-trace")]
            {
                use hyperreal::dispatch_trace;
                dispatch_trace::reset();
                dispatch_trace::with_recording(probe);
                assert!(dispatch_trace::take_trace().dispatch.is_empty());
                dispatch_trace::with_recording(reduce);
                assert_eq!(
                    dispatch_trace::take_trace().path_count(
                        "rational",
                        "canonicalization",
                        "lazy-internal-coordinate"
                    ),
                    1
                );
                dispatch_trace::with_recording(probe);
                assert!(dispatch_trace::take_trace().dispatch.is_empty());
            }
            #[cfg(not(feature = "dispatch-trace"))]
            {
                probe();
                reduce();
                probe();
            }
        }
    }
}
