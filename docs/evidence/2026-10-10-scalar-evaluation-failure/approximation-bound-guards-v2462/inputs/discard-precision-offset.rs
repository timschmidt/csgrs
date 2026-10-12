// The same associated function is included by the node facade and proof target.
#[cfg(not(verus_keep_ghost))]
macro_rules! proof {
    ($($tokens:tt)*) => {};
}

#[cfg(verus_keep_ghost)]
verus! {
pub(crate) open spec fn approximation_bound_expected(
    magnitude: nat, bits: u64, precision: i32, sign: Sign,
) -> BoundInfo {
    if magnitude <= 1 {
        BoundInfo::Unknown
    } else {
        let all_ones = magnitude + 1 == pow2(bits as nat);
        let power_of_two = magnitude == pow2((bits as nat - 1) as nat);
        let exponent = checked_metadata_exponent(
            precision as int + bits as int - if all_ones { 0int } else { 1int });
        BoundInfo::NonZero {
            sign: Some(sign), msd: exponent,
            exact_msd: exponent.is_some() && !power_of_two && !all_ones,
        }
    }
}
}

impl Computable {
    #[cfg_attr(verus_keep_ghost, allow(unused, verus_impl_method_marker))]
    #[cfg_attr(verus_keep_ghost, verus_spec(result =>
        requires appr.metadata_normalized(), appr.metadata_storage_bits() <= u64::MAX,
        ensures result == BoundInfo::Unknown <==> appr.metadata_magnitude() <= 1,
            forall|bits: u64| #[trigger] crate::magnitude_model::has_bit_length(
                appr.metadata_magnitude(), bits as nat)
                ==> result == approximation_bound_expected(
                    appr.metadata_magnitude(), bits, prec, appr.metadata_sign()),
            forall|value: real| #[trigger] approximates(
                value, appr.metadata_value(), binary_unit(prec as int))
                ==> bound_denotes(result, value),
    ))]
    fn bound_from_approx(prec: Precision, appr: &BigInt) -> BoundInfo {
        // Approximation values with magnitude <= 1 are within the allowed error
        // band, so they cannot certify sign or nonzero status.
        let magnitude = appr.magnitude();
        let magnitude_bits = magnitude.bits();
        proof! {
            lemma2_to64();
            assert(magnitude.metadata_value() == appr.metadata_magnitude());
            assert(magnitude_bits > 0 ==> crate::magnitude_model::has_bit_length(
                magnitude.metadata_value(), magnitude_bits as nat));
        }
        if magnitude_bits <= 1 {
            proof! {
                assert(magnitude.metadata_value() <= 1);
            }
            BoundInfo::Unknown
        } else {
            proof! {
                lemma_pow2_strictly_increases(0, (magnitude_bits - 1) as nat);
                assert(magnitude.metadata_value() > 1);
            }
            let trailing_ones = magnitude.trailing_ones();
            proof! {
                crate::bit_length_model::trailing_ones_determine_all_ones(
                    magnitude.metadata_value(), magnitude_bits as nat, trailing_ones as nat,
                    (magnitude.metadata_value() + 1) / pow2(trailing_ones as nat));
            }
            let reaches_next_binade = trailing_ones == magnitude_bits;
            let msd = crate::verified::word::checked_bit_length_msd(
                magnitude_bits,
                if reaches_next_binade { 0 } else { 1 },
                false,
                0,
            );
            // Keep this scan inside the successful-exponent branch. Proof
            // annotations add no runtime work to the lazy expression.
            let exact_msd = msd.is_some()
                && {
                    let trailing_zeros = magnitude.trailing_zeros();
                    proof! {
                        match trailing_zeros {
                            Some(zeros) => {
                                crate::bit_length_model::trailing_zeros_determine_power_of_two(
                                    magnitude.metadata_value(), magnitude_bits as nat, zeros as nat,
                                    magnitude.metadata_value() / pow2(zeros as nat));
                            },
                            None => { assert(false); },
                        }
                        assert((trailing_zeros == Some((magnitude_bits - 1) as u64))
                            == (magnitude.metadata_value() == pow2((magnitude_bits - 1) as nat)));
                    }
                    trailing_zeros != Some(magnitude_bits - 1)
                }
                && !reaches_next_binade;
            let sign = appr.sign();
            let result = BoundInfo::with_sign_msd(sign, msd, exact_msd);
            proof! {
                assert(exact_msd == (msd.is_some()
                    && magnitude.metadata_value() != pow2((magnitude_bits - 1) as nat)
                    && !reaches_next_binade));
                assert(result == approximation_bound_expected(
                    magnitude.metadata_value(), magnitude_bits, prec, sign));
                assert forall|bits: u64| #[trigger] crate::magnitude_model::has_bit_length(
                    appr.metadata_magnitude(), bits as nat)
                    implies result == approximation_bound_expected(
                        appr.metadata_magnitude(), bits, prec, appr.metadata_sign()) by {
                    if bits < magnitude_bits {
                        crate::bit_length_model::bit_lengths_determine_order(
                            magnitude.metadata_value(), magnitude.metadata_value(),
                            bits as nat, magnitude_bits as nat);
                    } else if bits > magnitude_bits {
                        crate::bit_length_model::bit_lengths_determine_order(
                            magnitude.metadata_value(), magnitude.metadata_value(),
                            magnitude_bits as nat, bits as nat);
                    }
                }
                assert forall|value: real| #[trigger] approximates(
                    value, appr.metadata_value(), binary_unit(prec as int))
                    implies bound_denotes(result, value) by {
                    approximation_metadata_produces_valid_bound(
                        value, appr.metadata_value(), magnitude.metadata_value(), magnitude_bits,
                        prec, reaches_next_binade,
                        magnitude.metadata_value() == pow2((magnitude_bits - 1) as nat),
                        sign, msd, exact_msd, result);
                }
            }
            result
        }
    }
}
