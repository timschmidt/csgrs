    fn bound_from_approx(prec: Precision, appr: &BigInt) -> BoundInfo {
        // Approximation values with magnitude <= 1 are within the allowed error
        // band, so they cannot certify sign or nonzero status.
        let magnitude = appr.magnitude();
        let magnitude_bits = magnitude.bits();
        if magnitude_bits <= 1 {
            BoundInfo::Unknown
        } else {
            let reaches_next_binade = magnitude.trailing_ones() == magnitude_bits;
            let msd = crate::verified::word::checked_bit_length_msd(
                magnitude_bits,
                if reaches_next_binade { 0 } else { 1 },
                false,
                prec,
            );
            // The closed one-unit interval [|a|-1, |a|+1] has one binary
            // exponent unless a is a power of two or one below the next power.
            // Inspect trailing bits instead of allocating endpoint integers.
            let exact_msd = msd.is_some()
                && magnitude.trailing_zeros() != Some(magnitude_bits - 1)
                && !reaches_next_binade;
            BoundInfo::with_sign_msd(appr.sign(), msd, exact_msd)
        }
    }

