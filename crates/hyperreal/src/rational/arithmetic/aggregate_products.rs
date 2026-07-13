impl Rational {
    fn gcd_word(mut left: u128, mut right: u128) -> u128 {
        while right != 0 {
            (left, right) = (right, left % right);
        }
        left
    }

    fn from_word_magnitude_difference(
        positive: u128,
        negative: u128,
        denominator: u128,
    ) -> Self {
        let (sign, magnitude) = match positive.cmp(&negative) {
            Ordering::Greater => (Plus, positive - negative),
            Ordering::Less => (Minus, negative - positive),
            Ordering::Equal => {
                crate::trace_dispatch!("rational", "word-result", "zero");
                return Self::zero();
            }
        };
        let divisor = Self::gcd_word(magnitude, denominator);
        let magnitude = magnitude / divisor;
        let denominator = denominator / divisor;
        if magnitude == denominator {
            crate::trace_dispatch!("rational", "word-result", "unit");
            return if sign == Minus {
                Self::minus_one()
            } else {
                Self::one()
            };
        }
        if denominator == 1
            && let Some(value) = Self::small_integer(sign, magnitude)
        {
            crate::trace_dispatch!("rational", "word-result", "cached-small-integer");
            return value;
        }
        if denominator == 1 {
            #[cfg(feature = "dispatch-trace")]
            {
                let path = match magnitude {
                    0..=127 => "uncached-integer-65-127",
                    128..=255 => "uncached-integer-128-255",
                    256..=1023 => "uncached-integer-256-1023",
                    1024..=4095 => "uncached-integer-1024-4095",
                    _ => "uncached-integer-wide",
                };
                crate::trace_dispatch!("rational", "word-result", path);
            }
        } else if denominator.is_power_of_two() {
            crate::trace_dispatch!("rational", "word-result", "dyadic-fraction");
        } else if magnitude <= u128::from(u64::MAX) && denominator <= u128::from(u64::MAX) {
            crate::trace_dispatch!("rational", "word-result", "small-general-fraction");
        } else {
            crate::trace_dispatch!("rational", "word-result", "wide-general-fraction");
        }
        Self::from_parts_raw(
            sign,
            BigUint::from(magnitude),
            BigUint::from(denominator),
        )
    }

    fn product_term_words<const FACTORS: usize>(
        term: [&Self; FACTORS],
    ) -> Option<(u128, u128)> {
        let mut magnitude = 1_u128;
        let mut denominator = 1_u128;
        for factor in term {
            let numerator = factor.numerator.to_u128()?;
            let factor_denominator = factor.denominator.to_u128()?;
            let Some(next_magnitude) = magnitude.checked_mul(numerator) else {
                return Self::product_term_words_cross_cancelled(term);
            };
            let Some(next_denominator) = denominator.checked_mul(factor_denominator) else {
                return Self::product_term_words_cross_cancelled(term);
            };
            magnitude = next_magnitude;
            denominator = next_denominator;
        }
        Some((magnitude, denominator))
    }

    fn product_term_words_cross_cancelled<const FACTORS: usize>(
        term: [&Self; FACTORS],
    ) -> Option<(u128, u128)> {
        let mut numerators = [0_u128; FACTORS];
        let mut denominators = [1_u128; FACTORS];
        for i in 0..FACTORS {
            numerators[i] = term[i].numerator.to_u128()?;
            denominators[i] = term[i].denominator.to_u128()?;
        }

        for numerator in &mut numerators {
            if *numerator == 1 {
                continue;
            }
            for denominator in &mut denominators {
                if *denominator == 1 {
                    continue;
                }
                let divisor = Self::gcd_word(*numerator, *denominator);
                *numerator /= divisor;
                *denominator /= divisor;
                if *numerator == 1 {
                    break;
                }
            }
        }

        let mut magnitude = 1_u128;
        let mut denominator = 1_u128;
        for factor in numerators {
            magnitude = magnitude.checked_mul(factor)?;
        }
        for factor in denominators {
            denominator = denominator.checked_mul(factor)?;
        }
        Some((magnitude, denominator))
    }

    fn signed_product_sum_words<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<Self> {
        let (positive, negative, common_denominator) =
            Self::signed_product_sum_word_totals(terms, signs)?;
        Some(Self::from_word_magnitude_difference(
            positive,
            negative,
            common_denominator,
        ))
    }

    fn signed_product_sum_word_totals<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<(u128, u128, u128)> {
        let mut magnitudes = [0_u128; TERMS];
        let mut denominators = [1_u128; TERMS];
        let mut common_denominator = 1_u128;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let (magnitude, denominator) = Self::product_term_words(terms[i])?;
            magnitudes[i] = magnitude;
            denominators[i] = denominator;
            let divisor = Self::gcd_word(common_denominator, denominator);
            common_denominator =
                common_denominator.checked_mul(denominator / divisor)?;
        }

        let mut positive = 0_u128;
        let mut negative = 0_u128;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = magnitudes[i]
                .checked_mul(common_denominator / denominators[i])?;
            match sign {
                Plus => positive = positive.checked_add(magnitude)?,
                Minus => negative = negative.checked_add(magnitude)?,
                NoSign => {}
            }
        }
        Some((positive, negative, common_denominator))
    }

    fn dot_products_dyadic<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<Self> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; N];
        let mut any_nonzero = false;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let shift =
                left[i].dyadic_denominator_shift()? + right[i].dyadic_denominator_shift()?;
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
            any_nonzero = true;
        }
        if !any_nonzero {
            return Some(Self::zero());
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic dot-product scale should fit in usize");
            let mut magnitude = &left[i].numerator * &right[i].numerator;
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        let denominator =
            BigUint::one() << usize::try_from(max_shift).expect("shift should fit in usize");
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    fn dot_products_equal_denominator<const N: usize>(
        left: [&Self; N],
        right: [&Self; N],
        signs: [Sign; N],
    ) -> Option<Self> {
        let mut shared_denominator = None::<BigUint>;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            match &shared_denominator {
                None => shared_denominator = Some(denominator),
                Some(shared) if *shared == denominator => {}
                Some(_) => return None,
            }
        }

        let Some(denominator) = shared_denominator else {
            return Some(Self::zero());
        };

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = &left[i].numerator * &right[i].numerator;
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    fn product_term_denominator<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut denominator = BigUint::one();
        for factor in term {
            denominator *= &factor.denominator;
        }
        denominator
    }

    fn product_term_magnitude<const FACTORS: usize>(term: [&Self; FACTORS]) -> BigUint {
        let mut magnitude = BigUint::one();
        for factor in term {
            magnitude *= &factor.numerator;
        }
        magnitude
    }

    fn product_term_sign<const FACTORS: usize>(positive: bool, term: [&Self; FACTORS]) -> Sign {
        let mut sign = if positive { Plus } else { Minus };
        for factor in term {
            sign = sign * factor.sign;
        }
        sign
    }

    fn signed_product_sum_dyadic<const TERMS: usize, const FACTORS: usize>(
        terms: [[&Self; FACTORS]; TERMS],
        signs: [Sign; TERMS],
    ) -> Option<Self> {
        let mut max_shift = 0_u64;
        let mut denominator_shifts = [0_u64; TERMS];
        let mut any_nonzero = false;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let mut shift = 0_u64;
            for factor in terms[i] {
                shift += factor.dyadic_denominator_shift()?;
            }
            denominator_shifts[i] = shift;
            max_shift = max_shift.max(shift);
            any_nonzero = true;
        }
        if !any_nonzero {
            return Some(Self::zero());
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let scale_shift = usize::try_from(max_shift - denominator_shifts[i])
                .expect("dyadic product-sum scale should fit in usize");
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            if scale_shift != 0 {
                magnitude <<= scale_shift;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        let denominator =
            BigUint::one() << usize::try_from(max_shift).expect("shift should fit in usize");
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Evaluate a signed product sum when every live factor shares one
    /// reduced denominator.
    ///
    /// The caller is expected to have carried an object-level common-scale
    /// certificate, but this method still validates the denominator fact before
    /// using it. Returning `None` means the certificate was too weak for this
    /// particular product shape, so callers should fall back to
    /// [`Self::signed_product_sum`]. Keeping the validation in `Rational`
    /// preserves scalar storage ownership while giving geometric kernels the
    /// denominator-specialized schedule requested by Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997). The
    /// final delayed reduction is the same Bareiss-style fraction-delay idea
    /// used by the generic reducer; see Bareiss, "Sylvester's Identity and
    /// Multistep Integer-Preserving Gaussian Elimination," *Mathematics of
    /// Computation* 22.103 (1968).
    pub fn signed_product_sum_shared_denominator<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Option<Self> {
        debug_assert!(FACTORS > 0);
        let mut signs = [NoSign; TERMS];
        let mut nonzero_count = 0_usize;
        let mut shared_denominator = None::<&BigUint>;
        for i in 0..TERMS {
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign == NoSign {
                signs[i] = sign;
                continue;
            }
            nonzero_count += 1;
            signs[i] = sign;
            for factor in terms[i] {
                match shared_denominator {
                    None => shared_denominator = Some(&factor.denominator),
                    Some(shared) if shared == &factor.denominator => {}
                    Some(_) => return None,
                }
            }
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!(
                "rational",
                "product_sum",
                "shared-factor-denominator-all-zero"
            );
            return Some(Self::zero());
        }
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "product_sum", "word-sized-shared-scale");
            return Some(word);
        }
        let exponent = u32::try_from(FACTORS).ok()?;
        let denominator = shared_denominator
            .expect("nonzero product sum has a live factor denominator")
            .pow(exponent);
        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let magnitude = Self::product_term_magnitude(terms[i]);
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }
        crate::trace_dispatch!("rational", "product_sum", "shared-factor-denominator");
        Some(Self::from_signed_magnitude_difference(
            positive,
            negative,
            denominator,
        ))
    }

    /// Evaluate a fixed-size signed sum of products exactly.
    ///
    /// Each row in `terms` is multiplied, then added or subtracted according to
    /// the matching entry in `positive_terms`. The implementation delays
    /// rational reduction until the final sum and has structural fast paths for
    /// dyadic and equal-denominator products. This is the scalar reducer that
    /// geometry crates use for small determinant schedules while preserving
    /// their abstraction boundary: they pass a known polynomial shape, but do
    /// not inspect `Rational` storage internals.
    ///
    /// The fraction-delay strategy follows Bareiss, "Sylvester's Identity and
    /// Multistep Integer-Preserving Gaussian Elimination," *Mathematics of
    /// Computation* 22.103 (1968), and matches Yap's exact-geometric-
    /// computation guidance to keep algebraic object shape visible before
    /// scalar expansion; see Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7.1-2 (1997).
    pub fn signed_product_sum<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Self {
        // Short determinant and cofactor polynomials are exact rational sums of
        // products. As with `dot_products`, build one denominator and reduce
        // only the final row. This targets the trace rows where fixed 3x3/4x4
        // inverse, division, and negative-powi kernels still paid repeated gcd
        // work after dot products had already been fused. The algebraic
        // strategy follows the same fraction-delay idea as Bareiss fraction
        // free elimination (Bareiss, Math. Comp. 22(103), 1968,
        // https://www.ams.org/mcom/1968-22-103/S0025-5718-1968-0226829-0/S0025-5718-1968-0226829-0.pdf)
        // and common-factor exact matrix work
        // (https://link.springer.com/article/10.1007/s11786-020-00495-9),
        // but keeps the public fixed-size cofactor formulas division-free.
        // Structural note: keep this hook scalar-local. Hyperlattice can use it
        // for exact cofactor and determinant kernels, while predicate and
        // triangulation crates should consume only the resulting exact signs or
        // values through their own abstraction boundaries.
        debug_assert!(FACTORS > 0);
        let mut signs = [NoSign; TERMS];
        let mut nonzero_count = 0_usize;
        for i in 0..TERMS {
            // Term signs are pure structural facts from stored rational signs.
            // Compute them once and reuse them across dyadic, equal-denominator,
            // and LCM reducers. This preserves Bareiss-style delayed reduction
            // while removing repeated sign walks from exact cofactor/determinant
            // product sums.
            let sign = Self::product_term_sign(positive_terms[i], terms[i]);
            if sign != NoSign {
                nonzero_count += 1;
            }
            signs[i] = sign;
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!("rational", "product_sum", "all-zero");
            return Self::zero();
        }
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "product_sum", "word-sized");
            return word;
        }
        if nonzero_count == 1 {
            for i in 0..TERMS {
                match signs[i] {
                    Plus => {
                        crate::trace_dispatch!("rational", "product_sum", "single-term-product");
                        let denominator = Self::product_term_denominator(terms[i]);
                        return Self::from_signed_magnitude_difference(
                            Self::product_term_magnitude(terms[i]),
                            BigUint::ZERO,
                            denominator,
                        );
                    }
                    Minus => {
                        crate::trace_dispatch!("rational", "product_sum", "single-term-product");
                        let denominator = Self::product_term_denominator(terms[i]);
                        return Self::from_signed_magnitude_difference(
                            BigUint::ZERO,
                            Self::product_term_magnitude(terms[i]),
                            denominator,
                        );
                    }
                    NoSign => {}
                }
            }
        }

        if let Some(dyadic) = Self::signed_product_sum_dyadic(terms, signs) {
            // Structural-dispatch note: callers that know coordinates are
            // lifted from a common binary grid could pass that grid exponent
            // with the terms and jump directly to this reducer, avoiding the
            // exploratory denominator scans used by generic exact rationals.
            crate::trace_dispatch!("rational", "product_sum", "dyadic-shared-denominator");
            return dyadic;
        }

        let mut denominators: [BigUint; TERMS] = std::array::from_fn(|_| BigUint::ZERO);
        let mut shared_denominator = None::<BigUint>;
        let mut equal_denominator = true;
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = Self::product_term_denominator(terms[i]);
            match &shared_denominator {
                None => shared_denominator = Some(denominator.clone()),
                Some(shared) if *shared == denominator => {}
                Some(_) => equal_denominator = false,
            }
            denominators[i] = denominator;
        }

        if equal_denominator {
            let denominator = shared_denominator.expect("nonzero product sum has denominator");
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..TERMS {
                let sign = signs[i];
                if sign == NoSign {
                    continue;
                }
                let magnitude = Self::product_term_magnitude(terms[i]);
                match sign {
                    Plus => positive += magnitude,
                    Minus => negative += magnitude,
                    NoSign => {}
                }
            }

            crate::trace_dispatch!("rational", "product_sum", "equal-product-denominator");
            return Self::from_signed_magnitude_difference(positive, negative, denominator);
        }

        crate::trace_dispatch!("rational", "product_sum", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        for i in 0..TERMS {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &denominators[i];
            if denominator != ONE.deref() {
                let divisor = num::Integer::gcd(&common_denominator, denominator);
                trace_rational_gcd!(&common_denominator, denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..TERMS {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let mut magnitude = Self::product_term_magnitude(terms[i]);
            let denominator = &denominators[i];
            if denominator != &common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

    /// Compare a fixed signed sum of products with zero without materializing
    /// the resulting rational when the complete expression fits in `u128`.
    pub fn signed_product_sum_ordering<const TERMS: usize, const FACTORS: usize>(
        positive_terms: [bool; TERMS],
        terms: [[&Self; FACTORS]; TERMS],
    ) -> Ordering {
        debug_assert!(FACTORS > 0);
        let signs = std::array::from_fn(|i| Self::product_term_sign(positive_terms[i], terms[i]));
        if let Some((positive, negative, _)) = Self::signed_product_sum_word_totals(terms, signs) {
            crate::trace_dispatch!("rational", "product_sum_ordering", "word-sized");
            return positive.cmp(&negative);
        }
        crate::trace_dispatch!("rational", "product_sum_ordering", "arbitrary-precision");
        match Self::signed_product_sum(positive_terms, terms).sign() {
            Minus => Ordering::Less,
            NoSign => Ordering::Equal,
            Plus => Ordering::Greater,
        }
    }

    pub(crate) fn dot_products<const N: usize>(left: [&Self; N], right: [&Self; N]) -> Self {
        // Dense vector and matrix dot products are exact rational linear
        // forms when all inputs are rational. Build one shared denominator and
        // canonicalize only the final sum instead of reducing every product
        // and partial sum. This is the same "delay fractions until the end"
        // idea used by fraction-free exact linear algebra; see Bareiss-style
        // exact division/common-factor discussion in
        // https://link.springer.com/article/10.1007/s11786-020-00495-9 and
        // the pedagogical Gauss-Jordan fraction-delay variant at
        // https://openjournals.libs.uga.edu/tme/article/view/1957/1862.
        // Current 2026-05 trace goal: keep exact matrix rows at one rational
        // constructor per output cell. This dropped mat4 powi from-f64 trace
        // activity from 161.75 to 32 reductions/call and from 462.25 to 67.75
        // temporaries/call; keep future changes within noise of those counts.
        let mut signs = [NoSign; N];
        let mut nonzero_count = 0_usize;
        for i in 0..N {
            let sign = left[i].sign * right[i].sign;
            if sign != NoSign {
                nonzero_count += 1;
            }
            signs[i] = sign;
        }
        if nonzero_count == 0 {
            crate::trace_dispatch!("rational", "dot_product", "all-zero");
            return Self::zero();
        }
        let terms = std::array::from_fn(|i| [left[i], right[i]]);
        if let Some(word) = Self::signed_product_sum_words(terms, signs) {
            crate::trace_dispatch!("rational", "dot_product", "word-sized");
            return word;
        }
        if nonzero_count == 1 {
            let mut positive = BigUint::ZERO;
            let mut negative = BigUint::ZERO;
            for i in 0..N {
                match signs[i] {
                    Plus => {
                        let denominator = &left[i].denominator * &right[i].denominator;
                        positive = &left[i].numerator * &right[i].numerator;
                        crate::trace_dispatch!("rational", "dot_product", "single-term-product");
                        return Self::from_signed_magnitude_difference(
                            positive,
                            negative,
                            denominator,
                        );
                    }
                    Minus => {
                        let denominator = &left[i].denominator * &right[i].denominator;
                        negative = &left[i].numerator * &right[i].numerator;
                        crate::trace_dispatch!("rational", "dot_product", "single-term-product");
                        return Self::from_signed_magnitude_difference(
                            positive,
                            negative,
                            denominator,
                        );
                    }
                    NoSign => {}
                }
            }
            return Self::zero();
        }

        if let Some(dyadic) = Self::dot_products_dyadic(left, right, signs) {
            // Dyadic f64 imports are the hottest exact-rational matrix path.
            // A common power-of-two denominator lets us scale numerators with
            // shifts and lets `maybe_reduce` avoid a BigInt gcd.
            // Structural-dispatch note: matrix/vector callers with retained
            // grid-scale metadata can route straight here and reserve the LCM
            // path for genuinely mixed rational inputs.
            crate::trace_dispatch!("rational", "dot_product", "dyadic-shared-denominator");
            return dyadic;
        }
        if let Some(equal_denominator) = Self::dot_products_equal_denominator(left, right, signs) {
            // Decimal rational fixtures often enter with identical product
            // denominators even after exact parsing. The LCM algorithm below is
            // still the right general fallback, but this structural fact means
            // there is no LCM to build and no per-term scale division. 2026-05
            // tracing target: lower non-dyadic rational dot-product gcd counts
            // without perturbing the dyadic fast path above. Targeted
            // Criterion, 200 samples/8s: hyperlattice hyperreal-rational
            // mat3 powi improved 2.83%, mat4 div_matrix improved 3.88%,
            // mat3 inverse_checked and mat4 powi stayed within noise.
            crate::trace_dispatch!("rational", "dot_product", "equal-product-denominator");
            return equal_denominator;
        }

        crate::trace_dispatch!("rational", "dot_product", "lcm-shared-denominator");
        let mut common_denominator = BigUint::one();
        let mut any_nonzero = false;
        for i in 0..N {
            if signs[i] == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            if denominator != *ONE.deref() {
                let divisor = num::Integer::gcd(&common_denominator, &denominator);
                trace_rational_gcd!(&common_denominator, &denominator, &divisor);
                common_denominator *= denominator / &divisor;
            }
            any_nonzero = true;
        }
        if !any_nonzero {
            return Self::zero();
        }

        let mut positive = BigUint::ZERO;
        let mut negative = BigUint::ZERO;
        for i in 0..N {
            let sign = signs[i];
            if sign == NoSign {
                continue;
            }
            let denominator = &left[i].denominator * &right[i].denominator;
            let mut magnitude = &left[i].numerator * &right[i].numerator;
            if denominator != common_denominator {
                magnitude *= &common_denominator / denominator;
            }
            match sign {
                Plus => positive += magnitude,
                Minus => negative += magnitude,
                NoSign => {}
            }
        }

        Self::from_signed_magnitude_difference(positive, negative, common_denominator)
    }

}
