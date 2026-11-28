use core::fmt;

/// Present canonical parts as an integer, mixed fraction or truncated decimal.
///
/// Default output uses an integer for denominator one, `n/d` for a proper
/// fraction, and `whole remainder/d` for an improper fraction. A leading minus
/// applies to the entire value. Mixed fractions are presentation text: `FromStr`
/// does not accept their intervening space, so Display is not a general replay
/// format. Marked unreduced storage is canonicalized before formatting.
///
/// For nonintegers, alternate formatting (`#`) performs decimal long division,
/// truncating toward zero after at most the requested precision (default 1,000
/// fractional digits). It stops early at a zero remainder and does not round or
/// pad trailing zeros. Precision zero still prints the decimal point. Integers
/// ignore alternate/precision and use integral padding. Nonintegers honor the
/// explicit plus flag but ignore width, alignment and zero-padding flags.
///
/// Formatting propagates sink errors. Canonicalization and arbitrary-width
/// division/allocation have no local work, memory or cancellation budget; the
/// fractional digit limit does not bound integer output or total arithmetic work.
///
/// ```
/// use hyperreal::Rational;
/// let value = Rational::fraction(5, 3).unwrap();
/// assert_eq!(value.to_string(), "1 2/3");
/// assert_eq!(format!("{value:#.3}"), "1.666");
/// assert_eq!(format!("{value:#.0}"), "1.");
/// assert_eq!(format!("{:#.4}", Rational::fraction(1, 8).unwrap()), "0.125");
/// ```
impl fmt::Display for Rational {
    /// Delegate to a cached canonical value, then apply the selected presentation.
    ///
    /// Integer output uses `pad_integral`; fractional output writes sign and
    /// components directly. Decimal digits are exact quotient/remainder steps
    /// until termination or truncation. Dispatch events describe the chosen local
    /// path; backend radix labels are inferred diagnostics, not observed kernels.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let canonical = self.canonicalized_ref();
        if !std::sync::Arc::ptr_eq(&self.0, &canonical.0) {
            return fmt::Display::fmt(canonical, f);
        }
        if self.denominator == *ONE.deref() {
            crate::trace_dispatch!("rational_algorithm", "binary-to-radix", "integer");
            trace_rational_radix_output_algorithm!(&self.numerator);
            let int = self.numerator.to_string();
            return f.pad_integral(self.sign != Minus, "", &int);
        }

        if self.sign == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            f.write_str("+")?;
        }
        if f.alternate() {
            crate::trace_dispatch!(
                "rational_algorithm",
                "binary-to-radix",
                "rational-repeated-digit-division"
            );
            let whole = &self.numerator / &self.denominator;
            write!(f, "{whole}.")?;
            let round = &whole * &self.denominator;
            let mut left = &self.numerator - &round;
            let mut digits = f.precision().unwrap_or(1000);
            if digits == 0 {
                return Ok(());
            }
            loop {
                left *= &*TEN;
                let digit = &left / &self.denominator;
                write!(f, "{digit}")?;
                left -= digit * &self.denominator;
                if left.is_zero() {
                    break;
                }
                digits -= 1;
                if digits == 0 {
                    break;
                }
            }
            Ok(())
        } else {
            crate::trace_dispatch!("rational_algorithm", "binary-to-radix", "mixed-fraction");
            trace_rational_radix_output_algorithm!(&self.numerator);
            trace_rational_radix_output_algorithm!(&self.denominator);
            let whole = &self.numerator / &self.denominator;
            let round = &whole * &self.denominator;
            let left = &self.numerator - &round;
            if whole.is_zero() {
                write!(f, "{left}/{}", self.denominator)
            } else {
                write!(f, "{whole} {left}/{}", self.denominator)
            }
        }
    }
}

/// Shared exact decimal parsing helpers and the scientific-expansion guard.
///
/// Checked word parsing precedes arbitrary-width construction. Scientific input
/// validates ASCII digits explicitly; general magnitude fallback delegates leaf
/// grammar to num-bigint. Decimal scale counts are byte counts, which is exact
/// for digit-only input but exposes the documented separator limitations.
impl Rational {
    // GMP reports a commonly large SET_STR crossover. Local paired benchmarks
    // put this backend's crossover between 5,120 digits (product tree slower)
    // and 10,240 digits (product tree faster), so retain the power-of-two
    // boundary between them.
    /// Byte-length threshold for the local product-tree decimal parser.
    ///
    /// The word parser is tried first regardless of length, so long leading-zero
    /// inputs that still fit u128 can bypass the tree. Otherwise lengths below
    /// 8,192 use the backend and lengths at/above it use recursive splitting.
    /// Tree leaves use half this threshold. The adjacent historical benchmark
    /// comment motivates this selection; it is not a new performance measurement.
    const RADIX_INPUT_DIVIDE_CONQUER_THRESHOLD: usize = 8192;

    // A textual significand already accounts for its own storage cost, but a
    // short scientific literal can otherwise request an effectively unbounded
    // numerator or denominator. Permit generous, useful exact inputs while
    // bounding exponent-only amplification to roughly 415 KiB of BigUint
    // limbs. Longer significands retain a proportional input-sized budget.
    /// Extra decimal-power allowance beyond the source significand byte count.
    ///
    /// This limits exponent-driven amplification of nonzero scientific literals,
    /// not source length, total allocations, arithmetic work or all parser paths.
    /// The allowance is checked on the net scale before and after trailing-zero
    /// removal; intermediate allocations and backend powers remain unbudgeted.
    const MAX_SCIENTIFIC_EXPANSION_DIGITS: usize = 1_000_000;

    /// Read a nonempty ASCII digit sequence into u128 by checked multiply/add.
    ///
    /// Leading zeros are accepted without a length cap. Empty input, nondigits or
    /// numeric overflow return `None`; once folding fails, later bytes are not
    /// visited. There is no allocation, sign handling or separator acceptance.
    #[inline]
    fn parse_decimal_word(digits: &[u8]) -> Option<u128> {
        if digits.is_empty() {
            return None;
        }
        digits.iter().try_fold(0_u128, |value, &digit| {
            let digit = digit.checked_sub(b'0')?;
            if digit > 9 {
                return None;
            }
            value.checked_mul(10)?.checked_add(u128::from(digit))
        })
    }

    /// Read two concatenated ASCII digit slices without allocating a joined buffer.
    ///
    /// At least one slice must be nonempty. The point's position is not represented
    /// in the result; callers separately account for fractional length. Checked
    /// u128 overflow or a nondigit returns `None` and stops the fold. Either slice
    /// may be empty, supporting `.5` and `5.` in the outer parser.
    #[inline]
    fn parse_decimal_word_parts(whole: &[u8], fraction: &[u8]) -> Option<u128> {
        if whole.is_empty() && fraction.is_empty() {
            return None;
        }
        whole
            .iter()
            .chain(fraction)
            .try_fold(0_u128, |value, &digit| {
                let digit = digit.checked_sub(b'0')?;
                if digit > 9 {
                    return None;
                }
                value.checked_mul(10)?.checked_add(u128::from(digit))
            })
    }

    /// Validate an optional exponent sign followed by at least one ASCII digit.
    ///
    /// Returns `(negative, magnitude)`. A syntactically valid magnitude too large
    /// for usize becomes `None`, preserving the distinction from `BadDecimal`.
    /// Scanning continues after overflow so a later nondigit still wins as a syntax
    /// error. The boolean records a minus even for zero. No exponent-sized integer
    /// is allocated and no input-length or total-scan budget is imposed.
    fn parse_scientific_exponent(exponent: &[u8]) -> Result<(bool, Option<usize>), Problem> {
        let (negative, digits) = match exponent.first() {
            Some(b'-') => (true, &exponent[1..]),
            Some(b'+') => (false, &exponent[1..]),
            _ => (false, exponent),
        };
        if digits.is_empty() {
            return Err(Problem::BadDecimal);
        }

        // Keep scanning after overflow so malformed input is still reported as
        // malformed rather than as a resource-limit error.
        let mut magnitude = Some(0_usize);
        for &byte in digits {
            let digit = byte.checked_sub(b'0').ok_or(Problem::BadDecimal)?;
            if digit > 9 {
                return Err(Problem::BadDecimal);
            }
            magnitude = magnitude.and_then(|value| {
                value
                    .checked_mul(10)
                    .and_then(|value| value.checked_add(usize::from(digit)))
            });
        }
        Ok((negative, magnitude))
    }

    /// Cancel decimal powers from the exponent, point and stripped trailing zeros.
    ///
    /// The net exponent is signed exponent + significand_trailing_zeros minus
    /// fractional_digits. Returns `(in_denominator, magnitude)`, with `(false, 0)`
    /// for complete cancellation. Checked additions on the numerator/denominator
    /// side occur before cancellation, so intermediate usize overflow returns
    /// `Exhausted` even when a mathematical difference might fit. This helper does
    /// not enforce the amplification guard or materialize a power of ten.
    fn scientific_decimal_power(
        exponent_negative: bool,
        exponent_magnitude: usize,
        fractional_digits: usize,
        significand_trailing_zeros: usize,
    ) -> Result<(bool, usize), Problem> {
        let mut numerator_power = significand_trailing_zeros;
        let mut denominator_power = fractional_digits;
        if exponent_negative {
            denominator_power = denominator_power
                .checked_add(exponent_magnitude)
                .ok_or(Problem::Exhausted)?;
        } else {
            numerator_power = numerator_power
                .checked_add(exponent_magnitude)
                .ok_or(Problem::Exhausted)?;
        }

        let common_power = numerator_power.min(denominator_power);
        numerator_power -= common_power;
        denominator_power -= common_power;
        Ok((denominator_power != 0, numerator_power.max(denominator_power)))
    }

    /// Test the inclusive net-power allowance against source bytes plus one million.
    ///
    /// The addition saturates at usize::MAX. `source_digits` is the significand
    /// length without its decimal point/sign; scientific grammar makes those bytes
    /// ASCII digits. This boolean is an amplification gate, not a whole-operation
    /// resource receipt or a check for actual allocator exhaustion.
    #[inline]
    fn scientific_power_is_within_budget(power: usize, source_digits: usize) -> bool {
        power
            <= source_digits.saturating_add(Self::MAX_SCIENTIFIC_EXPANSION_DIGITS)
    }

    /// Construct an exact rational from a signed decimal significand and exponent.
    ///
    /// The caller supplies `Plus` or `Minus` separately and unsigned significand
    /// text with at most one point and at least one ASCII digit. Either side of
    /// the point may be empty. The exponent accepts one optional sign and digits;
    /// separators, extra signs/points and whitespace are rejected as `BadDecimal`.
    /// Both components are validated before a zero significand returns canonical
    /// zero, even if the exponent magnitude overflows usize.
    ///
    /// Nonzero values require a representable exponent and checked decimal-power
    /// arithmetic within the source-sized amplification allowance. A u128 scale
    /// and magnitude are preferred. The wider path joins parts if needed, removes
    /// trailing decimal zeros, recomputes the scale, and forms a power of ten.
    /// A denominator scale is reduced via binary cancellation when the final
    /// digit is even, is already coprime when that digit is odd and not five, or
    /// uses general reduction for five. These decisions preserve exact parts.
    ///
    /// `Exhausted` reports checked size/exponent conversion or reservation failures
    /// and the amplification guard; it is not evidence that every allocation/work
    /// limit is enforced. The initial guard precedes trailing-zero removal, so a
    /// rejected scale is not retried after that cancellation. Backend integer
    /// powers, products and reductions have no total-work or cancellation hook.
    fn parse_scientific(
        sign: Sign,
        significand: &str,
        exponent: &str,
    ) -> Result<Self, Problem> {
        let (whole, fraction) = significand.split_once('.').unwrap_or((significand, ""));
        let source_digits = whole
            .len()
            .checked_add(fraction.len())
            .ok_or(Problem::Exhausted)?;
        if source_digits == 0 {
            return Err(Problem::BadDecimal);
        }

        let (exponent_negative, exponent_magnitude) =
            Self::parse_scientific_exponent(exponent.as_bytes())?;
        let word_magnitude =
            Self::parse_decimal_word_parts(whole.as_bytes(), fraction.as_bytes());
        if word_magnitude == Some(0) {
            // Zero does not need exponent materialization, but both the
            // significand and exponent have still been fully validated.
            crate::trace_dispatch!("rational", "parse", "scientific-zero");
            return Ok(Self::zero());
        }

        if word_magnitude.is_none()
            && (!whole.bytes().all(|byte| byte.is_ascii_digit())
                || !fraction.bytes().all(|byte| byte.is_ascii_digit()))
        {
            return Err(Problem::BadDecimal);
        }
        let exponent_magnitude = exponent_magnitude.ok_or(Problem::Exhausted)?;
        let (negative_power, power) = Self::scientific_decimal_power(
            exponent_negative,
            exponent_magnitude,
            fraction.len(),
            0,
        )?;
        if !Self::scientific_power_is_within_budget(power, source_digits) {
            return Err(Problem::Exhausted);
        }

        if let Some(magnitude) = word_magnitude {
            let power_u32 = u32::try_from(power).map_err(|_| Problem::Exhausted)?;
            if let Some(scale) = 10_u128.checked_pow(power_u32) {
                if negative_power {
                    crate::trace_dispatch!("rational", "parse", "word-sized-scientific");
                    let (positive, negative) = if sign == Minus {
                        (0, magnitude)
                    } else {
                        (magnitude, 0)
                    };
                    return Ok(Self::from_word_magnitude_difference(
                        positive, negative, scale,
                    ));
                }
                if let Some(magnitude) = magnitude.checked_mul(scale) {
                    crate::trace_dispatch!("rational", "parse", "word-sized-scientific");
                    return Ok(Self::from_primitive_integer(sign, magnitude));
                }
            }
        }

        let digits = if fraction.is_empty() {
            std::borrow::Cow::Borrowed(whole.as_bytes())
        } else {
            let mut digits = Vec::new();
            digits
                .try_reserve_exact(source_digits)
                .map_err(|_| Problem::Exhausted)?;
            digits.extend_from_slice(whole.as_bytes());
            digits.extend_from_slice(fraction.as_bytes());
            std::borrow::Cow::Owned(digits)
        };
        let significant_len = digits
            .iter()
            .rposition(|&digit| digit != b'0')
            .map(|index| index + 1)
            .expect("zero significands exit on the word path");
        let trailing_zeros = digits.len() - significant_len;
        let digits = &digits[..significant_len];

        let (negative_power, power) = Self::scientific_decimal_power(
            exponent_negative,
            exponent_magnitude,
            fraction.len(),
            trailing_zeros,
        )?;
        if !Self::scientific_power_is_within_budget(power, source_digits) {
            return Err(Problem::Exhausted);
        }
        let power = u32::try_from(power).map_err(|_| Problem::Exhausted)?;
        let mut magnitude =
            Self::parse_decimal_magnitude(digits).ok_or(Problem::BadDecimal)?;
        if power == 0 {
            crate::trace_dispatch!("rational", "parse", "scientific-integer");
            return Ok(Self::from_integer_magnitude(sign, magnitude));
        }

        let mut scale = TEN.pow(power);
        if negative_power {
            let final_digit = *digits.last().expect("scientific magnitude is nonzero");
            if final_digit.is_multiple_of(2) {
                // Removing trailing decimal zeroes proves this magnitude has
                // no factor of five. Cancel its binary factor directly and the
                // remaining parts are coprime without a general BigUint GCD.
                let common_shift = magnitude
                    .trailing_zeros()
                    .expect("scientific magnitude is nonzero")
                    .min(u64::from(power));
                if common_shift != 0 {
                    let common_shift =
                        usize::try_from(common_shift).map_err(|_| Problem::Exhausted)?;
                    magnitude >>= common_shift;
                    scale >>= common_shift;
                }
                crate::trace_dispatch!(
                    "rational",
                    "parse",
                    "scientific-binary-factor-cancel"
                );
                Ok(Self::from_fraction_parts(sign, magnitude, scale))
            } else if final_digit != b'5' {
                crate::trace_dispatch!("rational", "parse", "scientific-coprime-scale");
                Ok(Self::from_fraction_parts(sign, magnitude, scale))
            } else {
                crate::trace_dispatch!("rational", "parse", "scientific-five-factor-reduction");
                Ok(Self::from_fraction_parts_reduced(sign, magnitude, scale))
            }
        } else {
            crate::trace_dispatch!("rational", "parse", "scientific-numerator-scale");
            magnitude *= scale;
            Ok(Self::from_integer_magnitude(sign, magnitude))
        }
    }

    /// Parse an unsigned decimal magnitude through word, backend and tree paths.
    ///
    /// Digit-only nonempty ASCII input gives its exact integer value, subject to
    /// resources. The checked u128 path is attempted before the length threshold.
    /// Shorter fallback inputs use BigUint's radix-10 parser; larger ones allocate
    /// a per-call decimal-power cache and recurse. `None` combines parse failure
    /// with an unrepresentable tree exponent; allocation failure is not uniformly
    /// converted to an Option or typed resource error. No total-work cap applies.
    ///
    /// The backend also accepts a leading plus and some underscore separators.
    /// Those extensions are not validated uniformly here: recursive scale lengths
    /// count bytes, including separators, and leaf boundaries affect acceptance.
    /// For example, a trailing underscore on an 8,192-byte input can change its
    /// value relative to the backend parser. This is an implementation limitation,
    /// not an exact grammar guarantee for separator-bearing input.
    fn parse_decimal_magnitude(digits: &[u8]) -> Option<BigUint> {
        if let Some(value) = Self::parse_decimal_word(digits) {
            crate::trace_dispatch!("rational_algorithm", "radix-to-binary", "word-sized");
            return Some(BigUint::from(value));
        }
        if digits.len() < Self::RADIX_INPUT_DIVIDE_CONQUER_THRESHOLD {
            crate::trace_dispatch!(
                "rational_algorithm",
                "radix-to-binary",
                "backend-chunked-multiply-add"
            );
            return BigUint::parse_bytes(digits, 10);
        }

        crate::trace_dispatch!(
            "rational_algorithm",
            "radix-to-binary",
            "divide-conquer-product-tree"
        );
        let mut powers = std::collections::BTreeMap::new();
        Self::parse_decimal_magnitude_tree(digits, &mut powers)
    }

    /// Combine decimal halves as left * 10^(right byte length) + right.
    ///
    /// For digit-only input this is the exact positional-value identity. Leaves
    /// shorter than 4,096 bytes delegate to BigUint; larger spans recurse left then
    /// right. A caller-owned BTreeMap caches powers by right length. Every prefilled
    /// entry must equal 10^key: this invariant is trusted, not revalidated. Hits
    /// clone a BigUint, and misses compute, insert a clone, then use the power.
    ///
    /// A failed leaf or right length outside u32 returns `None`; the cache may
    /// already contain entries from earlier recursion. It has no eviction, total
    /// work or byte budget. Recursion depth is logarithmic in input length, but
    /// power construction, multiplication and retained limbs still grow with it.
    /// Backend separator/plus acceptance in leaves does not justify byte-count
    /// scales for those inputs and can cause incorrect values or split-dependent
    /// rejection. The outer magnitude helper creates a fresh cache for each call.
    fn parse_decimal_magnitude_tree(
        digits: &[u8],
        powers: &mut std::collections::BTreeMap<usize, BigUint>,
    ) -> Option<BigUint> {
        if digits.len() < Self::RADIX_INPUT_DIVIDE_CONQUER_THRESHOLD / 2 {
            return BigUint::parse_bytes(digits, 10);
        }

        let midpoint = digits.len() / 2;
        let (left, right) = digits.split_at(midpoint);
        let left_value = Self::parse_decimal_magnitude_tree(left, powers)?;
        let right_value = Self::parse_decimal_magnitude_tree(right, powers)?;
        let right_power = match powers.get(&right.len()) {
            Some(power) => power.clone(),
            None => {
                let exponent = u32::try_from(right.len()).ok()?;
                let power = TEN.pow(exponent);
                powers.insert(right.len(), power.clone());
                power
            }
        };
        let value = left_value * &right_power + right_value;
        Some(value)
    }
}

/// Parse exact signed integers, fractions, decimals and scientific literals.
///
/// The intended decimal grammar uses ASCII digits and an optional leading sign.
/// Fractions are `numerator/denominator`; decimals permit `.5` and `5.`; scientific
/// forms add `e` or `E` and an optionally signed integer exponent. No floating-point
/// conversion occurs. Whitespace and Display's `whole remainder/denominator` mixed
/// notation are not accepted. After removing one leading sign, dispatch checks
/// scientific notation first, then slash, point and finally integer syntax.
///
/// Malformed input maps to `BadDecimal`, `BadFraction` or `BadInteger` according
/// to that selected branch. Zero denominators yield `DivideByZero`, including
/// `0/0`. Scientific exponent amplification and checked size conversions can
/// yield `Exhausted`; the parser as a whole has no total-work, memory or
/// cancellation budget, and ordinary integer/decimal/fraction input has no
/// matching source-length guard.
///
/// Known fallback limitations: num-bigint accepts extra plus signs and underscores
/// that the word/scientific grammar rejects. Examples include `++1` becoming one
/// and `1/+2` becoming one half. Fractional scale counts raw bytes, so `1.2_5`
/// currently becomes `41/40`, not the separator-free decimal `1.25`; large
/// product-tree inputs have analogous byte-count and split-boundary defects.
/// These are implementation gaps, not additional exact syntax contracts.
///
/// ```
/// use hyperreal::{Problem, Rational};
/// assert_eq!(".125e1".parse::<Rational>().unwrap(), Rational::fraction(5, 4).unwrap());
/// assert_eq!("9/18".parse::<Rational>().unwrap(), Rational::fraction(1, 2).unwrap());
/// assert_eq!("0/0".parse::<Rational>(), Err(Problem::DivideByZero));
/// assert_eq!("1 2/3".parse::<Rational>(), Err(Problem::BadFraction));
/// ```
impl std::str::FromStr for Rational {
    /// Syntax, zero-denominator or checked expansion/resource failure for this parse.
    /// Backend allocations are not all fallible through this error channel.
    type Err = Problem;

    /// Select a grammar branch after removing at most one leading sign.
    ///
    /// Checked word construction precedes arbitrary-width parsing in each branch.
    /// Fallback fractions reduce common factors; fallback decimals parse the whole
    /// and fractional pieces separately, reduce the fraction and add signed parts.
    /// A zero fractional piece returns the whole before constructing its scale.
    /// Scientific dispatch validates both components through the dedicated helper;
    /// other fallback branches inherit the backend syntax limitations above.
    fn from_str(s: &str) -> Result<Self, Problem> {
        let mut sign: Sign = Plus;
        let s = match s.as_bytes().first() {
            Some(b'-') => {
                sign = Minus;
                &s[1..]
            }
            Some(b'+') => &s[1..],
            _ => s,
        };
        if let Some((significand, exponent)) = s.split_once(['e', 'E']) {
            crate::trace_dispatch!("rational", "parse", "scientific");
            Self::parse_scientific(sign, significand, exponent)
        } else if let Some((n, d)) = s.split_once('/') {
            crate::trace_dispatch!("rational", "parse", "fraction");
            if let (Some(numerator), Some(denominator)) = (
                Self::parse_decimal_word(n.as_bytes()),
                Self::parse_decimal_word(d.as_bytes()),
            ) {
                if denominator == 0 {
                    return Err(Problem::DivideByZero);
                }
                crate::trace_dispatch!("rational", "parse", "word-sized-fraction");
                let (positive, negative) = if sign == Minus {
                    (0, numerator)
                } else {
                    (numerator, 0)
                };
                return Ok(Self::from_word_magnitude_difference(
                    positive,
                    negative,
                    denominator,
                ));
            }
            let numerator = Self::parse_decimal_magnitude(n.as_bytes())
                .ok_or(Problem::BadFraction)?;
            if numerator.is_zero() {
                sign = NoSign;
            }
            let denominator = Self::parse_decimal_magnitude(d.as_bytes())
                .ok_or(Problem::BadFraction)?;
            if denominator.is_zero() {
                return Err(Problem::DivideByZero);
            }
            Ok(Self::from_fraction_parts(sign, numerator, denominator).reduce())
        } else if let Some((i, d)) = s.split_once('.') {
            crate::trace_dispatch!("rational", "parse", "decimal");
            if let (Some(numerator), Ok(exponent)) = (
                Self::parse_decimal_word_parts(i.as_bytes(), d.as_bytes()),
                u32::try_from(d.len()),
            ) && let Some(denominator) = 10_u128.checked_pow(exponent)
            {
                crate::trace_dispatch!("rational", "parse", "word-sized-decimal");
                let (positive, negative) = if sign == Minus {
                    (0, numerator)
                } else {
                    (numerator, 0)
                };
                return Ok(Self::from_word_magnitude_difference(
                    positive,
                    negative,
                    denominator,
                ));
            }
            if i.is_empty() && d.is_empty() {
                return Err(Problem::BadDecimal);
            }
            let numerator = if i.is_empty() {
                BigUint::zero()
            } else {
                Self::parse_decimal_magnitude(i.as_bytes()).ok_or(Problem::BadDecimal)?
            };
            let whole = if numerator.is_zero() {
                Self::from_parts_raw(NoSign, numerator, One::one())
            } else {
                Self::from_parts_raw(sign, numerator, One::one())
            };
            if d.is_empty() {
                return Ok(whole);
            }
            let numerator =
                Self::parse_decimal_magnitude(d.as_bytes()).ok_or(Problem::BadDecimal)?;
            if numerator.is_zero() {
                return Ok(whole);
            }
            let exponent = u32::try_from(d.len()).map_err(|_| Problem::Exhausted)?;
            let denominator = TEN.pow(exponent);
            // Digits and 10^k may share factors. Stored parts must be reduced:
            // equality fast paths, hashing and dyadic facts read them directly.
            let fraction = Self::from_fraction_parts(sign, numerator, denominator).reduce();
            Ok(whole + fraction)
        } else {
            crate::trace_dispatch!("rational", "parse", "integer");
            if let Some(numerator) = Self::parse_decimal_word(s.as_bytes()) {
                crate::trace_dispatch!("rational", "parse", "word-sized-integer");
                return Ok(Self::from_primitive_integer(sign, numerator));
            }
            let numerator = Self::parse_decimal_magnitude(s.as_bytes())
                .ok_or(Problem::BadInteger)?;
            if numerator.is_zero() {
                sign = NoSign;
            }
            Ok(Self::from_parts_raw(sign, numerator, One::one()))
        }
    }
}
