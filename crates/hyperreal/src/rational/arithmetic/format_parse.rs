use core::fmt;

impl fmt::Display for Rational {
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

impl Rational {
    // GMP reports a commonly large SET_STR crossover. Local paired benchmarks
    // put this backend's crossover between 5,120 digits (product tree slower)
    // and 10,240 digits (product tree faster), so retain the power-of-two
    // boundary between them.
    const RADIX_INPUT_DIVIDE_CONQUER_THRESHOLD: usize = 8192;

    // A textual significand already accounts for its own storage cost, but a
    // short scientific literal can otherwise request an effectively unbounded
    // numerator or denominator. Permit generous, useful exact inputs while
    // bounding exponent-only amplification to roughly 415 KiB of BigUint
    // limbs. Longer significands retain a proportional input-sized budget.
    const MAX_SCIENTIFIC_EXPANSION_DIGITS: usize = 1_000_000;

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

    #[inline]
    fn scientific_power_is_within_budget(power: usize, source_digits: usize) -> bool {
        power
            <= source_digits.saturating_add(Self::MAX_SCIENTIFIC_EXPANSION_DIGITS)
    }

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

impl std::str::FromStr for Rational {
    type Err = Problem;

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
            let fraction = Self::from_parts_raw(sign, numerator, denominator);
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
