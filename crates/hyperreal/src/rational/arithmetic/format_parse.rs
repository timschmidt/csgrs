use core::fmt;

impl fmt::Display for Rational {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
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
        let s = match s.strip_prefix('-') {
            Some(s) => {
                sign = Minus;
                s
            }
            None => s,
        };
        if let Some((n, d)) = s.split_once('/') {
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
            if let (Some(whole), Some(fraction), Ok(exponent)) = (
                Self::parse_decimal_word(i.as_bytes()),
                Self::parse_decimal_word(d.as_bytes()),
                u32::try_from(d.len()),
            ) && let Some(denominator) = 10_u128.checked_pow(exponent)
                && let Some(numerator) = whole
                    .checked_mul(denominator)
                    .and_then(|whole| whole.checked_add(fraction))
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
            let numerator = Self::parse_decimal_magnitude(i.as_bytes())
                .ok_or(Problem::BadDecimal)?;
            let whole = if numerator.is_zero() {
                Self::from_parts_raw(NoSign, numerator, One::one())
            } else {
                Self::from_parts_raw(sign, numerator, One::one())
            };
            let numerator = Self::parse_decimal_magnitude(d.as_bytes())
                .ok_or(Problem::BadDecimal)?;
            if numerator.is_zero() {
                return Ok(whole);
            }
            let denominator = TEN.pow(d.len() as u32);
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
