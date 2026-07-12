impl Rational {
    const EXTRACT_SQUARE_MAX_LEN: u64 = 5000;

    fn make_squares() -> Vec<(BigUint, BigUint)> {
        // Tiny prime-square table covers the residuals that appear most often
        // in exact trig and matrix examples without running full factorization.
        vec![
            (BigUint::from(2_u8), BigUint::from(4_u8)),
            (BigUint::from(3_u8), BigUint::from(9_u8)),
            (BigUint::from(5_u8), BigUint::from(25_u8)),
            (BigUint::from(7_u8), BigUint::from(49_u8)),
            (BigUint::from(11_u8), BigUint::from(121_u8)),
            // 13 and 17 fit in u8; their squares need u16.
            (BigUint::from(13_u8), BigUint::from(169_u16)),
            (BigUint::from(17_u8), BigUint::from(289_u16)),
        ]
    }

    // Some(root) squared is n, otherwise None
    fn try_perfect(n: BigUint) -> Option<BigUint> {
        // BigUint::sqrt is cheap enough as a final check once small square
        // factors have been stripped.
        let root = n.sqrt();
        let square = &root * &root;
        if square == n { Some(root) } else { None }
    }

    // (root squared times rest) = n
    fn extract_square(n: BigUint) -> (BigUint, BigUint) {
        static SQUARES: LazyLock<Vec<(BigUint, BigUint)>> = LazyLock::new(Rational::make_squares);

        // Partial square extraction is a performance shortcut, not full prime
        // factorization. It peels common small squares and then tests a few
        // residual divisors so sqrt simplification remains bounded.
        let one: BigUint = One::one();
        let mut root = one.clone();
        let mut rest = n;
        if rest.bits() > Self::EXTRACT_SQUARE_MAX_LEN {
            return (root, rest);
        }
        for (p, s) in &*SQUARES {
            if rest == one {
                break;
            }
            while (&rest % s).is_zero() {
                rest /= s;
                root *= p;
            }
        }

        let divisors = if rest.bit(0) {
            // Odd number so dividing by an even number won't get a whole result
            // `u8` covers this fixed probe table and converts straight to BigUint.
            [1_u8, 3, 5, 7, 11, 13, 15, 17, 19]
        } else {
            [1_u8, 2, 3, 5, 6, 7, 8, 10, 11]
        };

        for n in divisors {
            let divisor = BigUint::from(n);
            if rest == divisor {
                return (root, rest);
            }
            if (&rest % &divisor).is_zero() {
                let square = &rest / &divisor;
                if let Some(factor) = Self::try_perfect(square) {
                    return (root * factor, divisor);
                }
            }
        }
        (root, rest)
    }

    /// For a value n, the result of this function is a pair (a, b)
    /// such that a * a * b = n.
    ///
    /// Where b is zero, a is the exact square root of n
    /// Otherwise, b is a residual for which no exact rational square root exists.
    pub fn extract_square_reduced(self) -> (Self, Self) {
        if self.sign == NoSign {
            return (Self::zero(), Self::zero());
        }
        let (nroot, nrest) = Self::extract_square(self.numerator);
        let (droot, drest) = Self::extract_square(self.denominator);
        (
            Self {
                sign: Plus,
                numerator: nroot,
                denominator: droot,
            },
            Self {
                sign: self.sign,
                numerator: nrest,
                denominator: drest,
            },
        )
    }

    /// For very big rationals, the algorithm used for calculating a square
    /// root is not viable, in this case the predicate is false.
    pub fn extract_square_will_succeed(&self) -> bool {
        self.numerator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
            && self.denominator.bits() < Self::EXTRACT_SQUARE_MAX_LEN
    }

    /// Return the exact nth root when both numerator and denominator are
    /// perfect nth powers. Negative values are supported for odd `n`.
    pub fn perfect_nth_root(&self, n: u32) -> Option<Self> {
        if n == 0 {
            return None;
        }
        if self.sign == NoSign {
            return Some(Self::zero());
        }
        if self.sign == Minus && n.is_multiple_of(2) {
            return None;
        }

        let numerator = self.numerator.nth_root(n);
        if numerator.pow(n) != self.numerator {
            return None;
        }

        let denominator = self.denominator.nth_root(n);
        if denominator.pow(n) != self.denominator {
            return None;
        }

        Some(Self {
            sign: self.sign,
            numerator,
            denominator,
        })
    }

    // This could grow unreasonably in terms of object size
    // so only call this for modest exp values
    fn pow_up(&self, exp: &BigUint) -> Self {
        if exp == &BigUint::ZERO {
            return Self::one();
        }
        if let Some(exp) = exp.to_u32().filter(|exp| *exp <= 64) {
            return Self {
                numerator: self.numerator.pow(exp),
                denominator: self.denominator.pow(exp),
                sign: if self.sign == Minus && exp % 2 == 1 {
                    Minus
                } else {
                    Plus
                },
            };
        }
        let mut result = Self::one();
        let mut factor = self.clone();
        let bits = exp.bits();
        for b in 0..bits {
            if exp.bit(b) {
                result *= &factor;
            }
            if b + 1 < bits {
                factor = &factor * &factor;
            }
        }
        result
    }

    /// Integer exponentiation. Raise this Rational to an integer exponent.
    pub fn powi(self, exp: BigInt) -> Result<Self, Problem> {
        const TOO_MANY_BITS: u64 = 1000;
        if exp == BigInt::ZERO {
            return Ok(Self::one());
        }
        if self.sign == NoSign {
            return if exp.sign() == Minus {
                Err(Problem::DivideByZero)
            } else {
                Ok(Self::zero())
            };
        }
        // Plus or minus one exactly
        if self.is_integer() && self.numerator == *ONE.deref() {
            if self.sign == Minus && exp.bit(0) {
                return Ok(Self::new(-1));
            } else {
                return Ok(Self::one());
            }
        }
        if exp.bits() >= TOO_MANY_BITS {
            return Err(Problem::Exhausted);
        }
        match exp.sign() {
            Minus => Ok(self.inverse()?.pow_up(exp.magnitude())),
            Plus => Ok(self.pow_up(exp.magnitude())),
            NoSign => unreachable!(),
        }
    }
}
