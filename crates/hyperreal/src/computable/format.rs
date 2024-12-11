use crate::computable::{unsigned, Precision};
use crate::Computable;
use core::fmt;
use num::bigint::Sign::Minus;

impl fmt::LowerExp for Computable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        const DEFAULT_PRECISION: usize = 32;
        if self.sign() == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            // Even for zero
            f.write_str("+")?;
        }
        let msd = self.iter_msd();
        let precision = f.precision();
        // Precision does not include the first digit before the decimal point
        let bits: Precision = ((precision.unwrap_or(DEFAULT_PRECISION) + 1) * 4)
            .try_into()
            .expect("Bits of precision should fit");
        let appr = self.approx(msd - bits);
        let magn = appr.magnitude();
        let mut slack = unsigned::TEN.clone();
        let mut exp: i32 = 0;
        let mut divisor = unsigned::ONE.clone();
        let mut excess = msd - bits;

        // If we have enough bits already then just divide off the powers of two
        if excess < 0 {
            divisor <<= bits - msd;
        }

        // Regardless, adjust until we've calculated the decimal exponent
        loop {
            while divisor <= *magn {
                if excess > 0 {
                    excess -= 1;
                    exp += 1;
                    divisor *= &*unsigned::FIVE;
                } else {
                    exp += 1;
                    divisor *= &*unsigned::TEN;
                }
            }
            while divisor > *magn {
                exp -= 1;
                divisor /= &*unsigned::TEN;
            }
            if excess <= 0 {
                break;
            }
        }

        let whole = magn / &divisor;
        f.write_fmt(format_args!("{whole}."))?;
        let round = &whole * &divisor;
        let mut digits = 0;
        let mut left = magn - round;
        while digits < precision.unwrap_or(DEFAULT_PRECISION) {
            left *= &*unsigned::TEN;
            slack *= &*unsigned::TEN;
            let digit = &left / &divisor;
            f.write_fmt(format_args!("{digit}"))?;
            left -= digit * &divisor;
            // The residual may never become zero as it is an approximation
            if left < slack && precision.is_none() {
                break;
            }
            digits += 1;
        }
        f.write_fmt(format_args!("e{exp}"))?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Rational;

    #[test]
    fn pi() {
        let pi = Computable::pi();
        assert_eq!(format!("{pi:.4e}"), "3.1415e0");
        assert_eq!(format!("{pi:.8e}"), "3.14159265e0");
        assert_eq!(format!("{pi:e}"), "3.14159265358979323846264338327950e0");
    }

    #[test]
    fn smol() {
        let smol = Computable::rational(Rational::fraction(1, 1_000_000_000));
        assert_eq!(format!("{smol:.4e}"), "1.0000e-9");
        assert_eq!(format!("{smol:.8e}"), "1.00000000e-9");
        assert_eq!(format!("{smol:e}"), "1.0e-9");
    }
}
