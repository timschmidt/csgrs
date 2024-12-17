use crate::computable::{unsigned, Precision};
use crate::Computable;
use core::fmt;
use num::bigint::Sign::Minus;

fn bits(p: usize) -> Precision {
    let b = ((p + 4) * 32) / 10;
    b.try_into().expect("Bits of precision should fit")
}

fn trim(num: &mut Vec<u8>) {
    loop {
        match num.pop() {
            Some(0) => (),
            Some(other) => {
                num.push(other);
                return;
            }
            None => return,
        }
    }
}

fn up(num: &mut Vec<u8>) -> bool {
    let mut flag = false;
    let mut nines = 0;
    loop {
        match num.pop() {
            Some(9) => {
                nines += 1;
            }
            Some(other) => {
                num.push(other + 1);
                break;
            }
            None => {
                num.push(1);
                flag = true;
                nines -= 1;
                break;
            }
        }
    }
    for _ in 0..nines {
        num.push(0);
    }
    flag
}

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
        let exact = precision.unwrap_or(DEFAULT_PRECISION);
        // Precision does not include the first digit before the decimal point
        let bits = bits(exact);
        let appr = self.approx(msd - bits);
        let magn = appr.magnitude();
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

        let mut num: Vec<u8> = Vec::with_capacity(exact);
        let whole = magn / &divisor;
        let round = &whole * &divisor;
        num.push(whole.try_into().unwrap());
        let mut left = magn - round;
        for _ in 0..exact {
            left *= &*unsigned::TEN;
            let digit = &left / &divisor;
            left -= &digit * &divisor;
            num.push(digit.try_into().unwrap());
        }
        left *= &*unsigned::TWO;
        if left > divisor && up(&mut num) {
            // All nines rounded up
            exp += 1;
        }

        if precision.is_none() {
            trim(&mut num);
        }

        for (n, digit) in num.into_iter().enumerate() {
            if n == 1 {
                f.write_str(".")?;
            }
            f.write_fmt(format_args!("{digit}"))?;
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
    fn exp_smol() {
        let smol = Computable::rational(Rational::fraction(1, 1_000_000_000_000));
        let ans = smol.clone().multiply(smol);
        assert_eq!(format!("{ans:.0e}"), "1e-24");
        assert_eq!(format!("{ans:.2e}"), "1.00e-24");
        assert_eq!(format!("{ans:.4e}"), "1.0000e-24");
        assert_eq!(format!("{ans:.8e}"), "1.00000000e-24");
        assert_eq!(format!("{ans:e}"), "1e-24");
    }

    #[test]
    fn pinch() {
        let ans = Computable::rational(Rational::new(11));
        assert_eq!(format!("{ans:.0e}"), "1e1");
        assert_eq!(format!("{ans:.1e}"), "1.1e1");
        assert_eq!(format!("{ans:e}"), "1.1e1");
        let ans = Computable::rational(Rational::new(101));
        assert_eq!(format!("{ans:.0e}"), "1e2");
        assert_eq!(format!("{ans:.1e}"), "1.0e2");
        assert_eq!(format!("{ans:.2e}"), "1.01e2");
        assert_eq!(format!("{ans:e}"), "1.01e2");
        let ans = Computable::rational(Rational::new(10001));
        assert_eq!(format!("{ans:.0e}"), "1e4");
        assert_eq!(format!("{ans:.2e}"), "1.00e4");
        assert_eq!(format!("{ans:.4e}"), "1.0001e4");
        assert_eq!(format!("{ans:e}"), "1.0001e4");
        let ans = Computable::rational(Rational::new(1_000_000_001));
        assert_eq!(format!("{ans:.0e}"), "1e9");
        assert_eq!(format!("{ans:.8e}"), "1.00000000e9");
        assert_eq!(format!("{ans:.10e}"), "1.0000000010e9");
        assert_eq!(format!("{ans:e}"), "1.000000001e9");
    }

    #[test]
    fn almost() {
        let ans = Computable::rational(Rational::fraction(99, 10));
        assert_eq!(format!("{ans:.0e}"), "1e1");
        let ans = Computable::rational(Rational::fraction(999, 10));
        assert_eq!(format!("{ans:.0e}"), "1e2");
        let ans = Computable::rational(Rational::fraction(9999, 10));
        assert_eq!(format!("{ans:.0e}"), "1e3");
        let ans = Computable::rational(Rational::new(12346));
        assert_eq!(format!("{ans:.0e}"), "1e4");
        assert_eq!(format!("{ans:.3e}"), "1.235e4");
    }

    #[test]
    fn exp_huge_neg() {
        let huge = Computable::rational(Rational::new(1_000_000_000_000));
        let minus_fifty = Computable::rational(Rational::new(-50));
        let ans = huge.clone().multiply(huge).multiply(minus_fifty);
        assert_eq!(format!("{ans:.4e}"), "-5.0000e25");
        assert_eq!(format!("{ans:.0e}"), "-5e25");
        assert_eq!(format!("{ans:.2e}"), "-5.00e25");
        assert_eq!(format!("{ans:.8e}"), "-5.00000000e25");
        assert_eq!(format!("{ans:e}"), "-5e25");
    }

    #[test]
    fn exp_two_thirds() {
        let tt = Computable::rational(Rational::fraction(2, 3));
        assert_eq!(format!("{tt:.0e}"), "7e-1");
        assert_eq!(format!("{tt:.2e}"), "6.67e-1");
        assert_eq!(format!("{tt:.4e}"), "6.6667e-1");
        assert_eq!(format!("{tt:.8e}"), "6.66666667e-1");
        assert_eq!(format!("{tt:e}"), "6.66666666666666666666666666666667e-1");
    }

    #[test]
    fn exp_pi() {
        let pi = Computable::pi();
        assert_eq!(format!("{pi:.2e}"), "3.14e0");
        assert_eq!(format!("{pi:.4e}"), "3.1416e0");
        assert_eq!(format!("{pi:.8e}"), "3.14159265e0");
        assert_eq!(format!("{pi:.16e}"), "3.1415926535897932e0");
        assert_eq!(format!("{pi:.32e}"), "3.14159265358979323846264338327950e0");
        assert_eq!(format!("{pi:e}"), "3.1415926535897932384626433832795e0");
    }
}
