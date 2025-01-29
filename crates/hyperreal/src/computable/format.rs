use crate::computable::{unsigned, Precision};
use crate::Computable;
use core::fmt;
use num::bigint::Sign::Minus;
use num::{BigUint, Zero};

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

const DEFAULT_PRECISION: usize = 32;

fn enough_bits(msd: Precision, prec: Option<usize>) -> Precision {
    fn bits(p: usize) -> Precision {
        let b = ((p + 4) * 32) / 10;
        b.try_into().expect("Bits of precision should fit")
    }

    let bits = bits(prec.unwrap_or(DEFAULT_PRECISION)) as Precision;
    if msd > 0 {
        bits + msd
    } else {
        bits
    }
}

#[derive(Copy, Clone, Debug)]
enum Places {
    Exp(usize),
    Zero(usize),
}

impl Places {
    fn digits(self, exp: i32) -> usize {
        use Places::*;

        match self {
            Exp(n) => n + 1,
            Zero(n) => {
                let places = n as i32 + exp + 1;
                if places < 0 {
                    0
                } else {
                    places as usize
                }
            }
        }
    }
}

// digits but when we know it's all zeroes
fn zeroes(places: Places, stop: bool) -> (Vec<u8>, i32) {
    if stop {
        (vec![0], 0)
    } else {
        let count = places.digits(0);
        (vec![0; count], 0)
    }
}

// output decimal digits (as bytes) Vec<u8> and a exponent
fn digits(
    magn: &BigUint,
    places: Places,
    bits: Precision,
    msd: Precision,
    stop: bool,
) -> (Vec<u8>, i32) {
    let mut exp: i32 = 0;
    let mut divisor = unsigned::ONE.clone();
    let mut excess = msd - bits;

    if *magn == BigUint::zero() {
        return zeroes(places, stop);
    }

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

    let count = places.digits(exp);
    // If we're not actually here to calculate any digits, but rounding occurs...
    if count == 0 {
        divisor *= &*unsigned::FIVE;
        if magn > &divisor {
            return (vec![1], exp + 1);
        } else {
            return (vec![], exp);
        }
    }

    let mut num: Vec<u8> = Vec::with_capacity(count);
    let mut left = magn.clone();

    for k in 0..count {
        if k > 0 {
            left *= &*unsigned::TEN;
        }
        let digit = &left / &divisor;
        left -= &digit * &divisor;
        num.push(digit.try_into().unwrap());
    }
    left *= &*unsigned::TWO;
    if left > divisor && up(&mut num) {
        // All nines rounded up
        exp += 1;
    }

    if stop {
        trim(&mut num);
    }

    (num, exp)
}

impl fmt::Display for Computable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.sign() == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            // Even for zero
            f.write_str("+")?;
        }
        let msd = self.iter_msd();
        let bits = enough_bits(msd, f.precision());
        let appr = self.approx(msd - bits);
        let mut dp = f.precision().unwrap_or(DEFAULT_PRECISION);
        let (num, mut exp) = digits(appr.magnitude(), Places::Zero(dp), bits, msd, true);
        let mut num = num.into_iter().peekable();

        if exp < 0 {
            f.write_str("0")?;
        }
        while exp >= 0 {
            let digit = num.next().unwrap_or_default();
            write!(f, "{digit}")?;
            exp -= 1;
        }
        // Decimal point or early exit if we won't write any decimal places
        if dp == 0 {
            return Ok(());
        }
        if f.precision().is_none() && num.peek().is_none() {
            return Ok(());
        }
        f.write_str(".")?;

        // After the decimal point
        while exp < -1 && dp > 0 {
            f.write_str("0")?;
            exp += 1;
            dp -= 1;
        }
        for digit in num {
            if dp == 0 {
                return Ok(());
            }
            dp -= 1;
            write!(f, "{digit}")?;
        }
        if f.precision().is_none() {
            return Ok(());
        }
        for _ in 0..dp {
            f.write_str("0")?;
        }
        Ok(())
    }
}

impl fmt::UpperExp for Computable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.sign() == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            // Even for zero
            f.write_str("+")?;
        }
        let msd = self.iter_msd();
        let precision = f.precision();
        // Precision does not include the first digit before the decimal point
        let exact = precision.unwrap_or(DEFAULT_PRECISION);
        let bits = enough_bits(msd, f.precision());
        let appr = self.approx(msd - bits);
        let (num, exp) = digits(
            appr.magnitude(),
            Places::Exp(exact),
            bits,
            msd,
            precision.is_none(),
        );

        for (n, digit) in num.into_iter().enumerate() {
            if n == 1 {
                f.write_str(".")?;
            }
            write!(f, "{digit}")?;
        }
        write!(f, "E{exp}")?;
        Ok(())
    }
}

impl fmt::LowerExp for Computable {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.sign() == Minus {
            f.write_str("-")?;
        } else if f.sign_plus() {
            // Even for zero
            f.write_str("+")?;
        }
        let msd = self.iter_msd();
        let precision = f.precision();
        // Precision does not include the first digit before the decimal point
        let exact = precision.unwrap_or(DEFAULT_PRECISION);
        let bits = enough_bits(msd, f.precision());
        let appr = self.approx(msd - bits);
        let (num, exp) = digits(
            appr.magnitude(),
            Places::Exp(exact),
            bits,
            msd,
            precision.is_none(),
        );

        for (n, digit) in num.into_iter().enumerate() {
            if n == 1 {
                f.write_str(".")?;
            }
            write!(f, "{digit}")?;
        }
        write!(f, "e{exp}")?;
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
        assert_eq!(format!("{ans:.2E}"), "1.00E-24");
        assert_eq!(format!("{ans:.4e}"), "1.0000e-24");
        assert_eq!(format!("{ans:.8E}"), "1.00000000E-24");
        assert_eq!(format!("{ans:e}"), "1e-24");
    }

    #[test]
    fn pinch() {
        let ans = Computable::rational(Rational::new(11));
        assert_eq!(format!("{ans:.0e}"), "1e1");
        assert_eq!(format!("{ans:.1E}"), "1.1E1");
        assert_eq!(format!("{ans:e}"), "1.1e1");
        let ans = Computable::rational(Rational::new(101));
        assert_eq!(format!("{ans:.0e}"), "1e2");
        assert_eq!(format!("{ans:.1E}"), "1.0E2");
        assert_eq!(format!("{ans:.2e}"), "1.01e2");
        assert_eq!(format!("{ans:e}"), "1.01e2");
        let ans = Computable::rational(Rational::new(10001));
        assert_eq!(format!("{ans:.0e}"), "1e4");
        assert_eq!(format!("{ans:.2E}"), "1.00E4");
        assert_eq!(format!("{ans:.4e}"), "1.0001e4");
        assert_eq!(format!("{ans:e}"), "1.0001e4");
        let ans = Computable::rational(Rational::new(1_000_000_001));
        assert_eq!(format!("{ans:.0e}"), "1e9");
        assert_eq!(format!("{ans:.8E}"), "1.00000000E9");
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
        assert_eq!(format!("{ans:.0E}"), "1E4");
        assert_eq!(format!("{ans:.3e}"), "1.235e4");
    }

    #[test]
    fn exp_huge_neg() {
        let huge = Computable::rational(Rational::new(1_000_000_000_000_000_000));
        let minus_fifty = Computable::rational(Rational::new(-50));
        let ans = huge.clone().multiply(huge).multiply(minus_fifty);
        assert_eq!(format!("{ans:.4e}"), "-5.0000e37");
        assert_eq!(format!("{ans:.0e}"), "-5e37");
        assert_eq!(format!("{ans:.2e}"), "-5.00e37");
        assert_eq!(format!("{ans:.8e}"), "-5.00000000e37");
        assert_eq!(format!("{ans:e}"), "-5e37");
    }

    #[test]
    fn exp_bigger() {
        let mut huge = Computable::rational(Rational::new(1_000_000_000_000_000_000));
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        let fraction = Computable::rational(Rational::fraction(1_000_000_000_000_000_000, 3));
        let ans = huge.clone().multiply(huge).multiply(fraction);
        assert_eq!(format!("{ans:.4e}"), "3.3333e593");
        assert_eq!(format!("{ans:.0e}"), "3e593");
        assert_eq!(format!("{ans:.2e}"), "3.33e593");
        assert_eq!(format!("{ans:.8e}"), "3.33333333e593");
        assert_eq!(format!("{ans:e}"), "3.33333333333333333333333333333333e593");
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

    #[test]
    fn disp_tiny() {
        let tiny = Computable::rational(Rational::fraction(8, 1_000_000_000));
        assert_eq!(format!("{tiny:.0}"), "0");
        assert_eq!(format!("{tiny:.2}"), "0.00");
        assert_eq!(format!("{tiny:.8}"), "0.00000001");
        assert_eq!(format!("{tiny}"), "0.000000008");
    }

    #[test]
    fn disp_small() {
        let smol = Computable::rational(Rational::fraction(4, 1000_000));
        assert_eq!(format!("{smol:.0}"), "0");
        assert_eq!(format!("{smol:.2}"), "0.00");
        assert_eq!(format!("{smol}"), "0.000004");
    }

    #[test]
    fn disp_big() {
        let big = Computable::rational(Rational::new(123456789));
        assert_eq!(format!("{big:.0}"), "123456789");
        assert_eq!(format!("{big:.2}"), "123456789.00");
        assert_eq!(format!("{big}"), "123456789");
    }

    #[test]
    fn actual_zero() {
        let zero = Computable::rational(Rational::zero());
        assert_eq!(format!("{zero}"), "0");
        assert_eq!(format!("{zero:.10}"), "0.0000000000");
        assert_eq!(format!("{zero:.5E}"), "0.00000E0");
        assert_eq!(format!("{zero:.0e}"), "0e0");
    }

    #[test]
    fn disp_too_small() {
        let ratios = [(1, 3), (1, 4), (2, 5), (1, 6), (3, 7)];
        for ratio in ratios {
            let ans = Computable::rational(Rational::fraction(ratio.0, ratio.1));
            assert_eq!(format!("{ans:.0}"), "0");
        }
    }

    #[test]
    fn disp_one() {
        let ratios = [(1, 2), (3, 4), (3, 5), (5, 6), (4, 7)];
        for ratio in ratios {
            let ans = Computable::rational(Rational::fraction(ratio.0, ratio.1));
            assert_eq!(format!("{ans:.0}"), "1");
        }
    }

    #[test]
    fn disp_one_third() {
        let ot = Computable::rational(Rational::fraction(1, 3));
        assert_eq!(format!("{ot:.0}"), "0");
        assert_eq!(format!("{ot:.2}"), "0.33");
        assert_eq!(format!("{ot}"), "0.33333333333333333333333333333333");
    }

    #[test]
    fn disp_sixty_pi() {
        let pi = Computable::pi();
        let sixty = Computable::rational(Rational::new(60));
        let sp = pi.multiply(sixty);
        assert_eq!(format!("{sp:.0}"), "188");
        assert_eq!(format!("{sp:.2}"), "188.50");
        assert_eq!(format!("{sp:.8}"), "188.49555922");
        assert_eq!(format!("{sp:.16}"), "188.4955592153875943");
        assert_eq!(format!("{sp:.32}"), "188.49555921538759430775860299677017");
        assert_eq!(format!("{sp}"), "188.49555921538759430775860299677017");
    }

    #[test]
    fn disp_pi() {
        let pi = Computable::pi();
        assert_eq!(format!("{pi:.2}"), "3.14");
        assert_eq!(format!("{pi:.4}"), "3.1416");
        assert_eq!(format!("{pi:.8}"), "3.14159265");
        assert_eq!(format!("{pi:.16}"), "3.1415926535897932");
        assert_eq!(format!("{pi:.32}"), "3.14159265358979323846264338327950");
        assert_eq!(format!("{pi}"), "3.1415926535897932384626433832795");
    }

    #[test]
    fn disp_two_thirds() {
        let tt = Computable::rational(Rational::fraction(2, 3));
        assert_eq!(format!("{tt:.0}"), "1");
        assert_eq!(format!("{tt:.2}"), "0.67");
        assert_eq!(format!("{tt:.4}"), "0.6667");
        assert_eq!(format!("{tt:.8}"), "0.66666667");
        assert_eq!(format!("{tt}"), "0.66666666666666666666666666666667");
    }

    #[test]
    fn disp_threes() {
        let mut huge = Computable::rational(Rational::new(1_000_000_000_000_000_000));
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        huge = huge.clone().multiply(huge);
        let fraction = Computable::rational(Rational::fraction(1_000_000_000_000_000_000, 3));
        let ans = huge.clone().multiply(huge).multiply(fraction);
        assert_eq!(format!("{ans:.4}").trim_matches('3'), ".");
        assert_eq!(format!("{ans:.0}").trim_matches('3'), "");
        assert_eq!(format!("{ans:.2}").trim_matches('3'), ".");
        assert_eq!(format!("{ans:.8}").trim_matches('3'), ".");
        assert_eq!(format!("{ans}").trim_matches('3'), ".");
    }
}
