use num::{BigInt, BigUint, One};
use crate::{Real, RealProblem, Rational};

impl From<i64> for Real {
    fn from(n: i64) -> Real {
        Real::new(Rational::new(n))
    }
}

impl From<i32> for Real {
    fn from(n: i32) -> Real {
        Real::new(Rational::new(n.into()))
    }
}

impl From<Rational> for Real {
    fn from(rational: Rational) -> Real {
        Real::new(rational)
    }
}

fn signed(n: Rational, neg: bool) -> Real {
    if neg {
        Real::new(-n)
    } else {
        Real::new(n)
    }
}

impl TryFrom<f32> for Real {
    type Error = RealProblem;

    fn try_from(n: f32) -> Result<Real,Self::Error> {
        let bits = n.to_bits();
        let neg = (bits & 0x8000_0000) == 0x8000_0000;
        let exp = (bits & 0x7f80_0000) >> 23;
        let sig = bits & 0x7f_ffff;
        match exp {
            0 => {
                if sig == 0 {
                    Ok(Real::new(Rational::zero()))
                } else {
                    let numerator: BigInt = sig.into();
                    let denominator = BigUint::one() << 149;
                    Ok(signed(Rational::from_bigint_fraction(numerator, denominator), neg))
                }
            }
            1..=150 => {
                let n = 0x80_0000 + sig;
                let numerator: BigInt = n.into();
                let denominator = BigUint::one() << (150 - exp);
                Ok(signed(Rational::from_bigint_fraction(numerator, denominator), neg))
            }
            151..=254 => {
                let n = 0x80_0000 + sig;
                let mut big: BigInt = n.into();
                big <<= exp - 150;
                Ok(signed(Rational::from_bigint(big), neg))
            }
            255 => {
                if sig == 0 {
                    Err(RealProblem::Infinity)
                } else {
                    Err(RealProblem::NotANumber)
                }
            }
            _ => unreachable!()
        }
    }
}

impl TryFrom<f64> for Real {
    type Error = RealProblem;

    fn try_from(n: f64) -> Result<Real,Self::Error> {
        let bits = n.to_bits();
        let neg = (bits & 0x8000_0000_0000_0000) == 0x8000_0000_0000_0000;
        let exp = (bits & 0x7ff0_0000_0000_0000) >> 52;
        let sig = bits & 0xf_ffff_ffff_ffff;
        match exp {
            0 => {
                println!("Z>>> {neg} {exp} {sig}");
                if sig == 0 {
                    Ok(Real::new(Rational::zero()))
                } else {
                    let numerator: BigInt = sig.into();
                    let denominator = BigUint::one() << 1074;
                    Ok(signed(Rational::from_bigint_fraction(numerator, denominator), neg))
                }
            }
            1..=1075 => {
                println!("v>>> {neg} {exp} {sig}");
                let n = 0x10_0000_0000_0000 + sig;
                let numerator: BigInt = n.into();
                let denominator = BigUint::one() << (1075 - exp);
                Ok(signed(Rational::from_bigint_fraction(numerator, denominator), neg))
            }
            1076..=2046 => {
                println!("^>>> {neg} {exp} {sig}");
                let n = 0x10_0000_0000_0000 + sig;
                let mut big: BigInt = n.into();
                big <<= exp - 1075;
                Ok(signed(Rational::from_bigint(big), neg))
            }
            2047 => {
                if sig == 0 {
                    Err(RealProblem::Infinity)
                } else {
                    Err(RealProblem::NotANumber)
                }
            }
            _ => unreachable!()
        }
    }
}
