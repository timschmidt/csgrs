use hyperreal::{CertifiedRealSign, Rational, Real, RealSign};
use num::{BigInt, BigUint, One};

fn argument(case: usize) -> Real {
    match case {
        0 => Real::from(Rational::fraction(1, 3).unwrap()),
        1 => Real::from(2).sqrt().unwrap(),
        2 => Real::from(1).sin(),
        3 => Real::from(2).sqrt().unwrap() + Real::from(3).sqrt().unwrap(),
        _ => unreachable!(),
    }
}

fn main() {
    println!("case,delta_bits,delta_sign,reversed,precision,sign");
    for case in 0..4 {
        for bits in [767usize, 999, 2048] {
            for delta_sign in [-1, 1] {
                for reversed in [false, true] {
                    for precision in [-64, -256, -512] {
                        // Fresh independent construction; monotonicity of exp
                        // specifies the sign without a floating-point oracle.
                        let left = argument(case).exp().unwrap().sqrt().unwrap();
                        let delta = Rational::from_bigint_fraction(
                            BigInt::from(delta_sign), BigUint::one() << bits,
                        ).unwrap();
                        let right = (argument(case) * Real::from(Rational::fraction(1, 2).unwrap())
                            + Real::from(delta)).exp().unwrap();
                        let difference = if reversed { right - left } else { left - right };
                        let positive = (delta_sign < 0) != reversed;
                        let result = match difference.certified_sign_until(precision) {
                            CertifiedRealSign::Known { sign: RealSign::Positive, .. } => {
                                assert!(positive); "Positive"
                            }
                            CertifiedRealSign::Known { sign: RealSign::Negative, .. } => {
                                assert!(!positive); "Negative"
                            }
                            CertifiedRealSign::Known { sign: RealSign::Zero, .. } => panic!("false zero"),
                            CertifiedRealSign::Unknown { .. } => "Unknown",
                        };
                        println!("{case},{bits},{delta_sign},{reversed},{precision},{result}");
                    }
                }
            }
        }
    }
}
