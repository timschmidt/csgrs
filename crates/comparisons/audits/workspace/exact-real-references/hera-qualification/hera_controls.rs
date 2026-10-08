use hyperreal::{Computable, Rational, Real};
use rug::{Float, Integer, float::Round};

fn pow2(exponent: i32) -> Rational {
    let integer = Integer::from(1) << exponent.unsigned_abs();
    let q: Rational = integer.to_string().parse().unwrap();
    if exponent < 0 { Rational::one() / q } else { q }
}

fn exact(value: &Computable, expected: &Rational, bits: i32) {
    let got: Rational = value.approx(-bits).to_string().parse().unwrap();
    let error = got - expected * pow2(bits);
    assert!(error >= -Rational::one() && error <= Rational::one());
}

fn main() {
    let mut imports = 0;
    let mut roots = 0;
    let mut identities = 0;
    for bits in [64, 256, 1000, 1023, 1024, 1025, 2048, 4096] {
        for sign in [-1, 1] {
            let q = (pow2(bits) + Rational::one()) * Rational::from(sign);
            let r: Real = q.to_string().parse().unwrap();
            assert_eq!(r, Real::new(q.clone()));
            let x = Computable::rational(q.clone());
            for precision in [0, 16, 128, 2048, 16, 0] {
                exact(&x, &q, precision);
                imports += 1;
            }
        }
    }
    for text in ["0.1", "-0.1", "1000000000000.0001", "0.0000000000000000001"] {
        let r: Real = text.parse().unwrap();
        let q: Rational = text.parse().unwrap();
        assert_eq!(r, Real::new(q.clone()));
        let x = Computable::rational(q.clone());
        for bits in [2, 16, 128, 1024, 16, 2] {
            exact(&x, &q, bits);
            imports += 1;
        }
    }
    for offset in 0..8 {
        let x = (offset+1..=offset+32).map(|i| Computable::rational(Rational::from(i*i+1)).sqrt())
            .reduce(|a,b| a.add(b)).unwrap();
        let p = 8192;
        let mut lo = Float::with_val(p, 0);
        let mut hi = Float::with_val(p, 0);
        for i in offset+1..=offset+32 {
            let mut a = Float::with_val(p, i*i+1);
            let mut b = a.clone();
            a.sqrt_round(Round::Down);
            b.sqrt_round(Round::Up);
            lo = Float::with_val_round(p, lo + a, Round::Down).0;
            hi = Float::with_val_round(p, hi + b, Round::Up).0;
        }
        for bits in [2, 32, 256, 1024, 4096, 32, 4096, 2, 256] {
            let got = Float::with_val(p, Integer::from_str_radix(&x.approx(-bits).to_string(),10).unwrap());
            let a = Float::with_val(p, &lo) << bits;
            let b = Float::with_val(p, &hi) << bits;
            assert!(Float::with_val(p, &got-a).abs() <= 1);
            assert!(Float::with_val(p, &got-b).abs() <= 1);
            roots += 1;
        }
    }
    let cases = [
        (Computable::zero().exp(), Rational::one()),
        (Computable::one().ln(), Rational::zero()),
        (Computable::zero().sqrt(), Rational::zero()),
        (Computable::zero().sin(), Rational::zero()),
        (Computable::zero().cos(), Rational::one()),
    ];
    for (x,q) in cases {
        for bits in [0, 2, 32, 256, 2048, 16, 0] {
            exact(&x,&q,bits);
            identities += 1;
        }
    }
    println!("PASS imports/history={imports}; MPFR radical-sum/history={roots}; exact zero/one identities={identities}");
}
