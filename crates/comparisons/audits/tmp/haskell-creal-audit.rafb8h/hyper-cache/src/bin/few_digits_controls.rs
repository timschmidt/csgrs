use hyperreal::{Computable, Rational};
use rug::{Float, Integer, float::Round};
use std::{env, fs};

fn apply(x: Computable, name: &str) -> Computable {
    match name {
        "sqrt" => x.sqrt(), "exp" => x.exp(), "ln" => x.ln(),
        "sin" => x.sin(), "cos" => x.cos(), "asin" => x.asin(),
        "acos" => x.acos(), "atan" => x.atan(),
        // Computable has no named sinh/cosh; exercise their defining tower formulas.
        "sinh" => x.clone().exp().add(x.negate().exp().negate())
            .multiply(Computable::rational("1/2".parse().unwrap())),
        "cosh" => x.clone().exp().add(x.negate().exp())
            .multiply(Computable::rational("1/2".parse().unwrap())),
        "asinh" => x.asinh(), "acosh" => x.acosh(),
        "atanh" => x.atanh(), "erf" => x.erf(), _ => panic!("unknown operation"),
    }
}
fn mpfr(x: &mut Float, name: &str, round: Round) {
    match name {
        "sqrt" => { x.sqrt_round(round); }, "exp" => { x.exp_round(round); },
        "ln" => { x.ln_round(round); }, "sin" => { x.sin_round(round); },
        "cos" => { x.cos_round(round); }, "asin" => { x.asin_round(round); },
        "acos" => { x.acos_round(round); }, "atan" => { x.atan_round(round); },
        "sinh" => { x.sinh_round(round); }, "cosh" => { x.cosh_round(round); },
        "asinh" => { x.asinh_round(round); }, "acosh" => { x.acosh_round(round); },
        "atanh" => { x.atanh_round(round); }, "erf" => { x.erf_round(round); },
        _ => panic!("unknown operation"),
    }
}
fn elementary(name: &str, n: &str, d: &str, bits: i32) {
    let input: Rational = format!("{n}/{d}").parse().unwrap();
    let x = apply(Computable::rational(input), name);
    let p = 2048;
    let mut lo = Float::with_val(p, Integer::from_str_radix(n, 10).unwrap())
        / Integer::from_str_radix(d, 10).unwrap();
    let mut hi = lo.clone();
    mpfr(&mut lo, name, Round::Down);
    mpfr(&mut hi, name, Round::Up);
    for request in [bits, 2, bits, 0] {
        let got = Float::with_val(p, Integer::from_str_radix(&x.approx(-request).to_string(), 10).unwrap());
        assert!(Float::with_val(p, &got - Float::with_val(p, &lo << request)).abs() <= 1, "{name} {n}/{d} {request}");
        assert!(Float::with_val(p, &got - Float::with_val(p, &hi << request)).abs() <= 1, "{name} {n}/{d} {request}");
    }
}
fn exact(x: &Computable, q: &Rational) -> usize {
    for bits in [0, 8, 16, 32, 64, 128, 1024, 8] {
        let got: Rational = x.approx(-bits).to_string().parse().unwrap();
        let scale: Rational = (Integer::from(1) << bits).to_string().parse().unwrap();
        let error = got - q * scale;
        assert!(error >= -Rational::one() && error <= Rational::one());
    }
    8
}
fn main() {
    let mut interior = 0;
    for line in fs::read_to_string(env::args().nth(1).unwrap()).unwrap().lines() {
        let fields: Vec<_> = line.split('\t').collect();
        elementary(fields[0], fields[1], fields[2], fields[3].parse().unwrap());
        interior += 4;
    }
    let mut endpoints = 0;
    for (op, n) in [("sqrt", "0"), ("asin", "1"), ("asin", "-1"),
        ("acos", "1"), ("acos", "-1"), ("acosh", "1")] {
        elementary(op, n, "1", 1024);
        endpoints += 4;
    }
    let root = Computable::rational("5/4".parse().unwrap()).add(Computable::one())
        .multiply(Computable::rational("1/16".parse().unwrap())).sqrt();
    let mut identities = exact(&Computable::zero().cos(), &Rational::one());
    identities += exact(&root, &"3/8".parse().unwrap());
    identities += exact(&root.clone().multiply(root.clone()), &"9/64".parse().unwrap());
    identities += exact(&root.multiply(Computable::one()), &"3/8".parse().unwrap());
    println!("PASS directed-MPFR interior/history={interior}; endpoint/history={endpoints}; exact wrapper-witness/history={identities}");
}
