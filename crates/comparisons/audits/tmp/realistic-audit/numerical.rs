use num::{BigInt, BigRational, BigUint, One, Signed};
use realistic::{Computable, Rational, Real, Simple};
use rug::{Float, Integer, float::{Constant, Round}};
use std::collections::{BTreeMap, HashMap};
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};

#[derive(Default)]
struct Tally { counts: BTreeMap<&'static str, (usize, usize)> }
impl Tally {
    fn check(&mut self, group: &'static str, ok: bool, detail: impl FnOnce() -> String) {
        let count = self.counts.entry(group).or_default();
        count.0 += 1;
        if !ok {
            count.1 += 1;
            if count.1 <= 4 { println!("FAIL {group}: {}", detail()); }
        }
    }
}

fn rat(n: i64, d: u64) -> Rational { Rational::fraction(n, d).unwrap() }
fn comp(n: i64, d: u64) -> Computable { Computable::rational(rat(n, d)) }

fn exact_check(t: &mut Tally, name: &'static str, c: Computable, r: BigRational) {
    for p in [4, 1, 0, -1, -8, -64, -128, -16] {
        let actual = c.approx(p);
        let mut scaled = r.clone();
        if p <= 0 { scaled *= BigInt::one() << (-p as usize); }
        else { scaled /= BigInt::one() << (p as usize); }
        let error = (BigRational::from_integer(actual.clone()) - scaled).abs();
        t.check(name, error <= BigRational::one(), || format!("value={r}, p={p}, approx={actual}, error={error}"));
    }
}

fn interval_check(t: &mut Tally, name: &'static str, c: Computable, low: Float, high: Float) {
    for p in [4, 1, 0, -1, -8, -64, -128, -256, -16] {
        let actual = c.approx(p);
        let a = Float::with_val(1024, Integer::from_str_radix(&actual.to_string(), 10).unwrap());
        let mut lo = low.clone();
        let mut hi = high.clone();
        if p <= 0 { lo <<= -p; hi <<= -p; } else { lo >>= p; hi >>= p; }
        let e1 = Float::with_val(1024, &a - &lo).abs();
        let e2 = Float::with_val(1024, &a - &hi).abs();
        t.check(name, e1 <= 1 && e2 <= 1, || format!("p={p}, approx={actual}, errors=[{e1:.6e},{e2:.6e}]"));
    }
}

fn main() {
    let mut t = Tally::default();
    for n in -32_i64..=32 {
        for d in 1_u64..=16 {
            let r = BigRational::new(n.into(), d.into());
            exact_check(&mut t, "ratio", comp(n,d), r.clone());
            exact_check(&mut t, "negate", comp(n,d).negate(), -r.clone());
            exact_check(&mut t, "square", comp(n,d).square(), &r * &r);
            let s = BigRational::new((n+7).into(), (d+3).into());
            exact_check(&mut t, "add", comp(n,d).add(comp(n+7,d+3)), &r + &s);
            exact_check(&mut t, "multiply", comp(n,d).multiply(comp(n+7,d+3)), &r * &s);
            if n != 0 { exact_check(&mut t, "inverse", comp(n,d).inverse(), r.recip()); }
        }
    }
    for n in -32_i64..=32 {
        for op in ["exp", "sin", "cos", "sqrt", "ln"] {
            if (op == "sqrt" && n < 0) || (op == "ln" && n <= 0) { continue; }
            let x: Float = Float::with_val(1024, n) / 8i32;
            let mut lo = x.clone();
            let mut hi = x;
            let c = comp(n,8);
            let c = match op {
                "exp" => {lo.exp_round(Round::Down); hi.exp_round(Round::Up); c.exp()},
                "sin" => {lo.sin_round(Round::Down); hi.sin_round(Round::Up); c.sin()},
                "cos" => {lo.cos_round(Round::Down); hi.cos_round(Round::Up); c.cos()},
                "sqrt" => {lo.sqrt_round(Round::Down); hi.sqrt_round(Round::Up); c.sqrt()},
                "ln" => {lo.ln_round(Round::Down); hi.ln_round(Round::Up); c.ln()},
                _ => unreachable!(),
            };
            interval_check(&mut t, op, c, lo, hi);
        }
    }
    interval_check(&mut t, "pi", Computable::pi(),
        Float::with_val_round(1024, Constant::Pi, Round::Down).0,
        Float::with_val_round(1024, Constant::Pi, Round::Up).0);

    for n in -20..=20 {
        let angle = Real::new(rat(n,6)) * Real::pi();
        let ha = hyperreal::Real::new(hyperreal::Rational::fraction(n,6).unwrap()) * hyperreal::Real::pi();
        // Exact sine values at integer multiples of pi/6.
        if let Some(expected) = match n.rem_euclid(12) {0|6=>Some(0.0),1|5=>Some(0.5),3=>Some(1.0),7|11=>Some(-0.5),9=>Some(-1.0),_=>None} {
            let actual = f64::from(angle.sin());
            t.check("real_sin_pi", actual == expected, || format!("n={n}/6, actual={actual}, expected={expected}"));
            t.check("hyper_sin_pi", ha.sin().to_f64_lossy() == Some(expected), || format!("n={n}/6"));
        }
    }
    for n in [-1,1] {
        let value = rat(n,2).trunc();
        t.check("canonical_trunc", value == Rational::zero(), || format!("n={n}, value={value:?}"));
    }
    for s in ["2/2", "6/3", "-8/2"] {
        let r: Rational = s.parse().unwrap();
        t.check("parsed_integer", r.is_integer(), || s.to_owned());
    }
    let mut r = rat(2,3); r *= rat(3,2);
    t.check("assign_canonical", r.is_integer(), || format!("{r:?}"));
    t.check("negative_zero_power", Rational::zero().powi((-1).into()).is_err(), || "zero^-1 accepted".to_owned());
    let r: Rational = "0.1_2".parse().unwrap();
    t.check("decimal_underscore", r == rat(12,100), || format!("actual={r}"));
    let x = Real::pi().sqrt().unwrap();
    let z = x.clone() - x;
    t.check("unknown_inequality", !z.definitely_not_equal(&Real::zero()), || "sqrt(pi)-sqrt(pi) declared unequal to zero".to_owned());

    // A finite, ordinary approximation request before and after clearing cancellation.
    for op in ["sqrt", "exp", "cos", "ln", "pi"] {
        let make = || match op {"sqrt"=>comp(2,1).sqrt(),"exp"=>comp(1,4).exp(),"cos"=>comp(1,4).cos(),"ln"=>comp(5,4).ln(),_=>Computable::pi()};
        let value = make();
        let flag = Arc::new(AtomicBool::new(true));
        let signal = Some(flag.clone());
        let interrupted = value.approx_signal(&signal,-128);
        flag.store(false,Ordering::Relaxed);
        let resumed = value.approx_signal(&signal,-128);
        let expected = make().approx(-128);
        t.check("resume_cache", (&resumed-&expected).abs() <= BigInt::from(2), || format!("op={op}, interrupted={interrupted}, resumed={resumed}, fresh={expected}"));
    }
    for bits in [24usize,53usize] {
        for numerator in [1u32,2,3,5,6,7] {
            // quarter-ulp increments around one, including exact halfway values.
            let denom = BigUint::one() << (bits+1);
            let n = BigInt::from(denom.clone()) + BigInt::from(numerator);
            let rr = Rational::from_bigint_fraction(n.clone(),denom.clone()).unwrap();
            let hr = hyperreal::Real::new(hyperreal::Rational::from_bigint_fraction(n.clone(),denom.clone()).unwrap());
            let exact = rug::Rational::from((Integer::from_str_radix(&n.to_string(),10).unwrap(),Integer::from_str_radix(&denom.to_string(),10).unwrap()));
            let f = Float::with_val(256,exact);
            if bits == 24 {
                let actual = f32::from(Real::new(rr)); let expected = f.to_f32();
                t.check("f32_nearest", actual.to_bits()==expected.to_bits(), || format!("quarter ulps={numerator}, actual={:x}, expected={:x}",actual.to_bits(),expected.to_bits()));
                t.check("hyper_f32_nearest", f32::from(hr).to_bits()==expected.to_bits(), || format!("quarter ulps={numerator}"));
            } else {
                let actual = f64::from(Real::new(rr)); let expected = f.to_f64();
                t.check("f64_nearest", actual.to_bits()==expected.to_bits(), || format!("quarter ulps={numerator}, actual={:x}, expected={:x}",actual.to_bits(),expected.to_bits()));
                t.check("hyper_f64_nearest", hr.to_f64_lossy().unwrap().to_bits()==expected.to_bits(), || format!("quarter ulps={numerator}"));
            }
        }
    }
    for (s, valid) in [("(log10 100)",true),("(+ 1 2) trailing",false)] {
        let parsed = s.parse::<Simple>();
        t.check("parser",parsed.is_ok()==valid,|| format!("input={s:?}, parsed={parsed:?}"));
        if let Ok(p) = parsed { let _ = p.evaluate(&HashMap::new()); }
    }
    for (name,(checks,failures)) in t.counts { println!("RESULT {name}: checks={checks}, failures={failures}"); }
}
