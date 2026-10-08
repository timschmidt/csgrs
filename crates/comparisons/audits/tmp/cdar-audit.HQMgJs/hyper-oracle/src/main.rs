use hyperreal::{Computable, Rational, Real};
use num_bigint::{BigInt, BigUint};
use rug::{Float, Integer, Rational as Q, float::{Constant, Round}};

fn hyper_q(q: &Q) -> Computable {
    let n: BigInt = q.numer().to_string().parse().unwrap();
    let d: BigUint = q.denom().to_string().parse().unwrap();
    Computable::rational(Rational::from_bigint_fraction(n, d).unwrap())
}

fn apply(op: &str, x: Computable) -> Computable {
    match op {
        "exp" => x.exp(), "log" => x.ln(), "sin" => x.sin(), "cos" => x.cos(),
        "sqrt" => x.sqrt(), "atan" => x.atan(), "asin" => x.asin(), "acos" => x.acos(),
        _ => unreachable!(),
    }
}

fn reference(op: &str, q: &Q, round: Round) -> Float {
    let (mut x, direction) = Float::with_val_round(2048, q, Round::Nearest);
    assert_eq!(direction, std::cmp::Ordering::Equal, "input must be exact at oracle precision");
    match op {
        "exp" => { x.exp_round(round); }, "log" => { x.ln_round(round); },
        "sin" => { x.sin_round(round); }, "cos" => { x.cos_round(round); },
        "sqrt" => { x.sqrt_round(round); }, "atan" => { x.atan_round(round); },
        "asin" => { x.asin_round(round); }, "acos" => { x.acos_round(round); },
        _ => unreachable!(),
    }
    x
}

fn check(op: &str, q: &Q, value: &Computable, p: i32, mut lo: Float, mut hi: Float) {
    let result = value.approx(p);
    let a = Integer::from_str_radix(&result.to_string(), 10).unwrap();
    let a_lo = Integer::from(&a - 1);
    let a_hi = Integer::from(&a + 1);
    lo >>= p;
    hi >>= p;
    // The interval is (result +/- 1)*2^p. Endpoints are exact integers here.
    assert!(lo >= a_lo && hi <= a_hi, "{op}({q}) p={p}: result={result}, reference=[{lo},{hi}]");
}

fn main() {
    let mut points: Vec<Q> = (-64..=64).map(|n| Q::from((n,8))).collect();
    points.extend([Q::from(-128), Q::from(128)]);
    for k in [5_u32,30,80,256] {
        let eps = Q::from((Integer::from(1),Integer::from(1)<<k));
        points.push(eps.clone()); points.push(-eps.clone());
        points.push(Q::from(1)-&eps); points.push(Q::from(1)+&eps);
        points.push(Q::from(-1)-&eps); points.push(Q::from(-1)+&eps);
    }
    for k in [8_u32,32,64,128] {
        points.push(Q::from(Integer::from(1)<<k));
        points.push(Q::from(-(Integer::from(1)<<k)));
    }
    points.sort(); points.dedup();
    let mut total = 0;
    for op in ["exp","log","sin","cos","sqrt","atan","asin","acos"] {
        let mut count = 0;
        for q in &points {
            if op=="log" && q<=&0 { continue; }
            if op=="sqrt" && q<&0 { continue; }
            if (op=="asin" || op=="acos") && (q<&(-1) || q>&1) { continue; }
            if op=="exp" && (q<&(-128) || q>&128) { continue; }
            let c = apply(op, hyper_q(q));
            let lo=reference(op,q,Round::Down);
            let hi=reference(op,q,Round::Up);
            for p in [0,-1,-20,-100,-300,-30] {
                check(op,q,&c,p,lo.clone(),hi.clone());
                count+=1;
            }
        }
        total+=count;
        println!("{op}: {count} directed-MPFR unit-error checks PASS");
    }
    let pi=Computable::pi();
    for p in [0,-1,-20,-100,-300,-30] {
        let lo=Float::with_val_round(2048,Constant::Pi,Round::Down).0;
        let hi=Float::with_val_round(2048,Constant::Pi,Round::Up).0;
        check("pi",&Q::from(0),&pi,p,lo,hi);
        total+=1;
    }
    for x in [f64::INFINITY,f64::NEG_INFINITY,f64::NAN] {
        assert!(Rational::try_from(x).is_err());
        assert!(Real::try_from(x).is_err());
    }
    for x in [0.0,-0.0,f64::from_bits(1),f64::MIN_POSITIVE,f64::MAX,-f64::MAX] {
        assert!(Rational::try_from(x).is_ok());
        assert!(Real::try_from(x).is_ok());
    }
    println!("{total} approximation checks PASS; 18 finite/nonfinite conversion checks PASS");
}
