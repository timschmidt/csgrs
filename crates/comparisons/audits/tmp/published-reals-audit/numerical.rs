use computable_real::{Real as CR, Comparator as CC};
use reals::Real;
use num::{BigInt, BigRational, One, Signed};
use rug::{Float, Integer, float::{Constant, Round}};
use std::collections::BTreeMap;

#[derive(Default)]
struct Tally { counts: BTreeMap<&'static str, (usize, usize)> }
impl Tally {
    fn check(&mut self, name: &'static str, ok: bool, detail: impl FnOnce() -> String) {
        let count = self.counts.entry(name).or_default(); count.0 += 1;
        if !ok { count.1 += 1; if count.1 <= 3 { println!("FAIL {name}: {}", detail()); } }
    }
}
fn rat(n: i64, d: u64) -> BigRational { BigRational::new(n.into(), d.into()) }
fn comp(n: i64, d: u64) -> CR { rat(n,d).into() }
fn exact(t: &mut Tally, name: &'static str, mut c: CR, r: BigRational) {
    for p in [4,1,0,-1,-8,-64,-128,-16] {
        let a = c.appr(p).value;
        let mut s = r.clone();
        if p <= 0 { s *= BigInt::one() << (-p as usize); }
        else { s /= BigInt::one() << (p as usize); }
        let e = (BigRational::from_integer(a.clone()) - s).abs();
        t.check(name, e < BigRational::one(), || format!("x={r}, p={p}, approx={a}, error={e}"));
    }
}
fn interval(t: &mut Tally, name: &'static str, mut c: CR, lo: Float, hi: Float) {
    for p in [4,1,0,-1,-8,-64,-128,-256,-16] {
        let value = c.appr(p).value;
        let a = Float::with_val(1024, Integer::from_str_radix(&value.to_string(),10).unwrap());
        let (mut l,mut h) = (lo.clone(),hi.clone());
        if p <= 0 { l <<= -p; h <<= -p; } else { l >>= p; h >>= p; }
        let e1 = Float::with_val(1024,&a-&l).abs();
        let e2 = Float::with_val(1024,&a-&h).abs();
        t.check(name, e1 < 1 && e2 < 1, || format!("p={p}, approx={value}, errors=[{e1:.5e},{e2:.5e}]"));
    }
}
fn main() {
    let mut t = Tally::default();
    for n in -24..=24 {
        for d in 1..=12 {
            let r=rat(n,d); let s=rat(n+7,d+3);
            exact(&mut t,"ratio",comp(n,d),r.clone());
            exact(&mut t,"negate",-comp(n,d),-r.clone());
            exact(&mut t,"square",comp(n,d).squared(),&r*&r);
            exact(&mut t,"sum",comp(n,d)+comp(n+7,d+3),&r+&s);
            exact(&mut t,"multiply",comp(n,d)*comp(n+7,d+3),&r*&s);
            if n != 0 { exact(&mut t,"inverse",comp(n,d).inv(),r.recip()); }
        }
    }
    for n in -16..=16 {
        for op in ["exp","sin","cos","sqrt","ln","asin","atan"] {
            if (op=="sqrt" && n<0) || (op=="ln" && n<=0) { continue; }
            // Do not exercise the source-identified unbounded negative-asin branch.
            if (op=="asin" && !(-4..=7).contains(&n)) || (op=="atan" && n < -4) { continue; }
            let x:Float=Float::with_val(1024,n)/8i32;
            let (mut lo,mut hi)=(x.clone(),x); let c=comp(n,8);
            let c=match op {
                "exp"=>{lo.exp_round(Round::Down);hi.exp_round(Round::Up);c.exp()},
                "sin"=>{lo.sin_round(Round::Down);hi.sin_round(Round::Up);c.sin()},
                "cos"=>{lo.cos_round(Round::Down);hi.cos_round(Round::Up);c.cos()},
                "sqrt"=>{lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);c.sqrt()},
                "ln"=>{lo.ln_round(Round::Down);hi.ln_round(Round::Up);c.ln()},
                "asin"=>{lo.asin_round(Round::Down);hi.asin_round(Round::Up);c.asin()},
                "atan"=>{lo.atan_round(Round::Down);hi.atan_round(Round::Up);c.atan()},
                _=>unreachable!()
            }; interval(&mut t,op,c,lo,hi);
        }
    }
    interval(&mut t,"pi",CR::pi(),Float::with_val_round(1024,Constant::Pi,Round::Down).0,Float::with_val_round(1024,Constant::Pi,Round::Up).0);
    for bits in [0,2,8,16,24,32,48] {
        let root=BigInt::one() << (bits/2);
        exact(&mut t,"sqrt_magnitude",CR::from(BigInt::one()<<bits).sqrt(),BigRational::from_integer(root));
    }
    // A finite tolerance must not assert a wrong strict mathematical order.
    for n in -12..=12 {
        for d in 1..=12 {
            let r=rat(n,d);
            for p in [0,-4,-16] {
                let mut a:CR=r.clone().into();
                let mut b=comp(n,d)*comp(3,1)/comp(3,1);
                let order=CC::new().abs(p).cmp(&mut a,&mut b);
                t.check("equal_expression_order",order.is_eq(),||format!("x={r}, abs={p}, order={order:?}"));
            }
        }
    }
    for (s,expected) in [("-1.25",rat(-5,4)),("-12.75",rat(-51,4)),("-0.25",rat(-1,4)),("1.25",rat(5,4))] {
        let x:Real=s.parse().unwrap(); let got=BigRational::try_from(x).unwrap();
        t.check("negative_decimal",got==expected,||format!("{s} gives {got} expected {expected}"));
        let h:hyperreal::Rational=s.parse().unwrap();
        let expected_h=hyperreal::Rational::from_bigint_fraction(expected.numer().clone(),expected.denom().to_biguint().unwrap()).unwrap();
        t.check("hyper_negative_decimal",h==expected_h,||format!("{s} gives {h}"));
    }
    t.check("multiple_decimal_points","1.2.3".parse::<CR>().is_err(),||"accepted 1.2.3".into());
    for base in [2,3,5] {
        for exponent in [1,2,3,4,5,6,7,8,10,16] {
            let x=Real::from(base).ln().unwrap()*Real::from(exponent);
            let got=BigRational::try_from(x.exp()).unwrap();
            let expected=BigRational::from_integer(BigInt::from(base).pow(exponent as u32));
            t.check("exp_symbolic_power",got==expected,||format!("exp({exponent}ln({base})) gives {got}, expected {expected}"));
            let x=hyperreal::Real::new(hyperreal::Rational::new(base)).ln().unwrap()*hyperreal::Real::new(hyperreal::Rational::new(exponent));
            t.check("hyper_exp_symbolic_power",x.exp().unwrap().to_f64_lossy()==Some((base as f64).powi(exponent as i32)),||format!("{base}^{exponent}"));
        }
    }
    for op in ["sqrt","exp","ln","atan"] {
        let make=|n| {let x=Real::from(n);match op {"sqrt"=>x.sqrt().unwrap(),"exp"=>x.exp(),"ln"=>x.ln().unwrap(),_=>x.atan()}};
        for sign in [-1,1] {
            let a=make(2)*Real::from(sign);let b=make(3)*Real::from(sign);
            let got=a.partial_cmp(&b);let expected=Some(if sign==1 {std::cmp::Ordering::Less}else{std::cmp::Ordering::Greater});
            t.check("monotonic_multiplier_order",got==expected,||format!("{op}, factor={sign}, got={got:?}, expected={expected:?}"));
            let make_h=|n| {let x=hyperreal::Real::new(hyperreal::Rational::new(n));match op {"sqrt"=>x.sqrt().unwrap(),"exp"=>x.exp().unwrap(),"ln"=>x.ln().unwrap(),_=>x.atan().unwrap()}};
            let a=make_h(2)*hyperreal::Real::new(hyperreal::Rational::new(sign));
            let b=make_h(3)*hyperreal::Real::new(hyperreal::Rational::new(sign));
            t.check("hyper_monotonic_multiplier_order",a.partial_cmp(&b)==expected,||format!("{op}, factor={sign}"));
        }
    }
    for (a,b) in [(2,3),(-2,3),(2,-3)] {
        let left=Real::pi()*Real::from(a);let right=Real::e()*Real::from(b);
        let got=left*right;let mut cr:CR=got.into();let actual:f64=cr.appr(-64).value.to_string().parse::<f64>().unwrap()/2f64.powi(64);
        let expected=(a*b) as f64*std::f64::consts::PI*std::f64::consts::E;
        t.check("product_factor_once",(actual-expected).abs()<1e-12,||format!("{a}pi * {b}e = {actual}, expected {expected}"));
        let got=(hyperreal::Real::pi()*hyperreal::Real::new(hyperreal::Rational::new(a)))*(hyperreal::Real::e()*hyperreal::Real::new(hyperreal::Rational::new(b)));
        t.check("hyper_product_factor_once",(got.to_f64_lossy().unwrap()-expected).abs()<1e-12,||format!("{a}pi * {b}e"));
    }
    for factor in [4,9,16] {
        let x=(Real::from(factor)*Real::from(2).exp()).sqrt().unwrap().ln().unwrap();
        let mut c:CR=x.into();let actual:f64=c.appr(-64).value.to_string().parse::<f64>().unwrap()/2f64.powi(64);
        let expected=1.0+(factor as f64).ln()/2.0;
        t.check("scaled_sqrt_exp_tag",(actual-expected).abs()<1e-12,||format!("ln(sqrt({factor}exp(2))) gives {actual}, expected {expected}"));
        let x=(hyperreal::Real::new(hyperreal::Rational::new(factor))*hyperreal::Real::new(hyperreal::Rational::new(2)).exp().unwrap()).sqrt().unwrap().ln().unwrap();
        t.check("hyper_scaled_sqrt_exp_tag",(x.to_f64_lossy().unwrap()-expected).abs()<1e-12,||format!("factor={factor}"));
        let x=(hyperreal::Real::new(hyperreal::Rational::new(factor))*hyperreal::Real::pi()).sqrt().unwrap();
        let expected=(factor as f64*std::f64::consts::PI).sqrt();
        t.check("hyper_scaled_sqrt_pi",(x.to_f64_lossy().unwrap()-expected).abs()<1e-12,||format!("factor={factor}"));
    }
    for (name,(checks,failures)) in t.counts { println!("RESULT {name}: checks={checks}, failures={failures}"); }
}
