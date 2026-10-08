use hyperreal::{Computable, Rational, Real};
use rug::{Float, Integer, Rational as GmpQ, float::{Constant, Round}};
use std::{collections::BTreeMap, env, fs};
const P: u32 = 8192;
fn integer(s: &str) -> Integer { Integer::from_str_radix(s, 10).unwrap() }
fn reference(name: &str, q: &Float) -> (Float, Float) {
    if name.starts_with("pi/") {
        let k: u32 = name[3..].parse().unwrap();
        let lo = Float::with_val_round(P, Constant::Pi, Round::Down).0 / k;
        let hi = Float::with_val_round(P, Constant::Pi, Round::Up).0 / k;
        return (lo, hi);
    }
    if name == "inv" {
        let d = Float::with_val(P, 2) - q;
        let one=Float::with_val(P,1);
        return (Float::with_val_round(P, &one/&d, Round::Down).0,
                Float::with_val_round(P, &one/&d, Round::Up).0);
    }
    if name == "mlni" && q == &0 { return (Float::with_val(P,0.5),Float::with_val(P,0.5)); }
    let mut lo = Float::with_val(P, q / 2);
    let mut hi = lo.clone();
    match name {
        "mexp" => {lo.exp_round(Round::Down); hi.exp_round(Round::Up);}
        "msin" => {lo.sin_round(Round::Down); hi.sin_round(Round::Up);}
        "mcos" => {lo.cos_round(Round::Down); hi.cos_round(Round::Up);}
        "marctan" => {lo.atan_round(Round::Down); hi.atan_round(Round::Up);}
        "marcsin" => {lo.asin_round(Round::Down); hi.asin_round(Round::Up);}
        "mln" | "mlni" => {lo.ln_1p_round(Round::Down); hi.ln_1p_round(Round::Up);}
        _ => panic!("unknown {name}"),
    }
    if name == "mlni" {
        if q < &0 {std::mem::swap(&mut lo,&mut hi);}
        (Float::with_val_round(P,&lo/q,Round::Down).0,
         Float::with_val_round(P,&hi/q,Round::Up).0)
    } else if name == "mln" { (lo,hi) }
    else { (lo/2,hi/2) }
}
fn rational(n: &str, d: &str) -> Computable {
    Computable::rational(format!("{n}/{d}").parse().unwrap())
}
fn hyper(name: &str, n: &str, d: &str) -> Computable {
    let x=rational(n,d); let half=rational("1","2");
    if name.starts_with("pi/") {return Computable::pi().multiply(rational("1",&name[3..]));}
    if name == "inv" {return rational("2","1").add(x.negate()).inverse();}
    let y=x.clone().multiply(half.clone());
    match name {
        "mexp" => y.exp().multiply(half), "msin" => y.sin().multiply(half),
        "mcos" => y.cos().multiply(half), "marctan" => y.atan().multiply(half),
        "marcsin" => y.asin().multiply(half),
        "mln" => y.add(Computable::one()).ln(),
        "mlni" if n=="0" => half,
        "mlni" => y.add(Computable::one()).ln().multiply(x.inverse()),
        _ => panic!("unknown {name}"),
    }
}
fn enclosed(got: &Float, lo: &Float, hi: &Float, bits: i32) -> bool {
    let e=Float::with_val(P,1)>>bits;
    let l=Float::with_val_round(P,hi-&e,Round::Up).0;
    let h=Float::with_val_round(P,lo+&e,Round::Down).0;
    got>=&l && got<=&h
}
fn exact(x: &Computable, q: &GmpQ) -> usize {
    for bits in [0,8,32,64,128,1100,8] {
        let got=GmpQ::from(integer(&x.approx(-bits).to_string()));
        let truth=GmpQ::from(q * (Integer::from(1)<<bits));
        assert!((got-truth).abs()<=1);
    }
    7
}
fn main() {
    let args:Vec<_>=env::args().collect();
    let mode=&args[1];
    if mode=="decimal" {
        let raw=fs::read_to_string(&args[2]).unwrap();
        let s=raw.trim().trim_matches('"'); assert_eq!(s.len(),500);
        let ten=Integer::from(10).pow(498);
        let digits=integer(&s.replace('.',""));
        let pi_lo=Float::with_val_round(P,Constant::Pi,Round::Down).0;
        let pi_hi=Float::with_val_round(P,Constant::Pi,Round::Up).0;
        let lo=Float::with_val_round(P,&pi_lo * &ten,Round::Down).0;
        let hi=Float::with_val_round(P,&pi_hi * &ten,Round::Up).0;
        assert_eq!(lo.to_integer_round(Round::Down).unwrap().0,digits);
        assert_eq!(hi.to_integer_round(Round::Down).unwrap().0,digits);
        println!("PASS all 498 native pi decimal digits against directed8192-bit MPFR");
        return;
    }
    let mut counts=BTreeMap::<String,(usize,usize)>::new();
    for line in fs::read_to_string(&args[2]).unwrap().lines() {
        let f:Vec<_>=line.split('\t').collect(); assert_eq!(f.len(),6);
        let q=Float::with_val(P,integer(f[1]))/integer(f[2]);
        let (lo,hi)=reference(f[0],&q);
        let bits:i32=f[3].parse().unwrap();
        let mut outcomes=Vec::new();
        if mode=="hyper" {
            let x=hyper(f[0],f[1],f[2]);
            for p in [bits,2,bits,0] {
                let got=Float::with_val(P,integer(&x.approx(-p).to_string()))>>p;
                outcomes.push((p,got));
            }
        } else {
            outcomes.push((bits,Float::with_val(P,integer(f[4]))/integer(f[5])));
        }
        for (bits,got) in outcomes {
            let entry=counts.entry(f[0].into()).or_default();entry.0+=1;
            if !enclosed(&got,&lo,&hi,bits) {
                entry.1+=1;
                if entry.1<=2 {println!("FAIL {} {}/{} bits={bits} got={} reference={}",f[0],f[1],f[2],got.to_f64(),lo.to_f64());}
            }
        }
    }
    for (name,(n,bad)) in &counts {println!("{name}\tchecks={n}\tfailures={bad}");}
    if mode=="hyper" {
        assert!(counts.values().all(|(_,bad)|*bad==0));
        let mut controls=exact(&rational("1","2").multiply(rational("3","4")),&GmpQ::from((3,8)));
        for (a,b) in [(0,1),(1,1024),(1,8),(1,4),(1,2),(3,4),(1,1)] {
            controls+=exact(&rational(&(a*a).to_string(),&(b*b).to_string()).sqrt(),&GmpQ::from((a,b)));
        }
        for exponent in [0,-1,-8,-52,-53,-54,-55,-56,-64,-128,-1022,-1074] {
            for sign in [-1.0,1.0] {
                let d=sign * if exponent>=-1022 {2.0_f64.powi(exponent)} else {f64::from_bits(1)};
                let q=GmpQ::from_f64(d).unwrap();
                controls+=exact(&Computable::rational(Rational::try_from(d).unwrap()),&q);
            }
        }
        for d in [0.0,-0.0] {controls+=exact(&Computable::rational(Rational::try_from(d).unwrap()),&GmpQ::from(0));}
        let a=Real::from("-1/4".parse::<Rational>().unwrap());
        let b=Real::from("-3/8".parse::<Rational>().unwrap());
        assert_eq!(a.min(&b),&b);assert_eq!(a.max(&b),&a);
        println!("PASS matching exact product/root/float/history={controls}; min/max=2");
    }
}
use rug::ops::Pow;
