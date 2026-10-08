use hyperreal::{Computable, Rational};
use rug::{Float, Integer, Rational as Q, float::{Constant, Round}};
use std::{collections::BTreeMap, env, fs};
const P: u32 = 8192;
fn int(s: &str) -> Integer { Integer::from_str_radix(s, 10).unwrap() }
fn frac(n: &str,d: &str) -> Q { Q::from((int(n),int(d))) }
fn reference(op: &str, q: &Q) -> (Float,Float) {
    let x=Float::with_val(P,q);
    assert_eq!(x.to_rational().unwrap(),*q,"oracle input must be exact dyadic");
    let mut lo=x.clone(); let mut hi=x;
    match op {
        "exp" => {lo.exp_round(Round::Down);hi.exp_round(Round::Up);},
        "ln" => {lo.ln_round(Round::Down);hi.ln_round(Round::Up);},
        "sin" => {lo.sin_round(Round::Down);hi.sin_round(Round::Up);},
        "cos" => {lo.cos_round(Round::Down);hi.cos_round(Round::Up);},
        "atan" => {lo.atan_round(Round::Down);hi.atan_round(Round::Up);},
        "sqrt" => {lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);},
        "pi" => {lo=Float::with_val_round(P,Constant::Pi,Round::Down).0;
                 hi=Float::with_val_round(P,Constant::Pi,Round::Up).0;},
        _ => panic!("unknown operation {op}"),
    }
    (lo,hi)
}
fn enclosed(c: &Q,r: &Q,lo: &Float,hi: &Float) -> bool {
    let lower=Q::from(c-r); let upper=Q::from(c+r);
    Float::with_val_round(P,&lower,Round::Up).0<=*lo &&
    Float::with_val_round(P,&upper,Round::Down).0>=*hi
}
fn hyper(op: &str,q: &Q) -> Computable {
    let x=Computable::rational(q.to_string().parse::<Rational>().unwrap());
    match op {"exp"=>x.exp(),"ln"=>x.ln(),"sin"=>x.sin(),"cos"=>x.cos(),
              "atan"=>x.atan(),"sqrt"=>x.sqrt(),"pi"=>Computable::pi(),_=>panic!()}
}
fn exact(x: &Computable,q: &Q) -> usize {
    for bits in [0,8,32,128,512,8] {
        let got=Q::from(int(&x.approx(-bits).to_string()));
        let truth=Q::from(q*(Integer::from(1)<<bits));
        assert!((got-truth).abs()<=1);
    }
    6
}
fn controls() -> usize {
    let mut n=0;
    for a in -16..=16 {for b in -16..=16 {
        let q=Q::from((a,16));let r=Q::from((b,16));
        let x=Computable::rational(q.to_string().parse().unwrap());
        let y=Computable::rational(r.to_string().parse().unwrap());
        n+=exact(&x.clone().add(y.clone()),&Q::from(&q+&r));
        n+=exact(&x.clone().add(y.clone().negate()),&Q::from(&q-&r));
        n+=exact(&x.clone().multiply(y.clone()),&Q::from(&q*&r));
        if b!=0 {n+=exact(&x.multiply(y.inverse()),&Q::from(&q/&r));}
    }}
    for e in [249u32,250,500,1024] {
        let zero=Computable::rational(Rational::from(0));
        let huge: Rational=(Integer::from(1)<<e).to_string().parse().unwrap();
        n+=exact(&zero.multiply(Computable::rational(huge)),&Q::from(0));
    }
    for d in [i64::MIN,i64::MAX,1i64<<62,(1i64<<62)+1,-((1i64<<62)+1)] {
        for a in -4..=4 {
            let q=Q::from((a,4));let r=Q::from(d);
            let x=Computable::rational(q.to_string().parse().unwrap());
            let y=Computable::rational(r.to_string().parse().unwrap());
            n+=exact(&x.multiply(y.inverse()),&Q::from(&q/&r));
        }
    }
    n
}
fn main() {
    let args:Vec<_>=env::args().collect();
    let mode=&args[1];
    let mut counts=BTreeMap::<String,(usize,usize)>::new();
    for line in fs::read_to_string(&args[2]).unwrap().lines() {
        let f:Vec<_>=line.split_whitespace().collect(); assert_eq!(f.len(),8);
        let q=frac(f[1],f[2]); let bits:i32=f[3].parse().unwrap();
        let (lo,hi)=reference(f[0],&q);
        let outputs=if mode=="hyper" {
            let x=hyper(f[0],&q);
            [bits,2,bits,0].into_iter().map(|p|{
                let scale=Integer::from(1)<<p;
                (Q::from((int(&x.approx(-p).to_string()),scale.clone())),Q::from((1,scale)))
            }).collect::<Vec<_>>()
        } else {vec![(frac(f[4],f[5]),frac(f[6],f[7]))]};
        for (c,r) in outputs {
            let count=counts.entry(f[0].into()).or_default();count.0+=1;
            if !enclosed(&c,&r,&lo,&hi) {
                count.1+=1;
                if count.1<=4 {println!("FAIL {} {}/{} bits={} got={} expected={}",f[0],f[1],f[2],bits,c.to_f64(),lo.to_f64());}
            }
        }
    }
    for (op,(n,b)) in &counts {println!("{op}\tchecks={n}\tfailures={b}");}
    let (n,b)=counts.values().fold((0,0),|(n,b),(x,y)|(n+x,b+y));
    println!("TOTAL\tchecks={n}\tfailures={b}");
    if mode=="hyper" {assert_eq!(b,0);println!("PASS exact Hyper scalar/history controls={}",controls());}
}
