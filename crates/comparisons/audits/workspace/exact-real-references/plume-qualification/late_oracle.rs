use hyperreal::{Computable, Rational};
use rug::{Float,Integer,Rational as Q,float::Round};
use std::{collections::BTreeMap,env,fs};
const PRECISION:u32=4096;
fn integer(s:&str)->Integer {Integer::from_str_radix(s,10).unwrap()}
fn ratio(n:&str,d:&str)->Q {Q::from((integer(n),integer(d)))}
fn round(q:&Q,direction:Round)->Q {Float::with_val_round(PRECISION,q,direction).0.to_rational().unwrap()}
fn logistic_bounds(input:&Q,iterations:usize)->(Q,Q) {
    let (mut lo,mut hi)=(input.clone(),input.clone());
    assert!(lo>=0&&hi<=1);
    for _ in 0..iterations {
        // Dependency-safe interval extension of4*x*(1-x), not endpoint-only
        // evaluation of a nonmonotone polynomial. Round every step outward.
        let lower=lo.clone()*(Q::from(1)-&hi)*4;
        let upper=hi.clone()*(Q::from(1)-&lo)*4;
        lo=round(&lower,Round::Down).max(Q::from(0));
        hi=round(&upper,Round::Up).min(Q::from(1));
        assert!(lo<=hi);
    }
    (lo,hi)
}
fn maximum_bounds()->(Q,Q) {
    let mut lo=Float::with_val(PRECISION,60); let mut hi=lo.clone();
    lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);
    (Q::from(30)-hi.to_rational().unwrap()*2,Q::from(30)-lo.to_rational().unwrap()*2)
}
fn hyper(input:Q,k:usize,bits:i32,progress:bool)->usize {
    let (lo,hi)=logistic_bounds(&input,k);
    let mut x=Computable::rational(input.to_string().parse::<Rational>().unwrap());
    for i in 0..k {
        x=x.clone().multiply(Computable::one().add(x.negate())).multiply(Computable::rational(Rational::from(4)));
        if progress {eprintln!("constructed {}",i+1);}
    }
    for p in [bits,0,bits+64,8] {
        let scale=Integer::from(1)<<p;
        let actual=Q::from((integer(&x.approx(-p).to_string()),scale.clone()));
        let radius=Q::from((1,scale));
        assert!(Q::from(&actual-&radius)<=lo&&Q::from(&actual+&radius)>=hi);
    }
    4
}
fn main() {
    let args:Vec<_>=env::args().collect();
    if args[1]=="selfcheck" {
        let mut checks=0;
        for (a,b) in [(0,1),(1,4),(1,2),(3,4),(1,1),(1,10),(5467,10000)] {
            let input=Q::from((a,b)); let mut exact=input.clone();
            for k in 0..=12 {
                let (lo,hi)=logistic_bounds(&input,k);
                assert!(lo<=exact&&exact<=hi);
                checks+=1;
                // Expanded exact rational polynomial, with no precision cap.
                if k<12 {exact=exact.clone()*4-Q::from(&exact*&exact)*4;}
            }
        }
        println!("PASS directed-interval oracle against exact rational recurrence={checks}");
        return;
    }
    if args[1]=="hyper-grid" {
        let mut checks=0;
        for (a,b) in [(0,1),(1,4),(1,2),(3,4),(1,1),(1,10),(5467,10000)] {
            for k in 0..=3 {checks+=hyper(Q::from((a,b)),k,12,false);}
        }
        for (a,b) in [(0,1),(1,1),(1,10),(5467,10000)] {
            for k in [10,40,60] {checks+=hyper(Q::from((a,b)),k,32,false);}
        }
        println!("PASS Hyper logistic interval/history checks={checks}; 40 input/iteration cases");
        return;
    }
    if args[1]=="hyper" {
        let (input,k,bits)=(ratio(&args[2],&args[3]),args[4].parse::<usize>().unwrap(),args[5].parse::<i32>().unwrap());
        hyper(input,k,bits,true);
        println!("PASS Hyper logistic k={k}, four precision/history queries");
        return;
    }
    let mode=&args[1];
    let mut counts=BTreeMap::<String,(usize,usize)>::new();
    for line in fs::read_to_string(&args[2]).unwrap().lines() {
        let f:Vec<_>=line.split_whitespace().collect();
        let (name,lo,hi,c,r)=if mode=="functional" {
            assert_eq!(f.len(),8);
            let (lo,hi)=match f[0] {
                "quadratic-min"=>(Q::from((-1,4)),Q::from((-1,4))),
                "quadratic-max"=>(Q::from((213,400)),Q::from((213,400))),
                "square-integral"=>(Q::from((1,3)),Q::from((1,3))),
                "reciprocal-max"=>maximum_bounds(), _=>panic!("unknown functional"),
            };
            (f[0],lo,hi,ratio(f[2],f[3]),ratio(f[4],f[5]))
        } else {
            assert_eq!(f.len(),12);
            let (lo,hi)=logistic_bounds(&ratio(f[2],f[3]),f[4].parse().unwrap());
            (f[0],lo,hi,ratio(f[6],f[7]),ratio(f[8],f[9]))
        };
        assert!(r>0);
        let count=counts.entry(name.into()).or_default();count.0+=1;
        if Q::from(&c-&r)>lo||Q::from(&c+&r)<hi {
            count.1+=1;
            if count.1<=3 {println!("FAIL {mode}: {line}; expected interval [{lo},{hi}]");}
        }
    }
    for (name,(n,bad)) in &counts {println!("{name}\tchecks={n}\tfailures={bad}");}
    let (n,bad)=counts.values().fold((0,0),|(n,b),(nn,bb)|(n+nn,b+bb));
    println!("TOTAL {mode}\tchecks={n}\tfailures={bad}");
}
