use hyperreal::{Computable, Rational, Real, RealSign};
use rug::{Float, Integer, Rational as Q, float::Round};
use std::{collections::BTreeMap, env, fs};
const P:u32=4096;
fn int(s:&str)->Integer {Integer::from_str_radix(s,10).unwrap()}
fn ratio(n:&str,d:&str)->Q {Q::from((int(n),int(d)))}
fn reference(op:&str,q:&Q)->(Q,Q) {
    let x=Float::with_val(P,q);
    assert_eq!(x.to_rational().unwrap(),*q,"oracle requires exactly represented input");
    let (mut lo,mut hi)=(x.clone(),x);
    match op {
        "sqrt"=>{lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);},
        "exp"=>{lo.exp_round(Round::Down);hi.exp_round(Round::Up);},
        "log"=>{lo.ln_round(Round::Down);hi.ln_round(Round::Up);},
        "sin"=>{lo.sin_round(Round::Down);hi.sin_round(Round::Up);},
        "cos"=>{lo.cos_round(Round::Down);hi.cos_round(Round::Up);},
        "atan"=>{lo.atan_round(Round::Down);hi.atan_round(Round::Up);},
        "asin"=>{lo.asin_round(Round::Down);hi.asin_round(Round::Up);},
        "acos"=>{lo.acos_round(Round::Down);hi.acos_round(Round::Up);},
        "asinh"=>{lo.asinh_round(Round::Down);hi.asinh_round(Round::Up);},
        "acosh"=>{lo.acosh_round(Round::Down);hi.acosh_round(Round::Up);},
        "atanh"=>{lo.atanh_round(Round::Down);hi.atanh_round(Round::Up);},
        _=>panic!("unknown operation"),
    }
    let bounds=(lo.to_rational().unwrap(),hi.to_rational().unwrap());
    assert!(bounds.0<=bounds.1);
    bounds
}
fn hyper(op:&str,q:&Q)->Computable {
    let x=Computable::rational(q.to_string().parse::<Rational>().unwrap());
    match op {"sqrt"=>x.sqrt(),"exp"=>x.exp(),"log"=>x.ln(),"sin"=>x.sin(),"cos"=>x.cos(),
        "atan"=>x.atan(),"asin"=>x.asin(),"acos"=>x.acos(),"asinh"=>x.asinh(),
        "acosh"=>x.acosh(),"atanh"=>x.atanh(),_=>panic!()}
}
fn selfcheck() {
    let mut checks=0;
    for (op,a,b) in [("exp",0,1),("log",1,0),("sin",0,0),("cos",0,1),("atan",0,0),
                    ("asin",0,0),("acos",1,0),("asinh",0,0),("acosh",1,0),("atanh",0,0)] {
        assert_eq!(reference(op,&Q::from(a)),(Q::from(b),Q::from(b)));checks+=1;
    }
    for n in 0..=32 {let r=Q::from((n,4));let q=Q::from(&r*&r);
        assert_eq!(reference("sqrt",&q),(r.clone(),r));checks+=1;}
    println!("PASS oracle exact identities={checks}");
}
fn exact_controls() {
    let mut checks=0;
    for k in [32,64,136,137,138,140,160,256,4096] {for s in [-1,1] {
        let q=Q::from((s,Integer::from(1)<<k));
        let x=Computable::rational(q.to_string().parse().unwrap());
        assert_eq!(x.sign_until(0),Some(if s<0 {RealSign::Negative}else{RealSign::Positive}));
        checks+=1;
        for p in [0,32,k+64,8] {
            let a=Q::from((int(&x.approx(-p).to_string()),Integer::from(1)<<p));
            assert!((a-&q).abs()<=Q::from((1,Integer::from(1)<<p)));checks+=1;
        }
    }}
    for n in -64..=64 {
        let x=Real::new(Rational::fraction(n,8).unwrap());
        assert_eq!(x.trunc_certified().unwrap().to_string(),(n/8).to_string());checks+=1;
    }
    println!("PASS Hyper tiny-sign/cache-history/certified-truncation controls={checks}");
}
fn opaque_atan(n:i32,scale:i32,bits:i32,warm:bool) {
    let mut lo=Float::with_val(P,n);let mut hi=lo.clone();
    lo.sin_round(Round::Down);hi.sin_round(Round::Up);
    assert!(scale>0);
    lo=Float::with_val_round(P,lo.to_rational().unwrap()*scale,Round::Down).0;
    hi=Float::with_val_round(P,hi.to_rational().unwrap()*scale,Round::Up).0;
    lo.atan_round(Round::Down);hi.atan_round(Round::Up);
    let input=Computable::rational(Rational::from(n)).sin()
        .multiply(Computable::rational(Rational::from(scale)));
    if warm {let _=input.approx(-64);}
    let x=input.atan();
    eprintln!("opaque atan constructed n={n} scale={scale} bits={bits} warm={warm}");
    let a=Q::from((int(&x.approx(-bits).to_string()),Integer::from(1)<<bits));
    let r=Q::from((1,Integer::from(1)<<bits));
    assert!(Q::from(&a-&r)<=lo.to_rational().unwrap()&&Q::from(&a+&r)>=hi.to_rational().unwrap());
    println!("PASS opaque atan n={n} scale={scale} bits={bits} warm={warm}");
}
fn main() {
    let args:Vec<_>=env::args().collect();
    if args[1]=="selfcheck" {selfcheck();return;}
    if args[1]=="exact-controls" {exact_controls();return;}
    if args[1]=="opaque-atan" {opaque_atan(args[2].parse().unwrap(),args[3].parse().unwrap(),args[4].parse().unwrap(),args.get(5).is_some_and(|x|x=="warm"));return;}
    let mode=&args[1];
    let mut counts=BTreeMap::<String,(usize,usize)>::new();
    for line in fs::read_to_string(&args[2]).unwrap().lines() {
        let f:Vec<_>=line.split_whitespace().collect();
        assert_eq!(f.len(),if mode=="fixed" {6}else{5});
        let q=ratio(f[1],f[2]);let bits:i32=f[3].parse().unwrap();
        let (lo,hi)=reference(f[0],&q);
        let outputs=if mode=="hyper" {
            let x=hyper(f[0],&q);
            [bits,0,bits+64,8].into_iter().map(|p|{
                let scale=Integer::from(1)<<p;
                (Q::from((int(&x.approx(-p).to_string()),scale.clone())),Q::from((1,scale)))
            }).collect::<Vec<_>>()
        } else {
            let scale=Integer::from(1)<<bits;
            let c=if mode=="fixed" {ratio(f[4],f[5])}else{Q::from((int(f[4]),scale.clone()))};
            vec![(c,Q::from((1,scale)))]
        };
        for (c,r) in outputs {
            let count=counts.entry(f[0].into()).or_default();count.0+=1;
            let lower=Q::from(&c-&r);let upper=Q::from(&c+&r);
            if lower>lo||upper<hi {
                // Do not label a narrow oracle overlap as a proved rejection.
                assert!(lower>hi||upper<lo,"inconclusive overlap; refine oracle");
                count.1+=1;
                if count.1<=8 {println!("FAIL {} {}/{} p={} error_ulps={:.9e}",f[0],f[1],f[2],bits,Q::from((c-&lo)/r).to_f64());}
            }
        }
    }
    for (op,(n,b)) in &counts {println!("{op}\tchecks={n}\tfailures={b}");}
    let (n,b)=counts.values().fold((0,0),|(n,b),(x,y)|(n+x,b+y));
    println!("TOTAL {mode}\tchecks={n}\tfailures={b}");
    if mode=="hyper" {assert_eq!(b,0);}
}
