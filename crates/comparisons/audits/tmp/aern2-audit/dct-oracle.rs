use rug::{Float, Integer, Rational, float::{Constant, Round}};
use std::{collections::BTreeMap, io::{self, BufRead}};

fn ratio(n: &str, d: &str) -> Rational {
    Rational::from((n.parse::<Integer>().unwrap(), d.parse::<Integer>().unwrap()))
}
fn coefficient(seed: i32, k: i32) -> Rational {
    Rational::from(((17*seed + 11*k)%23-11, 8))
}
fn cos_bounds(r: i32, m: i32, pi_lo: &Rational, pi_hi: &Rational) -> (Rational,Rational) {
    if r == 0 { return (Rational::from(1),Rational::from(1)); }
    if 2*r == m { return (Rational::from(0),Rational::from(0)); }
    let q = Rational::from((r,m));
    let lo = pi_lo.clone()*&q;
    let hi = pi_hi.clone()*q;
    let x = Float::with_val(2048,&lo);
    let xr = x.to_rational().unwrap();
    let err = (xr.clone()-lo).abs().max((xr-hi).abs());
    let mut down = x.clone(); down.cos_round(Round::Down);
    let mut up = x; up.cos_round(Round::Up);
    // Global 1-Lipschitz cosine encloses both pi and argument-rounding error.
    (down.to_rational().unwrap()-&err, up.to_rational().unwrap()+err)
}
fn main() {
    let pi_lo = Float::with_val_round(2048,Constant::Pi,Round::Down).0.to_rational().unwrap();
    let pi_hi = Float::with_val_round(2048,Constant::Pi,Round::Up).0.to_rational().unwrap();
    let mut cache = BTreeMap::<(i32,i32),(Rational,Rational)>::new();
    let mut counts = BTreeMap::<String,[usize;3]>::new();
    for (index,line) in io::stdin().lock().lines().enumerate() {
        let line=line.unwrap();
        let f:Vec<_>=line.split_whitespace().collect();
        assert_eq!(f.len(),9);
        let kind=f[0]; let p=f[1];
        let n:i32=f[2].parse().unwrap(); let s:i32=f[3].parse().unwrap();
        let j:i32=f[4].parse().unwrap();
        let actual_lo=ratio(f[5],f[6]); let actual_hi=ratio(f[7],f[8]);
        assert!(actual_lo<=actual_hi);
        let (count,m) = if kind.starts_with("I-") {(n+1,n)} else {(n,2*n)};
        let mut weights = BTreeMap::<i32,Rational>::new();
        for k in 0..count {
            let (t,weight) = if kind.starts_with("I-") {
                (j*k, if k==0 || k==n {Rational::from((1,2))} else {Rational::from(1)})
            } else if kind.starts_with("III-") {
                ((2*j+1)*k,if k==0 {Rational::from((1,2))} else {Rational::from(1)})
            } else { ((4*j+1)*k,Rational::from(1)) };
            // Exact cosine symmetries collect cancellation before the MPFR
            // sum, including exact zero and endpoint transform results.
            let mut r=t%(2*m);
            if r>m {r=2*m-r;}
            let mut c=coefficient(s,k)*weight;
            if 2*r>m {r=m-r;c = -c;}
            *weights.entry(r).or_default() += c;
        }
        let mut lo=Rational::from(0); let mut hi=Rational::from(0);
        for (r,c) in weights {
            if c==0 {continue;}
            let (cl,ch)=cache.entry((r,m)).or_insert_with(||cos_bounds(r,m,&pi_lo,&pi_hi));
            if c>0 {lo+=cl.clone()*&c;hi+=ch.clone()*c;}
            else {lo+=ch.clone()*&c;hi+=cl.clone()*c;}
        }
        assert!(lo<=hi);
        let tally=counts.entry(format!("{kind}/{p}")).or_default();
        if actual_lo<=lo && hi<=actual_hi {tally[0]+=1;}
        else if actual_hi<lo || hi<actual_lo {
            tally[1]+=1; eprintln!("FAIL {} {} n={} seed={} j={}",index+1,kind,n,s,j);
        } else {
            tally[2]+=1; eprintln!("INDETERMINATE {} {} n={} seed={} j={}",index+1,kind,n,s,j);
        }
    }
    let mut total=[0;3];
    for (k,v) in counts {println!("{k}: {} pass, {} fail, {} indeterminate",v[0],v[1],v[2]);
        for i in 0..3 {total[i]+=v[i];}}
    println!("TOTAL: {} pass, {} fail, {} indeterminate",total[0],total[1],total[2]);
    if total[1]+total[2]!=0 {std::process::exit(1);}
}
