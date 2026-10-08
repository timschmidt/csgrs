use hyperreal::{Computable, Rational, Real};
use rug::{Float, Integer, Rational as Q, float::Round};
fn root(n: i32) -> (Q,Q) {
    let mut lo=Float::with_val(4096,n);let mut hi=lo.clone();
    lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);
    (lo.to_rational().unwrap(),hi.to_rational().unwrap())
}
fn h(q:&Q)->Real {Real::from(q.to_string().parse::<Rational>().unwrap())}
fn c(q:&Q)->Computable {Computable::rational(q.to_string().parse::<Rational>().unwrap())}
fn check_view(label:&str,view:Q,lo:&Q,hi:&Q) {
    let radius=lo.clone().abs().max(hi.clone().abs()) / (Integer::from(1)<<48);
    assert!(Q::from(lo-&radius)<=view&&view<=Q::from(hi+&radius),"{label} outside relative 2^-48 envelope");
}
fn main() {
    let path=std::env::args().nth(1).unwrap();let text=std::fs::read_to_string(path).unwrap();
    let mut close=(Q::new(),Q::new(),Real::zero(),Computable::rational(Rational::zero()));
    for (sgn,ns) in [(1,[7,14,39,70,72,76,85]),(-1,[13,16,46,55,67,73,79])] {
        for n in ns {
            let (lo,hi)=root(n);
            if sgn>0 {close.0+=lo;close.1+=hi;}else{close.0-=hi;close.1-=lo;}
            close.2+=Real::from(sgn)*Real::from(n).sqrt().unwrap();
            close.3=close.3.add(Computable::rational(Rational::from(n)).sqrt().multiply(Computable::rational(Rational::from(sgn))));
        }
    }
    assert!(close.0>0&&close.1>close.0);
    let mut count=0;
    for row in text.lines() {
        let cols:Vec<_>=row.split_whitespace().collect();if cols[0]=="SUMMARY" {assert_eq!(cols[1],"194");continue;}
        assert_eq!(cols.len(),8);let sign:i32=cols[2].parse().unwrap();assert!(sign.abs()==1);
        let p=Q::from_str_radix(cols[3],10).unwrap();let q=Q::from_str_radix(cols[4],10).unwrap();
        let (mut lo,mut hi,mut value,mut raw)=if cols[0]=="close" {close.clone()} else {
            assert_eq!(cols[0],"pell");let norm=Q::from(&p*&p)-Q::from(&q*&q)*2;assert!(norm==1||norm== -1);
            let (lo,hi)=root(2);
            (lo*&q-&p,hi*&q-&p,h(&q)*Real::from(2).sqrt().unwrap()-h(&p),c(&q).multiply(Computable::rational(Rational::from(2)).sqrt()).add(c(&(-p.clone()))))
        };
        if sign<0 {(lo,hi)=(-hi,-lo);value= -value;raw=raw.negate();}
        assert!(lo<=hi);
        let donor=Q::from((Integer::from_str_radix(cols[5],10).unwrap(),Integer::from_str_radix(cols[6],10).unwrap()));
        let donor_float:f64=cols[7].parse().unwrap();assert_eq!(Q::from_f64(donor_float).unwrap(),donor);
        check_view("donor",donor,&lo,&hi);
        let hyper=value.to_f64_lossy().expect("finite view");assert!(hyper.is_finite());check_view("Hyper",Q::from_f64(hyper).unwrap(),&lo,&hi);
        let approximation=raw.approx(-512);let scale:Integer=Integer::from(1)<<512;
        let center=Q::from((Integer::from_str_radix(&approximation.to_string(),10).unwrap(),scale.clone()));let radius=Q::from((1,scale));
        assert!(Q::from(&center-&radius)<=lo&&Q::from(&center+&radius)>=hi,"Hyper nonenclosure");
        count+=1;println!("PASS\t{}\t{}\t{}",cols[0],cols[1],sign);
    }
    assert_eq!(count,194);println!("SUMMARY\t194\t582");
}
