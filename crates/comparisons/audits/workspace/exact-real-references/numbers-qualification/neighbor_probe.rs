use hyperreal::{Computable,Rational};
use rug::{Float,Integer,Rational as Q,float::Round};
fn c(n:i64,d:u64)->Computable {Computable::rational(Rational::fraction(n,d).unwrap())}
fn root(n:i32)->(Q,Q) {let mut lo=Float::with_val(4096,n);let mut hi=lo.clone();lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);(lo.to_rational().unwrap(),hi.to_rational().unwrap())}
fn main() {
    let args:Vec<_>=std::env::args().collect();let op=&args[1];let terms:i32=args[2].parse().unwrap();let p:i32=args[3].parse().unwrap();
    let (a,b)=root(5);let (u,v)=root(7);
    let lower:Q=a/64+u*terms/2048;let upper:Q=b/64+v*terms/2048;
    assert!(lower>0&&upper<1,"probe is only for valid interior inputs");
    let mut lo=Float::with_val_round(4096,&lower,Round::Down).0;
    let mut hi=Float::with_val_round(4096,&upper,Round::Up).0;
    match op.as_str(){"asin"=>{lo.asin_round(Round::Down);hi.asin_round(Round::Up);},"atanh"=>{lo.atanh_round(Round::Down);hi.atanh_round(Round::Up);},_=>panic!()}
    let mut input=c(5,1).sqrt().multiply(c(1,64));let term=c(7,1).sqrt().multiply(c(1,2048));
    for _ in 0..terms {input=input.add(term.clone());}
    eprintln!("constructed {op} terms={terms} input~{}",lower.to_f64());
    let value=match op.as_str(){"asin"=>input.asin(),"atanh"=>input.atanh(),_=>panic!()};
    let scale=Integer::from(1)<<p;
    let q=Q::from((Integer::from_str_radix(&value.approx(-p).to_string(),10).unwrap(),scale.clone()));
    let radius=Q::from((1,scale));
    let lo=lo.to_rational().unwrap();let hi=hi.to_rational().unwrap();
    let enclosed=Q::from(&q-&radius)<=lo&&Q::from(&q+&radius)>=hi;
    if !enclosed {assert!(Q::from(&q-&radius)>hi||Q::from(&q+&radius)<lo,"inconclusive oracle overlap");}
    println!("{} {op} terms={terms} precision={p} error_ulps={}",if enclosed{"PASS"}else{"FAIL"},Q::from((q-lo)/radius).to_f64());
}
