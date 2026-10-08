use hyperreal::{Computable,Rational};
use rug::{Float,Integer,Rational as Q,float::Round};
use std::time::Instant;
fn main(){
    let args:Vec<_>=std::env::args().collect();let op=&args[1];let bits:u32=args[2].parse().unwrap();
    assert!((64..=1_000_000).contains(&bits));
    let q=Q::from((1,16));let mut lo=Float::with_val(bits+256,&q);assert_eq!(lo.to_rational().unwrap(),q);let mut hi=lo.clone();
    match op.as_str(){"asin"=>{lo.asin_round(Round::Down);hi.asin_round(Round::Up);},"asinh"=>{lo.asinh_round(Round::Down);hi.asinh_round(Round::Up);},_=>panic!("op")}
    let lo=lo.to_rational().unwrap();let hi=hi.to_rational().unwrap();
    let input=Computable::rational(Rational::fraction(1,16).unwrap());
    let value=match op.as_str(){"asin"=>input.asin(),"asinh"=>input.asinh(),_=>unreachable!()};
    eprintln!("START {op}(1/16) bits={bits} oracle_bits={}",bits+256);
    let start=Instant::now();let answer=value.approx(-(bits as i32));let elapsed=start.elapsed().as_nanos();
    let unit=Q::from((1,Integer::from(1)<<bits));
    let center=Q::from(Integer::from_str_radix(&answer.to_string(),10).unwrap())*&unit;
    let ok=Q::from(&center-&unit)<=lo&&Q::from(&center+&unit)>=hi;
    if !ok {assert!(Q::from(&center-&unit)>hi||Q::from(&center+&unit)<lo,"oracle overlap is inconclusive");}
    println!("{} {op}(1/16) bits={bits} elapsed_ns={elapsed}",if ok{"PASS"}else{"FAIL"});
    if !ok {std::process::exit(2);}
}
