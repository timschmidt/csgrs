use hyperreal::{Rational,Real};
use std::{cmp::Ordering,fs,time::Instant};
fn expression<'a>(tokens:&mut impl Iterator<Item=&'a str>)->Real {
    match tokens.next().expect("expression") {
        "q"=>{let n:i64=tokens.next().unwrap().parse().unwrap();let d:u64=tokens.next().unwrap().parse().unwrap();Real::from(Rational::fraction(n,d).unwrap())},
        "n"=>-expression(tokens),"s"=>expression(tokens).sqrt().unwrap(),
        op@("+"|"*"|"/")=>{let x=expression(tokens);let y=expression(tokens);match op{"+"=>x+y,"*"=>x*y,"/"=>(x/y).unwrap(),_=>unreachable!()}},
        _=>panic!("malformed expression"),
    }
}
fn parse(s:&str)->Real {let mut tokens=s.split_whitespace();let value=expression(&mut tokens);assert!(tokens.next().is_none());value}
fn main(){
    let args:Vec<_>=std::env::args().collect();let corpus=fs::read_to_string(&args[1]).unwrap();let group=&args[2];let p:i32=args[3].parse().unwrap();
    assert!((-131072..=0).contains(&p));let(mut passed,mut unknown,mut wrong)=(0,0,0);
    for row in corpus.lines().skip(1){let cols:Vec<_>=row.split('\t').collect();assert_eq!(cols.len(),5);if group!="all"&&cols[0]!=group{continue;}
        let start=Instant::now();let lhs=parse(cols[3]);let rhs=parse(cols[4]);
        let result=lhs.certified_cmp_until(&rhs,p).ordering();let elapsed=start.elapsed().as_nanos();
        let got=match result{Some(Ordering::Less)=>"LT",Some(Ordering::Equal)=>"EQ",Some(Ordering::Greater)=>"GT",None=>"UNKNOWN"};
        let status=if result.is_none(){unknown+=1;"UNKNOWN"}else if got==cols[2]{passed+=1;"PASS"}else{wrong+=1;"WRONG"};
        println!("{status}\t{}\t{}\t{got}\t{elapsed}",cols[0],cols[1]);
    }
    assert!(passed+unknown+wrong>0);println!("SUMMARY\t{passed}\t{unknown}\t{wrong}\t{}",passed+unknown+wrong);
    if wrong>0{std::process::exit(2)}else if unknown>0{std::process::exit(3)}
}
