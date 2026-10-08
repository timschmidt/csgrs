use hyperreal::{Computable, Rational, RealSign};
use num::{BigUint, One};
use rug::{Float, Integer, Rational as Q, float::Round};
use std::sync::{Arc, Barrier, atomic::{AtomicBool, Ordering}};

fn argument(case: usize, depth: usize) -> Computable {
    let mut x=Computable::rational(Rational::fraction((case+1) as i64,32).unwrap());
    for _ in 0..depth { x=x.sin(); }
    x
}
fn pair(case: usize, depth: usize, numerator: i64, bits: usize) -> (Computable,Computable) {
    let delta=Rational::from_bigint_fraction(numerator.into(),BigUint::one()<<bits).unwrap();
    (argument(case,depth).exp().sqrt(),argument(case,depth).multiply(Computable::rational(Rational::fraction(1,2).unwrap()))
        .add(Computable::rational(delta)).exp())
}
fn reference(case: usize, depth: usize, numerator: i64, bits: usize) -> (Q,Q) {
    let q=Q::from(((case+1) as i32,32));
    let mut lo=Float::with_val_round(2400,&q,Round::Down).0;
    let mut hi=Float::with_val_round(2400,&q,Round::Up).0;
    assert_eq!(lo.to_rational().unwrap(),q);
    assert_eq!(hi.to_rational().unwrap(),q);
    for _ in 0..depth {
        // All arguments stay in (0,1), a monotone sine interval.
        assert!(lo>0&&hi<1);
        lo.sin_round(Round::Down);hi.sin_round(Round::Up);
    }
    lo>>=1;hi>>=1;
    let delta=Q::from((Integer::from(numerator),Integer::from(1)<<bits));
    // Add as exact rationals, then round each endpoint outward before exp.
    let lq=lo.to_rational().unwrap()+&delta;
    let hq=hi.to_rational().unwrap()+&delta;
    let mut l=Float::with_val_round(2400,lq,Round::Down).0;
    let mut h=Float::with_val_round(2400,hq,Round::Up).0;
    l.exp_round(Round::Down);h.exp_round(Round::Up);
    (l.to_rational().unwrap(),h.to_rational().unwrap())
}
fn check(x:&Computable,p:i32,bounds:&(Q,Q)) {
    let unit=Q::from((1,Integer::from(1)<<(-p)));
    let actual=Integer::from_str_radix(&x.approx(p).to_string(),10).unwrap();
    let middle=Q::from(actual)*&unit;
    assert!(Q::from(&middle-&unit)<=bounds.0&&Q::from(&middle+&unit)>=bounds.1);
}
fn expected(numerator:i64,bits:usize)->Option<RealSign> {
    if numerator==0 {Some(RealSign::Zero)} else if bits>1023 {None}
    else if numerator<0 {Some(RealSign::Positive)} else {Some(RealSign::Negative)}
}
fn main() {
    let mut queries=0;let mut enclosures=0;
    for depth in [1,8] {for numerator in [-1,0,1] {for bits in [767,999,2048] {
        let (a,b)=pair(3,depth,numerator,bits);
        let left=reference(3,depth,0,bits);let right=reference(3,depth,numerator,bits);
        let before=(serde_json::to_string(&a).unwrap(),serde_json::to_string(&b).unwrap());
        let signal=Arc::new(AtomicBool::new(true));
        let _=a.approx_signal(&Some(signal.clone()),-512);
        signal.store(false,Ordering::Relaxed);
        let barrier=Barrier::new(8);
        std::thread::scope(|scope| {
            let handles:Vec<_>=(0..8).map(|thread|{
                let (a,b,left,right,barrier)=(&a,&b,&left,&right,&barrier);
                scope.spawn(move || {
                    barrier.wait();
                    for iteration in 0..12 {
                        // Rebuild only the difference to exercise per-thread
                        // reuse while other workers refine the same operands.
                        let (a,b)=if iteration%3==0 {
                            (serde_json::from_str::<Computable>(&serde_json::to_string(a).unwrap()).unwrap(),
                             serde_json::from_str::<Computable>(&serde_json::to_string(b).unwrap()).unwrap())
                        }else{(a.clone(),b.clone())};
                        let d=a.clone().add(b.clone().negate());
                        assert_eq!(d.sign_until(-64),expected(numerator,bits));
                        let p=[-32,-128,-512][(thread+iteration)%3];
                        check(&a,p,left);check(&b,p,right);
                    }
                })
            }).collect();
            for h in handles{h.join().unwrap();}
        });
        queries+=96;enclosures+=192;
        assert_eq!((serde_json::to_string(&a).unwrap(),serde_json::to_string(&b).unwrap()),before);
    }}}
    // An earlier Unknown result and an occupied cache cannot block later
    // numerical separation outside the structural coefficient cap.
    for numerator in [-1,1] {
        let (a,b)=pair(1,1,numerator,2048);
        let d=a.add(b.negate());
        assert_eq!(d.sign_until(-64),None);
        assert_eq!(d.sign_until(-2304),Some(if numerator<0{RealSign::Positive}else{RealSign::Negative}));
        queries+=2;
    }
    println!("{{\"suite\":\"concurrent-proof-state\",\"sign_queries\":{queries},\"enclosure_checks\":{enclosures},\"workers_per_case\":8}}");
}
