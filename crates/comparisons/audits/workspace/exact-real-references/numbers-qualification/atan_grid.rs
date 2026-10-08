use hyperreal::{Computable,Rational};
use rug::{Float,Integer,Rational as Q,float::Round};
const P:u32=4096;
fn c(q:&Q)->Computable {Computable::rational(q.to_string().parse::<Rational>().unwrap())}
fn elementary(q:&Q,root:bool)->(Q,Q) {
    let mut lo=Float::with_val(P,q);assert_eq!(lo.to_rational().unwrap(),*q);
    let mut hi=lo.clone();
    if root {lo.sqrt_round(Round::Down);hi.sqrt_round(Round::Up);}else{lo.sin_round(Round::Down);hi.sin_round(Round::Up);}
    (lo.to_rational().unwrap(),hi.to_rational().unwrap())
}
fn atan_bounds((lo,hi):(Q,Q))->(Q,Q) {
    let mut lo=Float::with_val_round(P,lo,Round::Down).0;
    let mut hi=Float::with_val_round(P,hi,Round::Up).0;
    lo.atan_round(Round::Down);hi.atan_round(Round::Up);
    (lo.to_rational().unwrap(),hi.to_rational().unwrap())
}
fn check(make:impl Fn()->Computable,bounds:(Q,Q))->usize {
    let bounds=atan_bounds(bounds);
    let mut count=0;
    for history in 0..4 {
        let input=make();
        match history {
            0=>{},1=>{let _=input.approx(-4);},2=>{let _=input.approx(-256);},
            3=>{for p in [-2,-128,0] {let _=input.approx(p);}},_=>unreachable!(),
        }
        let x=input.atan();
        for p in [0,8,32,128,512,2] {
            let scale=Integer::from(1)<<p;
            let got=Q::from((Integer::from_str_radix(&x.approx(-p).to_string(),10).unwrap(),scale.clone()));
            let radius=Q::from((1,scale));
            assert!(Q::from(&got-&radius)<=bounds.0&&Q::from(&got+&radius)>=bounds.1,
                "atan bound violation history={history} precision={p}");
            count+=1;
        }
    }
    count
}
fn main() {
    let mut trig=0;let mut sums=0;let mut endpoints=0;
    for n in [-100,-4,-1,0,1,4,100] {for scale in [Q::from((1,16)),Q::from(1),Q::from(64)] {
        let (lo,hi)=elementary(&Q::from(n),false);
        for numerator in [-17,-9,-8,-7,-1,0,1,7,8,9,17] {
            let offset=Q::from((numerator,16));
            trig+=check(||c(&Q::from(n)).sin().multiply(c(&scale)).add(c(&offset)),
                (lo.clone()*&scale+&offset,hi.clone()*&scale+&offset));
        }
    }}
    let (a,b)=elementary(&Q::from(5),true);let (u,v)=elementary(&Q::from(7),true);
    for terms in [0,1,4,8,16,32,64] {for sign in [-1,1] {
        let bounds:(Q,Q)=(a.clone()/8+u.clone()*terms/64,b.clone()/8+v.clone()*terms/64);
        let bounds=if sign<0 {(-bounds.1,-bounds.0)}else{bounds};
        sums+=check(||{
            let mut x=c(&Q::from(5)).sqrt().multiply(c(&Q::from((1,8))));
            let term=c(&Q::from(7)).sqrt().multiply(c(&Q::from((1,64))));
            for _ in 0..terms {x=x.add(term.clone());}
            if sign<0 {x.negate()}else{x}
        },bounds);
    }}
    // Opaque near-boundary inputs use sin of a nonzero exact dyadic, not a
    // rational leaf that can bypass the generic atan dispatcher.
    for base in [-1,0,1] {for shift in [8,32,128,1024] {for sign in [-1,1] {
        let tiny=Q::from((sign,Integer::from(1)<<shift));
        let offset=Q::from((base,2));let (lo,hi)=elementary(&tiny,false);
        endpoints+=check(||c(&tiny).sin().add(c(&offset)),(lo+&offset,hi+&offset));
    }}}
    println!("PASS atan public controls: trig-offset-history={trig}, estimated-sum-history={sums}, half-boundary-history={endpoints}, total={}",trig+sums+endpoints);
}
