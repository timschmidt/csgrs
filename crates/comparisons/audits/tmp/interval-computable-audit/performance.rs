use std::{hint::black_box,time::Instant,num::NonZeroU32};
fn time(mut f:impl FnMut(),n:usize){for _ in 0..n.min(100){f();}let start=Instant::now();for _ in 0..n{f();}println!("{:.3}",start.elapsed().as_nanos() as f64/n as f64);}
fn main(){
    let args:Vec<_>=std::env::args().collect();let lib=&args[1];let case=args[2].as_str();let p:usize=args[3].parse().unwrap();let n:usize=args[4].parse().unwrap();
    if lib=="donor" {
        use computable::{Binary,Computable as C,XUsize};
        let constant=|v:i32|C::constant(Binary::new(v.into(),0.into()));
        let sqrt=||constant(2).nth_root(NonZeroU32::new(2).unwrap());
        match case {
            "clone"|"cached"=>{let mut c=sqrt();for _ in 0..32{c=c+constant(1);}black_box(c.refine_to::<192>(XUsize::Finite(p)).unwrap());
                if case=="clone" {time(||{black_box(c.clone());},n)}else{time(||{black_box(c.refine_to::<192>(XUsize::Finite(p)).unwrap());},n)}
            },
            "sqrt"=>time(||{black_box(sqrt().refine_to::<192>(XUsize::Finite(p)).unwrap());},n),
            "inverse"=>time(||{black_box(constant(3).inv().refine_to::<8>(XUsize::Finite(p)).unwrap());},n),
            "mixed"=>time(||{black_box((sqrt()+computable::pi()).refine_to::<192>(XUsize::Finite(p)).unwrap());},n),
            "layout"=>println!("Computable={} Binary={} Bounds={}",size_of::<C>(),size_of::<Binary>(),size_of::<computable::Bounds>()),
            _=>panic!("unknown case")
        }
    }else{
        use hyperreal::{Computable as C,Rational as Q};use num::BigInt;
        // Center +/- one at p+1 has width 2^-p, matching donor tolerance.
        let bounds=|c:&C|{let a=c.approx(-(p as i32)-1);black_box([&a-BigInt::from(1),a+BigInt::from(1)]);};
        let sqrt=||C::rational(Q::new(2)).sqrt();
        match case {
            "clone"|"cached"=>{let mut c=sqrt();for _ in 0..32{c=c.add(C::rational(Q::new(1)));}bounds(&c);
                if case=="clone" {time(||{black_box(c.clone());},n)}else{time(||bounds(&c),n)}
            },
            "sqrt"=>time(||bounds(&sqrt()),n),
            "inverse"=>time(||bounds(&C::rational(Q::new(3)).inverse()),n),
            "mixed"=>time(||bounds(&sqrt().add(C::pi())),n),
            "layout"=>println!("Computable={} Rational={} ApproxCache={}",size_of::<C>(),size_of::<Q>(),size_of::<(BigInt,i32)>()),
            _=>panic!("unknown case")
        }
    }
}
