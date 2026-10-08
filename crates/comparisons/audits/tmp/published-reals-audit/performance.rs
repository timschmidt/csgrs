use std::{hint::black_box,time::Instant};
fn time(mut f:impl FnMut(),n:usize) {for _ in 0..n.min(100){f();}let t=Instant::now();for _ in 0..n{f();}println!("{:.3}",t.elapsed().as_nanos() as f64/n as f64);}
fn main(){
    let args:Vec<_>=std::env::args().collect();let lib=&args[1];let case=args[2].as_str();let arg:usize=args[3].parse().unwrap();let n:usize=args[4].parse().unwrap();
    if lib=="published" {
        use computable_real::Real as C;
        match case {
            "clone"|"cached"|"refine"=>{
                let mut c=C::pi();for _ in 0..arg{c=c+C::pi();}black_box(c.appr(-128));
                match case {"clone"=>time(||{black_box(c.clone());},n),"cached"=>time(||{black_box(c.appr(-128));},n),_=>time(||{let mut d=c.clone();black_box(d.appr(-256));},n)}
            },
            "sqrt_cold"=>time(||{black_box(C::from(2).sqrt().appr(-(arg as i32)));},n),
            "cos_cold"=>time(||{black_box((C::one()/C::from(4)).cos().appr(-(arg as i32)));},n),
            "real_clone"=>{let c=reals::Real::from(2).sqrt().unwrap()+reals::Real::from(3).sqrt().unwrap();time(||{black_box(c.clone());},n)},
            "layout"=>println!("Real={} Rational={} Computable={}",size_of::<reals::Real>(),size_of::<num::BigRational>(),size_of::<C>()),
            _=>panic!("unknown case")
        }
    }else{
        use hyperreal::{Computable as C,Rational as Q,Real};
        match case{
            "clone"|"cached"|"refine"=>{
                let mut c=C::pi();for _ in 0..arg{c=c.add(C::pi());}black_box(c.approx(-128));
                match case {"clone"=>time(||{black_box(c.clone());},n),"cached"=>time(||{black_box(c.approx(-128));},n),_=>time(||{let d=c.clone();black_box(d.approx(-256));},n)}
            },
            "sqrt_cold"=>time(||{black_box(C::rational(Q::new(2)).sqrt().approx(-(arg as i32)));},n),
            "cos_cold"=>time(||{black_box(C::rational(Q::fraction(1,4).unwrap()).cos().approx(-(arg as i32)));},n),
            "real_clone"=>{let c=Real::new(Q::new(2)).sqrt().unwrap()+Real::new(Q::new(3)).sqrt().unwrap();time(||{black_box(c.clone());},n)},
            "layout"=>println!("Real={} Rational={} Computable={}",size_of::<Real>(),size_of::<Q>(),size_of::<C>()),
            _=>panic!("unknown case")
        }
    }
}
