use computable::{Binary, Bounds, Computable, FiniteBounds, XBinary, XUsize};
use computable::binary_utils::bisection::{normalize_bounds, normalize_finite_to_bounds};
use num::{BigInt, BigRational, One, ToPrimitive, Zero};
use rug::{Float, Integer, float::{Constant, Round}};
use std::{collections::BTreeMap, num::NonZeroU32};

#[derive(Default)]
struct Tally(BTreeMap<&'static str, (usize, usize)>);
impl Tally {
    fn check(&mut self, name: &'static str, ok: bool, detail: impl FnOnce() -> String) {
        let count=self.0.entry(name).or_default(); count.0+=1;
        if !ok {count.1+=1; if count.1<=3 {println!("FAIL {name}: {}",detail());}}
    }
}
fn bin(n: i64, e: i32) -> Binary {Binary::new(n.into(), e.into())}
fn comp(n: i64,e:i32)->Computable {Computable::constant(bin(n,e))}
fn ratio(b:&Binary)->BigRational {
    let e=b.exponent().to_i32().unwrap();
    if e>=0 {BigRational::from_integer(b.mantissa()<<e as usize)}
    else {BigRational::new(b.mantissa().clone(),BigInt::one()<<(-e as usize))}
}
fn endpoints(b:&Bounds)->Option<(BigRational,BigRational)> {
    match (b.small(),b.large()) {(XBinary::Finite(l),XBinary::Finite(h))=>Some((ratio(l),ratio(&h))),_=>None}
}
fn as_float(r:&BigRational)->Float {
    let n=Integer::from_str_radix(&r.numer().to_string(),10).unwrap();
    let d=Integer::from_str_radix(&r.denom().to_string(),10).unwrap();
    Float::with_val(2048,rug::Rational::from((n,d)))
}
fn hyper_rat(r: &BigRational)->hyperreal::Rational {
    hyperreal::Rational::from_bigint_fraction(r.numer().clone(),r.denom().to_biguint().unwrap()).unwrap()
}
fn from_hyper(r: &hyperreal::Rational)->BigRational {
    BigRational::new(BigInt::from_biguint(r.sign(),r.numerator().clone()),BigInt::from(r.denominator().clone()))
}
fn hyper_check(t:&mut Tally,name:&'static str,c:hyperreal::Real,p:usize,l:&Float,h:&Float){
    let [a,z]=c.certified_dyadic_interval(-(p as i32)-8).unwrap();
    t.check(name,as_float(&from_hyper(&a))<=*l && as_float(&from_hyper(&z))>=*h,||format!("p={p}, bounds=[{a},{z}], expected=[{l:.8e},{h:.8e}]"));
}
fn contains(b:&Bounds,l:&Float,h:&Float)->bool {
    endpoints(b).is_some_and(|(a,z)|as_float(&a)<=*l && as_float(&z)>=*h)
}
fn refined(t:&mut Tally,name:&'static str,c:Computable,p:usize,l:Float,h:Float) {
    let got=c.refine_to::<192>(XUsize::Finite(p));
    t.check(name,got.as_ref().is_ok_and(|b|contains(b,&l,&h) && endpoints(b).is_some_and(|(a,z)|z-a<=BigRational::new(BigInt::one(),BigInt::one()<<p))),||format!("p={p}, got={got:?}, expected=[{l:.8e},{h:.8e}]"));
}
fn main(){
    let mut t=Tally::default();
    for n in -64..=64 {for w in 1..=16 {for e in [-40,-4,0,40] {
        let b=FiniteBounds::new(bin(n,e),bin(n+w,e));
        let a=normalize_bounds(&b).unwrap();
        let category=if n<0 && n+w>0 {"normalize_crossing"}else{"normalize_same_sign"};
        t.check(category,a.small()<=b.small() && a.hi()>=b.hi(),||format!("[{n},{next}]*2^{e} -> {a:?}",next=n+w));
    }}}
    for sign in [-1,1] {for k in -16..=16 {for w in 1..=16 {
        let center=sign*((1i64<<40)+k);
        let b=FiniteBounds::new(bin(center,-40),bin(center+w,-40));
        let a=normalize_finite_to_bounds(&b).unwrap();
        let (l,h)=endpoints(&a).unwrap();
        t.check("normalize_wrapper",l<=ratio(b.small()) && h>=ratio(&b.hi()),||format!("input={b:?}, output={a:?}"));
    }}}
    for n in -24..=24 {for e in [-8,-1,0,8] {
        let a=ratio(&bin(n,e));let b=ratio(&bin(n+7,e-2));
        for (name,c,r) in [
            ("add",comp(n,e)+comp(n+7,e-2),&a+&b),
            ("mul",comp(n,e)*comp(n+7,e-2),&a*&b),
            ("neg",-comp(n,e),-a.clone()),
            ("pow3",comp(n,e).pow(3),a.pow(3)),
        ] {
            let got=c.bounds().unwrap();
            t.check(name,endpoints(&got).is_some_and(|(l,h)|l==r && h==r),||format!("n={n}, e={e}, got={got:?}, expected={r}"));
        }
    }}
    for n in 1..=32 {for e in [-4,0,4] {for degree in [2,3,5] {
        let input=ratio(&bin(n,e));
        let root=comp(n,e).nth_root(NonZeroU32::new(degree).unwrap());
        let got=root.refine_to::<192>(XUsize::Finite(32));
        let ok=got.as_ref().is_ok_and(|b| endpoints(b).is_some_and(|(l,h)|l>=BigRational::zero() && l.pow(degree as i32)<=input && h.pow(degree as i32)>=input));
        t.check("constant_root",ok,||format!("input={input}, degree={degree}, got={got:?}"));
        let h=hyperreal::Real::new(hyper_rat(&input)).root_n(degree).unwrap();
        let [l,u]=h.certified_dyadic_interval(-64).unwrap();
        let (l,u)=(from_hyper(&l),from_hyper(&u));
        t.check("hyper_constant_root",l>=BigRational::zero() && l.pow(degree as i32)<=input && u.pow(degree as i32)>=input,||format!("input={input}, degree={degree}"));
    }}}
    for e in [80,100] {
        let c=comp(1,e).inv();
        let _=c.refine_to::<4>(XUsize::Finite(8)).unwrap();
        let got=c.refine_to::<4>(XUsize::Finite(128));
        t.check("inverse_after_coarse",got.is_ok(),||format!("input=2^{e}, got={got:?}"));
        let h=hyperreal::Computable::rational(hyper_rat(&ratio(&bin(1,e)))).inverse();
        let _=h.approx(-8);let got=h.approx(-128);
        t.check("hyper_inverse_after_coarse",got==(BigInt::one()<<(128-e as usize)),||format!("input=2^{e}, got={got}"));
    }
    let leaf=Computable::new(0u32,|s|Ok(Bounds::new(XBinary::Finite(bin(4,0)),XBinary::Finite(bin(4,0).add(&bin(1,-(*s as i32)))))),|s|Ok(s+1));
    let got=leaf.nth_root(NonZeroU32::new(2).unwrap()).refine_to::<192>(XUsize::Finite(32));
    t.check("root_refining_input",got.as_ref().is_ok_and(|b| endpoints(b).is_some_and(|(l,h)|l<=BigRational::from_integer(2.into()) && h>=BigRational::from_integer(2.into()))),||format!("sqrt(4), got={got:?}"));
    for sign in [-1,1] {for n in [1,3,17] {for e in [-32,0,32,80,100] {
        let r=ratio(&bin(sign*n,e)).recip();
        let got=comp(sign*n,e).inv().refine_to::<4>(XUsize::Finite(128));
        t.check("inverse_magnitude",got.as_ref().is_ok_and(|b|endpoints(b).is_some_and(|(l,h)|l<=r && h>=r)),||format!("input={}*2^{e}, got={got:?}",sign*n));
    }}}
    for p in [8,32,64,128,256,512] {
        let (pl,ph)=(Float::with_val_round(2048,Constant::Pi,Round::Down).0,Float::with_val_round(2048,Constant::Pi,Round::Up).0);
        let (l,h)=computable::pi_bounds_at_precision(p);
        let raw=Bounds::new(XBinary::Finite(l),XBinary::Finite(h));
        t.check("raw_pi",contains(&raw,&pl,&ph),||format!("p={p}, bounds={raw:?}"));
        refined(&mut t,"node_pi",computable::pi(),p,pl.clone(),ph.clone());
        hyper_check(&mut t,"hyper_pi",hyperreal::Real::pi(),p,&pl,&ph);
        if p<=128 {let(mut l,mut h)=(pl,ph);l.sqrt_round(Round::Down);h.sqrt_round(Round::Up);
            hyper_check(&mut t,"hyper_sqrt_pi",hyperreal::Real::pi().sqrt().unwrap(),p,&l,&h);
            refined(&mut t,"sqrt_pi",computable::pi().nth_root(NonZeroU32::new(2).unwrap()),p,l,h);
        }
    }
    for n in -24..=24 {for p in [16,64,128] {
        let x=as_float(&ratio(&bin(n,-2)));let(mut l,mut h)=(x.clone(),x);
        l.sin_round(Round::Down);h.sin_round(Round::Up);
        hyper_check(&mut t,"hyper_sin_grid",hyperreal::Real::new(hyper_rat(&ratio(&bin(n,-2)))).sin(),p,&l,&h);
        refined(&mut t,"sin_grid",comp(n,-2).sin(),p,l,h);
    }}
    for n in [-10000,-100,-1,1,100,10000] {for p in [16,64] {
        let x=Float::with_val(2048,n);let(mut l,mut h)=(x.clone(),x);
        l.sin_round(Round::Down);h.sin_round(Round::Up);
        hyper_check(&mut t,"hyper_sin_magnitude",hyperreal::Real::new(hyperreal::Rational::new(n)).sin(),p,&l,&h);
        refined(&mut t,"sin_magnitude",comp(n,0).sin(),p,l,h);
    }}
    // A valid nested leaf that becomes the exact real 1 on its first step.
    let leaf=Computable::new(false,|exact|Ok(if *exact {Bounds::point(XBinary::Finite(bin(1,0)))}else{Bounds::new(XBinary::Finite(bin(0,0)),XBinary::Finite(bin(2,0)))}),|_|Ok(true));
    let expression=leaf.clone()+comp(1,0);
    let _=expression.bounds().unwrap();
    let _=leaf.refine_to::<4>(XUsize::Finite(32)).unwrap();
    let got=expression.refine_to::<4>(XUsize::Finite(32));
    t.check("cached_parent_exact_child",got.as_ref().is_ok_and(|b|endpoints(b).is_some_and(|(l,h)|l==BigRational::from_integer(2.into()) && h==l)),||format!("got={got:?}"));
    println!("SUMMARY {:?}",t.0);
}
