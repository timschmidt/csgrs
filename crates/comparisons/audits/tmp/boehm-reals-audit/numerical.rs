use boehm_reals::evaluation::{bounded_rational::BoundedRational as B,creals::StringFloatRep};
use num::{BigInt,BigRational,One,Zero,Signed};
use std::{collections::BTreeMap,hash::{Hash,Hasher}};
#[derive(Default)]struct Tally(BTreeMap<&'static str,(usize,usize)>);
impl Tally {fn check(&mut self,k:&'static str,b:bool,f:impl FnOnce()->String){let c=self.0.entry(k).or_default();c.0+=1;if !b {c.1+=1;if c.1<=3{println!("FAIL {k}: {}",f());}}}}
fn q(n:i64,d:i64)->BigRational{BigRational::new(n.into(),d.into())}
fn from_b(b:&B)->BigRational{BigRational::new(b.numerator().to_string().parse().unwrap(),b.denominator().to_string().parse().unwrap())}
fn to_b(r:&BigRational)->B{B::new(r.numer().to_string().parse().unwrap(),r.denom().to_string().parse().unwrap()).unwrap()}
fn to_h(r:&BigRational)->hyperreal::Rational{hyperreal::Rational::from_bigint_fraction(r.numer().clone(),r.denom().to_biguint().unwrap()).unwrap()}
fn from_h(r:&hyperreal::Rational)->BigRational{BigRational::new(BigInt::from_biguint(r.sign(),r.numerator().clone()),BigInt::from(r.denominator().clone()))}
fn power(e:i32)->BigRational{if e>=0{BigRational::from_integer(BigInt::one()<<e as usize)}else{BigRational::new(BigInt::one(),BigInt::one()<<(-e as usize))}}
fn main(){let mut t=Tally::default();
    for n in -32..=32 {for d in -16..=16 {if d==0{continue;}let a=q(n,d);let b=q(n+5,19);let x=B::from_longs(n,d).unwrap();let y=B::from_longs(n+5,19).unwrap();
        for (name,got,expected) in [("add",&x+&y,&a+&b),("sub",B::subtract(x.clone(),y.clone()),&a-&b),("mul",B::multiply(x.clone(),y.clone()),&a*&b)]{t.check(name,from_b(&got)==expected,||format!("{a}, {b}, got={got}, expected={expected}"));}
        if !b.is_zero(){let got=B::divide(x.clone(),y.clone()).unwrap();t.check("div",from_b(&got)==&a/&b,||format!("{a}/{b} gives {got}"));}
        t.check("cmp",x.compare_to(&y)==a.cmp(&b),||format!("{a} vs {b}"));
        let mut h1=std::collections::hash_map::DefaultHasher::new();let mut h2=std::collections::hash_map::DefaultHasher::new();x.hash(&mut h1);to_b(&a).hash(&mut h2);t.check("hash",h1.finish()==h2.finish(),||format!("{a}"));
        for p in [0,1,8]{let z=(a.abs()*BigInt::from(10).pow(p)).to_integer();let mut s=z.to_string();while s.len()<=p as usize{s.insert(0,'0');}s.insert(s.len()-p as usize,'.');if a.is_negative(){s.insert(0,'-');}let got=x.to_string_truncated(p);t.check("truncated",got==s,||format!("{a}, p={p}, got={got}, expected={s}"));}
    }}
    let mut state=1234u64;
    for _ in 0..10_000 {state=state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);let f=f64::from_bits(state);if !f.is_finite(){continue;}let expected=BigRational::from_float(f).unwrap();let got=B::value_of_double(f).unwrap();t.check("f64_import_random",from_b(&got)==expected,||format!("bits={state:x}, got={got}"));let out=got.double_value();t.check("f64_roundtrip_random",out.to_bits()==f.to_bits() || (out==0.0&&f==0.0),||format!("bits={state:x}, got={:x}",out.to_bits()));}
    for f in [2f64.powi(63),-2f64.powi(63),f64::from_bits(2f64.powi(63).to_bits()-1),f64::from_bits(2f64.powi(63).to_bits()+1)]{let r=BigRational::from_float(f).unwrap();let got=B::value_of_double(f).unwrap();t.check("f64_import_boundary",from_b(&got)==r,||format!("f={f}, got={got}, expected={r}"));let h=hyperreal::Rational::try_from(f).unwrap();t.check("hyper_import_boundary",from_h(&h)==r,||format!("f={f}, got={h}"));}
    for e in [-1022,-100,-1,0,1,100,1000] {for sign in [-1,1] {let r=(power(e)-power(e-54))*BigInt::from(sign);let got=to_b(&r).double_value();let expected=(sign as f64)*2f64.powi(e);t.check("f64_carry",got.to_bits()==expected.to_bits(),||format!("near {sign}*2^{e}, got={got:e}, expected={expected:e}"));let h=hyperreal::Real::new(to_h(&r)).to_f64_lossy().unwrap();t.check("hyper_carry_lossy_ulp",h.to_bits().abs_diff(expected.to_bits())<=1,||format!("near 2^{e}, got={h:e}"));let h=f64::from(hyperreal::Real::new(to_h(&r)));t.check("hyper_carry_owned",h.to_bits()==expected.to_bits(),||format!("near 2^{e}, got={h:e}"));}}
    for e in [53,54,60] {for sign in [-1,1] {let r=(power(e)+power(e-53))*BigInt::from(sign);let got=to_b(&r).double_value();let expected=f64::from_bits(2f64.powi(e).to_bits()+1)*(sign as f64);t.check("integer_ties_away",got.to_bits()==expected.to_bits(),||format!("r={r}, got={got}, expected={expected}"));}}
    let zero=StringFloatRep::new(0,"5".into(),10,"1".into()).unwrap();t.check("format_zero_sign",zero.to_string().parse::<f64>().unwrap()==0.0,||format!("sign=0 gives {zero}"));
    t.check("format_sign_validation",StringFloatRep::new(2,"5".into(),10,"1".into()).is_err(),||"sign 2 accepted".into());
    t.check("format_mantissa_validation",StringFloatRep::new(1,"-5".into(),10,"1".into()).is_err(),||"negative mantissa accepted".into());
    println!("SUMMARY {:?}",t.0);
}
