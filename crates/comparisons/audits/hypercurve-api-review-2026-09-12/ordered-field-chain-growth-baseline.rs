use hypercurve::{Real, RealSign};
use hypersolve::{IsolatedRootInterval, OrderedFieldPolynomialContext, ordered_field_sign_at_selected_root};
use std::cmp::Ordering;
use std::time::Instant;

#[derive(Default)]
struct Field {
    max_bits: u64,
    products: usize,
}
impl Field {
    fn record(&mut self, value: Real) -> Result<Real, &'static str> {
        let bits = value.exact_rational_ref().and_then(|x| x.to_big_integer()).ok_or("integer fixture left its coefficient domain")?.magnitude().bits();
        self.max_bits = self.max_bits.max(bits);
        if bits > 16_384 { return Err("coefficient budget exceeded"); }
        Ok(value)
    }
}
impl OrderedFieldPolynomialContext<Real> for Field {
    type Error = &'static str;
    fn zero(&mut self) -> Result<Real, Self::Error> { Ok(Real::zero()) }
    fn add(&mut self, a:&Real,b:&Real)->Result<Real,Self::Error> {self.record(a+b)}
    fn multiply(&mut self,a:&Real,b:&Real)->Result<Real,Self::Error> {self.products+=1; self.record(a*b)}
    fn scale(&mut self,a:&Real,b:&Real)->Result<Real,Self::Error> {self.record(a*b)}
    fn sign(&mut self,a:&Real)->Result<Ordering,Self::Error> {match a.immediate_sign().ok_or("integer sign")? {RealSign::Negative=>Ok(Ordering::Less),RealSign::Zero=>Ok(Ordering::Equal),RealSign::Positive=>Ok(Ordering::Greater)}}
    fn sign_if_separated(&mut self,a:&Real)->Result<Option<Ordering>,Self::Error> {self.sign(a).map(Some)}
}
fn main() {
    // (t-1)(t^2+1)(t^2+2)(t^2+3) has exactly one real root, t=1.
    let original = [-6,6,-11,11,-6,6,-1,1].map(Real::from);
    let query = [2,-3,4,-2,5,-6,1].map(Real::from);
    assert_eq!(Real::eval_poly(&query,&Real::one()),Real::one());
    let interval=IsolatedRootInterval{lower:Real::zero(),upper:Real::from(2),exact_root:None,distinct_root_count:1};
    for scale in [Real::one(),Real::from(-1),Real::from(65536)] {
        let defining:Vec<_>=original.iter().map(|x|x*&scale).collect();
        let mut field=Field::default();
        let started=Instant::now();
        let result=ordered_field_sign_at_selected_root(&defining,&query,&interval,&mut field);
        println!("scale={scale:?} result={result:?} max_bits={} products={} elapsed={:?}",field.max_bits,field.products,started.elapsed());
        if let Ok(answer)=result { assert_eq!(answer,Some(Ordering::Greater)); }
    }
}
