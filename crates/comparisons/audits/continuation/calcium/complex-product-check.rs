#[allow(dead_code)]
mod corpus { include!("complex-product-corpus.rs"); }
use corpus::*;
use rug::{Integer, Rational as Q};
fn main() {
    let mut fixtures=0_u64;
    let mut components=0_u64;
    let mut input_checks=0_u64;
    let mut zero_components=0_u64;
    for bits in [32,64,127,128,129,191,192,193,255,256,257,511,512,513,1024,2048] {
        for family in FAMILIES { for scale in ["integer","dyadic","odd"] { for mask in 0..16 {
            let f=fixture(bits,family,scale,mask);
            let q: [Q;4]=std::array::from_fn(|i| Q::from((
                Integer::from_str_radix(&f.nums[i].to_string(),10).unwrap(),
                Integer::from_str_radix(&f.dens[i].to_string(),10).unwrap(),
            )));
            let [a,b,c,d]=&q;
            let ac=Q::from(a*c); let bd=Q::from(b*d);
            let ad=Q::from(a*d); let bc=Q::from(b*c);
            let norm=Q::from(c*c)+Q::from(d*d);
            let product=[ac.clone()-&bd,ad.clone()+&bc];
            let quotient=[(ac+bd)/&norm,(bc-ad)/norm];
            let input=f.inputs();
            let before=input.r.each_ref().map(|x| x.to_string());
            for route in ["product","quotient","lattice"] {
                let expected=if route=="quotient" {&quotient} else {&product};
                // Repeat on the same objects after the first call to cover
                // fact/cache learning without treating repetitions as new identities.
                for _ in 0..2 {
                    let result=calculate(&input,route);
                    for (x,e) in [&result.0,&result.1].into_iter().zip(expected) {
                        let got=Q::from((
                            Integer::from_str_radix(&num::BigInt::from_biguint(x.sign(),x.numerator().clone()).to_string(),10).unwrap(),
                            Integer::from_str_radix(&x.denominator().to_string(),10).unwrap(),
                        ));
                        assert_eq!(&got,e,"{bits}/{family}/{scale}/{mask}/{route}");
                        assert_eq!(num::Integer::gcd(x.numerator(),x.denominator()),num::BigUint::from(1_u8));
                        zero_components+=u64::from(got==0);
                        components+=1;
                    }
                }
            }
            assert_eq!(before,input.r.each_ref().map(|x| x.to_string()));
            input_checks+=4;fixtures+=1;
        }}}
        println!("{{\"bits\":{bits},\"fixtures\":{fixtures},\"components\":{components},\"inputChecks\":{input_checks},\"zeroComponents\":{zero_components}}}");
    }
    assert_eq!(fixtures,6912);assert_eq!(components,82_944);assert_eq!(input_checks,27_648);
    println!("{{\"status\":\"pass\",\"fixtures\":{fixtures},\"components\":{components},\"inputChecks\":{input_checks},\"zeroComponents\":{zero_components}}}");
}
