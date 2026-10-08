use hyperreal::{Computable, Rational, Real};

fn pow2(exponent: i32) -> Rational {
    let mut value=Rational::one();
    for _ in 0..exponent.unsigned_abs() {value=value*Rational::from(2);}
    if exponent<0 {Rational::one()/value} else {value}
}

fn check(value: &Computable, exact: &Rational, bits: i32) {
    let got: Rational=value.approx(-bits).to_string().parse().unwrap();
    let expected=exact*pow2(bits);
    let error=if got>=expected {got-expected} else {expected-got};
    assert!(error<=Rational::one(),"error={error}, exact={exact}, bits={bits}");
}

fn main() {
    let mut integer_checks=0;
    let mut abs_checks=0;
    let mut arithmetic_checks=0;
    let mut scale_checks=0;
    for exponent in [8,32,60,63,64,65,128,256,1024] {
        for offset in [-17,-1,0,1,17] {
            for sign in [-1,1] {
                let q=(pow2(exponent)+Rational::from(offset))*Rational::from(sign);
                let value=Computable::rational(q.clone());
                let real=Real::new(q.clone());
                let magnitude=if sign<0 {-q.clone()} else {q.clone()};
                assert_eq!(real.abs(),Real::new(magnitude));
                abs_checks+=1;
                for bits in [-4,0,2,16,64,128,512,16,0] {
                    check(&value,&q,bits);
                    integer_checks+=1;
                }
            }
        }
    }
    for m in -257..=257 {
        for denominator in [1,3,32,1024] {
            let q=Rational::fraction(m,denominator).unwrap();
            let r=Real::new(q.clone());
            assert_eq!(r.abs(),Real::new(Rational::fraction(m.abs(),denominator).unwrap()));
            abs_checks+=1;
            let c=Computable::rational(q.clone());
            for shift in [-128,-8,-1,0,1,8,128] {
                let scale=pow2(shift);
                let scaled=c.clone().multiply(Computable::rational(scale.clone()));
                for bits in [2,16,128,2] {
                    check(&scaled,&(&q*&scale),bits);
                    scale_checks+=1;
                }
            }
        }
    }
    for n in -15..=15 {
        for m in -15..=15 {
            let q=Rational::fraction(n,8).unwrap();
            let r=Rational::fraction(m,8).unwrap();
            let x=Computable::rational(q.clone());
            let y=Computable::rational(r.clone());
            let mut cases=vec![(x.clone().add(y.clone()),&q+&r),
                               (x.clone().add(y.clone().negate()),&q-&r),
                               (x.clone().multiply(y.clone()),&q*&r)];
            if m!=0 {cases.push((x.multiply(y.inverse()),&q/&r));}
            for (value,exact) in cases {
                for bits in [0,2,16,128,2] {
                    check(&value,&exact,bits);
                    arithmetic_checks+=1;
                }
            }
        }
    }
    println!("PASS integer/history={integer_checks}; exact Real::abs={abs_checks}; dyadic-scale/history={scale_checks}; rational-arithmetic/history={arithmetic_checks}");
}
