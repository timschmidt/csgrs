use hyperreal::{Computable, Rational, Real, RealSign};
use rug::{Float, Integer, float::{Constant, Round}};

fn main() {
    let mut decimal_cases=0;
    for n in -33..=33 {
        for d in [1,3,7,32,1024] {
            let q=Rational::fraction(n,d).unwrap();
            for digits in [0,1,2,3,6,16,32] {
                let value=Computable::rational(q.clone());
                let output=format!("{value:.digits$}");
                let actual: Rational=output.parse().unwrap();
                let error=if actual>=q {&actual-&q} else {&q-&actual};
                let mut tolerance=Rational::one();
                for _ in 0..digits { tolerance=tolerance/Rational::from(10); }
                assert!(error<=tolerance,"{q} digits {digits}: {output}");
                decimal_cases+=1;
            }
        }
    }
    let mut angle_cases=0;
    for k in [0,10,100,300,2500] {
        let mut tiny=Rational::one();
        for _ in 0..k {tiny=tiny/Rational::from(2);}
        for y_sign in [-1,1] {
            for x_sign in [-1,1] {
                let y=tiny.clone()*Rational::from(y_sign);
                let x=tiny.clone()*Rational::from(x_sign);
                let real_y=Real::new(y.clone());
                assert_eq!(real_y.certified_sign_until(-2).sign(),Some(if y_sign>0 {RealSign::Positive} else {RealSign::Negative}));
                let angle=Computable::rational(y).try_atan2_until(Computable::rational(x),-2).unwrap();
                let coefficient: i32=if x_sign>0 {y_sign} else {3*y_sign};
                for bits in [2,16,128,512,16,2] {
                    let got=Float::with_val(2048,Integer::from_str_radix(&angle.approx(-bits).to_string(),10).unwrap());
                    let mut lo=Float::with_val_round(2048,Constant::Pi,Round::Down).0;
                    let mut hi=Float::with_val_round(2048,Constant::Pi,Round::Up).0;
                    lo=Float::with_val_round(2048,lo*coefficient.abs(),Round::Down).0;
                    hi=Float::with_val_round(2048,hi*coefficient.abs(),Round::Up).0;
                    if coefficient<0 {(lo,hi)=(-hi,-lo);}
                    lo<<=bits-2;hi<<=bits-2;
                    assert!(Float::with_val(2048,&got-lo).abs()<=1 && Float::with_val(2048,&got-hi).abs()<=1,"angle k={k}, {y_sign}/{x_sign} p={bits}");
                    angle_cases+=1;
                }
            }
        }
    }
    println!("PASS {decimal_cases} exact-rational decimal bounds; {angle_cases} MPFR tiny-quadrant/history controls; 20 certified signs");
}
