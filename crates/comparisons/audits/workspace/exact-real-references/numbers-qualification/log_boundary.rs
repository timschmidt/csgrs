use hyperreal::{Computable,Rational};
use rug::{Float,Integer,Rational as Q,float::Round};
fn main() {
    let mut checks=0;let mut failed=0;
    for (op,radical,n) in [("asinh",false,2),("asinh",false,-2),("asinh",false,4),("asinh",false,6),
        ("asinh",true,2),("asinh",true,5),("asinh",true,100),
        ("acosh",true,2),("acosh",true,5),("acosh",true,17),("acosh",true,100)] {
        let mut lower=Float::with_val(4096,n);let mut upper=lower.clone();
        if radical {lower.sqrt_round(Round::Down);upper.sqrt_round(Round::Up);}
        match op {"asinh"=>{lower.asinh_round(Round::Down);upper.asinh_round(Round::Up);},
            "acosh"=>{assert!(lower>=1);lower.acosh_round(Round::Down);upper.acosh_round(Round::Up);},_=>unreachable!()}
        let lower=lower.to_rational().unwrap();let upper=upper.to_rational().unwrap();
        for bits in [0,1,2,8,32,80,160] {
            let input=Computable::rational(Rational::from(n));let input=if radical{input.sqrt()}else{input};
            let value=if op=="asinh"{input.asinh()}else{input.acosh()};
            let scale=Integer::from(1)<<bits;
            let center=Q::from((Integer::from_str_radix(&value.approx(-bits).to_string(),10).unwrap(),scale.clone()));
            let radius=Q::from((1,scale));
            let ok=Q::from(&center-&radius)<=lower&&Q::from(&center+&radius)>=upper;
            if !ok {assert!(Q::from(&center-&radius)>upper||Q::from(&center+&radius)<lower,"inconclusive oracle overlap");failed+=1;}
            checks+=1;
            println!("{} {op} radical={radical} input={n} bits={bits} error_ulps={}",if ok{"PASS"}else{"FAIL"},Q::from((center-&lower)/radius).to_f64());
        }
    }
    println!("TOTAL log-boundary checks={checks} failures={failed}");
}
