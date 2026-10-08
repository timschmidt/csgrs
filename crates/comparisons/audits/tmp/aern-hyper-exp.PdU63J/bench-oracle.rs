use hyperreal::{Computable, Rational};
use rug::{Float, Integer, Rational as R, float::Round};
fn main() {
    let cases = [(1,4,-128),(1,2,-128),(7,5,-128),(7,2,-128),(15,2,-128),
        (-7,2,-128),(2,1,-128),(128,1,-128),(257,1,-128),(-32,1,-128),(3,2,1)];
    let mut failures = 0;
    for (n,d,p) in cases {
        let input = Rational::fraction(n,d).unwrap();
        let oracle = R::from((n,d));
        let mut low = Float::with_val_round(2048,&oracle,Round::Down).0;
        let mut high = Float::with_val_round(2048,&oracle,Round::Up).0;
        low.exp_round(Round::Down);
        high.exp_round(Round::Up);
        low >>= p;
        high >>= p;
        let actual = Computable::rational(input).exp().approx(p);
        let actual = Float::with_val(2048,Integer::from_str_radix(&actual.to_string(),10).unwrap());
        let error = Float::with_val(2048,&actual-low).abs()
            .max(&Float::with_val(2048,&actual-high).abs());
        println!("{n}/{d} p={p}: error={:.8} valid={}",error.to_f64(),error<=1);
        if error>1 { failures+=1; }
    }
    println!("{failures} invalid benchmark answers");
}
