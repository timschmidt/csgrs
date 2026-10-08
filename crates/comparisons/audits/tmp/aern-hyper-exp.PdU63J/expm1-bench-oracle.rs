use hyperreal::{Computable, Rational};
use rug::{Float, Integer, Rational as R, float::Round};
fn main() {
    let cases = [(1,1000000,-128),(1,4,-128),(1,2,-128),(7,2,-128),
        (-7,2,-128),(10000,1,-128),(-10000,1,-128),
        (1,4,1),(-32,1,1),(3,2,1)];
    let mut failures = 0;
    for (n,d,p) in cases {
        let input = Rational::fraction(n,d).unwrap();
        let oracle = R::from((n,d));
        let bits = 16384;
        let mut low = Float::with_val_round(bits,&oracle,Round::Down).0;
        let mut high = Float::with_val_round(bits,&oracle,Round::Up).0;
        low.exp_m1_round(Round::Down);
        high.exp_m1_round(Round::Up);
        low >>= p;
        high >>= p;
        let actual = Computable::rational(input).expm1().approx(p);
        let actual = Float::with_val(bits,Integer::from_str_radix(&actual.to_string(),10).unwrap());
        let error = Float::with_val(bits,&actual-low).abs()
            .max(&Float::with_val(bits,&actual-high).abs());
        println!("{n}/{d} p={p}: error={:.8} valid={}",error.to_f64(),error<=1);
        if error>1 { failures+=1; }
    }
    println!("{failures} invalid benchmark answers");
}
