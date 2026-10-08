use hyperreal::{Computable, Rational};
use rug::{Float, Integer, Rational as R, float::Round};

fn main() {
    let expm1 = std::env::args().nth(1).is_some_and(|s| s == "expm1");
    let operation = if expm1 { "expm1" } else { "exp" };
    let mut failures = 0;
    let mut cases = 0;
    for denominator in [2_u64, 3, 7, 16] {
        for numerator in -8_i64 * denominator as i64..=8 * denominator as i64 {
            let exact = Rational::fraction(numerator, denominator).unwrap();
            let oracle = R::from((numerator, denominator));
            let mut low = Float::with_val_round(2048, &oracle, Round::Down).0;
            let mut high = Float::with_val_round(2048, &oracle, Round::Up).0;
            if expm1 {
                low.exp_m1_round(Round::Down);
                high.exp_m1_round(Round::Up);
            } else {
                low.exp_round(Round::Down);
                high.exp_round(Round::Up);
            }
            for precision in [4_i32, 1, 0, -1, -8, -32, -64, -128, -256] {
                let input = Computable::rational(exact.clone());
                let value = if expm1 { input.expm1() } else { input.exp() };
                let computed = value.approx(precision);
                let value = Float::with_val(2048, Integer::from_str_radix(&computed.to_string(), 10).unwrap());
                let mut lower = low.clone();
                let mut upper = high.clone();
                lower >>= precision;
                upper >>= precision;
                let error = Float::with_val(2048, &value - lower).abs()
                    .max(&Float::with_val(2048, &value - upper).abs());
                cases += 1;
                if error > 1 {
                    failures += 1;
                    if failures <= 16 {
                        println!("{operation}({numerator}/{denominator}) p={precision}: actual={computed}, error={:.6} ulps", error.to_f64());
                    }
                }
            }
        }
    }
    println!("operation={operation}, cases={cases}, one-ulp violations={failures}");
    if failures != 0 { std::process::exit(1); }
}
