use rug::{Float, Integer, Rational, float::Round};
use std::{collections::BTreeMap, io::{self, BufRead}};

fn ratio(n: &str, d: &str) -> Rational {
    Rational::from((n.parse::<Integer>().unwrap(), d.parse::<Integer>().unwrap()))
}

fn evaluate(op: &str, mut value: Float, rounding: Round) -> Rational {
    match op {
        "sqrt" => value.sqrt_round(rounding),
        "exp" => value.exp_round(rounding),
        "log" => value.ln_round(rounding),
        "sin" => value.sin_round(rounding),
        "cos" => value.cos_round(rounding),
        _ => panic!("unknown operation {op}"),
    };
    value.to_rational().expect("finite oracle result")
}

fn main() {
    let mut counts = BTreeMap::<String, [usize; 3]>::new();
    for (index, line) in io::stdin().lock().lines().enumerate() {
        let line = line.unwrap();
        let fields: Vec<_> = line.split_whitespace().collect();
        assert_eq!(fields.len(), 8);
        let [op, precision, xn, xd, ln, ld, un, ud] = fields[..] else { unreachable!() };
        let x = ratio(xn, xd);
        let lower = ratio(ln, ld);
        let upper = ratio(un, ud);
        assert!(lower <= upper);
        let (oracle_lower, oracle_upper) = if op == "sin" || op == "cos" {
            let point = Float::with_val(2048, &x);
            // Both sin and cos are globally 1-Lipschitz. This accounts for
            // rational input rounding without assuming local monotonicity.
            let input_error = (point.to_rational().unwrap() - &x).abs();
            (evaluate(op, point.clone(), Round::Down) - &input_error,
             evaluate(op, point, Round::Up) + &input_error)
        } else {
            let down = Float::with_val_round(2048, &x, Round::Down).0;
            let up = Float::with_val_round(2048, &x, Round::Up).0;
            (evaluate(op, down, Round::Down), evaluate(op, up, Round::Up))
        };
        assert!(oracle_lower <= oracle_upper);
        let tally = counts.entry(format!("{op}/{precision}")).or_default();
        if lower <= oracle_lower && oracle_upper <= upper {
            tally[0] += 1;
        } else if oracle_upper < lower || upper < oracle_lower {
            tally[1] += 1;
            eprintln!("FAIL line {} {op} p={precision} x={x}", index + 1);
        } else {
            tally[2] += 1;
            eprintln!("INDETERMINATE line {} {op} p={precision} x={x}", index + 1);
        }
    }
    let mut total = [0; 3];
    for (key, count) in counts {
        println!("{key}: {} pass, {} fail, {} indeterminate", count[0], count[1], count[2]);
        for i in 0..3 { total[i] += count[i]; }
    }
    println!("TOTAL: {} pass, {} fail, {} indeterminate", total[0], total[1], total[2]);
    if total[1] + total[2] != 0 { std::process::exit(1); }
}
