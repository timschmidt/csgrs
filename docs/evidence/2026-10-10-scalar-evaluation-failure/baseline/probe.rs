use hypercurve::{PredicatePolicy, WorkBudget, evaluate_with_budget};
use hyperreal::{Computable, Rational};
use serde_json::json;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let family = args[0].as_str();
    let preparation = args[1].as_str();
    let operation = args[2].as_str();
    let precision: i32 = args[3].parse().unwrap();
    let rational = |n| Computable::rational(Rational::new(n));
    let mut value = match family {
        "one" => rational(1),
        "rational" => Computable::rational(Rational::fraction(7, 8).unwrap()),
        "pi" => Computable::pi(),
        "e" => Computable::e(),
        "sqrt_two" => rational(2).sqrt(),
        "ln_two" => rational(2).ln(),
        "sin_one" => rational(1).sin(),
        "cos_one" => rational(1).cos(),
        "pi_plus_e" => Computable::pi() + Computable::e(),
        "pi_times_e" => Computable::pi() * Computable::e(),
        _ => panic!("unknown family"),
    };
    if preparation == "warm" {
        std::hint::black_box(value.approx(precision));
    }
    let signal = Arc::new(AtomicBool::new(preparation != "control"));
    value.abort(signal.clone());
    let evaluate = || match operation {
        "approx" | "curve_scope" => value.approx(precision),
        "explicit_signal" => value.approx_signal(&Some(signal.clone()), precision),
        "near_integer" => value.near_integer(),
        _ => panic!("unknown operation"),
    };
    let (result, scope) = if operation == "curve_scope" {
        let outcome =
            evaluate_with_budget(PredicatePolicy::STRICT, WorkBudget::ZERO, evaluate);
        let usage = format!("{:?}", outcome.usage);
        let result = outcome.result.unwrap().certified().unwrap();
        (result, Some(usage))
    } else {
        (evaluate(), None)
    };
    let text = (result.bits() <= 4096).then(|| result.to_string());
    println!(
        "{}",
        json!({"family":family,"preparation":preparation,"operation":operation,"precision":precision,"aborted":signal.load(Ordering::Relaxed),"result":text,"bits":result.bits(),"curve_scope_success":scope})
    );
    signal.store(false, Ordering::Relaxed);
    let retried = evaluate();
    println!(
        "{}",
        json!({"retry":true,"result":(retried.bits()<=4096).then(||retried.to_string()),"bits":retried.bits()})
    );
}
