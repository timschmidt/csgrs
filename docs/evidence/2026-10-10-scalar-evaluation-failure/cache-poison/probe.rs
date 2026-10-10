use hyperreal::{Computable, Rational, Real, ScientificCase};
use serde_json::json;

fn main() {
    let mode = std::env::args().nth(1).unwrap();
    if mode == "warm-poison" {
        std::hint::black_box(Computable::pi().approx(-256));
    }
    let extreme = if mode == "control" {
        None
    } else {
        Some(Computable::pi().approx(i32::MIN).to_string())
    };
    let ordinary = [0, -64, -256].map(
        |p| json!({"precision":p,"approximation":Computable::pi().approx(p).to_string()}),
    );
    let [lower, upper] = Real::pi().certified_rational_interval(-64).unwrap();
    println!(
        "{}",
        json!({
            "mode":mode,"extreme_result":extreme,"ordinary":ordinary,
            "interval_lower":lower.to_string(),"interval_upper":upper.to_string(),
            "interval_excludes_pi":upper<Rational::new(3) || lower>Rational::new(4),
            "checked_scientific":format!("{:?}",Real::pi().try_scientific(6,-256,ScientificCase::Lower)),
            "abort_signal_was_used":false
        })
    );
}
