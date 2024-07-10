use realistic::Expression;
use std::io;

pub fn main() {
    loop {
        let mut input = String::new();

        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read calculator input");

        let expr = input.trim();
        let expr: Expression = expr.parse().expect("Parsing your input failed");
        if expr.is_empty() {
            break;
        }

        println!("expression parsed as: {expr:?}");

        let ans = expr.calculate();
        if let Ok(ans) = ans {
            println!("{ans} ~= {ans:#}");
        } else {
            println!("Calculation failed: {ans:?}");
        }
    }
}
