use realistic::{Expression,RealProblem};
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
        match ans {
            Ok(ans) => println!("{ans} ~= {ans:#}"),
            Err(RealProblem::NotFound) => println!("Symbol not found"),
            Err(RealProblem::DivideByZero) => println!("Attempted division by zero"),
            _ => println!("Calculation failed: {ans:?}"),
        }
    }
}
