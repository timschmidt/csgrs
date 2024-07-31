use realistic::{RealProblem, Simple};
use std::io;

pub fn main() {
    loop {
        let mut input = String::new();

        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read calculator input");

        let expr = input.trim();
        if expr.is_empty() {
            break;
        }

        let expr: Simple = match expr.parse() {
            Ok(parsed) => parsed,
            Err(text) => {
                eprintln!("Parsing your input failed: {text}");
                continue;
            }
        };

        println!("expression parsed as: {expr:?}");

        let ans = expr.evaluate();
        match ans {
            Ok(ans) => {
                if ans.is_whole() {
                    println!("{ans}");
                } else if ans.prefer_decimal() {
                    println!("{ans} ~= {ans:#.20}");
                } else {
                    println!("{ans} ~= {ans:#.5}");
                }
            }
            Err(RealProblem::InsufficientParameters) => {
                println!("The operator needs more parameters")
            }
            Err(RealProblem::NotFound) => println!("Symbol not found"),
            Err(RealProblem::DivideByZero) => println!("Attempted division by zero"),
            _ => println!("Calculation failed: {ans:?}"),
        }
    }
}
