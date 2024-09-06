use realistic::{RealProblem, Simple};
use std::io;

pub fn main() {
    let debug_parse = false;

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

        if debug_parse {
            eprintln!("expression parsed as: {expr:?}");
        }

        use RealProblem::*;
        let ans = expr.evaluate();
        match ans {
            Ok(ans) => {
                if ans.is_whole() {
                    println!("answer: {ans}");
                } else if ans.is_rational() {
                    if ans.prefer_fraction() {
                        println!("answer: {ans} ~= {ans:#.10}");
                    } else {
                        println!("answer: {ans} = {ans:#}");
                    }
                } else {
                    println!("answer: {ans} ~= {ans:#.20}");
                }
            }
            Err(InsufficientParameters) => {
                println!("The operator needs more parameters")
            }
            Err(NotFound) => println!("Symbol not found"),
            Err(DivideByZero) => println!("Attempted division by zero"),
            _ => println!("Calculation failed: {ans:?}"),
        }
    }
}
