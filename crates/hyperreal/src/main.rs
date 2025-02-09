use realistic::{Problem, Real, Simple};
use std::collections::HashMap;
use std::io;

pub fn main() {
    let debug_parse = false;

    let mut names = HashMap::new();
    let mut n: u32 = 0;
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

        use Problem::*;
        let ans = expr.evaluate(&names);
        match ans {
            Ok(ans) => {
                n += 1;
                let name = format!("#{n}");
                answer(&ans, &name);
                names.insert("last".to_owned(), ans.clone());
                names.insert(name, ans);
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

fn answer(ans: &Real, name: &str) {
    if ans.is_integer() {
        println!("{name}: {ans}");
    } else if ans.is_rational() {
        if ans.prefer_fraction() {
            println!("{name}: {ans} ~= {ans:#}");
        } else {
            println!("{name}: {ans} = {ans:#}");
        }
    } else {
        println!("{name}: {ans} ~= {ans:#}");
    }
}
