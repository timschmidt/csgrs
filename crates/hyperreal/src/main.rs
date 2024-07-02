use realistic::BoundedRational;
use realistic::Real;
use std::io;

use realistic::Expression;

pub fn main() {
    loop {
        let mut input = String::new();

        io::stdin()
            .read_line(&mut input)
            .expect("Failed to read calculator input");

        let expr = input.trim();
        println!("input was '{expr}'");
        let expr: Expression = expr.parse().expect("Parsing your input failed");
//        let expr = example1(Real::new(num));
        println!("expression parsed as: {expr:?}");

        let ans = expr.calculate().expect("Calculation should work");

        println!("{ans}");
    }
}
