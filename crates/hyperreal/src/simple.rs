use crate::{BoundedRational, Real, RealProblem};
use std::iter::Peekable;
use std::str::Chars;

#[derive(Clone, Debug, PartialEq)]
enum Operator {
    Plus,
    Minus,
    Star,
    Slash,
    Sqrt,
    Exp,
    Ln,
}

#[derive(Clone, Debug, PartialEq)]
enum Operand {
    Literal(BoundedRational), // e.g. 123_456.789
    Symbol(String),           // e.g. "pi"
    SubExpression(Simple),    // e.g. (+ 1 2 3)
}

impl Operand {
    pub fn value(&self) -> Result<Real, RealProblem> {
        match self {
            Operand::Literal(n) => Ok(Real::new(n.clone())),
            Operand::Symbol(s) => Simple::lookup(s),
            Operand::SubExpression(xpr) => xpr.evaluate(),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Simple {
    op: Operator,
    operands: Vec<Operand>,
}

fn parse_problem(problem: RealProblem) -> &'static str {
    match problem {
        RealProblem::DivideByZero => "Attempting to divide by zero",
        RealProblem::NotFound => "Symbol not found",
        RealProblem::ParseError => "Unable to parse number",
        _ => {
            eprintln!("Specifically the problem is {problem:?}");
            "Some unknown problem during parsing"
        }
    }
}

impl Simple {
    fn lookup(name: &str) -> Result<Real, RealProblem> {
        match name {
            "pi" => Ok(Real::pi()),
            "e" => Ok(Real::e()),
            _ => Err(RealProblem::NotFound),
        }
    }

    pub fn evaluate(&self) -> Result<Real, RealProblem> {
        match self.op {
            Operator::Plus => {
                let mut value = Real::zero();
                for operand in &self.operands {
                    value = value + operand.value()?;
                }
                Ok(value)
            }
            Operator::Minus => match self.operands.len() {
                0 => Err(RealProblem::InsufficientParameters),
                1 => {
                    let operand = self.operands.first().unwrap();
                    let value = -(operand.value()?);
                    Ok(value)
                }
                _ => {
                    let mut value: Real = self.operands.first().unwrap().value()?;
                    let operands = self.operands.iter().skip(1);
                    for operand in operands {
                        value = value - (operand.value()?);
                    }
                    Ok(value)
                }
            },
            Operator::Star => {
                let mut value = Real::new(BoundedRational::one());
                for operand in &self.operands {
                    value = value * operand.value()?;
                }
                Ok(value)
            }
            Operator::Slash => match self.operands.len() {
                0 => Err(RealProblem::InsufficientParameters),
                1 => {
                    let operand = self.operands.first().unwrap();
                    operand.value()?.inverse()
                }
                _ => {
                    let mut value: Real = self.operands.first().unwrap().value()?;
                    let operands = self.operands.iter().skip(1);
                    for operand in operands {
                        value = (value / operand.value()?)?;
                    }
                    Ok(value)
                }
            },
            Operator::Exp => {
                if self.operands.len() != 1 {
                    return Err(RealProblem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value()?.exp()?;
                Ok(value)
            }
            Operator::Ln => {
                if self.operands.len() != 1 {
                    return Err(RealProblem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value()?.ln()?;
                Ok(value)
            }
            Operator::Sqrt => {
                if self.operands.len() != 1 {
                    return Err(RealProblem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value()?.sqrt()?;
                Ok(value)
            }
        }
    }

    pub fn parse(chars: &mut Peekable<Chars>) -> Result<Self, &'static str> {
        if let Some('(') = chars.peek() {
            chars.next();
        } else {
            return Err("No parenthetical expression");
        }

        // One operator
        let op: Operator = match chars.peek() {
            Some('+') => {
                chars.next();
                Operator::Plus
            }
            Some('-') => {
                chars.next();
                Operator::Minus
            }
            Some('*') => {
                chars.next();
                Operator::Star
            }
            Some('/') => {
                chars.next();
                Operator::Slash
            }
            Some('l') => {
                chars.next();
                Operator::Ln
            }
            Some('e') => {
                chars.next();
                Operator::Exp
            }
            Some('√' | 's') => {
                chars.next();
                Operator::Sqrt
            }
            _ => return Err("Unexpected symbol while looking for an operator"),
        };

        // One whitespace character
        match chars.peek() {
            Some(' ' | '\t') => {
                chars.next();
            }
            _ => return Err("No whitespace after operator"),
        }

        let mut operands: Vec<Operand> = Vec::new();

        // Operands
        while let Some(c) = chars.peek() {
            match c {
                ' ' | '\t' => {
                    // ignore
                    chars.next();
                }
                'a'..='z' => {
                    let operand = Self::consume_symbol(chars);
                    operands.push(operand);
                }
                '-' | '0'..='9' => {
                    let operand = Self::consume_literal(chars).map_err(parse_problem)?;
                    operands.push(operand);
                }
                '(' => {
                    let xpr = Self::parse(chars)?;
                    operands.push(Operand::SubExpression(xpr));
                }
                ')' => {
                    chars.next();
                    return Ok(Simple { op, operands });
                }
                _ => return Err("Unexpected character while looking for operands ..."),
            }
        }

        Err("Incomplete expression")
    }

    // Consume a symbol, starting with a letter and consisting of zero or more:
    // letters, underscores or digits
    fn consume_symbol(c: &mut Peekable<Chars>) -> Operand {
        let mut sym = String::new();

        while let Some(item) = c.peek() {
            match item {
                'A'..='Z' | 'a'..='z' | '0'..='9' => sym.push(*item),
                _ => break,
            }
            c.next();
        }

        Operand::Symbol(sym)
    }

    // Consume a literal, for now presumably a single number consisting of:
    // a possible leading minus symbol, then
    // digits, the decimal point and optionally commas, underscores etc. which are ignored
    fn consume_literal(c: &mut Peekable<Chars>) -> Result<Operand, RealProblem> {
        let mut num = String::new();

        if let Some('-') = c.peek() {
            num.push('-');
            c.next();
        }
        while let Some(item) = c.peek() {
            match item {
                '0'..='9' | '.' => num.push(*item),
                '_' | ',' | '\'' => { /* ignore */ }
                _ => break,
            }
            c.next();
        }

        let n: BoundedRational = num.parse().map_err(|_| RealProblem::ParseError)?;

        Ok(Operand::Literal(n))
    }
}

use std::str::FromStr;
impl FromStr for Simple {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let mut chars = s.chars().peekable();
        Simple::parse(&mut chars)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_close() {
        let xpr: Result<Simple, &str> = "(+ (* (e 4) (e 6))".parse();
        assert_eq!(xpr, Err("Incomplete expression"))
    }

    #[test]
    fn division_zero() {
        let xpr: Simple = "(/ 0)".parse().unwrap();
        let result = xpr.evaluate();
        assert_eq!(result, Err(RealProblem::DivideByZero))
    }

    #[test]
    fn simple_arithmetic() {
        let xpr: Simple = "(+ 1 (* 2 3) 4)".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        assert!(result.is_whole());
        let ans = format!("{result}");
        assert_eq!(ans, "11");
    }

    #[test]
    fn fractions() {
        let xpr: Simple = "(/ (+ 1 2) (* 3 4))".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        let ans = format!("{result}");
        assert_eq!(ans, "1/4");
        let decimal = format!("{result:#}");
        assert_eq!(decimal, "0.25");
    }

    #[test]
    fn sqrts() {
        let xpr: Simple = "(* (√ 40) (√ 90))".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        let ans = format!("{result}");
        assert_eq!(ans, "60");
    }

    #[test]
    fn sqrt_pi() {
        let xpr: Simple = "(√ (+ pi pi pi pi))".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        let ans = format!("{result:#.32}");
        assert_eq!(ans, "3.54490770181103205459633496668229...");
    }

    #[test]
    fn pi() {
        let xpr: Simple = "(* (+ pi pi) (* 3 pi))".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        let ans = format!("{result:#.32}");
        assert_eq!(ans, "59.21762640653615171300694599925690...");
    }

    #[test]
    fn pi_e_4() {
        let xpr: Simple = "(* pi e 4)".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        let ans = format!("{result:#.32}");
        assert_eq!(ans, "34.15893689069426826185420347818629...");
    }

    #[test]
    fn ln_e() {
        let xpr: Simple = "(l (* (e 4) (e 6)))".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        assert!(result.is_whole());
        let ans = format!("{result}");
        assert_eq!(ans, "10");
    }

    #[test]
    fn div_pi_e_4() {
        let xpr: Simple = "(/ pi e 4)".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        let ans = format!("{result:#.32}");
        assert_eq!(ans, "0.28893183744773042947752329582817...");
    }

    #[test]
    fn e_minus_one() {
        let xpr: Simple = "(/ e)".parse().unwrap();
        let result = xpr.evaluate().unwrap();
        let ans = format!("{result:#.32}");
        assert_eq!(ans, "0.36787944117144232159552377016146...");
    }
}
