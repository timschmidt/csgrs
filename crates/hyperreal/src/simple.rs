use crate::{Problem, Rational, Real};
use std::collections::HashMap;
use std::iter::Peekable;
use std::str::Chars;

type Symbols = HashMap<String, Real>;

#[derive(Clone, Debug, PartialEq)]
enum Operator {
    Plus,
    Minus,
    Star,
    Slash,
    Sqrt,
    Exp,
    Ln,
    Cos,
    Sin,
    Pow,
}

#[derive(Clone, Debug, PartialEq)]
enum Operand {
    Literal(Rational),     // e.g. 123_456.789
    Symbol(String),        // e.g. "pi"
    SubExpression(Simple), // e.g. (+ 1 2 3)
}

impl Operand {
    pub fn value(&self, names: &Symbols) -> Result<Real, Problem> {
        match self {
            Operand::Literal(n) => Ok(Real::new(n.clone())),
            Operand::Symbol(s) => Simple::lookup(s, names),
            Operand::SubExpression(xpr) => xpr.evaluate(names),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Simple {
    op: Operator,
    operands: Vec<Operand>,
}

fn parse_problem(problem: Problem) -> &'static str {
    use Problem::*;
    match problem {
        DivideByZero => "Attempting to divide by zero",
        NotFound => "Symbol not found",
        ParseError => "Unable to parse number",
        _ => {
            eprintln!("Specifically the problem is {problem:?}");
            "Some unknown problem during parsing"
        }
    }
}

impl Simple {
    fn lookup(name: &str, names: &Symbols) -> Result<Real, Problem> {
        if let Some(value) = names.get(name) {
            return Ok(value.clone());
        }
        match name {
            "pi" => Ok(Real::pi()),
            "e" => Ok(Real::e()),
            _ => Err(Problem::NotFound),
        }
    }

    pub fn evaluate(&self, names: &Symbols) -> Result<Real, Problem> {
        use Operator::*;
        match self.op {
            Plus => {
                let mut value = Real::zero();
                for operand in &self.operands {
                    value = value + operand.value(names)?;
                }
                Ok(value)
            }
            Minus => match self.operands.len() {
                0 => Err(Problem::InsufficientParameters),
                1 => {
                    let operand = self.operands.first().unwrap();
                    let value = -(operand.value(names)?);
                    Ok(value)
                }
                _ => {
                    let mut value: Real = self.operands.first().unwrap().value(names)?;
                    let operands = self.operands.iter().skip(1);
                    for operand in operands {
                        value = value - (operand.value(names)?);
                    }
                    Ok(value)
                }
            },
            Star => {
                let mut value = Real::new(Rational::one());
                for operand in &self.operands {
                    value = value * operand.value(names)?;
                }
                Ok(value)
            }
            Slash => match self.operands.len() {
                0 => Err(Problem::InsufficientParameters),
                1 => {
                    let operand = self.operands.first().unwrap();
                    operand.value(names)?.inverse()
                }
                _ => {
                    let mut value: Real = self.operands.first().unwrap().value(names)?;
                    let operands = self.operands.iter().skip(1);
                    for operand in operands {
                        let inverse = operand.value(names)?.inverse()?;
                        value = value * inverse;
                    }
                    Ok(value)
                }
            },
            Exp => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.exp()?;
                Ok(value)
            }
            Ln => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.ln()?;
                Ok(value)
            }
            Sqrt => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.sqrt()?;
                Ok(value)
            }
            Cos => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.cos();
                Ok(value)
            }
            Sin => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.sin();
                Ok(value)
            }
            Pow => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let op1 = &self.operands[0];
                let op2 = &self.operands[1];
                let v1 = op1.value(names)?;
                let v2 = op2.value(names)?;
                let value = v1.pow(v2)?;
                Ok(value)
            }
        }
    }

    fn operator(chars: &mut Peekable<Chars>) -> Result<Operator, &'static str> {
        let mut op = String::new();

        while let Some(c) = chars.peek() {
            match c {
                'A'..='Z' | 'a'..='z' => op.push(*c),
                _ => break,
            }
            chars.next();
        }
        op.make_ascii_lowercase();

        use Operator::*;
        match op.as_str() {
            "ln" | "l" => Ok(Ln),
            "exp" | "e" => Ok(Exp),
            "sqrt" | "s" => Ok(Sqrt),
            "cos" => Ok(Cos),
            "sin" => Ok(Sin),
            "pow" => Ok(Pow),
            _ => Err("No such operator"),
        }
    }

    pub fn parse(chars: &mut Peekable<Chars>) -> Result<Self, &'static str> {
        if let Some('(') = chars.peek() {
            chars.next();
        } else {
            return Err("No parenthetical expression");
        }

        use Operator::*;
        // One operator
        let op: Operator = match chars.peek() {
            Some('+') => {
                chars.next();
                Plus
            }
            Some('-') => {
                chars.next();
                Minus
            }
            Some('*') => {
                chars.next();
                Star
            }
            Some('/') => {
                chars.next();
                Slash
            }
            Some('^') => {
                chars.next();
                Pow
            }
            Some('√') => {
                chars.next();
                Sqrt
            }
            Some('a'..='z') => Self::operator(chars)?,
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
                '#' | 'a'..='z' => {
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

    // Consume a symbol, starting with # or a letter and consisting of zero or more:
    // letters, underscores or digits
    fn consume_symbol(c: &mut Peekable<Chars>) -> Operand {
        let mut sym = String::new();

        if let Some('#') = c.peek() {
            sym.push('#');
            c.next();
        }
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
    // digits, the decimal point or a slash and optionally commas, underscores etc. which are ignored
    fn consume_literal(c: &mut Peekable<Chars>) -> Result<Operand, Problem> {
        let mut num = String::new();

        if let Some('-') = c.peek() {
            num.push('-');
            c.next();
        }
        while let Some(item) = c.peek() {
            match item {
                '0'..='9' | '.' | '/' => num.push(*item),
                '_' | ',' | '\'' => { /* ignore */ }
                _ => break,
            }
            c.next();
        }

        let n: Rational = num.parse()?;

        Ok(Operand::Literal(n))
    }
}

impl std::str::FromStr for Simple {
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
    fn two() {
        let empty = HashMap::new();
        let xpr: Simple = "(* 1/3 15/4 1.6)".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result}");
        assert_eq!(ans, "2");
    }

    #[test]
    fn division_zero() {
        let empty = HashMap::new();
        let xpr: Simple = "(/ 0)".parse().unwrap();
        let result = xpr.evaluate(&empty);
        assert_eq!(result, Err(Problem::DivideByZero))
    }

    #[test]
    fn simple_arithmetic() {
        let empty = HashMap::new();
        let xpr: Simple = "(+ 1 (* 2 3) 4)".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        assert!(result.is_integer());
        let ans = format!("{result}");
        assert_eq!(ans, "11");
    }

    #[test]
    fn fractions() {
        let empty = HashMap::new();
        let xpr: Simple = "(/ (+ 1 2) (* 3 4))".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result}");
        assert_eq!(ans, "1/4");
        let decimal = format!("{result:e}");
        assert_eq!(decimal, "2.5e-1");
    }

    #[test]
    fn sqrts() {
        let empty = HashMap::new();
        let xpr: Simple = "(* (√ 40) (√ 90))".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result}");
        assert_eq!(ans, "60");
        let xpr: Simple = "(* (√ 14) (√ 1666350))".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result}");
        assert_eq!(ans, "4830");
    }

    #[test]
    fn sqrt_pi() {
        let empty = HashMap::new();
        let xpr: Simple = "(√ (+ pi pi pi pi))".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result:.32e}");
        assert_eq!(ans, "3.54490770181103205459633496668229e0");
    }

    #[test]
    fn pi() {
        let empty = HashMap::new();
        let xpr: Simple = "(* (+ pi pi) (* 3 pi))".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result:.32e}");
        assert_eq!(ans, "5.92176264065361517130069459992569e1");
    }

    #[test]
    fn pi_e_4() {
        let empty = HashMap::new();
        let xpr: Simple = "(* pi e 4)".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result:.32e}");
        assert_eq!(ans, "3.41589368906942682618542034781863e1");
    }

    #[test]
    fn ln_e() {
        let empty = HashMap::new();
        let xpr: Simple = "(l (* (e 4) (e 6)))".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        assert!(result.is_integer());
        let ans = format!("{result}");
        assert_eq!(ans, "10");
    }

    #[test]
    fn div_pi_e_4() {
        let empty = HashMap::new();
        let xpr: Simple = "(/ pi e 4)".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result:.32e}");
        assert_eq!(ans, "2.88931837447730429477523295828174e-1");
    }

    #[test]
    fn e_minus_one() {
        let empty = HashMap::new();
        let xpr: Simple = "(/ e)".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result:.32e}");
        assert_eq!(ans, "3.67879441171442321595523770161461e-1");
    }

    #[test]
    fn precision() {
        let empty = HashMap::new();
        let xpr: Simple =
            "(* 35088.93592003040493454779969771102629 35088.93592003040493454779969771102629)"
                .parse()
                .unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let ans = format!("{result:#.29}");
        assert_eq!(ans, "1231233424.00000000000000000000000000032");
    }
}
