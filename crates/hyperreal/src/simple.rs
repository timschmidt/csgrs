//! A Lisp-like expression parser for mathematical expressions.
//!
//! This module parses and evaluates prefix-notation expressions with operators like
//! `+`, `-`, `*`, `/`, `sqrt`, `sin`, `cos`, `pow`, etc.
//!
//! # Examples
//!
//! Basic arithmetic:
//!
//! ```
//! # use hyperreal::Simple;
//! use std::collections::HashMap;
//! let expr: Simple = "(+ 1 2 3)".parse().unwrap();
//! let result = expr.evaluate(&HashMap::new()).unwrap();
//! assert_eq!(result.to_string(), "6");
//! ```
//!
//! Nested expressions:
//!
//! ```
//! # use hyperreal::Simple;
//! use std::collections::HashMap;
//! let expr: Simple = "(* (+ 1 2) (- 5 3))".parse().unwrap();
//! let result = expr.evaluate(&HashMap::new()).unwrap();
//! assert_eq!(result.to_string(), "6");
//! ```
//!
//! Mathematical constants and functions:
//!
//! ```
//! # use hyperreal::Simple;
//! use std::collections::HashMap;
//! let expr: Simple = "(√ (+ pi pi))".parse().unwrap();
//! let result = expr.evaluate(&HashMap::new()).unwrap();
//! assert_eq!(format!("{result:.4e}"), "2.5066e0");
//! ```

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
    Log10,
    Ln,
    Cos,
    Sin,
    Tan,
    Acos,
    Asin,
    Atan,
    Acosh,
    Asinh,
    Atanh,
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

    fn exact_value(&self, names: &Symbols) -> Result<Option<Rational>, Problem> {
        match self {
            Operand::Literal(n) => Ok(Some(n.clone())),
            Operand::Symbol(s) => Ok(Simple::lookup_exact(s, names)),
            Operand::SubExpression(xpr) => xpr.evaluate_exact(names),
        }
    }

    fn literal(&self) -> Option<&Rational> {
        match self {
            Operand::Literal(n) => Some(n),
            _ => None,
        }
    }

    fn could_be_exact(&self) -> bool {
        match self {
            Operand::Literal(_) => true,
            Operand::Symbol(s) => s != "pi" && s != "e",
            Operand::SubExpression(xpr) => xpr.could_evaluate_exact(),
        }
    }
}

/// An expression consisting of an operator and operands.
/// These are typically constructed by parsing a string.
///
/// ```rust
/// # use hyperreal::Simple;
/// let expression: Simple = "(+ 1 4)".parse().unwrap();
/// assert_eq!(format!("{:?}", expression), "Simple { op: Plus, operands: [Literal(Rational { sign: Plus, numerator: 1, denominator: 1 }), Literal(Rational { sign: Plus, numerator: 4, denominator: 1 })] }");
/// ```
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
        match name {
            "pi" => Ok(Real::pi()),
            "e" => Ok(Real::e()),
            _ => names.get(name).cloned().ok_or(Problem::NotFound),
        }
    }

    fn lookup_exact(name: &str, names: &Symbols) -> Option<Rational> {
        match name {
            "pi" | "e" => None,
            _ => names.get(name).and_then(Real::exact_rational),
        }
    }

    fn evaluate_exact(&self, names: &Symbols) -> Result<Option<Rational>, Problem> {
        use Operator::*;
        match self.op {
            Plus => {
                let mut operands = self.operands.iter();
                let Some(first) = operands.next() else {
                    return Ok(Some(Rational::zero()));
                };
                let Some(mut value) = first.exact_value(names)? else {
                    return Ok(None);
                };
                for operand in operands {
                    let Some(exact) = operand.exact_value(names)? else {
                        return Ok(None);
                    };
                    value = value + exact;
                }
                Ok(Some(value))
            }
            Minus => match self.operands.len() {
                0 => Err(Problem::InsufficientParameters),
                1 => Ok(self.operands[0].exact_value(names)?.map(|value| -value)),
                _ => {
                    let Some(mut value) = self.operands[0].exact_value(names)? else {
                        return Ok(None);
                    };
                    for operand in self.operands.iter().skip(1) {
                        let Some(exact) = operand.exact_value(names)? else {
                            return Ok(None);
                        };
                        value = value - exact;
                    }
                    Ok(Some(value))
                }
            },
            Star => {
                let mut operands = self.operands.iter();
                let Some(first) = operands.next() else {
                    return Ok(Some(Rational::one()));
                };
                let Some(mut value) = first.exact_value(names)? else {
                    return Ok(None);
                };
                for operand in operands {
                    let Some(exact) = operand.exact_value(names)? else {
                        return Ok(None);
                    };
                    value = value * exact;
                }
                Ok(Some(value))
            }
            Slash => match self.operands.len() {
                0 => Err(Problem::InsufficientParameters),
                1 => Ok(self.operands[0]
                    .exact_value(names)?
                    .map(|value| value.inverse())
                    .transpose()?),
                _ => {
                    let Some(mut value) = self.operands[0].exact_value(names)? else {
                        return Ok(None);
                    };
                    for operand in self.operands.iter().skip(1) {
                        let Some(exact) = operand.exact_value(names)? else {
                            return Ok(None);
                        };
                        if exact.sign() == num::bigint::Sign::NoSign {
                            return Err(Problem::DivideByZero);
                        }
                        value = value / exact;
                    }
                    Ok(Some(value))
                }
            },
            Pow => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let Some(base) = self.operands[0].exact_value(names)? else {
                    return Ok(None);
                };
                let Some(exponent) = self.operands[1].exact_value(names)? else {
                    return Ok(None);
                };
                let Some(exponent) = exponent.to_big_integer() else {
                    return Ok(None);
                };
                Ok(Some(base.powi(exponent)?))
            }
            _ => Ok(None),
        }
    }

    fn could_evaluate_exact(&self) -> bool {
        use Operator::*;
        match self.op {
            Plus | Minus | Star | Slash | Pow => self.operands.iter().all(Operand::could_be_exact),
            _ => false,
        }
    }

    pub fn evaluate(&self, names: &Symbols) -> Result<Real, Problem> {
        use Operator::*;
        match self.op {
            Plus => {
                if self.could_evaluate_exact()
                    && let Some(value) = self.evaluate_exact(names)?
                {
                    return Ok(Real::new(value));
                }
                if let Some(first) = self.operands.first().and_then(Operand::literal) {
                    let mut value = first.clone();
                    let literals = self.operands.iter().skip(1);
                    if literals.clone().all(|operand| operand.literal().is_some()) {
                        for operand in literals {
                            value = value + operand.literal().unwrap();
                        }
                        return Ok(Real::new(value));
                    }
                }
                let mut operands = self.operands.iter();
                let Some(first) = operands.next() else {
                    return Ok(Real::zero());
                };
                let mut value = first.value(names)?;
                for operand in operands {
                    value = value + operand.value(names)?;
                }
                Ok(value)
            }
            Minus => match self.operands.len() {
                0 => Err(Problem::InsufficientParameters),
                1 => {
                    if self.could_evaluate_exact()
                        && let Some(value) = self.evaluate_exact(names)?
                    {
                        return Ok(Real::new(value));
                    }
                    let operand = self.operands.first().unwrap();
                    if let Some(literal) = operand.literal() {
                        return Ok(Real::new(-literal.clone()));
                    }
                    let value = -(operand.value(names)?);
                    Ok(value)
                }
                _ => {
                    if self.could_evaluate_exact()
                        && let Some(value) = self.evaluate_exact(names)?
                    {
                        return Ok(Real::new(value));
                    }
                    if let Some(first) = self.operands.first().and_then(Operand::literal) {
                        let mut value = first.clone();
                        let literals = self.operands.iter().skip(1);
                        if literals.clone().all(|operand| operand.literal().is_some()) {
                            for operand in literals {
                                value = value - operand.literal().unwrap();
                            }
                            return Ok(Real::new(value));
                        }
                    }
                    let mut value: Real = self.operands.first().unwrap().value(names)?;
                    let operands = self.operands.iter().skip(1);
                    for operand in operands {
                        value = value - (operand.value(names)?);
                    }
                    Ok(value)
                }
            },
            Star => {
                if self.could_evaluate_exact()
                    && let Some(value) = self.evaluate_exact(names)?
                {
                    return Ok(Real::new(value));
                }
                if let Some(first) = self.operands.first().and_then(Operand::literal) {
                    let mut value = first.clone();
                    let literals = self.operands.iter().skip(1);
                    if literals.clone().all(|operand| operand.literal().is_some()) {
                        for operand in literals {
                            value = value * operand.literal().unwrap();
                        }
                        return Ok(Real::new(value));
                    }
                }
                let mut operands = self.operands.iter();
                let Some(first) = operands.next() else {
                    return Ok(Real::new(Rational::one()));
                };
                let mut value = first.value(names)?;
                for operand in operands {
                    value = value * operand.value(names)?;
                }
                Ok(value)
            }
            Slash => match self.operands.len() {
                0 => Err(Problem::InsufficientParameters),
                1 => {
                    if self.could_evaluate_exact()
                        && let Some(value) = self.evaluate_exact(names)?
                    {
                        return Ok(Real::new(value));
                    }
                    let operand = self.operands.first().unwrap();
                    if let Some(literal) = operand.literal() {
                        return Ok(Real::new(literal.clone().inverse()?));
                    }
                    operand.value(names)?.inverse()
                }
                _ => {
                    if self.could_evaluate_exact()
                        && let Some(value) = self.evaluate_exact(names)?
                    {
                        return Ok(Real::new(value));
                    }
                    if let Some(first) = self.operands.first().and_then(Operand::literal) {
                        let mut value = first.clone();
                        let literals = self.operands.iter().skip(1);
                        if literals.clone().all(|operand| operand.literal().is_some()) {
                            for operand in literals {
                                let literal = operand.literal().unwrap();
                                if literal.sign() == num::bigint::Sign::NoSign {
                                    return Err(Problem::DivideByZero);
                                }
                                value = value / literal;
                            }
                            return Ok(Real::new(value));
                        }
                    }
                    let mut value: Real = self.operands.first().unwrap().value(names)?;
                    let operands = self.operands.iter().skip(1);
                    for operand in operands {
                        value = (value / operand.value(names)?)?;
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
            Log10 => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.log10()?;
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
            Tan => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.tan()?;
                Ok(value)
            }
            Acos => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.acos()
            }
            Asin => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.asin()
            }
            Atan => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.atan()
            }
            Acosh => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.acosh()
            }
            Asinh => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.asinh()
            }
            Atanh => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.atanh()
            }
            Pow => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                if self.could_evaluate_exact()
                    && let Some(value) = self.evaluate_exact(names)?
                {
                    return Ok(Real::new(value));
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

    fn consume_operator_token(chars: &mut Peekable<Chars>) -> String {
        let mut token = String::new();
        while let Some(c) = chars.peek() {
            match c {
                'A'..='Z' | 'a'..='z' | '0'..='9' => token.push(*c),
                _ => break,
            }
            chars.next();
        }
        token
    }

    fn operator(chars: &mut Peekable<Chars>) -> Result<Operator, &'static str> {
        use Operator::*;
        match Self::consume_operator_token(chars).as_str() {
            "log10" | "log" => Ok(Log10),
            "ln" | "l" => Ok(Ln),
            "exp" | "e" => Ok(Exp),
            "sqrt" | "s" => Ok(Sqrt),
            "cos" => Ok(Cos),
            "sin" => Ok(Sin),
            "pow" => Ok(Pow),
            "tan" => Ok(Tan),
            "acos" => Ok(Acos),
            "asin" => Ok(Asin),
            "atan" => Ok(Atan),
            "acosh" => Ok(Acosh),
            "asinh" => Ok(Asinh),
            "atanh" => Ok(Atanh),
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
    use std::collections::HashMap;

    #[test]
    fn missing_close() {
        let xpr: Result<Simple, &str> = "(+ (* (e 4) (e 6))".parse();
        assert_eq!(xpr, Err("Incomplete expression"))
    }

    #[test]
    fn parse_named_operators() {
        let cases = [
            "(ln 5)",
            "(l 5)",
            "(log 5)",
            "(log10 5)",
            "(exp 5)",
            "(e 5)",
            "(sqrt 5)",
            "(s 5)",
            "(cos 5)",
            "(sin 5)",
            "(tan 5)",
            "(acos 1/2)",
            "(asin 1/2)",
            "(atan 1)",
            "(acosh 1)",
            "(asinh 0)",
            "(atanh 0)",
            "(pow 5 2)",
        ];
        for case in cases {
            let parsed: Result<Simple, &str> = case.parse();
            assert!(parsed.is_ok(), "{case}");
        }
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
    fn log_aliases_parse_as_log10() {
        let empty = HashMap::new();
        for case in ["(log 100)", "(log10 100)"] {
            let xpr: Simple = case.parse().unwrap();
            let result = xpr.evaluate(&empty).unwrap();
            assert!(result.is_integer(), "{case}");
            assert_eq!(format!("{result}"), "2", "{case}");
        }
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

    #[test]
    fn tan() {
        let empty = HashMap::new();
        let xpr: Simple = "(/ (* (tan (* pi 3.8)) 7.9) (tan (/ pi 5)))"
            .parse()
            .unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        let m79: Real = "-7.9".parse().unwrap();
        assert_eq!(result, m79);
    }

    #[test]
    fn nested_exact_subexpressions() {
        let empty = HashMap::new();
        let xpr: Simple = "(/ (* (+ 1/2 1/3) (- 7/5 2/5)) (+ 1/7 2/7))"
            .parse()
            .unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        assert_eq!(result, Real::new(Rational::fraction(35, 18).unwrap()));
    }

    #[test]
    fn exact_symbol_subexpressions() {
        let mut names = HashMap::new();
        names.insert("x".to_string(), Real::new(Rational::fraction(3, 2).unwrap()));
        names.insert("y".to_string(), Real::new(Rational::fraction(5, 4).unwrap()));
        let xpr: Simple = "(* (+ x 1/2) (/ y 5/2))".parse().unwrap();
        let result = xpr.evaluate(&names).unwrap();
        assert_eq!(result, Real::new(Rational::one()));
    }
}
