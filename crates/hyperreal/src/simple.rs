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
use num::ToPrimitive;
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
    Log2,
    Ln,
    Ln1p,
    Expm1,
    Softplus,
    Logit,
    Sigmoid,
    Cos,
    Sin,
    Tan,
    Erf,
    Erfc,
    Erfcx,
    Dnorm,
    Pnorm,
    NormalSf,
    PnormUpper,
    NormalInterval,
    PnormDiff,
    LogPnorm,
    LogNormalSf,
    LogDnorm,
    Erfinv,
    Erfcinv,
    Qnorm,
    QnormUpper,
    NormalPdf,
    NormalCdf,
    NormalSurvival,
    NormalQuantile,
    NormalHazard,
    NormalLogHazard,
    NormalMills,
    NormalInverseMills,
    HermiteProbabilists,
    DnormDerivative,
    GaussianDerivative,
    StandardNormalMoment,
    NormalIntervalMoment,
    TruncatedNormalMean,
    TruncatedNormalVariance,
    RegularizedGammaP,
    RegularizedGammaQ,
    ChiSquareCdf,
    ChiSquareSf,
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
            "tau" => Ok(Real::tau()),
            "e" => Ok(Real::e()),
            _ => names.get(name).cloned().ok_or(Problem::NotFound),
        }
    }

    fn lookup_exact(name: &str, names: &Symbols) -> Option<Rational> {
        match name {
            "pi" | "tau" | "e" => None,
            _ => names.get(name).and_then(Real::exact_rational),
        }
    }

    fn exact_usize_operand(&self, index: usize, names: &Symbols) -> Result<usize, Problem> {
        let Some(exact) = self.operands.get(index).unwrap().exact_value(names)? else {
            return Err(Problem::NotANumber);
        };
        let Some(integer) = exact.to_big_integer() else {
            return Err(Problem::NotANumber);
        };
        integer.to_usize().ok_or(Problem::NotANumber)
    }

    fn exact_u64_operand(&self, index: usize, names: &Symbols) -> Result<u64, Problem> {
        let Some(exact) = self.operands.get(index).unwrap().exact_value(names)? else {
            return Err(Problem::NotANumber);
        };
        let Some(integer) = exact.to_big_integer() else {
            return Err(Problem::NotANumber);
        };
        integer.to_u64().ok_or(Problem::NotANumber)
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
                    value *= exact;
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

    /// Evaluate this parsed expression using the supplied symbol table.
    pub fn evaluate(&self, names: &Symbols) -> Result<Real, Problem> {
        use Operator::*;
        match self.op {
            Plus => {
                if self.could_evaluate_exact()
                    && let Some(value) = self.evaluate_exact(names)?
                {
                    // Fold fully exact parser subtrees immediately.  This avoids building
                    // symbolic/computable expression graphs for literal arithmetic that the
                    // caller expects to stay cheap and exact.
                    return Ok(Real::new(value));
                }
                if let Some(first) = self.operands.first().and_then(Operand::literal) {
                    let mut value = first.clone();
                    let literals = self.operands.iter().skip(1);
                    if literals.clone().all(|operand| operand.literal().is_some()) {
                        // The all-literal path keeps simple parsed sums in the rational
                        // representation instead of allocating a chain of Real additions.
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
                    value += operand.value(names)?;
                }
                Ok(value)
            }
            Minus => match self.operands.len() {
                0 => Err(Problem::InsufficientParameters),
                1 => {
                    if self.could_evaluate_exact()
                        && let Some(value) = self.evaluate_exact(names)?
                    {
                        // Unary exact negation is kept rational so sign/MSD queries can be
                        // answered structurally without touching computable approximations.
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
                        // Multi-operand exact subtraction is another parser-level fold; it
                        // prevents cheap constants such as "pi - 3" from being polluted by
                        // unrelated literal arithmetic around them.
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
                        value -= operand.value(names)?;
                    }
                    Ok(value)
                }
            },
            Star => {
                if self.could_evaluate_exact()
                    && let Some(value) = self.evaluate_exact(names)?
                {
                    // Preserve exact products as rationals whenever every operand can be
                    // evaluated exactly; generic multiplication is much more expensive for
                    // values that do not need symbolic structure.
                    return Ok(Real::new(value));
                }
                if let Some(first) = self.operands.first().and_then(Operand::literal) {
                    let mut value = first.clone();
                    let literals = self.operands.iter().skip(1);
                    if literals.clone().all(|operand| operand.literal().is_some()) {
                        for operand in literals {
                            value *= operand.literal().unwrap();
                        }
                        return Ok(Real::new(value));
                    }
                }
                let mut operands = self.operands.iter();
                let Some(first) = operands.next() else {
                    return Ok(Real::one());
                };
                let mut value = first.value(names)?;
                for operand in operands {
                    value *= operand.value(names)?;
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
            Log2 => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let operand = self.operands.first().unwrap();
                let value = operand.value(names)?.log2()?;
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
            Ln1p => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.ln_1p()
            }
            Expm1 => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                Ok(self.operands.first().unwrap().value(names)?.expm1())
            }
            Softplus => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.softplus()
            }
            Logit => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.logit()
            }
            Sigmoid => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.sigmoid()
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
            Erf => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                Ok(self.operands.first().unwrap().value(names)?.erf())
            }
            Erfc => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                Ok(self.operands.first().unwrap().value(names)?.erfc())
            }
            Erfcx => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.erfcx()
            }
            Dnorm => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.dnorm()
            }
            Pnorm => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.pnorm()
            }
            NormalSf => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.normal_sf()
            }
            PnormUpper => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.pnorm_upper()
            }
            NormalInterval => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let lo = self.operands.first().unwrap().value(names)?;
                let hi = self.operands.get(1).unwrap().value(names)?;
                Real::normal_interval(&lo, &hi)
            }
            PnormDiff => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let lo = self.operands.first().unwrap().value(names)?;
                let hi = self.operands.get(1).unwrap().value(names)?;
                Real::pnorm_diff(&lo, &hi)
            }
            LogPnorm => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.log_pnorm()
            }
            LogNormalSf => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.log_normal_sf()
            }
            LogDnorm => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.log_dnorm()
            }
            Erfinv => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.erfinv()
            }
            Erfcinv => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.erfcinv()
            }
            Qnorm => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.qnorm()
            }
            QnormUpper => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.qnorm_upper()
            }
            NormalPdf => {
                if self.operands.len() != 3 {
                    return Err(Problem::ParseError);
                }
                let x = self.operands.first().unwrap().value(names)?;
                let mean = self.operands.get(1).unwrap().value(names)?;
                let sigma = self.operands.get(2).unwrap().value(names)?;
                x.normal_pdf(&mean, &sigma)
            }
            NormalCdf => {
                if self.operands.len() != 3 {
                    return Err(Problem::ParseError);
                }
                let x = self.operands.first().unwrap().value(names)?;
                let mean = self.operands.get(1).unwrap().value(names)?;
                let sigma = self.operands.get(2).unwrap().value(names)?;
                x.normal_cdf(&mean, &sigma)
            }
            NormalSurvival => {
                if self.operands.len() != 3 {
                    return Err(Problem::ParseError);
                }
                let x = self.operands.first().unwrap().value(names)?;
                let mean = self.operands.get(1).unwrap().value(names)?;
                let sigma = self.operands.get(2).unwrap().value(names)?;
                x.normal_survival(&mean, &sigma)
            }
            NormalQuantile => {
                if self.operands.len() != 3 {
                    return Err(Problem::ParseError);
                }
                let p = self.operands.first().unwrap().value(names)?;
                let mean = self.operands.get(1).unwrap().value(names)?;
                let sigma = self.operands.get(2).unwrap().value(names)?;
                p.normal_quantile(&mean, &sigma)
            }
            NormalHazard => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.normal_hazard()
            }
            NormalLogHazard => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands
                    .first()
                    .unwrap()
                    .value(names)?
                    .normal_log_hazard()
            }
            NormalMills => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands.first().unwrap().value(names)?.normal_mills()
            }
            NormalInverseMills => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                self.operands
                    .first()
                    .unwrap()
                    .value(names)?
                    .normal_inverse_mills()
            }
            HermiteProbabilists => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let n = self.exact_usize_operand(0, names)?;
                let x = self.operands.get(1).unwrap().value(names)?;
                Ok(Real::hermite_probabilists(n, &x))
            }
            DnormDerivative => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let n = self.exact_usize_operand(0, names)?;
                self.operands
                    .get(1)
                    .unwrap()
                    .value(names)?
                    .dnorm_derivative(n)
            }
            GaussianDerivative => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let n = self.exact_usize_operand(0, names)?;
                self.operands
                    .get(1)
                    .unwrap()
                    .value(names)?
                    .gaussian_derivative(n)
            }
            StandardNormalMoment => {
                if self.operands.len() != 1 {
                    return Err(Problem::ParseError);
                }
                let n = self.exact_usize_operand(0, names)?;
                Ok(Real::standard_normal_moment(n))
            }
            NormalIntervalMoment => {
                if self.operands.len() != 3 {
                    return Err(Problem::ParseError);
                }
                let lo = self.operands.first().unwrap().value(names)?;
                let hi = self.operands.get(1).unwrap().value(names)?;
                let n = self.exact_usize_operand(2, names)?;
                Real::normal_interval_moment(&lo, &hi, n)
            }
            TruncatedNormalMean => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let lo = self.operands.first().unwrap().value(names)?;
                let hi = self.operands.get(1).unwrap().value(names)?;
                Real::truncated_normal_mean(&lo, &hi)
            }
            TruncatedNormalVariance => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let lo = self.operands.first().unwrap().value(names)?;
                let hi = self.operands.get(1).unwrap().value(names)?;
                Real::truncated_normal_variance(&lo, &hi)
            }
            RegularizedGammaP => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let a = self.operands.first().unwrap().value(names)?;
                let x = self.operands.get(1).unwrap().value(names)?;
                Real::regularized_gamma_p(&a, &x)
            }
            RegularizedGammaQ => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let a = self.operands.first().unwrap().value(names)?;
                let x = self.operands.get(1).unwrap().value(names)?;
                Real::regularized_gamma_q(&a, &x)
            }
            ChiSquareCdf => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let x = self.operands.first().unwrap().value(names)?;
                let k = self.exact_u64_operand(1, names)?;
                Real::chi_square_cdf(&x, k)
            }
            ChiSquareSf => {
                if self.operands.len() != 2 {
                    return Err(Problem::ParseError);
                }
                let x = self.operands.first().unwrap().value(names)?;
                let k = self.exact_u64_operand(1, names)?;
                Real::chi_square_sf(&x, k)
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
                'A'..='Z' | 'a'..='z' | '0'..='9' | '_' => token.push(*c),
                _ => break,
            }
            chars.next();
        }
        token
    }

    fn operator(chars: &mut Peekable<Chars>) -> Result<Operator, &'static str> {
        use Operator::*;
        match Self::consume_operator_token(chars).as_str() {
            "log10" => Ok(Log10),
            "log2" => Ok(Log2),
            "ln" => Ok(Ln),
            "ln_1p" | "log1p" => Ok(Ln1p),
            "exp" => Ok(Exp),
            "expm1" => Ok(Expm1),
            "softplus" => Ok(Softplus),
            "logit" => Ok(Logit),
            "sigmoid" => Ok(Sigmoid),
            "sqrt" => Ok(Sqrt),
            "cos" => Ok(Cos),
            "sin" => Ok(Sin),
            "pow" => Ok(Pow),
            "tan" => Ok(Tan),
            "erf" => Ok(Erf),
            "erfc" => Ok(Erfc),
            "erfcx" => Ok(Erfcx),
            "dnorm" => Ok(Dnorm),
            "pnorm" => Ok(Pnorm),
            "normal_sf" => Ok(NormalSf),
            "pnorm_upper" => Ok(PnormUpper),
            "normal_interval" => Ok(NormalInterval),
            "pnorm_diff" => Ok(PnormDiff),
            "log_pnorm" => Ok(LogPnorm),
            "log_normal_sf" => Ok(LogNormalSf),
            "log_dnorm" => Ok(LogDnorm),
            "erfinv" => Ok(Erfinv),
            "erfcinv" => Ok(Erfcinv),
            "qnorm" => Ok(Qnorm),
            "qnorm_upper" => Ok(QnormUpper),
            "normal_pdf" => Ok(NormalPdf),
            "normal_cdf" => Ok(NormalCdf),
            "normal_survival" => Ok(NormalSurvival),
            "normal_quantile" => Ok(NormalQuantile),
            "normal_hazard" => Ok(NormalHazard),
            "normal_log_hazard" => Ok(NormalLogHazard),
            "normal_mills" => Ok(NormalMills),
            "normal_inverse_mills" => Ok(NormalInverseMills),
            "hermite_probabilists" => Ok(HermiteProbabilists),
            "dnorm_derivative" => Ok(DnormDerivative),
            "gaussian_derivative" => Ok(GaussianDerivative),
            "standard_normal_moment" => Ok(StandardNormalMoment),
            "normal_interval_moment" => Ok(NormalIntervalMoment),
            "truncated_normal_mean" => Ok(TruncatedNormalMean),
            "truncated_normal_variance" => Ok(TruncatedNormalVariance),
            "regularized_gamma_p" => Ok(RegularizedGammaP),
            "regularized_gamma_q" => Ok(RegularizedGammaQ),
            "chi_square_cdf" => Ok(ChiSquareCdf),
            "chi_square_sf" => Ok(ChiSquareSf),
            "acos" => Ok(Acos),
            "asin" => Ok(Asin),
            "atan" => Ok(Atan),
            "acosh" => Ok(Acosh),
            "asinh" => Ok(Asinh),
            "atanh" => Ok(Atanh),
            _ => Err("No such operator"),
        }
    }

    /// Parse one parenthesized prefix expression from a character stream.
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
        let parsed = Simple::parse(&mut chars)?;
        while let Some(c) = chars.peek() {
            match c {
                ' ' | '\t' => {
                    chars.next();
                }
                _ => return Err("Trailing input after expression"),
            }
        }
        Ok(parsed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    #[test]
    fn missing_close() {
        let xpr: Result<Simple, &str> = "(+ (* (exp 4) (exp 6))".parse();
        assert_eq!(xpr, Err("Incomplete expression"))
    }

    #[test]
    fn parse_rejects_trailing_input() {
        let xpr: Result<Simple, &str> = "(+ 1 2) junk".parse();
        assert_eq!(xpr, Err("Trailing input after expression"));

        let xpr: Result<Simple, &str> = "(+ 1 2) \t".parse();
        assert!(xpr.is_ok());
    }

    #[test]
    fn parse_named_operators() {
        let cases = [
            "(ln 5)",
            "(ln_1p 5)",
            "(log1p 5)",
            "(log10 5)",
            "(log2 5)",
            "(exp 5)",
            "(expm1 5)",
            "(softplus 5)",
            "(logit 1/2)",
            "(sigmoid 5)",
            "(sqrt 5)",
            "(cos 5)",
            "(sin 5)",
            "(tan 5)",
            "(erf 1)",
            "(erfc 1)",
            "(erfcx 1)",
            "(dnorm 0)",
            "(pnorm 1)",
            "(normal_sf 1)",
            "(pnorm_upper 1)",
            "(normal_interval 0 1)",
            "(pnorm_diff 0 1)",
            "(log_pnorm 0)",
            "(log_normal_sf 0)",
            "(log_dnorm 0)",
            "(erfinv 0)",
            "(erfcinv 1)",
            "(qnorm 1/2)",
            "(qnorm_upper 1/2)",
            "(normal_pdf 5 2 3)",
            "(normal_cdf 5 2 3)",
            "(normal_survival 5 2 3)",
            "(normal_quantile 975/1000 2 3)",
            "(normal_hazard 1)",
            "(normal_log_hazard 1)",
            "(normal_mills 1)",
            "(normal_inverse_mills 1)",
            "(hermite_probabilists 3 2)",
            "(dnorm_derivative 1 1)",
            "(gaussian_derivative 3 1)",
            "(standard_normal_moment 6)",
            "(normal_interval_moment 0 1 2)",
            "(truncated_normal_mean 0 1)",
            "(truncated_normal_variance 0 1)",
            "(regularized_gamma_p 3/2 1)",
            "(regularized_gamma_q 3/2 1)",
            "(chi_square_cdf 2 2)",
            "(chi_square_sf 1 1)",
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
    fn parser_rejects_removed_short_operator_names() {
        for case in ["(l 5)", "(log 5)", "(lg 5)", "(e 5)", "(s 5)"] {
            let parsed: Result<Simple, &str> = case.parse();
            assert_eq!(parsed, Err("No such operator"), "{case}");
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
        let xpr: Simple = "(ln (* (exp 4) (exp 6)))".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        assert!(result.is_integer());
        let ans = format!("{result}");
        assert_eq!(ans, "10");
    }

    #[test]
    fn log10_parses_as_base10() {
        let empty = HashMap::new();
        let xpr: Simple = "(log10 100)".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        assert!(result.is_integer());
        assert_eq!(format!("{result}"), "2");
    }

    #[test]
    fn log2_parses_as_base2() {
        let empty = HashMap::new();
        let xpr: Simple = "(log2 1024)".parse().unwrap();
        let result = xpr.evaluate(&empty).unwrap();
        assert!(result.is_integer());
        assert_eq!(format!("{result}"), "10");
    }

    #[test]
    fn log2_parser_rejects_bad_domain_and_arity() {
        let empty = HashMap::new();

        let negative: Simple = "(log2 -1)".parse().unwrap();
        assert_eq!(negative.evaluate(&empty), Err(Problem::NotANumber));

        let wrong_arity: Simple = "(log2 2 4)".parse().unwrap();
        assert_eq!(wrong_arity.evaluate(&empty), Err(Problem::ParseError));
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
    fn normal_functions_evaluate() {
        let empty = HashMap::new();
        for (case, expected) in [
            ("(erf 1)", "8.42700792949714869341220635082609e-1"),
            ("(erfc 1)", "1.57299207050285130658779364917391e-1"),
            ("(erfcx 1)", "4.27583576155807004410750344490515e-1"),
            ("(dnorm 0)", "3.98942280401432677939946059934382e-1"),
            ("(pnorm 1)", "8.41344746068542948585232545632038e-1"),
            ("(normal_sf 1)", "1.58655253931457051414767454367962e-1"),
            ("(pnorm_upper 1)", "1.58655253931457051414767454367962e-1"),
            (
                "(normal_interval 0 1)",
                "3.41344746068542948585232545632038e-1",
            ),
            ("(pnorm_diff 0 1)", "3.41344746068542948585232545632038e-1"),
            ("(log_pnorm 0)", "-6.93147180559945309417232121458177e-1"),
            (
                "(log_normal_sf 0)",
                "-6.93147180559945309417232121458177e-1",
            ),
            ("(log_dnorm 0)", "-9.18938533204672741780329736405618e-1"),
            ("(qnorm 975/1000)", "1.95996398454005423552459443052055e0"),
            (
                "(qnorm_upper 25/1000)",
                "1.95996398454005423552459443052055e0",
            ),
        ] {
            let xpr: Simple = case.parse().unwrap();
            let result = xpr.evaluate(&empty).unwrap();
            assert_eq!(format!("{result:.32e}"), expected, "{case}");
        }

        for (case, expected, tolerance) in [
            ("(normal_pdf 5 2 3)", 0.08065690817304778, 1e-15),
            ("(normal_cdf 5 2 3)", 0.8413447460685429, 1e-15),
            ("(normal_survival 5 2 3)", 0.15865525393145707, 1e-15),
            ("(normal_quantile 975/1000 2 3)", 7.879891953620163, 1e-14),
            ("(normal_mills 1)", 0.6556795424187986, 1e-15),
            ("(normal_hazard 1)", 1.525135276160981, 1e-15),
            ("(normal_log_hazard 1)", 0.4220831118045907, 1e-15),
            ("(normal_inverse_mills 1)", 0.2875999709391784, 1e-15),
            ("(dnorm_derivative 1 1)", -0.24197072451914337, 1e-15),
            ("(gaussian_derivative 3 1)", 0.48394144903828673, 1e-15),
            ("(normal_interval_moment 0 1 1)", 0.15697155588228934, 1e-15),
            ("(normal_interval_moment 0 1 2)", 0.09937402154939956, 1e-15),
            ("(regularized_gamma_p 3/2 1)", 0.4275932955291202, 1e-15),
            ("(regularized_gamma_q 3/2 1)", 0.5724067044708798, 1e-15),
            ("(chi_square_cdf 2 2)", 0.6321205588285577, 1e-15),
            ("(chi_square_sf 1 1)", 0.31731050786291404, 1e-15),
        ] as [(&str, f64, f64); 16]
        {
            let xpr: Simple = case.parse().unwrap();
            let actual: f64 = xpr.evaluate(&empty).unwrap().into();
            let scale = expected.abs().max(1.0);
            assert!(
                (actual - expected).abs() <= tolerance * scale,
                "actual {actual}, expected {expected}, tolerance {tolerance}, case {case}"
            );
        }

        for (case, expected) in [
            (
                "(truncated_normal_mean 0 1)",
                "0.45986222928642650033302670255646",
            ),
            (
                "(truncated_normal_variance 0 1)",
                "0.07965182484851131233334055314679",
            ),
        ] {
            let xpr: Simple = case.parse().unwrap();
            assert_eq!(
                format!("{:#}", xpr.evaluate(&empty).unwrap()),
                expected,
                "{case}"
            );
        }
    }

    #[test]
    fn normal_functions_exact_cases_and_domains() {
        let empty = HashMap::new();

        let erf_zero: Simple = "(erf 0)".parse().unwrap();
        assert!(erf_zero.evaluate(&empty).unwrap().definitely_zero());

        let erfc_zero: Simple = "(erfc 0)".parse().unwrap();
        assert_eq!(erfc_zero.evaluate(&empty).unwrap(), Real::one());

        let erfcx_zero: Simple = "(erfcx 0)".parse().unwrap();
        assert_eq!(erfcx_zero.evaluate(&empty).unwrap(), Real::one());

        let pnorm_zero: Simple = "(pnorm 0)".parse().unwrap();
        assert_eq!(
            pnorm_zero.evaluate(&empty).unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );

        let normal_sf_zero: Simple = "(normal_sf 0)".parse().unwrap();
        assert_eq!(
            normal_sf_zero.evaluate(&empty).unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );

        let normal_interval_zero: Simple = "(normal_interval 1 1)".parse().unwrap();
        assert!(
            normal_interval_zero
                .evaluate(&empty)
                .unwrap()
                .definitely_zero()
        );

        let qnorm_half: Simple = "(qnorm 1/2)".parse().unwrap();
        assert!(qnorm_half.evaluate(&empty).unwrap().definitely_zero());

        let erfinv_zero: Simple = "(erfinv 0)".parse().unwrap();
        assert!(erfinv_zero.evaluate(&empty).unwrap().definitely_zero());

        let erfcinv_one: Simple = "(erfcinv 1)".parse().unwrap();
        assert!(erfcinv_one.evaluate(&empty).unwrap().definitely_zero());

        let qnorm_upper_half: Simple = "(qnorm_upper 1/2)".parse().unwrap();
        assert!(qnorm_upper_half.evaluate(&empty).unwrap().definitely_zero());

        let normal_cdf_mean: Simple = "(normal_cdf 2 2 3)".parse().unwrap();
        assert_eq!(
            normal_cdf_mean.evaluate(&empty).unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );

        let normal_survival_mean: Simple = "(normal_survival 2 2 3)".parse().unwrap();
        assert_eq!(
            normal_survival_mean.evaluate(&empty).unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );

        let normal_quantile_half: Simple = "(normal_quantile 1/2 2 3)".parse().unwrap();
        assert_eq!(
            normal_quantile_half.evaluate(&empty).unwrap(),
            Real::from(2_i32)
        );

        let hermite: Simple = "(hermite_probabilists 3 2)".parse().unwrap();
        assert_eq!(hermite.evaluate(&empty).unwrap(), Real::from(2_i32));

        let standard_moment: Simple = "(standard_normal_moment 6)".parse().unwrap();
        assert_eq!(
            standard_moment.evaluate(&empty).unwrap(),
            Real::from(15_i32)
        );

        let ln_1p_zero: Simple = "(ln_1p 0)".parse().unwrap();
        assert!(ln_1p_zero.evaluate(&empty).unwrap().definitely_zero());

        let log1p_zero: Simple = "(log1p 0)".parse().unwrap();
        assert!(log1p_zero.evaluate(&empty).unwrap().definitely_zero());

        let expm1_zero: Simple = "(expm1 0)".parse().unwrap();
        assert!(expm1_zero.evaluate(&empty).unwrap().definitely_zero());

        let logit_half: Simple = "(logit 1/2)".parse().unwrap();
        assert!(logit_half.evaluate(&empty).unwrap().definitely_zero());

        let sigmoid_zero: Simple = "(sigmoid 0)".parse().unwrap();
        assert_eq!(
            sigmoid_zero.evaluate(&empty).unwrap(),
            Real::new(Rational::fraction(1, 2).unwrap())
        );

        let gamma_zero: Simple = "(regularized_gamma_p 1 0)".parse().unwrap();
        assert!(gamma_zero.evaluate(&empty).unwrap().definitely_zero());

        let gamma_q_zero: Simple = "(regularized_gamma_q 1 0)".parse().unwrap();
        assert_eq!(gamma_q_zero.evaluate(&empty).unwrap(), Real::one());

        for case in [
            "(qnorm 0)",
            "(qnorm 1)",
            "(qnorm -1)",
            "(qnorm 2)",
            "(erfinv 1)",
            "(erfinv -1)",
            "(erfcinv 0)",
            "(erfcinv 2)",
            "(qnorm_upper 0)",
            "(qnorm_upper 1)",
            "(normal_pdf 5 0 0)",
            "(normal_cdf 5 0 -1)",
            "(normal_survival 5 0 -1)",
            "(normal_quantile 1/2 0 0)",
            "(hermite_probabilists -1 2)",
            "(hermite_probabilists 3/2 2)",
            "(dnorm_derivative -1 1)",
            "(gaussian_derivative 3/2 1)",
            "(standard_normal_moment -1)",
            "(standard_normal_moment 3/2)",
            "(normal_interval_moment 0 1 -1)",
            "(normal_interval_moment 0 1 3/2)",
            "(normal_interval_moment 2 1 1)",
            "(truncated_normal_mean 1 1)",
            "(truncated_normal_variance 2 1)",
            "(ln_1p -1)",
            "(log1p -2)",
            "(logit 0)",
            "(logit 1)",
            "(regularized_gamma_p 0 1)",
            "(regularized_gamma_q 1/3 1)",
            "(regularized_gamma_p 1 -1)",
            "(chi_square_cdf 1 0)",
            "(chi_square_sf -1 1)",
        ] {
            let xpr: Simple = case.parse().unwrap();
            assert_eq!(xpr.evaluate(&empty), Err(Problem::NotANumber), "{case}");
        }

        for case in [
            "(pnorm 11)",
            "(normal_sf 11)",
            "(pnorm_upper 11)",
            "(normal_interval -11 0)",
            "(log_pnorm 11)",
            "(log_normal_sf 11)",
            "(normal_log_hazard 11)",
            "(normal_inverse_mills 11)",
            "(dnorm_derivative 1 11)",
            "(gaussian_derivative 1 11)",
            "(normal_interval_moment -11 0 1)",
            "(dnorm -600)",
        ] {
            let xpr: Simple = case.parse().unwrap();
            assert_eq!(xpr.evaluate(&empty), Err(Problem::Exhausted), "{case}");
        }

        let reversed: Simple = "(normal_interval 2 1)".parse().unwrap();
        assert_eq!(reversed.evaluate(&empty), Err(Problem::NotANumber));
    }

    #[test]
    fn normal_function_wrong_arity_errors() {
        let empty = HashMap::new();
        for case in [
            "(erf )",
            "(erfc 1 2)",
            "(erfcx )",
            "(dnorm 0 1)",
            "(pnorm )",
            "(normal_sf )",
            "(pnorm_upper 1 2)",
            "(normal_interval 0)",
            "(pnorm_diff 0 1 2)",
            "(log_pnorm )",
            "(log_normal_sf 0 1)",
            "(log_dnorm )",
            "(erfinv )",
            "(erfcinv 1 2)",
            "(qnorm 1/2 3/4)",
            "(qnorm_upper )",
            "(normal_pdf 5 2)",
            "(normal_cdf 5 2 3 4)",
            "(normal_survival 5)",
            "(normal_quantile 1/2 2)",
            "(normal_hazard )",
            "(normal_log_hazard 1 2)",
            "(normal_mills )",
            "(normal_inverse_mills 1 2)",
            "(hermite_probabilists 3)",
            "(dnorm_derivative 1)",
            "(gaussian_derivative 1 1 1)",
            "(standard_normal_moment )",
            "(normal_interval_moment 0 1)",
            "(truncated_normal_mean 0)",
            "(truncated_normal_variance 0 1 2)",
            "(regularized_gamma_p 1)",
            "(regularized_gamma_q 1 2 3)",
            "(chi_square_cdf 1)",
            "(chi_square_sf 1 2 3)",
        ] {
            let xpr: Simple = case.parse().unwrap();
            assert_eq!(xpr.evaluate(&empty), Err(Problem::ParseError), "{case}");
        }
    }

    #[test]
    fn inverse_function_domain_errors_propagate() {
        let empty = HashMap::new();
        for case in [
            "(asin 11/10)",
            "(acos -11/10)",
            "(asin (sqrt 2))",
            "(acos (sqrt 2))",
            "(acosh 0)",
            "(acosh -2)",
            "(atanh (sqrt 2))",
        ] {
            let xpr: Simple = case.parse().unwrap();
            assert_eq!(xpr.evaluate(&empty), Err(Problem::NotANumber), "{case}");
        }

        for case in ["(atanh 1)", "(atanh -1)"] {
            let xpr: Simple = case.parse().unwrap();
            assert_eq!(xpr.evaluate(&empty), Err(Problem::Infinity), "{case}");
        }
    }

    #[test]
    fn inverse_function_nested_valid_values_evaluate() {
        let empty = HashMap::new();
        for (case, expected) in [
            ("(asinh (sqrt 2))", "1.14621583478058884390039365567401e0"),
            ("(acosh (sqrt 2))", "8.81373587019543025232609324979792e-1"),
            ("(atanh -1/2)", "-5.49306144334054845697622618461263e-1"),
        ] {
            let xpr: Simple = case.parse().unwrap();
            let result = xpr.evaluate(&empty).unwrap();
            assert_eq!(format!("{result:.32e}"), expected, "{case}");
        }
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
        names.insert(
            "x".to_string(),
            Real::new(Rational::fraction(3, 2).unwrap()),
        );
        names.insert(
            "y".to_string(),
            Real::new(Rational::fraction(5, 4).unwrap()),
        );
        let xpr: Simple = "(* (+ x 1/2) (/ y 5/2))".parse().unwrap();
        let result = xpr.evaluate(&names).unwrap();
        assert_eq!(result, Real::new(Rational::one()));
    }
}
