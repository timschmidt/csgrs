//! Prepared low-degree polynomial residual blocks.
//!
//! SolveSpace keeps symbolic expressions and derivative structure available
//! before Newton iteration. This module adds the exact-stack equivalent for a
//! deliberately narrow family: active univariate quadratic residuals. The row
//! is still a nonlinear proposal row, but its bounded polynomial structure is
//! preserved for exact replay, future interval/Krawczyk certificates, and
//! solver dispatch. This follows the exact-geometric-computation model: retain object/expression structure
//! and choose an arithmetic package before scalar evaluation.

use std::collections::{BTreeMap, HashMap};

use hyperreal::{Real, RealExactSetFacts};

use crate::model::{Problem, Variable};
use crate::symbolic::{Expr, ExprEvalError, SymbolId, SymbolRef};

/// One exact linear term retained in a prepared multivariate quadratic row.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedLinearTerm {
    /// Solver symbol multiplied by this coefficient.
    pub symbol: SymbolId,
    /// Exact coefficient for `symbol`.
    pub coefficient: Real,
}

/// One exact quadratic term retained in a prepared multivariate quadratic row.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedQuadraticTerm {
    /// First solver symbol in canonical sorted order.
    pub first: SymbolId,
    /// Second solver symbol in canonical sorted order.
    pub second: SymbolId,
    /// Exact coefficient for `first * second`.
    pub coefficient: Real,
}

/// Prepared exact coefficients for one degree-at-most-two residual.
///
/// This is the multivariate sibling of [`PreparedUnivariateQuadraticResidual`].
/// It preserves the bounded polynomial package
/// `c + sum_i l_i*x_i + sum_ij q_ij*x_i*x_j` without expanding into a generic
/// CAS. SolveSpace-style nonlinear dispatch can inspect the row before Newton
/// iteration, while the exact-computation discipline is preserved because
/// the original expression remains authoritative and exact replay is still
/// required before accepting a candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedQuadraticResidual {
    constant: Real,
    linear_terms: Vec<PreparedLinearTerm>,
    quadratic_terms: Vec<PreparedQuadraticTerm>,
    coefficient_exact: RealExactSetFacts,
}

impl PreparedQuadraticResidual {
    /// Return the constant coefficient.
    pub fn constant(&self) -> &Real {
        &self.constant
    }

    /// Return exact retained linear terms.
    pub fn linear_terms(&self) -> &[PreparedLinearTerm] {
        &self.linear_terms
    }

    /// Return exact retained quadratic and cross terms.
    pub fn quadratic_terms(&self) -> &[PreparedQuadraticTerm] {
        &self.quadratic_terms
    }

    /// Return exact-set facts for all retained coefficients.
    pub fn coefficient_exact(&self) -> RealExactSetFacts {
        self.coefficient_exact
    }

    /// Evaluate the prepared quadratic row with bound variable values.
    pub fn eval_real(
        &self,
        variables: &[Variable],
        bindings: &HashMap<SymbolId, Real>,
    ) -> Result<Real, ExprEvalError> {
        for term in &self.linear_terms {
            ensure_problem_symbol(variables, term.symbol)?;
        }
        for term in &self.quadratic_terms {
            ensure_problem_symbol(variables, term.first)?;
            ensure_problem_symbol(variables, term.second)?;
        }

        let mut value = self.constant.clone();
        for term in &self.linear_terms {
            let variable = bindings
                .get(&term.symbol)
                .ok_or_else(|| ExprEvalError::UnboundSymbol(SymbolRef::new(term.symbol, None)))?;
            value += term.coefficient.clone() * variable.clone();
        }
        for term in &self.quadratic_terms {
            let first = bindings
                .get(&term.first)
                .ok_or_else(|| ExprEvalError::UnboundSymbol(SymbolRef::new(term.first, None)))?;
            let second = bindings
                .get(&term.second)
                .ok_or_else(|| ExprEvalError::UnboundSymbol(SymbolRef::new(term.second, None)))?;
            value += term.coefficient.clone() * first.clone() * second.clone();
        }
        Ok(value)
    }
}

/// Prepared exact coefficients for one univariate quadratic residual.
///
/// The row represents `quadratic * x^2 + linear * x + constant` for one solver
/// symbol. It is not a general CAS form; it is a bounded structural summary
/// meant to keep small polynomial rows inspectable before a lossy nonlinear
/// backend proposes a candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedUnivariateQuadraticResidual {
    symbol: SymbolId,
    constant: Real,
    linear: Real,
    quadratic: Real,
    coefficient_exact: RealExactSetFacts,
}

impl PreparedUnivariateQuadraticResidual {
    /// Return the solver symbol used by this polynomial row.
    pub const fn symbol(&self) -> SymbolId {
        self.symbol
    }

    /// Return the constant coefficient.
    pub fn constant(&self) -> &Real {
        &self.constant
    }

    /// Return the linear coefficient.
    pub fn linear(&self) -> &Real {
        &self.linear
    }

    /// Return the quadratic coefficient.
    pub fn quadratic(&self) -> &Real {
        &self.quadratic
    }

    /// Return exact-set facts for the three retained coefficients.
    pub fn coefficient_exact(&self) -> RealExactSetFacts {
        self.coefficient_exact
    }

    /// Evaluate the prepared quadratic row with bound variable values.
    pub fn eval_real(
        &self,
        variables: &[Variable],
        bindings: &HashMap<SymbolId, Real>,
    ) -> Result<Real, ExprEvalError> {
        if !variables
            .iter()
            .any(|variable| variable.symbol == self.symbol)
        {
            return Err(ExprEvalError::UnboundSymbol(SymbolRef::new(
                self.symbol,
                None,
            )));
        }
        let value = bindings
            .get(&self.symbol)
            .ok_or_else(|| ExprEvalError::UnboundSymbol(SymbolRef::new(self.symbol, None)))?;
        Ok(self.quadratic.clone() * value.clone() * value.clone()
            + self.linear.clone() * value.clone()
            + self.constant.clone())
    }
}

/// Try to prepare a univariate quadratic residual for the given problem.
pub fn prepare_univariate_quadratic_residual(
    expression: &Expr,
    problem: &Problem,
) -> Option<PreparedUnivariateQuadraticResidual> {
    let poly = collect_polynomial(expression)?;
    let symbol = poly.symbol?;
    if !problem
        .variables
        .iter()
        .any(|variable| variable.symbol == symbol)
    {
        return None;
    }
    if !poly.is_higher_zero() {
        return None;
    }
    let coefficient_exact = Real::exact_set_facts([&poly.constant, &poly.linear, &poly.quadratic]);
    Some(PreparedUnivariateQuadraticResidual {
        symbol,
        constant: poly.constant,
        linear: poly.linear,
        quadratic: poly.quadratic,
        coefficient_exact,
    })
}

/// Try to prepare any degree-at-most-two polynomial residual for the problem.
pub fn prepare_quadratic_residual(
    expression: &Expr,
    problem: &Problem,
) -> Option<PreparedQuadraticResidual> {
    let poly = collect_multivariate_quadratic(expression)?;
    if !poly.is_higher_zero() {
        return None;
    }
    for symbols in poly.terms.keys() {
        for symbol in symbols {
            if !problem
                .variables
                .iter()
                .any(|variable| variable.symbol == *symbol)
            {
                return None;
            }
        }
    }

    let mut constant = Real::zero();
    let mut linear_terms = Vec::new();
    let mut quadratic_terms = Vec::new();
    let mut coefficient_refs = Vec::new();

    for (symbols, coefficient) in poly.terms {
        if is_structural_zero(&coefficient) {
            continue;
        }
        match symbols.as_slice() {
            [] => constant = coefficient,
            [symbol] => linear_terms.push(PreparedLinearTerm {
                symbol: *symbol,
                coefficient,
            }),
            [first, second] => quadratic_terms.push(PreparedQuadraticTerm {
                first: *first,
                second: *second,
                coefficient,
            }),
            _ => return None,
        }
    }
    coefficient_refs.push(&constant);
    coefficient_refs.extend(linear_terms.iter().map(|term| &term.coefficient));
    coefficient_refs.extend(quadratic_terms.iter().map(|term| &term.coefficient));
    let coefficient_exact = Real::exact_set_facts(coefficient_refs);

    Some(PreparedQuadraticResidual {
        constant,
        linear_terms,
        quadratic_terms,
        coefficient_exact,
    })
}

#[derive(Clone, Debug)]
struct PolynomialAccumulator {
    symbol: Option<SymbolId>,
    constant: Real,
    linear: Real,
    quadratic: Real,
    higher: Real,
}

impl PolynomialAccumulator {
    fn constant(value: Real) -> Self {
        Self {
            symbol: None,
            constant: value,
            linear: Real::zero(),
            quadratic: Real::zero(),
            higher: Real::zero(),
        }
    }

    fn symbol(symbol: SymbolId) -> Self {
        Self {
            symbol: Some(symbol),
            constant: Real::zero(),
            linear: Real::one(),
            quadratic: Real::zero(),
            higher: Real::zero(),
        }
    }

    fn scale(mut self, scale: Real) -> Self {
        self.constant *= scale.clone();
        self.linear *= scale.clone();
        self.quadratic *= scale.clone();
        self.higher *= scale;
        self
    }

    fn add(self, other: Self) -> Option<Self> {
        let symbol = merge_symbol(self.symbol, other.symbol)?;
        Some(Self {
            symbol,
            constant: self.constant + other.constant,
            linear: self.linear + other.linear,
            quadratic: self.quadratic + other.quadratic,
            higher: self.higher + other.higher,
        })
    }

    fn multiply(self, other: Self) -> Option<Self> {
        let symbol = merge_symbol(self.symbol, other.symbol)?;
        let constant = self.constant.clone() * other.constant.clone();
        let linear = self.constant.clone() * other.linear.clone()
            + self.linear.clone() * other.constant.clone();
        let quadratic = self.constant.clone() * other.quadratic.clone()
            + self.linear.clone() * other.linear.clone()
            + self.quadratic.clone() * other.constant.clone();
        let higher = self.linear * other.quadratic.clone()
            + self.quadratic.clone() * other.linear
            + self.quadratic * other.quadratic
            + self.higher
            + other.higher;
        Some(Self {
            symbol,
            constant,
            linear,
            quadratic,
            higher,
        })
    }

    fn is_higher_zero(&self) -> bool {
        matches!(
            self.higher.structural_facts().zero,
            hyperreal::ZeroKnowledge::Zero
        )
    }
}

fn collect_polynomial(expression: &Expr) -> Option<PolynomialAccumulator> {
    match expression {
        Expr::Constant(value) => Some(PolynomialAccumulator::constant(value.clone())),
        Expr::Symbol(symbol) => Some(PolynomialAccumulator::symbol(symbol.id)),
        Expr::Add(left, right) => collect_polynomial(left)?.add(collect_polynomial(right)?),
        Expr::Sub(left, right) => {
            collect_polynomial(left)?.add(collect_polynomial(right)?.scale(-Real::one()))
        }
        Expr::Neg(value) => Some(collect_polynomial(value)?.scale(-Real::one())),
        Expr::Mul(left, right) => collect_polynomial(left)?.multiply(collect_polynomial(right)?),
        Expr::Div(left, right) => {
            let denominator = constant_value(right)?;
            let reciprocal = (Real::one() / denominator).ok()?;
            Some(collect_polynomial(left)?.scale(reciprocal))
        }
        Expr::PowI(value, 0) => {
            let _ = value;
            Some(PolynomialAccumulator::constant(Real::one()))
        }
        Expr::PowI(value, 1) => collect_polynomial(value),
        Expr::PowI(value, 2) => {
            let value = collect_polynomial(value)?;
            value.clone().multiply(value)
        }
        Expr::PowI(_, _)
        | Expr::Sqrt(_)
        | Expr::Sin(_)
        | Expr::Cos(_)
        | Expr::Ln(_)
        | Expr::Log10(_)
        | Expr::Asin(_)
        | Expr::Acos(_)
        | Expr::Acosh(_)
        | Expr::Atanh(_) => None,
    }
}

fn merge_symbol(left: Option<SymbolId>, right: Option<SymbolId>) -> Option<Option<SymbolId>> {
    match (left, right) {
        (None, None) => Some(None),
        (Some(symbol), None) | (None, Some(symbol)) => Some(Some(symbol)),
        (Some(left), Some(right)) if left == right => Some(Some(left)),
        (Some(_), Some(_)) => None,
    }
}

fn constant_value(expression: &Expr) -> Option<Real> {
    let facts = expression.structural_facts();
    if !facts.dependencies.is_empty() {
        return None;
    }
    expression.eval_real(&HashMap::new()).ok()
}

#[derive(Clone, Debug)]
struct MultivariateQuadraticAccumulator {
    terms: BTreeMap<Vec<SymbolId>, Real>,
    higher: Real,
}

impl MultivariateQuadraticAccumulator {
    fn constant(value: Real) -> Self {
        let mut terms = BTreeMap::new();
        terms.insert(Vec::new(), value);
        Self {
            terms,
            higher: Real::zero(),
        }
    }

    fn symbol(symbol: SymbolId) -> Self {
        let mut terms = BTreeMap::new();
        terms.insert(vec![symbol], Real::one());
        Self {
            terms,
            higher: Real::zero(),
        }
    }

    fn scale(mut self, scale: Real) -> Self {
        for coefficient in self.terms.values_mut() {
            *coefficient = coefficient.clone() * scale.clone();
        }
        self.higher *= scale;
        self
    }

    fn add(mut self, other: Self) -> Self {
        for (symbols, coefficient) in other.terms {
            let entry = self.terms.entry(symbols).or_insert_with(Real::zero);
            *entry = entry.clone() + coefficient;
        }
        self.higher += other.higher;
        self
    }

    fn multiply(self, other: Self) -> Self {
        let mut result = Self {
            terms: BTreeMap::new(),
            higher: self.higher + other.higher,
        };
        for (left_symbols, left_coefficient) in self.terms {
            for (right_symbols, right_coefficient) in &other.terms {
                let mut symbols = left_symbols.clone();
                symbols.extend(right_symbols.iter().copied());
                symbols.sort();
                let coefficient = left_coefficient.clone() * right_coefficient.clone();
                if symbols.len() <= 2 {
                    let entry = result.terms.entry(symbols).or_insert_with(Real::zero);
                    *entry = entry.clone() + coefficient;
                } else {
                    result.higher += coefficient;
                }
            }
        }
        result
    }

    fn is_higher_zero(&self) -> bool {
        is_structural_zero(&self.higher)
    }
}

fn collect_multivariate_quadratic(expression: &Expr) -> Option<MultivariateQuadraticAccumulator> {
    match expression {
        Expr::Constant(value) => Some(MultivariateQuadraticAccumulator::constant(value.clone())),
        Expr::Symbol(symbol) => Some(MultivariateQuadraticAccumulator::symbol(symbol.id)),
        Expr::Add(left, right) => {
            Some(collect_multivariate_quadratic(left)?.add(collect_multivariate_quadratic(right)?))
        }
        Expr::Sub(left, right) => Some(
            collect_multivariate_quadratic(left)?
                .add(collect_multivariate_quadratic(right)?.scale(-Real::one())),
        ),
        Expr::Neg(value) => Some(collect_multivariate_quadratic(value)?.scale(-Real::one())),
        Expr::Mul(left, right) => Some(
            collect_multivariate_quadratic(left)?.multiply(collect_multivariate_quadratic(right)?),
        ),
        Expr::Div(left, right) => {
            let denominator = constant_value(right)?;
            let reciprocal = (Real::one() / denominator).ok()?;
            Some(collect_multivariate_quadratic(left)?.scale(reciprocal))
        }
        Expr::PowI(value, 0) => {
            let _ = value;
            Some(MultivariateQuadraticAccumulator::constant(Real::one()))
        }
        Expr::PowI(value, 1) => collect_multivariate_quadratic(value),
        Expr::PowI(value, 2) => {
            let value = collect_multivariate_quadratic(value)?;
            Some(value.clone().multiply(value))
        }
        Expr::PowI(_, _)
        | Expr::Sqrt(_)
        | Expr::Sin(_)
        | Expr::Cos(_)
        | Expr::Ln(_)
        | Expr::Log10(_)
        | Expr::Asin(_)
        | Expr::Acos(_)
        | Expr::Acosh(_)
        | Expr::Atanh(_) => None,
    }
}

fn ensure_problem_symbol(variables: &[Variable], symbol: SymbolId) -> Result<(), ExprEvalError> {
    if variables.iter().any(|variable| variable.symbol == symbol) {
        Ok(())
    } else {
        Err(ExprEvalError::UnboundSymbol(SymbolRef::new(symbol, None)))
    }
}

fn is_structural_zero(value: &Real) -> bool {
    matches!(
        value.structural_facts().zero,
        hyperreal::ZeroKnowledge::Zero
    )
}
