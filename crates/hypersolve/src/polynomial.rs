//! Low-degree polynomial residual forms.
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

// These extractors are bounded quadratic recognizers, not unrestricted CAS
// expansion. Retain enough of a higher-degree tail to prove ordinary
// cancellations, but never turn resource exhaustion into a zero coefficient.
const MAX_CANCELLATION_DEGREE: usize = 64;
const MAX_CANCELLATION_TERMS: usize = 256;

/// One exact linear term retained in a multivariate quadratic row.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticLinearTerm {
    /// Solver symbol multiplied by this coefficient.
    pub symbol: SymbolId,
    /// Exact coefficient for `symbol`.
    pub coefficient: Real,
}

/// One exact quadratic term retained in a multivariate quadratic row.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticTerm {
    /// First solver symbol in canonical sorted order.
    pub first: SymbolId,
    /// Second solver symbol in canonical sorted order.
    pub second: SymbolId,
    /// Exact coefficient for `first * second`.
    pub coefficient: Real,
}

/// Exact coefficients for one degree-at-most-two residual.
///
/// This is the multivariate sibling of [`UnivariateQuadraticResidual`].
/// It preserves the bounded polynomial package
/// `c + sum_i l_i*x_i + sum_ij q_ij*x_i*x_j` without expanding into a generic
/// CAS. SolveSpace-style nonlinear dispatch can inspect the row before Newton
/// iteration, while the exact-computation discipline is preserved because
/// the original expression remains authoritative and exact replay is still
/// required before accepting a candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct QuadraticResidual {
    constant: Real,
    linear_terms: Vec<QuadraticLinearTerm>,
    quadratic_terms: Vec<QuadraticTerm>,
    coefficient_exact: RealExactSetFacts,
}

impl QuadraticResidual {
    /// Extract a degree-at-most-two residual form from an expression.
    ///
    /// Returns `None` when the expression is not quadratic in the problem's
    /// variables or refers to a symbol outside the problem.
    /// Intermediate higher-degree cancellations are retained through degree
    /// 64 and at most 256 nonzero higher-degree terms. Exceeding either limit
    /// returns `None`, even if a larger expansion would eventually cancel.
    pub fn from_expr(expression: &Expr, problem: &Problem) -> Option<Self> {
        let poly = collect_multivariate_quadratic(expression, problem)?;
        if !poly.is_higher_zero() {
            return None;
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
                [symbol] => linear_terms.push(QuadraticLinearTerm {
                    symbol: *symbol,
                    coefficient,
                }),
                [first, second] => quadratic_terms.push(QuadraticTerm {
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

        Some(Self {
            constant,
            linear_terms,
            quadratic_terms,
            coefficient_exact,
        })
    }

    /// Return the constant coefficient.
    pub fn constant(&self) -> &Real {
        &self.constant
    }

    /// Return exact retained linear terms.
    pub fn linear_terms(&self) -> &[QuadraticLinearTerm] {
        &self.linear_terms
    }

    /// Return exact retained quadratic and cross terms.
    pub fn quadratic_terms(&self) -> &[QuadraticTerm] {
        &self.quadratic_terms
    }

    /// Return exact-set facts for all retained coefficients.
    pub fn coefficient_exact(&self) -> RealExactSetFacts {
        self.coefficient_exact
    }

    /// Evaluate the quadratic row with bound variable values.
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

/// Exact coefficients for one univariate quadratic residual.
///
/// The row represents `quadratic * x^2 + linear * x + constant` for one solver
/// symbol. It is not a general CAS form; it is a bounded structural summary
/// meant to keep small polynomial rows inspectable before a lossy nonlinear
/// backend proposes a candidate.
#[derive(Clone, Debug, PartialEq)]
pub struct UnivariateQuadraticResidual {
    symbol: SymbolId,
    constant: Real,
    linear: Real,
    quadratic: Real,
    coefficient_exact: RealExactSetFacts,
}

impl UnivariateQuadraticResidual {
    /// Extract a univariate quadratic residual form from an expression.
    ///
    /// Returns `None` when the expression is not a quadratic in exactly one
    /// problem variable.
    /// Intermediate higher-degree cancellations are retained through degree
    /// 64; exceeding this expansion limit returns `None`.
    pub fn from_expr(expression: &Expr, problem: &Problem) -> Option<Self> {
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
        let coefficient_exact =
            Real::exact_set_facts([&poly.constant, &poly.linear, &poly.quadratic]);
        Some(Self {
            symbol,
            constant: poly.constant,
            linear: poly.linear,
            quadratic: poly.quadratic,
            coefficient_exact,
        })
    }

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

    /// Evaluate the quadratic row with bound variable values.
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

#[derive(Clone, Debug)]
struct PolynomialAccumulator {
    symbol: Option<SymbolId>,
    constant: Real,
    linear: Real,
    quadratic: Real,
    higher: Vec<(usize, Real)>,
}

impl PolynomialAccumulator {
    fn constant(value: Real) -> Self {
        Self {
            symbol: None,
            constant: value,
            linear: Real::zero(),
            quadratic: Real::zero(),
            higher: Vec::new(),
        }
    }

    fn symbol(symbol: SymbolId) -> Self {
        Self {
            symbol: Some(symbol),
            constant: Real::zero(),
            linear: Real::one(),
            quadratic: Real::zero(),
            higher: Vec::new(),
        }
    }

    fn scale(mut self, scale: Real) -> Self {
        self.constant *= scale.clone();
        self.linear *= scale.clone();
        self.quadratic *= scale.clone();
        self.higher.retain_mut(|(_, coefficient)| {
            *coefficient *= scale.clone();
            !is_structural_zero(coefficient)
        });
        self
    }

    fn add(self, other: Self) -> Option<Self> {
        let symbol = merge_symbol(self.symbol, other.symbol)?;
        let mut higher = self.higher;
        for (degree, coefficient) in other.higher {
            add_univariate_coefficient(&mut higher, degree, coefficient);
        }
        Some(Self {
            symbol,
            constant: self.constant + other.constant,
            linear: self.linear + other.linear,
            quadratic: self.quadratic + other.quadratic,
            higher,
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
        let mut higher = Vec::new();
        add_univariate_coefficient(
            &mut higher,
            3,
            self.linear.clone() * other.quadratic.clone()
                + self.quadratic.clone() * other.linear.clone(),
        );
        add_univariate_coefficient(
            &mut higher,
            4,
            self.quadratic.clone() * other.quadratic.clone(),
        );
        if !self.higher.is_empty() || !other.higher.is_empty() {
            multiply_higher_terms(&mut higher, &self, &other)?;
        }
        Some(Self {
            symbol,
            constant,
            linear,
            quadratic,
            higher,
        })
    }

    fn is_higher_zero(&self) -> bool {
        self.higher.is_empty()
    }
}

fn add_univariate_coefficient(terms: &mut Vec<(usize, Real)>, degree: usize, coefficient: Real) {
    if is_structural_zero(&coefficient) {
        return;
    }
    match terms.binary_search_by_key(&degree, |(degree, _)| *degree) {
        Ok(index) => {
            terms[index].1 += coefficient;
            if is_structural_zero(&terms[index].1) {
                terms.remove(index);
            }
        }
        Err(index) => terms.insert(index, (degree, coefficient)),
    }
}

fn add_coefficient(
    terms: &mut BTreeMap<Vec<SymbolId>, Real>,
    key: Vec<SymbolId>,
    coefficient: Real,
) {
    use std::collections::btree_map::Entry;
    if is_structural_zero(&coefficient) {
        return;
    }
    match terms.entry(key) {
        Entry::Vacant(entry) => {
            entry.insert(coefficient);
        }
        Entry::Occupied(mut entry) => {
            *entry.get_mut() += coefficient;
            if is_structural_zero(entry.get()) {
                entry.remove();
            }
        }
    }
}

#[cold]
fn multiply_higher_terms(
    higher: &mut Vec<(usize, Real)>,
    left: &PolynomialAccumulator,
    right: &PolynomialAccumulator,
) -> Option<()> {
    let left_low = [&left.constant, &left.linear, &left.quadratic];
    let right_low = [&right.constant, &right.linear, &right.quadratic];
    let mut accumulate = |degree: usize, first: &Real, second: &Real| {
        if is_structural_zero(first) || is_structural_zero(second) {
            return Some(());
        }
        if degree > MAX_CANCELLATION_DEGREE {
            return None;
        }
        add_univariate_coefficient(higher, degree, first.clone() * second.clone());
        Some(())
    };
    for (degree, coefficient) in &left.higher {
        for (offset, low) in right_low.iter().enumerate() {
            accumulate(degree + offset, coefficient, low)?;
        }
        for (other_degree, other_coefficient) in &right.higher {
            accumulate(degree + other_degree, coefficient, other_coefficient)?;
        }
    }
    for (degree, coefficient) in &right.higher {
        for (offset, low) in left_low.iter().enumerate() {
            accumulate(degree + offset, low, coefficient)?;
        }
    }
    Some(())
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
        // Constant nonzero bases are folded by `Expr::simplify`. A remaining
        // zero exponent still carries the `base != 0` domain obligation and
        // therefore cannot be replaced by a total polynomial constant.
        Expr::PowI(_, 0) => None,
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
}

impl MultivariateQuadraticAccumulator {
    fn constant(value: Real) -> Self {
        let mut terms = BTreeMap::new();
        terms.insert(Vec::new(), value);
        Self { terms }
    }

    fn symbol(symbol: SymbolId) -> Self {
        let mut terms = BTreeMap::new();
        terms.insert(vec![symbol], Real::one());
        Self { terms }
    }

    fn scale(mut self, scale: Real) -> Self {
        self.terms.retain(|_, coefficient| {
            *coefficient *= scale.clone();
            !is_structural_zero(coefficient)
        });
        self
    }

    fn add(mut self, other: Self) -> Option<Self> {
        for (symbols, coefficient) in other.terms {
            add_coefficient(&mut self.terms, symbols, coefficient);
        }
        self.within_cancellation_limit().then_some(self)
    }

    fn multiply(self, other: Self) -> Option<Self> {
        let mut result = Self {
            terms: BTreeMap::new(),
        };
        let mut higher_count = 0;
        for (left_symbols, left_coefficient) in self.terms {
            if is_structural_zero(&left_coefficient) {
                continue;
            }
            for (right_symbols, right_coefficient) in &other.terms {
                if is_structural_zero(right_coefficient) {
                    continue;
                }
                if left_symbols.len() + right_symbols.len() > MAX_CANCELLATION_DEGREE {
                    return None;
                }
                let mut symbols = left_symbols.clone();
                symbols.extend(right_symbols.iter().copied());
                symbols.sort();
                let coefficient = left_coefficient.clone() * right_coefficient.clone();
                let higher = symbols.len() > 2;
                let before = result.terms.len();
                add_coefficient(&mut result.terms, symbols, coefficient);
                if higher {
                    higher_count = higher_count + result.terms.len() - before;
                    if higher_count > MAX_CANCELLATION_TERMS {
                        return None;
                    }
                }
            }
        }
        Some(result)
    }

    fn is_higher_zero(&self) -> bool {
        self.terms.keys().all(|symbols| symbols.len() <= 2)
    }

    fn within_cancellation_limit(&self) -> bool {
        self.terms
            .keys()
            .filter(|symbols| symbols.len() > 2)
            .count()
            <= MAX_CANCELLATION_TERMS
    }
}

fn collect_multivariate_quadratic(
    expression: &Expr,
    problem: &Problem,
) -> Option<MultivariateQuadraticAccumulator> {
    match expression {
        Expr::Constant(value) => Some(MultivariateQuadraticAccumulator::constant(value.clone())),
        Expr::Symbol(symbol) => problem
            .variables
            .iter()
            .any(|variable| variable.symbol == symbol.id)
            .then(|| MultivariateQuadraticAccumulator::symbol(symbol.id)),
        Expr::Add(left, right) => collect_multivariate_quadratic(left, problem)?
            .add(collect_multivariate_quadratic(right, problem)?),
        Expr::Sub(left, right) => collect_multivariate_quadratic(left, problem)?
            .add(collect_multivariate_quadratic(right, problem)?.scale(-Real::one())),
        Expr::Neg(value) => {
            Some(collect_multivariate_quadratic(value, problem)?.scale(-Real::one()))
        }
        Expr::Mul(left, right) => collect_multivariate_quadratic(left, problem)?
            .multiply(collect_multivariate_quadratic(right, problem)?),
        Expr::Div(left, right) => {
            let denominator = constant_value(right)?;
            let reciprocal = (Real::one() / denominator).ok()?;
            Some(collect_multivariate_quadratic(left, problem)?.scale(reciprocal))
        }
        // See the univariate collector above: retaining `x^0` is necessary to
        // preserve the undefined `0^0` case.
        Expr::PowI(_, 0) => None,
        Expr::PowI(value, 1) => collect_multivariate_quadratic(value, problem),
        Expr::PowI(value, 2) => {
            let value = collect_multivariate_quadratic(value, problem)?;
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

fn ensure_problem_symbol(variables: &[Variable], symbol: SymbolId) -> Result<(), ExprEvalError> {
    if variables.iter().any(|variable| variable.symbol == symbol) {
        Ok(())
    } else {
        Err(ExprEvalError::UnboundSymbol(SymbolRef::new(symbol, None)))
    }
}

fn is_structural_zero(value: &Real) -> bool {
    match value.zero_status() {
        hyperreal::ZeroKnowledge::Zero => true,
        hyperreal::ZeroKnowledge::NonZero => false,
        // Preserve any additional exact-rational normalization available to
        // structural_facts without collecting magnitude facts in the common
        // rational and named-symbolic cases.
        hyperreal::ZeroKnowledge::Unknown => matches!(
            value.structural_facts().zero,
            hyperreal::ZeroKnowledge::Zero
        ),
    }
}
