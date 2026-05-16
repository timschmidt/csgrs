use std::collections::{BTreeSet, HashMap};
use std::ops::{Add, Div, Mul, Neg, Sub};

use hyperreal::{Problem as HyperProblem, Rational, Real, SymbolicDependencyMask};

/// Stable identifier for a solve variable.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct SymbolId(pub u32);

/// Metadata attached to a symbolic variable reference.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SymbolRef {
    pub id: SymbolId,
    pub name: Option<String>,
}

impl SymbolRef {
    pub fn new(id: SymbolId, name: Option<String>) -> Self {
        Self { id, name }
    }
}

/// Conservative degree classification for a solver expression.
///
/// This is not a general CAS proof. It is a cheap structural summary over the
/// current expression tree, meant to select solver strategies before lowering
/// variables into [`Real`]. Polynomial degrees are reported only when the tree
/// shape proves them directly; division by a variable expression and
/// transcendental operations are marked non-polynomial. This follows Yap's
/// exact-geometric-computation guidance to preserve expression structure and
/// choose arithmetic/solver packages before forcing scalar evaluation. See
/// Yap, "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
/// (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExprDegree {
    /// The expression has no solver-variable dependency.
    Constant,
    /// The expression is structurally polynomial in solver variables.
    Polynomial(u32),
    /// The expression depends on variables through a non-polynomial form.
    NonPolynomial,
}

impl ExprDegree {
    /// Returns whether this expression is structurally affine or constant.
    pub const fn is_affine_or_constant(self) -> bool {
        matches!(self, Self::Constant | Self::Polynomial(0 | 1))
    }

    /// Returns whether this expression is structurally polynomial.
    pub const fn is_polynomial(self) -> bool {
        matches!(self, Self::Constant | Self::Polynomial(_))
    }
}

/// Conservative structural facts for a solver expression.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExprFacts {
    /// Solver variables referenced by the expression.
    pub dependencies: BTreeSet<SymbolId>,
    /// Conservative expression-degree classification.
    pub degree: ExprDegree,
    /// Coarse symbolic families present in embedded [`Real`] constants.
    pub real_symbolic_dependencies: SymbolicDependencyMask,
}

impl ExprFacts {
    /// Returns whether the expression has no solver-variable dependencies.
    pub fn is_constant(&self) -> bool {
        self.dependencies.is_empty()
    }

    /// Returns whether the expression is structurally affine in solver
    /// variables.
    pub fn is_affine(&self) -> bool {
        !self.is_constant() && self.degree == ExprDegree::Polynomial(1)
    }
}

/// Variable-aware expression tree.
///
/// This deliberately lives in hypersolve first. `Real` keeps its current
/// invariants, while `Expr` can track dependencies and lower to `Real` after
/// variables are bound.
#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    Constant(Real),
    Symbol(SymbolRef),
    Add(Box<Expr>, Box<Expr>),
    Sub(Box<Expr>, Box<Expr>),
    Mul(Box<Expr>, Box<Expr>),
    Div(Box<Expr>, Box<Expr>),
    Neg(Box<Expr>),
    PowI(Box<Expr>, i64),
    Sqrt(Box<Expr>),
    Sin(Box<Expr>),
    Cos(Box<Expr>),
    Log10(Box<Expr>),
}

impl Expr {
    pub fn real(value: Real) -> Self {
        Self::Constant(value)
    }

    pub fn int(value: i64) -> Self {
        Self::Constant(Real::new(Rational::new(value)))
    }

    pub fn zero() -> Self {
        Self::int(0)
    }

    pub fn symbol(id: SymbolId, name: impl Into<String>) -> Self {
        Self::Symbol(SymbolRef::new(id, Some(name.into())))
    }

    pub fn anonymous_symbol(id: SymbolId) -> Self {
        Self::Symbol(SymbolRef::new(id, None))
    }

    pub fn powi(self, exponent: i64) -> Self {
        Self::PowI(Box::new(self), exponent)
    }

    pub fn sqrt(self) -> Self {
        Self::Sqrt(Box::new(self))
    }

    pub fn sin(self) -> Self {
        Self::Sin(Box::new(self))
    }

    pub fn cos(self) -> Self {
        Self::Cos(Box::new(self))
    }

    pub fn log10(self) -> Self {
        Self::Log10(Box::new(self))
    }

    pub fn dependencies(&self) -> BTreeSet<SymbolId> {
        let mut deps = BTreeSet::new();
        self.collect_dependencies(&mut deps);
        deps
    }

    /// Returns conservative structural facts for this expression.
    ///
    /// The facts are intended for solver dispatch and Jacobian sparsity:
    /// residual blocks can detect constants, affine rows, polynomial rows, and
    /// non-polynomial rows before asking for dense f64 estimates. Embedded
    /// `Real` constants contribute only their public symbolic dependency mask,
    /// preserving `hyperreal`'s scalar abstraction boundary.
    pub fn structural_facts(&self) -> ExprFacts {
        let mut dependencies = BTreeSet::new();
        let mut real_symbolic_dependencies = SymbolicDependencyMask::NONE;
        let degree = self.collect_facts(&mut dependencies, &mut real_symbolic_dependencies);
        ExprFacts {
            dependencies,
            degree,
            real_symbolic_dependencies,
        }
    }

    fn collect_dependencies(&self, deps: &mut BTreeSet<SymbolId>) {
        match self {
            Self::Constant(_) => {}
            Self::Symbol(symbol) => {
                deps.insert(symbol.id);
            }
            Self::Add(left, right)
            | Self::Sub(left, right)
            | Self::Mul(left, right)
            | Self::Div(left, right) => {
                left.collect_dependencies(deps);
                right.collect_dependencies(deps);
            }
            Self::Neg(value)
            | Self::PowI(value, _)
            | Self::Sqrt(value)
            | Self::Sin(value)
            | Self::Cos(value)
            | Self::Log10(value) => value.collect_dependencies(deps),
        }
    }

    fn collect_facts(
        &self,
        deps: &mut BTreeSet<SymbolId>,
        real_dependencies: &mut SymbolicDependencyMask,
    ) -> ExprDegree {
        match self {
            Self::Constant(value) => {
                *real_dependencies =
                    real_dependencies.union(value.detailed_facts().symbolic.dependencies);
                ExprDegree::Constant
            }
            Self::Symbol(symbol) => {
                deps.insert(symbol.id);
                ExprDegree::Polynomial(1)
            }
            Self::Add(left, right) | Self::Sub(left, right) => merge_additive_degrees(
                left.collect_facts(deps, real_dependencies),
                right.collect_facts(deps, real_dependencies),
            ),
            Self::Mul(left, right) => merge_multiplicative_degrees(
                left.collect_facts(deps, real_dependencies),
                right.collect_facts(deps, real_dependencies),
            ),
            Self::Div(left, right) => {
                let left_degree = left.collect_facts(deps, real_dependencies);
                let right_degree = right.collect_facts(deps, real_dependencies);
                match (left_degree, right_degree) {
                    (degree, ExprDegree::Constant) => degree,
                    (ExprDegree::Constant, ExprDegree::Polynomial(_)) => ExprDegree::NonPolynomial,
                    (ExprDegree::Polynomial(_), ExprDegree::Polynomial(_))
                    | (ExprDegree::NonPolynomial, _)
                    | (_, ExprDegree::NonPolynomial) => ExprDegree::NonPolynomial,
                }
            }
            Self::Neg(value) => value.collect_facts(deps, real_dependencies),
            Self::PowI(value, exponent) => {
                let degree = value.collect_facts(deps, real_dependencies);
                pow_degree(degree, *exponent)
            }
            Self::Sqrt(value) | Self::Sin(value) | Self::Cos(value) | Self::Log10(value) => {
                match value.collect_facts(deps, real_dependencies) {
                    ExprDegree::Constant => ExprDegree::Constant,
                    ExprDegree::Polynomial(_) | ExprDegree::NonPolynomial => {
                        ExprDegree::NonPolynomial
                    }
                }
            }
        }
    }

    pub fn eval_real(&self, bindings: &HashMap<SymbolId, Real>) -> Result<Real, ExprEvalError> {
        match self {
            Self::Constant(value) => Ok(value.clone()),
            Self::Symbol(symbol) => bindings
                .get(&symbol.id)
                .cloned()
                .ok_or(ExprEvalError::UnboundSymbol(symbol.clone())),
            Self::Add(left, right) => Ok(left.eval_real(bindings)? + right.eval_real(bindings)?),
            Self::Sub(left, right) => Ok(left.eval_real(bindings)? - right.eval_real(bindings)?),
            Self::Mul(left, right) => Ok(left.eval_real(bindings)? * right.eval_real(bindings)?),
            Self::Div(left, right) => (left.eval_real(bindings)? / right.eval_real(bindings)?)
                .map_err(ExprEvalError::Hyperreal),
            Self::Neg(value) => Ok(-value.eval_real(bindings)?),
            Self::PowI(value, exponent) => value
                .eval_real(bindings)?
                .powi((*exponent).into())
                .map_err(ExprEvalError::Hyperreal),
            Self::Sqrt(value) => value
                .eval_real(bindings)?
                .sqrt()
                .map_err(ExprEvalError::Hyperreal),
            Self::Sin(value) => Ok(value.eval_real(bindings)?.sin()),
            Self::Cos(value) => Ok(value.eval_real(bindings)?.cos()),
            Self::Log10(value) => value
                .eval_real(bindings)?
                .log10()
                .map_err(ExprEvalError::Hyperreal),
        }
    }

    pub fn simplify(self) -> Self {
        match self {
            Self::Add(left, right) => simplify_add(left.simplify(), right.simplify()),
            Self::Sub(left, right) => simplify_sub(left.simplify(), right.simplify()),
            Self::Mul(left, right) => simplify_mul(left.simplify(), right.simplify()),
            Self::Div(left, right) => simplify_div(left.simplify(), right.simplify()),
            Self::Neg(value) => simplify_neg(value.simplify()),
            Self::PowI(value, exponent) => simplify_powi(value.simplify(), exponent),
            Self::Sqrt(value) => Self::Sqrt(Box::new(value.simplify())),
            Self::Sin(value) => Self::Sin(Box::new(value.simplify())),
            Self::Cos(value) => Self::Cos(Box::new(value.simplify())),
            Self::Log10(value) => Self::Log10(Box::new(value.simplify())),
            value => value,
        }
    }

    pub fn derivative(&self, symbol: SymbolId) -> Self {
        match self {
            Self::Constant(_) => Self::zero(),
            Self::Symbol(reference) if reference.id == symbol => Self::int(1),
            Self::Symbol(_) => Self::zero(),
            Self::Add(left, right) => {
                left.derivative(symbol).simplify() + right.derivative(symbol).simplify()
            }
            Self::Sub(left, right) => {
                left.derivative(symbol).simplify() - right.derivative(symbol).simplify()
            }
            Self::Mul(left, right) => {
                ((*left.clone()) * right.derivative(symbol))
                    + (left.derivative(symbol) * (*right.clone()))
            }
            Self::Div(left, right) => {
                let numerator = (left.derivative(symbol) * (*right.clone()))
                    - ((*left.clone()) * right.derivative(symbol));
                let denominator = (*right.clone()).powi(2);
                numerator / denominator
            }
            Self::Neg(value) => -value.derivative(symbol),
            Self::PowI(value, exponent) => {
                if *exponent == 0 {
                    Self::zero()
                } else {
                    Self::int(*exponent)
                        * (*value.clone()).powi(*exponent - 1)
                        * value.derivative(symbol)
                }
            }
            Self::Sqrt(value) => {
                value.derivative(symbol) / (Self::int(2) * (*value.clone()).sqrt())
            }
            Self::Sin(value) => value.derivative(symbol) * (*value.clone()).cos(),
            Self::Cos(value) => -(value.derivative(symbol) * (*value.clone()).sin()),
            Self::Log10(value) => {
                let ln_10 = Real::new(Rational::new(10))
                    .ln()
                    .expect("ln(10) is in domain");
                value.derivative(symbol) / ((*value.clone()) * Self::real(ln_10))
            }
        }
        .simplify()
    }
}

fn merge_additive_degrees(left: ExprDegree, right: ExprDegree) -> ExprDegree {
    match (left, right) {
        (ExprDegree::NonPolynomial, _) | (_, ExprDegree::NonPolynomial) => {
            ExprDegree::NonPolynomial
        }
        (ExprDegree::Constant, degree) | (degree, ExprDegree::Constant) => degree,
        (ExprDegree::Polynomial(left), ExprDegree::Polynomial(right)) => {
            ExprDegree::Polynomial(left.max(right))
        }
    }
}

fn merge_multiplicative_degrees(left: ExprDegree, right: ExprDegree) -> ExprDegree {
    match (left, right) {
        (ExprDegree::NonPolynomial, _) | (_, ExprDegree::NonPolynomial) => {
            ExprDegree::NonPolynomial
        }
        (ExprDegree::Constant, ExprDegree::Constant) => ExprDegree::Constant,
        (ExprDegree::Constant, degree) | (degree, ExprDegree::Constant) => degree,
        (ExprDegree::Polynomial(left), ExprDegree::Polynomial(right)) => {
            ExprDegree::Polynomial(left.saturating_add(right))
        }
    }
}

fn pow_degree(value: ExprDegree, exponent: i64) -> ExprDegree {
    match (value, exponent) {
        (_, 0) => ExprDegree::Constant,
        (ExprDegree::Constant, _) => ExprDegree::Constant,
        (ExprDegree::Polynomial(degree), exponent) if exponent > 0 => {
            ExprDegree::Polynomial(degree.saturating_mul(exponent as u32))
        }
        (ExprDegree::Polynomial(_), _) | (ExprDegree::NonPolynomial, _) => {
            ExprDegree::NonPolynomial
        }
    }
}

#[derive(Clone, Debug)]
pub enum ExprEvalError {
    UnboundSymbol(SymbolRef),
    /// A prepared expression object no longer matches the model shape it was
    /// derived from.
    ///
    /// Exact-computation metadata is only valid while its structural
    /// assumptions still hold. Yap frames this as choosing an arithmetic and
    /// representation package for the current problem formulation, not as
    /// creating a model-independent scalar cache. See Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7.1-2 (1997).
    PreparedShapeMismatch {
        expected_coefficients: usize,
        actual_variables: usize,
    },
    Hyperreal(HyperProblem),
}

impl Add for Expr {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::Add(Box::new(self), Box::new(rhs))
    }
}

impl Sub for Expr {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::Sub(Box::new(self), Box::new(rhs))
    }
}

impl Mul for Expr {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self::Mul(Box::new(self), Box::new(rhs))
    }
}

impl Div for Expr {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        Self::Div(Box::new(self), Box::new(rhs))
    }
}

impl Neg for Expr {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::Neg(Box::new(self))
    }
}

fn simplify_add(left: Expr, right: Expr) -> Expr {
    match (left, right) {
        (left, Expr::Constant(value)) if value == Real::zero() => left,
        (Expr::Constant(value), right) if value == Real::zero() => right,
        (Expr::Constant(left), Expr::Constant(right)) => Expr::Constant(left + right),
        (left, right) => Expr::Add(Box::new(left), Box::new(right)),
    }
}

fn simplify_sub(left: Expr, right: Expr) -> Expr {
    match (left, right) {
        (left, Expr::Constant(value)) if value == Real::zero() => left,
        (Expr::Constant(left), Expr::Constant(right)) => Expr::Constant(left - right),
        (left, right) => Expr::Sub(Box::new(left), Box::new(right)),
    }
}

fn simplify_mul(left: Expr, right: Expr) -> Expr {
    match (left, right) {
        (_, Expr::Constant(value)) if value == Real::zero() => Expr::zero(),
        (Expr::Constant(value), _) if value == Real::zero() => Expr::zero(),
        (left, Expr::Constant(value)) if value == Real::one() => left,
        (Expr::Constant(value), right) if value == Real::one() => right,
        (Expr::Constant(left), Expr::Constant(right)) => Expr::Constant(left * right),
        (left, right) => Expr::Mul(Box::new(left), Box::new(right)),
    }
}

fn simplify_div(left: Expr, right: Expr) -> Expr {
    match (left, right) {
        (Expr::Constant(value), _) if value == Real::zero() => Expr::zero(),
        (left, Expr::Constant(value)) if value == Real::one() => left,
        (Expr::Constant(left), Expr::Constant(right)) => match left.clone() / right.clone() {
            Ok(value) => Expr::Constant(value),
            Err(_) => Expr::Div(
                Box::new(Expr::Constant(left)),
                Box::new(Expr::Constant(right)),
            ),
        },
        (left, right) => Expr::Div(Box::new(left), Box::new(right)),
    }
}

fn simplify_neg(value: Expr) -> Expr {
    match value {
        Expr::Constant(value) => Expr::Constant(-value),
        Expr::Neg(value) => *value,
        value => Expr::Neg(Box::new(value)),
    }
}

fn simplify_powi(value: Expr, exponent: i64) -> Expr {
    match (value, exponent) {
        (_, 0) => Expr::int(1),
        (value, 1) => value,
        (Expr::Constant(value), exponent) => match value.powi(exponent.into()) {
            Ok(value) => Expr::Constant(value),
            Err(_) => Expr::PowI(Box::new(Expr::zero()), exponent),
        },
        (value, exponent) => Expr::PowI(Box::new(value), exponent),
    }
}
