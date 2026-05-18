//! Typed genetic-programming expression genomes.
//!
//! Genetic programming can propose symbolic expressions, residual templates, or
//! domain constructors, but generated programs must be checked before
//! evaluation. This module starts with a small typed `Real` expression AST and
//! report-bearing validation for arity, depth, node budget, unbound inputs, and
//! division by structurally zero constants. The GP framing follows Koza,
//! "Genetic Programming" (1992); the exact/replay boundary follows Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7(1-2),
//! 1997: candidate programs propose structure, but exact values and domain
//! reports decide acceptance.

use std::collections::HashMap;

use hyperreal::{Real, RealSign};

/// Typed GP expression over exact `Real` values.
#[derive(Clone, Debug, PartialEq)]
pub enum GpRealExpr {
    /// Exact constant.
    Constant(Box<Real>),
    /// Input slot by arity index.
    Input(usize),
    /// Addition.
    Add(Box<GpRealExpr>, Box<GpRealExpr>),
    /// Subtraction.
    Sub(Box<GpRealExpr>, Box<GpRealExpr>),
    /// Multiplication.
    Mul(Box<GpRealExpr>, Box<GpRealExpr>),
    /// Checked division.
    Div(Box<GpRealExpr>, Box<GpRealExpr>),
    /// Negation.
    Neg(Box<GpRealExpr>),
}

/// Validation limits for GP expression genomes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GpValidationLimits {
    /// Number of input slots available to the expression.
    pub input_arity: usize,
    /// Maximum allowed tree depth. A leaf has depth 1.
    pub max_depth: usize,
    /// Maximum allowed node count.
    pub max_nodes: usize,
}

/// One validation or evaluation issue.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum GpValidationIssue {
    /// An input index was outside the declared arity.
    InputOutOfBounds {
        /// Referenced input.
        input: usize,
        /// Declared arity.
        arity: usize,
    },
    /// Tree exceeded the configured depth budget.
    DepthExceeded {
        /// Observed depth.
        depth: usize,
        /// Allowed depth.
        max_depth: usize,
    },
    /// Tree exceeded the configured node budget.
    NodeBudgetExceeded {
        /// Observed node count.
        nodes: usize,
        /// Allowed nodes.
        max_nodes: usize,
    },
    /// A division denominator is structurally known to be zero.
    DivisionByZero,
    /// Evaluation input was missing.
    MissingInput {
        /// Missing input index.
        input: usize,
    },
    /// Exact scalar division was not supported by the scalar layer.
    UnsupportedDivision,
}

/// Report produced before evaluating a GP expression.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GpValidationReport {
    /// Observed depth.
    pub depth: usize,
    /// Observed node count.
    pub nodes: usize,
    /// Validation issues.
    pub issues: Vec<GpValidationIssue>,
}

impl GpValidationReport {
    /// Returns whether the expression passed all checks.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

impl GpRealExpr {
    /// Validates arity and bloat limits before evaluation.
    pub fn validate(&self, limits: GpValidationLimits) -> GpValidationReport {
        let mut issues = Vec::new();
        let depth = self.depth();
        let nodes = self.node_count();
        if depth > limits.max_depth {
            issues.push(GpValidationIssue::DepthExceeded {
                depth,
                max_depth: limits.max_depth,
            });
        }
        if nodes > limits.max_nodes {
            issues.push(GpValidationIssue::NodeBudgetExceeded {
                nodes,
                max_nodes: limits.max_nodes,
            });
        }
        self.collect_validation_issues(limits.input_arity, &mut issues);
        GpValidationReport {
            depth,
            nodes,
            issues,
        }
    }

    /// Evaluates a validated GP expression over exact inputs.
    pub fn eval(&self, inputs: &[Real]) -> Result<Real, GpValidationIssue> {
        match self {
            Self::Constant(value) => Ok((**value).clone()),
            Self::Input(index) => inputs
                .get(*index)
                .cloned()
                .ok_or(GpValidationIssue::MissingInput { input: *index }),
            Self::Add(left, right) => Ok(left.eval(inputs)? + right.eval(inputs)?),
            Self::Sub(left, right) => Ok(left.eval(inputs)? - right.eval(inputs)?),
            Self::Mul(left, right) => Ok(left.eval(inputs)? * right.eval(inputs)?),
            Self::Div(left, right) => (left.eval(inputs)? / right.eval(inputs)?)
                .map_err(|_| GpValidationIssue::UnsupportedDivision),
            Self::Neg(value) => Ok(-value.eval(inputs)?),
        }
    }

    /// Returns the expression depth. A leaf has depth 1.
    pub fn depth(&self) -> usize {
        match self {
            Self::Constant(_) | Self::Input(_) => 1,
            Self::Neg(value) => 1 + value.depth(),
            Self::Add(left, right)
            | Self::Sub(left, right)
            | Self::Mul(left, right)
            | Self::Div(left, right) => 1 + left.depth().max(right.depth()),
        }
    }

    /// Returns the expression node count.
    pub fn node_count(&self) -> usize {
        match self {
            Self::Constant(_) | Self::Input(_) => 1,
            Self::Neg(value) => 1 + value.node_count(),
            Self::Add(left, right)
            | Self::Sub(left, right)
            | Self::Mul(left, right)
            | Self::Div(left, right) => 1 + left.node_count() + right.node_count(),
        }
    }

    fn collect_validation_issues(&self, arity: usize, issues: &mut Vec<GpValidationIssue>) {
        match self {
            Self::Input(index) if *index >= arity => {
                issues.push(GpValidationIssue::InputOutOfBounds {
                    input: *index,
                    arity,
                })
            }
            Self::Div(_, right)
                if matches!(
                    right
                        .constant_value()
                        .map(|value| value.structural_facts().sign),
                    Some(Some(RealSign::Zero))
                ) =>
            {
                issues.push(GpValidationIssue::DivisionByZero);
            }
            _ => {}
        }
        for child in self.children() {
            child.collect_validation_issues(arity, issues);
        }
    }

    fn children(&self) -> Vec<&GpRealExpr> {
        match self {
            Self::Add(left, right)
            | Self::Sub(left, right)
            | Self::Mul(left, right)
            | Self::Div(left, right) => vec![left, right],
            Self::Neg(value) => vec![value],
            Self::Constant(_) | Self::Input(_) => Vec::new(),
        }
    }

    fn constant_value(&self) -> Option<Real> {
        match self {
            Self::Constant(value) => Some((**value).clone()),
            Self::Neg(value) => value.constant_value().map(|value| -value),
            Self::Add(left, right) => Some(left.constant_value()? + right.constant_value()?),
            Self::Sub(left, right) => Some(left.constant_value()? - right.constant_value()?),
            Self::Mul(left, right) => Some(left.constant_value()? * right.constant_value()?),
            Self::Div(left, right) => (left.constant_value()? / right.constant_value()?).ok(),
            Self::Input(_) => None,
        }
    }
}

/// Evaluates a batch of validated expressions, preserving per-expression errors.
pub fn eval_gp_batch(
    expressions: &[GpRealExpr],
    inputs: &HashMap<usize, Real>,
) -> Vec<Result<Real, GpValidationIssue>> {
    let dense_inputs = dense_input_vector(inputs);
    expressions
        .iter()
        .map(|expression| expression.eval(&dense_inputs))
        .collect()
}

fn dense_input_vector(inputs: &HashMap<usize, Real>) -> Vec<Real> {
    let len = inputs
        .keys()
        .copied()
        .max()
        .map(|index| index + 1)
        .unwrap_or(0);
    let mut values = vec![Real::zero(); len];
    for (index, value) in inputs {
        values[*index] = value.clone();
    }
    values
}
