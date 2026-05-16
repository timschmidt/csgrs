//! Prepared affine residual blocks for exact solver evaluation.
//!
//! This module keeps affine row structure next to the solver model instead of
//! rediscovering it while forming residuals or Jacobians. The dense f64 solver
//! adapter still owns lossy lowering, but affine rows can now preserve exact
//! coefficient/product-sum shape up to that boundary.

use std::collections::HashMap;

use hyperreal::{Real, RealExactSetFacts};

use crate::model::{Problem, Variable};
use crate::symbolic::{Expr, ExprEvalError, SymbolId, SymbolRef};

/// Prepared exact coefficients for one affine residual row.
///
/// The row represents `constant + sum(coefficients[i] * variables[i])` in the
/// source problem's variable order. It is deliberately solver-owned metadata:
/// it is not a general symbolic optimizer, and it does not decide feasibility.
/// It preserves the bounded affine product-sum shape so exact or lossy solver
/// adapters can choose an arithmetic package before scalar expansion. This is
/// the expression-layer separation advocated by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997). The exact rational
/// product-sum route follows the same delayed-normalization idea as Bareiss,
/// "Sylvester's Identity and Multistep Integer-Preserving Gaussian
/// Elimination," *Mathematics of Computation* 22.103 (1968).
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedAffineResidual {
    constant: Real,
    coefficients: Vec<Real>,
    coefficient_exact: RealExactSetFacts,
    nonzero_coefficient_count: usize,
}

impl PreparedAffineResidual {
    /// Return the residual constant term.
    pub fn constant(&self) -> &Real {
        &self.constant
    }

    /// Return coefficients in source-problem variable order.
    pub fn coefficients(&self) -> &[Real] {
        &self.coefficients
    }

    /// Return exact-set facts for coefficients plus the constant term.
    pub fn coefficient_exact(&self) -> RealExactSetFacts {
        self.coefficient_exact
    }

    /// Return the number of structurally nonzero coefficients.
    pub fn nonzero_coefficient_count(&self) -> usize {
        self.nonzero_coefficient_count
    }

    /// Returns whether this row has no variable coefficients.
    pub fn is_constant(&self) -> bool {
        self.nonzero_coefficient_count == 0
    }

    /// Returns whether all coefficients and the constant are exact rationals.
    pub fn is_exact_rational(&self) -> bool {
        self.coefficient_exact.all_exact_rational
    }

    /// Evaluate the affine row with bound variable values.
    ///
    /// Rows with up to four variables use fixed-arity product-sum builders so
    /// the affine polynomial is preserved as one scalar package. Larger rows
    /// fall back to exact scalar accumulation; that keeps the API general while
    /// avoiding an unbounded symbolic optimizer in this hot path.
    pub fn eval_real(
        &self,
        variables: &[Variable],
        bindings: &HashMap<SymbolId, Real>,
    ) -> Result<Real, ExprEvalError> {
        if self.coefficients.len() != variables.len() {
            return Err(ExprEvalError::PreparedShapeMismatch {
                expected_coefficients: self.coefficients.len(),
                actual_variables: variables.len(),
            });
        }

        let values = variables
            .iter()
            .map(|variable| {
                bindings.get(&variable.symbol).ok_or_else(|| {
                    ExprEvalError::UnboundSymbol(SymbolRef::new(
                        variable.symbol,
                        Some(variable.name.clone()),
                    ))
                })
            })
            .collect::<Result<Vec<_>, _>>()?;

        let value_exact = Real::exact_set_facts(values.iter().copied());
        if self.is_exact_rational() && value_exact.all_exact_rational {
            match self.coefficients.len() {
                0 => return Ok(self.constant.clone()),
                1 => {
                    let one = Real::one();
                    return Ok(Real::exact_rational_signed_product_sum_known_exact(
                        [true, true],
                        [[&self.coefficients[0], values[0]], [&self.constant, &one]],
                    ));
                }
                2 => {
                    let one = Real::one();
                    return Ok(Real::exact_rational_signed_product_sum_known_exact(
                        [true, true, true],
                        [
                            [&self.coefficients[0], values[0]],
                            [&self.coefficients[1], values[1]],
                            [&self.constant, &one],
                        ],
                    ));
                }
                3 => {
                    let one = Real::one();
                    return Ok(Real::exact_rational_signed_product_sum_known_exact(
                        [true, true, true, true],
                        [
                            [&self.coefficients[0], values[0]],
                            [&self.coefficients[1], values[1]],
                            [&self.coefficients[2], values[2]],
                            [&self.constant, &one],
                        ],
                    ));
                }
                4 => {
                    let one = Real::one();
                    return Ok(Real::exact_rational_signed_product_sum_known_exact(
                        [true, true, true, true, true],
                        [
                            [&self.coefficients[0], values[0]],
                            [&self.coefficients[1], values[1]],
                            [&self.coefficients[2], values[2]],
                            [&self.coefficients[3], values[3]],
                            [&self.constant, &one],
                        ],
                    ));
                }
                _ => {}
            }
        }

        match self.coefficients.len() {
            0 => Ok(self.constant.clone()),
            1 => {
                let one = Real::one();
                Ok(Real::signed_product_sum(
                    [true, true],
                    [[&self.coefficients[0], values[0]], [&self.constant, &one]],
                ))
            }
            2 => {
                let one = Real::one();
                Ok(Real::signed_product_sum(
                    [true, true, true],
                    [
                        [&self.coefficients[0], values[0]],
                        [&self.coefficients[1], values[1]],
                        [&self.constant, &one],
                    ],
                ))
            }
            3 => {
                let one = Real::one();
                Ok(Real::signed_product_sum(
                    [true, true, true, true],
                    [
                        [&self.coefficients[0], values[0]],
                        [&self.coefficients[1], values[1]],
                        [&self.coefficients[2], values[2]],
                        [&self.constant, &one],
                    ],
                ))
            }
            4 => {
                let one = Real::one();
                Ok(Real::signed_product_sum(
                    [true, true, true, true, true],
                    [
                        [&self.coefficients[0], values[0]],
                        [&self.coefficients[1], values[1]],
                        [&self.coefficients[2], values[2]],
                        [&self.coefficients[3], values[3]],
                        [&self.constant, &one],
                    ],
                ))
            }
            _ => self.eval_fallback(variables, bindings),
        }
    }

    fn eval_fallback(
        &self,
        variables: &[Variable],
        bindings: &HashMap<SymbolId, Real>,
    ) -> Result<Real, ExprEvalError> {
        let mut value = self.constant.clone();
        for (coefficient, variable) in self.coefficients.iter().zip(variables) {
            let binding = bindings.get(&variable.symbol).ok_or_else(|| {
                ExprEvalError::UnboundSymbol(SymbolRef::new(
                    variable.symbol,
                    Some(variable.name.clone()),
                ))
            })?;
            value = value + coefficient.clone() * binding.clone();
        }
        Ok(value)
    }
}

/// Try to prepare an affine residual row for the given problem variables.
pub fn prepare_affine_residual(
    expression: &Expr,
    problem: &Problem,
) -> Option<PreparedAffineResidual> {
    let mut builder = AffineBuilder::new(problem);
    builder.collect(expression, Real::one())?;
    Some(builder.finish())
}

struct AffineBuilder<'a> {
    problem: &'a Problem,
    coefficients: Vec<Real>,
    constant: Real,
}

impl<'a> AffineBuilder<'a> {
    fn new(problem: &'a Problem) -> Self {
        Self {
            problem,
            coefficients: vec![Real::zero(); problem.variables.len()],
            constant: Real::zero(),
        }
    }

    fn collect(&mut self, expression: &Expr, scale: Real) -> Option<()> {
        match expression {
            Expr::Constant(value) => {
                self.constant = self.constant.clone() + scale * value.clone();
                Some(())
            }
            Expr::Symbol(symbol) => {
                let column = self
                    .problem
                    .variables
                    .iter()
                    .position(|variable| variable.symbol == symbol.id)?;
                self.coefficients[column] = self.coefficients[column].clone() + scale;
                Some(())
            }
            Expr::Add(left, right) => {
                self.collect(left, scale.clone())?;
                self.collect(right, scale)
            }
            Expr::Sub(left, right) => {
                self.collect(left, scale.clone())?;
                self.collect(right, -scale)
            }
            Expr::Neg(value) => self.collect(value, -scale),
            Expr::Mul(left, right) => {
                if let Some(constant) = constant_value(left) {
                    self.collect(right, scale * constant)
                } else if let Some(constant) = constant_value(right) {
                    self.collect(left, scale * constant)
                } else {
                    None
                }
            }
            Expr::Div(left, right) => {
                let denominator = constant_value(right)?;
                let reciprocal = (Real::one() / denominator).ok()?;
                self.collect(left, scale * reciprocal)
            }
            Expr::PowI(value, exponent) if *exponent == 0 => {
                self.constant = self.constant.clone() + scale;
                let _ = value;
                Some(())
            }
            Expr::PowI(value, exponent) if *exponent == 1 => self.collect(value, scale),
            Expr::PowI(_, _) | Expr::Sqrt(_) | Expr::Sin(_) | Expr::Cos(_) | Expr::Log10(_) => None,
        }
    }

    fn finish(self) -> PreparedAffineResidual {
        let mut values = self.coefficients.iter().collect::<Vec<_>>();
        values.push(&self.constant);
        let coefficient_exact = Real::exact_set_facts(values);
        let nonzero_coefficient_count = self
            .coefficients
            .iter()
            .filter(|coefficient| {
                !matches!(
                    coefficient.structural_facts().zero,
                    hyperreal::ZeroKnowledge::Zero
                )
            })
            .count();
        PreparedAffineResidual {
            constant: self.constant,
            coefficients: self.coefficients,
            coefficient_exact,
            nonzero_coefficient_count,
        }
    }
}

fn constant_value(expression: &Expr) -> Option<Real> {
    let facts = expression.structural_facts();
    if !facts.dependencies.is_empty() {
        return None;
    }
    expression.eval_real(&HashMap::new()).ok()
}
