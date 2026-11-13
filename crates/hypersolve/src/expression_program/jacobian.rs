//! Source-bound differentiation through bounded local Expr templates.
use std::{collections::BTreeSet, sync::Arc};

use hyperreal::Real;

use super::{
    BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
    ExpressionProgram, UnaryOperation,
};
use crate::{Expr, SymbolId};

// One local derivative rule, expanded into the original visitation loop or
// the demanded-partial traversal. This keeps operation and temporary order
// while avoiding another symbolic differentiation authority. Keep unary and
// binary checked indexing at distinct caller sites, as in the original loop:
// merging their panic locations also moves the downstream global address table.
macro_rules! derive_node {
    ($builder:ident, $node:expr, $variable:expr, $unary_input:expr, $binary_input:expr, $zero:ident, $one:ident) => {
        match $node {
            $crate::expression_program::ExpressionNode::Constant(_) => $zero,
            $crate::expression_program::ExpressionNode::Variable(reference) => {
                if reference.id == $variable {
                    $one
                } else {
                    $zero
                }
            }
            $crate::expression_program::ExpressionNode::Unary { operation, input } => {
                let derivative = ($unary_input)(*input);
                if derivative == $zero {
                    $zero
                } else {
                    let template = $crate::expression_program::jacobian::unary(
                        *operation,
                        $crate::Expr::anonymous_symbol($crate::SymbolId(0)),
                    );
                    let partial = $builder.partial(template, 0, &[*input], $zero, $one)?;
                    $builder.chain(partial, derivative, $zero, $one)?
                }
            }
            $crate::expression_program::ExpressionNode::Binary {
                operation,
                left,
                right,
            } => {
                let mut terms = [$zero; 2];
                for (column, input) in [*left, *right].into_iter().enumerate() {
                    let derivative = ($binary_input)(input);
                    if derivative != $zero {
                        let template = $crate::expression_program::jacobian::binary(
                            *operation,
                            $crate::Expr::anonymous_symbol($crate::SymbolId(0)),
                            $crate::Expr::anonymous_symbol($crate::SymbolId(1)),
                        );
                        let partial = $builder.partial(
                            template,
                            column as u32,
                            &[*left, *right],
                            $zero,
                            $one,
                        )?;
                        terms[column] = $builder.chain(partial, derivative, $zero, $one)?;
                    }
                }
                match terms {
                    [a, b] if a == $zero => b,
                    [a, b] if b == $zero => a,
                    [left, right] => {
                        $builder.push($crate::expression_program::ExpressionNode::Binary {
                            operation: $crate::expression_program::BinaryOperation::Add,
                            left,
                            right,
                        })?
                    }
                }
            }
        }
    };
}
#[cfg(feature = "taylor-model")]
pub(super) use derive_node;

/// An owned source and its generated symbolic partial derivatives.
///
/// The executable program first evaluates every original output, then partials
/// in column-major order. Keeping these source outputs preserves domain guards
/// even when a derivative is identically zero. A derivative formula can have a
/// smaller domain than a differentiable composite (for example at a removable
/// chain-rule singularity); failure to evaluate it is not proof of
/// nondifferentiability. Construction alone makes no smoothness or trajectory
/// certificate. The `interval-program` feature supplies `enclose_regular` for
/// a separate sufficient source-domain regularity proof.
///
/// Identity and replay consist of [`Self::source`]'s complete definition and the
/// ordered [`Self::variables`]. Replay validates the source and calls
/// [`ExpressionProgram::jacobian`] again; serialized derivative nodes or flags
/// are not evidence of the association.
#[derive(Clone, Debug)]
pub struct JacobianProgram {
    source: ExpressionProgram,
    variables: Arc<[SymbolId]>,
    program: ExpressionProgram,
}

impl JacobianProgram {
    /// Original validated source, including its exact constants and bindings.
    pub fn source(&self) -> &ExpressionProgram {
        &self.source
    }

    /// Distinct differentiation variables, in column order.
    pub fn variables(&self) -> &[SymbolId] {
        &self.variables
    }

    /// Evaluate source outputs followed by all symbolic partial derivatives.
    ///
    /// Its ordinary point and interval evaluators retain their resource and
    /// domain contracts. Do not discard the original outputs' domain checks
    /// when using derivative values as evidence about the source.
    pub fn program(&self) -> &ExpressionProgram {
        &self.program
    }

    /// Index of the requested partial in the combined evaluation output.
    pub fn partial_output(&self, row: usize, column: usize) -> Option<usize> {
        let rows = self.source.definition().outputs.len();
        (row < rows && column < self.variables.len()).then(|| rows + column * rows + row)
    }
}

impl ExpressionProgram {
    /// Build source-bound symbolic partials with bounded graph expansion.
    ///
    /// Each reachable source node is visited once per requested variable. Small
    /// local templates use the existing `Expr::derivative` rules, then substitute
    /// original node references into a shared flat graph. No source expression
    /// tree is reconstructed. Generated zero derivatives elide only derivative
    /// work; the complete source is still evaluated first.
    ///
    /// Graph budget charges include source copies, variable validation, local
    /// rule expansions, template compilation/import and final validation.
    /// Template expansion is a fixed-size operation for one grammar node,
    /// including i64 exponent boundaries. Scalar cloning and arithmetic inside
    /// those templates remain outside total-work/heap accounting, as with
    /// `compile`. Cancellation is cooperative between charged stages. Work
    /// already spent is not refunded; failed attempts retain no derived cache.
    pub fn jacobian(
        &self,
        variables: &[SymbolId],
        budget: &mut ExpressionBudget,
    ) -> Result<JacobianProgram, ExpressionError> {
        budget.charge(0)?;
        let source = self.definition();
        let outputs = source
            .outputs
            .len()
            .checked_mul(variables.len())
            .and_then(|n| n.checked_add(source.outputs.len()))
            .ok_or(ExpressionError::SizeExhausted)?;
        budget.entries(&[source.variables.len(), source.nodes.len(), outputs])?;
        let mut seen = BTreeSet::new();
        for variable in variables {
            budget.charge(1)?;
            if !seen.insert(*variable) {
                return Err(ExpressionError::DuplicateVariable(*variable));
            }
            // Charge the complete binding scan, independent of early matches.
            budget.charge(source.variables.len())?;
            if !source.variables.contains(variable) {
                return Err(ExpressionError::UnboundVariable(*variable));
            }
        }
        budget.charge(variables.len())?;
        let variables: Arc<[SymbolId]> = variables.into();
        if variables.is_empty() || source.outputs.is_empty() {
            budget.charge(0)?;
            return Ok(JacobianProgram {
                source: self.clone(),
                variables,
                program: self.clone(),
            });
        }
        let uses = self.operand_uses(budget)?;
        budget.charge(source.variables.len() + source.outputs.len())?;
        let mut definition = ExpressionDefinition {
            variables: source.variables.clone(),
            nodes: Vec::new(),
            outputs: source.outputs.clone(),
        };
        for node in &source.nodes {
            budget.charge(1)?;
            definition.nodes.push(node.clone());
        }
        let mut builder = Builder {
            definition,
            outputs,
            budget,
        };
        let zero = builder.push(ExpressionNode::Constant(Real::zero()))?;
        let one = builder.push(ExpressionNode::Constant(Real::one()))?;
        for variable in variables.iter() {
            builder.budget.charge(source.nodes.len())?;
            let mut derivatives = vec![zero; source.nodes.len()];
            for (index, node) in source.nodes.iter().enumerate() {
                builder.budget.charge(1)?;
                if uses[index] == 0 {
                    continue;
                }
                derivatives[index] = derive_node!(
                    builder,
                    node,
                    *variable,
                    |input: usize| derivatives[input],
                    |input: usize| derivatives[input],
                    zero,
                    one
                );
            }
            for &root in &source.outputs {
                builder.budget.charge(1)?;
                builder.definition.outputs.push(derivatives[root]);
            }
        }
        let program = Self::from_definition(builder.definition, builder.budget)?;
        budget.charge(0)?;
        Ok(JacobianProgram {
            source: self.clone(),
            variables,
            program,
        })
    }
}

pub(super) struct Builder<'a> {
    pub(super) definition: ExpressionDefinition,
    pub(super) outputs: usize,
    pub(super) budget: &'a mut ExpressionBudget,
}
impl Builder<'_> {
    pub(super) fn push(&mut self, node: ExpressionNode) -> Result<usize, ExpressionError> {
        self.budget.entries(&[
            self.definition.variables.len(),
            self.definition.nodes.len(),
            1,
            self.outputs,
        ])?;
        self.budget.charge(1)?;
        let index = self.definition.nodes.len();
        self.definition.nodes.push(node);
        Ok(index)
    }
    pub(super) fn chain(
        &mut self,
        partial: usize,
        derivative: usize,
        zero: usize,
        one: usize,
    ) -> Result<usize, ExpressionError> {
        if partial == zero || derivative == zero {
            Ok(zero)
        } else if partial == one {
            Ok(derivative)
        } else if derivative == one {
            Ok(partial)
        } else {
            self.push(ExpressionNode::Binary {
                operation: BinaryOperation::Mul,
                left: partial,
                right: derivative,
            })
        }
    }
    pub(super) fn partial(
        &mut self,
        template: Expr,
        variable: u32,
        inputs: &[usize],
        zero: usize,
        one: usize,
    ) -> Result<usize, ExpressionError> {
        self.budget.charge(1)?;
        let derivative = template.derivative(SymbolId(variable));
        let template_variables: Vec<_> = (0..inputs.len()).map(|i| SymbolId(i as u32)).collect();
        let program = ExpressionProgram::compile(&[derivative], &template_variables, self.budget)?;
        let source = program.definition();
        let mut imported = Vec::new();
        for node in &source.nodes {
            self.budget.charge(1)?;
            let index = match node {
                ExpressionNode::Variable(reference) => inputs[reference.id.0 as usize],
                ExpressionNode::Constant(value) if value == &Real::zero() => zero,
                ExpressionNode::Constant(value) if value == &Real::one() => one,
                ExpressionNode::Constant(value) => {
                    self.push(ExpressionNode::Constant(value.clone()))?
                }
                ExpressionNode::Unary { operation, input } => self.push(ExpressionNode::Unary {
                    operation: *operation,
                    input: imported[*input],
                })?,
                ExpressionNode::Binary {
                    operation,
                    left,
                    right,
                } => self.push(ExpressionNode::Binary {
                    operation: *operation,
                    left: imported[*left],
                    right: imported[*right],
                })?,
            };
            imported.push(index);
        }
        Ok(imported[source.outputs[0]])
    }
}

pub(super) fn binary(operation: BinaryOperation, left: Expr, right: Expr) -> Expr {
    match operation {
        BinaryOperation::Add => left + right,
        BinaryOperation::Sub => left - right,
        BinaryOperation::Mul => left * right,
        BinaryOperation::Div => left / right,
    }
}
pub(super) fn unary(operation: UnaryOperation, value: Expr) -> Expr {
    match operation {
        UnaryOperation::Neg => -value,
        UnaryOperation::PowI(n) => value.powi(n),
        UnaryOperation::Sqrt => value.sqrt(),
        UnaryOperation::Sin => value.sin(),
        UnaryOperation::Cos => value.cos(),
        UnaryOperation::Ln => value.ln(),
        UnaryOperation::Log10 => value.log10(),
        UnaryOperation::Asin => value.asin(),
        UnaryOperation::Acos => value.acos(),
        UnaryOperation::Acosh => value.acosh(),
        UnaryOperation::Atanh => value.atanh(),
    }
}
