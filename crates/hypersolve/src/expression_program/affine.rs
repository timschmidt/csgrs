//! Source-bound affine parameter changes, retaining graph sharing and guards.
use super::{
    BinaryOperation, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode,
    ExpressionProgram,
};
use crate::SymbolId;
use hyperreal::Real;
use std::sync::Arc;

/// Exact affine parameter change for a vector-valued derivative program.
///
/// Replaces the selected input `t` by `offset + scale*tau` and multiplies each
/// output by `scale`: `g(tau,x) = scale*f(offset+scale*tau,x)`. This is an
/// algebraic program transformation, not an ODE or invertibility certificate.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct AffineParameterization {
    /// Binding identifier to reinterpret as the new parameter.
    pub variable: SymbolId,
    /// Exact input and output scale, shared by the generated graph.
    pub scale: Real,
    /// Exact offset of the selected input.
    pub offset: Real,
}

/// Immutable association between an original program and its parameter change.
/// Replay the original definition and specification; generated nodes or flags
/// alone do not establish the transformation's meaning.
#[derive(Clone, Debug)]
pub struct AffineProgram {
    source: ExpressionProgram,
    parameterization: Arc<AffineParameterization>,
    program: ExpressionProgram,
    mapping: Arc<[usize]>,
}
impl AffineProgram {
    /// Original complete source, including unused nodes and variable metadata.
    pub fn source(&self) -> &ExpressionProgram {
        &self.source
    }
    /// Exact input/output transformation.
    pub fn parameterization(&self) -> &AffineParameterization {
        &self.parameterization
    }
    /// Validated transformed graph, in the original binding and output order.
    pub fn program(&self) -> &ExpressionProgram {
        &self.program
    }
    /// Transformed node implementing an original node, before final output scaling.
    /// Repeated references to the selected variable share one affine expression.
    pub fn transformed_node(&self, source_node: usize) -> Option<usize> {
        self.mapping.get(source_node).copied()
    }
}

impl ExpressionProgram {
    /// Change a parameter and scale vector outputs through an owned shared DAG.
    ///
    /// Original source guards remain reachable even for zero output scale. All
    /// selected-variable nodes share one affine expression, and all outputs share
    /// its scale constant. No expression tree is reconstructed or simplified.
    /// Work and entry checks cover linear graph construction, source mapping,
    /// validation and publication. Existing scalar payload/ingress and total-heap
    /// exclusions remain; cancellation is checked at charged stages.
    pub fn reparameterize(
        &self,
        parameterization: AffineParameterization,
        budget: &mut ExpressionBudget,
    ) -> Result<AffineProgram, ExpressionError> {
        budget.charge(0)?;
        let source = self.definition();
        budget.charge(source.variables.len())?;
        if !source.variables.contains(&parameterization.variable) {
            return Err(ExpressionError::UnboundVariable(parameterization.variable));
        }
        let nodes = source
            .nodes
            .len()
            .checked_add(4)
            .and_then(|n| n.checked_add(source.outputs.len()))
            .ok_or(ExpressionError::SizeExhausted)?;
        budget.check_entries(&[
            source.variables.len(),
            nodes,
            source.outputs.len(),
            source.nodes.len(),
        ])?;
        budget.charge(
            source
                .variables
                .len()
                .checked_add(2)
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        let mut definition = ExpressionDefinition {
            variables: source.variables.clone(),
            nodes: vec![
                ExpressionNode::Constant(parameterization.offset.clone()),
                ExpressionNode::Constant(parameterization.scale.clone()),
            ],
            outputs: Vec::new(),
        };
        let mut mapping = Vec::new();
        let mut parameter = None;
        for node in &source.nodes {
            budget.charge(2)?;
            if let ExpressionNode::Variable(reference) = node
                && reference.id == parameterization.variable
            {
                let index = match parameter {
                    Some(index) => index,
                    None => {
                        budget.charge(3)?;
                        let variable = definition.nodes.len();
                        definition.nodes.push(node.clone());
                        definition.nodes.push(ExpressionNode::Binary {
                            operation: BinaryOperation::Mul,
                            left: 1,
                            right: variable,
                        });
                        definition.nodes.push(ExpressionNode::Binary {
                            operation: BinaryOperation::Add,
                            left: 0,
                            right: variable + 1,
                        });
                        let index = variable + 2;
                        parameter = Some(index);
                        index
                    }
                };
                mapping.push(index);
                continue;
            }
            let transformed = match node {
                ExpressionNode::Constant(_) | ExpressionNode::Variable(_) => node.clone(),
                ExpressionNode::Unary { operation, input } => ExpressionNode::Unary {
                    operation: *operation,
                    input: mapping[*input],
                },
                ExpressionNode::Binary {
                    operation,
                    left,
                    right,
                } => ExpressionNode::Binary {
                    operation: *operation,
                    left: mapping[*left],
                    right: mapping[*right],
                },
            };
            mapping.push(definition.nodes.len());
            definition.nodes.push(transformed);
        }
        for &root in &source.outputs {
            budget.charge(2)?;
            definition.outputs.push(definition.nodes.len());
            definition.nodes.push(ExpressionNode::Binary {
                operation: BinaryOperation::Mul,
                left: 1,
                right: mapping[root],
            });
        }
        let program = Self::from_definition(definition, budget)?;
        budget.charge(
            mapping
                .len()
                .checked_add(2)
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        let result = AffineProgram {
            source: self.clone(),
            parameterization: Arc::new(parameterization),
            program,
            mapping: mapping.into(),
        };
        budget.charge(0)?;
        Ok(result)
    }
}
