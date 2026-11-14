//! Strict partial-function composition with retained inner output guards.
use super::{
    ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionNode, ExpressionProgram,
};
use std::{fmt, sync::Arc};

/// Invalid composition bindings or an original graph/resource diagnostic.
#[derive(Clone, Debug)]
pub enum CompositionError {
    /// There must be one selected inner output per outer binding slot.
    BindingCount { expected: usize, actual: usize },
    /// A binding selects an output outside the inner program's output table.
    InvalidInput { binding: usize, output: usize },
    /// Original validation, resource or cooperative cancellation failure.
    Expression(ExpressionError),
}
impl fmt::Display for CompositionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "program composition: {self:?}")
    }
}
impl std::error::Error for CompositionError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Expression(error) => Some(error),
            _ => None,
        }
    }
}
impl From<ExpressionError> for CompositionError {
    fn from(error: ExpressionError) -> Self {
        Self::Expression(error)
    }
}

/// Immutable association of two sources and their selected input bindings.
///
/// For outer `f`, inner `g` and selection `s`, the executable outputs are
/// `(g(x), f(g(x)[s]))`. The inner prefix is mandatory: strict partial-function
/// composition preserves the domain of every inner output, even if the outer
/// program ignores it or has no outputs. Original unreachable nodes remain
/// validated data and are not forced into evaluation.
///
/// Use [`Self::composed_output`] to locate a composed value after evaluating or
/// certifying the complete program. Selecting only its final roots into a new
/// definition discards this domain contract. Existing Jacobian and Taylor
/// consumers retain the prefix; Taylor output projection can select the final
/// values while retaining the complete source proof. Their existing regularity
/// restrictions apply to all retained outputs, including the inner prefix.
///
/// Replay both original definitions and [`Self::bindings`] through
/// [`ExpressionProgram::compose`]. Generated nodes, structural equality or
/// serialized validity flags alone do not prove the composition association.
#[derive(Clone, Debug)]
pub struct CompositionProgram {
    outer: ExpressionProgram,
    inner: ExpressionProgram,
    bindings: Arc<[usize]>,
    program: ExpressionProgram,
    outer_mapping: Arc<[usize]>,
}
impl CompositionProgram {
    /// Original outer source; its identifiers are scoped independently of the inner source.
    pub fn outer(&self) -> &ExpressionProgram {
        &self.outer
    }
    /// Original inner source, including its complete ordered output contract.
    pub fn inner(&self) -> &ExpressionProgram {
        &self.inner
    }
    /// Inner output indices in outer binding-slot order; repetition is permitted.
    pub fn bindings(&self) -> &[usize] {
        &self.bindings
    }
    /// Guard-preserving executable: every inner output, then every composed output.
    /// Its argument order is exactly the inner source's binding order.
    pub fn program(&self) -> &ExpressionProgram {
        &self.program
    }
    /// Position of one outer result in the complete executable's output vector.
    pub fn composed_output(&self, output: usize) -> Option<usize> {
        (output < self.outer.definition().outputs.len())
            .then(|| self.inner.definition().outputs.len() + output)
    }
    /// Generated node implementing one outer node, including selected variables.
    /// Inner nodes retain their original indices. This map also locates an outer
    /// operation's scalar/domain diagnostic in the generated program.
    pub fn outer_node(&self, node: usize) -> Option<usize> {
        self.outer_mapping.get(node).copied()
    }
}

impl ExpressionProgram {
    /// Compose this outer program with selected outputs of an inner program.
    ///
    /// `bindings[i]` selects the inner output substituted for outer binding slot
    /// `i`; identifiers in the two programs need not match and never capture one
    /// another. All variable occurrences selecting the same inner output share
    /// its existing node. This builds a linear-size flat DAG without reconstructing
    /// expression trees, changing arithmetic or simplifying away source guards.
    /// The returned program retains all inner outputs before the outer results.
    ///
    /// One caller-owned budget covers source visits/copies, binding and node maps,
    /// validation and publication. Entry checks include the new definition and
    /// composition mappings at construction stages, not total retained input
    /// storage, scalar payloads or heap bytes. Scalar cloning/destruction retain
    /// `ExpressionBudget`'s exclusions. Cancellation is checked between charges
    /// and before publication. Failure mutates neither source and retains no cache.
    #[inline]
    pub fn compose(
        &self,
        inner: &ExpressionProgram,
        bindings: &[usize],
        budget: &mut ExpressionBudget,
    ) -> Result<CompositionProgram, CompositionError> {
        budget.charge(0)?;
        let outer = self.definition();
        let input = inner.definition();
        if bindings.len() != outer.variables.len() {
            return Err(CompositionError::BindingCount {
                expected: outer.variables.len(),
                actual: bindings.len(),
            });
        }
        for (binding, &output) in bindings.iter().enumerate() {
            budget.charge(1)?;
            if output >= input.outputs.len() {
                return Err(CompositionError::InvalidInput { binding, output });
            }
        }
        budget.charge(outer.nodes.len())?;
        let nonvariables = self.slots.iter().filter(|slot| slot.is_none()).count();
        let nodes = input
            .nodes
            .len()
            .checked_add(nonvariables)
            .ok_or(ExpressionError::SizeExhausted)?;
        let outputs = input
            .outputs
            .len()
            .checked_add(outer.outputs.len())
            .ok_or(ExpressionError::SizeExhausted)?;
        budget.check_entries(&[
            input.variables.len(),
            nodes,
            outputs,
            bindings.len(),
            outer.nodes.len(),
        ])?;
        budget.charge(input.variables.len())?;
        budget.charge(input.outputs.len())?;
        let mut definition = ExpressionDefinition {
            variables: input.variables.clone(),
            nodes: Vec::new(),
            outputs: input.outputs.clone(),
        };
        for node in &input.nodes {
            budget.charge(1)?;
            definition.nodes.push(node.clone());
        }
        let mut mapping = Vec::new();
        for (index, node) in outer.nodes.iter().enumerate() {
            budget.charge(2)?;
            if let Some(slot) = self.slots[index] {
                mapping.push(input.outputs[bindings[slot]]);
                continue;
            }
            budget.charge(1)?;
            let mapped = match node {
                ExpressionNode::Constant(_) => node.clone(),
                ExpressionNode::Variable(_) => unreachable!("validated variable binding"),
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
            definition.nodes.push(mapped);
        }
        for &output in &outer.outputs {
            budget.charge(1)?;
            definition.outputs.push(mapping[output]);
        }
        let program = Self::from_definition(definition, budget)?;
        budget.charge(mapping.len())?;
        budget.charge(bindings.len())?;
        budget.charge(3)?;
        let result = CompositionProgram {
            outer: self.clone(),
            inner: inner.clone(),
            bindings: bindings.into(),
            program,
            outer_mapping: mapping.into(),
        };
        budget.charge(0)?;
        Ok(result)
    }
}
