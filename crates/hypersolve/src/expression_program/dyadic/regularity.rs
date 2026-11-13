//! Sufficient source-domain regularity, separate from formula evaluability.
use super::{
    DyadicArithmetic, DyadicBudget, DyadicInterval, ExpressionBudget, ExpressionError,
    ExpressionNode, IntervalError, UnaryOperation,
};
use crate::expression_program::JacobianProgram;

/// Source-bound continuous image and continuously differentiable selected partials.
///
/// Original outputs are jointly continuous on the retained argument product.
/// For fixed other arguments they are C1 in the selected variables on a
/// neighborhood of their box, with partials enclosed by the generated program.
/// Primitive boundary checks are sufficient, not necessary: removable composite
/// singularities can remain unproved. Empty products make these claims vacuous.
/// No deserialization of this evidence is provided; replay source, variables
/// and arguments through the validated constructors and `enclose_regular`.
#[derive(Clone, Debug)]
pub struct RegularJacobianEnclosure {
    program: JacobianProgram,
    arguments: Vec<DyadicInterval>,
    precision: i32,
    values: Vec<DyadicInterval>,
}
impl RegularJacobianEnclosure {
    /// Exact source and generated derivative association.
    pub fn program(&self) -> &JacobianProgram {
        &self.program
    }
    /// Cartesian domain, in original binding order.
    pub fn arguments(&self) -> &[DyadicInterval] {
        &self.arguments
    }
    /// Requested enclosure grid; not a final-width bound.
    pub fn precision(&self) -> i32 {
        self.precision
    }
    /// Original outputs followed by column-major partials.
    pub fn values(&self) -> &[DyadicInterval] {
        &self.values
    }
    /// Consume the association and return its ordinary numerical enclosures.
    pub fn into_values(self) -> Vec<DyadicInterval> {
        self.values
    }
}
impl JacobianProgram {
    /// Prove source continuity and C1 regularity in the differentiation variables.
    ///
    /// Reached source operations must cover their full operand enclosures.
    /// State-dependent sqrt/asin/acos/acosh operands must additionally lie in
    /// their open smooth domains. Inactive-variable-only operands may touch a
    /// continuous boundary, e.g. sqrt(time) at time zero when differentiating
    /// only state. Division, logarithm and integer powers retain their existing
    /// full-domain guards. Generated partials are then enclosed as usual.
    ///
    /// Arguments and source association are retained immutably. Native interval
    /// copies and guard comparisons share the supplied budgets. Existing scalar
    /// refinement, payload/ingress and total-heap exclusions remain in force.
    pub fn enclose_regular(
        &self,
        values: &[DyadicInterval],
        precision: i32,
        graph: &mut ExpressionBudget,
        arithmetic: &mut DyadicBudget,
    ) -> Result<RegularJacobianEnclosure, ExpressionError> {
        graph.charge(0)?;
        let original = self.source().definition();
        let combined = self.program().definition();
        if values.len() != original.variables.len() {
            return Err(ExpressionError::BindingCount {
                expected: original.variables.len(),
                actual: values.len(),
            });
        }
        graph.check_entries(&[
            combined.variables.len(),
            combined.nodes.len(),
            combined.outputs.len(),
            original.nodes.len(),
            values.len(),
        ])?;
        let mut arguments = Vec::new();
        for (slot, value) in values.iter().enumerate() {
            graph.charge(1)?;
            arguments.push(value.checked_pow(1, arithmetic).map_err(|source| {
                // This is an argument copy, before a source node is selected.
                ExpressionError::ArgumentEnclosure { slot, source }
            })?);
        }
        graph.charge(values.len())?;
        let output = if values.iter().any(DyadicInterval::is_empty) {
            self.program()
                .eval_dyadic(values, precision, graph, arithmetic)?
        } else {
            let mut active = Vec::new();
            for node in &original.nodes {
                graph.charge(1)?;
                let depends = match node {
                    ExpressionNode::Constant(_) => false,
                    ExpressionNode::Variable(reference) => {
                        graph.charge(self.variables().len())?;
                        self.variables().contains(&reference.id)
                    }
                    ExpressionNode::Unary { input, .. } => active[*input],
                    ExpressionNode::Binary { left, right, .. } => active[*left] || active[*right],
                };
                active.push(depends);
            }
            self.program().evaluate_dyadic(
                values,
                graph,
                &mut DyadicArithmetic {
                    precision,
                    budget: arithmetic,
                },
                Some(&active),
            )?
        };
        graph.charge(1)?;
        let result = RegularJacobianEnclosure {
            program: self.clone(),
            arguments,
            precision,
            values: output,
        };
        graph.charge(0)?;
        Ok(result)
    }
}
impl DyadicArithmetic<'_> {
    pub(super) fn regular_input(
        &mut self,
        operation: UnaryOperation,
        input: &DyadicInterval,
    ) -> Result<bool, IntervalError> {
        match operation {
            UnaryOperation::Sqrt => Ok(input.signs() == hyperinterval::SignSet::Positive),
            UnaryOperation::Asin | UnaryOperation::Acos => {
                let open = DyadicInterval::new((-1).into(), 1.into(), 0)?;
                input.interior_subset_of(&open, self.budget)
            }
            UnaryOperation::Acosh => {
                let one = DyadicInterval::point(1.into(), 0)?;
                one.strictly_left_of(input, self.budget)
            }
            _ => Ok(true),
        }
    }
}
