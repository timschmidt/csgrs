//! Shared, configuration-preserving graph accounting for domain-owned adapters.
use super::{
    EnclosureBudget, ExpressionBudget, ExpressionDefinition, ExpressionError, ExpressionProgram,
    SymbolId,
};
use hypersolve::Expr;

/// Borrowed graph operations using an enclosure budget's existing counter.
///
/// This view permits charges and source construction without exposing a mutable
/// counter that could be replaced with different limits or reset work. Native
/// arithmetic remains in the enclosing budget. Existing graph/scalar resource
/// exclusions are unchanged; this is not a total heap or scalar-work bound.
pub struct EnclosureGraphBudget<'a> {
    graph: &'a mut ExpressionBudget,
}
impl EnclosureGraphBudget<'_> {
    /// Charge adapter work against the existing counter, checking cancellation.
    pub fn charge(&mut self, amount: usize) -> Result<(), ExpressionError> {
        self.graph.charge(amount)
    }
    /// Check aggregate structural entries against the existing configured limit.
    pub fn check_entries(&self, counts: &[usize]) -> Result<(), ExpressionError> {
        self.graph.check_entries(counts)
    }
    /// Validate an owned flat source with the same graph limits and work counter.
    pub fn validate(
        &mut self,
        definition: ExpressionDefinition,
    ) -> Result<ExpressionProgram, ExpressionError> {
        ExpressionProgram::from_definition(definition, self.graph)
    }
    /// Compile source trees with the same graph limits and work counter.
    pub fn compile(
        &mut self,
        outputs: &[Expr],
        variables: &[SymbolId],
    ) -> Result<ExpressionProgram, ExpressionError> {
        ExpressionProgram::compile(outputs, variables, self.graph)
    }
}
impl EnclosureBudget {
    /// Borrow bounded graph operations for a domain adapter without resetting or
    /// exposing mutable configuration. Nested construction consumes this budget's
    /// existing structural work; it does not start an independent allowance.
    pub fn graph_budget(&mut self) -> EnclosureGraphBudget<'_> {
        EnclosureGraphBudget {
            graph: &mut self.graph,
        }
    }
}
