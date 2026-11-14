//! Reuse prepared normalized Lie coefficients at a lower polynomial order.
use super::*;

/// Invalid order restriction or original structural-resource diagnostic.
#[derive(Clone, Debug)]
pub enum FlowTaylorOrderError {
    /// Restriction cannot generate coefficients beyond the supplied program.
    UnpreparedOrder {
        /// Requested polynomial order, excluding its mandatory remainder stage.
        requested: usize,
        /// Highest polynomial order admitted by the supplied program.
        prepared: usize,
    },
    /// Original graph-entry, cumulative-work or cancellation failure.
    Resource(ExpressionError),
}
impl fmt::Display for FlowTaylorOrderError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "trajectory Taylor order restriction: {self:?}")
    }
}
impl std::error::Error for FlowTaylorOrderError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Resource(error) => Some(error),
            Self::UnpreparedOrder { .. } => None,
        }
    }
}
impl From<ExpressionError> for FlowTaylorOrderError {
    fn from(error: ExpressionError) -> Self {
        Self::Resource(error)
    }
}
impl<const N: usize> FlowTaylorProgram<N> {
    /// Reuse this program at any prepared polynomial order, including zero.
    ///
    /// The result retains stages `1..=order+1`; the last is the complete original
    /// normalized Lie coefficient used for the remainder. Original source guards,
    /// time/state bindings and coefficient associations remain shared. This is
    /// preparation only: use `certify_step` or `integrate` to prove trajectories.
    /// No coefficient is differentiated or evaluated during restriction.
    ///
    /// Lower orders copy `order+1` stage handles and their fixed-size output indices,
    /// while sharing the exact expression DAGs and derivative requests through Arc.
    /// Equal order shares the entire stage vector. Structural entries and copy work
    /// spend the supplied budget; scalar work is unchanged. Shared input graphs,
    /// scalar payloads and total heap bytes are not bounded by these counters.
    /// Failure does not mutate this program or cache an unsuccessful attempt.
    /// Replay the original field definition, prepared order and requested order;
    /// no serialized derived proof or cached flag is accepted as authority.
    pub fn restrict_order(
        &self,
        order: usize,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, FlowTaylorOrderError> {
        budget.graph.charge(0)?;
        if order > self.order {
            return Err(FlowTaylorOrderError::UnpreparedOrder {
                requested: order,
                prepared: self.order,
            });
        }
        budget.graph.charge(1)?;
        if order == self.order {
            let result = self.clone();
            budget.graph.charge(0)?;
            return Ok(result);
        }
        let count = order.checked_add(1).ok_or(ExpressionError::SizeExhausted)?;
        let outputs = count.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
        budget.graph.check_entries(&[count, outputs])?;
        let copy_work = N.checked_add(8).ok_or(ExpressionError::SizeExhausted)?;
        let mut stages = Vec::with_capacity(count);
        for stage in &self.stages[..count] {
            budget.graph.charge(copy_work)?;
            stages.push(stage.clone());
        }
        budget.graph.charge(count)?;
        let result = Self {
            field: self.field.clone(),
            order,
            stages: stages.into(),
        };
        budget.graph.charge(0)?;
        Ok(result)
    }
}
