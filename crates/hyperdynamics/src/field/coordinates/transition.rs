//! Direct endpoint transport between validated state charts.
use super::*;
use hypersolve::expression_program::{CoordinateTransition, CoordinateTransitionError};

/// Incompatible field semantics or original transition/materialization diagnostic.
#[derive(Clone, Debug)]
pub enum CoordinateTransportError {
    /// The charts assign different time or ordered state meaning to their source.
    InterpretationMismatch,
    /// Original exact source/transition construction failure.
    Transition(CoordinateTransitionError),
    /// Original graph, scalar or interval carrier failure.
    Field(FieldError),
}
impl fmt::Display for CoordinateTransportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "coordinate transport: {self:?}")
    }
}
impl std::error::Error for CoordinateTransportError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Transition(e) => Some(e),
            Self::Field(e) => Some(e),
            _ => None,
        }
    }
}
impl From<ExpressionError> for CoordinateTransportError {
    fn from(e: ExpressionError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalLatticeError> for CoordinateTransportError {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Field(e.into())
    }
}

/// Shared exact chart relation with the source's original time/state meaning.
#[derive(Clone, Debug)]
pub struct CoordinateFieldTransition<const N: usize> {
    from: CoordinateField<N>,
    to: CoordinateField<N>,
    relation: CoordinateTransition,
}
impl<const N: usize> CoordinateField<N> {
    /// Prepare direct transport into an independently validated chart of this field.
    ///
    /// This composes exact matrices before projecting intervals. It does not
    /// select a frame, perform QR factorization or assert a width improvement.
    /// Source and interpretation mismatches fail before any enclosure, even for
    /// an eventual empty input. Both original chart proofs are retained.
    pub fn transition_to(
        &self,
        target: &Self,
        budget: &mut EnclosureBudget,
    ) -> Result<CoordinateFieldTransition<N>, CoordinateTransportError> {
        budget
            .graph
            .charge(N.checked_add(1).ok_or(ExpressionError::SizeExhausted)?)?;
        let from = self.original_field().identity();
        let to = target.original_field().identity();
        if from.time() != to.time() || from.state() != to.state() {
            return Err(CoordinateTransportError::InterpretationMismatch);
        }
        let relation = self
            .transformation()
            .transition_to(target.transformation(), &mut budget.graph)
            .map_err(CoordinateTransportError::Transition)?;
        budget.graph.charge(1)?;
        let result = CoordinateFieldTransition {
            from: self.clone(),
            to: target.clone(),
            relation,
        };
        budget.graph.charge(0)?;
        Ok(result)
    }
}
impl<const N: usize> CoordinateFieldTransition<N> {
    /// The originating field chart.
    pub fn from(&self) -> &CoordinateField<N> {
        &self.from
    }
    /// The destination field chart; its operations consume `enclose` output.
    pub fn to(&self) -> &CoordinateField<N> {
        &self.to
    }
    /// Retained exact affine map and both original inverse proofs.
    pub fn relation(&self) -> &CoordinateTransition {
        &self.relation
    }
    /// Enclose the exact affine image of an owned native box in the next chart.
    ///
    /// Input and relation remain authoritative. The returned box is an explicit
    /// outer projection, generally not equal to that correlated image. It is a
    /// sound enlarged initial set for subsequent operations in `to()`. Empty and
    /// unit sets remain distinct. No ODE existence claim is made by transport.
    ///
    /// The shared budget covers graph evaluation, coefficient enclosure and
    /// publication; its scalar-payload, refinement and total-heap exclusions
    /// remain. A failed final check returns no completed transport certificate.
    pub fn enclose(
        &self,
        initial: BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<CoordinateTransport<N>, CoordinateTransportError> {
        budget.graph.charge(0)?;
        budget.graph.check_entries(&[N, N])?;
        let enclosure = if let Some(lanes) = initial.lanes() {
            let values = self.relation.program().eval_dyadic(
                lanes,
                budget.policy.precision,
                &mut budget.graph,
                &mut budget.arithmetic,
            )?;
            budget.graph.charge(N)?;
            BoxN::new(values.try_into().expect("validated transition dimension"))?
        } else {
            BoxN::empty()?
        };
        budget.graph.charge(1)?;
        let result = CoordinateTransport {
            transition: self.clone(),
            initial,
            enclosure,
            policy: budget.policy(),
        };
        budget.graph.charge(0)?;
        Ok(result)
    }
}
/// Completed direct chart transport, retaining the exact image's definition.
///
/// Reconstruct the original field and both charts and call `enclose` on the
/// original input. No serialized relation, enclosure or flag is accepted as proof.
#[derive(Clone, Debug)]
pub struct CoordinateTransport<const N: usize> {
    transition: CoordinateFieldTransition<N>,
    initial: BoxN<DyadicInterval, N>,
    enclosure: BoxN<DyadicInterval, N>,
    policy: EnclosurePolicy,
}
impl<const N: usize> CoordinateTransport<N> {
    /// Original chart association and exact transition coefficients.
    pub fn transition(&self) -> &CoordinateFieldTransition<N> {
        &self.transition
    }
    /// Exact Cartesian parameter set in the originating chart.
    pub fn initial(&self) -> &BoxN<DyadicInterval, N> {
        &self.initial
    }
    /// Certified Cartesian outer enclosure in the destination chart.
    pub fn enclosure(&self) -> &BoxN<DyadicInterval, N> {
        &self.enclosure
    }
    /// Enclosure policy used for this complete transport.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
}
