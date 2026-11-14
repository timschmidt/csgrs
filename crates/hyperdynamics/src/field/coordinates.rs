//! Exact state charts consumed by the existing field and Picard owners.
mod evolution;
pub use evolution::{CoordinateEvolution, CoordinateEvolutionStatus, CoordinateSegment};

use super::{
    picard::{PicardFailure, PicardProposal, PicardReport, PicardStatus},
    *,
};
use hypersolve::expression_program::{CoordinateChange, CoordinateError, CoordinateProgram};

/// Replay data; source, inverse proof and derivatives are revalidated on load.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct CoordinateFieldDefinition {
    /// Original field and its time/state interpretation.
    pub source: FieldDefinition,
    /// Exact state chart in the source field's state order.
    pub change: CoordinateChange,
}
/// Invalid state interpretation or an original construction/materialization error.
#[derive(Clone, Debug)]
pub enum CoordinateFieldError {
    /// The chart must name exactly the field's ordered state variables, excluding time.
    StateOrder,
    /// Original exact chart, inverse or graph diagnostic.
    Coordinate(CoordinateError),
    /// Original field, native interval or carrier diagnostic.
    Field(FieldError),
}
impl fmt::Display for CoordinateFieldError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "coordinate field: {self:?}")
    }
}
impl std::error::Error for CoordinateFieldError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Coordinate(e) => Some(e),
            Self::Field(e) => Some(e),
            _ => None,
        }
    }
}
impl From<CoordinateError> for CoordinateFieldError {
    fn from(e: CoordinateError) -> Self {
        Self::Coordinate(e)
    }
}
impl From<FieldError> for CoordinateFieldError {
    fn from(e: FieldError) -> Self {
        Self::Field(e)
    }
}
impl From<ExpressionError> for CoordinateFieldError {
    fn from(e: ExpressionError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalError> for CoordinateFieldError {
    fn from(e: IntervalError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalLatticeError> for CoordinateFieldError {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Field(e.into())
    }
}
#[derive(Debug)]
struct CoordinateData<const N: usize> {
    original: ExpressionField<N>,
    transformation: CoordinateProgram,
    field: ExpressionField<N>,
}
/// Shared original field, exact state chart and transformed derivative field.
///
/// A box of new coordinates represents the exact correlated original set c+A*Y.
/// `source_enclosure` explicitly projects it to a Cartesian outer enclosure;
/// that box never replaces the authoritative chart in this object or its proofs.
/// Time is unchanged. This supplies fixed coordinates, not adaptive QR selection.
#[derive(Clone, Debug)]
pub struct CoordinateField<const N: usize> {
    data: Arc<CoordinateData<N>>,
}
impl<const N: usize> ExpressionField<N> {
    /// Certify x=c+A*y and construct y'=A^-1*f(t,c+A*y), retaining the association.
    /// Exact scalar/predicate payload and total-heap exclusions remain explicit.
    pub fn in_coordinates(
        &self,
        change: CoordinateChange,
        budget: &mut EnclosureBudget,
    ) -> Result<CoordinateField<N>, CoordinateFieldError> {
        budget.graph.charge(N)?;
        if change.variables != self.identity().state() {
            return Err(CoordinateFieldError::StateOrder);
        }
        let transformation = self
            .data
            .identity
            .0
            .source
            .change_coordinates(change, &mut budget.graph)?;
        budget.graph.charge(N)?;
        let field = Self::from_validated_source(
            transformation.program().clone(),
            self.data.identity.time(),
            self.data.identity.state().to_vec(),
            self.data.inputs.len(),
            budget,
        )?;
        budget.graph.charge(1)?;
        let result = CoordinateField {
            data: Arc::new(CoordinateData {
                original: self.clone(),
                transformation,
                field,
            }),
        };
        budget.graph.charge(0)?;
        Ok(result)
    }
}
impl<const N: usize> CoordinateField<N> {
    /// Reconstruct the source and regenerate every inverse/derivative proof.
    pub fn from_definition(
        definition: CoordinateFieldDefinition,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, CoordinateFieldError> {
        ExpressionField::<N>::from_definition(definition.source, budget)?
            .in_coordinates(definition.change, budget)
    }
    /// Original field, with the original state/time meaning.
    pub fn original_field(&self) -> &ExpressionField<N> {
        &self.data.original
    }
    /// Exact chart, original source mapping and inverse evidence.
    pub fn transformation(&self) -> &CoordinateProgram {
        &self.data.transformation
    }
    /// Owned transformed field consumable by existing Jacobian/Taylor/ODE routines.
    pub fn transformed_field(&self) -> &ExpressionField<N> {
        &self.data.field
    }
    /// Cartesian enclosure of the exact image c+A*Y. Correlation stays in the chart.
    /// Uses the caller's grid and shared native budget; empty/unit remain distinct.
    pub fn source_enclosure(
        &self,
        state: &BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<BoxN<DyadicInterval, N>, CoordinateFieldError> {
        budget.graph.charge(0)?;
        let square = N.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
        budget.graph.check_entries(&[square, N, N, N])?;
        if state.is_empty() {
            let empty = BoxN::empty()?;
            budget.graph.charge(0)?;
            return Ok(empty);
        }
        let chart = self.transformation().change();
        let mut matrix = Vec::new();
        let mut translation = Vec::new();
        for row in &chart.to_source {
            budget.graph.charge(1)?;
            let mut values = Vec::new();
            for value in row {
                budget.graph.charge(1)?;
                values.push(DyadicInterval::from_real(
                    value,
                    budget.policy.precision,
                    &mut budget.arithmetic,
                )?);
            }
            matrix.push(values.try_into().expect("validated chart row"));
        }
        for value in &chart.origin {
            budget.graph.charge(1)?;
            translation.push(DyadicInterval::from_real(
                value,
                budget.policy.precision,
                &mut budget.arithmetic,
            )?);
        }
        budget.graph.charge(square)?;
        let matrix = IntervalMatrix::<DyadicInterval, N, N>::new(
            matrix.try_into().expect("validated chart dimension"),
        )?;
        budget.graph.charge(N)?;
        let translation =
            IntervalVector::new(translation.try_into().expect("validated translation"))?;
        let result = matrix.affine_enclosure(state, &translation, &mut budget.arithmetic)?;
        budget.graph.charge(0)?;
        Ok(result)
    }
    /// Verify one step in chart coordinates and transport its complete tube/end.
    ///
    /// The original initial set is c+A*proposal.initial, generally correlated.
    /// Existence is for that exact set, not every point of its Cartesian envelope.
    /// Transport failure preserves the native report but cannot publish success.
    pub fn certify_step(
        &self,
        proposal: PicardProposal<N>,
        budget: &mut EnclosureBudget,
    ) -> CoordinateStep<N> {
        let native = self.transformed_field().certify_step(proposal, budget);
        let status = match native.status() {
            PicardStatus::Certified => CoordinateStatus::Certified,
            PicardStatus::Empty => CoordinateStatus::Empty,
            PicardStatus::Rejected(reason) => CoordinateStatus::Rejected(reason.clone()),
        };
        let mut result = CoordinateStep {
            chart: self.clone(),
            native,
            status,
            source_boxes: None,
        };
        if matches!(
            result.status,
            CoordinateStatus::Certified | CoordinateStatus::Empty
        ) {
            let materialized = (|| {
                let initial = self.source_enclosure(&result.native.proposal().initial, budget)?;
                let tube = self.source_enclosure(
                    result
                        .native
                        .tube()
                        .expect("certified or empty native tube"),
                    budget,
                )?;
                let end = self.source_enclosure(
                    result
                        .native
                        .end_enclosure()
                        .expect("certified or empty native report"),
                    budget,
                )?;
                budget.graph.charge(1)?;
                Ok::<_, CoordinateFieldError>((initial, tube, end))
            })();
            match materialized {
                Ok(boxes) => result.source_boxes = Some(boxes),
                Err(e) => result.status = CoordinateStatus::TransportFailed(e),
            }
        }
        result
    }
}
/// Whole chart step status, including final original-coordinate materialization.
#[derive(Clone, Debug)]
pub enum CoordinateStatus {
    /// The transformed proof and original-coordinate tube/end transport succeeded.
    Certified,
    /// The original initial set is empty; no existence claim is made.
    Empty,
    /// Original native proposal failure, retained without relabeling.
    Rejected(PicardFailure),
    /// Native proof exists, but final original-coordinate output could not be completed.
    TransportFailed(CoordinateFieldError),
}
/// Native Picard proof and exact chart association, with explicit outer projections.
#[derive(Clone, Debug)]
pub struct CoordinateStep<const N: usize> {
    chart: CoordinateField<N>,
    native: PicardReport<N>,
    status: CoordinateStatus,
    source_boxes: Option<(
        BoxN<DyadicInterval, N>,
        BoxN<DyadicInterval, N>,
        BoxN<DyadicInterval, N>,
    )>,
}
impl<const N: usize> CoordinateStep<N> {
    /// Retained original/transformed field association and inverse proof.
    pub fn chart(&self) -> &CoordinateField<N> {
        &self.chart
    }
    /// Full native attempt, including its own local status and intermediate evidence.
    pub fn native(&self) -> &PicardReport<N> {
        &self.native
    }
    /// Whole-step status; native success alone does not establish transported success.
    pub fn status(&self) -> &CoordinateStatus {
        &self.status
    }
    /// Cartesian envelope of the exact correlated original initial set.
    pub fn source_initial_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.source_boxes.as_ref().map(|v| &v.0)
    }
    /// Original-coordinate trajectory envelope, only after all transport succeeds.
    pub fn source_tube(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.source_boxes.as_ref().map(|v| &v.1)
    }
    /// Original-coordinate endpoint envelope, only after all transport succeeds.
    pub fn source_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.source_boxes.as_ref().map(|v| &v.2)
    }
}

#[cfg(feature = "coordinate-transition")]
mod transition;
#[cfg(feature = "coordinate-transition")]
pub use transition::{CoordinateFieldTransition, CoordinateTransport, CoordinateTransportError};
