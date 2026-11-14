//! Finite trajectory Taylor formulas, justified by the original field's
//! analyticity on a certified Picard tube. For a1=F and
//! a(k+1)=(partial_t a(k)+D_x a(k)*F)/(k+1), the complete remainder is
//! a(n+1)(T,Y)*h^(n+1). A spatial Taylor model of F alone is not a flow proof.
use super::picard::{PicardFailure, PicardProposal, PicardReport, PicardStatus};
use super::taylor::{FieldTaylorError, FieldTaylorModel, FieldTaylorRequest};
use super::*;
use hyperreal::Real;
use hypersolve::expression_program::{
    BinaryOperation, CoefficientProgram, CoefficientRequest, ExpressionNode, TaylorError,
};

/// Preparation, original Picard, analytic-domain or resource failure.
#[derive(Clone, Debug)]
pub enum FlowTaylorError {
    /// The requested remainder order cannot be represented.
    OrderOverflow,
    /// The original existence/uniqueness attempt did not certify this step.
    Picard(PicardFailure),
    /// Original field, graph, native arithmetic or cancellation diagnostic.
    Field(FieldError),
    /// Original source-bound symbolic derivative diagnostic.
    Coefficient(TaylorError),
    /// Original whole-tube analytic Taylor-domain proof diagnostic.
    Analytic(FieldTaylorError),
    /// Independently certified enclosures unexpectedly have empty intersection.
    InconsistentEnclosure,
}
impl fmt::Display for FlowTaylorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "trajectory Taylor enclosure: {self:?}")
    }
}
impl std::error::Error for FlowTaylorError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Picard(e) => Some(e),
            Self::Field(e) => Some(e),
            Self::Coefficient(e) => Some(e),
            Self::Analytic(e) => Some(e),
            _ => None,
        }
    }
}
impl From<FieldError> for FlowTaylorError {
    fn from(e: FieldError) -> Self {
        Self::Field(e)
    }
}
impl From<ExpressionError> for FlowTaylorError {
    fn from(e: ExpressionError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalError> for FlowTaylorError {
    fn from(e: IntervalError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalLatticeError> for FlowTaylorError {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Field(e.into())
    }
}

/// One exact normalized Lie coefficient, with retained derivative association.
#[derive(Clone, Debug)]
pub struct FlowTaylorStage<const N: usize> {
    degree: usize,
    derivatives: Option<CoefficientProgram>,
    program: ExpressionProgram,
    outputs: [usize; N],
}
impl<const N: usize> FlowTaylorStage<N> {
    /// Power of elapsed time multiplying this coefficient.
    pub fn degree(&self) -> usize {
        self.degree
    }
    /// Source-bound first partials used in the recurrence; absent for a1=F.
    pub fn derivatives(&self) -> Option<&CoefficientProgram> {
        self.derivatives.as_ref()
    }
    /// Exact executable including every original and generated guard output.
    pub fn program(&self) -> &ExpressionProgram {
        &self.program
    }
    /// Coefficient row positions in the complete executable's output vector.
    pub fn outputs(&self) -> &[usize; N] {
        &self.outputs
    }
}

/// Prepared coefficients through the required remainder order, not a flow proof.
///
/// Replay the original field definition and order. Stages retain exact flat DAGs
/// and demanded derivative associations, never sampled trajectories. Preparation
/// does not assert that a source domain is analytic or an ODE solution exists.
/// Graph budgets cover new structural work and per-stage entries, not total live
/// heap bytes or retained input graphs. Existing scalar-payload/refinement and
/// within-scalar cancellation exclusions remain those of `EnclosureBudget`.
#[derive(Clone, Debug)]
pub struct FlowTaylorProgram<const N: usize> {
    field: ExpressionField<N>,
    order: usize,
    stages: Arc<[FlowTaylorStage<N>]>,
}
impl<const N: usize> ExpressionField<N> {
    /// Prepare a finite trajectory Taylor formula including its next-order remainder.
    /// This never changes the ordinary Picard or evolution algorithm.
    pub fn flow_taylor(
        &self,
        order: usize,
        budget: &mut EnclosureBudget,
    ) -> Result<FlowTaylorProgram<N>, FlowTaylorError> {
        budget.graph.charge(0)?;
        let count = order.checked_add(1).ok_or(FlowTaylorError::OrderOverflow)?;
        let columns = N.checked_add(1).ok_or(ExpressionError::SizeExhausted)?;
        let requests_len = N
            .checked_mul(columns)
            .ok_or(ExpressionError::SizeExhausted)?;
        budget
            .graph
            .check_entries(&[count, N, columns, requests_len, requests_len])?;
        budget.graph.charge(N)?;
        let mut stages = vec![FlowTaylorStage {
            degree: 1,
            derivatives: None,
            program: self.data.jacobian.source().clone(),
            outputs: std::array::from_fn(|i| i),
        }];
        for degree in 2..=count {
            budget.graph.charge(1)?;
            let previous = stages.last().expect("first coefficient exists");
            let mut requests = Vec::new();
            for &output in &previous.outputs {
                for variable in std::iter::once(self.data.identity.time())
                    .chain(self.data.identity.state().iter().copied())
                {
                    budget.graph.charge(2)?;
                    requests.push(CoefficientRequest {
                        output,
                        powers: vec![(variable, 1)],
                    });
                }
            }
            let derivatives = previous
                .program
                .coefficients(&requests, &mut budget.graph)
                .map_err(FlowTaylorError::Coefficient)?;
            let source = derivatives.program().definition();
            let added_nodes = N
                .checked_mul(
                    N.checked_mul(2)
                        .and_then(|v| v.checked_add(1))
                        .ok_or(ExpressionError::SizeExhausted)?,
                )
                .and_then(|v| v.checked_add(1))
                .ok_or(ExpressionError::SizeExhausted)?;
            let nodes = source
                .nodes
                .len()
                .checked_add(added_nodes)
                .ok_or(ExpressionError::SizeExhausted)?;
            let outputs = source
                .outputs
                .len()
                .checked_add(N)
                .ok_or(ExpressionError::SizeExhausted)?;
            budget.graph.check_entries(&[
                count,
                source.variables.len(),
                nodes,
                outputs,
                requests_len,
                requests_len,
                N,
            ])?;
            budget.graph.charge(source.variables.len())?;
            budget.graph.charge(source.nodes.len())?;
            budget.graph.charge(source.outputs.len())?;
            // No public graph builder exists. This is only the ODE recurrence:
            // differentiation, validation and scalar evaluation stay in Hypersolve.
            let mut definition = source.clone();
            budget.graph.charge(1)?;
            let denominator = definition.nodes.len();
            definition
                .nodes
                .push(ExpressionNode::Constant(Real::from(degree as u128)));
            let first = definition.outputs.len();
            for row in 0..N {
                budget.graph.charge(1)?;
                let partial = |column: usize| {
                    source.outputs[derivatives
                        .coefficient_output(row * columns + column)
                        .expect("requested partial")]
                };
                let mut sum = partial(0);
                for state in 0..N {
                    budget.graph.charge(2)?;
                    let product = definition.nodes.len();
                    definition.nodes.push(ExpressionNode::Binary {
                        operation: BinaryOperation::Mul,
                        left: partial(state + 1),
                        right: source.outputs[state],
                    });
                    let next = definition.nodes.len();
                    definition.nodes.push(ExpressionNode::Binary {
                        operation: BinaryOperation::Add,
                        left: sum,
                        right: product,
                    });
                    sum = next;
                }
                budget.graph.charge(2)?;
                let value = definition.nodes.len();
                definition.nodes.push(ExpressionNode::Binary {
                    operation: BinaryOperation::Div,
                    left: sum,
                    right: denominator,
                });
                definition.outputs.push(value);
            }
            let program = ExpressionProgram::from_definition(definition, &mut budget.graph)?;
            budget.graph.charge(N)?;
            stages.push(FlowTaylorStage {
                degree,
                derivatives: Some(derivatives),
                program,
                outputs: std::array::from_fn(|i| first + i),
            });
        }
        budget.graph.charge(count)?;
        let result = FlowTaylorProgram {
            field: self.clone(),
            order,
            stages: stages.into(),
        };
        budget.graph.charge(0)?;
        Ok(result)
    }
}

/// Mathematical completion of one attempted trajectory formula.
#[derive(Clone, Debug)]
pub enum FlowTaylorStatus {
    /// Existence, uniqueness and the entire Taylor remainder are certified.
    Certified,
    /// The initial product is empty; no nonempty trajectory claim is made.
    Empty,
    /// An incomplete attempt; this is not proof of no solution.
    Rejected(FlowTaylorError),
}
/// Completed intermediate stages. Partial stages alone are not a flow certificate.
#[derive(Clone, Debug, Default)]
pub struct FlowTaylorEvidence<const N: usize> {
    analytic: Option<FieldTaylorModel<N>>,
    coefficients: Vec<IntervalVector<DyadicInterval, N>>,
    remainder: Option<IntervalVector<DyadicInterval, N>>,
    end_terms: Vec<IntervalVector<DyadicInterval, N>>,
    tube_terms: Vec<IntervalVector<DyadicInterval, N>>,
    end: Option<BoxN<DyadicInterval, N>>,
    tube: Option<BoxN<DyadicInterval, N>>,
}
impl<const N: usize> FlowTaylorEvidence<N> {
    /// Original field's analytic-domain proof over the whole certified Picard tube.
    pub fn analytic(&self) -> Option<&FieldTaylorModel<N>> {
        self.analytic.as_ref()
    }
    /// a1..an enclosed at the original initial time and uncertain initial state.
    pub fn coefficients(&self) -> &[IntervalVector<DyadicInterval, N>] {
        &self.coefficients
    }
    /// a(n+1) enclosed on the complete physical time and trajectory product.
    pub fn remainder(&self) -> Option<&IntervalVector<DyadicInterval, N>> {
        self.remainder.as_ref()
    }
    /// Full endpoint contributions, including the last remainder contribution.
    pub fn end_terms(&self) -> &[IntervalVector<DyadicInterval, N>] {
        &self.end_terms
    }
    /// Full tube contributions, using elapsed time `[0,h]`, including the remainder.
    pub fn tube_terms(&self) -> &[IntervalVector<DyadicInterval, N>] {
        &self.tube_terms
    }
}
/// Immutable attempted flow proof. Reconstruct from field definition, order and proposal.
/// This report cannot be deserialized as evidence.
#[derive(Clone, Debug)]
pub struct FlowTaylorReport<const N: usize> {
    program: FlowTaylorProgram<N>,
    picard: PicardReport<N>,
    evidence: FlowTaylorEvidence<N>,
    status: FlowTaylorStatus,
}
impl<const N: usize> FlowTaylorReport<N> {
    /// Exact source and normalized coefficient construction.
    pub fn program(&self) -> &FlowTaylorProgram<N> {
        &self.program
    }
    /// Original source-bound existence/uniqueness attempt and its policy.
    pub fn picard(&self) -> &PicardReport<N> {
        &self.picard
    }
    /// Completed stages, including those from rejected attempts.
    pub fn evidence(&self) -> &FlowTaylorEvidence<N> {
        &self.evidence
    }
    /// Whole-step completion or typed failure.
    pub fn status(&self) -> &FlowTaylorStatus {
        &self.status
    }
    /// Certified endpoint, available only after the complete remainder and final checks.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            FlowTaylorStatus::Certified | FlowTaylorStatus::Empty => self.evidence.end.as_ref(),
            _ => None,
        }
    }
    /// Certified complete trajectory enclosure, including successful canonical empty.
    pub fn tube(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            FlowTaylorStatus::Certified | FlowTaylorStatus::Empty => self.evidence.tube.as_ref(),
            _ => None,
        }
    }
}
impl<const N: usize> FlowTaylorProgram<N> {
    /// Original field and time/state interpretation.
    pub fn identity(&self) -> FieldIdentity {
        self.field.identity()
    }
    /// Retained polynomial order (the next order always supplies the remainder).
    pub fn order(&self) -> usize {
        self.order
    }
    /// All normalized coefficient stages through order+1.
    pub fn stages(&self) -> &[FlowTaylorStage<N>] {
        &self.stages
    }
    /// Verify Picard existence/uniqueness, then the full trajectory Taylor formula.
    /// One shared budget covers both attempts. A failed Taylor stage never publishes
    /// the earlier Picard result as Taylor success; it remains separately inspectable.
    pub fn certify_step(
        &self,
        proposal: PicardProposal<N>,
        budget: &mut EnclosureBudget,
    ) -> FlowTaylorReport<N> {
        let picard = self.field.certify_step(proposal, budget);
        let mut report = FlowTaylorReport {
            program: self.clone(),
            picard,
            evidence: FlowTaylorEvidence::default(),
            status: FlowTaylorStatus::Empty,
        };
        report.status = match self.verify(&report.picard, &mut report.evidence, budget) {
            Ok(status) => status,
            Err(e) => FlowTaylorStatus::Rejected(e),
        };
        report
    }
    fn verify(
        &self,
        picard: &PicardReport<N>,
        evidence: &mut FlowTaylorEvidence<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<FlowTaylorStatus, FlowTaylorError> {
        budget.graph.charge(0)?;
        match picard.status() {
            PicardStatus::Rejected(e) => return Err(FlowTaylorError::Picard(e.clone())),
            PicardStatus::Empty => {
                evidence.end = Some(BoxN::empty()?);
                evidence.tube = Some(BoxN::empty()?);
                budget.graph.charge(0)?;
                return Ok(FlowTaylorStatus::Empty);
            }
            PicardStatus::Certified => {}
        }
        let count = self.stages.len();
        let lanes = count.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
        budget
            .graph
            .check_entries(&[count, count, count, lanes, lanes, lanes, N, N, N, N])?;
        let time = picard.evidence().time().expect("certified time");
        let domain = picard.tube().expect("certified tube");
        // Reuse the existing exhaustive analytic-grammar/domain authority. Order
        // zero is enough for analyticity; this spatial model is not itself a flow.
        evidence.analytic = Some(
            self.field
                .taylor_model(
                    FieldTaylorRequest {
                        time: time.clone(),
                        state: domain.clone(),
                        order: 0,
                    },
                    budget,
                )
                .map_err(FlowTaylorError::Analytic)?,
        );
        let initial = &picard.proposal().initial;
        let start = &picard.proposal().start;
        let h = picard.evidence().duration().expect("certified duration");
        let elapsed = DyadicInterval::point(0.into(), 0)?.hull(h, &mut budget.arithmetic)?;
        let mut end_power = DyadicInterval::point(1.into(), 0)?;
        let mut tube_power = end_power.clone();
        budget
            .graph
            .charge(N.checked_mul(2).ok_or(ExpressionError::SizeExhausted)?)?;
        let mut end = initial.clone();
        let mut tube = initial.clone();
        for stage in self.stages.iter() {
            budget.graph.charge(1)?;
            let last = stage.degree == count;
            let arguments = self
                .field
                .arguments(
                    &stage.program,
                    if last { time } else { start },
                    if last { domain } else { initial },
                    budget,
                )?
                .expect("nonempty certified domains");
            let values = stage.program.eval_dyadic(
                &arguments,
                budget.policy.precision,
                &mut budget.graph,
                &mut budget.arithmetic,
            )?;
            budget.graph.charge(N)?;
            let coefficient =
                IntervalVector::new(std::array::from_fn(|i| values[stage.outputs[i]].clone()))?;
            if last {
                evidence.remainder = Some(coefficient.clone());
            } else {
                evidence.coefficients.push(coefficient.clone());
            }
            end_power = end_power.mul(h, &mut budget.arithmetic)?;
            tube_power = tube_power.mul(&elapsed, &mut budget.arithmetic)?;
            budget
                .graph
                .charge(N.checked_mul(4).ok_or(ExpressionError::SizeExhausted)?)?;
            let end_term = coefficient.scale(&end_power, &mut budget.arithmetic)?;
            let tube_term = coefficient.scale(&tube_power, &mut budget.arithmetic)?;
            end = end.add(&end_term, &mut budget.arithmetic)?;
            tube = tube.add(&tube_term, &mut budget.arithmetic)?;
            evidence.end_terms.push(end_term);
            evidence.tube_terms.push(tube_term);
        }
        budget
            .graph
            .charge(N.checked_mul(2).ok_or(ExpressionError::SizeExhausted)?)?;
        end = end.intersection(
            picard.end_enclosure().expect("certified end"),
            &mut budget.arithmetic,
        )?;
        tube = tube.intersection(domain, &mut budget.arithmetic)?;
        if end.is_empty() || tube.is_empty() {
            return Err(FlowTaylorError::InconsistentEnclosure);
        }
        budget.graph.charge(0)?;
        evidence.end = Some(end);
        evidence.tube = Some(tube);
        Ok(FlowTaylorStatus::Certified)
    }
}

/// Whole-horizon composition of certified trajectory Taylor endpoints.
#[cfg(feature = "taylor-evolution")]
pub mod evolution;

/// Bounded automatic order and duration selection with local remainder limits.
#[cfg(feature = "adaptive-taylor")]
pub mod adaptive;

/// Reuse prepared coefficient graphs across polynomial orders.
pub mod orders;
