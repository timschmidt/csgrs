//! Primitive RK4 selects candidate coordinates, never a certified trajectory.
//!
//! Four classical RK stages sample a lossy view of the owned source. Initial
//! uncertainty and heuristic padding enlarge the sampled box, without any error
//! bound. Exact IEEE endpoint ingress adopts that box as a candidate set only.
//! Consuming `Rk4Proposal::certify` replays the original exact source through the
//! existing Picard verifier; only that report can expose a certified endpoint.
//! Native integration and its deterministic proposal search remain independent.
use super::{evolution::EvolutionRequest, picard::*, *};
use core::cmp::Ordering;
use hyperinterval::F64ProposalInterval;
use hyperreal::Real;
use hypersolve::expression_program::{FloatProposalError, FloatProposalProgram};

/// Explicit heuristic padding, with no rounding or truncation-error claim.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Rk4Options {
    /// Add this fraction of the estimated trajectory span to both sides.
    pub relative_padding: f64,
    /// Also add this absolute amount in each state coordinate.
    pub absolute_padding: f64,
}
impl Default for Rk4Options {
    fn default() -> Self {
        Self {
            relative_padding: 0.25,
            absolute_padding: 0.0,
        }
    }
}

/// Which exact input could not be projected to a finite primitive estimate.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProposalInput {
    /// Exact start time.
    StartTime,
    /// Exact end time.
    EndTime,
    /// Initial state interval, in original coordinate order.
    State(usize),
}

/// Proposal failure; numerical failures do not classify the exact ODE.
#[derive(Clone, Debug)]
pub enum Rk4Error {
    /// Exact source/native preparation or its resource/cancellation failed.
    Field(FieldError),
    /// Native stage evaluation failed; stage inputs remain in the trace.
    Estimate {
        /// Zero-based RK stage.
        stage: usize,
        /// Original-node primitive diagnostic.
        source: FloatProposalError,
    },
    /// At least one exact time is empty or nonsingleton.
    NonPointTime,
    /// Exact end time is not strictly after start.
    NonPositiveDuration,
    /// Padding is negative, infinite or NaN.
    InvalidPadding,
    /// Exact input has no finite primitive view.
    ProjectionUnavailable(ProposalInput),
    /// Primitive time estimates collapse, reverse or overflow their duration.
    PrimitiveTimeUnavailable,
    /// The weighted endpoint or padded candidate is nonfinite or invalid.
    Candidate {
        /// State coordinate being assembled.
        coordinate: usize,
        /// Native proposal-coordinate diagnostic.
        source: IntervalError,
    },
}
impl fmt::Display for Rk4Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "RK4 proposal: {self:?}")
    }
}
impl std::error::Error for Rk4Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Field(error) => Some(error),
            Self::Estimate { source, .. } => Some(source),
            Self::Candidate { source, .. } => Some(source),
            _ => None,
        }
    }
}
impl From<ExpressionError> for Rk4Error {
    fn from(error: ExpressionError) -> Self {
        Self::Field(error.into())
    }
}
impl From<IntervalError> for Rk4Error {
    fn from(error: IntervalError) -> Self {
        Self::Field(error.into())
    }
}
impl From<IntervalLatticeError> for Rk4Error {
    fn from(error: IntervalLatticeError) -> Self {
        Self::Field(error.into())
    }
}

/// One attempted primitive sample. No containment or exact-source value is claimed.
#[derive(Clone, Debug)]
pub struct Rk4Stage<const N: usize> {
    time: f64,
    state: [f64; N],
    derivative: Option<[f64; N]>,
}
impl<const N: usize> Rk4Stage<N> {
    /// Native time estimate at this stage.
    pub fn time(&self) -> f64 {
        self.time
    }
    /// Attempted state estimates; may contain nonfinite values on a failed stage.
    pub fn state(&self) -> &[f64; N] {
        &self.state
    }
    /// Complete finite derivative estimates, or None if evaluation failed.
    pub fn derivative(&self) -> Option<&[f64; N]> {
        self.derivative.as_ref()
    }
}

/// Retained numerical search data, distinct from subsequent exact proof evidence.
#[derive(Clone, Debug)]
pub struct Rk4Trace<const N: usize> {
    options: Rk4Options,
    stages: Vec<Rk4Stage<N>>,
    end: Option<[f64; N]>,
    bounds: Vec<F64ProposalInterval>,
}
impl<const N: usize> Rk4Trace<N> {
    /// Recorded heuristic settings, including invalid settings in a failure.
    pub fn options(&self) -> Rk4Options {
        self.options
    }
    /// Every attempted RK stage in evaluation order.
    pub fn stages(&self) -> &[Rk4Stage<N>] {
        &self.stages
    }
    /// Weighted point endpoint estimate, possibly nonfinite on preparation failure.
    pub fn end_estimate(&self) -> Option<&[f64; N]> {
        self.end.as_ref()
    }
    /// Completed coordinate estimates; failures can leave only a prefix.
    pub fn bounds(&self) -> &[F64ProposalInterval] {
        &self.bounds
    }
}

/// Reusable source-bound primitive view of an exact vector field.
#[derive(Clone, Debug)]
pub struct Rk4Proposer<const N: usize> {
    field: ExpressionField<N>,
    program: FloatProposalProgram,
}
impl<const N: usize> ExpressionField<N> {
    /// Prepare a lossy RK proposal view; this does not weaken the original field.
    /// Constant projection/refinement retains Hypersolve's accounting exclusions.
    pub fn rk4_proposer(
        &self,
        budget: &mut EnclosureBudget,
    ) -> Result<Rk4Proposer<N>, FloatProposalError> {
        let program = self
            .data
            .identity
            .0
            .source
            .float_proposal(&mut budget.graph)?;
        budget.graph.charge(1)?;
        Ok(Rk4Proposer {
            field: self.clone(),
            program,
        })
    }
}

#[derive(Clone, Debug)]
struct FailureData<const N: usize> {
    proposer: Rk4Proposer<N>,
    request: EvolutionRequest<N>,
    trace: Rk4Trace<N>,
    error: Rk4Error,
}
/// An unavailable proposal with its original source, exact request and partial trace.
#[derive(Clone, Debug)]
pub struct Rk4Failure<const N: usize> {
    data: Arc<FailureData<N>>,
}
impl<const N: usize> Rk4Failure<N> {
    /// Original prepared source; projection failures are not exact-domain proofs.
    pub fn proposer(&self) -> &Rk4Proposer<N> {
        &self.data.proposer
    }
    /// Exact requested inputs, unchanged by failed numerical work.
    pub fn request(&self) -> &EvolutionRequest<N> {
        &self.data.request
    }
    /// All numerical stages completed or attempted before failure.
    pub fn trace(&self) -> &Rk4Trace<N> {
        &self.data.trace
    }
    /// Typed preparation failure, distinct from a rejected Picard certificate.
    pub fn error(&self) -> &Rk4Error {
        &self.data.error
    }
}
impl<const N: usize> fmt::Display for Rk4Failure<N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.error().fmt(f)
    }
}
impl<const N: usize> std::error::Error for Rk4Failure<N> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        Some(self.error())
    }
}

/// A source-bound candidate set, without a trajectory/enclosure certificate.
#[derive(Clone, Debug)]
pub struct Rk4Proposal<const N: usize> {
    proposer: Rk4Proposer<N>,
    exact: PicardProposal<N>,
    trace: Rk4Trace<N>,
}
impl<const N: usize> Rk4Proposal<N> {
    /// Original source and prepared primitive view.
    pub fn proposer(&self) -> &Rk4Proposer<N> {
        &self.proposer
    }
    /// Exact inputs and adopted candidate coordinates; candidate is still untrusted.
    pub fn proposal(&self) -> &PicardProposal<N> {
        &self.exact
    }
    /// Numerical estimates used to select this candidate.
    pub fn trace(&self) -> &Rk4Trace<N> {
        &self.trace
    }
    /// Consume the candidate through the original exact field's existing verifier.
    /// No original input or numerical trace is cloned or discarded. Failed proof
    /// attempts retain all exact intermediate evidence and the proposal trace.
    pub fn certify(self, budget: &mut EnclosureBudget) -> Rk4Attempt<N> {
        let proof = self.proposer.field.certify_step(self.exact, budget);
        Rk4Attempt {
            proposer: self.proposer,
            trace: self.trace,
            proof,
        }
    }
}

/// A numerical proposal plus its independent, source-bound exact verification.
#[derive(Clone, Debug)]
pub struct Rk4Attempt<const N: usize> {
    proposer: Rk4Proposer<N>,
    trace: Rk4Trace<N>,
    proof: PicardReport<N>,
}
impl<const N: usize> Rk4Attempt<N> {
    /// Retained original and primitive program association.
    pub fn proposer(&self) -> &Rk4Proposer<N> {
        &self.proposer
    }
    /// Non-authoritative numerical search data.
    pub fn trace(&self) -> &Rk4Trace<N> {
        &self.trace
    }
    /// The sole authority for status, certified tube and certified endpoint.
    pub fn proof(&self) -> &PicardReport<N> {
        &self.proof
    }
}

impl<const N: usize> Rk4Proposer<N> {
    /// Original exact vector field, available for independent native fallback.
    pub fn field(&self) -> &ExpressionField<N> {
        &self.field
    }
    /// Prepared native estimates retaining the exact expression source.
    pub fn program(&self) -> &FloatProposalProgram {
        &self.program
    }
    /// Propose one forward step using the classical four-stage RK formula.
    ///
    /// Midpoints and initial radii are selected explicitly for numerical search.
    /// Neither the sampled path nor its padded hull encloses the source by itself.
    /// Exact time validation and final source verification remain native. An empty
    /// initial set yields an empty candidate after time/options validation, without
    /// sampling. A failed proposal retains its stages and does not alter the field.
    pub fn propose(
        &self,
        request: EvolutionRequest<N>,
        options: Rk4Options,
        budget: &mut EnclosureBudget,
    ) -> Result<Rk4Proposal<N>, Rk4Failure<N>> {
        let mut trace = Rk4Trace {
            options,
            stages: Vec::new(),
            end: None,
            bounds: Vec::new(),
        };
        match self.candidate(&request, &mut trace, budget) {
            Ok(candidate) => Ok(Rk4Proposal {
                proposer: self.clone(),
                exact: PicardProposal {
                    start: request.start,
                    end: request.end,
                    initial: request.initial,
                    candidate,
                },
                trace,
            }),
            Err(error) => Err(Rk4Failure {
                data: Arc::new(FailureData {
                    proposer: self.clone(),
                    request,
                    trace,
                    error,
                }),
            }),
        }
    }

    fn candidate(
        &self,
        request: &EvolutionRequest<N>,
        trace: &mut Rk4Trace<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<BoxN<DyadicInterval, N>, Rk4Error> {
        budget.graph.charge(1)?;
        budget
            .graph
            .check_entries(&[N.checked_mul(20).ok_or(ExpressionError::SizeExhausted)?, 4])?;
        if !request.start.is_singleton() || !request.end.is_singleton() {
            return Err(Rk4Error::NonPointTime);
        }
        if request
            .start
            .compare_singleton(&request.end, &mut budget.arithmetic)?
            != Ordering::Less
        {
            return Err(Rk4Error::NonPositiveDuration);
        }
        let options = trace.options;
        if F64ProposalInterval::new(0.0, options.relative_padding).is_err()
            || F64ProposalInterval::new(0.0, options.absolute_padding).is_err()
        {
            return Err(Rk4Error::InvalidPadding);
        }
        if request.initial.is_empty() {
            budget.graph.charge(0)?;
            return Ok(BoxN::empty()?);
        }
        let start = project(&request.start, ProposalInput::StartTime, budget)?
            .estimates()
            .expect("nonempty time")
            .0;
        let end = project(&request.end, ProposalInput::EndTime, budget)?
            .estimates()
            .expect("nonempty time")
            .0;
        let h = end - start;
        if !h.is_finite() || h <= 0.0 {
            return Err(Rk4Error::PrimitiveTimeUnavailable);
        }
        let middle = F64ProposalInterval::new(start, end)?
            .midpoint_estimate()
            .expect("nonempty time");
        budget
            .graph
            .charge(N.checked_mul(2).ok_or(ExpressionError::SizeExhausted)?)?;
        let mut initial = [0.0; N];
        let mut radius = [0.0; N];
        for (i, lane) in request
            .initial
            .lanes()
            .expect("nonempty initial set")
            .iter()
            .enumerate()
        {
            let projected = project(lane, ProposalInput::State(i), budget)?;
            let (a, b) = projected.estimates().expect("nonempty initial lane");
            initial[i] = projected
                .midpoint_estimate()
                .expect("nonempty initial lane");
            radius[i] = b * 0.5 - a * 0.5;
        }
        let k1 = self.stage(start, initial, trace, budget)?;
        budget.graph.charge(N)?;
        let k2 = self.stage(
            middle,
            std::array::from_fn(|i| initial[i] + (h * 0.5) * k1[i]),
            trace,
            budget,
        )?;
        budget.graph.charge(N)?;
        let k3 = self.stage(
            middle,
            std::array::from_fn(|i| initial[i] + (h * 0.5) * k2[i]),
            trace,
            budget,
        )?;
        budget.graph.charge(N)?;
        let k4 = self.stage(
            end,
            std::array::from_fn(|i| initial[i] + h * k3[i]),
            trace,
            budget,
        )?;
        budget.graph.charge(N)?;
        let estimated_end: [f64; N] = std::array::from_fn(|i| {
            initial[i] + (h / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
        });
        trace.end = Some(estimated_end);
        let mut lanes = Vec::new();
        for (i, endpoint) in estimated_end.iter().copied().enumerate() {
            budget.graph.charge(8)?;
            if !endpoint.is_finite() {
                return Err(Rk4Error::Candidate {
                    coordinate: i,
                    source: IntervalError::Overflow,
                });
            }
            let mut lower = endpoint;
            let mut upper = endpoint;
            for stage in &trace.stages {
                lower = lower.min(stage.state[i]);
                upper = upper.max(stage.state[i]);
            }
            lower -= radius[i];
            upper += radius[i];
            // These are heuristic coordinates, not error or enclosure bounds.
            let padding = options.absolute_padding + options.relative_padding * (upper - lower);
            let bounds =
                F64ProposalInterval::new(lower - padding, upper + padding).map_err(|source| {
                    Rk4Error::Candidate {
                        coordinate: i,
                        source,
                    }
                })?;
            trace.bounds.push(bounds);
            let (a, b) = bounds.estimates().expect("nonempty candidate");
            // Adopt the IEEE coordinates exactly as a candidate set. This shared
            // ingress does not certify any relation between the guess and the ODE.
            lanes.push(DyadicInterval::from_f64(a, &mut budget.arithmetic)?.hull(
                &DyadicInterval::from_f64(b, &mut budget.arithmetic)?,
                &mut budget.arithmetic,
            )?);
        }
        let array: [DyadicInterval; N] =
            lanes.try_into().expect("one candidate lane per coordinate");
        let candidate = BoxN::new(array)?.hull(&request.initial, &mut budget.arithmetic)?;
        budget.graph.charge(0)?;
        Ok(candidate)
    }

    fn stage(
        &self,
        time: f64,
        state: [f64; N],
        trace: &mut Rk4Trace<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<[f64; N], Rk4Error> {
        budget.graph.charge(
            N.checked_mul(2)
                .and_then(|n| n.checked_add(1))
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        let index = trace.stages.len();
        trace.stages.push(Rk4Stage {
            time,
            state,
            derivative: None,
        });
        budget.graph.charge(self.field.data.inputs.len())?;
        let arguments: Vec<_> = self
            .field
            .data
            .inputs
            .iter()
            .map(|input| match input {
                Input::Time => time,
                Input::State(i) => state[*i],
            })
            .collect();
        let values = self
            .program
            .estimate(&arguments, &mut budget.graph)
            .map_err(|source| Rk4Error::Estimate {
                stage: index,
                source,
            })?;
        let derivative: [f64; N] = values.try_into().expect("validated field output count");
        trace.stages[index].derivative = Some(derivative);
        Ok(derivative)
    }
}

fn project(
    value: &DyadicInterval,
    input: ProposalInput,
    budget: &mut EnclosureBudget,
) -> Result<F64ProposalInterval, Rk4Error> {
    budget.graph.charge(1)?;
    let rational = value.to_rational(&mut budget.arithmetic)?;
    let a = rational
        .lower()
        .and_then(|x| Real::new(x.clone()).to_f64_lossy())
        .filter(|x| x.is_finite())
        .ok_or(Rk4Error::ProjectionUnavailable(input))?;
    let b = rational
        .upper()
        .and_then(|x| Real::new(x.clone()).to_f64_lossy())
        .filter(|x| x.is_finite())
        .ok_or(Rk4Error::ProjectionUnavailable(input))?;
    Ok(F64ProposalInterval::new(a, b)?)
}
