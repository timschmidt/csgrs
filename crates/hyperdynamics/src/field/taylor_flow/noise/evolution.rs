//! Bounded native searches whose accepted proofs retain affine endpoint uncertainty.
use super::*;
use crate::field::evolution::{
    EvolutionFailure, EvolutionOptions, EvolutionReport, EvolutionRequest, EvolutionStatus,
    copy_state,
};
use core::cmp::Ordering;

/// Native search limits and the retained uncertainty-column capacity.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FlowNoiseEvolutionOptions {
    /// Maximum retained columns per endpoint; all omitted terms remain in residuals.
    pub maximum_noise: usize,
    /// Endpoint model remainder form; see [`FlowNoiseRequest`]. The default,
    /// `Adaptive`, keeps the cheap mean-value form where it is negligible and
    /// otherwise intersects it with the order-two bound.
    #[cfg_attr(feature = "serde", serde(default))]
    pub remainder: TaylorRemainder,
    /// Round each endpoint onto the policy grid; see [`FlowNoiseRequest`].
    #[cfg_attr(feature = "serde", serde(default))]
    pub round_coefficients: bool,
    /// Relative column cutoff for each endpoint; see [`FlowNoiseRequest`].
    #[cfg_attr(feature = "serde", serde(default))]
    pub column_cutoff_bits: Option<u32>,
    /// Outer segment limit and native duration/halving/expansion controls.
    pub native: EvolutionOptions,
}
impl Default for FlowNoiseEvolutionOptions {
    fn default() -> Self {
        Self {
            maximum_noise: 16,
            remainder: TaylorRemainder::Adaptive,
            round_coefficients: true,
            column_cutoff_bits: Some(16),
            native: EvolutionOptions::default(),
        }
    }
}
/// Original search, structural-resource, or correlated endpoint failure.
#[derive(Clone, Debug)]
pub enum FlowNoiseEvolutionFailure {
    /// Original input validation, search, arithmetic or cancellation diagnostic.
    Search(EvolutionFailure),
    /// Original trajectory or affine endpoint construction diagnostic.
    Noise(FlowNoiseError),
}
impl fmt::Display for FlowNoiseEvolutionFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "affine-noise evolution: {self:?}")
    }
}
impl std::error::Error for FlowNoiseEvolutionFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Search(e) => Some(e),
            Self::Noise(e) => Some(e),
        }
    }
}
impl From<EvolutionFailure> for FlowNoiseEvolutionFailure {
    fn from(e: EvolutionFailure) -> Self {
        Self::Search(e)
    }
}
impl From<ExpressionError> for FlowNoiseEvolutionFailure {
    fn from(e: ExpressionError) -> Self {
        Self::Search(e.into())
    }
}
impl From<IntervalError> for FlowNoiseEvolutionFailure {
    fn from(e: IntervalError) -> Self {
        Self::Search(e.into())
    }
}
impl From<IntervalLatticeError> for FlowNoiseEvolutionFailure {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Search(e.into())
    }
}
impl From<FlowNoiseError> for FlowNoiseEvolutionFailure {
    fn from(e: FlowNoiseError) -> Self {
        Self::Noise(e)
    }
}
/// Whole-horizon outcome, distinct from any retained certified prefix.
#[derive(Clone, Debug)]
pub enum FlowNoiseEvolutionStatus {
    /// Every accepted endpoint reached the exact requested end time.
    Completed,
    /// Validated zero-duration identity; no ODE existence claim.
    Identity,
    /// Validated canonical empty set at the requested time.
    Empty,
    /// No whole-horizon endpoint is published.
    Stopped(FlowNoiseEvolutionFailure),
}
/// One native search and its optional complete or rejected noise refinement.
#[derive(Clone, Debug)]
pub struct FlowNoiseEvolutionAttempt<const N: usize> {
    search: EvolutionReport<N>,
    noise: Option<FlowNoiseReport<N>>,
}
impl<const N: usize> FlowNoiseEvolutionAttempt<N> {
    /// Every native candidate, including rejected expansions and halvings.
    pub fn search(&self) -> &EvolutionReport<N> {
        &self.search
    }
    /// None if search/bookkeeping failed before refinement; otherwise all proof stages.
    pub fn noise(&self) -> Option<&FlowNoiseReport<N>> {
        self.noise.as_ref()
    }
}
/// Automatic composition of source-bound affine outer states.
///
/// Every next native search covers the full independent affine parameter image.
/// Accepted native Picard proofs are reused, not evaluated a second time. The
/// analytic Taylor and complete noise/residual proofs must finish before advancing.
/// All attempts remain inspectable; only complete statuses expose `end_state`.
///
/// One shared budget covers nested calls and aggregate history-entry checks.
/// These counts do not bound scalar payloads/refinement, vector capacities, all
/// retained proof graphs, total heap bytes or within-scalar cancellation. Replay
/// the original box, field/order history, horizon and options; no deserialized
/// result flags or executable caches are certificates.
#[derive(Clone, Debug)]
pub struct FlowNoiseEvolution<const N: usize> {
    program: FlowTaylorProgram<N>,
    initial: Arc<FlowNoiseState<N>>,
    end: DyadicInterval,
    options: FlowNoiseEvolutionOptions,
    policy: EnclosurePolicy,
    attempts: Vec<FlowNoiseEvolutionAttempt<N>>,
    segments: Vec<usize>,
    status: FlowNoiseEvolutionStatus,
}
impl<const N: usize> FlowNoiseEvolution<N> {
    /// Prepared source and mandatory Taylor remainder order.
    pub fn program(&self) -> &FlowTaylorProgram<N> {
        &self.program
    }
    /// Exact initial affine outer state, including its earlier proof history.
    pub fn initial(&self) -> &Arc<FlowNoiseState<N>> {
        &self.initial
    }
    /// Original requested exact end time, including when invalid.
    pub fn requested_end(&self) -> &DyadicInterval {
        &self.end
    }
    /// Native search and retained-column configuration.
    pub fn options(&self) -> &FlowNoiseEvolutionOptions {
        &self.options
    }
    /// Shared enclosure policy; counters are not reset between stages.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// All searches and partial/complete refinements in execution order.
    pub fn attempts(&self) -> &[FlowNoiseEvolutionAttempt<N>] {
        &self.attempts
    }
    /// Contiguous certified positive-duration prefix, not whole-request success.
    pub fn segments(&self) -> impl ExactSizeIterator<Item = &FlowNoiseReport<N>> {
        self.segments.iter().map(|&i| {
            self.attempts[i]
                .noise
                .as_ref()
                .expect("accepted refinement")
        })
    }
    /// Last accepted state, available even if a later search or refinement fails.
    pub fn prefix_end_state(&self) -> Option<&Arc<FlowNoiseState<N>>> {
        self.segments.last().and_then(|&i| {
            self.attempts[i]
                .noise
                .as_ref()
                .and_then(FlowNoiseReport::end_state)
        })
    }
    /// Whole-horizon outcome.
    pub fn status(&self) -> &FlowNoiseEvolutionStatus {
        &self.status
    }
    /// Exact reached prefix time; validated empty success reaches the requested end.
    pub fn reached_time(&self) -> &DyadicInterval {
        if matches!(self.status, FlowNoiseEvolutionStatus::Empty) {
            &self.end
        } else {
            self.prefix_end_state().unwrap_or(&self.initial).time()
        }
    }
    /// Reusable final state only after complete, identity or empty success.
    pub fn end_state(&self) -> Option<&Arc<FlowNoiseState<N>>> {
        match self.status {
            FlowNoiseEvolutionStatus::Completed => self.prefix_end_state(),
            FlowNoiseEvolutionStatus::Identity => Some(&self.initial),
            FlowNoiseEvolutionStatus::Empty => Some(
                self.attempts
                    .last()
                    .and_then(|a| a.noise.as_ref())
                    .and_then(FlowNoiseReport::end_state)
                    .unwrap_or(&self.initial),
            ),
            FlowNoiseEvolutionStatus::Stopped(_) => None,
        }
    }
}
impl<const N: usize> FlowTaylorProgram<N> {
    /// Search a bounded forward horizon while retaining correlated affine states.
    /// Native input/options validation precedes empty, identity and zero-step exits.
    /// A failed noise proof never falls back to a Cartesian-only endpoint.
    pub fn integrate_noise(
        &self,
        initial: Arc<FlowNoiseState<N>>,
        end: DyadicInterval,
        options: FlowNoiseEvolutionOptions,
        budget: &mut EnclosureBudget,
    ) -> FlowNoiseEvolution<N> {
        let mut report = FlowNoiseEvolution {
            program: self.clone(),
            initial,
            end,
            options,
            policy: budget.policy(),
            attempts: Vec::new(),
            segments: Vec::new(),
            status: FlowNoiseEvolutionStatus::Identity,
        };
        report.status = match evolve(&mut report, budget) {
            Ok(status) => status,
            Err(e) => FlowNoiseEvolutionStatus::Stopped(e),
        };
        report
    }
}

fn refine<const N: usize>(
    program: &FlowTaylorProgram<N>,
    initial: Arc<FlowNoiseState<N>>,
    request: FlowNoiseRequest<N>,
    picard: &PicardReport<N>,
    budget: &mut EnclosureBudget,
) -> FlowNoiseReport<N> {
    certify_from(program, initial, request, budget, |_, budget| {
        // Copy fixed structural slots, sharing exact source identity. Scalar
        // mantissa payload copying retains the documented budget exclusion.
        let copies = N
            .checked_mul(N)
            .and_then(|v| N.checked_mul(5).and_then(|n| v.checked_add(n)))
            .and_then(|v| v.checked_add(12))
            .ok_or(ExpressionError::SizeExhausted)?;
        budget.graph.check_entries(&[copies])?;
        budget.graph.charge(copies)?;
        let mut flow = FlowTaylorReport {
            program: program.clone(),
            picard: picard.clone(),
            evidence: FlowTaylorEvidence::default(),
            status: FlowTaylorStatus::Empty,
        };
        flow.status = match program.verify(&flow.picard, &mut flow.evidence, budget) {
            Ok(status) => status,
            Err(e) => FlowTaylorStatus::Rejected(e),
        };
        Ok(flow)
    })
}

fn evolve<const N: usize>(
    report: &mut FlowNoiseEvolution<N>,
    budget: &mut EnclosureBudget,
) -> Result<FlowNoiseEvolutionStatus, FlowNoiseEvolutionFailure> {
    let mut search_entries = 0usize;
    loop {
        budget.graph.charge(0)?;
        if !report.attempts.is_empty()
            && report.segments.len() == report.options.native.maximum_steps
        {
            return Err(EvolutionFailure::StepLimit {
                limit: report.options.native.maximum_steps,
            }
            .into());
        }
        let count = report
            .attempts
            .len()
            .checked_add(1)
            .ok_or(ExpressionError::SizeExhausted)?;
        budget
            .graph
            .check_entries(&[search_entries, count, report.segments.len()])?;
        budget.graph.charge(1)?;
        let initial = report.prefix_end_state().unwrap_or(&report.initial).clone();
        let mut options = report.options.native.clone();
        options.maximum_steps = usize::from(options.maximum_steps != 0);
        if let Some(&i) = report.segments.last() {
            options.maximum_step = Some(
                report.attempts[i]
                    .search
                    .segment(0)
                    .expect("accepted search")
                    .evidence()
                    .duration()
                    .expect("positive duration")
                    .checked_pow(1, &mut budget.arithmetic)?,
            );
        }
        let search = report.program.field.integrate(
            EvolutionRequest {
                start: initial.time.checked_pow(1, &mut budget.arithmetic)?,
                end: report.end.checked_pow(1, &mut budget.arithmetic)?,
                initial: copy_state(initial.enclosure(), budget)?,
            },
            options,
            budget,
        );
        let native_count = search.attempts().len().checked_add(search.segments().len());
        let index = report.attempts.len();
        report.attempts.push(FlowNoiseEvolutionAttempt {
            search,
            noise: None,
        });
        search_entries = native_count
            .and_then(|n| search_entries.checked_add(n))
            .ok_or(ExpressionError::SizeExhausted)?;
        budget
            .graph
            .check_entries(&[search_entries, count, report.segments.len(), 1])?;
        let attempt = &mut report.attempts[index];
        match attempt.search.status() {
            EvolutionStatus::Identity => {
                budget.graph.charge(0)?;
                return Ok(FlowNoiseEvolutionStatus::Identity);
            }
            EvolutionStatus::Empty => {
                if initial
                    .time
                    .compare_singleton(&report.end, &mut budget.arithmetic)?
                    == Ordering::Equal
                {
                    budget.graph.charge(0)?;
                    return Ok(FlowNoiseEvolutionStatus::Empty);
                }
                // No Picard step was attempted for this native empty shortcut.
                // Retain the empty time transfer through the ordinary empty-step proof.
                let noise = report.program.certify_noise_step(
                    initial,
                    FlowNoiseRequest {
                        end: report.end.checked_pow(1, &mut budget.arithmetic)?,
                        candidate: BoxN::empty()?,
                        maximum_noise: report.options.maximum_noise,
                        remainder: report.options.remainder,
                        round_coefficients: report.options.round_coefficients,
                        column_cutoff_bits: report.options.column_cutoff_bits,
                    },
                    budget,
                );
                let status = noise.status().clone();
                attempt.noise = Some(noise);
                match status {
                    FlowNoiseStatus::Empty => {
                        budget.graph.charge(0)?;
                        return Ok(FlowNoiseEvolutionStatus::Empty);
                    }
                    FlowNoiseStatus::Rejected(e) => return Err(e.into()),
                    FlowNoiseStatus::Certified => {
                        unreachable!("empty input has no nonempty trajectory proof")
                    }
                }
            }
            EvolutionStatus::Completed
            | EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 }) => {}
            EvolutionStatus::Stopped(e) => return Err(e.clone().into()),
        }
        let picard = attempt.search.segment(0).expect("accepted one-step search");
        let request = FlowNoiseRequest {
            end: picard
                .proposal()
                .end
                .checked_pow(1, &mut budget.arithmetic)?,
            candidate: copy_state(&picard.proposal().candidate, budget)?,
            maximum_noise: report.options.maximum_noise,
            remainder: report.options.remainder,
            round_coefficients: report.options.round_coefficients,
            column_cutoff_bits: report.options.column_cutoff_bits,
        };
        let noise = refine(&report.program, initial.clone(), request, picard, budget);
        let status = noise.status().clone();
        attempt.noise = Some(noise);
        if let FlowNoiseStatus::Rejected(e) = status {
            return Err(e.into());
        }
        let next = attempt
            .noise
            .as_ref()
            .expect("stored refinement")
            .end_state()
            .expect("complete endpoint");
        if initial
            .time
            .compare_singleton(next.time(), &mut budget.arithmetic)?
            != Ordering::Less
        {
            return Err(EvolutionFailure::NonProgress.into());
        }
        let order = next
            .time()
            .compare_singleton(&report.end, &mut budget.arithmetic)?;
        if order == Ordering::Greater {
            return Err(EvolutionFailure::NonProgress.into());
        }
        let accepted = report
            .segments
            .len()
            .checked_add(1)
            .ok_or(ExpressionError::SizeExhausted)?;
        budget
            .graph
            .check_entries(&[search_entries, count, accepted])?;
        budget.graph.charge(1)?;
        report.segments.push(index);
        budget.graph.charge(0)?;
        if order == Ordering::Equal {
            return Ok(FlowNoiseEvolutionStatus::Completed);
        }
    }
}
