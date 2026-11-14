//! Compose certified Taylor endpoints using the existing bounded Picard search.
use super::*;
use crate::field::evolution::{
    EvolutionFailure, EvolutionOptions, EvolutionReport, EvolutionRequest, EvolutionStatus,
    copy_state,
};
use core::cmp::Ordering;

/// Search/bookkeeping failure or original trajectory Taylor failure.
#[derive(Clone, Debug)]
pub enum FlowTaylorEvolutionFailure {
    /// Original native search, input, resource or cancellation diagnostic.
    Search(EvolutionFailure),
    /// Original analytic-domain, coefficient, remainder or resource diagnostic.
    Taylor(FlowTaylorError),
}
impl fmt::Display for FlowTaylorEvolutionFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Taylor evolution: {self:?}")
    }
}
impl std::error::Error for FlowTaylorEvolutionFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Search(e) => Some(e),
            Self::Taylor(e) => Some(e),
        }
    }
}
impl From<EvolutionFailure> for FlowTaylorEvolutionFailure {
    fn from(e: EvolutionFailure) -> Self {
        Self::Search(e)
    }
}
impl From<ExpressionError> for FlowTaylorEvolutionFailure {
    fn from(e: ExpressionError) -> Self {
        Self::Search(e.into())
    }
}
impl From<IntervalError> for FlowTaylorEvolutionFailure {
    fn from(e: IntervalError) -> Self {
        Self::Search(e.into())
    }
}

/// Whole-horizon outcome, separate from an available certified prefix.
#[derive(Clone, Debug)]
pub enum FlowTaylorEvolutionStatus {
    /// Certified Taylor segments reach the exact requested horizon.
    Completed,
    /// Zero-duration identity, without an ODE existence claim.
    Identity,
    /// Validated empty initial-set evolution.
    Empty,
    /// Only the explicitly retained prefix is certified.
    Stopped(FlowTaylorEvolutionFailure),
}

/// One complete native search and its optional Taylor refinement.
#[derive(Clone, Debug)]
pub struct FlowTaylorEvolutionAttempt<const N: usize> {
    search: EvolutionReport<N>,
    evidence: FlowTaylorEvidence<N>,
    status: Option<FlowTaylorStatus>,
}
impl<const N: usize> FlowTaylorEvolutionAttempt<N> {
    /// All native attempts in order. StepLimit(1) after one accepted native step
    /// is the intended search prefix, not a whole-horizon completion claim.
    pub fn search(&self) -> &EvolutionReport<N> {
        &self.search
    }
    /// Completed Taylor stages, including partial evidence from a failed refinement.
    pub fn evidence(&self) -> &FlowTaylorEvidence<N> {
        &self.evidence
    }
    /// None when no Taylor refinement was invoked; otherwise its original result.
    pub fn status(&self) -> Option<&FlowTaylorStatus> {
        self.status.as_ref()
    }
}

/// One accepted Taylor segment, borrowing the retained original Picard proof.
#[derive(Clone, Copy, Debug)]
pub struct FlowTaylorEvolutionSegment<'a, const N: usize> {
    attempt: &'a FlowTaylorEvolutionAttempt<N>,
}
impl<'a, const N: usize> FlowTaylorEvolutionSegment<'a, N> {
    /// Exact inputs, time interval and existence/uniqueness evidence, without rechecking.
    pub fn picard(&self) -> &'a PicardReport<N> {
        self.attempt
            .search
            .segment(0)
            .expect("accepted search prefix")
    }
    /// Complete analytic-domain, coefficient and remainder evidence.
    pub fn evidence(&self) -> &'a FlowTaylorEvidence<N> {
        &self.attempt.evidence
    }
    /// Certified trajectory tube including the complete remainder.
    pub fn tube(&self) -> &'a BoxN<DyadicInterval, N> {
        self.attempt
            .evidence
            .tube
            .as_ref()
            .expect("accepted Taylor tube")
    }
    /// Certified endpoint used verbatim as the next segment's initial set.
    pub fn end_enclosure(&self) -> &'a BoxN<DyadicInterval, N> {
        self.attempt
            .evidence
            .end
            .as_ref()
            .expect("accepted Taylor endpoint")
    }
}

/// Immutable whole-horizon composition at one prepared Taylor order.
///
/// Replay the source, order, request and options; no report is deserializable as
/// proof. Every search retains its rejected attempts. Each refinement consumes
/// its original Picard proof by reference, never repeating its predicates. Only
/// the completed Taylor endpoint advances the outer controller. Native interval
/// overestimation can accumulate; no reachable-set equality is claimed.
///
/// One budget covers every search, refinement and publication. Aggregate record
/// counts include nested searches and accepted indices. Counts are checked again
/// after a search returns and before Taylor/segment publication; a stopped report
/// retains that search even if the aggregate check fails. These stage checks are
/// not total retained-graph or heap reservations. Scalar payloads, refinement,
/// vector capacity and within-scalar cancellation retain EnclosureBudget's limits.
#[derive(Clone, Debug)]
pub struct FlowTaylorEvolution<const N: usize> {
    program: FlowTaylorProgram<N>,
    request: EvolutionRequest<N>,
    options: EvolutionOptions,
    policy: EnclosurePolicy,
    attempts: Vec<FlowTaylorEvolutionAttempt<N>>,
    segments: Vec<usize>,
    status: FlowTaylorEvolutionStatus,
}
impl<const N: usize> FlowTaylorEvolution<N> {
    /// Source-bound prepared order and full exact coefficient programs.
    pub fn program(&self) -> &FlowTaylorProgram<N> {
        &self.program
    }
    /// Original exact horizon and initial set.
    pub fn request(&self) -> &EvolutionRequest<N> {
        &self.request
    }
    /// Outer step limit and unchanged native proposal-search options.
    pub fn options(&self) -> &EvolutionOptions {
        &self.options
    }
    /// Shared enclosure policy; counters are never reset between stages.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// Complete search/refinement history in execution order.
    pub fn attempts(&self) -> &[FlowTaylorEvolutionAttempt<N>] {
        &self.attempts
    }
    /// Certified contiguous prefix, distinct from whole-horizon success.
    pub fn segments(&self) -> impl ExactSizeIterator<Item = FlowTaylorEvolutionSegment<'_, N>> {
        self.segments.iter().map(|&i| FlowTaylorEvolutionSegment {
            attempt: &self.attempts[i],
        })
    }
    /// Whole-request status.
    pub fn status(&self) -> &FlowTaylorEvolutionStatus {
        &self.status
    }
    /// Reached certified prefix time; empty success reaches the requested end.
    /// Without a segment this is the supplied start, not an input-validation proof.
    pub fn reached_time(&self) -> &DyadicInterval {
        if matches!(self.status, FlowTaylorEvolutionStatus::Empty) {
            return &self.request.end;
        }
        self.segments.last().map_or(&self.request.start, |&i| {
            &self.attempts[i]
                .search
                .segment(0)
                .expect("accepted prefix")
                .proposal()
                .end
        })
    }
    /// Endpoint of the certified positive-duration prefix, including on later failure.
    pub fn prefix_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.segments
            .last()
            .and_then(|&i| self.attempts[i].evidence.end.as_ref())
    }
    /// Endpoint only after complete horizon, identity or validated empty success.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            FlowTaylorEvolutionStatus::Completed => self.prefix_end_enclosure(),
            FlowTaylorEvolutionStatus::Identity | FlowTaylorEvolutionStatus::Empty => {
                Some(&self.request.initial)
            }
            FlowTaylorEvolutionStatus::Stopped(_) => None,
        }
    }
}
impl<const N: usize> FlowTaylorProgram<N> {
    /// Integrate at this prepared order, adapting durations with the existing
    /// bounded Picard search. Complete Taylor remainders feed every next step.
    /// Taylor failure stops without substituting a weaker Picard endpoint.
    pub fn integrate(
        &self,
        request: EvolutionRequest<N>,
        options: EvolutionOptions,
        budget: &mut EnclosureBudget,
    ) -> FlowTaylorEvolution<N> {
        let mut report = FlowTaylorEvolution {
            program: self.clone(),
            request,
            options,
            policy: budget.policy(),
            attempts: Vec::new(),
            segments: Vec::new(),
            status: FlowTaylorEvolutionStatus::Identity,
        };
        report.status = match self.evolve(&mut report, budget) {
            Ok(status) => status,
            Err(e) => FlowTaylorEvolutionStatus::Stopped(e),
        };
        report
    }
    fn evolve(
        &self,
        report: &mut FlowTaylorEvolution<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<FlowTaylorEvolutionStatus, FlowTaylorEvolutionFailure> {
        let mut search_entries = 0usize;
        loop {
            budget.graph.charge(0)?;
            // The first native search validates all request/options fields before
            // empty/identity/zero-step shortcuts. Later iterations inherit that proof.
            if !report.attempts.is_empty() && report.segments.len() == report.options.maximum_steps
            {
                return Err(EvolutionFailure::StepLimit {
                    limit: report.options.maximum_steps,
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
            let start = report
                .reached_time()
                .checked_pow(1, &mut budget.arithmetic)?;
            let initial = copy_state(
                report
                    .prefix_end_enclosure()
                    .unwrap_or(&report.request.initial),
                budget,
            )?;
            let end = report.request.end.checked_pow(1, &mut budget.arithmetic)?;
            budget.graph.charge(1)?;
            let mut options = report.options.clone();
            options.maximum_steps = usize::from(report.options.maximum_steps != 0);
            if let Some(&i) = report.segments.last() {
                options.maximum_step = Some(
                    report.attempts[i]
                        .search
                        .segment(0)
                        .expect("accepted prefix")
                        .evidence()
                        .duration()
                        .expect("positive duration")
                        .checked_pow(1, &mut budget.arithmetic)?,
                );
            }
            let search = self.field.integrate(
                EvolutionRequest {
                    start,
                    end,
                    initial,
                },
                options,
                budget,
            );
            let native_count = search.attempts().len().checked_add(search.segments().len());
            let index = report.attempts.len();
            report.attempts.push(FlowTaylorEvolutionAttempt {
                search,
                evidence: FlowTaylorEvidence::default(),
                status: None,
            });
            search_entries = native_count
                .and_then(|n| search_entries.checked_add(n))
                .ok_or(ExpressionError::SizeExhausted)?;
            budget
                .graph
                .check_entries(&[search_entries, count, report.segments.len(), 1])?;
            let attempt = &mut report.attempts[index];
            match attempt.search.status() {
                EvolutionStatus::Empty => {
                    budget.graph.charge(0)?;
                    return Ok(FlowTaylorEvolutionStatus::Empty);
                }
                EvolutionStatus::Identity => {
                    budget.graph.charge(0)?;
                    return Ok(FlowTaylorEvolutionStatus::Identity);
                }
                EvolutionStatus::Completed
                | EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 }) => {}
                EvolutionStatus::Stopped(e) => return Err(e.clone().into()),
            }
            let picard = attempt
                .search
                .segment(0)
                .expect("successful one-step search");
            let status = match self.verify(picard, &mut attempt.evidence, budget) {
                Ok(status) => status,
                Err(e) => FlowTaylorStatus::Rejected(e),
            };
            attempt.status = Some(status.clone());
            if let FlowTaylorStatus::Rejected(e) = status {
                return Err(FlowTaylorEvolutionFailure::Taylor(e));
            }
            budget.graph.charge(1)?;
            report.segments.push(index);
            match report
                .reached_time()
                .compare_singleton(&report.request.end, &mut budget.arithmetic)?
            {
                Ordering::Equal => {
                    budget.graph.charge(0)?;
                    return Ok(FlowTaylorEvolutionStatus::Completed);
                }
                Ordering::Less => {}
                Ordering::Greater => return Err(EvolutionFailure::NonProgress.into()),
            }
        }
    }
}
