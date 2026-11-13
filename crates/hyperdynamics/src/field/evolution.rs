//! Deterministic finite-horizon composition of certified dyadic Picard steps.
//!
//! Every candidate is untrusted until the existing step verifier accepts it.
//! Rejected attempts are retained in execution order. Exact halving and horizon
//! capping use native dyadic arithmetic; no floating-point time controls a proof.
//! Step/refinement limits and shared work counters do not remove the symbolic
//! refinement, payload lifecycle or total-heap exclusions of `EnclosureBudget`.
use super::{picard::*, *};
use core::cmp::Ordering;

/// Exact requested time horizon and initial uncertainty.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EvolutionRequest<const N: usize> {
    /// Exact singleton start time.
    pub start: DyadicInterval,
    /// Exact singleton end time, at or after start.
    pub end: DyadicInterval,
    /// Initial state set; not a selected point trajectory.
    pub initial: BoxN<DyadicInterval, N>,
}

/// Finite search limits for native interval proposals.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct EvolutionOptions {
    /// Optional positive singleton cap. Later steps also do not exceed the last accepted duration.
    pub maximum_step: Option<DyadicInterval>,
    /// Maximum number of accepted positive-duration segments.
    pub maximum_steps: usize,
    /// Maximum exact halvings per segment, in addition to its initial proposal.
    pub maximum_halvings: u32,
    /// Maximum candidate expansions at each duration, in addition to its seed.
    pub maximum_expansions: u32,
}
impl Default for EvolutionOptions {
    fn default() -> Self {
        Self {
            maximum_step: None,
            maximum_steps: 256,
            maximum_halvings: 16,
            maximum_expansions: 4,
        }
    }
}

/// Replayable origin of an untrusted candidate box.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CandidateSource {
    /// The segment's initial box, including after each halving.
    InitialBox,
    /// Hull the named attempt's candidate and Picard image, then add its symmetric width.
    PicardExpansion {
        /// Index in the retained execution-order attempt list.
        attempt: usize,
    },
}

/// One actual invocation of the source-bound step verifier.
#[derive(Clone, Debug)]
pub struct EvolutionAttempt<const N: usize> {
    segment: usize,
    halvings: u32,
    expansions: u32,
    source: CandidateSource,
    report: PicardReport<N>,
}
impl<const N: usize> EvolutionAttempt<N> {
    /// Zero-based segment being attempted; rejected attempts do not advance it.
    pub fn segment(&self) -> usize {
        self.segment
    }
    /// Exact halvings since this segment's first attempted duration.
    pub fn halvings(&self) -> u32 {
        self.halvings
    }
    /// Candidate expansions since this duration's initial-box seed.
    pub fn expansions(&self) -> u32 {
        self.expansions
    }
    /// Untrusted proposal construction, distinct from its proof.
    pub fn source(&self) -> CandidateSource {
        self.source
    }
    /// All source, inputs, intermediate evidence and typed step status.
    pub fn report(&self) -> &PicardReport<N> {
        &self.report
    }
}

/// Why the requested horizon was not certified in full.
#[derive(Clone, Debug)]
pub enum EvolutionFailure {
    /// A time endpoint is empty or not a singleton.
    NonPointTime,
    /// End precedes start.
    ReversedTime,
    /// The configured step cap is not a positive singleton.
    InvalidMaximumStep,
    /// The actual accepted-segment limit was reached before the horizon.
    StepLimit {
        /// Configured maximum accepted segments.
        limit: usize,
    },
    /// The actual halving limit was reached; the last attempt retains its mathematical failure.
    HalvingLimit {
        /// Configured maximum halvings per segment.
        limit: u32,
    },
    /// Exact strict progress could not be established.
    NonProgress,
    /// A step failure for which the proposal search has no retry rule.
    Step(PicardFailure),
    /// Native preparation, resource or cancellation failure.
    Field(FieldError),
}
impl fmt::Display for EvolutionFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "dyadic evolution: {self:?}")
    }
}
impl std::error::Error for EvolutionFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Step(e) => Some(e),
            Self::Field(e) => Some(e),
            _ => None,
        }
    }
}
impl From<FieldError> for EvolutionFailure {
    fn from(e: FieldError) -> Self {
        Self::Field(e)
    }
}
impl From<ExpressionError> for EvolutionFailure {
    fn from(e: ExpressionError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalError> for EvolutionFailure {
    fn from(e: IntervalError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalLatticeError> for EvolutionFailure {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Field(e.into())
    }
}

/// Whole-request status; a certified prefix is not a completed horizon.
#[derive(Clone, Debug)]
pub enum EvolutionStatus {
    /// Certified positive-duration segments reach the exact requested end time.
    Completed,
    /// Zero-duration identity; no ODE existence claim is made.
    Identity,
    /// Successful empty initial-set evolution, after input validation.
    Empty,
    /// Only the explicitly retained prefix, if any, is certified.
    Stopped(EvolutionFailure),
}

/// Immutable source-bound evolution with ordered attempts and accepted segments.
///
/// Replay source/request/options through `integrate`; reports are not deserialized
/// as certificates. Each segment encloses every solution from its initial box,
/// including overestimation inherited from the preceding endpoint. No exact
/// reachable-set equality or cross-segment correlation is claimed.
#[derive(Clone, Debug)]
pub struct EvolutionReport<const N: usize> {
    identity: FieldIdentity,
    request: EvolutionRequest<N>,
    options: EvolutionOptions,
    policy: EnclosurePolicy,
    attempts: Vec<EvolutionAttempt<N>>,
    segments: Vec<usize>,
    status: EvolutionStatus,
}
impl<const N: usize> EvolutionReport<N> {
    /// Shared authoritative source and coordinate interpretation.
    pub fn identity(&self) -> &FieldIdentity {
        &self.identity
    }
    /// Original exact request.
    pub fn request(&self) -> &EvolutionRequest<N> {
        &self.request
    }
    /// Recorded deterministic search limits.
    pub fn options(&self) -> &EvolutionOptions {
        &self.options
    }
    /// One enclosure policy shared by all nested calls, without counter resets.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// Every invoked step verifier, including all rejections, in execution order.
    pub fn attempts(&self) -> &[EvolutionAttempt<N>] {
        &self.attempts
    }
    /// Certified positive-duration segments, forming a contiguous prefix.
    pub fn segments(&self) -> impl ExactSizeIterator<Item = &PicardReport<N>> {
        self.segments.iter().map(|&i| &self.attempts[i].report)
    }
    /// Indexed certified segment without scanning the retained attempts.
    pub fn segment(&self, index: usize) -> Option<&PicardReport<N>> {
        self.segments.get(index).map(|&i| &self.attempts[i].report)
    }
    /// Whole-horizon status, separate from the available prefix.
    pub fn status(&self) -> &EvolutionStatus {
        &self.status
    }
    /// End time of successful empty evolution or of the certified prefix.
    /// Otherwise returns the supplied start when no segment exists; that value
    /// alone makes no existence or input-validation claim.
    pub fn reached_time(&self) -> &DyadicInterval {
        if matches!(self.status, EvolutionStatus::Empty) {
            return &self.request.end;
        }
        self.segments.last().map_or(&self.request.start, |&i| {
            &self.attempts[i].report.proposal().end
        })
    }
    /// Endpoint of the certified positive-duration prefix, never a whole-horizon claim.
    pub fn prefix_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.segments
            .last()
            .and_then(|&i| self.attempts[i].report.end_enclosure())
    }
    /// Enclosure at the exact requested horizon, only after whole-request success.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            EvolutionStatus::Completed => self.prefix_end_enclosure(),
            EvolutionStatus::Identity | EvolutionStatus::Empty => Some(&self.request.initial),
            EvolutionStatus::Stopped(_) => None,
        }
    }
}

impl<const N: usize> ExpressionField<N> {
    /// Integrate by bounded native proposals and the source-bound Picard verifier.
    ///
    /// A failed self-map may enlarge its candidate by the symmetric width of the
    /// hull with its Picard image. After bounded expansions, exact duration halving
    /// restarts from the initial box. Domain/regularity proof failures and failed
    /// contraction may also halve. Resource/cancellation failures stop immediately.
    /// Every actual attempt is retained. All limits and counters remain shared;
    /// no finite search guarantees completion for an arbitrary field or horizon.
    pub fn integrate(
        &self,
        request: EvolutionRequest<N>,
        options: EvolutionOptions,
        budget: &mut EnclosureBudget,
    ) -> EvolutionReport<N> {
        let mut report = EvolutionReport {
            identity: self.identity(),
            request,
            options,
            policy: budget.policy(),
            attempts: Vec::new(),
            segments: Vec::new(),
            status: EvolutionStatus::Identity,
        };
        report.status = match self.evolve(&mut report, budget) {
            Ok(status) => status,
            Err(error) => EvolutionStatus::Stopped(error),
        };
        report
    }

    fn evolve(
        &self,
        report: &mut EvolutionReport<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<EvolutionStatus, EvolutionFailure> {
        budget.graph.charge(2)?;
        if !report.request.start.is_singleton() || !report.request.end.is_singleton() {
            return Err(EvolutionFailure::NonPointTime);
        }
        let order = report
            .request
            .start
            .compare_singleton(&report.request.end, &mut budget.arithmetic)?;
        if order == Ordering::Greater {
            return Err(EvolutionFailure::ReversedTime);
        }
        if let Some(cap) = &report.options.maximum_step {
            budget.graph.charge(1)?;
            if !cap.is_singleton() || cap.signs() != hyperinterval::SignSet::Positive {
                return Err(EvolutionFailure::InvalidMaximumStep);
            }
        }
        if report.request.initial.is_empty() {
            budget.graph.charge(0)?;
            return Ok(EvolutionStatus::Empty);
        }
        if order == Ordering::Equal {
            budget.graph.charge(0)?;
            return Ok(EvolutionStatus::Identity);
        }
        loop {
            budget.graph.charge(0)?;
            if report.segments.len() == report.options.maximum_steps {
                return Err(EvolutionFailure::StepLimit {
                    limit: report.options.maximum_steps,
                });
            }
            let start = report
                .reached_time()
                .checked_pow(1, &mut budget.arithmetic)?;
            let initial = copy_state(
                report
                    .prefix_end_enclosure()
                    .unwrap_or(&report.request.initial),
                budget,
            )?;
            let remaining = report.request.end.sub(&start, &mut budget.arithmetic)?;
            let limit = report
                .segments
                .last()
                .and_then(|&i| report.attempts[i].report.evidence().duration())
                .or(report.options.maximum_step.as_ref());
            let duration = match limit {
                Some(limit) => remaining.min_image(limit, &mut budget.arithmetic)?,
                None => remaining,
            };
            let mut end = start.add(&duration, &mut budget.arithmetic)?;
            let mut halvings = 0;
            'halving: loop {
                if start.compare_singleton(&end, &mut budget.arithmetic)? != Ordering::Less {
                    return Err(EvolutionFailure::NonProgress);
                }
                let mut candidate = copy_state(&initial, budget)?;
                let mut source = CandidateSource::InitialBox;
                let mut expansions = 0;
                loop {
                    let count = report
                        .attempts
                        .len()
                        .checked_add(1)
                        .ok_or(ExpressionError::SizeExhausted)?;
                    budget
                        .graph
                        .check_entries(&[count, report.segments.len()])?;
                    budget.graph.charge(1)?;
                    let proposal = PicardProposal {
                        start: start.checked_pow(1, &mut budget.arithmetic)?,
                        end: end.checked_pow(1, &mut budget.arithmetic)?,
                        initial: copy_state(&initial, budget)?,
                        candidate,
                    };
                    let step = self.certify_step(proposal, budget);
                    let status = step.status().clone();
                    let index = report.attempts.len();
                    report.attempts.push(EvolutionAttempt {
                        segment: report.segments.len(),
                        halvings,
                        expansions,
                        source,
                        report: step,
                    });
                    let reason = match status {
                        PicardStatus::Certified | PicardStatus::Empty => {
                            budget.graph.check_entries(&[
                                count,
                                report
                                    .segments
                                    .len()
                                    .checked_add(1)
                                    .ok_or(ExpressionError::SizeExhausted)?,
                            ])?;
                            budget.graph.charge(1)?;
                            report.segments.push(index);
                            break 'halving;
                        }
                        PicardStatus::Rejected(reason) => reason,
                    };
                    if !retryable(&reason) {
                        return Err(EvolutionFailure::Step(reason));
                    }
                    if matches!(reason, PicardFailure::SelfMapNotCertified)
                        && expansions < report.options.maximum_expansions
                    {
                        budget
                            .graph
                            .charge(N.checked_mul(3).ok_or(ExpressionError::SizeExhausted)?)?;
                        let previous = &report.attempts[index].report;
                        let hull = previous.proposal().candidate.hull(
                            previous.evidence().picard_image().expect("self-map stage"),
                            &mut budget.arithmetic,
                        )?;
                        candidate = hull.add(
                            &hull.sub(&hull, &mut budget.arithmetic)?,
                            &mut budget.arithmetic,
                        )?;
                        expansions += 1;
                        source = CandidateSource::PicardExpansion { attempt: index };
                    } else {
                        if halvings == report.options.maximum_halvings {
                            return Err(EvolutionFailure::HalvingLimit {
                                limit: report.options.maximum_halvings,
                            });
                        }
                        budget.graph.charge(1)?;
                        let next = start
                            .hull(&end, &mut budget.arithmetic)?
                            .midpoint(&mut budget.arithmetic)?;
                        if next.compare_singleton(&end, &mut budget.arithmetic)? != Ordering::Less {
                            return Err(EvolutionFailure::NonProgress);
                        }
                        end = next;
                        halvings += 1;
                        continue 'halving;
                    }
                }
            }
            // The final accepted contribution is already checked by Picard.
            // The horizon comparison and cancellation check still precede completion.
            match report
                .reached_time()
                .compare_singleton(&report.request.end, &mut budget.arithmetic)?
            {
                Ordering::Equal => {
                    budget.graph.charge(0)?;
                    return Ok(EvolutionStatus::Completed);
                }
                Ordering::Less => {}
                Ordering::Greater => return Err(EvolutionFailure::NonProgress),
            }
        }
    }
}

fn copy_state<const N: usize>(
    state: &BoxN<DyadicInterval, N>,
    budget: &mut EnclosureBudget,
) -> Result<BoxN<DyadicInterval, N>, EvolutionFailure> {
    budget.graph.charge(N)?;
    Ok(state.hull(&BoxN::empty()?, &mut budget.arithmetic)?)
}

// Domain overestimation can improve on a smaller interval. Resource failures,
// unavailable representations and invalid inputs are not relabeled as retries.
fn retryable(reason: &PicardFailure) -> bool {
    matches!(
        reason,
        PicardFailure::SelfMapNotCertified
            | PicardFailure::ContractionNotCertified
            | PicardFailure::EndNotCertified
            | PicardFailure::Field(FieldError::Expression(
                ExpressionError::DomainNotCertified { .. }
                    | ExpressionError::RegularityNotCertified { .. }
            ))
            | PicardFailure::Field(FieldError::Expression(ExpressionError::Enclosure {
                source: IntervalError::EmptyDomain
                    | IntervalError::ContainsZero
                    | IntervalError::Pole,
                ..
            }))
    )
}
