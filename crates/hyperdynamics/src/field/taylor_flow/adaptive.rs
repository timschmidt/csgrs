//! Bounded order/step selection over existing source-bound trajectory proofs.
use super::*;
use crate::field::evolution::{
    EvolutionFailure, EvolutionOptions, EvolutionReport, EvolutionRequest, EvolutionStatus,
    copy_state,
};
use core::cmp::Ordering;

/// Finite candidate orders, local truncation limits and native proposal controls.
#[derive(Clone, Debug)]
pub struct AdaptiveTaylorOptions<const N: usize> {
    /// First order tried on each certified proposal, inclusive; zero is permitted.
    pub minimum_order: usize,
    /// Last order tried, inclusive. Coefficients are prepared lazily once per run;
    /// each higher order continues the stages of the highest order already prepared.
    pub maximum_order: usize,
    /// Nonnegative exact singleton bounds on absolute endpoint remainder contributions.
    /// These bound neither global error nor uncertainty in the initial state.
    pub remainder_limits: [DyadicInterval; N],
    /// Maximum additional exact halvings after every candidate order fails quality.
    /// Native existence-search halvings are separately bounded in `native`.
    pub maximum_quality_halvings: u32,
    /// Native existence search controls and outer accepted-segment limit.
    /// After acceptance, duration may double, capped by maximum_step and the horizon.
    pub native: EvolutionOptions,
}
impl<const N: usize> Default for AdaptiveTaylorOptions<N> {
    fn default() -> Self {
        Self {
            minimum_order: 0,
            maximum_order: 6,
            remainder_limits: std::array::from_fn(|_| {
                DyadicInterval::new(1.into(), 1.into(), -16).expect("finite singleton")
            }),
            maximum_quality_halvings: 16,
            native: EvolutionOptions::default(),
        }
    }
}
/// Invalid selection contract, actual quality-search limit, or original proof failure.
#[derive(Clone, Debug)]
pub enum AdaptiveTaylorFailure {
    /// Minimum exceeds maximum, or maximum+1 cannot represent its remainder degree.
    InvalidOrderRange,
    /// The named coordinate's limit is empty, nonsingleton or negative.
    InvalidRemainderLimit {
        /// Zero-based state coordinate.
        coordinate: usize,
    },
    /// Every configured order failed quality and the configured halving limit was reached.
    /// The mathematical ODE may still have solutions; this is not undecidability.
    QualityHalvingLimit {
        /// Configured maximum additional quality halvings.
        limit: u32,
    },
    /// Original native search, resource or cancellation diagnostic.
    Search(EvolutionFailure),
    /// Original coefficient, analyticity, native arithmetic or resource diagnostic.
    Taylor(FlowTaylorError),
}
impl fmt::Display for AdaptiveTaylorFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "adaptive Taylor: {self:?}")
    }
}
impl std::error::Error for AdaptiveTaylorFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Search(e) => Some(e),
            Self::Taylor(e) => Some(e),
            _ => None,
        }
    }
}
impl From<EvolutionFailure> for AdaptiveTaylorFailure {
    fn from(e: EvolutionFailure) -> Self {
        Self::Search(e)
    }
}
impl From<ExpressionError> for AdaptiveTaylorFailure {
    fn from(e: ExpressionError) -> Self {
        Self::Search(e.into())
    }
}
impl From<IntervalError> for AdaptiveTaylorFailure {
    fn from(e: IntervalError) -> Self {
        Self::Search(e.into())
    }
}
impl From<FlowTaylorError> for AdaptiveTaylorFailure {
    fn from(e: FlowTaylorError) -> Self {
        Self::Taylor(e)
    }
}
/// Whole-request status, separate from the retained certified prefix.
#[derive(Clone, Debug)]
pub enum AdaptiveTaylorStatus {
    /// All segments and their local remainder bounds reached the exact requested end.
    Completed,
    /// Zero-duration identity; no ODE existence claim.
    Identity,
    /// Validated empty initial product.
    Empty,
    /// Only the separately exposed prefix is complete.
    Stopped(AdaptiveTaylorFailure),
}
/// Source and finite selection configuration; construction alone proves no trajectory.
#[derive(Clone, Debug)]
pub struct AdaptiveTaylorField<const N: usize> {
    field: ExpressionField<N>,
    options: AdaptiveTaylorOptions<N>,
}
impl<const N: usize> ExpressionField<N> {
    /// Select ascending Taylor orders and bounded step sizes during integration.
    /// Configuration is validated by `integrate`, before empty/identity shortcuts.
    /// Original fixed-order and native algorithms are unchanged.
    pub fn adaptive_taylor(&self, options: AdaptiveTaylorOptions<N>) -> AdaptiveTaylorField<N> {
        AdaptiveTaylorField {
            field: self.clone(),
            options,
        }
    }
}
/// One refinement of an already certified Picard proposal, retaining failed quality.
#[derive(Clone, Debug)]
pub struct AdaptiveTaylorRefinement<const N: usize> {
    program: usize,
    evidence: FlowTaylorEvidence<N>,
    status: FlowTaylorStatus,
    magnitudes: Option<[DyadicInterval; N]>,
    quality: Option<bool>,
}
impl<const N: usize> AdaptiveTaylorRefinement<N> {
    /// Index into the report's immutable prepared program list.
    pub fn program_index(&self) -> usize {
        self.program
    }
    /// Original complete/partial Taylor stages, including all remainder terms.
    pub fn evidence(&self) -> &FlowTaylorEvidence<N> {
        &self.evidence
    }
    /// Original mathematical refinement status; certification alone does not meet quality.
    pub fn status(&self) -> &FlowTaylorStatus {
        &self.status
    }
    /// Enclosures of absolute endpoint remainder contributions, after complete evaluation.
    pub fn remainder_magnitudes(&self) -> Option<&[DyadicInterval; N]> {
        self.magnitudes.as_ref()
    }
    /// None if quality was not fully checked, otherwise the exact all-coordinate decision.
    pub fn meets_limits(&self) -> Option<bool> {
        self.quality
    }
}
/// Native existence search followed by its ordered, bounded Taylor refinements.
#[derive(Clone, Debug)]
pub struct AdaptiveTaylorAttempt<const N: usize> {
    quality_halvings: u32,
    search: EvolutionReport<N>,
    refinements: Vec<AdaptiveTaylorRefinement<N>>,
}
impl<const N: usize> AdaptiveTaylorAttempt<N> {
    /// Additional quality-driven halvings for this segment's initial proposal.
    pub fn quality_halvings(&self) -> u32 {
        self.quality_halvings
    }
    /// Original one-segment search, including every rejected native proposal.
    pub fn search(&self) -> &EvolutionReport<N> {
        &self.search
    }
    /// All invoked refinements in ascending order, including failed quality or proof stages.
    pub fn refinements(&self) -> &[AdaptiveTaylorRefinement<N>] {
        &self.refinements
    }
}
/// Borrowed certified segment selected only after all local quality comparisons.
#[derive(Clone, Copy, Debug)]
pub struct AdaptiveTaylorSegment<'a, const N: usize> {
    attempt: &'a AdaptiveTaylorAttempt<N>,
    refinement: &'a AdaptiveTaylorRefinement<N>,
    program: &'a FlowTaylorProgram<N>,
}
impl<'a, const N: usize> AdaptiveTaylorSegment<'a, N> {
    /// Exact source-bound normalized coefficient program selected for this segment.
    pub fn program(&self) -> &'a FlowTaylorProgram<N> {
        self.program
    }
    /// Original existence/uniqueness proof; it is not rerun for candidate orders.
    pub fn picard(&self) -> &'a PicardReport<N> {
        self.attempt.search.segment(0).expect("accepted search")
    }
    /// Complete Taylor evidence and passed local-quality decision.
    pub fn refinement(&self) -> &'a AdaptiveTaylorRefinement<N> {
        self.refinement
    }
    /// Certified complete trajectory tube for this segment.
    pub fn tube(&self) -> &'a BoxN<DyadicInterval, N> {
        self.refinement
            .evidence
            .tube
            .as_ref()
            .expect("accepted tube")
    }
    /// Endpoint consumed verbatim by the next native proposal search.
    pub fn end_enclosure(&self) -> &'a BoxN<DyadicInterval, N> {
        self.refinement
            .evidence
            .end
            .as_ref()
            .expect("accepted endpoint")
    }
}
/// Immutable adaptive run retaining source, choices, proofs and qualified partial history.
///
/// Replay the original field/request/options; serialized flags are not certificates.
/// One shared budget covers preparation, every search/refinement and publication.
/// Aggregate record checks bound checked structural stages, not all retained proof
/// graphs, scalar payloads, vector capacity or total heap. Partial history may survive
/// exhaustion; only `segments` denotes accepted prefix, and only completed statuses
/// expose `end_enclosure`. Local remainder limits do not bound global trajectory error.
#[derive(Clone, Debug)]
pub struct AdaptiveTaylorEvolution<const N: usize> {
    controller: AdaptiveTaylorField<N>,
    request: EvolutionRequest<N>,
    policy: EnclosurePolicy,
    programs: Vec<FlowTaylorProgram<N>>,
    attempts: Vec<AdaptiveTaylorAttempt<N>>,
    segments: Vec<(usize, usize)>,
    status: AdaptiveTaylorStatus,
}
impl<const N: usize> AdaptiveTaylorEvolution<N> {
    /// Original source and finite selection options.
    pub fn controller(&self) -> &AdaptiveTaylorField<N> {
        &self.controller
    }
    /// Exact requested start/end and initial set.
    pub fn request(&self) -> &EvolutionRequest<N> {
        &self.request
    }
    /// Shared policy; nested calls never reset its counters.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// Each lazily prepared order, retained once and shared by all refinements.
    pub fn programs(&self) -> &[FlowTaylorProgram<N>] {
        &self.programs
    }
    /// Every native search and refinement, including rejected quality/proofs.
    pub fn attempts(&self) -> &[AdaptiveTaylorAttempt<N>] {
        &self.attempts
    }
    /// Whole-request completion, distinct from a certified prefix.
    pub fn status(&self) -> &AdaptiveTaylorStatus {
        &self.status
    }
    /// All accepted contiguous segments with complete remainder and quality evidence.
    pub fn segments(&self) -> impl ExactSizeIterator<Item = AdaptiveTaylorSegment<'_, N>> {
        self.segments.iter().map(|&(a, r)| {
            let attempt = &self.attempts[a];
            let refinement = &attempt.refinements[r];
            AdaptiveTaylorSegment {
                attempt,
                refinement,
                program: &self.programs[refinement.program],
            }
        })
    }
    /// Certified prefix endpoint, present after an accepted segment even on later failure.
    pub fn prefix_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.segments
            .last()
            .and_then(|&(a, r)| self.attempts[a].refinements[r].evidence.end.as_ref())
    }
    /// Reached time of the accepted prefix, or supplied start before any acceptance.
    /// Validated empty success reaches the requested end without nonempty trajectories.
    pub fn reached_time(&self) -> &DyadicInterval {
        if matches!(self.status, AdaptiveTaylorStatus::Empty) {
            return &self.request.end;
        }
        self.segments.last().map_or(&self.request.start, |&(a, _)| {
            &self.attempts[a]
                .search
                .segment(0)
                .expect("accepted search")
                .proposal()
                .end
        })
    }
    /// Endpoint only after whole-horizon, validated empty or identity success.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            AdaptiveTaylorStatus::Completed => self.prefix_end_enclosure(),
            AdaptiveTaylorStatus::Identity | AdaptiveTaylorStatus::Empty => {
                Some(&self.request.initial)
            }
            AdaptiveTaylorStatus::Stopped(_) => None,
        }
    }
    fn check_records(
        &self,
        native_records: usize,
        refinements: usize,
        extra: usize,
        budget: &mut EnclosureBudget,
    ) -> Result<(), ExpressionError> {
        budget.graph.check_entries(&[
            self.programs.len(),
            self.attempts.len(),
            self.segments.len(),
            native_records,
            refinements,
            extra,
        ])
    }
}
impl<const N: usize> AdaptiveTaylorField<N> {
    /// Exact original field and its ordered time/state interpretation.
    pub fn field(&self) -> &ExpressionField<N> {
        &self.field
    }
    /// Original finite order/step/quality configuration.
    pub fn options(&self) -> &AdaptiveTaylorOptions<N> {
        &self.options
    }
    /// Integrate using first satisfactory ascending order, then bounded exact halving.
    /// Accepted durations may double for the next proposal, capped exactly. A native
    /// or Taylor mathematical/resource failure stops; it is never disguised as a
    /// quality miss or weaker-method success. No minimum-cost/order optimality is claimed.
    pub fn integrate(
        &self,
        request: EvolutionRequest<N>,
        budget: &mut EnclosureBudget,
    ) -> AdaptiveTaylorEvolution<N> {
        let mut report = AdaptiveTaylorEvolution {
            controller: self.clone(),
            request,
            policy: budget.policy(),
            programs: Vec::new(),
            attempts: Vec::new(),
            segments: Vec::new(),
            status: AdaptiveTaylorStatus::Identity,
        };
        report.status = match self.evolve(&mut report, budget) {
            Ok(status) => status,
            Err(error) => AdaptiveTaylorStatus::Stopped(error),
        };
        report
    }
    fn evolve(
        &self,
        report: &mut AdaptiveTaylorEvolution<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<AdaptiveTaylorStatus, AdaptiveTaylorFailure> {
        budget.graph.charge(1)?;
        if self.options.minimum_order > self.options.maximum_order
            || self.options.maximum_order.checked_add(1).is_none()
        {
            return Err(AdaptiveTaylorFailure::InvalidOrderRange);
        }
        for (coordinate, limit) in self.options.remainder_limits.iter().enumerate() {
            budget.graph.charge(1)?;
            if !limit.is_singleton()
                || !matches!(
                    limit.signs(),
                    hyperinterval::SignSet::Zero | hyperinterval::SignSet::Positive
                )
            {
                return Err(AdaptiveTaylorFailure::InvalidRemainderLimit { coordinate });
            }
        }
        let mut cap = self.options.native.maximum_step.clone();
        let mut quality_halvings = 0;
        let mut native_records = 0usize;
        let mut refinements = 0usize;
        loop {
            budget.graph.charge(0)?;
            if !report.attempts.is_empty()
                && report.segments.len() == self.options.native.maximum_steps
            {
                return Err(EvolutionFailure::StepLimit {
                    limit: self.options.native.maximum_steps,
                }
                .into());
            }
            report.check_records(native_records, refinements, 1, budget)?;
            let start = report
                .reached_time()
                .checked_pow(1, &mut budget.arithmetic)?;
            let end = report.request.end.checked_pow(1, &mut budget.arithmetic)?;
            let initial = copy_state(
                report
                    .prefix_end_enclosure()
                    .unwrap_or(&report.request.initial),
                budget,
            )?;
            budget.graph.charge(1)?;
            let mut options = self.options.native.clone();
            options.maximum_steps = usize::from(self.options.native.maximum_steps != 0);
            options.maximum_step = cap.clone();
            // The first native search validates times/cap before empty/identity shortcuts.
            let search = self.field.integrate(
                EvolutionRequest {
                    start,
                    end,
                    initial,
                },
                options,
                budget,
            );
            let records = search.attempts().len().checked_add(search.segments().len());
            let a = report.attempts.len();
            report.attempts.push(AdaptiveTaylorAttempt {
                quality_halvings,
                search,
                refinements: Vec::new(),
            });
            native_records = records
                .and_then(|n| native_records.checked_add(n))
                .ok_or(ExpressionError::SizeExhausted)?;
            report.check_records(native_records, refinements, 0, budget)?;
            match report.attempts[a].search.status() {
                EvolutionStatus::Empty => {
                    budget.graph.charge(0)?;
                    return Ok(AdaptiveTaylorStatus::Empty);
                }
                EvolutionStatus::Identity => {
                    budget.graph.charge(0)?;
                    return Ok(AdaptiveTaylorStatus::Identity);
                }
                EvolutionStatus::Completed
                | EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 }) => {}
                EvolutionStatus::Stopped(e) => return Err(e.clone().into()),
            }
            let mut accepted = false;
            for order in self.options.minimum_order..=self.options.maximum_order {
                budget.graph.charge(
                    report
                        .programs
                        .len()
                        .checked_add(1)
                        .ok_or(ExpressionError::SizeExhausted)?,
                )?;
                let program = match report.programs.iter().position(|p| p.order() == order) {
                    Some(i) => i,
                    None => {
                        report.check_records(native_records, refinements, 1, budget)?;
                        // Orders are prepared as one ascending run from the minimum,
                        // so the last program is `order - 1`: continue its stages.
                        debug_assert!(
                            report
                                .programs
                                .last()
                                .is_none_or(|p| p.order() + 1 == order)
                        );
                        let prefix = report.programs.last().map_or(&[][..], |p| p.stages());
                        let prepared = self.field.continue_flow_taylor(prefix, order, budget)?;
                        budget.graph.charge(1)?;
                        let i = report.programs.len();
                        report.programs.push(prepared);
                        i
                    }
                };
                report.check_records(native_records, refinements, 1, budget)?;
                budget.graph.charge(1)?;
                let attempt = &mut report.attempts[a];
                let mut evidence = FlowTaylorEvidence::default();
                let status = match report.programs[program].verify(
                    attempt.search.segment(0).expect("accepted native prefix"),
                    &mut evidence,
                    budget,
                ) {
                    Ok(status) => status,
                    Err(e) => FlowTaylorStatus::Rejected(e),
                };
                let r = attempt.refinements.len();
                attempt.refinements.push(AdaptiveTaylorRefinement {
                    program,
                    evidence,
                    status,
                    magnitudes: None,
                    quality: None,
                });
                refinements = refinements
                    .checked_add(1)
                    .ok_or(ExpressionError::SizeExhausted)?;
                report.check_records(native_records, refinements, 0, budget)?;
                let refinement = &mut report.attempts[a].refinements[r];
                if let FlowTaylorStatus::Rejected(e) = &refinement.status {
                    return Err(e.clone().into());
                }
                let remainder = refinement
                    .evidence
                    .end_terms
                    .last()
                    .expect("positive-duration Taylor remainder")
                    .lanes()
                    .expect("nonempty remainder");
                budget.graph.check_entries(&[N, N])?;
                budget.graph.charge(N)?;
                let mut magnitudes = Vec::with_capacity(N);
                let mut quality = true;
                let zero = DyadicInterval::new(0.into(), 0.into(), 0)?;
                for (value, limit) in remainder.iter().zip(&self.options.remainder_limits) {
                    budget.graph.charge(1)?;
                    let magnitude = value.abs(&mut budget.arithmetic)?;
                    let allowed = zero.hull(limit, &mut budget.arithmetic)?;
                    quality &= magnitude.subset_of(&allowed, &mut budget.arithmetic)?;
                    magnitudes.push(magnitude);
                }
                budget.graph.charge(1)?;
                refinement.magnitudes =
                    Some(magnitudes.try_into().expect("fixed coordinate count"));
                refinement.quality = Some(quality);
                if quality {
                    report.check_records(native_records, refinements, 1, budget)?;
                    budget.graph.charge(1)?;
                    report.segments.push((a, r));
                    accepted = true;
                    break;
                }
            }
            let duration = report.attempts[a]
                .search
                .segment(0)
                .expect("accepted native prefix")
                .evidence()
                .duration()
                .expect("positive duration");
            if accepted {
                match report
                    .reached_time()
                    .compare_singleton(&report.request.end, &mut budget.arithmetic)?
                {
                    Ordering::Equal => {
                        budget.graph.charge(0)?;
                        return Ok(AdaptiveTaylorStatus::Completed);
                    }
                    Ordering::Greater => return Err(EvolutionFailure::NonProgress.into()),
                    Ordering::Less => {}
                }
                let two = DyadicInterval::new(2.into(), 2.into(), 0)?;
                let grown = duration.mul(&two, &mut budget.arithmetic)?;
                cap = Some(match &self.options.native.maximum_step {
                    Some(maximum) => grown.min_image(maximum, &mut budget.arithmetic)?,
                    None => grown,
                });
                quality_halvings = 0;
            } else {
                if quality_halvings == self.options.maximum_quality_halvings {
                    return Err(AdaptiveTaylorFailure::QualityHalvingLimit {
                        limit: self.options.maximum_quality_halvings,
                    });
                }
                let half = DyadicInterval::new(1.into(), 1.into(), -1)?;
                let smaller = duration.mul(&half, &mut budget.arithmetic)?;
                if smaller.compare_singleton(duration, &mut budget.arithmetic)? != Ordering::Less
                    || smaller.signs() != hyperinterval::SignSet::Positive
                {
                    return Err(EvolutionFailure::NonProgress.into());
                }
                cap = Some(smaller);
                quality_halvings += 1;
            }
        }
    }
}
