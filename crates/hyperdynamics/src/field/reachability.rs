//! Finite endpoint covers over exact initial-state partitions.
//!
//! On success, the cover C satisfies R ⊆ C ⊆ R + [-ε, ε]^N, where R is the
//! reachable set at the exact requested end time. Every positive-horizon leaf
//! has a complete existence proof and an endpoint box of infinity-diameter at
//! most ε. Its nonempty initial cell supplies a reachable point in that box;
//! hence every point of the box is within ε of R. Exact closed subdivision
//! preserves coverage. Identity and empty requests have exact set semantics.
//! Each native certified prefix is outward-regridded, with checked inclusion,
//! before continuation. Candidate padding uses displacement beyond the initial
//! width, but the existing Picard verifier remains the sole step authority.
//!
//! This sufficient accuracy test is conservative. Finite search need not
//! complete, including at zero ε or a rounding floor. Stopped reports expose
//! attempts, never a whole-request end cover. Replay definitions, requests and
//! policies; serialized success flags are not certificates.
use super::{evolution::*, *};
use core::cmp::Ordering;
mod transport;
pub use transport::{ReachabilityEvolution, ReachabilityStep};

/// Original evolution and requested absolute infinity-norm thickening bound.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ReachabilityRequest<const N: usize> {
    /// Exact time horizon and initial set.
    pub evolution: EvolutionRequest<N>,
    /// Nonnegative singleton, in the caller's state-coordinate units.
    pub epsilon: DyadicInterval,
}

/// Finite deterministic cover search, sharing one enclosing arithmetic budget.
///
/// These limits count retained cells, complete integration invocations and
/// refinement depth. They do not bound total heap bytes or the scalar refinement
/// excluded by `EnclosureBudget`. Every nested invocation spends the same counters.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct ReachabilityPolicy {
    /// Existing exact integration search; no alternate ODE authority is used.
    pub evolution: EvolutionOptions,
    /// Maximum retained partition cells, including internal nodes and the root.
    pub maximum_cells: usize,
    /// Maximum whole-cell evolution attempts, including rejected ones. Each attempt
    /// performs at most `max(1, evolution.maximum_steps)` native prefix searches.
    pub maximum_evolutions: usize,
    /// Maximum initial-box bisections along one root-to-leaf path.
    pub maximum_depth: u32,
    /// Maximum strict step-cap halvings per cell; children inherit the refined cap.
    pub maximum_time_refinements: u32,
}
impl Default for ReachabilityPolicy {
    fn default() -> Self {
        Self {
            evolution: EvolutionOptions::default(),
            maximum_cells: 4095,
            maximum_evolutions: 8192,
            maximum_depth: 32,
            maximum_time_refinements: 16,
        }
    }
}

/// One retained cell in an exact closed initial-state partition.
#[derive(Clone, Debug)]
pub struct ReachabilityCell<const N: usize> {
    initial: BoxN<DyadicInterval, N>,
    parent: Option<usize>,
    depth: u32,
    children: Option<[usize; 2]>,
    split_axis: Option<usize>,
    last_attempt: Option<usize>,
    diameter: Option<DyadicInterval>,
    time_refinements: u32,
    step_cap: Option<DyadicInterval>,
}
impl<const N: usize> ReachabilityCell<N> {
    /// Exact initial cell, not an endpoint approximation.
    pub fn initial(&self) -> &BoxN<DyadicInterval, N> {
        &self.initial
    }
    /// Parent cell index; only the root has none.
    pub fn parent(&self) -> Option<usize> {
        self.parent
    }
    /// Number of initial-box splits from the root.
    pub fn depth(&self) -> u32 {
        self.depth
    }
    /// Lower then upper child, sharing a closed midpoint facet.
    pub fn children(&self) -> Option<[usize; 2]> {
        self.children
    }
    /// Coordinate split by the exact Cartesian partition authority.
    pub fn split_axis(&self) -> Option<usize> {
        self.split_axis
    }
    /// Last integration attempt for this cell, if one was invoked.
    pub fn last_attempt(&self) -> Option<usize> {
        self.last_attempt
    }
    /// Exact diameter of the last completed endpoint box, when materialized.
    /// This alone does not certify the requested accuracy or full cover.
    pub fn end_diameter(&self) -> Option<&DyadicInterval> {
        self.diameter.as_ref()
    }
    /// Number of strict time-cap refinements performed for this cell.
    pub fn time_refinements(&self) -> u32 {
        self.time_refinements
    }
}

/// One whole-cell search, retaining every invocation of the native authority.
#[derive(Clone, Debug)]
pub struct ReachabilityAttempt<const N: usize> {
    cell: usize,
    evolution: ReachabilityEvolution<N>,
}
impl<const N: usize> ReachabilityAttempt<N> {
    /// Cell whose initial set is bound by this evolution report.
    pub fn cell(&self) -> usize {
        self.cell
    }
    /// Full ordered prefix proofs, outward transfers, source and exact horizon.
    pub fn evolution(&self) -> &ReachabilityEvolution<N> {
        &self.evolution
    }
}

/// Why the full requested cover was not certified.
#[derive(Clone, Debug)]
pub enum ReachabilityFailure {
    /// Epsilon is empty, nonsingleton or negative.
    InvalidEpsilon,
    /// The next cell allocation would exceed the declared capacity.
    CellCapacity {
        /// Configured bound.
        limit: usize,
        /// Required retained cells.
        required: usize,
    },
    /// The aggregate integration invocation bound has been reached.
    EvolutionLimit {
        /// Configured bound.
        limit: usize,
    },
    /// A cell reached its initial-subdivision depth bound.
    DepthLimit {
        /// Cell still needing refinement.
        cell: usize,
        /// Configured bound.
        limit: u32,
    },
    /// An indivisible initial state reached its time-refinement bound.
    TimeRefinementLimit {
        /// Cell still needing refinement.
        cell: usize,
        /// Configured bound.
        limit: u32,
    },
    /// A nested evolution failed without a supported cover retry.
    Evolution {
        /// Retained attempt index.
        attempt: usize,
        /// Original typed reason.
        reason: EvolutionFailure,
    },
    /// Required strict refinement could not be established.
    NonProgress,
    /// Arithmetic, structural resources, cancellation or preparation failed.
    Preparation(EvolutionFailure),
}
impl fmt::Display for ReachabilityFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "reachability: {self:?}")
    }
}
impl std::error::Error for ReachabilityFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Preparation(e) | Self::Evolution { reason: e, .. } => Some(e),
            _ => None,
        }
    }
}
impl From<EvolutionFailure> for ReachabilityFailure {
    fn from(e: EvolutionFailure) -> Self {
        Self::Preparation(e)
    }
}
impl From<ExpressionError> for ReachabilityFailure {
    fn from(e: ExpressionError) -> Self {
        Self::Preparation(e.into())
    }
}
impl From<IntervalError> for ReachabilityFailure {
    fn from(e: IntervalError) -> Self {
        Self::Preparation(e.into())
    }
}
impl From<IntervalLatticeError> for ReachabilityFailure {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Preparation(e.into())
    }
}

/// Whole-request coverage, distinct from any certified individual cell.
#[derive(Clone, Debug)]
pub enum ReachabilityStatus {
    /// Every leaf reaches the requested horizon and satisfies the thickening bound.
    Completed,
    /// Exact zero-horizon identity, without an ODE existence claim.
    Identity,
    /// Successful empty reachable set, after input validation.
    Empty,
    /// Retained attempts do not establish the full requested end cover.
    Stopped(ReachabilityFailure),
}

/// Immutable source-bound reachability evidence. Rebuild source and rerun for replay.
#[derive(Clone, Debug)]
pub struct ReachabilityReport<const N: usize> {
    identity: FieldIdentity,
    request: ReachabilityRequest<N>,
    policy: ReachabilityPolicy,
    enclosure_policy: EnclosurePolicy,
    cells: Vec<ReachabilityCell<N>>,
    attempts: Vec<ReachabilityAttempt<N>>,
    accepted: Vec<usize>,
    status: ReachabilityStatus,
}
impl<const N: usize> ReachabilityReport<N> {
    /// Exact retained source identity.
    pub fn identity(&self) -> &FieldIdentity {
        &self.identity
    }
    /// Original time, initial set and accuracy contract.
    pub fn request(&self) -> &ReachabilityRequest<N> {
        &self.request
    }
    /// Recorded finite search policy.
    pub fn policy(&self) -> &ReachabilityPolicy {
        &self.policy
    }
    /// Shared enclosure precision and resource configuration.
    pub fn enclosure_policy(&self) -> EnclosurePolicy {
        self.enclosure_policy
    }
    /// Exact partition tree, including unfinished cells.
    pub fn cells(&self) -> &[ReachabilityCell<N>] {
        &self.cells
    }
    /// Every whole-cell evolution attempt, in deterministic execution order.
    pub fn attempts(&self) -> &[ReachabilityAttempt<N>] {
        &self.attempts
    }
    /// Whole-request outcome.
    pub fn status(&self) -> &ReachabilityStatus {
        &self.status
    }
    /// Completed endpoint cover only; never exposes a partial union as full coverage.
    pub fn end_cover(&self) -> Option<EndCover<'_, N>> {
        match self.status {
            ReachabilityStatus::Stopped(_) => None,
            _ => Some(EndCover { report: self }),
        }
    }
}

/// Borrowed certified finite union at the exact requested horizon.
///
/// Each box may include unreachable points, bounded by the stated epsilon
/// thickening in infinity norm. The union is not asserted to be an exact reachable
/// set. Empty covers contain zero boxes; zero-dimensional unit covers contain one.
#[derive(Clone, Copy, Debug)]
pub struct EndCover<'a, const N: usize> {
    report: &'a ReachabilityReport<N>,
}
impl<'a, const N: usize> EndCover<'a, N> {
    /// Certified endpoint boxes in depth-first, lower-child-first order.
    pub fn boxes(&self) -> impl ExactSizeIterator<Item = &'a BoxN<DyadicInterval, N>> + '_ {
        self.report.accepted.iter().map(|&i| {
            let cell = &self.report.cells[i];
            self.report.attempts[cell.last_attempt.expect("accepted cell")]
                .evolution
                .end_enclosure()
                .expect("accepted horizon")
        })
    }
    /// Explicit accuracy bound supplied with the original request.
    pub fn epsilon(&self) -> &'a DyadicInterval {
        &self.report.request.epsilon
    }
    /// Exact common end time; all completed leaf reports bind this value.
    pub fn end_time(&self) -> &'a DyadicInterval {
        &self.report.request.evolution.end
    }
}

impl<const N: usize> ExpressionField<N> {
    /// Certify a finite endpoint cover by bounded exact initial-state subdivision.
    ///
    /// The widest initial coordinate is bisected, with first-axis tie breaking.
    /// Cells of initial diameter at most half epsilon first refine the exact
    /// time cap. After their time-refinement limit, nonpoint cells split again.
    /// Children inherit the refined cap; point cells can only refine time. Every nested
    /// integration uses the same budget, retains all rejections, and checks its
    /// final contribution before success. No finite completion bound is promised.
    pub fn reach(
        &self,
        request: ReachabilityRequest<N>,
        policy: ReachabilityPolicy,
        budget: &mut EnclosureBudget,
    ) -> ReachabilityReport<N> {
        let mut report = ReachabilityReport {
            identity: self.identity(),
            request,
            policy,
            enclosure_policy: budget.policy(),
            cells: Vec::new(),
            attempts: Vec::new(),
            accepted: Vec::new(),
            status: ReachabilityStatus::Empty,
        };
        report.status = match self.cover(&mut report, budget) {
            Ok(s) => s,
            Err(e) => ReachabilityStatus::Stopped(e),
        };
        report
    }

    fn cover(
        &self,
        r: &mut ReachabilityReport<N>,
        b: &mut EnclosureBudget,
    ) -> Result<ReachabilityStatus, ReachabilityFailure> {
        b.graph.charge(1)?;
        if !r.request.epsilon.is_singleton()
            || r.request.epsilon.signs() == hyperinterval::SignSet::Negative
        {
            return Err(ReachabilityFailure::InvalidEpsilon);
        }
        if r.policy.maximum_cells == 0 {
            return Err(ReachabilityFailure::CellCapacity {
                limit: 0,
                required: 1,
            });
        }
        b.graph.check_entries(&[1])?;
        let initial = copy_state(&r.request.evolution.initial, b)?;
        r.cells.push(new_cell(initial, None, 0));
        let mut pending = vec![0];
        while let Some(cell) = pending.pop() {
            b.graph.charge(0)?;
            if r.attempts.len() == r.policy.maximum_evolutions {
                return Err(ReachabilityFailure::EvolutionLimit {
                    limit: r.policy.maximum_evolutions,
                });
            }
            let count = r
                .attempts
                .len()
                .checked_add(1)
                .ok_or(ExpressionError::SizeExhausted)?;
            b.graph
                .check_entries(&[r.cells.len(), count, r.accepted.len(), pending.len()])?;
            b.graph.charge(1)?;
            let request = EvolutionRequest {
                start: r
                    .request
                    .evolution
                    .start
                    .checked_pow(1, &mut b.arithmetic)?,
                end: r.request.evolution.end.checked_pow(1, &mut b.arithmetic)?,
                initial: copy_state(&r.cells[cell].initial, b)?,
            };
            let source_cap = r.cells[cell].step_cap.as_ref().or(r
                .policy
                .evolution
                .maximum_step
                .as_ref());
            let options = EvolutionOptions {
                maximum_step: source_cap
                    .map(|v| v.checked_pow(1, &mut b.arithmetic))
                    .transpose()?,
                maximum_steps: r.policy.evolution.maximum_steps,
                maximum_halvings: r.policy.evolution.maximum_halvings,
                maximum_expansions: r.policy.evolution.maximum_expansions,
            };
            let evolution = transport::integrate_cell(self, request, options, b);
            let status = evolution.status().clone();
            let attempt = r.attempts.len();
            r.attempts.push(ReachabilityAttempt { cell, evolution });
            r.cells[cell].last_attempt = Some(attempt);
            r.cells[cell].diameter = None;
            let mut step_limit = false;
            match status {
                EvolutionStatus::Empty => {
                    b.graph.charge(0)?;
                    return Ok(ReachabilityStatus::Empty);
                }
                EvolutionStatus::Identity => {
                    b.graph
                        .check_entries(&[r.cells.len(), r.attempts.len(), 1])?;
                    b.graph.charge(1)?;
                    r.accepted.push(cell);
                    return Ok(ReachabilityStatus::Identity);
                }
                EvolutionStatus::Completed => {
                    let end = r.attempts[attempt]
                        .evolution
                        .end_enclosure()
                        .expect("complete evolution");
                    let diameter = diameter(end, b)?;
                    let fits = diameter.compare_singleton(&r.request.epsilon, &mut b.arithmetic)?
                        != Ordering::Greater;
                    r.cells[cell].diameter = Some(diameter);
                    if fits {
                        b.graph.check_entries(&[
                            r.cells.len(),
                            r.attempts.len(),
                            r.accepted
                                .len()
                                .checked_add(1)
                                .ok_or(ExpressionError::SizeExhausted)?,
                            pending.len(),
                        ])?;
                        b.graph.charge(1)?;
                        r.accepted.push(cell);
                        continue;
                    }
                }
                EvolutionStatus::Stopped(reason) => {
                    step_limit = matches!(reason, EvolutionFailure::StepLimit { .. });
                    if !matches!(
                        reason,
                        EvolutionFailure::HalvingLimit { .. } | EvolutionFailure::StepLimit { .. }
                    ) {
                        return Err(ReachabilityFailure::Evolution { attempt, reason });
                    }
                }
            }
            b.graph.charge(N)?;
            let axis = r.cells[cell].initial.widest_axis(&mut b.arithmetic)?;
            let split = axis.filter(|&axis| {
                !r.cells[cell].initial.lanes().expect("nonempty cell")[axis].is_singleton()
            });
            let refine_time = if !step_limit
                && r.cells[cell].time_refinements < r.policy.maximum_time_refinements
            {
                match split {
                    Some(axis) => {
                        let width = r.cells[cell].initial.lanes().expect("nonempty cell")[axis]
                            .width(&mut b.arithmetic)?;
                        width
                            .add(&width, &mut b.arithmetic)?
                            .compare_singleton(&r.request.epsilon, &mut b.arithmetic)?
                            != Ordering::Greater
                    }
                    None => true,
                }
            } else {
                false
            };
            if let Some(axis) = split.filter(|_| !refine_time) {
                let depth = r.cells[cell].depth;
                if depth == r.policy.maximum_depth {
                    return Err(ReachabilityFailure::DepthLimit {
                        cell,
                        limit: r.policy.maximum_depth,
                    });
                }
                let count = r
                    .cells
                    .len()
                    .checked_add(2)
                    .ok_or(ExpressionError::SizeExhausted)?;
                if count > r.policy.maximum_cells {
                    return Err(ReachabilityFailure::CellCapacity {
                        limit: r.policy.maximum_cells,
                        required: count,
                    });
                }
                b.graph.check_entries(&[
                    count,
                    r.attempts.len(),
                    r.accepted.len(),
                    pending
                        .len()
                        .checked_add(2)
                        .ok_or(ExpressionError::SizeExhausted)?,
                ])?;
                b.graph.charge(
                    N.checked_mul(2)
                        .and_then(|n| n.checked_add(2))
                        .ok_or(ExpressionError::SizeExhausted)?,
                )?;
                let [left, right] = r.cells[cell]
                    .initial
                    .bisect_axis(axis, &mut b.arithmetic)?
                    .ok_or(ReachabilityFailure::NonProgress)?;
                let indices = [r.cells.len(), r.cells.len() + 1];
                let mut left = new_cell(left, Some(cell), depth + 1);
                let mut right = new_cell(right, Some(cell), depth + 1);
                // Halving a cap that already exhausted the segment count cannot
                // help. Children restart at this cell's last completed cap,
                // retaining the failed finer attempt as evidence.
                let inherited = if step_limit {
                    r.attempts
                        .iter()
                        .rev()
                        .find(|a| {
                            a.cell == cell
                                && matches!(a.evolution.status(), EvolutionStatus::Completed)
                        })
                        .and_then(|a| a.evolution.segments().next())
                        .and_then(|s| s.evidence().duration())
                        .or(r.policy.evolution.maximum_step.as_ref())
                } else {
                    r.cells[cell].step_cap.as_ref()
                };
                left.step_cap = inherited
                    .map(|cap| cap.checked_pow(1, &mut b.arithmetic))
                    .transpose()?;
                right.step_cap = inherited
                    .map(|cap| cap.checked_pow(1, &mut b.arithmetic))
                    .transpose()?;
                r.cells.push(left);
                r.cells.push(right);
                r.cells[cell].children = Some(indices);
                r.cells[cell].split_axis = Some(axis);
                pending.push(indices[1]);
                pending.push(indices[0]);
            } else {
                if step_limit {
                    let EvolutionStatus::Stopped(reason) = r.attempts[attempt].evolution.status()
                    else {
                        unreachable!("recorded step limit")
                    };
                    return Err(ReachabilityFailure::Evolution {
                        attempt,
                        reason: reason.clone(),
                    });
                }
                if r.cells[cell].time_refinements == r.policy.maximum_time_refinements {
                    return Err(ReachabilityFailure::TimeRefinementLimit {
                        cell,
                        limit: r.policy.maximum_time_refinements,
                    });
                }
                let duration = r
                    .request
                    .evolution
                    .end
                    .sub(&r.request.evolution.start, &mut b.arithmetic)?;
                let cap = match r.attempts[attempt]
                    .evolution
                    .segments()
                    .next()
                    .and_then(|step| step.evidence().duration())
                    .or(r.attempts[attempt]
                        .evolution
                        .options()
                        .maximum_step
                        .as_ref())
                {
                    Some(cap) => duration.min_image(cap, &mut b.arithmetic)?,
                    None => duration,
                };
                let half = cap
                    .hull(&DyadicInterval::point(0.into(), 0)?, &mut b.arithmetic)?
                    .midpoint(&mut b.arithmetic)?;
                if half.signs() != hyperinterval::SignSet::Positive
                    || half.compare_singleton(&cap, &mut b.arithmetic)? != Ordering::Less
                {
                    return Err(ReachabilityFailure::NonProgress);
                }
                b.graph.charge(1)?;
                r.cells[cell].time_refinements += 1;
                r.cells[cell].step_cap = Some(half);
                pending.push(cell);
            }
        }
        b.graph.charge(0)?;
        Ok(ReachabilityStatus::Completed)
    }
}
fn new_cell<const N: usize>(
    initial: BoxN<DyadicInterval, N>,
    parent: Option<usize>,
    depth: u32,
) -> ReachabilityCell<N> {
    ReachabilityCell {
        initial,
        parent,
        depth,
        children: None,
        split_axis: None,
        last_attempt: None,
        diameter: None,
        time_refinements: 0,
        step_cap: None,
    }
}
fn diameter<const N: usize>(
    state: &BoxN<DyadicInterval, N>,
    b: &mut EnclosureBudget,
) -> Result<DyadicInterval, ReachabilityFailure> {
    b.graph.charge(N)?;
    let mut maximum = DyadicInterval::point(0.into(), 0)?;
    for lane in state.lanes().ok_or(ReachabilityFailure::NonProgress)? {
        let width = lane.width(&mut b.arithmetic)?;
        if width.compare_singleton(&maximum, &mut b.arithmetic)? == Ordering::Greater {
            maximum = width
        }
    }
    Ok(maximum)
}
