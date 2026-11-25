//! Automatic frame proposals composed with independent QR and trajectory proofs.
use super::{
    CoordinateField, CoordinateFieldError, CoordinateFieldTransition, CoordinateTransport,
    CoordinateTransportError, QrCoordinateField, QrFieldError,
};
use crate::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, FieldError,
    evolution::{
        EvolutionFailure, EvolutionOptions, EvolutionReport, EvolutionRequest, EvolutionStatus,
        copy_state,
    },
    taylor_flow::{
        FlowTaylorError, FlowTaylorProgram,
        evolution::{FlowTaylorEvolution, FlowTaylorEvolutionFailure, FlowTaylorEvolutionStatus},
    },
};
use hyperinterval::{DyadicInterval, IntervalError, SignSet};
use hyperlattice_interval::{BoxN, IntervalLatticeError, IntervalMatrix};
use hyperreal::Real;
use hypersolve::expression_program::ExpressionError;
use std::{cmp::Ordering, fmt};

/// Explicit finite window search contract; native step controls apply per window.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct QrEvolutionOptions {
    /// Positive singleton dyadic duration, capped by the exact remaining horizon.
    pub window: DyadicInterval,
    /// Maximum complete windows; zero permits only empty or identity requests.
    pub maximum_windows: usize,
    /// Optional trajectory Taylor order. None uses native Picard endpoints;
    /// Some(order) requires the existing whole-tube analytic Taylor contract.
    pub taylor_order: Option<usize>,
    /// Existing native search controls within each selected frame.
    pub native: EvolutionOptions,
    /// Which windows pay for a proposed frame and a second complete branch.
    #[cfg_attr(feature = "serde", serde(default))]
    pub proposals: QrProposalPolicy,
}
impl Default for QrEvolutionOptions {
    fn default() -> Self {
        Self {
            window: DyadicInterval::point(1.into(), -4).expect("finite duration"),
            maximum_windows: 256,
            taylor_order: Some(3),
            native: EvolutionOptions::default(),
            proposals: QrProposalPolicy::EveryWindow,
        }
    }
}

/// Schedule of frame proposals. Every branch is certified, so a schedule
/// changes cost and widths, never soundness.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum QrProposalPolicy {
    /// Propose a frame and compare both complete branches in every window.
    #[default]
    EveryWindow,
    /// Defer proposals after the incoming chart wins. Each consecutive
    /// retained win doubles the number of following windows that run only the
    /// incoming chart (1, 2, 4, ..., at most `maximum_interval`); a proposed win
    /// resets the schedule to every window. Zero behaves as `EveryWindow`.
    Backoff {
        /// Largest run of deferred windows between two proposals.
        maximum_interval: usize,
    },
}

/// A selection, construction, transport or original native failure.
#[derive(Clone, Debug)]
pub enum QrEvolutionFailure {
    /// Window duration is not a positive singleton.
    InvalidWindow,
    /// The configured number of complete windows was reached before the horizon.
    WindowLimit {
        /// Configured maximum.
        limit: usize,
    },
    /// Original time, native existence search, resource or cancellation failure.
    Native(EvolutionFailure),
    /// Original QR rank, undecided predicate, reconstruction or resource failure.
    Qr(QrFieldError),
    /// Original direct chart transport failure.
    Transport(CoordinateTransportError),
    /// Original trajectory coefficient or analytic-domain construction failure.
    Taylor(FlowTaylorError),
    /// Original trajectory evolution failure; no lower-order fallback.
    Flow(FlowTaylorEvolutionFailure),
    /// Original source-coordinate materialization failure.
    Coordinate(CoordinateFieldError),
}
impl fmt::Display for QrEvolutionFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "QR evolution: {self:?}")
    }
}
impl std::error::Error for QrEvolutionFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Native(e) => Some(e),
            Self::Qr(e) => Some(e),
            Self::Transport(e) => Some(e),
            Self::Coordinate(e) => Some(e),
            Self::Taylor(e) => Some(e),
            Self::Flow(e) => Some(e),
            _ => None,
        }
    }
}
impl From<EvolutionFailure> for QrEvolutionFailure {
    fn from(e: EvolutionFailure) -> Self {
        Self::Native(e)
    }
}
impl From<FieldError> for QrEvolutionFailure {
    fn from(e: FieldError) -> Self {
        Self::Native(e.into())
    }
}
impl From<ExpressionError> for QrEvolutionFailure {
    fn from(e: ExpressionError) -> Self {
        Self::Native(e.into())
    }
}
impl From<IntervalError> for QrEvolutionFailure {
    fn from(e: IntervalError) -> Self {
        Self::Native(e.into())
    }
}
impl From<IntervalLatticeError> for QrEvolutionFailure {
    fn from(e: IntervalLatticeError) -> Self {
        Self::Native(e.into())
    }
}

/// Whole-request outcome; no partial prefix is returned as a completed horizon.
#[derive(Clone, Debug)]
pub enum QrEvolutionStatus {
    /// Both frame branches, exact selection, native steps and source projections completed.
    Completed,
    /// Zero-duration identity; no ODE existence claim.
    Identity,
    /// Validated empty initial product.
    Empty,
    /// Only the explicitly exposed completed-window prefix is certified.
    Stopped(QrEvolutionFailure),
}

/// Replayable frame proposal, not an ODE or variational-flow certificate.
///
/// For the incoming chart x=c+A*y and midpoint m of its initial box, form
/// M=midpoint(J(t,m)), B=A*(I+h*M), origin=c+A*m. Entries of M are exact rational
/// midpoints of certified Jacobian enclosures; they need not equal the derivative.
/// The subsequent QR algorithm independently verifies the exact proposed B.
#[derive(Clone, Debug)]
pub struct QrFrameSelection<const N: usize> {
    chart: CoordinateField<N>,
    time: DyadicInterval,
    duration: DyadicInterval,
    initial: BoxN<DyadicInterval, N>,
    midpoint: BoxN<DyadicInterval, N>,
    jacobian: IntervalMatrix<DyadicInterval, N, N>,
    basis: Vec<Vec<Real>>,
    origin: Vec<Real>,
}
impl<const N: usize> QrFrameSelection<N> {
    /// Original incoming chart and its retained field identity.
    pub fn chart(&self) -> &CoordinateField<N> {
        &self.chart
    }
    /// Exact start of the proposed window.
    pub fn time(&self) -> &DyadicInterval {
        &self.time
    }
    /// Exact horizon-capped duration used only to propose the frame.
    pub fn duration(&self) -> &DyadicInterval {
        &self.duration
    }
    /// Incoming native enclosure in this chart.
    pub fn initial(&self) -> &BoxN<DyadicInterval, N> {
        &self.initial
    }
    /// Exact midpoint used for the Jacobian query.
    pub fn midpoint(&self) -> &BoxN<DyadicInterval, N> {
        &self.midpoint
    }
    /// Certified local Jacobian enclosure; midpoint selection does not preserve it as a set.
    pub fn jacobian(&self) -> &IntervalMatrix<DyadicInterval, N, N> {
        &self.jacobian
    }
    /// Exact proposed source-coordinate basis, later checked by QR.
    pub fn basis(&self) -> &[Vec<Real>] {
        &self.basis
    }
    /// Exact recentered source-coordinate origin.
    pub fn origin(&self) -> &[Real] {
        &self.origin
    }
}

/// One staged window attempt, retaining every successfully constructed relation.
/// A missing later stage records an interrupted attempt, never successful geometry.
#[derive(Clone, Debug)]
pub struct QrEvolutionWindow<const N: usize> {
    selection: QrFrameSelection<N>,
    frame: Option<QrCoordinateField<N>>,
    transition: Option<CoordinateFieldTransition<N>>,
    transport: Option<CoordinateTransport<N>>,
    proposed: Option<QrEvolutionBranch<N>>,
    retained: Option<QrEvolutionBranch<N>>,
    comparison: Option<QrFrameComparison<N>>,
    deferred: bool,
}
/// A fully verified frame choice; ties choose the recentered proposed frame.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QrFrameChoice {
    /// Consume the independently certified QR chart.
    Proposed,
    /// Continue in the incoming chart because its certified source diameter is smaller.
    Retained,
}
/// Exact comparison of the two completed source-coordinate endpoint enclosures.
/// This is a local maximum-coordinate-diameter criterion, not a global optimum.
#[derive(Clone, Debug)]
pub struct QrFrameComparison<const N: usize> {
    proposed_widths: [DyadicInterval; N],
    retained_widths: [DyadicInterval; N],
    proposed_diameter: DyadicInterval,
    retained_diameter: DyadicInterval,
    ordering: Ordering,
    choice: QrFrameChoice,
}
impl<const N: usize> QrFrameComparison<N> {
    /// Exact source-coordinate widths of the proposed-frame result.
    pub fn proposed_widths(&self) -> &[DyadicInterval; N] {
        &self.proposed_widths
    }
    /// Exact source-coordinate widths of the retained-frame result.
    pub fn retained_widths(&self) -> &[DyadicInterval; N] {
        &self.retained_widths
    }
    /// Largest proposed width, zero for the zero-dimensional unit product.
    pub fn proposed_diameter(&self) -> &DyadicInterval {
        &self.proposed_diameter
    }
    /// Largest retained width, zero for the zero-dimensional unit product.
    pub fn retained_diameter(&self) -> &DyadicInterval {
        &self.retained_diameter
    }
    /// Exact comparison, proposed versus retained.
    pub fn ordering(&self) -> Ordering {
        self.ordering
    }
    /// Selected branch; both complete proofs remain retained.
    pub fn choice(&self) -> QrFrameChoice {
        self.choice
    }
}
/// One source-bound native branch and all of its materialized original-coordinate output.
#[derive(Clone, Debug)]
pub struct QrEvolutionBranch<const N: usize> {
    chart: CoordinateField<N>,
    program: Option<FlowTaylorProgram<N>>,
    evolution: Option<QrWindowEvolution<N>>,
    source_segments: Vec<(BoxN<DyadicInterval, N>, BoxN<DyadicInterval, N>)>,
}
impl<const N: usize> QrEvolutionBranch<N> {
    fn new(chart: CoordinateField<N>) -> Self {
        Self {
            chart,
            program: None,
            evolution: None,
            source_segments: Vec::new(),
        }
    }
    /// Exact chart in which this branch's native proof is interpreted.
    pub fn chart(&self) -> &CoordinateField<N> {
        &self.chart
    }
    /// Prepared trajectory coefficient/source evidence, if selected and prepared.
    pub fn program(&self) -> Option<&FlowTaylorProgram<N>> {
        self.program.as_ref()
    }
    /// All native attempts, including any explicitly failed native prefix.
    pub fn evolution(&self) -> Option<&QrWindowEvolution<N>> {
        self.evolution.as_ref()
    }
    /// Source-coordinate projections, including a provisional prefix if materialization fails.
    pub fn source_segments(&self) -> &[(BoxN<DyadicInterval, N>, BoxN<DyadicInterval, N>)] {
        &self.source_segments
    }
    /// Last materialized endpoint; branch presence alone does not imply complete-window success.
    pub fn source_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.source_segments.last().map(|x| &x.1)
    }
}
impl<const N: usize> QrEvolutionWindow<N> {
    /// Exact inputs and proposal construction data.
    pub fn selection(&self) -> &QrFrameSelection<N> {
        &self.selection
    }
    /// Proposed certified QR and inverse, even if the retained chart is selected.
    pub fn frame(&self) -> Option<&QrCoordinateField<N>> {
        self.frame.as_ref()
    }
    /// Certified exact relation from incoming to proposed chart.
    pub fn transition(&self) -> Option<&CoordinateFieldTransition<N>> {
        self.transition.as_ref()
    }
    /// Retained outer projection of the exact input into the proposed chart.
    pub fn transport(&self) -> Option<&CoordinateTransport<N>> {
        self.transport.as_ref()
    }
    /// Proposed-frame native proof, including any failed stage.
    pub fn proposed(&self) -> Option<&QrEvolutionBranch<N>> {
        self.proposed.as_ref()
    }
    /// Incumbent-chart native proof, including any failed stage.
    pub fn retained(&self) -> Option<&QrEvolutionBranch<N>> {
        self.retained.as_ref()
    }
    /// Exact selection receipt, only after both branches complete. A deferred
    /// window has none.
    pub fn comparison(&self) -> Option<&QrFrameComparison<N>> {
        self.comparison.as_ref()
    }
    /// True when the proposal schedule ran only the incoming chart.
    pub fn deferred(&self) -> bool {
        self.deferred
    }
    /// Completed choice: the comparison's, or `Retained` for a deferred window
    /// whose incoming-chart branch completed.
    pub fn choice(&self) -> Option<QrFrameChoice> {
        if self.deferred {
            self.retained
                .as_ref()
                .and_then(QrEvolutionBranch::source_end_enclosure)
                .map(|_| QrFrameChoice::Retained)
        } else {
            self.comparison.as_ref().map(QrFrameComparison::choice)
        }
    }
    /// Chosen complete branch. No branch is silently chosen after another fails.
    pub fn chosen(&self) -> Option<&QrEvolutionBranch<N>> {
        match self.choice()? {
            QrFrameChoice::Proposed => self.proposed.as_ref(),
            QrFrameChoice::Retained => self.retained.as_ref(),
        }
    }
    /// Native proof from the chosen branch, after complete comparison.
    pub fn evolution(&self) -> Option<&QrWindowEvolution<N>> {
        self.chosen()?.evolution()
    }
    /// Coefficient evidence of the chosen branch, when Taylor refinement is selected.
    pub fn program(&self) -> Option<&FlowTaylorProgram<N>> {
        self.chosen()?.program()
    }
    /// Chosen source-coordinate projections in segment order.
    pub fn source_segments(&self) -> &[(BoxN<DyadicInterval, N>, BoxN<DyadicInterval, N>)] {
        self.chosen()
            .map_or(&[], QrEvolutionBranch::source_segments)
    }
    /// Chosen source endpoint; overall publication still requires completed_windows.
    pub fn source_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.chosen()?.source_end_enclosure()
    }
}

/// Native proof strategy, retaining all attempts, coefficients and remainder evidence.
#[derive(Clone, Debug)]
pub enum QrWindowEvolution<const N: usize> {
    /// Picard existence/contraction proofs with displacement-based candidate search.
    Picard(EvolutionReport<N>),
    /// Certified trajectory Taylor composition over original Picard proofs.
    Taylor(FlowTaylorEvolution<N>),
}
impl<const N: usize> QrWindowEvolution<N> {
    /// Original native request in the selected chart.
    pub fn request(&self) -> &EvolutionRequest<N> {
        match self {
            Self::Picard(r) => r.request(),
            Self::Taylor(r) => r.request(),
        }
    }
    /// Native time reached; not necessarily a whole-window success.
    pub fn reached_time(&self) -> &DyadicInterval {
        match self {
            Self::Picard(r) => r.reached_time(),
            Self::Taylor(r) => r.reached_time(),
        }
    }
    /// Selected-chart endpoint only after native whole-window success.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self {
            Self::Picard(r) => r.end_enclosure(),
            Self::Taylor(r) => r.end_enclosure(),
        }
    }
    fn record_count(&self, budget: &mut EnclosureBudget) -> Result<usize, ExpressionError> {
        match self {
            Self::Picard(r) => r
                .attempts()
                .len()
                .checked_add(r.segments().len())
                .ok_or(ExpressionError::SizeExhausted),
            Self::Taylor(r) => {
                budget.graph.charge(r.attempts().len())?;
                let mut count = r
                    .attempts()
                    .len()
                    .checked_add(r.segments().len())
                    .ok_or(ExpressionError::SizeExhausted)?;
                for a in r.attempts() {
                    count = count
                        .checked_add(a.search().attempts().len())
                        .and_then(|v| v.checked_add(a.search().segments().len()))
                        .ok_or(ExpressionError::SizeExhausted)?;
                }
                Ok(count)
            }
        }
    }
}

/// Source-bound whole-horizon run with explicit staged and completed-window prefixes.
///
/// Replay the original chart definition, request, options and policy. Serialized
/// flags/enclosures are not proof. New native boxes are outer projections of exact
/// chart images; dependency loss and interval wrapping remain explicit. No exact
/// reachable-set equality, optimal frame, width improvement or speedup is promised.
/// Shared budgets cover nested native work and aggregate retained record counts;
/// exact scalar refinement/payload, retained transformed graphs, vector capacity,
/// heap bytes and within-scalar cancellation retain their existing exclusions.
#[derive(Clone, Debug)]
pub struct QrEvolution<const N: usize> {
    chart: CoordinateField<N>,
    request: EvolutionRequest<N>,
    options: QrEvolutionOptions,
    policy: EnclosurePolicy,
    windows: Vec<QrEvolutionWindow<N>>,
    completed: usize,
    initial_projection: Option<BoxN<DyadicInterval, N>>,
    status: QrEvolutionStatus,
}
impl<const N: usize> QrEvolution<N> {
    /// Original exact source-bound chart.
    pub fn chart(&self) -> &CoordinateField<N> {
        &self.chart
    }
    /// Original input set and time horizon.
    pub fn request(&self) -> &EvolutionRequest<N> {
        &self.request
    }
    /// Immutable finite selection and native-search configuration.
    pub fn options(&self) -> &QrEvolutionOptions {
        &self.options
    }
    /// Shared enclosure policy used throughout the run.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// All frame attempts, including the final staged failure if present.
    pub fn windows(&self) -> &[QrEvolutionWindow<N>] {
        &self.windows
    }
    /// Windows whose native proofs and source materialization were published completely.
    pub fn completed_windows(&self) -> &[QrEvolutionWindow<N>] {
        &self.windows[..self.completed]
    }
    /// Whole-horizon status, separate from the completed prefix.
    pub fn status(&self) -> &QrEvolutionStatus {
        &self.status
    }
    /// End of successful empty evolution or of complete windows; a stopped native
    /// sub-prefix is separately inspectable.
    pub fn reached_time(&self) -> &DyadicInterval {
        if matches!(self.status, QrEvolutionStatus::Empty) {
            return &self.request.end;
        }
        self.completed_windows()
            .last()
            .map_or(&self.request.start, |w| {
                w.evolution().expect("completed window").reached_time()
            })
    }
    /// Source-coordinate outer enclosure after the complete-window prefix.
    /// Absent when validation or initial projection did not complete.
    pub fn prefix_source_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.completed_windows()
            .last()
            .map_or(self.initial_projection.as_ref(), |w| {
                w.source_end_enclosure()
            })
    }
    /// Endpoint only after whole-request success, never after a stopped run.
    pub fn source_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        if matches!(self.status, QrEvolutionStatus::Stopped(_)) {
            None
        } else {
            self.prefix_source_end_enclosure()
        }
    }

    fn run(
        &mut self,
        budget: &mut EnclosureBudget,
    ) -> Result<QrEvolutionStatus, QrEvolutionFailure> {
        budget.graph.charge(2)?;
        if !self.request.start.is_singleton() || !self.request.end.is_singleton() {
            return Err(EvolutionFailure::NonPointTime.into());
        }
        let order = self
            .request
            .start
            .compare_singleton(&self.request.end, &mut budget.arithmetic)?;
        if order == Ordering::Greater {
            return Err(EvolutionFailure::ReversedTime.into());
        }
        if !self.options.window.is_singleton() || self.options.window.signs() != SignSet::Positive {
            return Err(QrEvolutionFailure::InvalidWindow);
        }
        if let Some(cap) = &self.options.native.maximum_step
            && (!cap.is_singleton() || cap.signs() != SignSet::Positive)
        {
            return Err(EvolutionFailure::InvalidMaximumStep.into());
        }
        if self.options.taylor_order == Some(usize::MAX) {
            return Err(QrEvolutionFailure::Taylor(FlowTaylorError::OrderOverflow));
        }
        let projected = self
            .chart
            .source_enclosure(&self.request.initial, budget)
            .map_err(QrEvolutionFailure::Coordinate)?;
        budget.graph.charge(1)?;
        self.initial_projection = Some(projected);
        if self.request.initial.is_empty() {
            return Ok(QrEvolutionStatus::Empty);
        }
        if order == Ordering::Equal {
            return Ok(QrEvolutionStatus::Identity);
        }
        // Deferred windows left before the next proposal, and the current run length.
        let (mut wait, mut interval) = (0usize, 0usize);
        loop {
            budget.graph.charge(0)?;
            if self.completed == self.options.maximum_windows {
                return Err(QrEvolutionFailure::WindowLimit {
                    limit: self.options.maximum_windows,
                });
            }
            let time = self.reached_time().checked_pow(1, &mut budget.arithmetic)?;
            let remaining = self.request.end.sub(&time, &mut budget.arithmetic)?;
            let duration = remaining.min_image(&self.options.window, &mut budget.arithmetic)?;
            let end = time.add(&duration, &mut budget.arithmetic)?;
            if time.compare_singleton(&end, &mut budget.arithmetic)? != Ordering::Less {
                return Err(EvolutionFailure::NonProgress.into());
            }
            self.check_records(1, budget)?;
            let (chart, initial) = if let Some(last) = self.completed_windows().last() {
                (
                    last.chosen().expect("complete selection").chart().clone(),
                    copy_state(
                        last.evolution()
                            .expect("complete window")
                            .end_enclosure()
                            .expect("completed native endpoint"),
                        budget,
                    )?,
                )
            } else {
                (
                    self.chart.clone(),
                    copy_state(&self.request.initial, budget)?,
                )
            };
            let selection = select_frame(chart, time, duration, initial, budget)?;
            let deferred = wait > 0;
            budget.graph.charge(1)?;
            self.windows.push(QrEvolutionWindow {
                selection,
                frame: None,
                transition: None,
                transport: None,
                proposed: None,
                retained: None,
                comparison: None,
                deferred,
            });
            let window = self.windows.last_mut().expect("published proposal");
            window.run(
                end,
                self.options.native.clone(),
                self.options.taylor_order,
                budget,
            )?;
            if deferred {
                wait -= 1;
            } else if let QrProposalPolicy::Backoff { maximum_interval } = self.options.proposals {
                interval = match window.choice().expect("complete comparison") {
                    QrFrameChoice::Proposed => 0,
                    QrFrameChoice::Retained => {
                        interval.saturating_mul(2).max(1).min(maximum_interval)
                    }
                };
                wait = interval;
            }
            self.check_records(0, budget)?;
            budget.graph.charge(1)?;
            self.completed += 1;
            if self
                .reached_time()
                .compare_singleton(&self.request.end, &mut budget.arithmetic)?
                == Ordering::Equal
            {
                budget.graph.charge(0)?;
                return Ok(QrEvolutionStatus::Completed);
            }
        }
    }
    fn check_records(
        &self,
        extra: usize,
        budget: &mut EnclosureBudget,
    ) -> Result<(), QrEvolutionFailure> {
        let count = self
            .windows
            .len()
            .checked_add(extra)
            .ok_or(ExpressionError::SizeExhausted)?;
        let square = N.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
        let per = square
            .checked_mul(3)
            .and_then(|v| v.checked_add(N.checked_mul(10)?))
            .and_then(|v| v.checked_add(8))
            .ok_or(ExpressionError::SizeExhausted)?;
        let retained = count
            .checked_mul(per)
            .ok_or(ExpressionError::SizeExhausted)?;
        let mut native = 0usize;
        budget.graph.charge(self.windows.len())?;
        for w in &self.windows {
            for branch in [w.proposed.as_ref(), w.retained.as_ref()]
                .into_iter()
                .flatten()
            {
                if let Some(e) = &branch.evolution {
                    native = native
                        .checked_add(e.record_count(budget)?)
                        .ok_or(ExpressionError::SizeExhausted)?;
                    native = native
                        .checked_add(
                            branch
                                .source_segments
                                .len()
                                .checked_mul(
                                    N.checked_mul(2).ok_or(ExpressionError::SizeExhausted)?,
                                )
                                .ok_or(ExpressionError::SizeExhausted)?,
                        )
                        .ok_or(ExpressionError::SizeExhausted)?;
                }
            }
        }
        budget.graph.check_entries(&[retained, native])?;
        Ok(())
    }
}
impl<const N: usize> CoordinateField<N> {
    /// Integrate finite windows in automatically proposed, independently certified QR frames.
    ///
    /// Each window recomputes a Jacobian-midpoint Euler basis and exact recentering,
    /// certifies QR, directly transports the current native endpoint, and invokes
    /// existing native integration, with trajectory Taylor refinement when selected.
    /// Both the incoming and proposed charts must complete; their maximum source
    /// endpoint diameters are compared exactly, with ties choosing the proposal.
    /// [`QrProposalPolicy::Backoff`] instead runs only the incoming chart in
    /// windows that follow retained wins, skipping their QR and second branch.
    /// The selected native chart endpoint continues, and both proofs remain available.
    /// Singular or undecided frame proposals stop with
    /// their original diagnostic; there is no silent identity-frame fallback.
    /// This opt-in strategy can be slower or wider than fixed-chart integration.
    pub fn integrate_qr(
        &self,
        request: EvolutionRequest<N>,
        options: QrEvolutionOptions,
        budget: &mut EnclosureBudget,
    ) -> QrEvolution<N> {
        let mut result = QrEvolution {
            chart: self.clone(),
            request,
            options,
            policy: budget.policy(),
            windows: Vec::new(),
            completed: 0,
            initial_projection: None,
            status: QrEvolutionStatus::Identity,
        };
        result.status = match result.run(budget) {
            Ok(status) => status,
            Err(error) => QrEvolutionStatus::Stopped(error),
        };
        result
    }
}
impl<const N: usize> QrEvolutionWindow<N> {
    fn run(
        &mut self,
        end: DyadicInterval,
        options: EvolutionOptions,
        taylor_order: Option<usize>,
        budget: &mut EnclosureBudget,
    ) -> Result<(), QrEvolutionFailure> {
        let s = &self.selection;
        if self.deferred {
            let request = EvolutionRequest {
                start: s.time.checked_pow(1, &mut budget.arithmetic)?,
                end,
                initial: copy_state(&s.initial, budget)?,
            };
            self.retained = Some(QrEvolutionBranch::new(s.chart.clone()));
            return self.retained.as_mut().expect("retained branch").run(
                request,
                options,
                taylor_order,
                budget,
            );
        }
        budget.graph.charge(
            N.checked_mul(N)
                .and_then(|v| v.checked_add(N))
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        self.frame = Some(
            s.chart
                .original_field()
                .in_qr_coordinates(s.basis.clone(), s.origin.clone(), budget)
                .map_err(QrEvolutionFailure::Qr)?,
        );
        let frame = self.frame.as_ref().expect("certified frame");
        self.transition = Some(
            s.chart
                .transition_to(frame.chart(), budget)
                .map_err(QrEvolutionFailure::Transport)?,
        );
        self.transport = Some(
            self.transition
                .as_ref()
                .expect("certified relation")
                .enclose(copy_state(&s.initial, budget)?, budget)
                .map_err(QrEvolutionFailure::Transport)?,
        );
        let request = EvolutionRequest {
            start: s.time.checked_pow(1, &mut budget.arithmetic)?,
            end,
            initial: copy_state(
                self.transport
                    .as_ref()
                    .expect("certified transport")
                    .enclosure(),
                budget,
            )?,
        };
        self.proposed = Some(QrEvolutionBranch::new(frame.chart().clone()));
        self.proposed.as_mut().expect("proposed branch").run(
            request,
            options.clone(),
            taylor_order,
            budget,
        )?;
        let retained_request = EvolutionRequest {
            start: s.time.checked_pow(1, &mut budget.arithmetic)?,
            end: self
                .proposed
                .as_ref()
                .expect("complete proposed branch")
                .evolution
                .as_ref()
                .expect("complete native proof")
                .request()
                .end
                .checked_pow(1, &mut budget.arithmetic)?,
            initial: copy_state(&s.initial, budget)?,
        };
        self.retained = Some(QrEvolutionBranch::new(s.chart.clone()));
        self.retained.as_mut().expect("retained branch").run(
            retained_request,
            options,
            taylor_order,
            budget,
        )?;
        budget.graph.check_entries(&[N, N, 2])?;
        budget.graph.charge(
            N.checked_mul(2)
                .and_then(|n| n.checked_add(1))
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        let proposed_widths = self
            .proposed
            .as_ref()
            .expect("complete proposed branch")
            .source_end_enclosure()
            .expect("materialized endpoint")
            .widths(&mut budget.arithmetic)?
            .expect("nonempty endpoint");
        let retained_widths = self
            .retained
            .as_ref()
            .expect("complete retained branch")
            .source_end_enclosure()
            .expect("materialized endpoint")
            .widths(&mut budget.arithmetic)?
            .expect("nonempty endpoint");
        let mut proposed_diameter = DyadicInterval::point(0.into(), 0)?;
        let mut retained_diameter = DyadicInterval::point(0.into(), 0)?;
        for (a, b) in proposed_widths.iter().zip(&retained_widths) {
            proposed_diameter = proposed_diameter.max_image(a, &mut budget.arithmetic)?;
            retained_diameter = retained_diameter.max_image(b, &mut budget.arithmetic)?;
        }
        let ordering =
            proposed_diameter.compare_singleton(&retained_diameter, &mut budget.arithmetic)?;
        let choice = if ordering == Ordering::Greater {
            QrFrameChoice::Retained
        } else {
            QrFrameChoice::Proposed
        };
        budget.graph.charge(1)?;
        self.comparison = Some(QrFrameComparison {
            proposed_widths,
            retained_widths,
            proposed_diameter,
            retained_diameter,
            ordering,
            choice,
        });
        Ok(())
    }
}
impl<const N: usize> QrEvolutionBranch<N> {
    fn run(
        &mut self,
        request: EvolutionRequest<N>,
        options: EvolutionOptions,
        taylor_order: Option<usize>,
        budget: &mut EnclosureBudget,
    ) -> Result<(), QrEvolutionFailure> {
        if let Some(order) = taylor_order {
            self.program = Some(
                self.chart
                    .transformed_field()
                    .flow_taylor(order, budget)
                    .map_err(QrEvolutionFailure::Taylor)?,
            );
            let proof = self
                .program
                .as_ref()
                .expect("prepared coefficients")
                .integrate(request, options, budget);
            let status = proof.status().clone();
            self.evolution = Some(QrWindowEvolution::Taylor(proof));
            match status {
                FlowTaylorEvolutionStatus::Completed => {}
                FlowTaylorEvolutionStatus::Stopped(e) => return Err(QrEvolutionFailure::Flow(e)),
                _ => return Err(EvolutionFailure::NonProgress.into()),
            }
        } else {
            let proof = self
                .chart
                .transformed_field()
                .integrate_displacement(request, options, budget);
            let status = proof.status().clone();
            self.evolution = Some(QrWindowEvolution::Picard(proof));
            match status {
                EvolutionStatus::Completed => {}
                EvolutionStatus::Stopped(e) => return Err(QrEvolutionFailure::Native(e)),
                _ => return Err(EvolutionFailure::NonProgress.into()),
            }
        }
        let proof = self.evolution.as_ref().expect("retained evolution");
        let proof_records = proof.record_count(budget)?;
        let mut project = |tube: &BoxN<DyadicInterval, N>,
                           end: &BoxN<DyadicInterval, N>|
         -> Result<(), QrEvolutionFailure> {
            let count = self
                .source_segments
                .len()
                .checked_add(1)
                .ok_or(ExpressionError::SizeExhausted)?;
            let lanes = count
                .checked_mul(N)
                .and_then(|v| v.checked_mul(2))
                .ok_or(ExpressionError::SizeExhausted)?;
            budget.graph.check_entries(&[count, lanes, proof_records])?;
            let tube = self
                .chart
                .source_enclosure(tube, budget)
                .map_err(QrEvolutionFailure::Coordinate)?;
            let end = self
                .chart
                .source_enclosure(end, budget)
                .map_err(QrEvolutionFailure::Coordinate)?;
            budget.graph.charge(1)?;
            self.source_segments.push((tube, end));
            Ok(())
        };
        match proof {
            QrWindowEvolution::Picard(r) => {
                for segment in r.segments() {
                    project(
                        segment.tube().expect("certified tube"),
                        segment.end_enclosure().expect("certified endpoint"),
                    )?;
                }
            }
            QrWindowEvolution::Taylor(r) => {
                for segment in r.segments() {
                    project(segment.tube(), segment.end_enclosure())?;
                }
            }
        }
        budget.graph.charge(0)?;
        Ok(())
    }
}

fn midpoint_real(
    value: &DyadicInterval,
    budget: &mut EnclosureBudget,
) -> Result<Real, QrEvolutionFailure> {
    let point = value.midpoint(&mut budget.arithmetic)?;
    let (lower, _) = point
        .to_rational(&mut budget.arithmetic)?
        .into_bounds()
        .expect("nonempty midpoint");
    Ok(Real::from(lower))
}
fn select_frame<const N: usize>(
    chart: CoordinateField<N>,
    time: DyadicInterval,
    duration: DyadicInterval,
    initial: BoxN<DyadicInterval, N>,
    budget: &mut EnclosureBudget,
) -> Result<QrFrameSelection<N>, QrEvolutionFailure> {
    let square = N.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
    let cubic = square
        .checked_mul(N)
        .ok_or(ExpressionError::SizeExhausted)?;
    budget
        .graph
        .check_entries(&[square, square, square, N, N, N])?;
    budget.graph.charge(N)?;
    let mut midpoints = Vec::with_capacity(N);
    for lane in initial.lanes().expect("nonempty initial product") {
        midpoints.push(lane.midpoint(&mut budget.arithmetic)?);
    }
    let midpoint = BoxN::new(midpoints.try_into().expect("fixed dimension"))?;
    let jacobian = chart
        .transformed_field()
        .jacobian(&time, &midpoint, budget)?;
    budget.graph.charge(square)?;
    let h = midpoint_real(&duration, budget)?;
    let mut local = Vec::with_capacity(N);
    for (i, row) in jacobian
        .rows()
        .expect("nonempty Jacobian")
        .iter()
        .enumerate()
    {
        let mut values = Vec::with_capacity(N);
        for (j, value) in row.iter().enumerate() {
            values.push(Real::from(u8::from(i == j)) + h.clone() * midpoint_real(value, budget)?);
        }
        local.push(values);
    }
    budget.graph.charge(N)?;
    let mut center = Vec::with_capacity(N);
    for lane in midpoint.lanes().expect("nonempty midpoint") {
        center.push(midpoint_real(lane, budget)?);
    }
    // These exact proposal sums do not implement QR or certify a flow map.
    // Existing QR and transition kernels independently establish usable geometry.
    budget.graph.charge(
        cubic
            .checked_add(square)
            .ok_or(ExpressionError::SizeExhausted)?,
    )?;
    let change = chart.transformation().change();
    let mut basis = Vec::with_capacity(N);
    let mut origin = Vec::with_capacity(N);
    for (i, a) in change.to_source.iter().enumerate() {
        let mut row = Vec::with_capacity(N);
        for j in 0..N {
            row.push(
                a.iter()
                    .zip(&local)
                    .fold(Real::zero(), |sum, (a, b)| sum + a * &b[j]),
            );
        }
        basis.push(row);
        origin.push(
            a.iter()
                .zip(&center)
                .fold(change.origin[i].clone(), |sum, (a, c)| sum + a * c),
        );
    }
    budget.graph.charge(1)?;
    Ok(QrFrameSelection {
        chart,
        time,
        duration,
        initial,
        midpoint,
        jacobian,
        basis,
        origin,
    })
}
