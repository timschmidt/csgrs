//! Initial-state derivatives enclosed from existing certified Picard evidence.
//!
//! The derivative `S(t) = D_x0 φ(t, t0, x0)` solves the variational equation
//! `S' = D_x f(t, φ) S` with `S(t0) = I`. On each certified segment of duration
//! `h`, the segment's interval Jacobian `A` encloses `D_x f` at every time and
//! every state of the certified candidate, so it bounds every factor of the
//! time-ordered Peano–Baker series. The k-th term is an integral over a simplex
//! of volume `h^k/k!` whose integrand lies in the interval product `A^k`, so it
//! lies in `(h·A)^k/k!`. Independent interval products enclose it; no
//! commutativity is assumed and no time-varying matrix is replaced by a midpoint.
//!
//! The segment's certified contraction bound `q = h·sup‖A‖∞ < 1` bounds the
//! omitted norm after `p` terms by `q^(p+1)/(p+1)!/(1-q)`, which every entry
//! receives as a signed radius. Outward native arithmetic encloses rounding, and
//! products are regridded outward to the policy grid so bit sizes stay bounded
//! over long horizons. At an intermediate time `t ≤ h` the k-th term is the
//! same box scaled by `(t/h)^k ∈ [0, 1]`, so hulling each nonconstant term with
//! zero covers the whole segment. Local transition matrices multiply prior
//! global sensitivities on the left, following the chain rule along each
//! trajectory.
//!
//! Every supported evolution reduces to that one derivation. Taylor and adaptive
//! Taylor segments retain the Picard proof that each refinement consumed; its
//! Jacobian covers the candidate tube, which contains every trajectory the
//! tighter Taylor enclosure describes. Chart evolutions integrate
//! `y' = B f(t, c + A y)` with exact `A`, a strictly proved inverse `B`, and
//! retained native Picard proofs in `y`. The chain rule gives
//! `D_x0 x(t) = A_k · P(t) · B_0`, where `P` composes the chart-coordinate
//! segment derivatives and, at each change of QR frame, the exact linear part
//! `B_new A_old` of the certified chart transition. Exact chart coefficients
//! are enclosed on the policy grid, and the source-coordinate conjugation never
//! feeds back into `P`, so it widens each published matrix only once.
//!
//! Matrices are Cartesian outer enclosures, not exact reachable derivative sets.
//! A source's stopped prefix remains a prefix. Reports are never deserialized as
//! certificates: replay the field, the evolution request and options, and these
//! options. References: Teschl, *Ordinary Differential Equations and Dynamical
//! Systems*, on dependence on initial conditions; Baake and Schlägel, *The
//! Peano–Baker series*, arXiv:1011.1775, equations (3)–(5) and Theorem 1. The
//! interval-product inclusion and geometric tail above are this module's own
//! derivation.
use super::{
    evolution::*,
    picard::{PicardReport, PicardStatus},
    *,
};
use hyperinterval::SignSet;
use hyperreal::Real;

/// Finite expansion effort, separate from precision and shared resource limits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SensitivityOptions {
    /// Maximum nonconstant terms per segment. Zero keeps identity plus a
    /// certified tail. An exactly zero interval product ends the series early
    /// with a zero tail.
    pub terms: usize,
}
impl Default for SensitivityOptions {
    fn default() -> Self {
        Self { terms: 8 }
    }
}

/// Source rejection, or failure to finish this separate sensitivity derivation.
///
/// `F` is the source evolution's own stop reason.
#[derive(Clone, Debug)]
pub enum SensitivityError<F = EvolutionFailure> {
    /// The source stopped before certifying any positive-duration segment.
    Source(F),
    /// Shared structural, native arithmetic or cancellation failure.
    Field(FieldError),
}
impl<F: fmt::Debug> fmt::Display for SensitivityError<F> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "flow sensitivity: {self:?}")
    }
}
impl<F: std::error::Error + 'static> std::error::Error for SensitivityError<F> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Source(e) => Some(e),
            Self::Field(e) => Some(e),
        }
    }
}

type Matrix<const N: usize> = IntervalMatrix<DyadicInterval, N, N>;

/// Evidence for one certified native segment, in source segment order.
///
/// Local matrices are in the segment's own coordinates: the state for Picard
/// and Taylor sources, and the chart coordinates `y` for chart and QR sources.
/// Global matrices are always derivatives of the source state with respect to
/// the original request's source initial state.
#[derive(Clone, Debug)]
pub struct SensitivityStep<const N: usize> {
    terms: Vec<Matrix<N>>,
    tail: DyadicInterval,
    local_end: Matrix<N>,
    local_tube: Matrix<N>,
    global_end: Matrix<N>,
    global_tube: Matrix<N>,
}
impl<const N: usize> SensitivityStep<N> {
    /// Ordered term enclosures, starting with identity and ending with a zero
    /// product when one was proved. Terms need not be point matrices.
    pub fn terms(&self) -> &[Matrix<N>] {
        &self.terms
    }
    /// Interval from zero to an upper bound on the omitted series norm.
    pub fn tail_norm_bound(&self) -> &DyadicInterval {
        &self.tail
    }
    /// Endpoint derivative with respect to this segment's initial state, in
    /// the segment's own coordinates.
    pub fn local_end(&self) -> &Matrix<N> {
        &self.local_end
    }
    /// Derivative at every time of this segment, with respect to its initial
    /// state, in the segment's own coordinates.
    pub fn local_tube(&self) -> &Matrix<N> {
        &self.local_tube
    }
    /// Endpoint source-state derivative with respect to the original request's
    /// source initial state.
    pub fn global_end(&self) -> &Matrix<N> {
        &self.global_end
    }
    /// Source-state derivative at every time of this segment, with respect to
    /// the original request's source initial state.
    pub fn global_tube(&self) -> &Matrix<N> {
        &self.global_tube
    }
}

#[derive(Clone, Copy, Debug)]
enum Horizon {
    Completed,
    Initial,
    Stopped,
}

/// Immutable sensitivity evidence that shares its authoritative evolution.
///
/// `S` is the source evolution: [`EvolutionReport`] by default, or a Taylor,
/// adaptive Taylor, fixed-chart or QR evolution when those features are enabled.
/// Read the source's status and reached time through [`Self::source`].
#[derive(Clone, Debug)]
pub struct FlowSensitivity<const N: usize, S = EvolutionReport<N>> {
    source: Arc<S>,
    options: SensitivityOptions,
    policy: EnclosurePolicy,
    initial: Matrix<N>,
    horizon: Horizon,
    steps: Vec<SensitivityStep<N>>,
}
impl<const N: usize, S> FlowSensitivity<N, S> {
    /// Shared source, including its status, reached time, policy and proofs.
    pub fn source(&self) -> &Arc<S> {
        &self.source
    }
    /// Recorded series effort.
    pub fn options(&self) -> SensitivityOptions {
        self.options
    }
    /// Policy used for this derivation, which may differ from the source's.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// Evidence in the same order as the source's certified native segments.
    pub fn steps(&self) -> &[SensitivityStep<N>] {
        &self.steps
    }
    /// Derivative at the end of the certified prefix, without a whole-horizon claim.
    pub fn prefix_end_matrix(&self) -> Option<&Matrix<N>> {
        self.steps.last().map(|s| &s.global_end)
    }
    /// Whole-horizon derivative enclosure, only after the source completed.
    /// An empty initial set yields the empty matrix set; a zero-duration
    /// request yields identity and makes no ODE existence claim. A stopped
    /// source gives `None`.
    pub fn end_matrix(&self) -> Option<&Matrix<N>> {
        match self.horizon {
            Horizon::Completed => Some(self.prefix_end_matrix().unwrap_or(&self.initial)),
            Horizon::Initial => Some(&self.initial),
            Horizon::Stopped => None,
        }
    }
}

/// Native Picard proofs interpreted in one exact chart.
struct Window<'a, const N: usize> {
    /// Exact linear part of the affine map from the previous window's chart.
    entry: Option<&'a [Vec<Real>]>,
    /// Exact matrix taking this window's coordinates to source coordinates.
    chart: Option<&'a [Vec<Real>]>,
    segments: Box<dyn ExactSizeIterator<Item = &'a PicardReport<N>> + 'a>,
}

enum Status<'a, F> {
    Completed,
    Identity,
    Empty,
    Stopped(&'a F),
}

trait Source<const N: usize> {
    type Failure: Clone;
    fn horizon(&self) -> Status<'_, Self::Failure>;
    /// Exact inverse of the request's chart, from source to chart coordinates.
    fn inverse(&self) -> Option<&[Vec<Real>]> {
        None
    }
    /// Certified windows in order; a stopped source yields only its certified prefix.
    fn windows(&self) -> impl Iterator<Item = Window<'_, N>>;
}

fn native<'a, const N: usize>(
    segments: impl ExactSizeIterator<Item = &'a PicardReport<N>> + 'a,
) -> Window<'a, N> {
    Window {
        entry: None,
        chart: None,
        segments: Box::new(segments),
    }
}

impl<const N: usize> Source<N> for EvolutionReport<N> {
    type Failure = EvolutionFailure;
    fn horizon(&self) -> Status<'_, EvolutionFailure> {
        match self.status() {
            EvolutionStatus::Completed => Status::Completed,
            EvolutionStatus::Identity => Status::Identity,
            EvolutionStatus::Empty => Status::Empty,
            EvolutionStatus::Stopped(e) => Status::Stopped(e),
        }
    }
    fn windows(&self) -> impl Iterator<Item = Window<'_, N>> {
        core::iter::once(native(self.segments()))
    }
}

impl<const N: usize> EvolutionReport<N> {
    /// Enclose the initial-state derivative across every certified source segment.
    ///
    /// Reuses the source's Picard Jacobians and contraction bounds without
    /// evaluating its field again. A stopped source with no certified segment
    /// is an error. A stopped source with a prefix keeps its status, and the
    /// result then has no whole-horizon matrix. Any failure returns an error
    /// without partial output; the caller's `Arc` stays available for a retry
    /// with a larger budget.
    ///
    /// Charges count new retained matrix entries, term and step records and
    /// tail scalars, plus a scratch allowance of sixteen matrices and 32
    /// scalars. Source storage is shared and excluded. Work charges cover
    /// matrix cell operations, and native arithmetic is charged to the same
    /// budget. These are logical entries, not heap-byte reservations: scalar
    /// payload copies, allocator capacity, stack storage and single-operation
    /// cancellation keep the exclusions of `EnclosureBudget`. Cancellation is
    /// polled at every structural charge and before publication; it cannot
    /// interrupt one matrix product.
    pub fn sensitivity(
        self: &Arc<Self>,
        options: SensitivityOptions,
        budget: &mut EnclosureBudget,
    ) -> Result<FlowSensitivity<N>, SensitivityError> {
        run(self, options, budget)
    }
}

#[cfg(feature = "taylor-evolution")]
mod taylor {
    use super::*;
    use crate::field::taylor_flow::evolution::{
        FlowTaylorEvolution, FlowTaylorEvolutionFailure, FlowTaylorEvolutionStatus,
    };

    impl<const N: usize> Source<N> for FlowTaylorEvolution<N> {
        type Failure = FlowTaylorEvolutionFailure;
        fn horizon(&self) -> Status<'_, FlowTaylorEvolutionFailure> {
            match self.status() {
                FlowTaylorEvolutionStatus::Completed => Status::Completed,
                FlowTaylorEvolutionStatus::Identity => Status::Identity,
                FlowTaylorEvolutionStatus::Empty => Status::Empty,
                FlowTaylorEvolutionStatus::Stopped(e) => Status::Stopped(e),
            }
        }
        fn windows(&self) -> impl Iterator<Item = Window<'_, N>> {
            core::iter::once(native(self.segments().map(|s| s.picard())))
        }
    }

    impl<const N: usize> FlowTaylorEvolution<N> {
        /// Enclose the initial-state derivative across every certified Taylor segment.
        ///
        /// Each segment's derivative comes from the Picard proof its Taylor
        /// refinement consumed, so neither the field nor its Taylor
        /// coefficients are evaluated again. Prefix, retry and accounting rules
        /// are those of [`EvolutionReport::sensitivity`].
        pub fn sensitivity(
            self: &Arc<Self>,
            options: SensitivityOptions,
            budget: &mut EnclosureBudget,
        ) -> Result<FlowSensitivity<N, Self>, SensitivityError<FlowTaylorEvolutionFailure>>
        {
            run(self, options, budget)
        }
    }
}

#[cfg(feature = "adaptive-taylor")]
mod adaptive {
    use super::*;
    use crate::field::taylor_flow::adaptive::{
        AdaptiveTaylorEvolution, AdaptiveTaylorFailure, AdaptiveTaylorStatus,
    };

    impl<const N: usize> Source<N> for AdaptiveTaylorEvolution<N> {
        type Failure = AdaptiveTaylorFailure;
        fn horizon(&self) -> Status<'_, AdaptiveTaylorFailure> {
            match self.status() {
                AdaptiveTaylorStatus::Completed => Status::Completed,
                AdaptiveTaylorStatus::Identity => Status::Identity,
                AdaptiveTaylorStatus::Empty => Status::Empty,
                AdaptiveTaylorStatus::Stopped(e) => Status::Stopped(e),
            }
        }
        fn windows(&self) -> impl Iterator<Item = Window<'_, N>> {
            core::iter::once(native(self.segments().map(|s| s.picard())))
        }
    }

    impl<const N: usize> AdaptiveTaylorEvolution<N> {
        /// Enclose the initial-state derivative across every accepted adaptive segment.
        ///
        /// Uses each accepted segment's original Picard proof, whatever order
        /// and duration the controller selected. Prefix, retry and accounting
        /// rules are those of [`EvolutionReport::sensitivity`].
        pub fn sensitivity(
            self: &Arc<Self>,
            options: SensitivityOptions,
            budget: &mut EnclosureBudget,
        ) -> Result<FlowSensitivity<N, Self>, SensitivityError<AdaptiveTaylorFailure>> {
            run(self, options, budget)
        }
    }
}

#[cfg(feature = "coordinates")]
mod chart {
    use super::*;
    use crate::field::coordinates::CoordinateEvolution;

    impl<const N: usize> Source<N> for CoordinateEvolution<N> {
        type Failure = EvolutionFailure;
        // The derivative needs the native proofs and the exact chart, not the
        // Cartesian source projections, so it follows the native status.
        fn horizon(&self) -> Status<'_, EvolutionFailure> {
            self.native().horizon()
        }
        fn inverse(&self) -> Option<&[Vec<Real>]> {
            Some(&self.chart().transformation().change().from_source)
        }
        fn windows(&self) -> impl Iterator<Item = Window<'_, N>> {
            core::iter::once(Window {
                entry: None,
                chart: Some(&self.chart().transformation().change().to_source),
                segments: Box::new(self.native().segments()),
            })
        }
    }

    impl<const N: usize> CoordinateEvolution<N> {
        /// Enclose the source-coordinate initial-state derivative of a fixed-chart run.
        ///
        /// Derives chart-coordinate segment derivatives from the retained native
        /// proofs and conjugates them by the exact chart. The result follows the
        /// native status: a completed native run has a whole-horizon matrix even
        /// when the Cartesian source projection later failed, because that
        /// projection is not part of the derivative argument. Prefix, retry and
        /// accounting rules are those of [`EvolutionReport::sensitivity`].
        pub fn sensitivity(
            self: &Arc<Self>,
            options: SensitivityOptions,
            budget: &mut EnclosureBudget,
        ) -> Result<FlowSensitivity<N, Self>, SensitivityError> {
            run(self, options, budget)
        }
    }
}

#[cfg(feature = "qr-evolution")]
mod qr {
    use super::*;
    use crate::field::coordinates::qr_evolution::{
        QrEvolution, QrEvolutionFailure, QrEvolutionStatus, QrFrameChoice, QrWindowEvolution,
    };

    impl<const N: usize> Source<N> for QrEvolution<N> {
        type Failure = QrEvolutionFailure;
        fn horizon(&self) -> Status<'_, QrEvolutionFailure> {
            match self.status() {
                QrEvolutionStatus::Completed => Status::Completed,
                QrEvolutionStatus::Identity => Status::Identity,
                QrEvolutionStatus::Empty => Status::Empty,
                QrEvolutionStatus::Stopped(e) => Status::Stopped(e),
            }
        }
        fn inverse(&self) -> Option<&[Vec<Real>]> {
            Some(&self.chart().transformation().change().from_source)
        }
        fn windows(&self) -> impl Iterator<Item = Window<'_, N>> {
            self.completed_windows().iter().map(|window| {
                let branch = window.chosen().expect("completed window choice");
                let entry = match window.comparison().expect("completed window").choice() {
                    QrFrameChoice::Proposed => Some(
                        window
                            .transition()
                            .expect("completed proposal transition")
                            .relation()
                            .matrix(),
                    ),
                    QrFrameChoice::Retained => None,
                };
                let segments: Box<dyn ExactSizeIterator<Item = &PicardReport<N>>> =
                    match branch.evolution().expect("completed native window") {
                        QrWindowEvolution::Picard(r) => Box::new(r.segments()),
                        QrWindowEvolution::Taylor(r) => Box::new(r.segments().map(|s| s.picard())),
                    };
                Window {
                    entry,
                    chart: Some(&branch.chart().transformation().change().to_source),
                    segments,
                }
            })
        }
    }

    impl<const N: usize> QrEvolution<N> {
        /// Enclose the source-coordinate initial-state derivative across completed QR windows.
        ///
        /// Each window uses its chosen branch's native proofs. A change to the
        /// proposed frame multiplies by the exact linear part of the certified
        /// transition, so the frame choice never enters as a Cartesian box. A
        /// stopped run keeps only its completed windows; its partial window is
        /// excluded. Prefix, retry and accounting rules are otherwise those of
        /// [`EvolutionReport::sensitivity`].
        pub fn sensitivity(
            self: &Arc<Self>,
            options: SensitivityOptions,
            budget: &mut EnclosureBudget,
        ) -> Result<FlowSensitivity<N, Self>, SensitivityError<QrEvolutionFailure>> {
            run(self, options, budget)
        }
    }
}

fn run<const N: usize, S: Source<N>>(
    source: &Arc<S>,
    options: SensitivityOptions,
    budget: &mut EnclosureBudget,
) -> Result<FlowSensitivity<N, S>, SensitivityError<S::Failure>> {
    if let Status::Stopped(reason) = source.horizon()
        && source.windows().all(|w| w.segments.len() == 0)
    {
        return Err(SensitivityError::Source(reason.clone()));
    }
    derive(Arc::clone(source), options, budget).map_err(SensitivityError::Field)
}

fn size(value: Option<usize>) -> Result<usize, FieldError> {
    value.ok_or(ExpressionError::SizeExhausted.into())
}

// Apply a fallible scalar map to every entry, in row-major order, stopping at
// the first error. Matrix scaling stays private here: the public owner method
// was measured and rejected on latency (docs/research/matrix-scale-reconsideration.md).
fn map<const N: usize>(
    matrix: &Matrix<N>,
    budget: &mut DyadicBudget,
    mut f: impl FnMut(&DyadicInterval, &mut DyadicBudget) -> Result<DyadicInterval, IntervalError>,
) -> Result<Matrix<N>, FieldError> {
    let Some(rows) = matrix.rows() else {
        return Ok(Matrix::empty()?);
    };
    let mut failure = None;
    let out = core::array::from_fn(|r| {
        core::array::from_fn(|c| {
            if failure.is_some() {
                return DyadicInterval::empty();
            }
            f(&rows[r][c], budget).unwrap_or_else(|e| {
                failure = Some(e);
                DyadicInterval::empty()
            })
        })
    });
    match failure {
        Some(e) => Err(e.into()),
        None => Ok(Matrix::new(out)?),
    }
}

fn regrid<const N: usize>(
    matrix: &Matrix<N>,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Matrix<N>, FieldError> {
    map(matrix, budget, |x, b| x.regrid(precision, b))
}

fn product<const N: usize>(
    left: &Matrix<N>,
    right: &Matrix<N>,
    precision: i32,
    budget: &mut DyadicBudget,
) -> Result<Matrix<N>, FieldError> {
    regrid(&left.mul_matrix(right, budget)?, precision, budget)
}

fn constant<const N: usize>(
    entry: impl Fn(usize, usize) -> DyadicInterval,
) -> Result<Matrix<N>, FieldError> {
    Ok(Matrix::new(core::array::from_fn(|r| {
        core::array::from_fn(|c| entry(r, c))
    }))?)
}

// Outward policy-grid enclosure of an exact square matrix validated by its chart.
fn exact<const N: usize>(
    rows: &[Vec<Real>],
    budget: &mut EnclosureBudget,
) -> Result<Matrix<N>, FieldError> {
    budget.graph.charge(size(N.checked_mul(N))?)?;
    let mut failure = None;
    let precision = budget.policy.precision;
    let arithmetic = &mut budget.arithmetic;
    let out: [[DyadicInterval; N]; N] = core::array::from_fn(|r| {
        core::array::from_fn(|c| {
            if failure.is_some() {
                return DyadicInterval::empty();
            }
            DyadicInterval::from_real(&rows[r][c], precision, arithmetic).unwrap_or_else(|e| {
                failure = Some(e);
                DyadicInterval::empty()
            })
        })
    });
    match failure {
        Some(e) => Err(e.into()),
        None => Ok(Matrix::new(out)?),
    }
}

fn derive<const N: usize, S: Source<N>>(
    source: Arc<S>,
    options: SensitivityOptions,
    budget: &mut EnclosureBudget,
) -> Result<FlowSensitivity<N, S>, FieldError> {
    let cells = size(N.checked_mul(N))?;
    let scratch = size(cells.checked_mul(16).and_then(|n| n.checked_add(32)))?;
    let term_storage = size(cells.checked_add(1))?;
    let step_storage = size(cells.checked_mul(4).and_then(|n| n.checked_add(2)))?;
    let product_work = size(cells.checked_mul(N))?;
    let pair_work = size(
        product_work
            .checked_add(cells)
            .and_then(|n| n.checked_mul(2)),
    )?;
    let mut retained = 0;
    retain(budget, &mut retained, term_storage, scratch)?;
    let zero = DyadicInterval::point(0.into(), 0)?;
    let one = DyadicInterval::point(1.into(), 0)?;
    let identity = constant::<N>(|r, c| if r == c { one.clone() } else { zero.clone() })?;
    let zero_matrix = constant::<N>(|_, _| zero.clone())?;
    let (horizon, initial) = match source.horizon() {
        Status::Completed => (Horizon::Completed, identity.clone()),
        Status::Identity => (Horizon::Initial, identity.clone()),
        Status::Empty => (Horizon::Initial, Matrix::empty()?),
        Status::Stopped(_) => (Horizon::Stopped, identity.clone()),
    };
    let precision = budget.policy.precision;
    let inverse = source
        .inverse()
        .map(|b| exact::<N>(b, budget))
        .transpose()?;
    let count = source.windows().map(|w| w.segments.len()).sum();
    let mut steps: Vec<SensitivityStep<N>> = Vec::with_capacity(count);
    // Chart-coordinate derivative with respect to the request's chart state.
    let mut chain = initial.clone();
    for window in source.windows() {
        budget.graph.charge(0)?;
        if let Some(rows) = window.entry {
            let entry = exact::<N>(rows, budget)?;
            budget
                .graph
                .charge(size(product_work.checked_add(cells))?)?;
            chain = product(&entry, &chain, precision, &mut budget.arithmetic)?;
        }
        let chart = window.chart.map(|a| exact::<N>(a, budget)).transpose()?;
        // Source-coordinate derivative A·P·B0 of a chart-coordinate chain P.
        let conjugate =
            |a: &Matrix<N>, p: &Matrix<N>, b: &Matrix<N>, budget: &mut EnclosureBudget| {
                budget.graph.charge(pair_work)?;
                let left = product(a, p, precision, &mut budget.arithmetic)?;
                product(&left, b, precision, &mut budget.arithmetic)
            };
        for segment in window.segments {
            retain(budget, &mut retained, step_storage, scratch)?;
            retain(budget, &mut retained, term_storage, scratch)?;
            if matches!(segment.status(), PicardStatus::Empty) {
                // No state reaches this segment, so its derivative set is empty.
                let empty = Matrix::empty()?;
                chain = empty.clone();
                steps.push(SensitivityStep {
                    terms: vec![identity.clone()],
                    tail: zero.clone(),
                    local_end: empty.clone(),
                    local_tube: empty.clone(),
                    global_end: empty.clone(),
                    global_tube: empty,
                });
                continue;
            }
            let evidence = segment.evidence();
            let (Some(h), Some(jacobian), Some(q)) = (
                evidence.duration(),
                evidence.jacobian(),
                evidence.contraction(),
            ) else {
                unreachable!(
                    "a certified Picard segment retains its duration, Jacobian and contraction"
                )
            };
            budget.graph.charge(cells)?;
            let b = map(jacobian, &mut budget.arithmetic, |x, b| x.mul(h, b))?;
            let mut terms = vec![identity.clone()];
            let mut endpoint = identity.clone();
            let mut tube = identity.clone();
            // alpha encloses q^k/k! for the last computed term k.
            let mut alpha = one.clone();
            let mut vanished = N == 0;
            for k in 1..=options.terms {
                if vanished {
                    break;
                }
                retain(budget, &mut retained, term_storage, scratch)?;
                budget
                    .graph
                    .charge(size(product_work.checked_add(cells))?)?;
                let divisor = DyadicInterval::point(k.into(), 0)?;
                let reciprocal = divisor.reciprocal(precision, &mut budget.arithmetic)?;
                let product =
                    b.mul_matrix(terms.last().expect("identity term"), &mut budget.arithmetic)?;
                let term = map(&product, &mut budget.arithmetic, |x, b| {
                    x.mul(&reciprocal, b)?.regrid(precision, b)
                })?;
                vanished = term
                    .rows()
                    .is_some_and(|rows| rows.iter().flatten().all(|x| x.signs() == SignSet::Zero));
                budget.graph.charge(size(cells.checked_mul(3))?)?;
                endpoint = endpoint.add(&term, &mut budget.arithmetic)?;
                tube = tube.add(
                    &term.hull(&zero_matrix, &mut budget.arithmetic)?,
                    &mut budget.arithmetic,
                )?;
                alpha = alpha.mul(q, &mut budget.arithmetic)?.div(
                    &divisor,
                    precision,
                    &mut budget.arithmetic,
                )?;
                terms.push(term);
            }
            budget.graph.charge(cells)?;
            let tail = if vanished {
                zero.clone()
            } else {
                // terms.len() is p + 1, counting identity. 1 - q is exact and
                // positive because the certified contraction has upper bound below
                // one, so coarse rounding cannot produce a zero denominator.
                let next = DyadicInterval::point(terms.len().into(), 0)?;
                alpha
                    .mul(q, &mut budget.arithmetic)?
                    .div(&next, precision, &mut budget.arithmetic)?
                    .div(
                        &one.sub(q, &mut budget.arithmetic)?,
                        precision,
                        &mut budget.arithmetic,
                    )?
                    .hull(&zero, &mut budget.arithmetic)?
            };
            let radius = tail
                .neg(&mut budget.arithmetic)?
                .hull(&tail, &mut budget.arithmetic)?;
            let error = constant::<N>(|_, _| radius.clone())?;
            let local_end = endpoint.add(&error, &mut budget.arithmetic)?;
            let local_tube = tube.add(&error, &mut budget.arithmetic)?;
            budget.graph.charge(pair_work)?;
            // Native sources compose directly on the published source-coordinate
            // matrices, so they keep no separate chain and copy nothing.
            let previous = match chart {
                Some(_) => &chain,
                None => steps.last().map_or(&initial, |s| &s.global_end),
            };
            let end_chain = product(&local_end, previous, precision, &mut budget.arithmetic)?;
            let tube_chain = product(&local_tube, previous, precision, &mut budget.arithmetic)?;
            let (global_end, global_tube) = match (&chart, &inverse) {
                (Some(a), Some(b)) => {
                    let published = (
                        conjugate(a, &end_chain, b, budget)?,
                        conjugate(a, &tube_chain, b, budget)?,
                    );
                    chain = end_chain;
                    published
                }
                (None, None) => (end_chain, tube_chain),
                _ => unreachable!("a chart source names both its chart and its inverse"),
            };
            steps.push(SensitivityStep {
                terms,
                tail,
                local_end,
                local_tube,
                global_end,
                global_tube,
            });
        }
    }
    budget.graph.charge(0)?;
    Ok(FlowSensitivity {
        source,
        options,
        policy: budget.policy(),
        initial,
        horizon,
        steps,
    })
}

fn retain(
    budget: &mut EnclosureBudget,
    retained: &mut usize,
    additional: usize,
    scratch: usize,
) -> Result<(), FieldError> {
    let count = size(retained.checked_add(additional))?;
    budget.graph.check_entries(&[count, scratch])?;
    budget.graph.charge(additional)?;
    *retained = count;
    Ok(())
}
