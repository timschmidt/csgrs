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
//! Matrices are Cartesian outer enclosures, not exact reachable derivative sets.
//! A source's stopped prefix remains a prefix. Reports are never deserialized as
//! certificates: replay the field, the evolution request and options, and these
//! options. References: Teschl, *Ordinary Differential Equations and Dynamical
//! Systems*, on dependence on initial conditions; Baake and Schlägel, *The
//! Peano–Baker series*, arXiv:1011.1775, equations (3)–(5) and Theorem 1. The
//! interval-product inclusion and geometric tail above are this module's own
//! derivation.
use super::{evolution::*, picard::PicardStatus, *};
use hyperinterval::SignSet;

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
#[derive(Clone, Debug)]
pub enum SensitivityError {
    /// The source stopped before certifying any positive-duration segment.
    Source(EvolutionFailure),
    /// Shared structural, native arithmetic or cancellation failure.
    Field(FieldError),
}
impl fmt::Display for SensitivityError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "flow sensitivity: {self:?}")
    }
}
impl std::error::Error for SensitivityError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Source(e) => Some(e),
            Self::Field(e) => Some(e),
        }
    }
}

type Matrix<const N: usize> = IntervalMatrix<DyadicInterval, N, N>;

/// Evidence for the source segment with the same index.
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
    /// Endpoint derivative with respect to this segment's initial state.
    pub fn local_end(&self) -> &Matrix<N> {
        &self.local_end
    }
    /// Derivative at every time of this segment, with respect to its initial state.
    pub fn local_tube(&self) -> &Matrix<N> {
        &self.local_tube
    }
    /// Endpoint derivative with respect to the original request's initial state.
    pub fn global_end(&self) -> &Matrix<N> {
        &self.global_end
    }
    /// Derivative at every time of this segment, with respect to the original
    /// initial state.
    pub fn global_tube(&self) -> &Matrix<N> {
        &self.global_tube
    }
}

/// Immutable sensitivity evidence that shares its authoritative evolution report.
#[derive(Clone, Debug)]
pub struct FlowSensitivity<const N: usize> {
    source: Arc<EvolutionReport<N>>,
    options: SensitivityOptions,
    policy: EnclosurePolicy,
    initial: Matrix<N>,
    steps: Vec<SensitivityStep<N>>,
}
impl<const N: usize> FlowSensitivity<N> {
    /// Shared source, including its policy, attempts and Jacobians.
    pub fn source(&self) -> &Arc<EvolutionReport<N>> {
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
    /// Source horizon status; a stopped prefix is never promoted to completion.
    pub fn status(&self) -> &EvolutionStatus {
        self.source.status()
    }
    /// Exact end time of the certified source prefix or successful empty request.
    pub fn reached_time(&self) -> &DyadicInterval {
        self.source.reached_time()
    }
    /// Evidence in the same order as the source's certified segments.
    pub fn steps(&self) -> &[SensitivityStep<N>] {
        &self.steps
    }
    /// Derivative at the end of the certified prefix, without a whole-horizon claim.
    pub fn prefix_end_matrix(&self) -> Option<&Matrix<N>> {
        self.steps.last().map(|s| &s.global_end)
    }
    /// Whole-horizon derivative enclosure, only after the source completed.
    /// An empty initial set yields the empty matrix set; a zero-duration
    /// request yields identity and makes no ODE existence claim.
    pub fn end_matrix(&self) -> Option<&Matrix<N>> {
        match self.status() {
            EvolutionStatus::Completed => self.prefix_end_matrix(),
            EvolutionStatus::Empty | EvolutionStatus::Identity => Some(&self.initial),
            EvolutionStatus::Stopped(_) => None,
        }
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
    /// tail scalars, plus a scratch allowance of twelve matrices and 32
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
        if let EvolutionStatus::Stopped(reason) = self.status()
            && self.segments().len() == 0
        {
            return Err(SensitivityError::Source(reason.clone()));
        }
        derive(Arc::clone(self), options, budget).map_err(SensitivityError::Field)
    }
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

fn constant<const N: usize>(
    entry: impl Fn(usize, usize) -> DyadicInterval,
) -> Result<Matrix<N>, FieldError> {
    Ok(Matrix::new(core::array::from_fn(|r| {
        core::array::from_fn(|c| entry(r, c))
    }))?)
}

fn derive<const N: usize>(
    source: Arc<EvolutionReport<N>>,
    options: SensitivityOptions,
    budget: &mut EnclosureBudget,
) -> Result<FlowSensitivity<N>, FieldError> {
    let cells = size(N.checked_mul(N))?;
    let scratch = size(cells.checked_mul(12).and_then(|n| n.checked_add(32)))?;
    let term_storage = size(cells.checked_add(1))?;
    let step_storage = size(cells.checked_mul(4).and_then(|n| n.checked_add(2)))?;
    let product_work = size(cells.checked_mul(N))?;
    let mut retained = 0;
    retain(budget, &mut retained, term_storage, scratch)?;
    let zero = DyadicInterval::point(0.into(), 0)?;
    let one = DyadicInterval::point(1.into(), 0)?;
    let identity = constant::<N>(|r, c| if r == c { one.clone() } else { zero.clone() })?;
    let zero_matrix = constant::<N>(|_, _| zero.clone())?;
    let initial = match source.status() {
        EvolutionStatus::Empty => Matrix::empty()?,
        _ => identity.clone(),
    };
    let precision = budget.policy.precision;
    let mut steps: Vec<SensitivityStep<N>> = Vec::with_capacity(source.segments().len());
    for segment in source.segments() {
        retain(budget, &mut retained, step_storage, scratch)?;
        retain(budget, &mut retained, term_storage, scratch)?;
        let previous = steps.last().map_or(&initial, |s| &s.global_end);
        if matches!(segment.status(), PicardStatus::Empty) {
            // No state reaches this segment, so its derivative set is empty.
            let empty = Matrix::empty()?;
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
        budget.graph.charge(size(
            product_work
                .checked_add(cells)
                .and_then(|n| n.checked_mul(2)),
        )?)?;
        let global_end = regrid(
            &local_end.mul_matrix(previous, &mut budget.arithmetic)?,
            precision,
            &mut budget.arithmetic,
        )?;
        let global_tube = regrid(
            &local_tube.mul_matrix(previous, &mut budget.arithmetic)?,
            precision,
            &mut budget.arithmetic,
        )?;
        steps.push(SensitivityStep {
            terms,
            tail,
            local_end,
            local_tube,
            global_end,
            global_tube,
        });
    }
    budget.graph.charge(0)?;
    Ok(FlowSensitivity {
        source,
        options,
        policy: budget.policy(),
        initial,
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
