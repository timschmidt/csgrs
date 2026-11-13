//! Verify a proposed forward dyadic step with native interval evidence.
//!
//! The proposal is data, not a trajectory. For each initial point in X0,
//! `X0 + [0,h] F(T,Y) ⊆ Y` and `h sup ||J(T,Y)||∞ < 1` prove a Picard
//! self-map contraction on continuous Y-valued curves. The source regularity
//! certificate supplies continuity and state-local C1, rather than assuming
//! those properties from a finite derivative-formula value. The returned tube
//! and endpoint enclose the integral equation directly: no Taylor terms are
//! discarded and no numerical point trajectory is claimed exact. All scalar
//! rounding is included in the images at the recorded enclosure policy.
//!
//! This is one-step verification, not automatic proposal generation, halving
//! or horizon integration. General symbolic refinement and other budget
//! exclusions of `EnclosureBudget` still apply. Replay the exact field definition
//! and proposal; no serialized success flag is accepted as evidence.
use super::*;

/// Exact input and an untrusted candidate enclosure for one step.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct PicardProposal<const N: usize> {
    /// Exact singleton start time.
    pub start: DyadicInterval,
    /// Exact singleton end time, strictly greater than start.
    pub end: DyadicInterval,
    /// Initial state set at start time; uncertainty is retained.
    pub initial: BoxN<DyadicInterval, N>,
    /// Proposed trajectory box, independent of how it was selected.
    pub candidate: BoxN<DyadicInterval, N>,
}

/// Why this particular proposal could not be certified.
#[derive(Clone, Debug)]
pub enum PicardFailure {
    /// Start or end is empty or not a singleton.
    NonPointTime,
    /// End is not strictly after start.
    NonPositiveDuration,
    /// The initial set is not included in the candidate.
    InitialOutsideCandidate,
    /// The full Picard image is not included in the candidate.
    SelfMapNotCertified,
    /// The available Lipschitz bound does not prove a strict contraction.
    ContractionNotCertified,
    /// Final endpoint inclusion was not established.
    EndNotCertified,
    /// Native validation, domain, regularity, resource or cancellation failure.
    Field(FieldError),
}
impl fmt::Display for PicardFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Picard step: {self:?}")
    }
}
impl std::error::Error for PicardFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Field(error) => Some(error),
            _ => None,
        }
    }
}
impl From<FieldError> for PicardFailure {
    fn from(error: FieldError) -> Self {
        Self::Field(error)
    }
}
impl From<ExpressionError> for PicardFailure {
    fn from(error: ExpressionError) -> Self {
        Self::Field(error.into())
    }
}
impl From<IntervalError> for PicardFailure {
    fn from(error: IntervalError) -> Self {
        Self::Field(error.into())
    }
}
impl From<IntervalLatticeError> for PicardFailure {
    fn from(error: IntervalLatticeError) -> Self {
        Self::Field(error.into())
    }
}

/// Completion of one attempted proposal, retaining typed failure.
#[derive(Clone, Debug)]
pub enum PicardStatus {
    /// Existence and uniqueness proved for every initial point on this step.
    Certified,
    /// The initial set is empty; no existence claim is made.
    Empty,
    /// The proposal failed or its proof is unavailable. This is not no-solution evidence.
    Rejected(PicardFailure),
}

/// Retained intermediate enclosures, including those from a rejected proposal.
/// Getters expose only completed stages; they are evidence of their individual
/// inclusions, not by themselves a trajectory certificate.
#[derive(Clone, Debug, Default)]
pub struct PicardEvidence<const N: usize> {
    time: Option<DyadicInterval>,
    duration: Option<DyadicInterval>,
    image: Option<IntervalVector<DyadicInterval, N>>,
    jacobian: Option<IntervalMatrix<DyadicInterval, N, N>>,
    picard_image: Option<BoxN<DyadicInterval, N>>,
    lipschitz: Option<DyadicInterval>,
    contraction: Option<DyadicInterval>,
    end: Option<BoxN<DyadicInterval, N>>,
}
impl<const N: usize> PicardEvidence<N> {
    /// Exact closed time interval.
    pub fn time(&self) -> Option<&DyadicInterval> {
        self.time.as_ref()
    }
    /// Exact positive singleton duration.
    pub fn duration(&self) -> Option<&DyadicInterval> {
        self.duration.as_ref()
    }
    /// Continuous source image on time × candidate, with certified state regularity.
    pub fn image(&self) -> Option<&IntervalVector<DyadicInterval, N>> {
        self.image.as_ref()
    }
    /// Associated state partials on the same product.
    pub fn jacobian(&self) -> Option<&IntervalMatrix<DyadicInterval, N, N>> {
        self.jacobian.as_ref()
    }
    /// Full integral/Picard image; only a successful report makes this a flow tube.
    pub fn picard_image(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.picard_image.as_ref()
    }
    /// Induced Jacobian infinity-norm enclosure; its upper endpoint bounds L.
    pub fn lipschitz(&self) -> Option<&DyadicInterval> {
        self.lipschitz.as_ref()
    }
    /// Enclosure of h times the Jacobian norm; upper < 1 proves contraction.
    pub fn contraction(&self) -> Option<&DyadicInterval> {
        self.contraction.as_ref()
    }
    /// Final integral contribution, if materialized; certification also checks inclusion.
    pub fn end(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.end.as_ref()
    }
}

/// Immutable attempted step, retaining exact source, input, policy and stages.
///
/// The candidate source is explicitly caller-supplied. These values are never
/// deserialized as a certificate. Use `end_enclosure` only after it returns Some;
/// rejected partial stages are available separately through `evidence`.
#[derive(Clone, Debug)]
pub struct PicardReport<const N: usize> {
    identity: FieldIdentity,
    proposal: PicardProposal<N>,
    policy: EnclosurePolicy,
    evidence: PicardEvidence<N>,
    status: PicardStatus,
}
impl<const N: usize> PicardReport<N> {
    /// Original exact field identity and coordinate interpretation.
    pub fn identity(&self) -> &FieldIdentity {
        &self.identity
    }
    /// Caller-supplied exact inputs and candidate.
    pub fn proposal(&self) -> &PicardProposal<N> {
        &self.proposal
    }
    /// Policy governing this attempt; nested calls share its counters.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// Completed intermediate stages, including rejected attempts.
    pub fn evidence(&self) -> &PicardEvidence<N> {
        &self.evidence
    }
    /// Mathematical completion or exact typed rejection.
    pub fn status(&self) -> &PicardStatus {
        &self.status
    }
    /// Certified endpoint set, including successful canonical empty.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            PicardStatus::Certified | PicardStatus::Empty => self.evidence.end.as_ref(),
            PicardStatus::Rejected(_) => None,
        }
    }
    /// Certified trajectory enclosure, including successful canonical empty.
    pub fn tube(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            PicardStatus::Certified | PicardStatus::Empty => self.evidence.picard_image.as_ref(),
            PicardStatus::Rejected(_) => None,
        }
    }
}
impl<const N: usize> ExpressionField<N> {
    /// Verify one caller-supplied forward step, retaining every attempted stage.
    ///
    /// Exact singleton times are validated even for empty initial states. Empty
    /// success is distinct from a nonempty existence/uniqueness proof. Source
    /// regularity, self-map inclusion, strict contraction and the final endpoint
    /// contribution must all pass before certification. Unresolved conditions
    /// do not prove that an ODE solution is absent. No scalar budget is reset.
    pub fn certify_step(
        &self,
        proposal: PicardProposal<N>,
        budget: &mut EnclosureBudget,
    ) -> PicardReport<N> {
        let mut report = PicardReport {
            identity: self.identity(),
            proposal,
            policy: budget.policy(),
            evidence: PicardEvidence::default(),
            status: PicardStatus::Empty,
        };
        report.status = match self.verify_step(&report.proposal, &mut report.evidence, budget) {
            Ok(status) => status,
            Err(error) => PicardStatus::Rejected(error),
        };
        report
    }

    fn verify_step(
        &self,
        proposal: &PicardProposal<N>,
        evidence: &mut PicardEvidence<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<PicardStatus, PicardFailure> {
        budget.graph.charge(0)?;
        let matrix = N.checked_mul(N).ok_or(ExpressionError::SizeExhausted)?;
        budget.graph.check_entries(&[matrix, N, N, N, N, N, 4])?;
        budget.graph.charge(2)?;
        if !proposal.start.is_singleton() || !proposal.end.is_singleton() {
            return Err(PicardFailure::NonPointTime);
        }
        let duration = proposal.end.sub(&proposal.start, &mut budget.arithmetic)?;
        if duration.signs() != hyperinterval::SignSet::Positive {
            return Err(PicardFailure::NonPositiveDuration);
        }
        let time = proposal.start.hull(&proposal.end, &mut budget.arithmetic)?;
        evidence.duration = Some(duration);
        evidence.time = Some(time);
        if proposal.initial.is_empty() {
            evidence.picard_image = Some(BoxN::empty()?);
            evidence.end = Some(BoxN::empty()?);
            budget.graph.charge(0)?;
            return Ok(PicardStatus::Empty);
        }
        budget.graph.charge(N)?;
        if !proposal
            .initial
            .subset_of(&proposal.candidate, &mut budget.arithmetic)?
        {
            return Err(PicardFailure::InitialOutsideCandidate);
        }
        let time = evidence.time.as_ref().expect("time established");
        let arguments = self
            .arguments(
                self.data.jacobian.program(),
                time,
                &proposal.candidate,
                budget,
            )?
            .expect("nonempty candidate and time");
        let regular = self.data.jacobian.enclose_regular(
            &arguments,
            budget.policy.precision,
            &mut budget.graph,
            &mut budget.arithmetic,
        )?;
        budget.graph.charge(
            matrix
                .checked_add(N)
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        let mut values: Vec<_> = regular.into_values().into_iter().map(Some).collect();
        budget.graph.charge(
            matrix
                .checked_add(N)
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        let image = IntervalVector::new(core::array::from_fn(|i| {
            values[i].take().expect("source output")
        }))?;
        let jacobian = IntervalMatrix::new(core::array::from_fn(|row| {
            core::array::from_fn(|column| {
                values[self
                    .data
                    .jacobian
                    .partial_output(row, column)
                    .expect("validated shape")]
                .take()
                .expect("distinct partial")
            })
        }))?;
        evidence.image = Some(image);
        evidence.jacobian = Some(jacobian);
        let duration = evidence.duration.as_ref().expect("duration established");
        let zero = DyadicInterval::point(0.into(), 0)?;
        let elapsed = zero.hull(duration, &mut budget.arithmetic)?;
        budget
            .graph
            .charge(N.checked_mul(3).ok_or(ExpressionError::SizeExhausted)?)?;
        evidence.picard_image = Some(
            proposal.initial.add(
                &evidence
                    .image
                    .as_ref()
                    .expect("image")
                    .scale(&elapsed, &mut budget.arithmetic)?,
                &mut budget.arithmetic,
            )?,
        );
        if !evidence
            .picard_image
            .as_ref()
            .expect("Picard image")
            .subset_of(&proposal.candidate, &mut budget.arithmetic)?
        {
            return Err(PicardFailure::SelfMapNotCertified);
        }
        budget.graph.charge(
            matrix
                .checked_add(N)
                .ok_or(ExpressionError::SizeExhausted)?,
        )?;
        evidence.lipschitz = Some(
            evidence
                .jacobian
                .as_ref()
                .expect("Jacobian")
                .norm_infinity(&mut budget.arithmetic)?,
        );
        evidence.contraction = Some(duration.mul(
            evidence.lipschitz.as_ref().expect("norm"),
            &mut budget.arithmetic,
        )?);
        let one = DyadicInterval::point(1.into(), 0)?;
        if !evidence
            .contraction
            .as_ref()
            .expect("contraction")
            .strictly_left_of(&one, &mut budget.arithmetic)?
        {
            return Err(PicardFailure::ContractionNotCertified);
        }
        budget
            .graph
            .charge(N.checked_mul(3).ok_or(ExpressionError::SizeExhausted)?)?;
        evidence.end = Some(
            proposal.initial.add(
                &evidence
                    .image
                    .as_ref()
                    .expect("image")
                    .scale(duration, &mut budget.arithmetic)?,
                &mut budget.arithmetic,
            )?,
        );
        // Materialize and check the last new contribution before publishing success.
        if !evidence.end.as_ref().expect("end image").subset_of(
            evidence.picard_image.as_ref().expect("Picard image"),
            &mut budget.arithmetic,
        )? {
            return Err(PicardFailure::EndNotCertified);
        }
        budget.graph.charge(0)?;
        Ok(PicardStatus::Certified)
    }
}
