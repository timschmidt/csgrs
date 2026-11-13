//! Certified prefix composition with explicit outward endpoint transport.
use super::*;

/// A native one-segment search and its admitted outward continuation box.
///
/// The native request can stop at its deliberate one-segment limit. Only its
/// certified prefix is consumed, explicitly; that native report is never relabeled
/// complete. The outer evolution must concatenate all prefixes to the exact end.
#[derive(Clone, Debug)]
pub struct ReachabilityStep<const N: usize> {
    native: EvolutionReport<N>,
    carried: Option<BoxN<DyadicInterval, N>>,
}
impl<const N: usize> ReachabilityStep<N> {
    /// Full native request, search attempts, prefix proofs and local status.
    pub fn native(&self) -> &EvolutionReport<N> {
        &self.native
    }
    /// Outward continuation box, after exact inclusion was checked.
    pub fn carried(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.carried.as_ref()
    }
}

/// Whole-cell evolution composed from certified prefixes and outward transport.
///
/// Endpoint rounding uses the recorded enclosure policy's grid. It does not
/// modify the exact field or original initial set. Rounding is included in every
/// subsequent proof and in the final endpoint diameter; no loss is discarded.
#[derive(Clone, Debug)]
pub struct ReachabilityEvolution<const N: usize> {
    identity: FieldIdentity,
    request: EvolutionRequest<N>,
    options: EvolutionOptions,
    policy: EnclosurePolicy,
    steps: Vec<ReachabilityStep<N>>,
    status: EvolutionStatus,
}
impl<const N: usize> ReachabilityEvolution<N> {
    /// Source shared by every native prefix.
    pub fn identity(&self) -> &FieldIdentity {
        &self.identity
    }
    /// Original cell and whole horizon.
    pub fn request(&self) -> &EvolutionRequest<N> {
        &self.request
    }
    /// Whole-cell limits; maximum_steps counts transported positive segments.
    pub fn options(&self) -> &EvolutionOptions {
        &self.options
    }
    /// Shared arithmetic configuration, including outward transport grid.
    pub fn policy(&self) -> EnclosurePolicy {
        self.policy
    }
    /// All native invocations, including unsuccessful final attempts.
    pub fn steps(&self) -> &[ReachabilityStep<N>] {
        &self.steps
    }
    /// Certified, transported positive-duration segments in time order.
    pub fn segments(&self) -> impl Iterator<Item = &super::super::picard::PicardReport<N>> {
        self.steps
            .iter()
            .filter(|s| s.carried.is_some())
            .filter_map(|s| s.native.segment(0))
    }
    /// Whole-cell status; local one-segment limits remain in their native reports.
    pub fn status(&self) -> &EvolutionStatus {
        &self.status
    }
    /// Endpoint only after whole-cell completion, identity or successful empty.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            EvolutionStatus::Completed => self.steps.last().and_then(|s| s.carried.as_ref()),
            EvolutionStatus::Identity | EvolutionStatus::Empty => Some(&self.request.initial),
            EvolutionStatus::Stopped(_) => None,
        }
    }
}

pub(super) fn integrate_cell<const N: usize>(
    field: &ExpressionField<N>,
    request: EvolutionRequest<N>,
    options: EvolutionOptions,
    b: &mut EnclosureBudget,
) -> ReachabilityEvolution<N> {
    let mut r = ReachabilityEvolution {
        identity: field.identity(),
        request,
        options,
        policy: b.policy(),
        steps: Vec::new(),
        status: EvolutionStatus::Identity,
    };
    r.status = match run(field, &mut r, b) {
        Ok(status) => status,
        Err(reason) => EvolutionStatus::Stopped(reason),
    };
    r
}
fn run<const N: usize>(
    field: &ExpressionField<N>,
    r: &mut ReachabilityEvolution<N>,
    b: &mut EnclosureBudget,
) -> Result<EvolutionStatus, EvolutionFailure> {
    let mut start = r.request.start.checked_pow(1, &mut b.arithmetic)?;
    let mut initial = copy_state(&r.request.initial, b)?;
    let mut cap = r
        .options
        .maximum_step
        .as_ref()
        .map(|cap| cap.checked_pow(1, &mut b.arithmetic))
        .transpose()?;
    loop {
        b.graph.charge(0)?;
        // The first invocation still validates empty/identity requests even at
        // maximum_steps=0. Later invocations require actual remaining capacity.
        if !r.steps.is_empty() && r.steps.len() == r.options.maximum_steps {
            return Err(EvolutionFailure::StepLimit {
                limit: r.options.maximum_steps,
            });
        }
        let count = r
            .steps
            .len()
            .checked_add(1)
            .ok_or(ExpressionError::SizeExhausted)?;
        b.graph.check_entries(&[count])?;
        b.graph.charge(1)?;
        let request = EvolutionRequest {
            start,
            end: r.request.end.checked_pow(1, &mut b.arithmetic)?,
            initial,
        };
        let options = EvolutionOptions {
            maximum_step: cap,
            maximum_steps: r.options.maximum_steps.min(1),
            maximum_halvings: r.options.maximum_halvings,
            maximum_expansions: r.options.maximum_expansions,
        };
        let native = field.integrate_displacement(request, options, b);
        let status = native.status().clone();
        r.steps.push(ReachabilityStep {
            native,
            carried: None,
        });
        let step = r.steps.last_mut().expect("retained invocation");
        match status {
            EvolutionStatus::Empty => {
                b.graph.charge(0)?;
                return Ok(EvolutionStatus::Empty);
            }
            EvolutionStatus::Identity => {
                b.graph.charge(0)?;
                return Ok(EvolutionStatus::Identity);
            }
            EvolutionStatus::Completed => {}
            EvolutionStatus::Stopped(EvolutionFailure::StepLimit { limit: 1 })
                if step.native.segments().len() == 1 => {}
            EvolutionStatus::Stopped(reason) => return Err(reason),
        }
        let proof = step.native.segment(0).expect("one certified segment");
        let endpoint = proof.end_enclosure().expect("certified endpoint");
        b.graph
            .charge(N.checked_mul(2).ok_or(ExpressionError::SizeExhausted)?)?;
        let carried = endpoint.regrid(b.policy.precision, &mut b.arithmetic)?;
        if !endpoint.subset_of(&carried, &mut b.arithmetic)? {
            return Err(EvolutionFailure::NonProgress);
        }
        if proof
            .proposal()
            .start
            .compare_singleton(&proof.proposal().end, &mut b.arithmetic)?
            != Ordering::Less
        {
            return Err(EvolutionFailure::NonProgress);
        }
        let order = proof
            .proposal()
            .end
            .compare_singleton(&r.request.end, &mut b.arithmetic)?;
        if order == Ordering::Greater {
            return Err(EvolutionFailure::NonProgress);
        }
        // Final transport and its consumed inclusion proof precede success.
        b.graph.charge(0)?;
        step.carried = Some(carried);
        if order == Ordering::Equal {
            return Ok(EvolutionStatus::Completed);
        }
        start = proof.proposal().end.checked_pow(1, &mut b.arithmetic)?;
        initial = copy_state(step.carried.as_ref().expect("admitted transport"), b)?;
        cap = Some(
            proof
                .evidence()
                .duration()
                .expect("positive duration")
                .checked_pow(1, &mut b.arithmetic)?,
        );
    }
}
