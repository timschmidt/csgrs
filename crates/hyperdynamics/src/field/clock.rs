//! Exact rational physical time carried by an affine normalized clock.
//!
//! For a<b, t=a+(b-a)*tau maps `[0,1]` exactly onto the requested horizon and
//! dx/dtau=(b-a)*f(t,x). Hypersolve owns the source-bound graph transformation;
//! the existing dyadic integrator owns every ODE proof. Physical time is never
//! replaced by rounded dyadic endpoints. Search options use normalized tau units.
use super::{evolution::*, picard::PicardReport, *};
use core::cmp::Ordering;
use hyperinterval::RationalInterval;
use hyperreal::{Rational, Real};
use hypersolve::expression_program::{AffineParameterization, AffineProgram};

/// Replay data for an exact physical-time chart; not a certificate.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RationalTimeDefinition {
    /// Original physical vector field and its coordinate interpretation.
    pub source: FieldDefinition,
    /// Exact rational start time.
    pub start: Rational,
    /// Exact rational end time, at or after start.
    pub end: Rational,
}

/// Validated exact affine clock. No deserialization trusts its derived duration.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize))]
pub struct RationalClock {
    start: Rational,
    end: Rational,
    duration: Rational,
}
impl RationalClock {
    /// Exact physical start time.
    pub fn start(&self) -> &Rational {
        &self.start
    }
    /// Exact physical end time.
    pub fn end(&self) -> &Rational {
        &self.end
    }
    /// Exact nonnegative difference end-start.
    pub fn duration(&self) -> &Rational {
        &self.duration
    }
}

/// Rational clock construction or physical-evidence materialization failure.
#[derive(Clone, Debug)]
pub enum ClockError {
    /// Physical end precedes start.
    ReversedTime,
    /// Materialized physical coverage did not match the requested horizon.
    ClockMismatch,
    /// Source validation, native admission, resource or cancellation failure.
    Field(FieldError),
}
impl fmt::Display for ClockError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "rational clock: {self:?}")
    }
}
impl std::error::Error for ClockError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Field(e) => Some(e),
            _ => None,
        }
    }
}
impl From<FieldError> for ClockError {
    fn from(e: FieldError) -> Self {
        Self::Field(e)
    }
}
impl From<ExpressionError> for ClockError {
    fn from(e: ExpressionError) -> Self {
        Self::Field(e.into())
    }
}
impl From<IntervalError> for ClockError {
    fn from(e: IntervalError) -> Self {
        Self::Field(e.into())
    }
}

#[derive(Debug)]
struct RationalTimeData<const N: usize> {
    original: FieldIdentity,
    clock: RationalClock,
    transformation: AffineProgram,
    normalized: ExpressionField<N>,
}

/// An original field and its source-bound normalized-time representation.
///
/// Clones share source, clock and transformation evidence. Construction and
/// physical materialization use the same graph/native admission budget. Exact
/// rational admission weights do not bound Hyperreal internals, caches or total
/// heap; the existing resource exclusions remain explicit.
#[derive(Clone, Debug)]
pub struct RationalTimeField<const N: usize> {
    data: Arc<RationalTimeData<N>>,
}
impl<const N: usize> ExpressionField<N> {
    /// Retain an exact rational horizon and construct its normalized vector field.
    /// A zero horizon is allowed for identity evolution, with no ODE existence claim.
    pub fn on_time_span(
        &self,
        start: Rational,
        end: Rational,
        budget: &mut EnclosureBudget,
    ) -> Result<RationalTimeField<N>, ClockError> {
        budget.graph.charge(0)?;
        if budget.arithmetic.rational().compare(&start, &end)? == Ordering::Greater {
            return Err(ClockError::ReversedTime);
        }
        let duration = budget.arithmetic.rational().sub(&end, &start)?;
        let transformation = self.data.identity.0.source.reparameterize(
            AffineParameterization {
                variable: self.data.identity.time(),
                scale: Real::new(duration.clone()),
                offset: Real::new(start.clone()),
            },
            &mut budget.graph,
        )?;
        budget
            .graph
            .charge(N.checked_add(1).ok_or(ExpressionError::SizeExhausted)?)?;
        let normalized = Self::from_validated_source(
            transformation.program().clone(),
            self.data.identity.time(),
            self.data.identity.state().to_vec(),
            self.data.inputs.len(),
            budget,
        )?;
        budget.graph.charge(1)?;
        let result = RationalTimeField {
            data: Arc::new(RationalTimeData {
                original: self.identity(),
                clock: RationalClock {
                    start,
                    end,
                    duration,
                },
                transformation,
                normalized,
            }),
        };
        budget.graph.charge(0)?;
        Ok(result)
    }
}
impl<const N: usize> RationalTimeField<N> {
    /// Validate original replay data and regenerate the clock/program association.
    pub fn from_definition(
        definition: RationalTimeDefinition,
        budget: &mut EnclosureBudget,
    ) -> Result<Self, ClockError> {
        ExpressionField::<N>::from_definition(definition.source, budget)?.on_time_span(
            definition.start,
            definition.end,
            budget,
        )
    }
    /// Exact original physical vector field identity.
    pub fn original_identity(&self) -> &FieldIdentity {
        &self.data.original
    }
    /// Validated exact physical clock.
    pub fn clock(&self) -> &RationalClock {
        &self.data.clock
    }
    /// Source-bound affine graph transformation, including original node mapping.
    pub fn transformation(&self) -> &AffineProgram {
        &self.data.transformation
    }
    /// Owned normalized vector field; its parameter denotes tau, not physical t.
    pub fn normalized_field(&self) -> &ExpressionField<N> {
        &self.data.normalized
    }
    /// Integrate this physical horizon using explicitly normalized-time options.
    ///
    /// `maximum_step` is a bound in tau units. Every physical segment interval
    /// and the final horizon match are materialized and checked before whole-
    /// horizon success. Stopped normalized runs and mapping failures retain their
    /// separate typed reasons. No work counter is reset.
    pub fn integrate(
        &self,
        initial: BoxN<DyadicInterval, N>,
        options: EvolutionOptions,
        budget: &mut EnclosureBudget,
    ) -> ClockEvolution<N> {
        let zero = DyadicInterval::point(0.into(), 0).expect("exact literal");
        let end = DyadicInterval::point(
            if self.data.clock.duration.is_zero() {
                0
            } else {
                1
            }
            .into(),
            0,
        )
        .expect("exact literal");
        let normalized = self.data.normalized.integrate(
            EvolutionRequest {
                start: zero,
                end,
                initial,
            },
            options,
            budget,
        );
        let mut report = ClockEvolution {
            field: self.clone(),
            normalized,
            physical: Vec::new(),
            mapping_error: None,
            status: ClockStatus::Identity,
        };
        report.mapping_error = report.materialize(budget).err();
        report.status = match report.normalized.status() {
            EvolutionStatus::Stopped(e) => ClockStatus::Stopped(ClockFailure::Evolution(e.clone())),
            status => match &report.mapping_error {
                Some(e) => ClockStatus::Stopped(ClockFailure::Mapping(e.clone())),
                None => match status {
                    EvolutionStatus::Completed => ClockStatus::Completed,
                    EvolutionStatus::Identity => ClockStatus::Identity,
                    EvolutionStatus::Empty => ClockStatus::Empty,
                    EvolutionStatus::Stopped(_) => unreachable!("handled above"),
                },
            },
        };
        report
    }
}
#[cfg(feature = "serde")]
impl<const N: usize> serde::Serialize for RationalTimeField<N> {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        #[derive(serde::Serialize)]
        struct DefinitionRef<'a> {
            source: &'a FieldIdentity,
            start: &'a Rational,
            end: &'a Rational,
        }
        DefinitionRef {
            source: self.original_identity(),
            start: self.clock().start(),
            end: self.clock().end(),
        }
        .serialize(serializer)
    }
}

/// First failure of the physical-horizon attempt. A separate mapping error can
/// accompany an already stopped normalized run and remains available in its report.
#[derive(Clone, Debug)]
pub enum ClockFailure {
    /// The normalized integrator did not certify the complete horizon.
    Evolution(EvolutionFailure),
    /// Physical evidence materialization failed after normalized integration.
    Mapping(ClockError),
}
impl fmt::Display for ClockFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "physical evolution: {self:?}")
    }
}
impl std::error::Error for ClockFailure {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Evolution(e) => Some(e),
            Self::Mapping(e) => Some(e),
        }
    }
}
/// Whole physical-horizon completion, separate from normalized proof stages.
#[derive(Clone, Debug)]
pub enum ClockStatus {
    /// Certified evolution reaches the exact rational physical end.
    Completed,
    /// Zero physical horizon identity, with no ODE existence claim.
    Identity,
    /// Successful empty evolution after validation.
    Empty,
    /// Only explicitly materialized prefix evidence is available.
    Stopped(ClockFailure),
}

/// A certified normalized segment bound to its exact physical time interval.
#[derive(Clone, Copy, Debug)]
pub struct RationalSegment<'a, const N: usize> {
    field: &'a RationalTimeField<N>,
    time: &'a RationalInterval,
    proof: &'a PicardReport<N>,
}
impl<'a, const N: usize> RationalSegment<'a, N> {
    /// Exact physical interval covered by this segment.
    pub fn time(&self) -> &'a RationalInterval {
        self.time
    }
    /// Original physical field identity.
    pub fn original_identity(&self) -> &'a FieldIdentity {
        self.field.original_identity()
    }
    /// Validated affine relation between physical and normalized time.
    pub fn clock(&self) -> &'a RationalClock {
        self.field.clock()
    }
    /// Source-bound local proof in normalized coordinates.
    pub fn normalized_proof(&self) -> &'a PicardReport<N> {
        self.proof
    }
    /// State enclosure for the whole physical segment; state coordinates are unchanged.
    pub fn tube(&self) -> &'a BoxN<DyadicInterval, N> {
        self.proof.tube().expect("accepted segment")
    }
    /// State enclosure at this exact physical segment endpoint.
    pub fn end_enclosure(&self) -> &'a BoxN<DyadicInterval, N> {
        self.proof.end_enclosure().expect("accepted segment")
    }
}

/// Immutable physical evolution with all normalized attempts and exact time evidence.
/// Replay the field definition, initial box and normalized options; no success flag
/// or generated transformation is deserialized as a proof.
#[derive(Clone, Debug)]
pub struct ClockEvolution<const N: usize> {
    field: RationalTimeField<N>,
    normalized: EvolutionReport<N>,
    physical: Vec<RationalInterval>,
    mapping_error: Option<ClockError>,
    status: ClockStatus,
}
impl<const N: usize> ClockEvolution<N> {
    /// Original field, affine clock and validated graph association.
    pub fn field(&self) -> &RationalTimeField<N> {
        &self.field
    }
    /// All normalized attempts and stages; these alone are not physical materialization.
    pub fn normalized(&self) -> &EvolutionReport<N> {
        &self.normalized
    }
    /// Any mapping failure, including one after an already stopped normalized run.
    pub fn mapping_error(&self) -> Option<&ClockError> {
        self.mapping_error.as_ref()
    }
    /// Whole physical-horizon outcome.
    pub fn status(&self) -> &ClockStatus {
        &self.status
    }
    /// Certified prefix with successfully materialized exact physical time intervals.
    pub fn segments(&self) -> impl ExactSizeIterator<Item = RationalSegment<'_, N>> {
        self.physical
            .iter()
            .zip(self.normalized.segments())
            .map(|(time, proof)| RationalSegment {
                field: &self.field,
                time,
                proof,
            })
    }
    /// End of materialized physical prefix, or start if no segment exists.
    /// Successful empty evolution reaches the requested physical end.
    pub fn reached_time(&self) -> &Rational {
        if matches!(self.status, ClockStatus::Empty) {
            return self.field.clock().end();
        }
        self.physical
            .last()
            .and_then(RationalInterval::upper)
            .unwrap_or(self.field.clock().start())
    }
    /// Endpoint of the materialized positive-duration prefix, not the requested horizon.
    pub fn prefix_end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        self.physical
            .len()
            .checked_sub(1)
            .and_then(|i| self.normalized.segment(i))
            .and_then(PicardReport::end_enclosure)
    }
    /// Endpoint at the exact requested physical horizon, only after whole-request success.
    pub fn end_enclosure(&self) -> Option<&BoxN<DyadicInterval, N>> {
        match self.status {
            ClockStatus::Completed | ClockStatus::Identity | ClockStatus::Empty => {
                self.normalized.end_enclosure()
            }
            ClockStatus::Stopped(_) => None,
        }
    }
    fn materialize(&mut self, budget: &mut EnclosureBudget) -> Result<(), ClockError> {
        budget.graph.charge(0)?;
        for segment in self.normalized.segments() {
            let count = self
                .physical
                .len()
                .checked_add(1)
                .and_then(|n| n.checked_mul(2))
                .ok_or(ExpressionError::SizeExhausted)?;
            budget.graph.check_entries(&[
                self.normalized.attempts().len(),
                self.normalized.segments().len(),
                count,
            ])?;
            budget.graph.charge(3)?;
            let tau = segment.proposal().end.to_rational(&mut budget.arithmetic)?;
            let scaled = budget.arithmetic.rational().mul(
                self.field.clock().duration(),
                tau.lower().expect("singleton time"),
            )?;
            let end = budget
                .arithmetic
                .rational()
                .add(self.field.clock().start(), &scaled)?;
            let start = self
                .physical
                .last()
                .and_then(RationalInterval::upper)
                .unwrap_or(self.field.clock().start());
            let physical = budget.arithmetic.rational().interval(start.clone(), end)?;
            self.physical.push(physical);
        }
        if matches!(self.normalized.status(), EvolutionStatus::Completed) {
            let Some(end) = self.physical.last().and_then(RationalInterval::upper) else {
                return Err(ClockError::ClockMismatch);
            };
            if budget
                .arithmetic
                .rational()
                .compare(end, self.field.clock().end())?
                != Ordering::Equal
            {
                return Err(ClockError::ClockMismatch);
            }
        }
        budget.graph.charge(0)?;
        Ok(())
    }
}
